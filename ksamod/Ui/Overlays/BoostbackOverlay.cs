using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using PoweredGuidance.Flight;
using PoweredGuidance.Numerics;

// World-space overlay for the drag-integrated impact point, matching the G-FOLD and
// 6-DOF overlays so all three read the same way. Shares the projection plumbing in
// Ui/Overlays/OverlayCore.cs.
//
// WHAT IT SHOWS. Where the vehicle lands if it does nothing from here: RK4 through
// KSA's own drag model and a mirror of KSA's atmosphere, holding the retrograde
// attitude, down to the terrain. That is the drag landing point, as opposed to the
// vacuum instantaneous impact point the closed-form guidance would use - and the two
// are not close. On a boostback-shaped coast the drag point lands LONG of the vacuum
// one, which is the opposite of what most people expect; see Scvx.Console --impact.
//
// EVERYTHING DRAWN HERE IS BODY-FIXED, and that is the fix for a marker that would
// otherwise crawl. The trajectory is integrated in CCI, which is inertial, but it
// lands on ground that has been turning underneath it the whole time - a 200 s coast
// lets the equator move about 93 km. So the path and the impact point are converted
// to CCF ONCE, at prediction time, each sample carried back by the rotation that will
// have happened by the time the vehicle reaches it. After that nothing moves until
// the prediction itself changes: a cached INERTIAL path would slide across the
// terrain by about 90 m per 200 ms of its own age and snap back on every recompute.
//
// THREE SEPARATE THINGS MADE THE MARKER JUMP, and they wanted three different fixes:
//
//   the frame           the drift just described. Fixed by storing CCF, above.
//   terrain feedback    the target radius is the terrain height wherever the last
//                       pass landed, so a moving impact point resamples a moving
//                       height, and at a shallow descent angle that is amplified
//                       several-fold back into the impact point. Low-passed below.
//   the recompute rate  a prediction is milliseconds, so it cannot run every frame;
//                       whatever genuine motion happens between recomputes arrives
//                       all at once. Blended below.
public static partial class PoweredGuidanceWindow
{
    // Off by default, like the other two: it is an instrument, not chrome.
    private static bool _showImpactOverlay;

    /// <summary>How often the prediction is recomputed, ms. A prediction is about a
    /// millisecond, so this cannot be per-frame; at 60 ms the marker's own motion
    /// between recomputes is small enough for the display blend to absorb.</summary>
    private const long ImpactIntervalMs = 60;

    /// <summary>Path samples kept. At the default stride that covers a several-minute
    /// coast; beyond it the path is silently truncated, which costs the tail of a very
    /// long prediction and nothing else.</summary>
    private const int ImpactPathCapacity = 2048;

    /// <summary>How far ahead a prediction looks, minutes. Past this a vehicle is
    /// reported as having no impact rather than integrated further - which is the
    /// right answer for anything in a stable orbit.</summary>
    private const double ImpactHorizonMinutes = 60.0;

    /// <summary>Display blend time constant, s. Short enough that the marker is never
    /// meaningfully behind the prediction, long enough that a step between recomputes
    /// reads as movement rather than a jump.</summary>
    private const double ImpactSmoothTau = 0.12;

    /// <summary>Terrain-height low-pass time constant, s. Longer than the display
    /// blend because it is suppressing a genuine feedback loop rather than smoothing a
    /// sampled signal - see the header.</summary>
    private const double ImpactTerrainTau = 0.6;

    /// <summary>Whether the steering arrows are drawn. Separate from the impact
    /// overlay because the Jacobian is three seeded sweeps - about four times the cost
    /// of the prediction itself - and someone who only wants to see where the vehicle
    /// lands should not pay for it.</summary>
    private static bool _showSteerArrow;

    /// <summary>How often the velocity Jacobian is recomputed FOR THE ARROW, ms. Much
    /// slower than the prediction: it costs about 3.6 ms, and the direction it produces
    /// moves far more slowly than the impact point does. Guidance passes its own,
    /// faster interval - see BoostbackSteerIntervalMs.</summary>
    private const long SteerIntervalMs = 250;

    /// <summary>Arrow length as a fraction of the vehicle-to-impact distance, so the
    /// arrows stay readable from the pad to map zoom.</summary>
    private const double SteerArrowFraction = 0.22;

    /// <summary>Below this much vertical component in the free direction, shaping is
    /// not worth attempting - the geometry does not offer it.</summary>
    private const double ShapeMinVerticalAuthority = 0.05;

    /// <summary>How close to the geometric ceiling the pitch target may get, degrees.
    /// The dv needed runs to infinity at the ceiling, so the last couple of degrees are
    /// refused rather than approached.</summary>
    private const double ShapePitchMarginDeg = 2.0;

    /// <summary>Beyond this much movement in one recompute, snap instead of blending.
    /// A staging event or a burn moves the impact point kilometres, and sliding a
    /// marker across a continent over a tenth of a second looks worse - and reads as
    /// less trustworthy - than simply putting it where it now belongs.</summary>
    private const double ImpactSnapDistanceM = 20000.0;

    /// <summary>
    /// Recompute this vehicle's impact prediction, throttled.
    ///
    /// Driven by the overlay rather than by the guidance step, because nothing flies
    /// on it - it is a readout - and driven by the OVERLAY rather than by the
    /// Boostback tab so that it keeps updating whichever tab is open. The Boostback
    /// tab calls it once, with force, when the toggle is switched on.
    /// </summary>
    private static void UpdateImpactPrediction(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                               bool force)
    {
        if (vehicle == null || parent == null)
            return;

        long now = Environment.TickCount64;
        if (!force && now - _s.ImpactTick < ImpactIntervalMs)
            return;
        _s.ImpactTick = now;

        KsaAeroSweep.Result aero = _s.Aero;
        if (aero?.Table == null)
        {
            _s.HasImpact = false;
            return;
        }

        double mass = vehicle.TotalMass;
        if (!(mass > 0.0))
        {
            _s.HasImpact = false;
            return;
        }

        _s.ImpactPath ??= new double[ImpactPredictor.PathStrideDoubles * ImpactPathCapacity];
        _s.ImpactPathCcf ??= new double3[ImpactPathCapacity];
        _s.ImpactScratch ??= new Dual[ImpactPredictor.ScratchLength];

        var sys = new DragCoastSystem
        {
            Mu = parent.Mu,
            // The body spins about CCI +z (IParentBody.GetAngularVelocityCci is
            // (0,0,w) and the game relies on it), which is exactly the frame
            // DragCoastSystem is written in, so this is a scalar and not a vector.
            OmegaZ = parent.GetAngularVelocity(),
            MeanRadius = parent.MeanRadius,
            AreaOverMass = aero.ReferenceArea / mass,
            // Retrograde throughout: the coast assumption. See DragCoastSystem.
            Alpha = 0.0,
            Table = aero.Table,
            Atmosphere = aero.Atmosphere,
        };

        double3 r0 = orbit.StateVectors.PositionCci;
        double3 v0 = orbit.StateVectors.VelocityCci;

        Span<Dual> x0 = stackalloc Dual[ImpactPredictor.N];
        x0[0] = new Dual(r0.X); x0[1] = new Dual(r0.Y); x0[2] = new Dual(r0.Z);
        x0[3] = new Dual(v0.X); x0[4] = new Dual(v0.Y); x0[5] = new Dual(v0.Z);

        // FIRST PASS, deliberately coarse and with no path recorded. Its only job is
        // to find out roughly WHERE the vehicle lands so the terrain height there can
        // be sampled; a kilometre of error in that lookup point costs a few metres of
        // terrain height, which the second pass then absorbs. Running it at the
        // accurate step sizes would double the cost of the whole overlay to refine a
        // number that is about to be low-passed anyway.
        var scout = ImpactOptions.Default(parent.MeanRadius);
        scout.MaxTime = ImpactHorizonMinutes * 60.0;
        scout.StepAir = scout.StepVacuum;
        scout.PathStride = 0;
        ImpactPrediction p = ImpactPredictor.Predict(sys, x0, scout, _s.ImpactScratch, default);

        var opt = ImpactOptions.Default(parent.MeanRadius);
        opt.MaxTime = ImpactHorizonMinutes * 60.0;

        if (p.Hit && parent is Celestial celestial)
        {
            double3 scoutCcf = GroundCcf(parent, in p);
            double h = celestial.GetTerrainHeightFromDirCcf(scoutCcf.NormalizeOrZero());
            if (double.IsFinite(h))
            {
                // Low-passed, not used raw. The sample point moves every recompute, and
                // over rough ground the height under it can change by hundreds of
                // metres - which a shallow descent turns into kilometres of lateral
                // impact movement. See the header.
                double a = Blend(ImpactTerrainTau, ImpactIntervalMs * 0.001);
                _s.ImpactTerrainH = _s.ImpactTerrainValid
                    ? _s.ImpactTerrainH + (h - _s.ImpactTerrainH) * a
                    : h;
                _s.ImpactTerrainValid = true;
            }
        }
        if (_s.ImpactTerrainValid)
            opt.TargetRadius = parent.MeanRadius + _s.ImpactTerrainH;

        p = ImpactPredictor.Predict(sys, x0, opt, _s.ImpactScratch, _s.ImpactPath);

        _s.Impact = p;
        _s.HasImpact = p.Hit;
        _s.ImpactPathCount = 0;

        if (!p.Hit)
            return;

        // Convert the whole path to body-fixed coordinates ONCE, here. Each sample
        // carries its own time, so each is carried back by the rotation that will have
        // happened by the moment the vehicle reaches it.
        double omega = parent.GetAngularVelocity();
        doubleQuat cci2Ccf = parent.GetCci2Ccf();
        int n = Math.Min(p.PathPoints, ImpactPathCapacity);
        for (int i = 0; i < n; i++)
        {
            int b = i * ImpactPredictor.PathStrideDoubles;
            double3 cci = new double3(_s.ImpactPath[b], _s.ImpactPath[b + 1], _s.ImpactPath[b + 2]);
            _s.ImpactPathCcf[i] = RotZ(cci, -omega * _s.ImpactPath[b + 3]).Transform(cci2Ccf);
        }
        _s.ImpactPathCount = n;

        _s.ImpactCcfRaw = GroundCcf(parent, in p);

        double3 lla = parent.GetLlaFromCcf(_s.ImpactCcfRaw);
        _s.ImpactLatDeg = lla.X;
        _s.ImpactLonDeg = lla.Y;

        // Downrange over the GROUND, from the point below the vehicle now to the
        // point it will hit - both body-fixed, each at its own time.
        //
        // Not the inertial angle between the two position vectors, which is the easy
        // thing to write and is a different number: the body turns under the coast, so
        // on a 200 s flight the two differ by most of a degree, which is around 90 km
        // at the equator. The ground answer is the one that agrees with the marker
        // this is drawn beside.
        double3 fromDir = r0.Transform(cci2Ccf).NormalizeOrZero();
        double3 toDir = _s.ImpactCcfRaw.NormalizeOrZero();
        double dot = Math.Clamp(double3.Dot(fromDir, toDir), -1.0, 1.0);
        _s.ImpactDownrangeM = parent.MeanRadius * Math.Acos(dot);
    }

    /// <summary>
    /// The velocity correction that walks the predicted impact onto the landing site.
    ///
    /// Runs three seeded sweeps for d(ground impact)/d(v), then solves the damped,
    /// tangent-projected least squares in ImpactSteering. Throttled hard and gated on
    /// its own toggle - this is the expensive thing on this overlay.
    ///
    /// The interval is a parameter because there are two callers with different needs:
    /// the arrow, which only has to look right, and the boostback burn, which FLIES the
    /// answer and wants it fresh (see BoostbackSteerIntervalMs). Both share the one
    /// throttle and the one result, so having both on screen costs nothing extra.
    ///
    /// Returning early leaves HasSteer and SteerDv alone rather than clearing them,
    /// which is what lets the guidance step hold the last solution between solves.
    /// </summary>
    private static void UpdateSteering(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                       long intervalMs = SteerIntervalMs)
    {
        long now = Environment.TickCount64;
        if (now - _s.SteerTick < intervalMs)
            return;
        _s.SteerTick = now;
        _s.HasSteer = false;

        KsaAeroSweep.Result aero = _s.Aero;
        double mass = vehicle.TotalMass;
        if (aero?.Table == null || !(mass > 0.0))
            return;

        var sys = new DragCoastSystem
        {
            Mu = parent.Mu,
            OmegaZ = parent.GetAngularVelocity(),
            MeanRadius = parent.MeanRadius,
            AreaOverMass = aero.ReferenceArea / mass,
            Alpha = 0.0,
            Table = aero.Table,
            Atmosphere = aero.Atmosphere,
        };

        var opt = ImpactOptions.Default(parent.MeanRadius);
        opt.MaxTime = ImpactHorizonMinutes * 60.0;
        opt.PathStride = 0;
        if (_s.ImpactTerrainValid)
            opt.TargetRadius = parent.MeanRadius + _s.ImpactTerrainH;

        double3 r0 = orbit.StateVectors.PositionCci;
        double3 v0 = orbit.StateVectors.VelocityCci;
        Span<double> x0 = stackalloc double[ImpactPredictor.N]
            { r0.X, r0.Y, r0.Z, v0.X, v0.Y, v0.Z };

        _s.ImpactScratch ??= new Dual[ImpactPredictor.ScratchLength];

        Span<double> dG = stackalloc double[9];
        Span<double> dT = stackalloc double[3];
        ImpactPrediction nom = ImpactPredictor.VelocityJacobian(
            sys, x0, opt, _s.ImpactScratch, dG, default, dT);
        if (!nom.Hit)
            return;

        // The miss, in the frame J's ROWS are written in - the co-rotating frame,
        // which is the current body-fixed frame turned back into CCI axes. The site is
        // body-fixed, so it converts with Ccf2Cci; getting this backwards would give a
        // correction that is right in magnitude and wrong by the body's rotation.
        double3 siteCcf = SiteDirCcf() * (parent.MeanRadius + SiteTerrainHeight(parent));
        double3 siteF = siteCcf.Transform(parent.GetCcf2Cci());
        double3 hitF = new double3(nom.Fx.V, nom.Fy.V, nom.Fz.V);
        double3 missF = hitF - siteF;

        double hitLen = hitF.Length();
        if (!(hitLen > 0.0))
            return;
        double3 nrm = hitF / hitLen;

        Span<double> missSpan = stackalloc double[3] { missF.X, missF.Y, missF.Z };
        Span<double> nrmSpan = stackalloc double[3] { nrm.X, nrm.Y, nrm.Z };
        Span<double> dv = stackalloc double[3];
        Span<double> greedy = stackalloc double[3];

        if (!ImpactSteering.Correction(dG, missSpan, nrmSpan, ImpactSteering.DefaultLambda,
                                       dv, greedy))
            return;

        _s.SteerDv = new double3(dv[0], dv[1], dv[2]);
        double3 g = new double3(greedy[0], greedy[1], greedy[2]);
        _s.SteerGreedy = g.NormalizeOrZero();
        _s.SteerMissM = missF.Length();
        _s.HasSteer = _s.SteerDv.Length() > 0.0;

        ShapeFlightPathAngle(r0, v0, dG);
    }

    /// <summary>
    /// Spend the free direction on keeping the burn from pointing into the ground.
    ///
    /// THE PROBLEM. The targeting correction minimises dv and nothing else, and the
    /// cheapest way to drag an impact point backwards is often to thrust DOWNWARD - a
    /// steeper flight path shortens the time of flight and pulls the impact in. On the
    /// reference arc the correction has 94.5 m/s of its 173.7 pointing below the local
    /// horizon. That is the booster visibly diving during its own boostback, and it is
    /// the documented failure of instantaneous impact-point guidance (Jo, Han and Ahn
    /// section 2.4: "if the command associated with the maximum IIP rate falls within
    /// this region, the IIP guidance algorithm generates a descending trajectory").
    ///
    /// WHAT IS TARGETED, AND WHAT IS NOT. The COMMANDED DIRECTION is levelled - the
    /// burn is stopped from pointing down. The vehicle's own flight path angle is left
    /// alone. That distinction is worth being exact about because getting it wrong is
    /// expensive: this booster is past apogee and descending at 13 degrees, and
    /// levelling its VELOCITY costs 441.9 m/s against a 173.7 m/s correction. Cancelling
    /// the descent is not the boostback's job. Not thrusting further into it is.
    ///
    /// WHY IT IS NEARLY FREE. J maps three velocity components onto two impact
    /// directions, so one combination of velocity moves the impact point nowhere at
    /// all. Pushing along that direction re-aims the burn without disturbing where the
    /// vehicle lands - the standard task-priority arrangement from redundant-manipulator
    /// control, primary task untouched by construction. It is free only to FIRST order:
    /// a finite push leaks a little, measured at about 6% of what the same dv would
    /// move the impact if spent on steering.
    ///
    /// A FLOOR, NOT A SETPOINT, and the caps are what make it one. Fully levelling the
    /// command costs 119 m/s here, most of another correction; the caps hold the spend
    /// to something proportionate and leave the burn partly nose-down rather than
    /// pretending it can be free. The paper's own scheme is likewise "activated when
    /// the flight-path-angle rate falls below the predefined value", with that value
    /// tuned offline - <see cref="ShapeMaxDv"/> is the equivalent knob here and wants
    /// tuning against flight results, not derivation.
    /// </summary>
    private static void ShapeFlightPathAngle(double3 r0, double3 v0, ReadOnlySpan<double> dG)
    {
        _s.SteerShape = default;
        _s.SteerFreeVertical = 0.0;
        _s.SteerPitchUnreachable = false;
        _s.SteerMaxPitchDeg = 0.0;
        _s.SteerCmdPitchDeg = 0.0;

        if (!_s.HasSteer)
            return;

        double3 up = r0.NormalizeOrZero();
        double dvLen = _s.SteerDv.Length();
        if (!(dvLen > 1e-6))
            return;

        _s.SteerCmdPitchDeg = PitchAboveHorizonDeg(_s.SteerDv, up);

        // A FLOOR. Only ever raises the command; a burn already pointing high enough
        // is left alone, because forcing it back DOWN would be the same mistake in the
        // other direction.
        if (_s.SteerCmdPitchDeg >= _s.BoostbackPitchDeg)
            return;

        Span<double> nSpan = stackalloc double[3];
        if (!ImpactSteering.FreeDirection(dG, nSpan))
            return;
        double3 n = new double3(nSpan[0], nSpan[1], nSpan[2]);

        double vertical = double3.Dot(n, up);
        _s.SteerFreeVertical = vertical;
        if (Math.Abs(vertical) < ShapeMinVerticalAuthority)
            return;

        // Point the free direction UP, so more of it always means more pitch. Both
        // signs are equally free; only the sense differs.
        if (vertical < 0.0)
        {
            n = -n;
            vertical = -vertical;
        }

        // THE CEILING, and it is geometry rather than policy. cmd(t) = dv + t*n, so as
        // t grows the direction tends to n itself and the pitch tends to n's pitch.
        // Everything below that is reachable; at it the cost is unbounded.
        _s.SteerMaxPitchDeg = Math.Asin(Math.Clamp(vertical, -1.0, 1.0)) * 180.0 / Math.PI;

        double target = _s.BoostbackPitchDeg;
        if (target >= _s.SteerMaxPitchDeg - ShapePitchMarginDeg)
        {
            // Refused rather than approached: the last couple of degrees before the
            // asymptote cost more dv than the whole rest of the range put together.
            _s.SteerPitchUnreachable = true;
            target = _s.SteerMaxPitchDeg - ShapePitchMarginDeg;
            if (target <= _s.SteerCmdPitchDeg)
                return;
        }

        // Solve pitch(t) = target. The map is monotonic in t (verified by --impact),
        // so bracket by doubling and bisect - bulletproof, and cheap because each
        // evaluation is one vector add and a normalise. Newton would be faster and
        // would stall near the asymptote, which is exactly where robustness matters.
        double lo = 0.0, hi = dvLen;
        for (int i = 0; i < 40 && PitchAboveHorizonDeg(_s.SteerDv + n * hi, up) < target; i++)
            hi *= 2.0;
        for (int i = 0; i < 60; i++)
        {
            double mid = 0.5 * (lo + hi);
            if (PitchAboveHorizonDeg(_s.SteerDv + n * mid, up) < target) lo = mid;
            else hi = mid;
        }

        _s.SteerShape = n * hi;
        _s.SteerCmdPitchDeg = PitchAboveHorizonDeg(_s.SteerCommand, up);
    }

    /// <summary>Angle of a vector above the local horizon, degrees. Positive is up.</summary>
    private static double PitchAboveHorizonDeg(double3 v, double3 up)
    {
        double l = v.Length();
        return l > 1e-6
            ? Math.Asin(Math.Clamp(double3.Dot(v, up) / l, -1.0, 1.0)) * 180.0 / Math.PI
            : 0.0;
    }

    /// <summary>Frame-rate independent exponential blend factor for a time constant.</summary>
    private static double Blend(double tau, double dt)
        => tau > 0.0 ? 1.0 - Math.Exp(-dt / tau) : 1.0;

    /// <summary>
    /// The predicted impact point in the body-fixed frame.
    ///
    /// The de-rotation by omega * (time of flight) is NOT done here - the predictor
    /// has already done it, in Dual arithmetic, and reports the result as Fx/Fy/Fz.
    /// This only applies the current CCI->CCF orientation, which is a constant as far
    /// as the trajectory is concerned.
    ///
    /// Doing the de-rotation here instead would work for the display and be wrong for
    /// anything differentiated: the rotation angle contains the time of flight, so
    /// rotating the .V parts outside the Dual chain drops the d(tof)/dv term, which is
    /// worth about half of d(ground)/dv (measured by --impact). Reading the predictor's
    /// own answer is what stops the picture and the sensitivities disagreeing.
    /// </summary>
    private static double3 GroundCcf(IParentBody parent, in ImpactPrediction p)
        => new double3(p.Fx.V, p.Fy.V, p.Fz.V).Transform(parent.GetCci2Ccf());

    private static void DrawBoostbackOverlay(Viewport vp, Vehicle vehicle, Orbit orbit,
                                             IParentBody parent)
    {
        // The FOCUSED vehicle's prediction, resolved here rather than read from the
        // ambient current - the same trap the 6-DOF overlay documents.
        if (!_showImpactOverlay
            || !VehicleAutopilotState.TryGet(Program.ControlledVehicle, out VehicleAutopilotState st))
            return;

        // THE OVERLAY DRIVES ITS OWN PREDICTION. Doing it from the Boostback tab
        // instead would look identical until you switched tabs, at which point the
        // marker would stop moving and quietly show where the vehicle was going
        // several minutes ago - the worst kind of wrong, because it still looks like
        // an answer. This runs from DrawTrailingWindows every frame regardless of
        // which tab is open, and throttles itself.
        UpdateImpactPrediction(vehicle, orbit, parent, force: false);

        if (!st.HasImpact || st.ImpactPathCcf == null || st.ImpactPathCount < 2)
        {
            st.ImpactShownValid = false;
            return;
        }
        if (!SetupProjection(parent))
            return;

        // Blend the DRAWN point toward the latest prediction. The prediction itself
        // steps at the recompute rate; without this the marker teleports sixteen times
        // a second by however far the answer moved, which reads as noise even when
        // every step is a correct answer. Large moves snap - see ImpactSnapDistanceM.
        long now = Environment.TickCount64;
        double dt = st.ImpactSmoothTick > 0 ? (now - st.ImpactSmoothTick) * 0.001 : 0.0;
        st.ImpactSmoothTick = now;

        if (!st.ImpactShownValid
            || (st.ImpactCcfShown - st.ImpactCcfRaw).Length() > ImpactSnapDistanceM)
        {
            st.ImpactCcfShown = st.ImpactCcfRaw;
            st.ImpactShownValid = true;
        }
        else
        {
            double a = Blend(ImpactSmoothTau, dt);
            st.ImpactCcfShown += (st.ImpactCcfRaw - st.ImpactCcfShown) * a;
        }

        var pathCol = new ImColor8(230, 80, 60);
        var markCol = new ImColor8(255, 60, 60);
        var siteCol = new ImColor8(255, 120, 220);

        ImDrawListPtr dl = BeginOverlayWindow(vp, "##boostback_overlay");

        // Body-fixed straight through - no per-frame rotation, and glued to the ground.
        DrawCcfPolyline(dl, st.ImpactPathCcf.AsSpan(0, st.ImpactPathCount), pathCol, 2.0f);

        double3 hit = st.ImpactCcfShown;
        if (TryProjectCcf(hit, out float2 s))
        {
            // A ringed crosshair rather than a bare cross: over terrain at any zoom the
            // ring is what makes it findable, and the gap in the middle leaves the
            // actual point visible instead of covering it.
            const float g = 6f, r = 16f;
            ScreenLine(dl, s + new float2(g, 0f), s + new float2(r, 0f), markCol, 2.0f);
            ScreenLine(dl, s - new float2(r, 0f), s - new float2(g, 0f), markCol, 2.0f);
            ScreenLine(dl, s + new float2(0f, g), s + new float2(0f, r), markCol, 2.0f);
            ScreenLine(dl, s - new float2(0f, r), s - new float2(0f, g), markCol, 2.0f);
            dl.AddCircle(s, r, markCol, 0, 2.0f);
            dl.AddCircleFilled(s, 2.5f, markCol);

            // Shadowed text: the overlay has no depth test and draws over terrain,
            // ocean and cloud, and plain red on a bright desert is unreadable.
            float2 at = s + new float2(r + 6f, -20f);
            float line = ImGui.GetTextLineHeight();
            OvText(dl, at, markCol, "IMPACT");
            OvText(dl, at + new float2(0f, line), markCol,
                $"{st.ImpactLatDeg:F3}, {st.ImpactLonDeg:F3}");
            OvText(dl, at + new float2(0f, line * 2f), markCol,
                $"T+{st.Impact.TimeOfFlight.V:F0}s  {st.ImpactDownrangeM / 1000.0:F1}km");
            OvText(dl, at + new float2(0f, line * 3f), markCol,
                $"{ImpactSpeed(st):F0} m/s");
        }

        // A line from the landing site to the predicted impact: the miss distance is
        // what a boostback is trying to drive to zero, and seeing it is worth more
        // than reading it. Body-fixed at both ends, so it does not swing about.
        double3 siteCcf = SiteDirCcf() * (parent.MeanRadius + SiteTerrainHeight(parent));
        if (TryProjectCcf(siteCcf, out float2 a1) && TryProjectCcf(hit, out float2 a2))
            ScreenLine(dl, a1, a2, siteCol, 1.4f);

        // --- the steering arrows -------------------------------------------
        if (_showSteerArrow)
        {
            UpdateSteering(vehicle, orbit, parent);
            if (st.HasSteer)
            {
                double3 rv = orbit.StateVectors.PositionCci;
                double3 impactCci = st.ImpactCcfShown.Transform(parent.GetCcf2Cci());
                double scale = (impactCci - rv).Length() * SteerArrowFraction;

                // The correction that NULLS the miss - the one to fly.
                double3 dvDir = st.SteerDv.NormalizeOrZero();
                var burnCol = new ImColor8(90, 230, 255);
                OvArrow(dl, rv, rv + dvDir * scale, burnCol, 2.4f);

                // The greedy direction, thinner and dimmer. Drawn because the two are
                // easy to assume identical and are not: -J^T m is a descent direction,
                // -J^+ m is the one that points at the target.
                var greedyCol = new ImColor8(150, 150, 160);
                OvArrow(dl, rv, rv + st.SteerGreedy * (scale * 0.75), greedyCol, 1.4f, 8f);

                if (TryProjectCci(rv + dvDir * scale, out float2 tip))
                {
                    float line = ImGui.GetTextLineHeight();
                    OvText(dl, tip + new float2(8f, -line), burnCol,
                        $"dV {st.SteerDv.Length():F0} m/s");
                    OvText(dl, tip + new float2(8f, 0f), greedyCol,
                        $"greedy {AngleBetweenDeg(dvDir, st.SteerGreedy):F0} deg off");
                }
            }
        }

        // BeginOverlayWindow opens an ImGui window and leaves the End() to the caller.
        ImGui.End();
    }

    private static double AngleBetweenDeg(double3 a, double3 b)
    {
        double d = Math.Clamp(double3.Dot(a.NormalizeOrZero(), b.NormalizeOrZero()), -1.0, 1.0);
        return Math.Acos(d) * 180.0 / Math.PI;
    }

    private static double ImpactSpeed(VehicleAutopilotState st)
        => Math.Sqrt(st.Impact.Vx.V * st.Impact.Vx.V
                   + st.Impact.Vy.V * st.Impact.Vy.V
                   + st.Impact.Vz.V * st.Impact.Vz.V);
}
