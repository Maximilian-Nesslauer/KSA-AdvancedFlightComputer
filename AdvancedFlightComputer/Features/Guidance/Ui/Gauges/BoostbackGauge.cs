#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Features.Ui.Logic;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

// The boostback sections of LAND > BOOSTBACK and of the tuning page. The boostback itself, separate, turn, burn and glide to the landing site, is flown by Modes/Boostback.cs from the START BOOSTBACK verb of LAND > BOOSTBACK; these sections show what it does and the model it flies on.
//  The aero model is shown, in the diagnostics scope, because the surrogate is the thing most likely to be quietly wrong. A table sampled off the wrong frame, referenced to the wrong area, or built from a stale bounding box all produce a plausible-looking spline; the only way to catch it is to be able to read the numbers against a vehicle you can see.
public static partial class GuidanceWindow
{
    /// <summary>Alphas shown in the profile readout, degrees, retrograde-first. A
    /// selection rather than every breakpoint - twenty rows is a scroll, nine is a
    /// shape.</summary>
    private static readonly double[] BoostbackProfileAlphas =
        { 0.0, 5.0, 15.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0 };

    private static void DrawBoostbackContent(Vehicle vehicle, Orbit orbit,
                                                IParentBody parent, float innerW, bool liftNote = true)
    {
        // The guidance step samples the aero model on first sight of a vehicle, and again whenever its bounding box changes, while this content is on screen (ViewDemand) or a stage has a site, so a boostback finds a table whether the page was open or not. The impact prediction of an idle booster is kept current the same way.
        if (TierScope.IsPrimaryPass)
        {
            ViewDemand.Mark(vehicle, ViewDemand.Kind.Aero);
            ViewDemand.Mark(vehicle, ViewDemand.Kind.Impact);
        }

        if (liftNote && TierScope.IsPrimaryPass)
            DrawLiftNote(vehicle, LandWay.Boostback);

        DrawBoostbackGuidanceSection(vehicle, innerW);
        if (!TierScope.IsPrimaryPass || BoostbackLive)
            DrawBoostbackImpactSection(vehicle, parent, innerW);
        DrawBoostbackSurrogateSection(vehicle, innerW);
        DrawBoostbackProfileSection(innerW);
        DrawBoostbackAtmosphereSection(vehicle, orbit, parent, innerW);
    }

    /// <summary>The text START BOOSTBACK on LAND > BOOSTBACK explains itself with.</summary>
    private static string BoostbackCommitHint()
        => $"Starts at separation. The burn flies an optimized plan, pitch, yaw and duration shot for minimum propellant, "
         + FormattableString.Invariant($"re-solved every {BoostbackPlanIntervalS:F0} s, then every {BoostbackTerminalIntervalS:F1} s inside T-{BoostbackTerminalS:F0} s")
         + (BoostbackLockTgo > 0.0 ? FormattableString.Invariant($", and open loop for the last {BoostbackLockTgo:F0} s.") : ", all the way to cutoff.")
         + " The glide then steers the impact point onto the site.";

    private static void DrawBoostbackBurnSection()
    {
        if (!GuidanceRegion.Begin(FoldNames.Burn, GuidanceRegion.Expert))
            return;
        GaugeRow(Specs.BoostbackClearanceM, ref _s.BoostbackClearanceM);
        GaugeRow(Specs.BoostbackSeparationS, ref _s.BoostbackSeparationS);
        GaugeRow(Specs.BoostbackPitchDeg, ref _s.BoostbackPitchDeg);
        GuidanceRegion.End();
    }

    private static void DrawBoostbackGlideSection()
    {
        if (!GuidanceRegion.Begin(FoldNames.Glide, GuidanceRegion.Expert))
            return;
        GaugeRow(Specs.GlideMaxAoaDeg, ref _s.GlideMaxAoaDeg);
        GaugeRow(Specs.GlideNominalAoaDeg, ref _s.GlideNominalAoaDeg);
        GuidanceRegion.End();
    }

    // The state machine: which phase, what it is waiting for, and the numbers the burn is flown on. First, because it is the only thing here that commands the vehicle; everything below it is the model the commands are derived from.
    private static void DrawBoostbackGuidanceSection(Vehicle vehicle, float innerW)
    {
        if (!GuidanceRegion.Begin("BOOSTBACK GUIDANCE", GuidanceRegion.PlayerTuningDiagnostics, noHead: BoostbackLive))
            return;

        const UiTier P = UiTier.Player;
        const UiTier D = UiTier.Diagnostics;
        float4 dim = ConsoleStyle.TextMuted;
        float4 warn = ConsoleUi.WarnInk;
        float4 live = ConsoleStyle.TextPrimary;
        float4 good = ConsoleStyle.TextPrimary;

        if (BoostbackLive)
        {
            GaugeRowText(D, "PHASE", BoostbackPhaseName(_s.BoostbackPhase), live);

            double inPhase = SimNow() - _s.BoostbackPhaseStart;

            switch (_s.BoostbackPhase)
            {
                case BoostbackPhase.Separation:
                    // A booster adopted at separation first coasts engine-off until it is clear of the stage it left.
                    if (_s.BoostbackClearOf != null)
                    {
                        GaugeRowText(P, "CLEARING", FormattableString.Invariant($"{_s.BoostbackClearGapM,8:F0} / {_s.BoostbackClearanceM:F0} m"));
                        GaugeRowText(P, "THROTTLE", "0 %  (coasting clear)");
                        break;
                    }
                    GaugeRowText(P, "SETTLING", FormattableString.Invariant($"{inPhase,8:F1} / {_s.BoostbackSeparationS:F1} s"));
                    GaugeRowText(P, "THROTTLE", FormattableString.Invariant($"{_s.BoostbackThrottle * 100.0,8:F0} %  (craft minimum)"));
                    break;

                case BoostbackPhase.Rotation:
                {
                    if (!TierScope.Show(D))
                        break;
                    // Both errors, because both gate the transition: the command has to finish issuing AND the vehicle has to have followed it.
                    double3 aim = BoostbackPlanDirection(SimNow());
                    double cmdErr = aim.Length() > 0.5
                        ? AngleBetween(_s.CommandDir, aim) * 180.0 / Math.PI
                        : double.NaN;
                    double fcErr = vehicle.FlightComputer.ErrorAngles.Length() * 180.0 / Math.PI;
                    GaugeRowText(D, "COMMAND TO PLAN",
                        double.IsFinite(cmdErr) ? FormattableString.Invariant($"{cmdErr,8:F1} deg") : "  no plan yet",
                        cmdErr <= BoostbackAlignDeg ? good : dim);
                    GaugeRowText(D, "ATTITUDE ERROR", FormattableString.Invariant($"{fcErr,8:F1} deg"),
                        fcErr <= BoostbackAlignDeg ? good : dim);
                    GaugeRowText(D, "SLEW RATE", FormattableString.Invariant($"{_s.BoostbackSlewDegS,8:F1} deg/s"), dim);
                    break;
                }

                case BoostbackPhase.Boostback:
                    GaugeRowText(D, "DV TO GO", Fmt.Dv(_s.BoostbackDvGo));
                    GaugeRowText(D, "BURN TIME LEFT",
                        double.IsFinite(_s.BoostbackTgo) ? Fmt.TimeFine(_s.BoostbackTgo) : "No plan",
                        double.IsFinite(_s.BoostbackTgo) ? dim : warn);
                    // HOW OFTEN THE PLAN IS BEING RE-SOLVED. The same law flies the whole burn; what changes at the end is the cadence behind it, and that is what the readout tracks.
                    GaugeRowText(D, "PLAN",
                        _s.BoostbackLocked ? "open loop to cutoff"
                        : _s.BoostbackTerminal ? FormattableString.Invariant($"re-solved every {BoostbackTerminalIntervalS:F1} s")
                        : FormattableString.Invariant($"re-solved every {BoostbackPlanIntervalS:F0} s"),
                        _s.BoostbackLocked ? warn : good);
                    // Only ever shown when a tail is configured. BoostbackLockTgo is zero, so normally the plan runs to cutoff and this never appears.
                    if (_s.BoostbackLocked)
                        GaugeRowText(D, "FLOWN OPEN LOOP", FormattableString.Invariant($"{_s.BoostbackAccumDv,8:F1} m/s"), dim);
                    break;

                case BoostbackPhase.Glide:
                {
                    // What the PID is driving to zero, what it is commanding, and whether the air lets it.
                    GaugeRowText(D, "MISS DOWNRANGE", FormattableString.Invariant($"{_s.GlideMissDownM / 1000.0,8:F1} km  (+ long)"));
                    GaugeRowText(D, "MISS CROSSRANGE", FormattableString.Invariant($"{_s.GlideMissCrossM / 1000.0,8:F1} km  (+ left)"));
                    bool limited = _s.GlideAoaDeg >= Math.Clamp(_s.GlideMaxAoaDeg, 0.0, 60.0) - 1e-6 && _s.GlideAoaDeg > 0.0;
                    GaugeRowText(D, "AOA COMMAND", FormattableString.Invariant($"{_s.GlideAoaDeg,8:F1} deg"), limited ? warn : live);
                    double trim = Math.Sqrt(_s.GlideTrimUpDeg * _s.GlideTrimUpDeg + _s.GlideTrimLeftDeg * _s.GlideTrimLeftDeg);
                    GaugeRowText(D, "AOA FLOWN", FormattableString.Invariant($"{_s.GlideFlownAoaDeg,8:F1} deg   trim {trim:F1} deg"), dim);
                    GaugeRowText(D, "LATERAL ACCEL", FormattableString.Invariant($"{_s.GlideAccelCmd,8:F2} m/s^2")
                        + (double.IsFinite(_s.GlideTgo) ? FormattableString.Invariant($"   tgo {_s.GlideTgo:F0} s") : ""), dim);
                    GaugeRowText(D, "Q", FormattableString.Invariant($"{_s.GlideQ / 1000.0,8:F1} kPa"), dim);
                    GaugeRowText(D, "LIFT", _s.GlideLiftSign > 0 ? "towards the engine tilt"
                        : _s.GlideLiftSign < 0 ? "away from the engine tilt (blunt)"
                        : "none to steer with", _s.GlideLiftSign != 0 ? dim : warn);
                    GaugeRowText(D, "ATTITUDE ERROR",
                        FormattableString.Invariant($"{vehicle.FlightComputer.ErrorAngles.Length() * 180.0 / Math.PI,8:F1} deg"), dim);
                    if (_s.GlideStatus.Length > 0 && !(_s.GlideStatus == GlideNoLiftStatus && AeroLiftNote(vehicle.Orbit?.Parent).Length > 0))
                        GaugeRowText(P, "", _s.GlideStatus, warn);
                    if (_s.ThreeDofFromGlide && !double.IsFinite(_s.GlideTgo))
                        GaugeRowText(P, "ENTRY GUIDANCE", "Waiting for an impact prediction", dim);
                    break;
                }
            }
        }

        // THE PLAN. Its own rows because this is what the vehicle is flying, and because the solve time is the number that says whether the cadence is still affordable: it runs on the sim thread, so a plan that starts costing hundreds of milliseconds is a stutter every two seconds and wants moving to the worker thread rather than being left alone.
        if (_s.BoostbackHasPlan)
        {
            GaugeRowText(D, "BURN PLAN",
                FormattableString.Invariant($"{_s.BoostbackPlan.PitchDeg,8:F1} deg pitch, {_s.BoostbackPlan.Duration:F1} s"));
            if (Math.Abs(_s.BoostbackPlan.YawDeg) >= 0.05)
                GaugeRowText(D, "PLAN YAW", FormattableString.Invariant($"{_s.BoostbackPlan.YawDeg,8:F1} deg"), dim);
            GaugeRowText(D, "PLAN COST",
                FormattableString.Invariant($"{_s.BoostbackPlanPropellantKg,8:F0} kg   (miss {_s.BoostbackPlanMissM:F0} m)"), dim);
            GaugeRowText(D, "SOLVE TIME", FormattableString.Invariant($"{_s.BoostbackPlanSolveMs,8:F1} ms"), dim);
        }
        if (_s.BoostbackStatus.Length > 0)
            GaugeRowText(D, "STATUS", Fmt.Sentence(_s.BoostbackStatus), dim);
        if (_s.BoostbackPlanError.Length > 0 && BoostbackLive)
            GaugeRowText(D, "PLAN", _s.BoostbackPlanError
                + (_s.BoostbackHasPlan ? " (flying the previous one)" : ""), warn);

        GaugeRow(Specs.BoostbackSlewDegS, ref _s.BoostbackSlewDegS);

        // The glide PID. Its output is a lateral acceleration, (Kp miss + Ki integral + Kd rate) / tgo^2, flown through the measured lift curve; see BoostbackGlide.cs. Live, so they can be tuned in flight.
        GaugeRow(Specs.GlideKp, ref _s.GlideKp);
        GaugeRow(Specs.GlideKi, ref _s.GlideKi);
        GaugeRow(Specs.GlideKd, ref _s.GlideKd);

        if (_s.HasSteer && TierScope.Show(D))
        {
            double shaped = _s.SteerShape.Length();
            GaugeRowText(D, "BURN PITCH", FormattableString.Invariant($"{_s.SteerCmdPitchDeg,8:F1} deg")
                + (shaped > 0.05 ? FormattableString.Invariant($"   +{shaped:F0} m/s") : ""),
                _s.SteerPitchUnreachable ? warn : dim);

            if (_s.SteerPitchUnreachable)
                GaugeRowText(D, "", FormattableString.Invariant($"ceiling is {_s.SteerMaxPitchDeg:F1} deg - the burn ")
                               + "direction cannot pitch above the free direction", warn);
            else if (shaped <= 0.05 && _s.SteerFreeVertical != 0.0
                     && Math.Abs(_s.SteerFreeVertical) < 0.05)
                GaugeRowText(D, "", "no vertical authority on this geometry", warn);
            else if (shaped > 0.05)
                GaugeRowText(D, "", FormattableString.Invariant($"ceiling {_s.SteerMaxPitchDeg:F0} deg; total ")
                               + FormattableString.Invariant($"{_s.SteerCommand.Length():F0} m/s"), dim);
        }

        GuidanceRegion.End();
    }

    // The drag landing point: where this vehicle touches down if it does nothing more.
    private static void DrawBoostbackImpactSection(Vehicle vehicle, IParentBody parent, float innerW)
    {
        if (!GuidanceRegion.Begin("IMPACT PREDICTION", GuidanceRegion.PlayerDiagnostics, noHead: BoostbackLive))
            return;

        const UiTier P = UiTier.Player;
        const UiTier D = UiTier.Diagnostics;

        float4 dim = ConsoleStyle.TextMuted;
        float4 warn = ConsoleUi.WarnInk;

        if (!BoostbackLive && vehicle.Situation.HasAnyContact())
        {
            GaugeRowText(P, "IMPACT", "None, on the ground", dim);
            GuidanceRegion.End();
            return;
        }

        if (_s.Aero?.Table == null)
        {
            GaugeRowText(P, "PREDICTION", "No air data", warn);
        }
        else if (!_s.HasImpact)
        {
            // Not an error: an orbiting stage genuinely has no impact point inside the horizon, and saying so is more useful than a blank.
            string why = _s.Impact.Status == ImpactStatus.NoImpactWithinHorizon
                ? "None within " + Fmt.Time(ImpactHorizonMinutes * 60.0)
                : _s.Impact.Status.ToString();
            GaugeRowText(P, "IMPACT", why, warn);
            if (_s.Impact.Status == ImpactStatus.NoImpactWithinHorizon)
                GaugeRowText(P, "LOWEST POINT", Fmt.Distance(_s.Impact.MinAltitude), dim);
        }
        else
        {
            GaugeRowText(D, "IMPACT LAT/LON",
                FormattableString.Invariant($"{_s.ImpactLatDeg:F3}, {_s.ImpactLonDeg:F3}"));
            GaugeRowText(D, "TIME TO IMPACT", Fmt.Time(_s.Impact.TimeOfFlight.V));
            GaugeRowText(D, "IMPACT FROM BELOW YOU", Fmt.Distance(_s.ImpactDownrangeM));
            GaugeRowText(P, "SPEED WITH NO BURN", Fmt.Speed(ImpactSpeed(_s)));

            double miss = ImpactMissDistance(parent);
            if (double.IsFinite(miss))
                GaugeRowText(D, "MISS FROM SITE", Fmt.KmFine(miss / 1000.0));

            GaugeRowText(D, "INTEGRATION", $"{_s.Impact.Steps,8} steps");
        }

        // The steering arrows and the correction behind them. Its own toggle because the Jacobian is three seeded sweeps, about four times the cost of the prediction, and it is only meaningful once there is a site to aim at.
        GaugeRowCheck(D, "SHOW STEERING ARROW", "##steerarrow", ref _showSteerArrow);
        if (_showSteerArrow && _s.HasSteer)
        {
            GaugeRowText(D, "CORRECTION DV", FormattableString.Invariant($"{_s.SteerDv.Length(),8:F1} m/s"));
            // How far the greedy direction is from the one that actually nulls the miss. Zero when the miss lies along one of J's singular directions, which is why it reads zero for a purely downrange or purely crossrange miss and grows for anything in between.
            GaugeRowText(D, "GREEDY OFFSET",
                FormattableString.Invariant($"{AngleBetweenDeg(_s.SteerDv, _s.SteerGreedy),8:F1} deg"), dim);

            // The shaping split out: how much of the commanded dv is targeting and how much is buying pitch. The pitch itself is in the guidance section, next to the knob that sets it.
            double shape = _s.SteerShape.Length();
            if (shape > 0.05)
                GaugeRowText(D, "SHAPING DV", FormattableString.Invariant($"{shape,8:F1} m/s  (free direction)"), dim);
            GaugeRowText(D, "COMMANDED DV", FormattableString.Invariant($"{_s.SteerCommand.Length(),8:F1} m/s"));
        }
        else if (_showSteerArrow)
        {
            GaugeRowText(D, "CORRECTION", "no solution", warn);
        }

        // The assumption, stated where the numbers are rather than only in the code: it is the model's biggest simplification and the first thing to doubt if a prediction disagrees with what the vehicle does.
        if (TierScope.Show(D))
            ConsoleUi.MutedWrapped("Assumes retrograde attitude (alpha 0) for the whole coast, and no further thrust.");

        GuidanceRegion.End();
    }

    private static LandVerdict BoostbackVerdict(Vehicle vehicle)
    {
        if (vehicle.Situation.HasAnyContact())
            return LandVerdict.None;
        return LandVerdict.ForImpact(_s.Aero?.Table != null, _s.HasImpact,
            _s.HasImpact ? _s.Impact.TimeOfFlight.V : double.NaN, _s.HasImpact ? ImpactSpeed(_s) : double.NaN,
            _s.Impact.Status == ImpactStatus.NoImpactWithinHorizon, ImpactHorizonMinutes * 60.0, _s.Impact.MinAltitude, _s.Impact.Status.ToString(),
            timeInHead: SituationText.IsFalling(Ui.Console.ConsoleFrame.ReadFacts(vehicle).Where));
    }

    private static void DrawBoostbackImpactDetails()
    {
        if (!_s.HasImpact)
        {
            GaugeRowText("LOWEST POINT", Fmt.Distance(_s.Impact.MinAltitude));
            return;
        }
        GaugeRowText("IMPACT SPEED", Fmt.Speed(ImpactSpeed(_s)));
        GaugeRowText("IMPACT LAT/LON", FormattableString.Invariant($"{_s.ImpactLatDeg:F3}, {_s.ImpactLonDeg:F3}"));
        GaugeRowText("IMPACT FROM BELOW YOU", Fmt.Distance(_s.ImpactDownrangeM));
    }

    /// <summary>Great-circle distance from the predicted impact to the landing site,
    /// or NaN if there is no prediction.</summary>
    private static double ImpactMissDistance(IParentBody parent)
    {
        if (!_s.HasImpact)
            return double.NaN;
        double3 siteDir = SiteDirCciAt(parent, _s.Impact.TimeOfFlight.V).NormalizeOrZero();
        double3 hitDir = new double3(_s.Impact.Rx.V, _s.Impact.Ry.V, _s.Impact.Rz.V)
            .NormalizeOrZero();
        double dot = Math.Clamp(double3.Dot(siteDir, hitDir), -1.0, 1.0);
        return parent.MeanRadius * Math.Acos(dot);
    }

    /// <summary>The live bounding box, in the same terms the sweep records it.</summary>
    private static double3 LiveBoxExtents(Vehicle vehicle)
    {
        if (vehicle == null)
            return default;
        float3 half = vehicle.BoundingBoxHalfExtentsAsmb;
        return new double3(half.X * 2.0, half.Y * 2.0, half.Z * 2.0);
    }

    private static void ResampleBoostbackAero(Vehicle vehicle, IParentBody parent)
    {
        if (KsaAeroSweep.TryBuild(vehicle, parent, SimNow(),
                                  out KsaAeroSweep.Result result, out string error))
        {
            _s.Aero = result;
            _s.AeroError = "";
        }
        else
        {
            // Keep whatever we had. A failed resample on a vehicle mid-staging is not a reason to throw away a table that was correct a second ago, and the stale check will try again on the next frame anyway.
            _s.AeroError = error;
        }
    }

    private static bool MachAxisFlat(KsaAeroSweep.Result a) => a.MachSpread < 0.01;

    // What was sampled, and the three numbers that say whether it is worth trusting.
    private static void DrawBoostbackSurrogateSection(Vehicle vehicle, float innerW)
    {
        if (!GuidanceRegion.Begin("AERO SURROGATE", GuidanceRegion.Diagnostics))
            return;

        const UiTier D = UiTier.Diagnostics;

        KsaAeroSweep.Result a = _s.Aero;

        if (a == null)
        {
            GaugeRowText(D, "SURROGATE", _s.AeroError.Length > 0 ? _s.AeroError : "not sampled",
                _s.AeroError.Length > 0 ? ConsoleStyle.Danger : ConsoleStyle.TextMuted);
        }
        else
        {
            GaugeRowText(D, "GRID", $"{a.MachCount} Mach x {a.AlphaCount} alpha");
            GaugeRowText(D, "REF AREA", FormattableString.Invariant($"{a.ReferenceArea,8:F2} m^2  (nose face)"));
            GaugeRowText(D, "BOX (X,Y,Z)",
                FormattableString.Invariant($"{a.BoxExtents.X,6:F1} {a.BoxExtents.Y,5:F1} {a.BoxExtents.Z,5:F1} m"));

            GaugeRowText(D, "", "");

            if (a.Table == null || !MachAxisFlat(a))
            {
                double lowMach = a.MachGrid[0];
                GaugeRowText(D, "CD TAIL-FIRST", FormattableString.Invariant($"{a.CdTailFirst,8:F2}   (alpha 0, Mach {lowMach:0.##})"));
                GaugeRowText(D, "CD BROADSIDE", FormattableString.Invariant($"{a.CdBroadside,8:F2}   (alpha 90, Mach {lowMach:0.##})"));
                GaugeRowText(D, "CD NOSE-FIRST", FormattableString.Invariant($"{a.CdNoseFirst,8:F2}   (alpha 180, Mach {lowMach:0.##})"));
            }

            float4 warn = ConsoleUi.WarnInk;
            float4 plain = ConsoleStyle.TextPrimary;

            // Two different questions about attitude, and they can have opposite answers for a slender booster - which is why both are here.
            //  POINTING SENSITIVITY is local: how much drag 5 degrees of pointing error near the boostback attitude costs. Small is load-bearing for guidance.
            bool pointingMatters = a.PointingSensitivity > 0.15;
            GaugeRowText(D, "CD(5)/CD(0)", FormattableString.Invariant($"{(1.0 + a.PointingSensitivity),7:F2} x"),
                pointingMatters ? plain : warn);
            if (!pointingMatters)
                ConsoleUi.MutedWrapped("Near alpha 0 the drag barely changes with attitude, so small pointing errors cost little.");

            // ATTITUDE SENSITIVITY is global. Broadside drag is large whatever the number above says, because a slender stack's flank area dwarfs its nose area - so the alpha axis is carrying real information even when the vehicle is insensitive to attitude where it normally sits.
            GaugeRowText(D, "CD(90)/CD(0)", FormattableString.Invariant($"{a.AttitudeSensitivity,7:F1} x"));

            // Roll dependence the table cannot represent, because it has no roll input and stores the azimuthal mean. Zero only for an axisymmetric drag model: stock's is a box, so a square-section booster rolled 45 degrees presents sqrt(2) the flank it does at 0, about 25% for a slender stack.
            GaugeRowText(D, "ROLL SPREAD", FormattableString.Invariant($"{a.RollSpread * 100.0,7:F1} %  (averaged out)"),
                a.RollSpread > 0.35 ? warn : plain);

            GaugeRowText(D, "MACH AXIS", MachAxisFlat(a)
                ? "Flat, no compressibility in this model"
                : FormattableString.Invariant($"Measured, CD varies up to {a.MachSpread * 100.0:F0} %"));

            // The table is drag only. A model with lift pushes the vehicle sideways by this much that the predictions do not include.
            GaugeRowText(D, "LIFT", a.LiftToDragMax < 0.01
                ? "none"
                : FormattableString.Invariant($"L/D up to {a.LiftToDragMax:F2} - not in the table"), a.LiftToDragMax < 0.01 ? plain : warn);

            if (_s.AeroError.Length > 0)
                GaugeRowText(D, "LAST RESAMPLE", _s.AeroError, ConsoleStyle.Danger);
        }

        if (ConsoleWidgets.Button("RESAMPLE".AsSpan(), "AfcAeroResample".AsSpan(), default))
            ViewDemand.RequestResample(vehicle);
        if (a != null)
        {
            ImGui.SameLine();
            if (ConsoleWidgets.Button("COPY CSV".AsSpan(), "AfcAeroCopyCsv".AsSpan(), default))
                ImGui.SetClipboardText(KsaAeroSweep.ToCsv(a));
        }

        GuidanceRegion.End();
    }

    // The table itself, read off the FITTED spline rather than the sampled grid, so what is shown is what the solver would actually get - including any overshoot the fit introduced between breakpoints.
    private static void DrawBoostbackProfileSection(float innerW)
    {
        if (!GuidanceRegion.Begin("CD PROFILE", GuidanceRegion.Diagnostics))
            return;

        const UiTier D = UiTier.Diagnostics;

        KsaAeroSweep.Result a = _s.Aero;
        if (a?.Table == null)
        {
            GaugeRowText(D, "PROFILE", "no table");
            GuidanceRegion.End();
            return;
        }

        // One Mach, below the transonic rise, so the alpha profile shown is the subsonic one.
        const double AtMach = 0.8;
        const double Deg = Math.PI / 180.0;

        GaugeRowText(D, "ALPHA (DEG)", ProfileRow("CD", "CDA m^2"), ConsoleStyle.TextMuted);

        double peak = 0.0;
        for (int i = 0; i < BoostbackProfileAlphas.Length; i++)
            peak = Math.Max(peak, a.Table.Cd(AtMach, BoostbackProfileAlphas[i] * Deg));

        for (int i = 0; i < BoostbackProfileAlphas.Length; i++)
        {
            double deg = BoostbackProfileAlphas[i];
            double cd = a.Table.Cd(AtMach, deg * Deg);
            bool isPeak = cd >= peak;
            DrawProfileBar(cd / Math.Max(peak, 1e-9), isPeak);
            // CdA is the number that actually multiplies dynamic pressure, so show it beside the coefficient - it is what makes the drag force checkable against the game without doing arithmetic in your head.
            GaugeRowText(D, ProfileName(deg), ProfileRow(cd.ToString("F2", Fmt.Inv), (cd * a.ReferenceArea).ToString("F1", Fmt.Inv)), isPeak ? ConsoleStyle.TextPrimary : ConsoleStyle.TextLabel);
        }

        const string alphaNote = "Alpha is retrograde-first: 0 is engine into the wind, 180 is nose-first.";
        ConsoleUi.MutedWrapped(MachAxisFlat(a) ? alphaNote : FormattableString.Invariant($"{alphaNote} The table stands at Mach {AtMach:0.##}."));

        GuidanceRegion.End();
    }

    private static string ProfileName(double deg) => deg switch
    {
        0.0 => "0 TAIL-FIRST",
        90.0 => "90 BROADSIDE",
        180.0 => "180 NOSE-FIRST",
        _ => deg.ToString("F0", Fmt.Inv),
    };

    private static void DrawProfileBar(double share, bool isPeak)
    {
        float scale = ImGuiHelper.InterfaceScale;
        float2 at = ImGui.GetCursorScreenPos();
        float h = ConsoleStyle.FrameHeightPx;
        float width = ConsoleStyle.ContentAvailWidth();
        float from = at.X + ProfileBarLeft * scale;
        float to = at.X + width - ConsoleUi.ValueWidth(ProfileRow("00.00", "0000.0")) - 8f * scale;
        if (to <= from)
            return;
        float barH = 6f * scale;
        var min = new float2(from, at.Y + (h - barH) * 0.5f);
        var max = new float2(from + (to - from) * (float)Math.Clamp(share, 0.0, 1.0), min.Y + barH);
        uint ink = isPeak ? ConsoleStyle.ApplyAlpha(Ui.Canvas.Palette.Air.U32) : Ui.Canvas.Palette.Air.U32At(0.45f * ImGui.GetStyle().Alpha);
        ImGui.GetWindowDrawList().AddRectFilled(min, max, ink);
    }

    private const float ProfileBarLeft = 140f;

    private static string ProfileRow(string cd, string cda) => cd.PadLeft(7) + "   " + cda.PadLeft(8);

    // The mirrored rho(h), and the check that it really is a mirror.
    private static void DrawBoostbackAtmosphereSection(Vehicle vehicle, Orbit orbit,
                                                       IParentBody parent, float innerW)
    {
        if (!GuidanceRegion.Begin("##atmosphere", GuidanceRegion.Diagnostics))
            return;

        const UiTier D = UiTier.Diagnostics;

        ExponentialAtmosphere atm = _s.Aero?.Atmosphere;
        if (atm == null)
        {
            if (_s.Aero != null)
                GaugeRowText(D, "ATMOSPHERE", "none on this body");
            GuidanceRegion.End();
            return;
        }

        GaugeRowText(D, "BODY", _s.Aero.BodyName.Length > 0 ? _s.Aero.BodyName : "(unnamed)");
        GaugeRowText(D, "SEA LEVEL RHO", FormattableString.Invariant($"{atm.SeaLevelDensity:F4} kg/m^3"));
        GaugeRowText(D, "SEA LEVEL P", FormattableString.Invariant($"{atm.SeaLevelPressure / 1000.0,8:F2} kPa"));
        GaugeRowText(D, "SCALE HEIGHT", FormattableString.Invariant($"{atm.ScaleHeight / 1000.0,8:F2} km"));
        GaugeRowText(D, "TOP", FormattableString.Invariant($"{atm.TopAltitude / 1000.0,8:F1} km"));
        // Derived from P0/rho0, not assumed - an isothermal atmosphere has one speed of sound at every altitude. See ExponentialAtmosphere.
        GaugeRowText(D, "SPEED OF SOUND", FormattableString.Invariant($"{atm.SpeedOfSound,8:F1} m/s  (derived)"));

        // The mirror check. This is the number that says our self-contained rho really is the game's rho, re-verified on every resample rather than assumed once.
        double err = _s.Aero.AtmosphereMirrorError;
        GaugeRowText(D, "MIRROR ERROR", PageText.MirrorError(err), err < 1e-9 ? ConsoleStyle.TextPrimary : ConsoleStyle.Danger);

        GaugeRowText(D, "", "");

        // Live, so the model can be checked against a vehicle in flight.
        double alt = orbit.StateVectors.PositionCci.Length() - parent.MeanRadius;
        double rho = atm.Density(alt);
        double speed = vehicle.GetSurfaceSpeed();
        double q = 0.5 * rho * speed * speed;

        GaugeRowText(D, "ALTITUDE", Fmt.Height(alt));
        GaugeRowText(D, "RHO", FormattableString.Invariant($"{rho:F4} kg/m^3"));
        GaugeRowText(D, "AIRSPEED", FormattableString.Invariant($"{speed,8:F1} m/s"));
        GaugeRowText(D, "MACH", FormattableString.Invariant($"{atm.Mach(speed),8:F2}"));
        GaugeRowText(D, "Q", FormattableString.Invariant($"{q / 1000.0,8:F2} kPa"));

        if (TryLiveAlpha(vehicle, orbit, parent, out double alphaDeg))
        {
            GaugeRowText(D, "ALPHA (LIVE)", FormattableString.Invariant($"{alphaDeg,8:F1} deg"));
            KsaAeroSweep.Result a = _s.Aero;
            if (a?.Table != null)
            {
                double cd = a.Table.Cd(atm.Mach(speed), alphaDeg * Math.PI / 180.0);
                // The whole chain, end to end: what the surrogate says this vehicle is feeling right now. Comparable against the game by eye.
                GaugeRowText(D, "DRAG (MODEL)", FormattableString.Invariant($"{cd * a.ReferenceArea * q / 1000.0,8:F1} kN"));
            }
        }
        else
        {
            GaugeRowText(D, "ALPHA (LIVE)", "stationary");
        }

        GuidanceRegion.End();
    }

    /// <summary>
    /// The vehicle's CURRENT angle of attack, retrograde-first, in degrees.
    ///  Airspeed is surface-relative because KSA's atmosphere co-rotates rigidly with the body - the game subtracts omega x r itself before computing drag, and a readout that used inertial velocity would disagree with the sim by a full equatorial rotation speed near the ground.
    ///  Measured in KSA's OWN body frame, where +x is the nose, rather than the solver's model frame where the thrust axis is +z. That is deliberate: the surrogate was sampled against AerodynamicCdABody, which is indexed in these axes, so reading it back the same way is what makes the two comparable.
    /// </summary>
    private static bool TryLiveAlpha(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                     out double alphaDeg)
    {
        alphaDeg = 0.0;

        double3 r = orbit.StateVectors.PositionCci;
        double3 vAir = orbit.StateVectors.VelocityCci
                     - double3.Cross(parent.GetAngularVelocityCci(), r);
        if (vAir.LengthSquared() < 1e-6)
            return false;

        double3 vBody = vAir.Transform(doubleQuat.Inverse(vehicle.GetBody2Cci()));

        // atan2 of cross-flow against the TAIL-ward axial component: -x, so a vehicle flying engine-first reads zero.
        double cross = Math.Sqrt(vBody.Y * vBody.Y + vBody.Z * vBody.Z);
        alphaDeg = Math.Atan2(cross, -vBody.X) * 180.0 / Math.PI;
        return true;
    }
}
