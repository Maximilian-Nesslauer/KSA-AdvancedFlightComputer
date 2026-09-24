#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;

// Ascent overlay: the target orbit and the trajectory flown so far, drawn in the world so they can be read straight off the map/orbit view. Uses the shared projection helpers in Ui/Overlays/OverlayCore.cs.
public static partial class GuidanceWindow
{
    private static bool _showAscentOverlay = true;

    // Flown trajectory, in the parent body's inertial (CCI) frame - the frame the target orbit lives in, so the two are directly comparable.
    // A ring buffer: once full, the oldest sample is dropped, so a long flight shows its most recent stretch rather than growing without bound.
    // Public because each vehicle owns its own ring buffer of this size - the track belongs to the craft that flew it, so switching focus shows that craft's path instead of throwing away whichever one was on screen.
    public const int TraceCapacity = 2400;
    private const double TraceIntervalSec = 0.5;
    private const double TraceMinAltitude = 1000.0;

    // Newest samples are held back from the drawing. The most recent one sits within a sample interval of the vehicle, so drawing it puts a short line stuck to the hull - harmless at map zoom, but an ugly antenna in the close-in view. Dropping a couple of seconds' worth leaves a clean gap.
    private const int TraceTrimSamples = 4;

    // Unrolled copy of the ring buffer in oldest-to-newest order, for drawing. Stays static: it is scratch, filled and consumed inside one DrawTrace call.
    private static readonly double3[] _traceOrdered = new double3[TraceCapacity];

    // Called from ApplyAutopilot (the PrepareWorker prefix), so sampling is driven by the simulation rather than the frame rate and stays even under time warp.
    private static void RecordTrace(Vehicle vehicle, Orbit orbit)
    {
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;

        // A change of SOI makes the existing samples meaningless: they were positions in another body's frame. The vehicle no longer needs checking - the buffer is keyed on the vehicle, so it cannot be holding another craft's track.
        if (!ReferenceEquals(parent, _s.TraceParent))
        {
            ResetTrace();
            _s.TraceParent = parent;
        }

        // ONLY THE POWERED CLIMB. Sampling whenever the vehicle exists filled the ring with the pad it sat on and the orbit it coasted in afterwards, so the stretch actually worth looking at - the ascent - was a small part of a buffer mostly spent on a stationary dot and a closed ellipse.
        if (!IsAscentTraceWorthy(vehicle, orbit, parent))
            return;

        double now = SimNow();
        if (now - _s.TraceLastTime < TraceIntervalSec)
            return;
        _s.TraceLastTime = now;

        // Allocated on the first sample rather than with the state: this is by far the largest thing a vehicle's flight computer owns, and state now exists for every craft the panel draws, not only the ones being flown.
        _s.Trace ??= new double3[TraceCapacity];
        _s.Trace[_s.TraceHead] = orbit.StateVectors.PositionCci;
        _s.TraceHead = (_s.TraceHead + 1) % TraceCapacity;
        if (_s.TraceCount < TraceCapacity)
            _s.TraceCount++;
    }

    /// <summary>
    /// Above a kilometre and under thrust. The altitude floor drops the pad, where a lit engine has yet to move the vehicle anywhere; the thrust test drops the coast after cutoff. Thrust is the game's own live engine state - lit AND fed - which is the same pair the auto-stager trusts.
    /// </summary>
    private static bool IsAscentTraceWorthy(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        if (!vehicle.IsAnyEngineActive() || !vehicle.IsAnyEnginePropellantAvailable())
            return false;
        return orbit.StateVectors.PositionCci.Length() - parent.MeanRadius > TraceMinAltitude;
    }

    private static void ResetTrace()
    {
        _s.TraceCount = 0;
        _s.TraceHead = 0;
        _s.TraceLastTime = double.NegativeInfinity;
    }

    private static void DrawAscentOverlay(IGameViewport vp, Orbit orbit, IParentBody parent,
                                          double bodyRadius)
    {
        // MAP VIEW ONLY. The target orbit is a full ellipse tens of thousands of km across and the trace is the whole flown arc: from the flight camera they project to lines sweeping across the screen, over the vehicle you are trying to fly. There is one camera, not a separate map camera, so the mode is the only thing distinguishing the two.
        if (vp.Mode != CameraMode.Map || !_showAscentOverlay || !SetupProjection(parent))
            return;

        ImDrawListPtr dl = BeginOverlayWindow(vp, "##ascent_overlay");
        using WindowEnd end = default;

        var targetCol = new ImColor8(90, 225, 255);   // cyan  - target orbit
        var traceCol = new ImColor8(255, 60, 220);    // magenta - flown so far

        DrawTargetOrbit(dl, orbit, parent, bodyRadius, targetCol);
        DrawConvexPlan(dl, orbit, parent);
        DrawTrace(dl, traceCol);
    }

    // Scratch for the drawn plan, grown on demand.
    private static double3[] _planPoints = new double3[256];

    /// <summary>
    /// The convex plan's trajectory, from the pad to insertion, with its separations, the UPFG hand-over and the insertion marked.
    ///
    /// DRAWN FROM WHERE THE PAD IS NOW. The plan's positions are inertial, from the instant it was solved, and the pad has turned with the body since - so before lift-off the whole plan is turned about the spin axis by the angle the body has turned, which is where the same climb would go if it started now. From EXECUTE on it is frozen at the lift-off instant, so the magenta trace flown lies straight over it. After a flight, or once the vehicle has moved off its pad, the stored plan describes a lift-off that is no longer on offer and is not drawn.
    /// </summary>
    private static void DrawConvexPlan(ImDrawListPtr dl, Orbit orbit, IParentBody parent)
    {
        if (!_s.ShowConvexPlan)
            return;

        AscentPlan plan;
        double tRef;
        if (_s.Running && _s.FlyingPlan != null)
        {
            plan = _s.FlyingPlan;
            tRef = _s.FlyingPlanLaunchTime;
        }
        else
        {
            plan = _s.AscentPlan;
            if (plan?.Solution == null || !ReferenceEquals(plan.Body, parent))
                return;
            double3 ccf = orbit.StateVectors.PositionCci.Transform(parent.GetCci2Ccf());
            if ((ccf - plan.StartCcf).Length() > PlanStartToleranceM)
                return;
            tRef = SimNow();
        }
        AscentSolution sol = plan.Solution;
        int n = sol.Nodes;
        if (n < 2)
            return;

        if (_planPoints.Length < n)
            _planPoints = new double3[Math.Max(n, _planPoints.Length * 2)];
        double turn = plan.Omega * (tRef - plan.SolvedAt);
        for (int k = 0; k < n; k++)
            _planPoints[k] = RotZ(new double3(sol.Position[k * 3], sol.Position[k * 3 + 1], sol.Position[k * 3 + 2]), turn);

        var col = sol.Converged ? new ImColor8(255, 170, 40) : new ImColor8(255, 90, 70);
        DrawCciPolyline(dl, _planPoints.AsSpan(0, n), col, 2.0f);

        foreach (int k in plan.Series.StagingNodes)
            if (TryProjectCci(_planPoints[k], out float2 s))
            {
                dl.AddCircleFilled(s, 3.5f, col);
                OvText(dl, s + new float2(7f, -6f), col, $"stage {sol.NodeStage[k] + 1} at {plan.Series.AltitudeKm[k]:F0} km");
            }

        for (int k = 0; k < n; k++)
            if (plan.Series.AltitudeKm[k] >= _s.ConvexHandoverAltKm)
            {
                if (TryProjectCci(_planPoints[k], out float2 s))
                {
                    dl.AddCircle(s, 5f, col, 0, 1.5f);
                    OvText(dl, s + new float2(7f, 6f), col, $"UPFG {_s.ConvexHandoverAltKm:F0} km");
                }
                break;
            }

        if (TryProjectCci(_planPoints[n - 1], out float2 end))
        {
            dl.AddCircleFilled(end, 4f, col);
            OvText(dl, end + new float2(7f, -6f), col,
                $"insert {plan.Series.AltitudeKm[n - 1]:F0} km, {sol.Time[n - 1]:F0} s, {sol.FinalMass / 1000.0:F1} t");
        }
    }

    // The target orbit as a closed ellipse in the target plane.
    //  The plane and the periapsis/apoapsis radii are fully determined by the UI inputs, and so is the ARGUMENT OF PERIAPSIS when it is fixed: the ellipse is then drawn exactly where the ascent is aiming it. While it is free, where periapsis ends up depends on where the ascent actually reaches orbit, so it is drawn from the best information there is - see the free branch below.
    private static void DrawTargetOrbit(ImDrawListPtr dl, Orbit orbit, IParentBody parent,
                                        double bodyRadius, ImColor8 col)
    {
        double pe = _s.PeKm * 1000.0 + bodyRadius;
        double ap = _s.ApKm * 1000.0 + bodyRadius;
        if (ap < pe) (ap, pe) = (pe, ap);
        if (pe <= 0.0)
            return;

        double sma = 0.5 * (pe + ap);
        double ecc = (ap - pe) / (ap + pe);
        double semiLatus = sma * (1.0 - ecc * ecc);

        double inc = UpfgTarget.DegToRad(_s.IncDeg);
        double lan = UpfgTarget.DegToRad(_s.LanDeg);

        // In-plane basis: periapsis where a fixed argument puts it, otherwise where a free one is expected to fall; normal is UPFG's own plane normal, prograde completes the right-handed pair.
        double3 normal = UpfgTarget.OrbitNormal(inc, lan);
        double3 periapsis;
        if (_s.ArgPeFixed && ecc >= UpfgTarget.MinArgPeEccentricity)
        {
            periapsis = UpfgTarget.PeriapsisDirection(inc, lan, UpfgTarget.DegToRad(_s.ArgPeDeg));
        }
        else
        {
            // WHERE A FREE PERIAPSIS WILL ACTUALLY FALL, from the best information there is. Anchoring it to the vehicle's own position throughout kept the drawn periapsis riding along with the vehicle for the whole climb, and on round the orbit after cutoff.
            //  - Flying with a converged solution: UPFG's predicted cutoff, less the true anomaly the insertion is aimed at - zero at periapsis, the insertion search's pick, or the atmosphere-floor crossing. The prediction settles onto a point that holds still in inertial space, and the ellipse with it.
            //  - Not flying, on a closed orbit clear of the surface: that orbit's own periapsis, so a finished ascent shows where the ellipse it reached really sits.
            //  - Otherwise, on the pad or before the first solution, the vehicle's position, which is all there is.
            double3 r = orbit.StateVectors.PositionCci;
            double3 anchor = r;
            double behind = 0.0;
            UpfgTarget.Insertion aim = _s.Upfg.Aim;
            if (_s.Running && _s.Upfg.Converged && aim.Valid)
            {
                anchor = _s.Upfg.Rd;
                behind = aim.TrueAnomaly;
            }
            else if (!_s.Running && orbit.Eccentricity >= UpfgTarget.MinArgPeEccentricity && orbit.Eccentricity < 1.0
                     && orbit.Periapsis > bodyRadius)
            {
                // The eccentricity vector, which points at periapsis; its scale does not matter here.
                double3 v = orbit.StateVectors.VelocityCci;
                anchor = (double3.Dot(v, v) - parent.Mu / r.Length()) * r - double3.Dot(r, v) * v;
            }

            // Flattened into the target plane. An anchor on the plane's axis has no direction in it, so fall back to the ascending node there.
            double3 inPlane = anchor - double3.Dot(anchor, normal) * normal;
            periapsis = inPlane.Length() > 1e-6 * anchor.Length()
                ? double3.Normalize(inPlane)
                : UpfgTarget.NodeDirection(lan);
            periapsis = Math.Cos(behind) * periapsis - Math.Sin(behind) * double3.Cross(normal, periapsis);
        }
        double3 prograde = double3.Cross(normal, periapsis);

        const int segments = 160;
        Span<double3> ring = stackalloc double3[segments + 1];
        for (int i = 0; i <= segments; i++)
        {
            double nu = 2.0 * Math.PI * i / segments;
            double r = semiLatus / (1.0 + ecc * Math.Cos(nu));
            ring[i] = r * (Math.Cos(nu) * periapsis + Math.Sin(nu) * prograde);
        }
        DrawCciPolyline(dl, ring, col, 2.0f);

        // Periapsis and apoapsis markers, so the ellipse can be read at a glance.
        if (TryProjectCci(ring[0], out float2 peScreen))
        {
            dl.AddCircleFilled(peScreen, 4f, col);
            dl.AddText(peScreen + new float2(7f, -6f), col, $"Pe {_s.PeKm:F0} km");
        }
        if (TryProjectCci(ring[segments / 2], out float2 apScreen))
        {
            dl.AddCircleFilled(apScreen, 4f, col);
            dl.AddText(apScreen + new float2(7f, -6f), col, $"Ap {_s.ApKm:F0} km");
        }
    }

    private static void DrawTrace(ImDrawListPtr dl, ImColor8 col)
    {
        int count = _s.TraceCount - TraceTrimSamples;
        if (count < 2)
            return;

        // Unroll the ring into chronological order before drawing, so the polyline doesn't jump from the newest sample back to the oldest.
        int start = _s.TraceCount < TraceCapacity ? 0 : _s.TraceHead;
        for (int i = 0; i < count; i++)
            _traceOrdered[i] = _s.Trace[(start + i) % TraceCapacity];

        DrawCciPolyline(dl, _traceOrdered.AsSpan(0, count), col, 2.0f);
    }
}
