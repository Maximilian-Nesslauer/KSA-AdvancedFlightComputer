using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using PoweredGuidance.Upfg;

// Ascent overlay: the target orbit and the trajectory flown so far, drawn in the
// world so they can be read straight off the map/orbit view. Uses the shared
// projection helpers in Ui/Overlays/OverlayCore.cs.
public static partial class PoweredGuidanceWindow
{
    private static bool _showAscentOverlay = true;

    // Flown trajectory, in the parent body's inertial (CCI) frame — the frame the
    // target orbit lives in, so the two are directly comparable. A ring buffer:
    // once full, the oldest sample is dropped, so a long flight shows its most
    // recent stretch rather than growing without bound.
    // Public because each vehicle owns its own ring buffer of this size — the track
    // belongs to the craft that flew it, so switching focus shows that craft's path
    // instead of throwing away whichever one was on screen.
    public const int TraceCapacity = 2400;
    private const double TraceIntervalSec = 0.5;
    private const double TraceMinAltitude = 1000.0;

    // Newest samples are held back from the drawing. The most recent one sits
    // within a sample interval of the vehicle, so drawing it puts a short line
    // stuck to the hull — harmless at map zoom, but an ugly antenna in the
    // close-in view. Dropping a couple of seconds' worth leaves a clean gap.
    private const int TraceTrimSamples = 4;

    // Unrolled copy of the ring buffer in oldest-to-newest order, for drawing. Stays
    // static: it is scratch, filled and consumed inside one DrawTrace call.
    private static readonly double3[] _traceOrdered = new double3[TraceCapacity];

    // Called from ApplyAutopilot (the PrepareWorker prefix), so sampling is driven
    // by the simulation rather than the frame rate and stays even under time warp.
    private static void RecordTrace(Vehicle vehicle, Orbit orbit)
    {
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;

        // A change of SOI makes the existing samples meaningless: they were positions
        // in another body's frame. The vehicle no longer needs checking — the buffer
        // is keyed on the vehicle, so it cannot be holding another craft's track.
        if (!ReferenceEquals(parent, _s.TraceParent))
        {
            ResetTrace();
            _s.TraceParent = parent;
        }

        // ONLY THE POWERED CLIMB. Sampling whenever the vehicle exists filled the
        // ring with the pad it sat on and the orbit it coasted in afterwards, so the
        // stretch actually worth looking at — the ascent — was a small part of a
        // buffer mostly spent on a stationary dot and a closed ellipse.
        if (!IsAscentTraceWorthy(vehicle, orbit, parent))
            return;

        double now = SimNow();
        if (now - _s.TraceLastTime < TraceIntervalSec)
            return;
        _s.TraceLastTime = now;

        // Allocated on the first sample rather than with the state: this is by far the
        // largest thing a vehicle's flight computer owns, and state now exists for
        // every craft the panel draws, not only the ones being flown.
        _s.Trace ??= new double3[TraceCapacity];
        _s.Trace[_s.TraceHead] = orbit.StateVectors.PositionCci;
        _s.TraceHead = (_s.TraceHead + 1) % TraceCapacity;
        if (_s.TraceCount < TraceCapacity)
            _s.TraceCount++;
    }

    /// <summary>
    /// Above a kilometre and under thrust. The altitude floor drops the pad, where a
    /// lit engine has yet to move the vehicle anywhere; the thrust test drops the
    /// coast after cutoff. Thrust is the game's own live engine state — lit AND fed —
    /// which is the same pair the auto-stager trusts.
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
        // MAP VIEW ONLY. The target orbit is a full ellipse tens of thousands of km
        // across and the trace is the whole flown arc: from the flight camera they
        // project to lines sweeping across the screen, over the vehicle you are
        // trying to fly. There is one camera, not a separate map camera, so the mode
        // is the only thing distinguishing the two.
        if (vp.Mode != CameraMode.Map || !_showAscentOverlay || !SetupProjection(parent))
            return;

        ImDrawListPtr dl = BeginOverlayWindow(vp, "##ascent_overlay");

        var targetCol = new ImColor8(90, 225, 255);   // cyan  — target orbit
        var traceCol = new ImColor8(255, 60, 220);    // magenta — flown so far

        DrawTargetOrbit(dl, orbit.StateVectors.PositionCci, bodyRadius, targetCol);
        DrawTrace(dl, traceCol);

        ImGui.End();
    }

    // The target orbit as a closed ellipse in the target plane.
    //
    // The plane and the periapsis/apoapsis radii are fully determined by the UI
    // inputs, but the ARGUMENT OF PERIAPSIS is not: UpfgTarget inserts at
    // periapsis, so where periapsis ends up depends on where the ascent actually
    // reaches orbit. Periapsis is therefore anchored to the vehicle's CURRENT
    // position, projected into the target plane — so the drawn orbit passes
    // through where the vehicle is now and you can see the ascent lining up with
    // it as you fly, rather than an ellipse rotated arbitrarily within its plane.
    // The ellipse turns with the vehicle as a result, which is the point.
    private static void DrawTargetOrbit(ImDrawListPtr dl, double3 vehicleCci,
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

        // In-plane basis: periapsis at the vehicle's position flattened into the
        // target plane, normal is UPFG's own plane normal, prograde completes the
        // right-handed pair. If the vehicle happens to sit on the plane's axis the
        // projection vanishes, so fall back to the ascending node there.
        double3 normal = UpfgTarget.OrbitNormal(inc, lan);
        double3 periapsis = vehicleCci - double3.Dot(vehicleCci, normal) * normal;
        periapsis = periapsis.Length() > 1.0
            ? double3.Normalize(periapsis)
            : new double3(Math.Cos(lan), Math.Sin(lan), 0.0);
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

        // Unroll the ring into chronological order before drawing, so the polyline
        // doesn't jump from the newest sample back to the oldest.
        int start = _s.TraceCount < TraceCapacity ? 0 : _s.TraceHead;
        for (int i = 0; i < count; i++)
            _traceOrdered[i] = _s.Trace[(start + i) % TraceCapacity];

        DrawCciPolyline(dl, _traceOrdered.AsSpan(0, count), col, 2.0f);
    }
}
