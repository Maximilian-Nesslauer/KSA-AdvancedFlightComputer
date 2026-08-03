using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using PoweredGuidance.Upfg;

// Ascent overlay: the target orbit and the trajectory flown so far, drawn in the
// world so they can be read straight off the map/orbit view. Uses the shared
// projection helpers in PoweredGuidanceOverlayCore.cs.
public static partial class PoweredGuidanceWindow
{
    private static bool _showAscentOverlay = true;

    // Flown trajectory, in the parent body's inertial (CCI) frame — the frame the
    // target orbit lives in, so the two are directly comparable. A ring buffer:
    // once full, the oldest sample is dropped, so a long flight shows its most
    // recent stretch rather than growing without bound.
    private const int TraceCapacity = 2400;
    private const double TraceIntervalSec = 0.5;

    // Newest samples are held back from the drawing. The most recent one sits
    // within a sample interval of the vehicle, so drawing it puts a short line
    // stuck to the hull — harmless at map zoom, but an ugly antenna in the
    // close-in view. Dropping a couple of seconds' worth leaves a clean gap.
    private const int TraceTrimSamples = 4;
    private static readonly double3[] _trace = new double3[TraceCapacity];
    private static int _traceCount;
    private static int _traceHead;          // next write slot
    private static double _traceLastTime = double.NegativeInfinity;
    private static Vehicle _traceVehicle;
    private static IParentBody _traceParent;

    // Unrolled copy of the ring buffer in oldest-to-newest order, for drawing.
    private static readonly double3[] _traceOrdered = new double3[TraceCapacity];

    // Called from ApplyAutopilot (the PrepareWorker prefix), so sampling is driven
    // by the simulation rather than the frame rate and stays even under time warp.
    private static void RecordTrace(Vehicle vehicle, Orbit orbit)
    {
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;

        // A different vehicle, or a change of SOI, makes the existing samples
        // meaningless: they were positions in another body's frame.
        if (!ReferenceEquals(vehicle, _traceVehicle) || !ReferenceEquals(parent, _traceParent))
        {
            ResetTrace();
            _traceVehicle = vehicle;
            _traceParent = parent;
        }

        double now = SimNow();
        if (now - _traceLastTime < TraceIntervalSec)
            return;
        _traceLastTime = now;

        _trace[_traceHead] = orbit.StateVectors.PositionCci;
        _traceHead = (_traceHead + 1) % TraceCapacity;
        if (_traceCount < TraceCapacity)
            _traceCount++;
    }

    private static void ResetTrace()
    {
        _traceCount = 0;
        _traceHead = 0;
        _traceLastTime = double.NegativeInfinity;
    }

    private static void DrawAscentOverlay(Viewport vp, Orbit orbit, IParentBody parent,
                                          double bodyRadius)
    {
        if (!_showAscentOverlay || !SetupProjection(parent))
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
        double pe = _peKm * 1000.0 + bodyRadius;
        double ap = _apKm * 1000.0 + bodyRadius;
        if (ap < pe) (ap, pe) = (pe, ap);
        if (pe <= 0.0)
            return;

        double sma = 0.5 * (pe + ap);
        double ecc = (ap - pe) / (ap + pe);
        double semiLatus = sma * (1.0 - ecc * ecc);

        double inc = UpfgTarget.DegToRad(_incDeg);
        double lan = UpfgTarget.DegToRad(_lanDeg);

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
            dl.AddText(peScreen + new float2(7f, -6f), col, $"Pe {_peKm:F0} km");
        }
        if (TryProjectCci(ring[segments / 2], out float2 apScreen))
        {
            dl.AddCircleFilled(apScreen, 4f, col);
            dl.AddText(apScreen + new float2(7f, -6f), col, $"Ap {_apKm:F0} km");
        }
    }

    private static void DrawTrace(ImDrawListPtr dl, ImColor8 col)
    {
        int count = _traceCount - TraceTrimSamples;
        if (count < 2)
            return;

        // Unroll the ring into chronological order before drawing, so the polyline
        // doesn't jump from the newest sample back to the oldest.
        int start = _traceCount < TraceCapacity ? 0 : _traceHead;
        for (int i = 0; i < count; i++)
            _traceOrdered[i] = _trace[(start + i) % TraceCapacity];

        DrawCciPolyline(dl, _traceOrdered.AsSpan(0, count), col, 2.0f);
    }
}
