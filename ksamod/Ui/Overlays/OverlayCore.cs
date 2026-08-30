using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// Shared world-space overlay plumbing, used by both the G-FOLD descent overlay
// (Ui/Overlays/Overlay.cs) and the ascent overlay (Ui/Overlays/AscentOverlay.cs).
//
// Everything is drawn on a transparent, click-through, full-screen ImGui window
// using the active camera's world->screen projection. There is no depth test, so
// lines draw on top of terrain and planets rather than being occluded — fine for
// a guidance overlay, and it means the far side of an orbit stays visible.
//
// The same projection works at any zoom, so these draw correctly both in the
// close-in flight view and zoomed out to the map/orbit view: KSA has one camera
// (Program.GetMainCamera), not a separate map camera.
public static partial class PoweredGuidanceWindow
{
    // Per-frame projection context, set by SetupProjection so the helpers don't
    // each re-fetch the camera and body transforms.
    private static Camera _ovCam;
    private static double3 _ovBodyEcl;     // the parent body's position in ECL
    private static doubleQuat _ovCci2Ccf;  // body inertial -> body fixed
    private static doubleQuat _ovCcf2Ecl;  // body fixed -> ECL
    private static readonly float2[] _ovSeg = new float2[2];

    // Scratch for batched polylines; grown on demand, never shrunk.
    private static float2[] _ovPoly = new float2[256];

    private const ImGuiWindowFlags OverlayFlags =
        ImGuiWindowFlags.NoDecoration | ImGuiWindowFlags.NoMove |
        ImGuiWindowFlags.NoSavedSettings | ImGuiWindowFlags.NoBringToFrontOnFocus |
        ImGuiWindowFlags.NoFocusOnAppearing | ImGuiWindowFlags.NoInputs |
        ImGuiWindowFlags.NoBackground;

    // Open a full-screen, click-through overlay window and hand back its draw list.
    // Callers must ImGui.End() when done.
    private static ImDrawListPtr BeginOverlayWindow(Viewport vp, string name)
    {
        ImGui.SetNextWindowPos(new float2(0f, 0f));
        ImGui.SetNextWindowSize(new float2(vp.Width, vp.Height));
        ImGui.Begin(name, OverlayFlags);
        return ImGui.GetWindowDrawList();
    }

    // Set the per-frame projection context (camera + body transforms) used by
    // TryProjectCci. False if there's no camera or the parent isn't a Celestial.
    private static bool SetupProjection(IParentBody parent)
    {
        if (parent is not Celestial body)
            return false;
        Camera cam = Program.GetMainCamera();
        if (cam == null)
            return false;
        _ovCam = cam;
        _ovBodyEcl = body.GetPositionEcl();
        _ovCci2Ccf = parent.GetCci2Ccf();
        _ovCcf2Ecl = body.GetBodyFixed2Ecl();
        return true;
    }

    // Project a CCI point to screen pixels; false if it is behind the camera. The
    // chain is CCI -> body-fixed -> ECL (+ body ECL position) -> screen.
    private static bool TryProjectCci(double3 cci, out float2 screen)
    {
        double3 ecl = _ovBodyEcl + cci.Transform(_ovCci2Ccf).Transform(_ovCcf2Ecl);
        float4 clip = _ovCam.EgoToClip(_ovCam.EclToEgo(ecl));
        if (clip.W <= 0.001f) { screen = default; return false; } // behind the camera
        screen = _ovCam.EclToScreen(ecl, false);
        return true;
    }

    private static void OvLine(ImDrawListPtr dl, double3 cciA, double3 cciB, ImColor8 col, float thick)
    {
        if (TryProjectCci(cciA, out float2 a) && TryProjectCci(cciB, out float2 b))
        {
            _ovSeg[0] = a;
            _ovSeg[1] = b;
            dl.AddPolyline(_ovSeg, col, ImDrawFlags.None, thick);
        }
    }

    private static void ScreenLine(ImDrawListPtr dl, float2 a, float2 b, ImColor8 col, float thick)
    {
        _ovSeg[0] = a;
        _ovSeg[1] = b;
        dl.AddPolyline(_ovSeg, col, ImDrawFlags.None, thick);
    }

    // A run of CCI points as one batched polyline. Points behind the camera break
    // the run, so a long path (an orbit at map zoom, a launch trace) is drawn as
    // however many visible pieces it has rather than one wrong line across the
    // screen. Batching matters here: an orbit is ~128 segments and a trace can be
    // thousands, which is far too many individual AddPolyline calls.
    private static void DrawCciPolyline(ImDrawListPtr dl, ReadOnlySpan<double3> points,
                                        ImColor8 col, float thick)
    {
        if (_ovPoly.Length < points.Length)
            _ovPoly = new float2[Math.Max(points.Length, _ovPoly.Length * 2)];

        int run = 0;
        for (int i = 0; i < points.Length; i++)
        {
            if (TryProjectCci(points[i], out float2 s))
            {
                _ovPoly[run++] = s;
                continue;
            }
            if (run >= 2)
                dl.AddPolyline(_ovPoly.AsSpan(0, run), col, ImDrawFlags.None, thick);
            run = 0;
        }
        if (run >= 2)
            dl.AddPolyline(_ovPoly.AsSpan(0, run), col, ImDrawFlags.None, thick);
    }
}
