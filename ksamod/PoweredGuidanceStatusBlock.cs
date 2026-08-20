using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// The panel's status block, shared by every powered phase: the phase/tgo/dV readout,
// a schematic arc showing how much of the guidance solution is left to fly, and the
// staging plan as a bar. Ascent and descent differ only in which phase names they
// supply and which way the marker points.
//
// Drawn with an ordinary ImDrawList rather than gauge primitives. That is not a
// compromise: ImGauge has no line, arc or filled rect, and its Label is capped at 16
// uppercase characters — but because ImGauge registers its draw callback at
// BeginWindow, everything an ImGui draw list emits afterwards lands ON TOP of the
// dressed panel. So these sit inside the gauge chrome without fighting it.
public static partial class PoweredGuidanceWindow
{
    // The two ends of the pass strip's distance fade, in components. ImColor8 exposes
    // no way to read a colour back out once built, so the blend needs the numbers —
    // and the two swatches below are built from these so there is one source of truth.
    // Declared first: static initialisers run in textual order.
    private static readonly (byte R, byte G, byte B) PassNearRgb = (232, 238, 245);
    private static readonly (byte R, byte G, byte B) PassFarRgb = (48, 53, 60);

    // Palette, kept close to the gauge colours so the block doesn't read as foreign.
    private static readonly ImColor8 SchemInk = new ImColor8(205, 215, 225);
    private static readonly ImColor8 SchemDim = new ImColor8(120, 128, 138);
    private static readonly ImColor8 SchemRgo = new ImColor8(90, 190, 255);
    private static readonly ImColor8 SchemVgo = new ImColor8(120, 235, 130);
    private static readonly ImColor8 SchemBody = new ImColor8(PassNearRgb.R, PassNearRgb.G, PassNearRgb.B);
    private static readonly ImColor8 SchemBurn = new ImColor8(255, 176, 64);
    private static readonly ImColor8 SchemSpent = new ImColor8(70, 76, 84);
    private static readonly ImColor8 SchemTrack = new ImColor8(PassFarRgb.R, PassFarRgb.G, PassFarRgb.B);
    private static readonly ImColor8 SchemAlert = new ImColor8(255, 96, 96);

    // Pass strip: block width, the narrowest axis it will scale to, and the
    // green/red span for the nearest pass.
    private const float PassStripBlockW = 7f;
    private const float PassStripMinScaleKm = 25f;
    private const double PassGreenKm = 10.0;
    private const double PassRedKm = 400.0;
    // How far a distant pass fades toward the strip: 1 would erase it entirely.
    private const float PassFadeDepth = 0.75f;

    // One colour per stage, cycled. Distinct hues rather than a ramp, because the
    // point is to tell stages apart, not to imply an ordering between them.
    private static readonly ImColor8[] StagePalette =
    {
        new ImColor8(255, 176, 64),
        new ImColor8(90, 190, 255),
        new ImColor8(160, 220, 110),
        new ImColor8(235, 130, 200),
        new ImColor8(255, 225, 100),
        new ImColor8(140, 160, 255),
    };

    /// <summary>
    /// Seconds until engine cutoff. Read off the time LATCHED when the terminal phase
    /// began, not off UPFG's tgo: the solver keeps iterating through the freeze, and
    /// its tgo over a near-zero arc is exactly the quantity that misbehaves there.
    /// </summary>
    private static double AscentCutoffIn() => _s.CutoffTime - SimNow();

    /// <summary>
    /// Phase as the operator thinks of it, rather than as the state machine names it:
    /// what is steering right now.
    /// </summary>
    private static string AscentPhaseLabel()
    {
        if (!_s.Running)
            return _s.LaunchArmed ? "ARMED" : "IDLE";
        switch (_s.Phase)
        {
            case AscentPhase.Vertical: return "VERTICAL";
            case AscentPhase.Turn: return "GRAVITY TURN";
            case AscentPhase.ClosedLoop: return "UPFG";
            case AscentPhase.Terminal:
                return AscentCutoffIn() > 0.0 ? "TERMINAL FREEZE" : "CUTOFF";
            default: return _s.Phase.ToString();
        }
    }

    private static void DrawAscentStatus(float2 origin, float innerW, float rowH)
    {
        bool live = _s.Running;
        bool terminal = live && _s.Phase == AscentPhase.Terminal;
        ImColor8 col = !live ? SchemDim
            : terminal ? (AscentCutoffIn() > 0.0 ? SchemBurn : SchemAlert)
            : (_s.Phase == AscentPhase.ClosedLoop && !_s.Upfg.Converged ? SchemBurn : SchemVgo);

        // In the freeze the countdown is the latched one, so it keeps ticking down
        // cleanly while UPFG's own tgo wanders over the near-zero remaining arc.
        double tgoSec = terminal ? Math.Max(0.0, AscentCutoffIn()) : _s.Upfg.Tgo;

        DrawGuidanceStatusBlock(origin, innerW, rowH, AscentPhaseLabel(), col, live,
            tgoSec, _s.Upfg.VgoMag, decelerating: false);
    }

    /// <summary>
    /// The shared block. <paramref name="decelerating"/> only turns the marker round:
    /// on a descent the vehicle is flying backwards along its own track, and a nose
    /// pointing the way it is travelling would read as an ascent.
    /// </summary>
    private static void DrawGuidanceStatusBlock(float2 origin, float innerW, float rowH,
                                                string phase, ImColor8 phaseCol, bool live,
                                                double tgoSec, double dvVal, bool decelerating)
    {
        ImDrawListPtr dl = ImGui.GetWindowDrawList();

        dl.AddText(origin, phaseCol, phase);
        string tgo = live ? $"T-GO {tgoSec,6:F1} s" : "T-GO   --.- s";
        string dv = live ? $"dV {dvVal,6:F0} m/s" : "dV    --- m/s";
        dl.AddText(new float2(origin.X + innerW * 0.38f, origin.Y), live ? SchemInk : SchemDim, tgo);
        dl.AddText(new float2(origin.X + innerW * 0.70f, origin.Y), live ? SchemInk : SchemDim, dv);

        // Both displays run the full width: the arc needs the span to stay gentle,
        // and the bar needs it to keep short stages legible.
        float arcH = rowH * 4.2f;
        float gap = rowH * 0.3f;
        DrawGuidanceSchematic(dl, new float2(origin.X, origin.Y + rowH), new float2(innerW, arcH),
            live, decelerating);
        // Height depends on the stage count, so the bar reports what it used.
        float barH = DrawStagingBar(dl, new float2(origin.X, origin.Y + rowH + arcH + gap), innerW);

        // Reserve the space in ImGui's layout — the draw list writes pixels but
        // advances no cursor, so without this the sections would overlap it.
        ImGui.Dummy(new float2(innerW, rowH + arcH + gap + barH));
    }

    // --- the arc schematic --------------------------------------------------

    /// <summary>
    /// A gentle arc standing for the curvature of the body we are climbing away from,
    /// with the vehicle at its left end and the two guidance figures drawn as
    /// distances ALONG it. Deliberately not a vector plot: rgo and vgo are shown as
    /// SCALARS, each as a fraction of the largest value seen since EXECUTE, so both
    /// bands deplete toward the vehicle as the burn completes. Nothing here is to
    /// scale against anything else — it is a picture of progress, not geometry.
    /// </summary>
    private static void DrawGuidanceSchematic(ImDrawListPtr dl, float2 min, float2 size,
                                              bool running, bool decelerating)
    {
        dl.AddRect(min, min + size, SchemSpent, 3f);

        float pad = 10f;
        float span = size.X - pad * 2f;
        if (span < 32f)
            return;

        // A circle through the two ends and an apex bulged up by the sagitta. A large
        // radius for a small sagitta is exactly what makes the arc read as gentle.
        float sagitta = MathF.Max(6f, size.Y * 0.16f);
        float radius = (span * span * 0.25f + sagitta * sagitta) / (2f * sagitta);
        float apexY = min.Y + size.Y * 0.70f;
        float2 centre = new float2(min.X + size.X * 0.5f, apexY + radius);
        float half = MathF.Asin(MathF.Min(1f, span * 0.5f / radius));

        bool live = running && _s.Upfg.Tgo > 0.0;
        float thick = MathF.Max(4f, size.Y * 0.09f);
        float rgoOff = thick * 1.6f;
        float vgoOff = thick * 3.2f;

        // The surface itself, then faint full-length tracks — an almost-empty band
        // has to read as almost empty rather than as missing.
        DrawArcBand(dl, centre, radius, -half, half, 0f, SchemSpent, 2f);
        DrawArcBand(dl, centre, radius, -half, half, rgoOff, SchemTrack, thick);
        DrawArcBand(dl, centre, radius, -half, half, vgoOff, SchemTrack, thick);

        float rgoFrac = 0f, vgoFrac = 0f;
        if (live)
        {
            double rgoNow = _s.Upfg.Rgo.Length();
            double vgoNow = _s.Upfg.VgoMag;
            if (rgoNow > _s.RgoPeak) _s.RgoPeak = rgoNow;
            if (vgoNow > _s.VgoPeak) _s.VgoPeak = vgoNow;
            if (_s.RgoPeak > 1e-6) rgoFrac = (float)Math.Clamp(rgoNow / _s.RgoPeak, 0.0, 1.0);
            if (_s.VgoPeak > 1e-6) vgoFrac = (float)Math.Clamp(vgoNow / _s.VgoPeak, 0.0, 1.0);
        }

        DrawArcBand(dl, centre, radius, -half, -half + 2f * half * rgoFrac, rgoOff, SchemRgo, thick);
        DrawArcBand(dl, centre, radius, -half, -half + 2f * half * vgoFrac, vgoOff, SchemVgo, thick);

        // Start marker: aligned with the arc's TANGENT at its left end, so it points
        // the way the bands run rather than standing up off the surface.
        DrawStartMarker(dl, ArcPoint(centre, radius + vgoOff + thick * 1.7f, -half), -half,
            MathF.Max(5f, size.Y * 0.11f), decelerating);

        string rgoText = live ? $"RGO {_s.Upfg.Rgo.Length() / 1000.0:F0} km" : "RGO  --- km";
        string vgoText = live ? $"VGO {_s.Upfg.VgoMag:F0} m/s" : "VGO  --- m/s";
        dl.AddText(new float2(min.X + pad, min.Y + 4f), live ? SchemRgo : SchemDim, rgoText);
        float vgoW = ImGui.CalcTextSize(vgoText).X;
        dl.AddText(new float2(min.X + size.X - pad - vgoW, min.Y + 4f),
            live ? SchemVgo : SchemDim, vgoText);
    }

    // Angles run from straight up at the arc's centre; ImDrawList wants the standard
    // convention, which is a quarter turn behind.
    private static float2 ArcPoint(float2 centre, float radius, float a)
        => new float2(centre.X + radius * MathF.Sin(a), centre.Y - radius * MathF.Cos(a));

    private static void DrawArcBand(ImDrawListPtr dl, float2 centre, float radius,
                                    float aFrom, float aTo, float radialOffset,
                                    ImColor8 col, float thickness)
    {
        if (aTo - aFrom < 1e-4f)
            return;
        dl.PathArcTo(centre, radius + radialOffset,
            aFrom - MathF.PI * 0.5f, aTo - MathF.PI * 0.5f, 48);
        dl.PathStroke(col, ImDrawFlags.None, thickness);
    }

    /// <summary>
    /// The "we are here" marker at the start of the bands, pointing along the arc.
    /// The tangent at angle a is (cos a, sin a) — the derivative of ArcPoint — which
    /// is why this leans with the curve instead of standing radially like the vehicle
    /// glyph it replaced.
    /// </summary>
    private static void DrawStartMarker(ImDrawListPtr dl, float2 at, float a, float len,
                                        bool reversed)
    {
        float sense = reversed ? -1f : 1f;
        float2 fwd = new float2(MathF.Cos(a) * sense, MathF.Sin(a) * sense);
        float2 side = new float2(-fwd.Y, fwd.X);
        float halfW = len * 0.62f;

        dl.AddTriangleFilled(at + fwd * len,
                             at - fwd * (len * 0.5f) + side * halfW,
                             at - fwd * (len * 0.5f) - side * halfW, SchemBody);
    }

    // --- staging bar --------------------------------------------------------

    /// <summary>
    /// The staging plan as a horizontal bar, each stage's width proportional to its
    /// burn time — this one IS to scale — with the same stages listed underneath as
    /// dV and burn time. It is a PLAN, not a progress readout: the stage model
    /// describes the stack still to burn, so it carries no record of what has already
    /// gone, and marking elapsed time on it would be an invention.
    /// </summary>
    /// <returns>The height consumed, so the caller can reserve it.</returns>
    private static float DrawStagingBar(ImDrawListPtr dl, float2 min, float width)
    {
        float lineH = ImGui.GetTextLineHeight();

        var stages = (_s.Running || _s.LandingPhase != LandingPhase.Idle)
            ? _s.UpfgVehicle : _s.StageModel;
        if (stages == null || stages.Stages.Count == 0)
        {
            dl.AddText(min, SchemDim, "no staged model");
            return lineH;
        }

        // Burn time and dV per stage, by the same rules as the stage table: a
        // constant-acceleration (g-limited) stage burns for as long as its dV takes at
        // the limit, not as long as its propellant lasts at full thrust.
        int n = stages.Stages.Count;
        Span<float> burn = n <= 12 ? stackalloc float[12] : new float[n];
        Span<float> dv = n <= 12 ? stackalloc float[12] : new float[n];
        float total = 0f, totalDv = 0f;
        for (int i = 0; i < n; i++)
        {
            PoweredGuidance.Upfg.UpfgStage st = stages.Stages[i];
            double ve = st.Isp * 9.80665;
            double stageDv = ve * Math.Log(st.MassTotal / st.MassDry);
            double t = st.Mode == 2
                ? stageDv / (st.GLim * 9.80665)
                : (st.MassTotal - st.MassDry) / (st.Thrust / ve);
            burn[i] = (float)Math.Max(0.0, t);
            dv[i] = (float)Math.Max(0.0, stageDv);
            total += burn[i];
            totalDv += dv[i];
        }
        if (total <= 1e-3f)
        {
            dl.AddText(min, SchemDim, "no burn time");
            return lineH;
        }

        dl.AddText(min, SchemDim, $"staging   {totalDv:F0} m/s   {total:F0} s total");

        // --- the bar ---
        float barTop = min.Y + lineH + 3f;
        float barH = MathF.Max(8f, lineH * 0.95f);
        float x = min.X;
        for (int i = 0; i < n; i++)
        {
            float wSeg = width * (burn[i] / total);
            float2 a = new float2(x, barTop);
            float2 b = new float2(MathF.Min(x + wSeg, min.X + width), barTop + barH);
            dl.AddRectFilled(a, b, StagePalette[i % StagePalette.Length], 2f);

            string tag = $"S{i + 1}";
            float tagW = ImGui.CalcTextSize(tag).X;
            if (wSeg > tagW + 6f)
                dl.AddText(new float2(a.X + (wSeg - tagW) * 0.5f, a.Y + (barH - lineH) * 0.5f),
                    new ImColor8(20, 22, 26), tag);
            x += wSeg;
        }

        // --- the same stages as text, keyed by colour to the bar above ---
        float y = barTop + barH + 4f;
        float colDv = width * 0.16f;
        float colBurn = width * 0.42f;
        for (int i = 0; i < n; i++)
        {
            ImColor8 col = StagePalette[i % StagePalette.Length];
            dl.AddText(new float2(min.X, y), col, $"S{i + 1}");
            dl.AddText(new float2(min.X + colDv, y), SchemInk, $"{dv[i]:F0} m/s");
            dl.AddText(new float2(min.X + colBurn, y), SchemInk, $"{burn[i]:F0} s");
            y += lineH;
        }

        return y - min.Y;
    }
}
