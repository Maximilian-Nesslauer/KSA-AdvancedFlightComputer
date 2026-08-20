using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// The Descent tab's content: everything that happens before the vehicle is anywhere
// near the ground — where it is going, when the deorbit burn is, and how the approach
// is shaped. The shell, tab bar and EXECUTE/ABORT live in PoweredGuidancePanel.cs.
//
// Shares VehicleAutopilotState with the legacy Landing tab's Deorbit sub-tab, so both
// drive the same flow; that sub-tab can be deleted once this has flown.
public static partial class PoweredGuidanceWindow
{
    /// <summary>
    /// Phase as the operator thinks of it. The landing machine's own names are about
    /// which solver is driving, which is not the question being asked here.
    /// </summary>
    private static string DescentPhaseLabel()
    {
        switch (_s.LandingPhase)
        {
            case LandingPhase.Coast: return "COAST TO BURN";
            case LandingPhase.Prep: return "CONVERGING";
            case LandingPhase.Burn: return "DEORBIT BURN";
            case LandingPhase.GfoldDescent: return "G-FOLD";
            case LandingPhase.TerminalHover: return "HOVER";
            case LandingPhase.Done: return "DONE";
            default: return "IDLE";
        }
    }

    private static bool DescentLive =>
        _s.LandingPhase != LandingPhase.Idle && _s.LandingPhase != LandingPhase.Done;

    private static void DrawDescentStatus(float2 origin, float innerW, float rowH)
    {
        bool live = DescentLive;

        // Which countdown is meaningful depends on the phase: before ignition it is
        // time TO the burn, during it the burn's own tgo, and under G-FOLD the
        // predicted arrival. Showing one of them in all three would be wrong twice.
        double tgoSec;
        switch (_s.LandingPhase)
        {
            case LandingPhase.Coast:
            case LandingPhase.Prep:
                tgoSec = Math.Max(0.0, _s.BurnStartTime - SimNow());
                break;
            case LandingPhase.GfoldDescent:
                tgoSec = Math.Max(0.0, _s.GfoldArrivalTime - SimNow());
                break;
            default:
                tgoSec = _s.Upfg.Tgo;
                break;
        }

        ImColor8 col = !live ? SchemDim
            : _s.LandingPhase == LandingPhase.Prep && !_s.Upfg.Converged ? SchemBurn
            : SchemVgo;

        // decelerating: the marker turns round, because a descent flies its track
        // backwards — thrust opposes travel.
        DrawGuidanceStatusBlock(origin, innerW, rowH, DescentPhaseLabel(), col, live,
            tgoSec, _s.Upfg.VgoMag, decelerating: true);
    }

    private static void DrawDescentTabContent(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                              double mu, double bodyRadius, float innerW)
    {
        DrawDescentStatus(ImGui.GetCursorScreenPos(), ImGui.GetContentRegionAvail().X,
            ImGui.GetTextLineHeightWithSpacing());
        ImGui.Separator();

        // Directly under the status block, as a strip rather than a list.
        DrawPassStrip(orbit, parent, mu, bodyRadius,
            ImGui.GetCursorScreenPos(), ImGui.GetContentRegionAvail().X,
            ImGui.GetTextLineHeightWithSpacing());
        ImGui.Separator();

        DrawLandingSiteSection(parent, orbit, bodyRadius, innerW);
        DrawApproachSection(innerW);
    }

    // --- Landing site -------------------------------------------------------
    // The main levers: where to land, and whether the mod is allowed to fly it there.
    private static void DrawLandingSiteSection(IParentBody parent, Orbit orbit,
                                               double bodyRadius, float innerW)
    {
        if (!ImGuiHelper.BeginRegion("Landing site",
                ImGuiTreeNodeFlags.DefaultOpen | ImGuiTreeNodeFlags.SpanAllColumns, innerW))
            return;

        GaugeRow("Latitude (deg)", "##sitelat", ref _s.SiteLatDeg);
        GaugeRow("Longitude (deg)", "##sitelon", ref _s.SiteLonDeg);

        GaugeRowCheck("Engage autopilot", "##dengage", ref _s.Engage);
        GaugeRowCheck("Auto engines/staging", "##dautostage", ref _s.AutoStage);

        // The same toggle the Landing tab carries, on the same state: which solver
        // flies the powered descent is decided while planning it, not after arriving.
        ImGui.Text("Solver");
        ImGui.NextColumn();
        DrawSolverRadios();
        ImGui.NextColumn();

        double3 r = orbit.StateVectors.PositionCci;
        double3 siteDir = SiteDirCciAt(parent, 0);
        GaugeRowText("Ground distance", $"{AngleBetween(r, siteDir) * bodyRadius / 1000.0:F1} km");
        GaugeRowText("Site terrain", $"{SiteTerrainHeight(parent):F0} m");
        if (DescentLive)
            GaugeRowText("Burn downrange",
                $"{_s.BurnDownrangeKm:F1} km  ({_s.DownrangeFactor:F2}x)");

        ImGuiHelper.EndRegion();
    }

    // --- Approach -----------------------------------------------------------
    // Collapsed by default: shaping the approach is tuning, not aiming.
    private static void DrawApproachSection(float innerW)
    {
        if (!ImGuiHelper.BeginRegion("Approach parameters",
                ImGuiTreeNodeFlags.SpanAllColumns, innerW))
            return;

        GaugeRow("Downrange factor", "##downrange", ref _s.DownrangeFactor);
        GaugeRow("Aim altitude (km)", "##aimalt", ref _s.AimAltKm);
        GaugeRow("Descent rate (m/s)", "##descrate", ref _s.DescentRate);
        GaugeRow("Gate uprange (km)", "##gateuprange", ref _s.GateUprangeKm);
        // Where the braking burn ends and G-FOLD takes over. It shapes this phase, so
        // it belongs here rather than with the G-FOLD tuning.
        GaugeRow("G-FOLD handoff T-gate (s)", "##gfoldhandoff", ref _s.GfoldHandoffTgo);

        ImGuiHelper.EndRegion();
    }

    // --- Upcoming passes ----------------------------------------------------

    /// <summary>
    /// How close each upcoming orbit brings the ground track to the site, as one
    /// horizontal strip: the site is the centre line, and every pass is a block
    /// placed left or right of it by its SIGNED closest approach — which side of the
    /// track the site fell on. Reading down the strip you see the track walking past
    /// the site orbit by orbit, and whether it is converging on it or drifting away.
    ///
    /// The soonest pass is the one you can actually act on, so it is the only one
    /// coloured: green when it is close enough to be worth committing to, through to
    /// red when it is not. The rest stay white — they are context, not choices.
    /// </summary>
    private static void DrawPassStrip(Orbit orbit, IParentBody parent, double mu,
                                      double bodyRadius, float2 origin, float width, float rowH)
    {
        // Closed form and cheap enough to rebuild every frame, so the strip tracks the
        // orbit live instead of lagging a timer - see PoweredGuidancePasses.cs.
        RefreshPasses(orbit, parent, mu, bodyRadius);
        int closest = ClosestPassIndex();
        int next = NextPassIndex();

        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        float lineH = ImGui.GetTextLineHeight();
        float stripH = rowH * 1.5f;
        float2 stripMin = new float2(origin.X, origin.Y + lineH * 2f + 2f);
        float2 stripMax = new float2(origin.X + width, stripMin.Y + stripH);
        float midX = origin.X + width * 0.5f;

        // Scale to the widest pass, with a floor so a single very close pass doesn't
        // blow one kilometre up to half the panel and imply a precision we don't have.
        float scaleKm = PassStripMinScaleKm;
        double lastT = 0.0;
        for (int i = 0; i < _s.Passes.Count; i++)
        {
            scaleKm = MathF.Max(scaleKm, (float)Math.Abs(_s.Passes[i].crossKm));
            lastT = Math.Max(lastT, _s.Passes[i].tSec);
        }
        // Quantised, so the axis holds still. Scaling to the exact widest pass meant
        // every small change in it re-scaled the strip and slid every other block —
        // motion that looked like the passes moving when it was only the ruler.
        scaleKm = NiceScale(scaleKm);

        dl.AddRectFilled(stripMin, stripMax, SchemTrack, 3f);
        dl.AddRect(stripMin, stripMax, SchemSpent, 3f);

        // Both readouts, each coloured to match its own marking on the strip: the next
        // one white like its border, the closest one in the same red-to-green it is
        // drawn in. Together they say "this is what is coming, and this is what is
        // worth waiting for" - which is the whole question the strip exists to answer.
        DrawPassCaption(dl, origin, "next   ", next, SchemBody);
        DrawPassCaption(dl, new float2(origin.X, origin.Y + lineH), "closest", closest,
            closest >= 0 ? PassProximityColour(_s.Passes[closest].minKm) : SchemDim);

        string scaleText = $"+/-{scaleKm:F0} km";
        dl.AddText(new float2(origin.X + width - ImGui.CalcTextSize(scaleText).X, origin.Y),
            SchemDim, scaleText);

        // The site.
        dl.AddLine(new float2(midX, stripMin.Y + 2f), new float2(midX, stripMax.Y - 2f),
            SchemInk, 1.5f);

        float half = width * 0.5f - PassStripBlockW;
        for (int i = 0; i < _s.Passes.Count; i++)
        {
            var pass = _s.Passes[i];
            if (pass.minKm >= 1e6)
                continue;   // no solution for this revolution

            float x = midX + (float)(pass.crossKm / scaleKm) * half;
            float2 a = new float2(x - PassStripBlockW * 0.5f, stripMin.Y + 3f);
            float2 b = new float2(x + PassStripBlockW * 0.5f, stripMax.Y - 3f);

            // Brightness carries TIME - soonest solid, later ones fading back - and it
            // applies to the chosen pass too. Exempting that one made it the brightest
            // block on the strip regardless of when it arrived, which is exactly the
            // reading the fade is there to prevent.
            float timeT = lastT > 1.0 ? (float)(pass.tSec / lastT) : 0f;
            dl.AddRectFilled(a, b, PassBlockColour(i == closest, pass.minKm, timeT), 2f);

            // The next one to arrive gets a bright border. Position and brightness are
            // both already spoken for, so being NEXT needs a channel of its own.
            if (i == next)
                dl.AddRect(a - new float2(2f, 2f), b + new float2(2f, 2f), SchemBody,
                    2f, ImDrawFlags.None, 1.5f);
        }

        ImGui.Dummy(new float2(width, lineH * 2f + 2f + stripH));
    }

    private static void DrawPassCaption(ImDrawListPtr dl, float2 at, string label, int index,
                                        ImColor8 col)
    {
        dl.AddText(at, index >= 0 ? col : SchemDim, index >= 0
            ? $"{label} {_s.Passes[index].minKm,7:F1} km  in {_s.Passes[index].tSec,6:F0} s"
            : $"{label}     --- km  in    --- s");
    }

    /// <summary>Round up to a 1-2-5 step, so an auto-scaled axis stops jittering.</summary>
    private static float NiceScale(float v)
    {
        float mag = MathF.Pow(10f, MathF.Floor(MathF.Log10(MathF.Max(v, 1e-3f))));
        float norm = v / mag;
        float step = norm <= 1f ? 1f : norm <= 2f ? 2f : norm <= 5f ? 5f : 10f;
        return step * mag;
    }

    /// <summary>
    /// A block's colour: the chosen pass in its red-to-green proximity shade, every
    /// other one plain white, and ALL of them faded toward the strip by how far off
    /// they are in time. One path, so the fade cannot be skipped for a special case.
    /// </summary>
    private static ImColor8 PassBlockColour(bool chosen, double minKm, float timeT)
    {
        (byte r, byte g, byte b) = chosen ? PassProximityRgb(minKm) : PassNearRgb;
        float k = Math.Clamp(timeT, 0f, 1f) * PassFadeDepth;
        return new ImColor8(
            (byte)(r + (PassFarRgb.R - r) * k),
            (byte)(g + (PassFarRgb.G - g) * k),
            (byte)(b + (PassFarRgb.B - b) * k));
    }

    /// <summary>
    /// Green through red by how close a pass comes. The thresholds are a rule of
    /// thumb for "is this pass worth committing to", not a capability model — nothing
    /// here knows the vehicle's actual cross-range divert.
    /// </summary>
    private static ImColor8 PassProximityColour(double minKm)
    {
        (byte r, byte g, byte b) = PassProximityRgb(minKm);
        return new ImColor8(r, g, b);
    }

    private static (byte R, byte G, byte B) PassProximityRgb(double minKm)
    {
        float t = (float)Math.Clamp((minKm - PassGreenKm) / (PassRedKm - PassGreenKm), 0.0, 1.0);
        return ((byte)(90 + t * (255 - 90)),
                (byte)(235 - t * (235 - 70)),
                (byte)(130 - t * (130 - 70)));
    }
}
