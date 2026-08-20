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

        DrawLandingSiteSection(parent, orbit, bodyRadius, innerW);
        DrawApproachSection(innerW);
        DrawUpcomingPasses(orbit, parent, mu, bodyRadius);
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

        ImGui.Text("Retarget");
        ImGui.NextColumn();
        if (ImGui.Button(_retargetArmed ? "Click the surface (right-click cancels)" : "Click a spot"))
            _retargetArmed = !_retargetArmed;
        ImGui.NextColumn();

        GaugeRowCheck("Engage autopilot", "##dengage", ref _s.Engage);
        GaugeRowCheck("Auto engines/staging", "##dautostage", ref _s.AutoStage);

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
    // Left as plain text on purpose — a display for these is a separate job.
    private static void DrawUpcomingPasses(Orbit orbit, IParentBody parent, double mu,
                                           double bodyRadius)
    {
        ImGui.SeparatorText("Upcoming passes");

        // Time-sliced: start a scan while idle at normal speed and advance it a fixed
        // sample budget per frame, so a ~1200-propagation scan never lands as a hitch.
        bool refreshOk = !DescentLive
            && !Universe.IsAutoWarpActive
            && Universe.SimulationSpeed <= MaxScanSimSpeed;
        if (refreshOk && _s.ScanIndex < 0
            && Environment.TickCount64 - _s.PassesRefreshedAtMs > PassRefreshIntervalMs)
            StartPassScan(orbit, mu);
        StepPassScan(parent, mu, bodyRadius);

        // Always exactly PassesToShow lines, so the panel does not resize under a
        // scan that is still filling in.
        for (int i = 0; i < PassesToShow; i++)
        {
            if (i < _s.Passes.Count && _s.Passes[i].minKm < 1e6)
                ImGui.Text($"Pass {i + 1}:  closest {_s.Passes[i].minKm,8:F1} km   in {_s.Passes[i].tSec,7:F0} s");
            else if (i < _s.Passes.Count)
                ImGui.Text($"Pass {i + 1}:  (no solution)");
            else
                ImGui.Text($"Pass {i + 1}:  scanning...");
        }
    }
}
