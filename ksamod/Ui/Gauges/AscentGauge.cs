using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// The Ascent tab's content. The gauge shell, the tab bar and the EXECUTE/ABORT
// buttons live in Ui/Panel.cs; everything here draws inside the body
// child that panel opens, so it is plain ImGui under ImGaugeDressing's styling.
//
// The point of the restructure is that the legacy tab put thirty controls in one
// flat list, so the four that matter — the target orbit — sat among solver tuning.
// Here the two sections that shape a launch are open, and everything else is folded
// away behind "Expert settings", collapsed until asked for.
//
// It shares VehicleAutopilotState with the legacy Ascent tab, so both drive the same
// guidance; the tab can be deleted once this has flown.
public static partial class PoweredGuidanceWindow
{
    private static void DrawAscentTabContent(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                             double bodyRadius, float innerW)
    {
        // Status first: what the guidance is doing now, before anything you set.
        DrawAscentStatus(ImGui.GetCursorScreenPos(), ImGui.GetContentRegionAvail().X,
            ImGui.GetTextLineHeightWithSpacing());
        ImGui.Separator();

        DrawTargetOrbitSection(vehicle, orbit, parent, bodyRadius, innerW);
        DrawAscentSettingsSection(innerW);
        DrawExpertSettingsSection(innerW);
    }

    // --- Target orbit -------------------------------------------------------
    // The main levers. Picking a target turns the four orbit inputs into OUTPUTS of
    // that pick — continuously recomputed and greyed out — because a chase orbit that
    // disagreed with the target it was chasing was never anything but a mistake.
    private static void DrawTargetOrbitSection(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                               double bodyRadius, float innerW)
    {
        if (!ImGuiHelper.BeginRegion("Target orbit",
                ImGuiTreeNodeFlags.DefaultOpen | ImGuiTreeNodeFlags.SpanAllColumns, innerW))
            return;

        DrawTargetPicker(vehicle);

        ChaseStatus status = TryChaseOrbit(vehicle, orbit, parent, bodyRadius, out ChasePlan plan);
        bool driven = status == ChaseStatus.Ok || status == ChaseStatus.PlaneUnreachable;
        if (driven)
        {
            ApplyChaseOrbit(in plan);
            GaugeRow("SMA offset (km)", "##chaseoffset", ref _s.ChaseOffsetKm);
        }

        using (new ImGuiDisabledScope(driven))
        {
            GaugeRow("Periapsis (km)", "##pe", ref _s.PeKm);
            GaugeRow("Apoapsis (km)", "##ap", ref _s.ApKm);
            // A new inclination makes the old LAN meaningless — the plane it named
            // no longer passes overhead — so re-seed it from where the vehicle is.
            if (GaugeRow("Inclination (deg)", "##inc", ref _s.IncDeg))
                _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg, parent);
            GaugeRow("LAN (deg)", "##lan", ref _s.LanDeg);
        }

        if (!driven)
        {
            ImGui.Text("");
            ImGui.NextColumn();
            if (ImGui.Button("LAN from position"))
                _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg, parent);
            ImGui.NextColumn();
        }

        // Directly below the orbit parameters: these two decide what EXECUTE
        // actually does, so they belong with the launch, not buried in Expert.
        using (new ImGuiDisabledScope(!driven))
            GaugeRowCheck("Auto warp to window", "##autolaunch", ref _s.AutoLaunch);
        GaugeRowCheck("Auto engines/staging", "##autostage", ref _s.AutoStage);

        switch (status)
        {
            case ChaseStatus.NotFound:
                GaugeRowText("Target", "not found", new float4(1f, 0.4f, 0.4f, 1f));
                break;
            case ChaseStatus.DifferentBody:
                GaugeRowText("Target", "orbits another body", new float4(1f, 0.4f, 0.4f, 1f));
                break;
            case ChaseStatus.PlaneUnreachable:
                GaugeRowText("Launch window", "inclination below site latitude",
                    new float4(1f, 0.6f, 0.3f, 1f));
                break;
            case ChaseStatus.Ok:
                GaugeRowText("Target orbit",
                    $"{plan.TargetPeKm:F1} x {plan.TargetApKm:F1} km, inc {plan.IncDeg:F2} deg");
                // To IGNITION, which leads the plane crossing by LanLeadSeconds.
                GaugeRowText("Launch window",
                    $"T-{plan.WaitSec:F0} s ({(_s.LaunchDescending ? "descending" : "ascending")}, "
                    + $"{LanLeadSeconds:F0} s lead)");
                break;
        }

        ImGuiHelper.EndRegion();

        // Outside the region: it emits plain full-width status text, not column rows.
        if (status == ChaseStatus.Ok)
            DrawAutoLaunchArming();
    }

    private static void DrawTargetPicker(Vehicle vehicle)
    {
        ImGui.Text("Target");
        ImGui.NextColumn();
        ImGui.PushItemWidth(-1f);
        if (ImGui.BeginCombo("##ascenttarget", _s.TargetId.Length > 0 ? _s.TargetId : "(none)"))
        {
            if (ImGui.Selectable("(none)", _s.TargetId.Length == 0))
                _s.TargetId = "";

            CelestialSystem system = Universe.CurrentSystem;
            if (system != null)
            {
                ReadOnlySpan<Astronomical> all = system.All.AsSpan();
                for (int i = 0; i < all.Length; i++)
                {
                    if (all[i] is Vehicle v && !ReferenceEquals(v, vehicle)
                        && ImGui.Selectable(v.Id, v.Id == _s.TargetId))
                    {
                        _s.TargetId = v.Id;
                        // Mirror into the game's own targeting, so the map and the
                        // rendezvous gauge agree with us.
                        Universe.SetTarget(vehicle, v);
                    }
                }
            }
            ImGui.EndCombo();
        }
        ImGui.PopItemWidth();
        ImGui.NextColumn();
    }

    // --- Ascent settings ----------------------------------------------------
    // How the vehicle flies the ascent, as opposed to where it is going.
    private static void DrawAscentSettingsSection(float innerW)
    {
        if (!ImGuiHelper.BeginRegion("Ascent settings",
                ImGuiTreeNodeFlags.DefaultOpen | ImGuiTreeNodeFlags.SpanAllColumns, innerW))
            return;

        GaugeRowCheck("G-limit", "##glimit", ref _s.GLimitEnabled);
        // Greyed rather than hidden: the limit is still worth reading when it is off.
        using (new ImGuiDisabledScope(!_s.GLimitEnabled))
            GaugeRow("Max accel (g)", "##glimitg", ref _s.GLimitG);

        GaugeRow("Turn start alt (km)", "##turnalt", ref _s.TurnStartAltKm);
        GaugeRow("Turn rate (deg/s)", "##turnrate", ref _s.TurnRateDegS);

        // Roll is FREE by default — the ascent holds whatever the vehicle lifted off
        // with and commands a thrust direction only. Ticking this commands a roll as
        // well, and switches the flight computer out of decoupled roll so it tracks it
        // (see VehicleAutopilotState.ForceRoll).
        GaugeRowCheck("Force roll", "##forceroll", ref _s.ForceRoll);
        using (new ImGuiDisabledScope(!_s.ForceRoll))
            GaugeRow("Roll angle (deg)", "##rollangle", ref _s.ForceRollDeg);

        DrawBoosterReserve();

        ImGuiHelper.EndRegion();
    }

    // --- Booster reserve ----------------------------------------------------

    /// <summary>
    /// Stage the first stage EARLY, leaving it the dV it needs to fly itself home, and
    /// hand it to boostback with the landing site. Zero switches it off.
    ///
    /// A dV RATHER THAN A MASS, and the readout shows both numbers for the same reason:
    /// what a reserve costs depends on what it has to lift, and that is the booster
    /// alone - m_dry * (exp(dv/ve) - 1), with the upper stage cancelling out of the
    /// rocket equation entirely. So a reserve that looks wrong is nearly always a
    /// booster dry mass that looks wrong, and the booster mass is the one worth
    /// checking against the vehicle in the editor.
    /// </summary>
    private static void DrawBoosterReserve()
    {
        GaugeRow("Booster reserve dV", "##reservedv", ref _s.AscentReserveDvMs);
        if (_s.AscentReserveDvMs < 0.0)
            _s.AscentReserveDvMs = 0.0;
        if (!(_s.AscentReserveDvMs > 0.0))
            return;

        float4 dim = new float4(0.7f, 0.7f, 0.7f, 1f);
        float4 warn = new float4(1f, 0.8f, 0.3f, 1f);

        if (!_s.ReserveArmed)
        {
            // Idle is the normal state for most of a flight - a strap-on stack does not
            // arm until the solids are gone - so it says WHY rather than just going
            // quiet. See PoweredGuidanceWindow.NextSeparationDropsAllEngines.
            GaugeRowText("Reserve", _s.ReserveNote.Length > 0 ? _s.ReserveNote
                                                              : "waiting for a stage model", warn);
            return;
        }

        GaugeRowText("Reserve",
            $"{_s.ReserveKg / 1000.0,8:F2} t   ({_s.ReserveBoosterDryKg / 1000.0:F1} t booster)",
            _s.ReserveStaged ? dim : new float4(0.4f, 1f, 0.4f, 1f));
        if (_s.ReserveNote.Length > 0)
            GaugeRowText("", _s.ReserveNote, warn);
        if (_s.ReserveStaged)
            GaugeRowText("", "staged - booster handed to boostback", dim);
    }

    // --- Expert settings ----------------------------------------------------
    // Collapsed by default — dropping DefaultOpen is the whole mechanism.
    private static void DrawExpertSettingsSection(float innerW)
    {
        if (!ImGuiHelper.BeginRegion("Expert settings", ImGuiTreeNodeFlags.SpanAllColumns, innerW))
            return;

        GaugeRowCheck("Engage autopilot", "##engage", ref _s.Engage);
        GaugeRowCheck("Descending node (SE)", "##descending", ref _s.LaunchDescending);
        GaugeRowCheck("Show orbit & track", "##overlay", ref _showAscentOverlay);

        ImGui.Text("");
        ImGui.NextColumn();
        if (ImGui.Button("Clear track"))
            ResetTrace();
        ImGui.NextColumn();

        ImGuiHelper.EndRegion();
    }
}
