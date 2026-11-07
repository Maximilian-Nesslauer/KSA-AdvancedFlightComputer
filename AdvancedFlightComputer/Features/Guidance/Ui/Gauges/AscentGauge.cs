#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Logic;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

public static partial class GuidanceWindow
{
    private static bool AscentPlanBusy => _s.AscentPlanJob != null || _s.AscentPlanRequested;

    private const string FromPositionVerb = "FROM POSITION";

    private const float OrbitFieldLeast = 56f;

    // A row of the page. Picking a target turns the three orbit inputs into OUTPUTS of that pick, continuously recomputed and greyed out, because a chase orbit that disagreed with the target it was chasing was never anything but a mistake. Returns the width of its fields, so the row under it can line up.
    private static float DrawOrbitRow(Orbit orbit, IParentBody parent, bool driven)
    {
        float avail = ConsoleStyle.ContentAvailWidth();
        float cells = avail - ConsoleUi.LabelColumnWidth;
        float fieldW = CellFieldWidth(cells, out float rowWidth, ("PE", Specs.PeKm.Unit), ("AP", Specs.ApKm.Unit), ("INC", Specs.IncDeg.Unit));
        // In a narrow pane the fields need the room of the label, because PE, AP and INC name the row by themselves.
        bool label = fieldW >= OrbitFieldLeast * ImGuiHelper.InterfaceScale;
        if (!label)
        {
            cells = avail;
            fieldW = CellFieldWidth(cells, out rowWidth, ("PE", Specs.PeKm.Unit), ("AP", Specs.ApKm.Unit), ("INC", Specs.IncDeg.Unit));
        }
        // The name takes the violet of the band of the goal orbit in the picture, because these values set that band. With no room for the name PE and AP take it, and with no band no name has it.
        uint? bandInk = LaunchOrbitBandDraws ? Palette.Target.U32 : null;
        uint? cellInk = label ? null : bandInk;
        ConsoleUi.BeginRow(label ? "ORBIT" : "", cells, bandInk);
        float2 control = ImGui.GetCursorScreenPos();
        float x = control.X + ConsoleWidgets.RowControlWidth - rowWidth;

        using (new ImGuiDisabledScope(driven || AscentPlanBusy))
        {
            // Periapsis is always the lower: a pair entered the other way round swaps once the field is left (see OrderApsides).
            GaugeCell(Specs.PeKm, "PE", ref x, control.Y, fieldW, ref _s.PeKm, nameInk: cellInk);
            if (RowEditEnded(Specs.PeKm))
                OrderApsides();
            GaugeCell(Specs.ApKm, "AP", ref x, control.Y, fieldW, ref _s.ApKm, nameInk: cellInk);
            if (RowEditEnded(Specs.ApKm))
                OrderApsides();
            // A new inclination makes the old LAN meaningless, because the plane it named no longer passes overhead, so re-seed it from where the vehicle is.
            if (GaugeCell(Specs.IncDeg, "INC", ref x, control.Y, fieldW, ref _s.IncDeg))
                _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg, parent);
        }
        ConsoleUi.EndRow();
        return fieldW;
    }

    private static void DrawPlaneRow(Orbit orbit, IParentBody parent, bool driven, float fieldW)
    {
        BeginKitRow("PLANE", controlWidth: ConsoleStyle.ContentAvailWidth() - ConsoleUi.LabelColumnWidth);
        float2 control = ImGui.GetCursorScreenPos();
        float x = control.X + ConsoleWidgets.RowControlWidth - CellsWithoutFields([("LAN", Specs.LanDeg.Unit)]) - fieldW;
        if (!driven)
        {
            float buttonW = ConsoleWidgets.ButtonWidth(FromPositionVerb.AsSpan());
            ImGui.SetCursorScreenPos(new float2(x - CellGap - buttonW, control.Y));
            bool fromPosition;
            using (new ImGuiDisabledScope(AscentPlanBusy))
                fromPosition = ConsoleWidgets.Button(FromPositionVerb.AsSpan(), "AfcLanFromPosition".AsSpan(), new float2(buttonW, ConsoleStyle.FrameHeightPx));
            if (fromPosition && !AscentPlanBusy)
                _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg, parent);
            if (ImGui.IsItemHovered(ImGuiHoveredFlags.AllowWhenDisabled))
                ConsoleUi.Tooltip(PageText.LanFromPositionTip.AsSpan());
        }
        using (new ImGuiDisabledScope(driven || AscentPlanBusy))
            GaugeCell(Specs.LanDeg, "LAN", ref x, control.Y, fieldW, ref _s.LanDeg);
        ConsoleUi.EndRow();
    }

    private static void DrawTargetFacts(ChaseStatus status, in ChasePlan plan)
    {
        switch (status)
        {
            case ChaseStatus.NotFound:
                GaugeRowText("TARGET", "Not found", ConsoleStyle.Danger);
                break;
            case ChaseStatus.DifferentBody:
                GaugeRowText("TARGET", "Orbits another body", ConsoleStyle.Danger);
                break;
            case ChaseStatus.Ok:
                GaugeRowText("ITS ORBIT", PageText.TargetOrbit(plan.TargetPeKm, plan.TargetApKm, plan.IncDeg));
                break;
        }
    }

    private static void DrawLaunchWindowRows(ChaseStatus status, in ChasePlan plan, bool nodeChoiceExists)
    {
        LaunchWindowRow row = FormRules.WindowRow(_s.TargetId.Length > 0, status == ChaseStatus.Ok && double.IsFinite(plan.WaitSec),
            status == ChaseStatus.PlaneUnreachable, nodeChoiceExists && (!_s.FlyConvexAscent || BackupLaunchChosen));
        ConsoleUi.NoteShape("AfcLaunchWindow", (int)row);
        switch (row)
        {
            case LaunchWindowRow.NodeChoice:
                int picked = ConsoleUi.ChoiceRow("WINDOW", "AfcLaunchNode", PageText.NodeLabels, _s.LaunchDescending ? 1 : 0, default, PageText.NodeCards, "",
                    Specs.LaunchDescending.Help);
                if (picked >= 0)
                    _s.LaunchDescending = picked == 1;
                break;
            case LaunchWindowRow.Countdown:
                // To IGNITION, which leads the plane crossing by LanLeadSeconds, or, once the calculated climb chose its own launch time for this window, to that.
                GaugeRowText("WINDOW", PageText.WindowCountdown(plan.WaitSec, LaunchNodeDescending(in plan), LanLeadSeconds, ExecuteLaunchInstant() - WindowInstant()));
                GaugeRowCheck(Specs.AutoLaunch, ref _s.AutoLaunch);
                DrawAutoLaunchArming();
                break;
            case LaunchWindowRow.NoCrossing:
                GaugeRowText("WINDOW", PageText.NoCrossing, ConsoleStyle.Danger);
                break;
        }
    }

    // The LIMITS fold of the page: the limits of the climb that the plan and the flight read. MAX Q shows with either climb, because CALCULATE solves a plan with SIMPLE TURN too. The limit is part of the problem, so it is frozen while the problem is being solved.
    private static void DrawLimitsFold(IParentBody parent, bool openByDefault)
    {
        bool air = HasPhysicalAtmosphere(parent);
        if (!ConsoleUi.Fold(PageText.LimitsLabel(air, _s.ConvexQMaxKpa, _s.GLimitEnabled, _s.GLimitG), LimitsFoldId, openByDefault, PageText.LimitsTip(air)))
            return;
        ImGui.PushID(LimitsFoldId);
        try
        {
            using (ConsoleUi.Dim(_s.Running))
            using (new ImGuiDisabledScope(AscentPlanBusy))
            {
                if (air)
                    GaugeRow(Specs.ConvexQMaxKpa, ref _s.ConvexQMaxKpa);
                else
                    GaugeRowText(Specs.ConvexQMaxKpa.Tier, Specs.ConvexQMaxKpa.Label, PageText.NoAirRow, ConsoleStyle.TextMuted);
            }
            GaugeRowCheck(Specs.GLimitEnabled, ref _s.GLimitEnabled);
            // Greyed rather than hidden, because the limit is still worth reading when it is off.
            using (new ImGuiDisabledScope(!_s.GLimitEnabled))
                GaugeRow(Specs.GLimitG, ref _s.GLimitG);
        }
        finally
        {
            ImGui.PopID();
        }
    }

    // How the orbit plane is held, in the HOW IT FLIES fold. The LAN stands on the page, because it changes which orbit the craft goes to.
    private static void DrawOrbitPlaneSection(Vehicle vehicle, Orbit orbit, IParentBody parent, double bodyRadius)
    {
        if (!GuidanceRegion.Begin(FoldNames.Plane, GuidanceRegion.Expert))
            return;

        // Read only, because the rows grey out on it. The launch window step writes the chase result into the state (StepLaunchWindow), so the draw never does.
        ChaseStatus status = TryChaseOrbit(vehicle, orbit, parent, bodyRadius, out ChasePlan plan);
        bool driven = status == ChaseStatus.Ok || status == ChaseStatus.PlaneUnreachable;
        if (driven)
        {
            GaugeRow(Specs.ChaseOffsetKm, ref _s.ChaseOffsetKm);
            using (new ImGuiDisabledScope(double.IsNaN(plan.ArgPeDeg)))
                GaugeRowCheck(Specs.MatchTargetArgPe, ref _s.MatchTargetArgPe);
        }

        using (new ImGuiDisabledScope(driven || AscentPlanBusy))
        {
            GaugeRowCheck(Specs.ArgPeFixed, ref _s.ArgPeFixed);
            using (new ImGuiDisabledScope(!_s.ArgPeFixed))
                GaugeRow(Specs.ArgPeDeg, ref _s.ArgPeDeg);
        }

        GuidanceRegion.End();
    }

    private static void DrawInsertionSection()
    {
        if (!GuidanceRegion.Begin(FoldNames.Insertion, GuidanceRegion.Expert))
            return;

        // Outside the target's greying, since it is how to fly rather than where to go. Greyed instead while the argument of periapsis is fixed, which decides the insertion point by itself.
        using (new ImGuiDisabledScope(_s.ArgPeFixed))
        {
            GaugeRowCheck(Specs.OptimiseInsertion, ref _s.OptimiseInsertion);
            GaugeRowCheck(Specs.InsertBeforeApoapsis, ref _s.InsertBeforeApoapsis);
        }

        GuidanceRegion.End();
    }

    // Where the insertion row turns amber. Not limits, because nothing is refused and what an insertion costs depends on the vehicle, but where the cost has stopped being small: far round the ellipse from periapsis, or climbing steeply onto it.
    private const double InsertionWarnAnomalyDeg = 45.0;
    private const double InsertionWarnFpaDeg = 5.0;

    // Below this the insertion is before periapsis in any sense worth saying, rather than hovering over it.
    private const double InsertionDescendingFpaDeg = -0.5;

    /// <summary>
    /// Where the running ascent is aiming to insert on the target ellipse, whenever that is anywhere but periapsis: how far round from periapsis, at what altitude, and at what flight-path angle. These are the numbers a fixed argument of periapsis spends dV on, and the insertion search saves dV with. Drawn only while there is a solution to read them from.
    /// </summary>
    private static void DrawInsertionRows(double bodyRadius)
    {
        var aim = _s.Upfg.Aim;
        // A free insertion off periapsis that the floor did not put there is the search's doing.
        bool searched = !_s.ArgPeFixed && !aim.FloorLimited && aim.TrueAnomaly != 0.0;
        if (!_s.Running || !aim.Valid || !(_s.ArgPeFixed || aim.FloorLimited || searched))
            return;

        double nuDeg = aim.TrueAnomaly * (180.0 / Math.PI);
        double fpaDeg = aim.Fpa * (180.0 / Math.PI);
        // Only a fixed argument of periapsis can put the insertion somewhere dear: the search moves it only to save dV, and the floor only to keep the cutoff out of the air.
        bool far = _s.ArgPeFixed && (Math.Abs(nuDeg) > InsertionWarnAnomalyDeg || Math.Abs(fpaDeg) > InsertionWarnFpaDeg);
        bool descending = _s.ArgPeFixed && fpaDeg < InsertionDescendingFpaDeg;
        float4 warn = ConsoleUi.WarnInk;
        float4 fine = ConsoleStyle.TextPrimary;
        const UiTier D = UiTier.Diagnostics;

        GaugeRowText(D, "INSERTION",
            FormattableString.Invariant($"{nuDeg:+0.0;-0.0} deg from Pe, {(aim.Radius - bodyRadius) / 1000.0:F0} km, FPA {fpaDeg:+0.0;-0.0}"),
            far || descending || aim.FloorLimited ? warn : fine);
        if (aim.FloorLimited)
            GaugeRowText(D, "", "Pe inside the air, so it inserts at the top of the air", warn);
        if (far)
            GaugeRowText(D, "", "far round from Pe - costs dV", warn);
        else if (descending)
            GaugeRowText(D, "", "before Pe - a burn from orbit may not converge", warn);
        if (searched && double.IsFinite(_s.InsertionSearch.SavingMs))
            GaugeRowText(D, "", FormattableString.Invariant($"placed for dV, {Math.Max(_s.InsertionSearch.SavingS, 0.0):F1} s of burn ({Math.Max(_s.InsertionSearch.SavingMs, 0.0):F0} m/s) under inserting {(_s.InsertionSearch.AtApoapsis ? FormattableString.Invariant($"{UpfgInsertionSearch.ApoapsisLeadDeg:F0} deg before Ap") : "at Pe")}"), fine);
    }

    private static void DrawTargetPicker(Vehicle vehicle)
    {
        BeginKitRow("TARGET");
        bool rowHovered = ConsoleWidgets.RowHovered;
        ConsoleUi.RowChip("TARGET", Owner.Stock, "STOCK");
        bool open = ConsoleUi.BeginCombo("AfcAscentTarget".AsSpan(), _s.TargetId.Length > 0 ? _s.TargetId : "None");
        if (!open && rowHovered)
            ConsoleUi.Card("TARGET", "A craft to launch into the orbit of. LAUNCH then waits for its launch window. " + PageText.CraftTargetChipTip);
        if (open)
        {
            // Dropping the target frees the argument of periapsis again, because the target's was only ever copied for the chase.
            if (ImGui.Selectable("None", _s.TargetId.Length == 0) && _s.TargetId.Length > 0)
            {
                _s.TargetId = "";
                _s.ArgPeFixed = false;
            }

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
                        // Mirror into the game's own targeting, so the map and the rendezvous gauge agree with us. Through the input buffer, because the draw overlaps the vehicle solver and a direct SetTarget rebuilds the flight plan it reads.
                        Universe.SetTargetCommand(vehicle.Id, v.Id);
                    }
                }
            }
            ConsoleWidgets.EndComboControl();
        }
        ConsoleUi.EndRow();
    }

    // How the vehicle flies the ascent, as opposed to where it is going.
    private static void DrawAscentSettingsSection(float innerW)
    {
        if (!GuidanceRegion.Begin(FoldNames.Turn, GuidanceRegion.Expert))
            return;

        GaugeRow(Specs.TurnStartAltKm, ref _s.TurnStartAltKm);
        GaugeRow(Specs.TurnRateDegS, ref _s.TurnRateDegS);

        // FORCE ROLL is on by default: the ascent commands a roll as well as a thrust direction and switches the flight computer out of decoupled roll so it tracks it (see VehicleAutopilotState.ForceRoll). Off, the ascent holds whatever roll the vehicle lifted off with.
        GaugeRowCheck(Specs.ForceRoll, ref _s.ForceRoll);
        using (new ImGuiDisabledScope(!_s.ForceRoll))
            GaugeRow(Specs.ForceRollDeg, ref _s.ForceRollDeg);

        GuidanceRegion.End();
    }

    /// <summary>The booster reserve the calculated climb sized for the booster with a stage site, kg; NaN with no usable plan for that site.</summary>
    private static double PlannedReserveBoostbackDv(out double landingDvMs)
    {
        landingDvMs = double.NaN;
        AscentPlan p = _s.AscentPlan;
        AscentSolution sol = p?.Solution;
        if (p == null || !p.Usable || !double.IsFinite(sol.RecoveryReserveKg) || p.RecoverySite != RecoverySiteKey(BoostbackTarget()))
            return double.NaN;
        landingDvMs = sol.LandingDv;
        return sol.BoostbackDv;
    }

    /// <summary>
    /// What the booster with a stage site keeps for its way home. There is nothing to set: the stage site arms the reserve, and the calculated climb sizes it for the boostback to that site and the landing burn (RefreshReserve). Idle is normal for part of a flight, because a strap-on stack does not arm until the strap-ons are gone, so the row says why (NextSeparationDropsAllEngines).
    /// </summary>
    private static void DrawBoosterReserve()
    {
        double boostbackDv = PlannedReserveBoostbackDv(out double landingDv);
        string text = PageText.BoosterReserve(BoostbackTarget() != null, _s.ReserveArmed, _s.ReserveStaged, _s.ReserveKg, boostbackDv, landingDv, _s.ReserveNote, out bool warn);
        if (text.Length == 0)
            return;
        GaugeRowText(UiTier.Player, "RESERVE", text, warn ? ConsoleUi.WarnInk : _s.ReserveStaged ? ConsoleStyle.TextMuted : ConsoleStyle.TextPrimary);
    }

    /// <summary>
    /// The stages that can fly themselves home, one row each with its STAGE SITE.
    ///  CONTROLLABILITY IS THE TEST, not size: a stage can be flown if the subtree that separates carries a command pod, which is the same thing Vehicle.IsControllable asks. An interstage does not qualify however large it is, and a stage that separates once the climb is over is going orbital speed and is not listed.
    ///  A SITE ON A STAGE ASKS FOR ITS RESERVE: the calculated climb sizes what that booster keeps for the boostback and the landing, which the RESERVE row shows.
    /// </summary>
    private static void DrawReturnableStageRows()
    {
        var list = _s.ReturnableStages;
        if (list.Count == 0)
            return;

        float4 dim = ConsoleStyle.TextMuted;

        for (int i = 0; i < list.Count; i++)
        {
            GuidanceWindow.ReturnableStage stage = list[i];

            // The name row carries the set button, because the button is about the stage rather than about any one of its numbers.
            bool arming = _retargetStageId == stage.RootId;
            string pick = arming ? "CLICK A SPOT" : "PICK STAGE SITE";
            float clearW = stage.HasTarget ? ConsoleWidgets.ButtonWidth("X".AsSpan()) + ImGui.GetStyle().ItemSpacing.X : 0f;
            float buttonsW = ConsoleWidgets.ButtonWidth(pick.AsSpan()) + clearW;
            BeginKitRow(stage.Name.ToUpperInvariant() + (stage.IsNext ? "  NEXT" : ""), controlWidth: buttonsW);
            ConsoleUi.RowRight(buttonsW);
            if (ConsoleWidgets.Button(pick.AsSpan(), $"AfcStageSet{stage.RootId}".AsSpan(), default))
            {
                // Arms the world click for THIS stage. Same click handler the landing retarget uses; the binding is what sends the answer somewhere else.
                _retargetStageId = arming ? 0u : stage.RootId;
                _retargetArmed = !arming;
            }
            if (stage.HasTarget)
            {
                ImGui.SameLine();
                if (ConsoleWidgets.Button("X".AsSpan(), $"AfcStageClear{stage.RootId}".AsSpan(), default))
                {
                    _s.StageTargets.Remove(stage.RootId);
                    _s.StageModelDirty = true;
                }
            }
            ConsoleUi.EndRow();

            GaugeRowText(stage.HasTarget ? "STAGE SITE" : "", stage.HasTarget ? Fmt.Sentence(Fmt.LatLon(stage.TargetLatDeg, stage.TargetLonDeg)) : "No stage site set", dim);
        }
    }

    private static void DrawAscentOptionsSection(double bodyRadius)
    {
        if (!GuidanceRegion.Begin("LIVE##ascent", GuidanceRegion.Diagnostics))
            return;

        if (_s.Running && _s.Upfg.Tgo > 0.0)
        {
            GaugeRowText(UiTier.Diagnostics, "SPEED TO GAIN", Fmt.DvWhole(_s.Upfg.VgoMag));
            GaugeRowText(UiTier.Diagnostics, "DISTANCE TO GO", Fmt.Km(_s.Upfg.Rgo.Length() / 1000.0));
        }
        DrawInsertionRows(bodyRadius);

        if (_s.TraceCount > 0)
        {
            ConsoleUi.SectionRow();
            if (ConsoleUi.ButtonRight("CLEAR TRACK", "AfcClearTrack"))
                ResetTrace();
        }

        GuidanceRegion.End();
    }
}
