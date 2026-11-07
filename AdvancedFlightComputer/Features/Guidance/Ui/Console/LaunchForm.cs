#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using KSA;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

public static partial class GuidanceWindow
{
    private const string LimitsFoldId = "LaunchLimits";

    internal const string BoostersFoldId = "LaunchBoosters";

    /// <summary>Seeds the LAN from where the vessel is the first time the launch is looked at, or a launch would go toward LAN 0. It runs whenever the page draws, also while its rows are folded away.</summary>
    internal static void SeedLaunchLan(Vehicle vehicle)
    {
        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null || _s.LanSeeded)
            return;
        _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg, parent);
        _s.LanSeeded = true;
    }

    internal static bool DrawLaunchPlanRow()
    {
        string reads = FlightView.LaunchReads(_s.PeKm, _s.ApKm, _s.IncDeg, _s.LaunchArmed && !_s.Running, _s.AutoLaunch);
        ConsoleUi.Gap();
        return ConsoleUi.Fold(FlightView.PlanLabel(reads), FlightView.PlanFoldId, false, PageText.PlanTip);
    }

    internal static void DrawLaunchPlanLine() => ConsoleUi.MutedWrapped(FlightView.LaunchPlanLine(_s.Running, _s.LaunchArmed));

    /// <summary>The rows above the picture, in the order a pilot decides them: the orbit and its plane, the target, the launch window, how it climbs. Each changes what the craft does.</summary>
    internal static void DrawLaunchInputs(Vehicle vehicle, float innerW)
    {
        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;
        double bodyRadius = parent.MeanRadius;

        SeedLaunchLan(vehicle);

        // Read only. The launch window step writes the chase result into the state (StepLaunchWindow), so the draw never does.
        ChaseStatus status = TryChaseOrbit(vehicle, orbit, parent, bodyRadius, out ChasePlan plan);
        AscentMethodUi method = CurrentAscentMethodUi();

        bool driven = status == ChaseStatus.Ok || status == ChaseStatus.PlaneUnreachable;
        using (new ImGuiDisabledScope(AscentPlanBusy))
        {
            float fieldW = DrawOrbitRow(orbit, parent, driven);
            DrawPlaneRow(orbit, parent, driven, fieldW);
        }
        using (ConsoleUi.Dim(_s.Running))
        using (new ImGuiDisabledScope(AscentPlanBusy))
        {
            DrawTargetPicker(vehicle);
            DrawTargetFacts(status, in plan);
            DrawLaunchWindowRows(status, in plan, method.Id == AscentMethod.CatsUpfg);
            DrawAscentMethodRow();
        }
        method.DrawPlayer(vehicle, orbit, parent, bodyRadius, innerW);
    }

    internal static void DrawLaunchPage(Vehicle vehicle, float innerW, bool foldsOpen)
    {
        DrawLaunchLimits(vehicle, foldsOpen);
        DrawLaunchBoosters(vehicle, foldsOpen);
        DrawLaunchHow(vehicle, innerW, foldsOpen);
    }

    internal static void DrawLaunchLimits(Vehicle vehicle, bool foldsOpen)
    {
        IParentBody parent = vehicle.Orbit?.Parent;
        if (parent == null)
            return;
        ConsoleUi.Gap();
        if (CurrentAscentMethodUi().Id == AscentMethod.CatsUpfg)
            DrawLimitsFold(parent, foldsOpen);
    }

    internal static void DrawLaunchBoosters(Vehicle vehicle, bool foldsOpen)
    {
        if (vehicle.Orbit?.Parent != null)
            DrawBoostersFold(vehicle, foldsOpen);
    }

    internal static void DrawLaunchHow(Vehicle vehicle, float innerW, bool foldsOpen)
    {
        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;
        double bodyRadius = parent.MeanRadius;
        AscentMethodUi method = CurrentAscentMethodUi();

        int changed = SettingAccess.CountChanged(_s, Specs.LaunchExpert, Specs.LaunchExpertBools);
        Ui.Console.Pages.PageKit.HowItFlies("launch",
        [
            ResetPart("launch", vehicle, Specs.LaunchExpert, Specs.LaunchExpertBools, changed),
            // The header of the picture holds this switch once a plan or a track exists. Until then the map still draws the target orbit, so the switch stays reachable here. It stands over the heads of the fold, because it belongs to none of them, and its name stays at the end of the label.
            new FoldPart("", !LaunchPictureShows, DrawMapChoiceRow),
            new FoldPart(FoldNames.Plane, true, InExpert(() => DrawOrbitPlaneSection(vehicle, orbit, parent, bodyRadius))),
            new FoldPart(FoldNames.Insertion, true, InExpert(DrawInsertionSection)),
            .. method.ExpertParts(innerW),
            new FoldPart(FoldNames.InMap, !LaunchPictureShows),
        ], changed, foldsOpen, changed > 0 ? "" : PageText.HowItFliesDefaults);
    }

    private static void DrawMapChoiceRow()
    {
        int picked = ConsoleUi.ChoiceRow(PageText.ShowInMap, "AfcLaunchInMapRow", PageText.MapChoices, FormRules.MapChoice(_showAscentOverlay, _s.ShowConvexPlan), labelTip: PageText.MapChoiceTip);
        if (picked >= 0)
            (_showAscentOverlay, _s.ShowConvexPlan) = FormRules.ApplyMapChoice(picked, _s.ShowConvexPlan);
    }

    /// <summary>
    /// BOOSTERS: the stages of this craft that can fly themselves home, each with its SITE, before launch. A site asks the calculated climb for a reserve, which it sizes for the boostback to that site and the landing, and the RESERVE row shows what the booster keeps. Its rows draw while the craft has a returnable stage, and also after a reserve was staged, so the RESERVE row stays readable. With none of these the fold says why. After separation the boostback is on LAND > BOOSTBACK.
    /// </summary>
    private static void DrawBoostersFold(Vehicle vehicle, bool openByDefault)
    {
        int stages = _s.ReturnableStages.Count;
        int withSite = 0;
        foreach (ReturnableStage stage in _s.ReturnableStages)
            if (stage.HasTarget)
                withSite++;
        if (!ConsoleUi.Fold(PageText.BoostersLabel(stages, withSite, _s.ReserveArmed ? _s.ReserveKg : 0.0), BoostersFoldId, openByDefault, PageText.BoostersTip(stages)))
            return;
        ImGui.PushID(BoostersFoldId);
        try
        {
            if (stages == 0 && !_s.ReserveStaged)
            {
                ConsoleUi.MutedWrapped(PageText.BoostersNone);
                return;
            }
            DrawBoosterLanes(vehicle);
            DrawReturnableStageRows();
            DrawBoosterReserve();
            if (stages > 0)
                GaugeRowText("", "Stage sites apply at booster separation.", ConsoleStyle.TextMuted);
            if (ConsoleUi.LinkRight("AFTER SEPARATION: LAND > BOOSTBACK", "AfcGoBoostback",
                    "Opens LAND > BOOSTBACK. Shows and starts the boostback of a separated booster."))
                PendingNavigation = ConsolePages.CueTarget(ConsolePage.Land, ConsolePages.EnumKey(LandWay.Boostback));
        }
        finally
        {
            ImGui.PopID();
        }
    }
}
