#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Canvas;
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
            float fieldW = DrawOrbitRow(orbit, parent, driven, driven && plan.Target == null && _s.TargetId.Length == 0);
            DrawPlaneRow(orbit, parent, driven, fieldW);
        }
        using (ConsoleUi.Dim(_s.Running))
        using (new ImGuiDisabledScope(AscentPlanBusy))
        {
            DrawTargetPicker(vehicle);
            DrawMoonTransferPicker(vehicle);
            DrawTargetFacts(status, in plan);
            DrawReachRow(orbit, parent, status, in plan);
            DrawTwrRow();
            DrawLaunchWindowRows(status, in plan, method.Id == AscentMethod.CatsUpfg);
            DrawAscentMethodRow();
        }
        method.DrawPlayer(vehicle, orbit, parent, bodyRadius, innerW);
    }

    internal static void DrawLaunchPage(Vehicle vehicle, float innerW, bool foldsOpen)
    {
        DrawLaunchBoosters(vehicle);
        DrawLaunchLimits(vehicle, foldsOpen);
        DrawLaunchHow(vehicle, innerW, foldsOpen);
    }

    internal static void DrawLaunchLimits(Vehicle vehicle, bool foldsOpen)
    {
        IParentBody parent = vehicle.Orbit?.Parent;
        if (parent == null)
            return;
        if (CurrentAscentMethodUi().Id == AscentMethod.CatsUpfg)
            DrawLimitsFold(parent, foldsOpen);
    }

    /// <summary>REUSE, the first fold under the picture: the returnable stages and their sites, above LIMITS.</summary>
    internal static void DrawLaunchBoosters(Vehicle vehicle)
    {
        if (vehicle.Orbit?.Parent == null || (_s.ReturnableStages.Count == 0 && _s.ReturnControlRefusals.Count == 0
            && _s.ReserveStagedRoots.Count == 0 && _s.StageTargets.Count == 0 && _s.StageOptimalReturn.Count == 0))
            return;
        ConsoleUi.Gap();
        ConsoleUi.SectionHead("BOOSTER RETURN");
        ImGui.PushID(BoostersFoldId);
        try
        {
            DrawUnavailableReturns(vehicle);
            DrawReturnableStageRows(vehicle);
            DrawBoosterReserve();
        }
        finally
        {
            ImGui.PopID();
        }
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
        {
            (_showAscentOverlay, _s.ShowConvexPlan) = FormRules.ApplyMapChoice(picked, _s.ShowConvexPlan);
            SaveOverlayPreferences();
        }
    }

    private static void DrawUnavailableReturns(Vehicle vehicle)
    {
        uint? clear = null;
        foreach (var (rootId, name, sequence, reason) in _s.ReturnControlRefusals)
            Draw(rootId, $"{name} in sequence {sequence} cannot return. {ReturnControlReason(reason)}");
        foreach (uint rootId in _s.StageTargets.Keys)
            DrawSaved(rootId);
        foreach (uint rootId in _s.StageOptimalReturn)
            if (!_s.StageTargets.ContainsKey(rootId))
                DrawSaved(rootId);
        if (clear.HasValue)
            ClearStageReturn(_s, clear.Value);

        void DrawSaved(uint rootId)
        {
            if (UnlistedStageWithReturn(vehicle, _s, rootId) is { } part
                && !_s.ReturnControlRefusals.Exists(refusal => refusal.RootId == rootId))
                Draw(rootId, $"{part.DisplayName} has a saved return target. Planning for this stage is unavailable.");
        }

        void Draw(uint rootId, string text)
        {
            ConsoleUi.MutedWrapped(text);
            if (HasStageReturn(_s, rootId)
                && ConsoleWidgets.Button("CLEAR RETURN".AsSpan(), $"AfcUnavailableStageClear{rootId}".AsSpan(), default))
                clear = rootId;
        }
    }

    private static bool HasStageReturn(VehicleAutopilotState state, uint rootId)
        => state.StageTargets.ContainsKey(rootId) || state.StageOptimalReturn.Contains(rootId);

    internal static Part UnlistedStageWithReturn(Vehicle vehicle, VehicleAutopilotState state, uint rootId)
        => HasStageReturn(state, rootId) && !state.ReturnableStages.Exists(stage => stage.RootId == rootId)
            ? vehicle.Parts.Find(rootId)
            : null;

    internal static void ClearStageReturn(VehicleAutopilotState state, uint rootId)
    {
        state.StageTargets.Remove(rootId);
        state.StageOptimalReturn.Remove(rootId);
        state.OptimalReturnSites.Remove(rootId);
        if (_retargetStageId == rootId)
        {
            _retargetStageId = 0;
            _retargetArmed = false;
        }
        state.StageModelDirty = true;
    }

    internal static string ReturnControlReason(VehicleControl reason) => reason switch
    {
        VehicleControl.NoCrew => "Its command pod has no crew.",
        VehicleControl.CrewIncapacitated => "Its command pod has no able crew.",
        VehicleControl.ProbeCoreUnpowered => "Its probe core has no power after separation.",
        VehicleControl.ProbeCoreOff => "Its probe core is switched off.",
        VehicleControl.ProbeCoreFailed => "Its probe core has failed.",
        VehicleControl.ProbeCoreStormStandby => "Its probe core is in storm standby.",
        VehicleControl.ProbeCoreRebooting => "Its probe core is rebooting.",
        _ => "It has no usable control after separation."
    };
}
