#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

public static partial class GuidanceWindow
{
    private const string EngageTip =
        "AFC MAY STEER: guidance commands the attitude of this craft while one of its modes flies. Off while a mode holds the craft hands it back: the mode ends, the engine is cut and you fly. LAND NOW and HOVER turn it on when they start. DEORBIT AND LAND needs it on.";
    private const string AutoStageTip =
        "AFC MAY USE ENGINES: guidance lights the engines, throttles and stages this craft while one of its modes flies. Off while a mode flies keeps the steering with AFC and gives you the engine and the stages, and a landing from orbit ends. LAND NOW and HOVER turn it on when they start. DEORBIT AND LAND needs it on.";
    private const string AutoStageArmTip =
        "AUTOSTAGE: AFC stages this craft by itself when a stage burns out. The AUTOSTAGE gauge button and the STAGING page set the same switch.";

    /// <summary>
    /// The per-craft permissions of the drawer in one block: whether guidance may steer and whether it may stage and throttle, and whether AutoStage stages this craft by itself, with a link to STAGING. The AutoStage switch makes the same StagingDetector.Arm call as the STAGING page and the AUTOSTAGE gauge button. The two AFC MAY switches stay free while a mode flies, because a pilot can want the engine back and keep the steering, and each says under it what off does now.
    /// </summary>
    internal static void DrawCraftSwitches(Vehicle vehicle, bool guidanceLive, string autoStageNote = "")
    {
        if (vehicle == null)
            return;

        if (SharedVehicleHooks.GuidanceEnabled)
        {
            VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
            bool touchdown = state.Active || state.EngagePending;
            // A committed deorbit stops when either switch goes off (LandingSwitchesOff), so both are held while it waits or flies, and its STOP is the way out.
            bool deorbit = DeorbitLocks(state);
            string lockNote = deorbit ? PageText.DeorbitLockedSetting + "\n\n" : "";
            using (new ImGuiDisabledScope(deorbit))
                ConsoleUi.CheckboxRow(Specs.Engage.Label.AsSpan(), "AfcCraftEngage".AsSpan(), ref state.Engage,
                    (lockNote + EngageTip + "\n\n" + PageText.KeptRule).AsSpan());
            DrawSwitchNote(PageText.SteerSwitchNote(touchdown, state.ControlAcquired));
            using (new ImGuiDisabledScope(deorbit))
                ConsoleUi.CheckboxRow(Specs.AutoStage.Label.AsSpan(), "AfcCraftAutoStage".AsSpan(), ref state.AutoStage,
                    (lockNote + AutoStageTip).AsSpan());
            DrawSwitchNote(PageText.EnginesSwitchNote(touchdown, state.ControlAcquired));
        }

        if (SharedVehicleHooks.AutoStageEnabled)
            DrawAutoStageRow(vehicle, guidanceLive, "AUTOSTAGE", "", withLink: true, autoStageNote);
    }

    private static void DrawSwitchNote(string note)
    {
        if (note.Length > 0)
            ConsoleUi.MutedWrapped(note);
    }

    /// <summary>
    /// The AutoStage switch of a craft as a stock Checkbox row: the same StagingDetector.Arm call as the AUTOSTAGE gauge button. While
    /// guidance armed it for the flight that runs now, the tooltip says so, and a click still disarms it, because guidance keeps a
    /// player disarm for the rest of the flight (GuidanceWindow.AutoSequence). An unarmed switch is held with a <paramref name="refusal"/>, which stands under the row, on the page and on the card.
    /// With <paramref name="withLink"/> the row ends in a link to STAGING.
    /// </summary>
    internal static void DrawAutoStageRow(Vehicle vehicle, bool guidanceLive, string label, string refusal, bool withLink, string extraTip = "")
    {
        string note = AutoStageGuidanceNote(vehicle, guidanceLive);
        bool armed = StagingDetector.IsArmed(vehicle);
        string held = armed ? "" : refusal;
        ConsoleUi.BeginRow(label.AsSpan());
        float2 control = ImGui.GetCursorScreenPos();
        bool go = false;
        bool linkHovered = false;
        if (withLink)
        {
            go = ConsoleUi.Link("STAGING", "AfcCraftAutoStageGo" + label, "Opens STAGING: delays and spent stages.");
            linkHovered = ImGui.IsItemHovered();
            ImGui.SetCursorScreenPos(control);
        }
        bool flipped;
        using (new ImGuiDisabledScope(held.Length > 0))
            flipped = ConsoleUi.RowCheckbox(("AfcCraftAutoStageArm" + label).AsSpan(), ref armed);
        if (ConsoleWidgets.RowHovered && !linkHovered)
            ConsoleUi.Tooltip(((held.Length > 0 ? held + "\n\n" : "") + (note.Length > 0 ? note + "\n\n" : "") + AutoStageArmTip + (extraTip.Length > 0 ? " " + extraTip : "")
                + "\n" + PageText.TagNotKept).AsSpan());
        ConsoleUi.EndRow();
        ConsoleUi.NoteShape("AfcCraftAutoStageHeld" + label, held.Length > 0 ? 1 : 0);
        if (held.Length > 0)
            ConsoleUi.MutedWrapped(held);
        if (flipped && held.Length == 0)
            StagingDetector.Arm(vehicle, armed);
        if (go)
            PendingNavigation = ConsolePages.CueTarget(ConsolePage.Staging);
    }

    // The tooltip note while guidance armed AutoStage for the flight that runs now, or empty.
    internal static string AutoStageGuidanceNote(Vehicle vehicle, bool guidanceLive)
        => guidanceLive && VehicleAutopilotState.TryGet(vehicle, out VehicleAutopilotState s) && s.ArmedStaging && StagingDetector.IsArmed(vehicle)
            ? PageText.ArmedByGuidanceTip : "";

    internal static string MayOffLine(Vehicle vehicle)
        => vehicle != null && SharedVehicleHooks.GuidanceEnabled && VehicleAutopilotState.TryGet(vehicle, out VehicleAutopilotState s)
            ? PageText.MayOffLine(!s.Engage, !s.AutoStage)
            : "";
}
