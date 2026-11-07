using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

/// <summary>
/// The console's part of the AFC menu: OPEN AFC with the planner key as its shortcut while the key
/// runs on the controlled craft.
/// </summary>
internal static class ConsoleMenu
{
    public static void Register()
    {
        AfcMenu.Register(10, "console", Draw);
        AfcMenu.StopItem = StopItem;
    }

    private static (string Label, string Outcome, Action Act, bool AfcFlies)? StopItem()
    {
        if (!AfcConsole.Enabled)
            return null;
        StopSpec stop = AfcConsole.Frame.Stop;
        Vehicle? subject = AfcConsole.Frame.Subject;
        if (subject != null)
            return stop.Enabled ? ($"{stop.Label} ({subject.Id})", stop.Outcome, stop.Act!, stop.Target != StopTarget.StockAuto) : null;
        // Without a subject (the editor or an EVA) the menu offers the stop of the one craft AFC still flies, and leaves several to the drawer.
        return HeadDrawer.TryGetSingleOtherStop(out Vehicle? other, out StopSpec otherStop)
            ? ($"{otherStop.Label} ({other!.Id})", otherStop.Outcome, otherStop.Act!, otherStop.Target != StopTarget.StockAuto)
            : null;
    }

    /// <summary>The one AFC line in Settings > Mods: every AFC setting is in the console.</summary>
    internal static void DrawModsSettingsLine()
    {
        ConsoleWidgets.Rule();
        ConsoleUi.SectionHead("ADVANCED FLIGHT COMPUTER");
        string key = Input.GetAssignmentString(InputAction.ToggleTransferPlan);
        ConsoleUi.MutedWrapped(PlannerRoute.Active
            ? ConsoleUi.EscapeFormat($"AFC settings: console ({key}), STAGING + TUNE.")
            : "AFC settings: console (AFC menu > OPEN AFC), STAGING + TUNE.");
        if (ConsoleWidgets.Button("OPEN AFC".AsSpan(), "AfcSettingsOpen".AsSpan(), default))
            AfcConsole.Open(ConsolePage.Tune);
    }

    private static void Draw()
    {
        bool open = AfcConsole.IsShown;
        string key = Input.GetAssignmentString(InputAction.ToggleTransferPlan);
        string shortcut = PlannerRoute.Active ? key : "";
        if (ImGui.MenuItem("OPEN AFC", shortcut, ref open, AfcConsole.Enabled))
        {
            if (open)
                AfcConsole.Open();
            else
                AfcConsole.Close();
        }

        ImGui.Separator();
        foreach (ConsolePage page in ConsolePages.All)
        {
            if (page == ConsolePages.FirstSetupPage)
                ImGui.Separator();
            bool enabled = AfcConsole.Enabled && (!Program.IsEditorOpen || ConsolePages.WorksInEditor(page));
            bool noCheck = false;
            if (ImGui.MenuItem(ConsolePages.Label(page), "", ref noCheck, enabled))
                AfcConsole.Open(page);
            if (AfcConsole.IsShown && AfcConsole.Page == page)
            {
                float2 max = ImGui.GetItemRectMax();
                float2 min = ImGui.GetItemRectMin();
                float r = ImGui.GetFontSize() * 0.18f;
                ImGui.GetWindowDrawList().AddCircleFilled(new float2(max.X - ImGui.GetFontSize() * 0.6f, (min.Y + max.Y) * 0.5f), r, ConsoleStyle.ApplyAlpha(ConsoleStyle.TextPrimaryU32));
            }
        }

        ImGui.Separator();
        // The switch is here as well as on TUNE > PREFERENCES, so the stock planner can be had back without the console.
        bool route = PlannerRoute.Active;
        if (ImGui.MenuItem($"{key} OPENS AFC", "", ref route, AfcConsole.Enabled))
            PlannerRoute.SetRouting(route);

        if (ImGui.MenuItem("RESET WINDOW POSITION"))
            AfcConsole.ResetWindowPlace();

        Vehicle? subject = AfcConsole.Frame.Subject;
        if (subject != null && AfcConsole.Frame.GuidanceOn && !StopResolver.NothingToRelease(AfcConsole.Frame.Activity))
        {
            ImGui.Separator();
            if (ImGui.MenuItem("RELEASE ALL"))
                GuidanceWindow.ReleaseGuidance(subject);
            ImGui.PushTextWrapPos(ImGui.GetFontSize() * 24f);
            ImGui.TextDisabled(ConsoleUi.EscapeFormat(StopResolver.ReleaseOutcomeFor(AfcConsole.Frame.Activity)));
            ImGui.PopTextWrapPos();
        }

        int count = HeadDrawer.OtherCraftCount + (AfcConsole.Frame.SubjectBusy ? 1 : 0);
        if (count > 0)
        {
            ImGui.Separator();
            ImGui.TextDisabled(count == 1 ? "AFC: 1 VEHICLE" : $"AFC: {count} VEHICLES");
        }
    }
}
