using AdvancedFlightComputer.Features.Guidance;
using Brutal.ImGuiApi;
using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// The one AFC menu, in the game's menu bar. It draws from the postfix on
/// Program.DrawProgramMenusHook that the guidance diagnostics block owns, so a failed guidance
/// driver still leaves a menu with the guidance off switch. While AFC flies the controlled craft the
/// title is marked and the first item is the stop of what runs, so the stop is one click away also
/// with the console closed. Features add their items through <see cref="Register"/> instead of
/// patching the menu bar again.
/// </summary>
internal static class AfcMenu
{
    private sealed record Section(int Order, string Owner, Action Draw);

    private static readonly List<Section> _sections = new();

    /// <summary>The stop of what AFC flies on the controlled craft, set by the console each frame: its menu label, its outcome and its action, or null.</summary>
    internal static Func<(string Label, string Outcome, Action Act, bool AfcFlies)?>? StopItem;

    internal static void Register(int order, string owner, Action draw)
    {
        _sections.RemoveAll(s => s.Owner == owner);
        _sections.Add(new Section(order, owner, draw));
        _sections.Sort((a, b) => a.Order.CompareTo(b.Order));
    }

    internal static void Reset()
    {
        _sections.Clear();
        StopItem = null;
    }

    // The menu holds the guidance off switch, so a fault is logged once per kind and the menu keeps drawing. BeginMenu and EndMenu stay paired through the finally.
    internal static void Draw()
    {
        try
        {
            (string Label, string Outcome, Action Act, bool AfcFlies)? stop = StopItem?.Invoke();
            // The id after ### stays the same, so the open menu survives the change of its title. A stop of stock AUTO is offered, but AFC does not fly then.
            if (!ImGui.BeginMenu(stop is { AfcFlies: true } ? "AFC (FLYING)###AfcMenu" : "AFC###AfcMenu"))
                return;
            try
            {
                DrawItems();
            }
            finally
            {
                ImGui.EndMenu();
            }
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"afc-menu:{ex.GetType().Name}", $"[AFC] AFC menu failed: {ex}");
        }
    }

    private static void DrawItems()
    {
        if (StopItem?.Invoke() is (string label, string outcome, Action act, _))
        {
            ImGui.PushStyleColor(ImGuiCol.Text, in ConsoleStyle.Danger);
            bool stop = ImGui.MenuItem(label);
            ImGui.PopStyleColor();
            if (stop)
                act();
            if (outcome.Length > 0)
                ImGui.TextDisabled(ConsoleUi.EscapeFormat(outcome));
            ImGui.Spacing();
            ImGui.Separator();
            ImGui.Spacing();
        }
        foreach (Section section in _sections.ToArray())
        {
            try
            {
                section.Draw();
            }
            catch (Exception ex)
            {
                LogHelper.WarnOnce("afc-menu:" + section.Owner + ":" + ex.GetType().Name,
                    $"[AFC] AFC menu section '{section.Owner}' failed: {ex}");
            }
            ImGui.Separator();
        }
        DrawGuidanceSwitch();

        // A release error stays readable while the console is closed.
        string failure = GuidanceWindow.ReleaseFailure(Program.ControlledVehicle);
        if (failure.Length > 0)
        {
            ImGui.Separator();
            ImGui.PushTextWrapPos(ImGui.GetFontSize() * 24f);
            ImGui.Text(failure);
            ImGui.PopTextWrapPos();
        }
    }

    private static void DrawGuidanceSwitch()
    {
        if (!SharedVehicleHooks.GuidanceEnabled)
        {
            ImGui.PushTextWrapPos(ImGui.GetFontSize() * 24f);
            ImGui.Text(GuidanceFeature.UnavailableReason);
            ImGui.PopTextWrapPos();
            return;
        }
        bool active = GuidanceWindow.ModActive;
        if (ImGui.MenuItem("GUIDANCE ENABLED", "", ref active, true))
            GuidanceWindow.SetModActive(active);
        ImGui.TextDisabled("Off: AFC lets go of every craft it flies.");
    }
}
