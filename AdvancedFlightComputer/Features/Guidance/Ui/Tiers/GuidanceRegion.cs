using System;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// A tuning set has no head, because the tab over it names it. A label that starts with ## has no head in any scope, for a view with one region.
/// </summary>
internal static class GuidanceRegion
{
    public static readonly UiTier[] Player = { UiTier.Player };
    public static readonly UiTier[] PlayerExpert = { UiTier.Player, UiTier.Expert };
    public static readonly UiTier[] ExpertTuningDiagnostics = { UiTier.Expert, UiTier.Tuning, UiTier.Diagnostics };
    public static readonly UiTier[] PlayerTuningDiagnostics = { UiTier.Player, UiTier.Tuning, UiTier.Diagnostics };
    public static readonly UiTier[] PlayerDiagnostics = { UiTier.Player, UiTier.Diagnostics };
    public static readonly UiTier[] Expert = { UiTier.Expert };
    public static readonly UiTier[] ExpertTuning = { UiTier.Expert, UiTier.Tuning };
    public static readonly UiTier[] Tuning = { UiTier.Tuning };
    public static readonly UiTier[] Diagnostics = { UiTier.Diagnostics };

    private static int _open;

    /// <summary>Opens the region in the scope that shows it. <paramref name="noHead"/> leaves out the head in the Player scope, for the live rows of a page that flies, where the rows name themselves.</summary>
    public static bool Begin(string label, UiTier[] tiers, bool noHead = false)
    {
        UiTier scope = TierScope.Current;
        if (Array.IndexOf(tiers, scope) < 0)
            return false;

        int hidden = label.IndexOf("##", StringComparison.Ordinal);
        string head = hidden >= 0 ? label.Substring(0, hidden) : label;
        bool bare = head.Length == 0 || scope == UiTier.Tuning || (noHead && scope == UiTier.Player);
        Core.ConsoleUi.BeginSection(label + "/" + scope, bare ? null : head);
        ImGui.PushID(label);
        _open++;
        return true;
    }

    public static void End()
    {
        Core.ConsoleUi.EndSection();
        if (_open == 0)
            return;
        _open--;
        ImGui.PopID();
    }

    /// <summary>How many regions are open now, so a scope can close only the ones it opened.</summary>
    public static int OpenCount => _open;

    /// <summary>Closes every region a faulted section left open, so the body that hosts it ends on the window it began. Call it from the finally of that body, before its own end.</summary>
    public static void EndOpen() => EndDownTo(0);

    /// <summary>Closes the open regions until <paramref name="depth"/> are left.</summary>
    public static void EndDownTo(int depth)
    {
        while (_open > Math.Max(0, depth))
            End();
    }

    public static void Reset()
    {
        _open = 0;
        Core.ConsoleUi.EndSection();
    }
}
