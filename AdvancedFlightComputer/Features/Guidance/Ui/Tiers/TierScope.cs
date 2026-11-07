using System;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// The tier a guidance section is drawn for. A page draws one section function once per tier it
/// Tuning or Diagnostics scope on the tuning page. Each row draws only in the scope of its own tier,
/// and everything a section draws outside its rows, along with the work a section does in the
/// draw, belongs to the Player pass, so it happens once per frame.
/// </summary>
internal static class TierScope
{
    public static UiTier Current { get; private set; } = UiTier.Player;

    public static bool IsPrimaryPass => Current == UiTier.Player;

    public static bool Show(UiTier tier) => tier == Current;

    /// <summary>
    /// Switches the scope and pushes an ImGui id with the tier name, so one section drawn in two
    /// scopes in one window keeps its region headers and its ## ids apart.
    /// </summary>
    public static Scope Push(UiTier tier)
    {
        var scope = new Scope(Current, GuidanceRegion.OpenCount);
        ImGui.PushID(tier.ToString());
        Current = tier;
        return scope;
    }

    public static void Reset() => Current = UiTier.Player;

    public readonly struct Scope : IDisposable
    {
        private readonly UiTier _previous;
        private readonly int _regionDepth;

        public Scope(UiTier previous, int regionDepth)
        {
            _previous = previous;
            _regionDepth = regionDepth;
        }

        public void Dispose()
        {
            GuidanceRegion.EndDownTo(_regionDepth);
            Current = _previous;
            ImGui.PopID();
        }
    }
}
