using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Reads the per-rocket states of a part tree.
/// A <see cref="Rocket"/> owns the throttle, the burn timers and the propellant flag of its cores, and <c>PartTree</c> has no hot-path list for it, so the list comes from <c>PartTree.States</c> the way <c>Part.DrawPartInfo</c> reads it.
/// </summary>
internal static class RocketStates
{
    /// <summary>
    /// The rocket states of <paramref name="tree"/>, indexed by <c>Rocket.StatesIdx</c>.
    /// The span is empty when the tree has no rockets, because the game then creates no list for them.
    /// </summary>
    internal static ReadOnlySpan<RocketState> Of(PartTree tree)
    {
        if (tree?.States == null
            || !ModuleStateful<Rocket, RocketState, EmptyStruct, EmptyStruct>.TryGetFrom(tree.States, out var list)
            || list == null)
            return ReadOnlySpan<RocketState>.Empty;
        return list.States;
    }
}
