namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>EqualBurnTime equalizes the per-pass burn arc (Oberth-
/// optimal for raising apoapsis); EqualDv splits dV evenly (simpler,
/// slightly less optimal because later passes burn shorter arcs).</summary>
internal enum SplitMode
{
    EqualBurnTime,
    EqualDv,
}
