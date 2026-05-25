namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>Per-pass dV budget plus estimated firing time (0 when
/// the vehicle has no usable stage data).</summary>
internal readonly record struct PassAllocation(
    double DvCapacityMs,
    double EstimatedBurnTimeSec);
