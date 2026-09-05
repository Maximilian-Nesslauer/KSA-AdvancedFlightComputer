namespace AdvancedFlightComputer.Features.MultiPass;

// EstimatedBurnTimeSec is zero when no usable engine data is available.
internal readonly record struct PassAllocation(
    double DvCapacityMs,
    double EstimatedBurnTimeSec);
