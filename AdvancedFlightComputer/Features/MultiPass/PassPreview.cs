using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

internal readonly record struct PassPreview(
    UniverseTime BurnTime,
    double3 DvVlf,
    double EstimatedBurnTimeSec,
    FlightPlan FlightPlan);
