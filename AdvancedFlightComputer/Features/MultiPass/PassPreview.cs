using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>One scheduled pass: when to fire, what dV, and the
/// resulting flight plan for preview rendering.</summary>
internal readonly record struct PassPreview(
    SimTime BurnTime,
    double3 DvVlf,
    double EstimatedBurnTimeSec,
    FlightPlan FlightPlan);
