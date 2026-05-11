namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Single-pass planner output - the "what should I commit next" result
/// of <see cref="IManeuverIntent.RecomputePass"/>.
///
/// <see cref="Pass"/> is non-null on success; <see cref="FailureReason"/>
/// is non-null on failure. They are mutually exclusive: callers can
/// pattern-match either field. The reason string is for logging only,
/// never shown verbatim in the UI.
/// </summary>
internal readonly record struct PassPlanResult(
    PassPreview? Pass,
    string? FailureReason)
{
    public static PassPlanResult Success(PassPreview pass) => new(pass, null);
    public static PassPlanResult Failure(string reason) => new(null, reason);
}
