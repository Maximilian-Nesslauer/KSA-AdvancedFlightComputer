namespace AdvancedFlightComputer.Features.MultiPass;

// Pass and FailureReason are mutually exclusive for results built by these factories.
internal readonly record struct PassPlanResult(
    PassPreview? Pass,
    string? FailureReason)
{
    public static PassPlanResult Success(PassPreview pass) => new(pass, null);
    public static PassPlanResult Failure(string reason) => new(null, reason);
}
