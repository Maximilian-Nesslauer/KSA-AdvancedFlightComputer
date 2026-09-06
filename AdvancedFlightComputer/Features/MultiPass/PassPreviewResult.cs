namespace AdvancedFlightComputer.Features.MultiPass;

// Failed results can contain usable prefix passes. Advisory is a warning, not a failure.
// FailureKind selects the UI advice. FailureReason supplies diagnostic text.
internal readonly record struct PassPreviewResult(
    PassPreview[] Passes,
    bool Failed,
    string? FailureReason,
    PassPlanFailure FailureKind = PassPlanFailure.None,
    string? Advisory = null);
