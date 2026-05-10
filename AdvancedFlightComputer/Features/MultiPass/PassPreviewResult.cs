namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>Multi-pass planner output. <see cref="Failed"/> is true
/// when planning aborted early; <see cref="Passes"/> still holds the
/// partial result. <see cref="FailureReason"/> is for logs only.</summary>
internal readonly record struct PassPreviewResult(
    PassPreview[] Passes,
    bool Failed,
    string? FailureReason);
