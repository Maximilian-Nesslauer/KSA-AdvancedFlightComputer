namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>Multi-pass planner output. <see cref="Failed"/> is true
/// when planning aborted early; <see cref="Passes"/> still holds the
/// partial result. <see cref="FailureReason"/> is for logs only;
/// <see cref="FailureKind"/> drives UI banner advice and is the stable
/// machine-readable failure classifier (don't substring-match
/// FailureReason).
///
/// <see cref="Advisory"/> is a soft warning that does NOT set Failed: the
/// plan is usable, but the user should know something looked off (e.g. the
/// final-pass flight plan punches through the parent body, or the shifted-
/// Lambert candidate scan only found impacting transfers and we surfaced
/// the cheapest dirty one anyway). Stock's porkchop reject filter would
/// have hidden these entries; we surface them and let the user decide
/// whether to commit.</summary>
internal readonly record struct PassPreviewResult(
    PassPreview[] Passes,
    bool Failed,
    string? FailureReason,
    PassPlanFailure FailureKind = PassPlanFailure.None,
    string? Advisory = null);
