using System;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// In-flight state for one vehicle's multi-pass plan. One per vehicle,
/// owned by <see cref="MultiPassRegistry"/>.
///
/// Two state buckets, separated visually below:
///   * Persisted: written to / read from multipass.toml. Mutating any
///     of these mid-execution must keep the on-disk format in mind.
///   * Transient: in-memory only. Reset to default after a save load
///     and rebuilt by the postfix.
/// </summary>
internal sealed class MultiPassExecution
{
    // Persisted with :R round-trip format, so deserialized values are
    // bit-exact. Small tolerance absorbs any FP normalisation drift
    // through SimTime constructors / dV transforms, while staying
    // tight enough to disambiguate adjacent burns.
    private const double BurnTimeMatchToleranceSec = 0.05;
    private const double BurnDvMatchToleranceMs = 0.1;

    #region Persisted state (serialised to multipass.toml)

    /// <summary>The KSA save-game id this execution belongs to. Empty
    /// while running in an unsaved session; <see cref="MultiPassRegistry.RekeyTo"/>
    /// moves the session's entries whenever a save is written under a
    /// different id (first save, Save-As, overwrite of another save).</summary>
    public required string SaveId { get; internal set; }

    public required string VehicleId { get; init; }
    public required IManeuverIntent Intent { get; init; }
    public required SplitMode Mode { get; init; }
    public required int PassCountTotal { get; init; }

    public int PassIndex { get; set; }

    /// <summary>Persisted burn time + dV magnitude let
    /// <see cref="TryResolveCurrentBurn"/> reattach to the same burn
    /// after a save load (when the live reference is gone).</summary>
    public double? CurrentBurnTimeSec { get; set; }
    public double? CurrentBurnDvMagnitudeMs { get; set; }

    #endregion

    #region Transient state (in-memory only)

    /// <summary>Live reference to the burn currently in the BurnPlan;
    /// null between passes and immediately after load.</summary>
    public Burn? CurrentBurn { get; set; }

    /// <summary>Suppresses external-delete detection during the
    /// deferred-apply gap of InputEvents.BurnUpdateBuffer. We queue
    /// the Add on tick N; it materializes on tick N+1. Without this
    /// flag the postfix would briefly see "burn missing from plan"
    /// on tick N and cancel by mistake.</summary>
    public bool AwaitingMaterialization { get; set; }

    /// <summary>Tick budget for AwaitingMaterialization; the postfix
    /// cancels rather than wait forever if the queued Add is dropped.</summary>
    public int AwaitingMaterializationTicks { get; set; }

    /// <summary>Tick budget for retrying a failing
    /// <c>intent.RecomputePass</c>; the postfix cancels with a warning
    /// after the budget so persistent failures surface instead of
    /// looping silently.</summary>
    public int ConsecutiveScheduleFailures { get; set; }

    /// <summary>True between passes: the prior pass completed in Auto
    /// and another pass is queued. The postfix re-engages BurnMode=Auto
    /// once the queued burn materialises in the BurnPlan so the user
    /// does not have to toggle Auto between every pass.</summary>
    public bool ReengageAutoOnNextBurn { get; set; }

    /// <summary>True once <see cref="FlightComputer.BurnMode"/> was
    /// observed as <c>Auto</c> while the current pass burn was loaded.
    /// PassCompletionPatch sets this each tick the engine is firing
    /// under Auto control; DetectImplicitCompletion uses it to
    /// distinguish "burn fired and was cleaned up naturally" (e.g.,
    /// AutoRemoveFinishedBurns mod racing with our Auto->Manual
    /// detection) from "burn deleted externally before firing".
    /// Cleared on each new pass via AssignCurrentBurn.</summary>
    public bool BurnAutoEngagedThisPass { get; set; }

    /// <summary>One-shot guard for the stalled-pass hint (engine stopped
    /// mid-pass with dV remaining). Set when the hint fires; cleared on each
    /// new pass and whenever Auto is observed re-engaged, so a re-engage-then-
    /// stall-again cycle re-alerts the player.</summary>
    public bool StallHintShown { get; set; }

    #endregion

    public void AssignCurrentBurn(Burn burn)
    {
        CurrentBurn = burn;
        CurrentBurnTimeSec = burn.Time.Seconds();
        CurrentBurnDvMagnitudeMs = burn.DeltaVVlf.Length();
        AwaitingMaterialization = true;
        AwaitingMaterializationTicks = 0;
        BurnAutoEngagedThisPass = false;
        StallHintShown = false;
    }

    public void ClearCurrentBurn()
    {
        CurrentBurn = null;
        CurrentBurnTimeSec = null;
        CurrentBurnDvMagnitudeMs = null;
        AwaitingMaterialization = false;
        AwaitingMaterializationTicks = 0;
        BurnAutoEngagedThisPass = false;
        StallHintShown = false;
    }

    /// <summary>
    /// Records a post-load reattachment. CurrentBurnTimeSec /
    /// CurrentBurnDvMagnitudeMs are already populated from the loaded
    /// TOML, and AwaitingMaterialization is irrelevant because the
    /// burn is already in the BurnPlan.
    /// </summary>
    public void ReattachAfterLoad(Burn burn)
    {
        CurrentBurn = burn;
    }

    /// <summary>
    /// Returns the burn that represents this execution's currently
    /// queued pass: live <see cref="CurrentBurn"/> when available,
    /// otherwise a time+dV match against <paramref name="plan"/> for
    /// the post-load case.
    /// </summary>
    public Burn? TryResolveCurrentBurn(BurnPlan plan)
    {
        if (CurrentBurn != null && plan.TryGetBurn(CurrentBurn))
            return CurrentBurn;
        if (!CurrentBurnTimeSec.HasValue) return null;

        int count = plan.BurnCount;
        for (int i = 0; i < count; i++)
        {
            if (!plan.TryGetBurn(i, out Burn? b) || b == null) continue;
            if (Math.Abs(b.Time.Seconds() - CurrentBurnTimeSec.Value) >= BurnTimeMatchToleranceSec)
                continue;
            if (CurrentBurnDvMagnitudeMs.HasValue
                && Math.Abs(b.DeltaVVlf.Length() - CurrentBurnDvMagnitudeMs.Value) >= BurnDvMatchToleranceMs)
                continue;
            return b;
        }
        return null;
    }
}
