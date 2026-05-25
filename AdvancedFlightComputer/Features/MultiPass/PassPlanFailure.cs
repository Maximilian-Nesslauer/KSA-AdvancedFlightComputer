namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Classifies why a planner refused to produce a multi-pass schedule for
/// the requested N. The UI's auto-clamp banner uses this to give
/// context-appropriate advice (different fixes apply to time-budget
/// shortages vs SOI ceiling vs cumulative-dV-exceeds-escape vs rounding
/// artifacts at a specific N).
///
/// Default <see cref="None"/> means the result didn't fail, or the
/// planner couldn't classify the cause; the banner falls back to
/// generic advice.
/// </summary>
internal enum PassPlanFailure
{
    None,

    /// <summary>Vehicle's parking-orbit position at T_final is too soon;
    /// the K-schedule would need to fire pass 0 before now+margin. User
    /// fix: pick a later porkchop entry.</summary>
    TimeBudget,

    /// <summary>Even with the per-prior escape-velocity cap, cumulative
    /// dV pushes a prior orbit past the parent SOI envelope. User fix:
    /// reduce passes; no porkchop tweak helps.</summary>
    SoiCeiling,

    /// <summary>Per-pass dV would push v_p past parabolic before reaching
    /// the final pass. With auto-cap enabled this should only fire when
    /// the split is genuinely infeasible (priors degenerate after cap).
    /// User fix: reduce passes.</summary>
    ParabolicVp,

    /// <summary>Integer-sum rounding pulled the last prior K below the
    /// previous K. Specific to one N value; usually N-1 works. User fix:
    /// reduce passes by one.</summary>
    NonMonotonicK,

    /// <summary>The last prior K dropped below the zero-burn floor
    /// (KFloor = 1.01) after rounding. Means the chain is degenerate at
    /// this N. User fix: reduce passes.</summary>
    KFloor,

    /// <summary>Splitter allocation for one or more passes returned zero
    /// dV capacity because the vehicle is fuel-short. User fix: add fuel
    /// or pick a lower-energy transfer.</summary>
    FuelShort,

    /// <summary>Generic / unclassified planner failure. Banner uses the
    /// fallback advice.</summary>
    Other,
}
