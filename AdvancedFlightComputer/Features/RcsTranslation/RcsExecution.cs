using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Per-vehicle RCS translation state, owned by <see cref="RcsExecRegistry"/>.
/// Persisted fields round-trip through rcs-exec.toml; everything else is
/// rebuilt by the driver after a save load.
/// </summary>
internal sealed class RcsExecution
{
    #region Persisted state (serialised to rcs-exec.toml)

    /// <summary>Empty while running in an unsaved session;
    /// <see cref="RcsExecRegistry.RekeyTo"/> moves the session's entries
    /// whenever a save is written under a different id, same policy as the
    /// multi-pass registry.</summary>
    public required string SaveId { get; set; }

    public required string VehicleId { get; init; }

    public List<RcsBurnOptions> Options { get; } = new();

    /// <summary>Identity of the burn an active execution is running, null
    /// when no execution is in flight. Reattached after load by time + dV.</summary>
    public double? ActiveBurnTimeSec { get; set; }
    public double? ActiveBurnDvMs { get; set; }

    /// <summary>Resolved at activation so a save/load mid-burn does not
    /// re-decide the strategy with half-burned geometry.</summary>
    public RcsAttitudeStrategy ResolvedStrategy { get; set; } = RcsAttitudeStrategy.Hold;

    /// <summary>Group index (+X,-X,+Y,-Y,+Z,-Z) the Align strategy points at
    /// the burn vector; -1 for Hold.</summary>
    public int ResolvedAxis { get; set; } = -1;

    public RcsAllocator ResolvedAllocator { get; set; } = RcsAllocator.Groups;

    /// <summary>True once the Align tracker has been commanded. Deliberately
    /// deferred until the ignition lead window opens; before that the coast
    /// keeps whatever attitude the user had. Persisted because the game
    /// round-trips the attitude tracker through its save: after a mid-align
    /// load the restored tracker is still driving the burn attitude, and
    /// the handback on Cancel/Complete must know that.</summary>
    public bool AlignCommanded { get; set; }

    /// <summary>The executor forced FlightComputer.RCSMode to Enabled for
    /// this burn because the pilot had the stock RCS toggle (default key R)
    /// off, either at activation or via a mid-burn press. Restored to
    /// Disabled at Complete/Cancel. Persisted because the game round-trips
    /// RCSMode through its save, so a mid-burn load followed by completion
    /// must still hand the toggle back to off.</summary>
    public bool ForcedRcsOn { get; set; }

    #endregion

    #region Transient state

    public Burn? ActiveBurn;

    /// <summary>Set once the driver has re-resolved ActiveBurn after a load.</summary>
    public bool ReconciledAfterLoad;

    public RcsCapabilitySnapshot Capability;
    public double CapabilityProbedAtSec = double.NegativeInfinity;

    public RcsEstimates Estimates;
    public double EstimatesComputedAtSec = double.NegativeInfinity;

    /// <summary>No-progress watchdog baseline: the smallest to-go seen while
    /// firing was eligible. Infinity until the first eligible tick.</summary>
    public float WatchdogTogoMs = float.PositiveInfinity;

    /// <summary>Accumulated firing-eligible sim seconds without a to-go
    /// reduction. Accumulated rather than a rebased timestamp, so ticks that
    /// are merely ineligible (slewing, pre-ignition) pause the clock instead
    /// of resetting it; only actual progress resets it.</summary>
    public double NoProgressAccumSec;

    /// <summary>Accumulated slewing sim seconds since the last delivered
    /// progress; feeds the cannot-reach-attitude bound. Accumulated so an
    /// attitude chattering across the align gate cannot clear the clock with
    /// a single in-gate tick.</summary>
    public double SlewAccumSec;

    /// <summary>Sim time of the previous active driver tick, the delta source
    /// for the two accumulators above. NaN until the first active tick.</summary>
    public double LastTickSimSec = double.NaN;

    /// <summary>One-shot guard for the ignition-crossing debug log.</summary>
    public bool FiringLogged;

    #region Fuel telemetry (accumulated by the driver, reported at Complete/Cancel)

    /// <summary>Vehicle mass when the execution engaged (or re-baselined
    /// after a save load), kg. Zero means no baseline exists and the fuel
    /// line is skipped.</summary>
    public double StartMassKg;

    /// <summary>Mass at the previous driver tick; feeds the per-tick burn
    /// deltas below.</summary>
    public double LastTickMassKg;

    /// <summary>Total propellant spent, accumulated from per-tick mass
    /// LOSSES only. Not start-minus-now: a mid-burn mass gain (the stock
    /// refill action, resource transfer, docking) would turn that difference
    /// negative and invert the whole fuel report, while a clamped per-tick
    /// sum just ignores the gain and keeps counting real drain.</summary>
    public double BurnedPropellantKg;

    /// <summary>Propellant spent while the Align slew held firing back, kg
    /// (mass delta over slewing ticks, which is all attitude by construction).</summary>
    public double SlewPropellantKg;

    /// <summary>Propellant spent before the firing window opened while not
    /// slewing, kg: the attitude/rate hold cost of the pre-ignition coast.</summary>
    public double CoastPropellantKg;

    /// <summary>Model-attributed translation propellant, kg: delivered
    /// delta-V (game accounting) times the active allocator's cost per
    /// newton-second. The gap to the total is attitude hold plus pulse
    /// quantization plus model error.</summary>
    public double TranslationPropellantKg;

    /// <summary>DeltaVAccumCci at the previous driver tick, so each tick
    /// attributes only its own delivered delta-V.</summary>
    public float3 LastAccumCci;

    /// <summary>DeltaVAccumCci at the baseline. Nonzero when a mid-burn
    /// save load re-baselined the telemetry: the effective-ve number must
    /// pair the post-load delta-V with the post-load propellant, not the
    /// whole burn's delta-V.</summary>
    public float3 StartAccumCci;

    /// <summary>Sim time of the baseline; feeds the fuel line's elapsed.</summary>
    public double EngagedAtSec;

    /// <summary>Breakdown of the last finished execution (completed or
    /// cancelled); survives ClearActive so consumers can read it after
    /// the execution ends.</summary>
    public RcsFuelSummary LastFuel;

    /// <summary>(Re)starts the fuel accumulators from the current state.
    /// Called at activation and after a save-load reattach, so the fuel
    /// line always covers exactly the window this process observed.</summary>
    public void BaselineFuel(FlightComputer fc, double engagedAtSec)
    {
        StartMassKg = fc.TotalMassPropsBody.Mass;
        LastTickMassKg = StartMassKg;
        BurnedPropellantKg = 0.0;
        SlewPropellantKg = 0.0;
        CoastPropellantKg = 0.0;
        TranslationPropellantKg = 0.0;
        StartAccumCci = fc.Burn?.DeltaVAccumCci ?? default;
        LastAccumCci = StartAccumCci;
        EngagedAtSec = engagedAtSec;
    }

    #endregion

    #region LP allocator state (transient, ResolvedAllocator == Lp only)

    public RcsWrenchTable? Wrench;
    public double WrenchBuiltAtSec = double.NegativeInfinity;

    /// <summary>Seconds of firing per newton-second of net impulse along
    /// <see cref="LpDirCtrl"/>, index-aligned with the wrench table (and
    /// with VehicleConfig.Thrusters). Null while no valid solution exists.</summary>
    public float[]? LpSecondsPerImpulse;
    public float3 LpDirCtrl;
    public float LpImpulseCapNs;
    public double LpSolvedAtSec = double.NegativeInfinity;

    /// <summary>Propellant per newton-second of net impulse for the current
    /// solution, thruster columns only; feeds the LP-honest sufficiency
    /// numbers and the fuel telemetry's translation attribution.</summary>
    public double LpCostPerImpulse;

    /// <summary>Decision price paid to the torque-slack columns, kg per
    /// newton-second. The attitude hold spends this, not the pattern, so
    /// it joins the sufficiency numbers but never the translation
    /// attribution (the hold's real cost lands in the attitude bucket).</summary>
    public double LpSlackCostPerImpulse;

    /// <summary>The pattern's own net torque per newton-second of impulse
    /// (N m s per N s), i.e. what the attitude hold must counter.</summary>
    public float3 LpResidualTorquePerNs;

    /// <summary>Signature of the last logged firing pattern (support
    /// indices), so the solve log fires on pattern changes instead of
    /// every cadence re-solve.</summary>
    public int LpLoggedSupportSignature;

    /// <summary>One-shot guard for the LP-infeasible fallback warning.</summary>
    public bool LpFallbackLogged;

    #endregion

    public bool IsActive => ActiveBurnTimeSec.HasValue;

    #endregion

    public RcsBurnOptions? FindOptions(double timeSec, double dvMs)
    {
        foreach (RcsBurnOptions o in Options)
        {
            if (o.Matches(timeSec, dvMs))
                return o;
        }
        return null;
    }

    public RcsBurnOptions GetOrCreateOptions(double timeSec, double dvMs)
    {
        RcsBurnOptions? found = FindOptions(timeSec, dvMs);
        if (found != null)
        {
            // The user may have nudged the burn since arming; keep the key in
            // sync so the metadata follows the burn instead of orphaning.
            found.BurnTimeSec = timeSec;
            found.BurnDvMs = dvMs;
            return found;
        }
        RcsBurnOptions fresh = new() { BurnTimeSec = timeSec, BurnDvMs = dvMs };
        Options.Add(fresh);
        return fresh;
    }

    public void ClearActive()
    {
        ActiveBurn = null;
        ActiveBurnTimeSec = null;
        ActiveBurnDvMs = null;
        ResolvedAxis = -1;
        ResolvedStrategy = RcsAttitudeStrategy.Hold;
        WatchdogTogoMs = float.PositiveInfinity;
        NoProgressAccumSec = 0.0;
        SlewAccumSec = 0.0;
        LastTickSimSec = double.NaN;
        FiringLogged = false;
        AlignCommanded = false;
        ForcedRcsOn = false;
        StartMassKg = 0.0;
        LastTickMassKg = 0.0;
        BurnedPropellantKg = 0.0;
        SlewPropellantKg = 0.0;
        CoastPropellantKg = 0.0;
        TranslationPropellantKg = 0.0;
        LastAccumCci = default;
        StartAccumCci = default;
        EngagedAtSec = 0.0;
        ResolvedAllocator = RcsAllocator.Groups;
        Wrench = null;
        WrenchBuiltAtSec = double.NegativeInfinity;
        LpSecondsPerImpulse = null;
        LpDirCtrl = default;
        LpImpulseCapNs = 0f;
        LpSolvedAtSec = double.NegativeInfinity;
        LpCostPerImpulse = 0.0;
        LpSlackCostPerImpulse = 0.0;
        LpResidualTorquePerNs = default;
        LpLoggedSupportSignature = 0;
        LpFallbackLogged = false;
    }

    /// <summary>Drops per-burn options whose burn no longer exists in the
    /// plan, so stale entries cannot attach to a future unrelated burn at a
    /// coincidentally similar time.</summary>
    public void PruneOrphanedOptions(BurnPlan plan)
    {
        for (int i = Options.Count - 1; i >= 0; i--)
        {
            if (FindBurn(plan, Options[i].BurnTimeSec, Options[i].BurnDvMs) == null)
                Options.RemoveAt(i);
        }
    }

    public static Burn? FindBurn(BurnPlan plan, double timeSec, double dvMs)
    {
        int count = plan.BurnCount;
        for (int i = 0; i < count; i++)
        {
            if (!plan.TryGetBurn(i, out Burn? b) || b == null)
                continue;
            if (Math.Abs(b.Time.Seconds() - timeSec) >= RcsBurnOptions.TimeMatchToleranceSec)
                continue;
            if (Math.Abs(b.DeltaVVlf.Length() - dvMs) >= RcsBurnOptions.DvMatchToleranceMs)
                continue;
            return b;
        }
        return null;
    }
}

/// <summary>Fuel breakdown of one finished execution. Total is the sum of
/// per-tick mass losses (mass gains like a mid-burn refill are ignored, not
/// subtracted). Translation is model-attributed (delivered delta-V times the
/// allocator's cost model), so Attitude, the residual, also absorbs pulse
/// quantization and model error; Slew is the measured mass delta while the
/// Align slew held firing back, Coast the measured mass delta before the
/// firing window opened (attitude and rate hold through the wait). A
/// NEGATIVE Attitude therefore means the model over-attributed translation
/// relative to the measured drain; the known case is a partially present
/// reactant mix (dev-save territory), where ResourceManager.MassChange
/// withdraws only the available reactants' share while the nozzle keeps
/// firing at full thrust.</summary>
internal struct RcsFuelSummary
{
    public bool Valid;
    public double TotalKg;
    public double TranslationKg;
    public double SlewKg;
    public double CoastKg;
    public double AttitudeKg;

    /// <summary>Overall economy: start mass times accumulated delta-V over
    /// total propellant, m/s. A first-order proxy, not the rocket-equation
    /// Ve. Zero when no matching burn was loaded.</summary>
    public double EffectiveVeMs;

    /// <summary>Angle between the accumulated and the target delta-V
    /// vectors, degrees: did the burn push in the right direction.</summary>
    public double DvAngleDeg;

    public double ElapsedSec;
}

/// <summary>Propellant/duration estimates for the two attitude strategies,
/// recomputed periodically for the UI and the Auto decision.</summary>
internal struct RcsEstimates
{
    public bool Valid;

    public double HoldPropellantKg;
    public double HoldDurationSec;
    public bool HoldFeasible;

    public double AlignPropellantKg;
    public double AlignSlewPropellantKg;
    public double AlignDurationSec;
    public double AlignSlewDurationSec;
    public int AlignAxis;
    public bool AlignFeasible;

    public readonly double AlignTotalPropellantKg => AlignPropellantKg + AlignSlewPropellantKg;

    /// <summary>Propellant the given attitude selection would consume; Auto
    /// takes the cheaper feasible strategy, zero when nothing is feasible.
    /// Shared by the sufficiency warning in the burn editor and the
    /// activation-time alert so the two cannot drift.</summary>
    public readonly double RequiredPropellantKg(RcsAttitudeStrategy attitude) => attitude switch
    {
        RcsAttitudeStrategy.Hold => HoldFeasible ? HoldPropellantKg : 0.0,
        RcsAttitudeStrategy.Align => AlignFeasible ? AlignTotalPropellantKg : 0.0,
        _ => (HoldFeasible, AlignFeasible) switch
        {
            (true, true) => Math.Min(HoldPropellantKg, AlignTotalPropellantKg),
            (true, false) => HoldPropellantKg,
            (false, true) => AlignTotalPropellantKg,
            _ => 0.0,
        },
    };
}
