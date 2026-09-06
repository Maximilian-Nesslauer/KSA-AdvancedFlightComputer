using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>RcsExecRegistry owns this state for one vehicle. The persisted fields survive a save and load. The driver rebuilds the transient fields after loading.</summary>
internal sealed class RcsExecution
{
    #region Persisted state (serialised to rcs-exec.toml)

    /// <summary>An unsaved session uses an empty SaveId. RcsExecRegistry.RekeyTo moves its entries when the save is written under another id.</summary>
    public required string SaveId { get; set; }

    public required string VehicleId { get; init; }

    public List<RcsBurnOptions> Options { get; } = new();

    /// <summary>A null value means no execution is active. After loading, the driver identifies the burn by its time and delta V.</summary>
    public double? ActiveBurnTimeSec { get; set; }
    public double? ActiveBurnDvMs { get; set; }

    /// <summary>Keep the strategy selected at activation so loading a save does not select another strategy from a partly completed burn.</summary>
    public RcsAttitudeStrategy ResolvedStrategy { get; set; } = RcsAttitudeStrategy.Hold;

    /// <summary>Align uses the group order of positive X, negative X, positive Y, negative Y, positive Z, and negative Z. Hold uses minus one.</summary>
    public int ResolvedAxis { get; set; } = -1;

    public RcsAllocator ResolvedAllocator { get; set; } = RcsAllocator.Groups;

    /// <summary>Persist this flag because the game saves the attitude tracker. Cancellation must release a tracker that was commanded before the save, while an execution still coasting must leave the user target alone.</summary>
    public bool AlignCommanded { get; set; }

    /// <summary>Persist whether the executor enabled RCS after the pilot disabled it. Completion and cancellation restore the disabled setting, including after a save and load.</summary>
    public bool ForcedRcsOn { get; set; }

    #endregion

    #region Transient state

    public Burn? ActiveBurn;

    /// <summary>Set once the driver has re resolved ActiveBurn after a load.</summary>
    public bool ReconciledAfterLoad;

    public RcsCapabilitySnapshot Capability;
    public double CapabilityProbedAtSec = double.NegativeInfinity;

    public RcsEstimates Estimates;
    public double EstimatesComputedAtSec = double.NegativeInfinity;

    /// <summary>The smallest remaining delta V observed while firing was eligible is the progress baseline. It stays infinite until the first eligible tick.</summary>
    public float WatchdogTogoMs = float.PositiveInfinity;

    /// <summary>Count simulation seconds only while firing is eligible. Coast and slew pause the timeout. Delivered progress resets it.</summary>
    public double NoProgressAccumSec;

    /// <summary>Count slew time since the last delivered progress across all attitude gate crossings. A single tick inside the gate must not clear the timeout.</summary>
    public double SlewAccumSec;

    /// <summary>The previous active tick supplies elapsed simulation time for both watchdogs. NaN identifies the first tick.</summary>
    public double LastTickSimSec = double.NaN;

    /// <summary>One shot guard for the ignition crossing debug log.</summary>
    public bool FiringLogged;


    #region Fuel telemetry (accumulated by the driver, reported at Complete/Cancel)

    /// <summary>Record mass in kg at activation or after loading. Zero means no baseline exists and no fuel summary can be produced.</summary>
    public double StartMassKg;

    /// <summary>Mass at the previous driver tick. Feeds the per tick burn deltas below.</summary>
    public double LastTickMassKg;

    /// <summary>Accumulate mass losses only. Refills, transfers, and docking can increase vehicle mass but must not subtract propellant already consumed.</summary>
    public double BurnedPropellantKg;

    /// <summary>Measure mass lost in kg while the Align gate prevents translation. This is attributed to the slew.</summary>
    public double SlewPropellantKg;

    /// <summary>Measure mass lost in kg before ignition while the vehicle is not slewing. This is attributed to coast attitude control.</summary>
    public double CoastPropellantKg;

    /// <summary>Estimate translation propellant in kg from delivered delta V and allocator cost. The remaining measured consumption includes attitude control, pulse rounding, and model error.</summary>
    public double TranslationPropellantKg;

    /// <summary>DeltaVAccumCci at the previous driver tick, so each tick attributes only its own delivered delta V.</summary>
    public float3 LastAccumCci;

    /// <summary>Record the accumulated delta V at the baseline so a loaded execution pairs only the observed delta V with the observed propellant.</summary>
    public float3 StartAccumCci;

    /// <summary>Sim time of the baseline. Feeds the fuel line's elapsed.</summary>
    public double EngagedAtSec;

    /// <summary>Keep the last fuel summary after ClearActive so consumers can read it when execution ends.</summary>
    public RcsFuelSummary LastFuel;

    /// <summary>Start fuel accounting at activation and restart it after loading so the summary covers only the observed interval.</summary>
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

    /// <summary>Store seconds of firing per N s of impulse along LpDirCtrl. Indices match the wrench table and VehicleConfig.Thrusters. Null means no valid solution is available.</summary>
    public float[]? LpSecondsPerImpulse;
    public float3 LpDirCtrl;
    public float LpImpulseCapNs;
    public double LpSolvedAtSec = double.NegativeInfinity;

    /// <summary>Measure the thruster pattern cost in kg per N s of impulse. Attitude control cost is accounted for separately.</summary>
    public double LpCostPerImpulse;

    /// <summary>Measure the estimated attitude control cost in kg per N s of impulse. Include it in the sufficiency warning but not in the translation fuel bucket.</summary>
    public double LpSlackCostPerImpulse;

    /// <summary>Record the angular impulse that attitude control must counter, measured in N m s per N s of translation impulse.</summary>
    public float3 LpResidualTorquePerNs;

    /// <summary>Log a pattern only when its thruster support changes so repeated solves do not flood the log.</summary>
    public int LpLoggedSupportSignature;

    /// <summary>One shot guard for the LP infeasible fallback warning.</summary>
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
            // Keep the key aligned with user edits so the options continue to identify the same burn.
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

    /// <summary>Remove options for missing burns so they cannot attach to a later burn with a similar time and delta V.</summary>
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

/// <summary>Total fuel is the sum of measured mass losses. Translation is estimated from delivered delta V, while slew and coast use measured losses. The attitude bucket holds the remainder and can be negative when the translation model exceeds measured consumption.</summary>
internal struct RcsFuelSummary
{
    public bool Valid;
    public double TotalKg;
    public double TranslationKg;
    public double SlewKg;
    public double CoastKg;
    public double AttitudeKg;

    /// <summary>Estimate effective exhaust speed in m/s from baseline mass, observed delta V, and consumed propellant. This approximation returns zero when no matching burn is loaded.</summary>
    public double EffectiveVeMs;

    /// <summary>Measure the angle between accumulated and target delta V in degrees.</summary>
    public double DvAngleDeg;

    public double ElapsedSec;
}

/// <summary>Propellant/duration estimates for the two attitude strategies, recomputed periodically for the UI and the Auto decision.</summary>
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

    /// <summary>Auto selects the cheaper feasible propellant estimate. Return zero when neither strategy is feasible. The editor and activation warning share this calculation.</summary>
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
