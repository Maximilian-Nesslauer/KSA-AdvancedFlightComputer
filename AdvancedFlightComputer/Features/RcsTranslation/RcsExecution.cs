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
    /// <see cref="RcsExecRegistry.RekeyTransientsTo"/> promotes such entries
    /// at the first save, same policy as the multi-pass registry.</summary>
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

    #endregion

    #region Transient state

    public Burn? ActiveBurn;

    /// <summary>Set once the driver has re-resolved ActiveBurn after a load.</summary>
    public bool ReconciledAfterLoad;

    public RcsCapabilitySnapshot Capability;
    public double CapabilityProbedAtSec = double.NegativeInfinity;

    public RcsEstimates Estimates;
    public double EstimatesComputedAtSec = double.NegativeInfinity;

    /// <summary>One-shot guard so the propellant-stall alert fires once per stall.</summary>
    public bool StallAlerted;

    /// <summary>No-progress watchdog baseline: the smallest to-go seen and
    /// when it was seen. Zero/infinity until the first active tick.</summary>
    public float WatchdogTogoMs = float.PositiveInfinity;
    public double WatchdogAtSec;

    /// <summary>One-shot guard for the ignition-crossing debug log.</summary>
    public bool FiringLogged;

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
        StallAlerted = false;
        WatchdogTogoMs = float.PositiveInfinity;
        WatchdogAtSec = 0.0;
        FiringLogged = false;
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
}
