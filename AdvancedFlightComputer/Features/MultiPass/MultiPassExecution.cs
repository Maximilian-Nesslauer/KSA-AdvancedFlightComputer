using System;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

// Live burn references and control flags are transient. Persisted time and delta v support reattachment after load.
internal sealed class MultiPassExecution
{
    // The round trip format preserves burn time exactly. These tolerances allow small time edits and drift from delta v frame transforms while distinguishing adjacent burns.
    private const double BurnTimeMatchToleranceSec = 0.05;
    private const double BurnDvMatchToleranceMs = 0.1;

    #region Persisted state (serialised to multipass.toml)

    // The ID stays empty until the first save. RekeyTo follows Save As and overwrite operations.
    public required string SaveId { get; internal set; }

    public required string VehicleId { get; init; }
    public required IManeuverIntent Intent { get; init; }
    public required SplitMode Mode { get; init; }
    public required int PassCountTotal { get; init; }

    public int PassIndex { get; set; }

    // Time and delta v identify the restored burn after loading a save.
    public double? CurrentBurnTimeSec { get; set; }
    public double? CurrentBurnDvMagnitudeMs { get; set; }

    #endregion

    #region Transient state (in-memory only)

    // The reference is null between passes and before reattachment after load.
    public Burn? CurrentBurn { get; set; }

    // Suppress deletion detection while the buffered addition is pending.
    public bool AwaitingMaterialization { get; set; }

    public int AwaitingMaterializationTicks { get; set; }

    public int ConsecutiveScheduleFailures { get; set; }

    // Restore Auto only after FlightComputer.LoadBurn has reset the new pass to Manual.
    public bool ReengageAutoOnNextBurn { get; set; }

    // Auto includes alignment before ignition. Observing it does not prove that thrust occurred.
    public bool BurnAutoEngagedThisPass { get; set; }

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

    public void ReattachAfterLoad(Burn burn)
    {
        CurrentBurn = burn;
    }

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
