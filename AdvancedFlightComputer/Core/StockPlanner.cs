using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Typed access to the private <see cref="TransferPlanner"/> statics the mod observes and, in a few
/// places, drives, so the surface a game update can move is enumerable in one file.
///
/// A missing handle yields a neutral value rather than throwing.
/// <c>Mod</c> validates each feature's handles before it applies the feature patch block, so the
/// case only arises for a feature reading a field outside its own validation set,
/// and there it degrades instead of throwing out of a draw or physics callback. For
/// <see cref="SourceBody"/> the neutral value is <c>new TransferObject(-1)</c>, stock's own "none"
/// sentinel from the destination handling in <see cref="TransferPlanner.DrawPlanWindow"/>, whose
/// Body resolves to null and whose key reads "N/A".
///
/// The transfer-window time bounds are not covered, because HyperbolicTargets writes them as
/// inputs rather than observing them as state.
/// </summary>
internal static class StockPlanner
{
    /// <summary>Whether stock's Transfer Planning window is open.
    /// <see cref="TransferPlanner.ShowPlanWindow"/>'s public setter also clears stock's selection
    /// state, so close the window through that property, not through this one.</summary>
    public static bool ShowPlanWindow
    {
        get => Read(GameReflection.TransferPlanner_showPlanWindowRef);
        set => Write(GameReflection.TransferPlanner_showPlanWindowRef, value);
    }

    /// <summary>Stock's "Preview Selected Transfer" checkbox.</summary>
    public static bool DisplaySelectedTransfer
    {
        get => Read(GameReflection.TransferPlanner_displaySelectedTransferRef);
        set => Write(GameReflection.TransferPlanner_displaySelectedTransferRef, value);
    }

    /// <summary>Stock's "a porkchop result is on screen and safe to index" flag. It gates the whole
    /// selected-entry block in <see cref="TransferPlanner.DrawPlanWindow"/>, including the unchecked
    /// <c>PorkChopData[...]</c> reads, so setting it true asserts that the array behind it is
    /// populated.</summary>
    public static bool TransferCalculated
    {
        get => Read(GameReflection.TransferPlanner_transferCalculatedRef);
        set => Write(GameReflection.TransferPlanner_transferCalculatedRef, value);
    }

    /// <summary>True while a <see cref="TransferTask"/> runs. Stock holds
    /// <see cref="TransferCalculated"/> false for that whole span because
    /// <see cref="TransferTask.Run"/> replaces <c>TransferInfo.PorkChopData</c> with a fresh
    /// all-null array.</summary>
    public static bool TransferBeingCalculated
        => Read(GameReflection.TransferPlanner_transferBeingCalculatedRef);

    /// <summary>Selected plan type, or null when the field is gone.</summary>
    public static KSA.TransferType? TransferType
    {
        get => GameReflection.TransferPlanner_transferTypeRef is { } accessor ? accessor() : null;
        set
        {
            if (value.HasValue)
                Write(GameReflection.TransferPlanner_transferTypeRef, value.Value);
        }
    }

    public static string? TransferTypeKey => TransferType?.GetKey();

    public static TransferObject SourceBody
    {
        get => GameReflection.TransferPlanner_sourceBodyRef is { } accessor ? accessor() : new TransferObject(-1);
        set => Write(GameReflection.TransferPlanner_sourceBodyRef, value);
    }

    /// <summary>The source as a Vehicle, or null. Resolving the <see cref="TransferObject"/> goes
    /// through <see cref="CelestialSystem.GetIndex"/>, which throws for an index past the end of the
    /// lookup, so this is exactly as safe as the stock field it reads.</summary>
    public static Vehicle? SourceVehicle => SourceBody.Body as Vehicle;

    public static OrbitalTransfers.PorkChopEntry? SelectedEntry
        => Read(GameReflection.TransferPlanner_selectedEntryRef);

    public static OrbitalTransfers.TransferInfo? TransferInfo
        => Read(GameReflection.TransferPlanner_transferInfoRef);

    /// <summary>Stock's pending transfer burn. Multi-pass keeps this pointing at the current pass so
    /// stock's Create-button guard blocks re-clicks.</summary>
    public static Burn? TransferBurn
    {
        get => Read(GameReflection.TransferPlanner_transferBurnRef);
        set => Write(GameReflection.TransferPlanner_transferBurnRef, value);
    }

    /// <summary>Whether stock's selected-entry block in <c>TransferPlanner.DrawPlanWindow</c> can
    /// run without dereferencing a null porkchop cell, as decided by
    /// <see cref="CanIndexPorkChopData"/>.</summary>
    public static bool SelectedTransferBlockIsSafe
        => CanIndexPorkChopData(SelectedEntry, TransferBeingCalculated, TransferInfo);

    /// <summary>Explicit-input form of <see cref="SelectedTransferBlockIsSafe"/>, so the decision
    /// can be exercised without writing stock's statics.
    ///
    /// Two states leave the porkchop array unpopulated while <c>_selectedEntry</c> stays set, and
    /// stock's own <c>BestDvTransferIndex.X != -1</c> test catches neither, because nothing ever
    /// writes that sentinel. <c>TransferTask.Run</c> installs an all-null
    /// <c>PorkChopEntry[292, 292]</c> before queuing its workers and again before merging their
    /// results, which is the span <paramref name="transferBeingCalculated"/> covers, and
    /// <c>TransferPlanner.SetTransferInfo</c> installs a brand-new TransferInfo with an equally
    /// all-null array on every source, destination and plan-type change without clearing
    /// <c>_selectedEntry</c>.
    ///
    /// Outside the calculating window the array is all-or-nothing, because a completed run
    /// dereferences every cell in its best-index scan and a fresh TransferInfo has none, so
    /// checking the two cells stock can read is sufficient.</summary>
    public static bool CanIndexPorkChopData(
        OrbitalTransfers.PorkChopEntry? selectedEntry,
        bool transferBeingCalculated,
        OrbitalTransfers.TransferInfo? info)
    {
        if (selectedEntry == null || transferBeingCalculated)
            return false;

        OrbitalTransfers.PorkChopEntry[,]? data = info?.PorkChopData;
        if (data == null)
            return false;

        // Stock reads whichever index the "Select Best Dv" checkbox picks.
        return HasEntry(data, info!.BestDvTransferIndex)
               && HasEntry(data, info.BestDvRelSpeedTransferIndex);
    }

    private static bool HasEntry(OrbitalTransfers.PorkChopEntry[,] data, int2 index)
        => index.X >= 0 && index.X < data.GetLength(0)
           && index.Y >= 0 && index.Y < data.GetLength(1)
           && data[index.X, index.Y] != null;

    private static F? Read<F>(AccessTools.FieldRef<F>? field)
        => field == null ? default : field();

    private static void Write<F>(AccessTools.FieldRef<F>? field, F value)
    {
        if (field != null)
            field() = value;
    }
}
