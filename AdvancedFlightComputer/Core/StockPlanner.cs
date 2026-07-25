using System.Reflection;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Typed access to the private <see cref="TransferPlanner"/> statics the mod
/// observes and, in a few places, drives. Every read and write of one of those
/// fields goes through here, so the surface a game update can move is
/// enumerable in one file instead of spread across the features.
///
/// A missing handle yields a neutral value rather than throwing:
/// <see cref="GameReflection.ValidateManeuverTools"/> resolves all of these
/// before any feature sets Enabled, so in a loaded mod the case does not arise;
/// the forgiving form is here so a feature reading a field outside its own
/// validation set degrades instead of throwing out of a draw or physics
/// callback. For <see cref="SourceBody"/> the neutral value is
/// <c>new TransferObject(-1)</c>, which is stock's own "none" sentinel (see
/// <see cref="TransferPlanner.DrawPlanWindow"/>'s destination handling): its
/// Body resolves to null and its key to "N/A".
///
/// Deliberately not covered: the private <c>TransferPlanner.Source</c> property
/// getter, which only <see cref="MultiPass.HohmannCreateInterceptor"/>'s click
/// gate invokes, and the transfer-window time fields, which HyperbolicTargets
/// writes as inputs rather than observing as state.
/// </summary>
internal static class StockPlanner
{
    /// <summary>Whether stock's Transfer Planning window is open. Note that
    /// <see cref="TransferPlanner.ShowPlanWindow"/>'s public setter also clears
    /// stock's selection state, so write through that property, not this one,
    /// when the intent is to close the window.</summary>
    public static bool ShowPlanWindow
    {
        get => ReadBool(GameReflection.TransferPlanner_showPlanWindow);
        set => GameReflection.TransferPlanner_showPlanWindow?.SetValue(null, value);
    }

    /// <summary>Stock's "Preview Selected Transfer" checkbox.</summary>
    public static bool DisplaySelectedTransfer
    {
        get => ReadBool(GameReflection.TransferPlanner_displaySelectedTransfer);
        set => GameReflection.TransferPlanner_displaySelectedTransfer?.SetValue(null, value);
    }

    /// <summary>Stock's "a porkchop result is on screen and safe to index" flag.
    /// It gates the whole selected-entry block in
    /// <see cref="TransferPlanner.DrawPlanWindow"/>, including the unchecked
    /// <c>PorkChopData[...]</c> reads, so anything that sets it true is
    /// asserting that the array behind it is populated.</summary>
    public static bool TransferCalculated
    {
        get => ReadBool(GameReflection.TransferPlanner_transferCalculated);
        set => GameReflection.TransferPlanner_transferCalculated?.SetValue(null, value);
    }

    /// <summary>True while a <see cref="TransferTask"/> is running. Stock holds
    /// <see cref="TransferCalculated"/> false for that whole span because
    /// <see cref="TransferTask.Run"/> replaces
    /// <c>TransferInfo.PorkChopData</c> with a fresh all-null array.</summary>
    public static bool TransferBeingCalculated
        => ReadBool(GameReflection.TransferPlanner_transferBeingCalculated);

    /// <summary>Selected plan type, or null when the field is gone.</summary>
    public static KSA.TransferType? TransferType
    {
        get => (KSA.TransferType?)GameReflection.TransferPlanner_transferType?.GetValue(null);
        set
        {
            if (value.HasValue)
                GameReflection.TransferPlanner_transferType?.SetValue(null, value.Value);
        }
    }

    /// <summary>Plan-type key, or null when the field is gone. Saves the cast at
    /// the sites that only compare keys.</summary>
    public static string? TransferTypeKey => TransferType?.GetKey();

    public static TransferObject SourceBody
    {
        get => (TransferObject?)GameReflection.TransferPlanner_sourceBody?.GetValue(null)
               ?? new TransferObject(-1);
        set => GameReflection.TransferPlanner_sourceBody?.SetValue(null, value);
    }

    /// <summary>The source as a Vehicle, or null when the source is unset or is
    /// not a vehicle. Resolving the underlying <see cref="TransferObject"/> goes
    /// through <see cref="CelestialSystem.GetIndex"/>, which throws for an index
    /// past the end of the lookup, so this is not exception-free by
    /// construction - it is exactly as safe as the stock field it reads.</summary>
    public static Vehicle? SourceVehicle => SourceBody.Body as Vehicle;

    public static OrbitalTransfers.PorkChopEntry? SelectedEntry
        => GameReflection.TransferPlanner_selectedEntry?.GetValue(null)
            as OrbitalTransfers.PorkChopEntry;

    public static OrbitalTransfers.TransferInfo? TransferInfo
        => GameReflection.TransferPlanner_transferInfo?.GetValue(null)
            as OrbitalTransfers.TransferInfo;

    /// <summary>Stock's pending transfer burn. Multi-pass keeps this pointing at
    /// the current pass so stock's Create-button guard blocks re-clicks.</summary>
    public static Burn? TransferBurn
    {
        get => GameReflection.TransferPlanner_transferBurn?.GetValue(null) as Burn;
        set => GameReflection.TransferPlanner_transferBurn?.SetValue(null, value);
    }

    /// <summary>Whether stock's selected-entry block in
    /// <c>TransferPlanner.DrawPlanWindow</c> can run without dereferencing a null
    /// porkchop cell. That block reads
    /// <c>_transferInfo.PorkChopData[...].TransferData</c> unchecked, and
    /// <see cref="TransferCalculated"/> is the flag stock uses to assert the array
    /// behind it is populated, so anything that re-asserts the flag has to prove
    /// this rather than assume it.</summary>
    public static bool SelectedTransferBlockIsSafe
        => CanIndexPorkChopData(SelectedEntry, TransferBeingCalculated, TransferInfo);

    /// <summary>Explicit-input form of <see cref="SelectedTransferBlockIsSafe"/>,
    /// so the decision can be exercised without writing stock's statics.
    ///
    /// Two states leave the porkchop array unpopulated while
    /// <c>_selectedEntry</c> stays set, and stock's own
    /// <c>BestDvTransferIndex.X != -1</c> test catches neither: nothing ever writes
    /// that sentinel, since <c>TransferTask.Run</c> is its only assignment site and
    /// always writes a real index.
    /// <list type="bullet">
    /// <item><c>TransferTask.Run</c> installs an all-null
    /// <c>PorkChopEntry[292, 292]</c> before queuing its workers and again before
    /// merging their results, so cells are null or half-filled for the duration.
    /// <c>_transferBeingCalculated</c> spans exactly that window.</item>
    /// <item><c>TransferPlanner.SetTransferInfo</c> installs a brand-new
    /// TransferInfo whose PorkChopData is equally all-null and whose
    /// BestDvTransferIndex is the default (0, 0). It runs on every source,
    /// destination and plan-type change, and none of those clear
    /// <c>_selectedEntry</c>.</item>
    /// </list>
    ///
    /// Sampling the two indices stock can reach is sufficient because outside the
    /// calculating window the array is all-or-nothing: a completed run leaves every
    /// cell populated (its own best-index scan dereferences all of them, so a null
    /// would have thrown on the worker first), and a fresh TransferInfo leaves
    /// every cell null.</summary>
    public static bool CanIndexPorkChopData(
        OrbitalTransfers.PorkChopEntry? selectedEntry,
        bool transferBeingCalculated,
        OrbitalTransfers.TransferInfo? info)
    {
        if (selectedEntry == null) return false;
        if (transferBeingCalculated) return false;

        OrbitalTransfers.PorkChopEntry[,]? data = info?.PorkChopData;
        if (data == null) return false;

        // Stock reads whichever index the "Select Best Dv" checkbox picks, so both
        // have to be resolvable.
        return HasEntry(data, info!.BestDvTransferIndex)
               && HasEntry(data, info.BestDvRelSpeedTransferIndex);
    }

    private static bool HasEntry(OrbitalTransfers.PorkChopEntry[,] data, int2 index)
        => index.X >= 0 && index.X < data.GetLength(0)
           && index.Y >= 0 && index.Y < data.GetLength(1)
           && data[index.X, index.Y] != null;

    private static bool ReadBool(FieldInfo? field)
        => (bool)(field?.GetValue(null) ?? false);
}
