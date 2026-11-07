using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Typed access to the private <see cref="TransferPlanner"/> statics the unbound-target support reads
/// and writes for stock's own planner window, so the surface a game update can move is enumerable in
/// one file. The AFC console plans transfers in its own session and reads none of it.
///
/// A missing handle yields a neutral value rather than throwing. For <see cref="SourceBody"/> the
/// neutral value is <c>new TransferObject(-1)</c>, stock's own "none" sentinel, whose Body resolves
/// to null and whose key reads "N/A".
/// </summary>
internal static class StockPlanner
{
    /// <summary>The time units stock offers for the transfer window, or null when the field is gone.</summary>
    public static List<TimeObject>? TimeUnits => Read(GameReflection.TransferPlanner_timeUnitsRef);

    public static TimeObject SelectedTimeUnit
    {
        get => Read(GameReflection.TransferPlanner_selectedTimeUnitRef);
        set => Write(GameReflection.TransferPlanner_selectedTimeUnitRef, value);
    }

    public static TransferObject SourceBody
        => GameReflection.TransferPlanner_sourceBodyRef is { } accessor ? accessor() : new TransferObject(-1);

    /// <summary>The source as a Vehicle, or null. Resolving the <see cref="TransferObject"/> goes
    /// through <see cref="CelestialSystem.GetIndex"/>, which throws for an index past the end of the
    /// lookup, so this is exactly as safe as the stock field it reads.</summary>
    public static Vehicle? SourceVehicle => SourceBody.Body as Vehicle;

    public static OrbitalTransfers.TransferInfo? TransferInfo
        => Read(GameReflection.TransferPlanner_transferInfoRef);

    private static F? Read<F>(AccessTools.FieldRef<F>? field)
        => field == null ? default : field();

    private static void Write<F>(AccessTools.FieldRef<F>? field, F value)
    {
        if (field != null)
            field() = value;
    }
}
