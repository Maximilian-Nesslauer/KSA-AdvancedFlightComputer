using System;
using System.Reflection;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Centralized registry of all reflection targets for game internals.
/// Resolved once at assembly load. Per-feature validation methods check
/// that all targets for a feature resolved successfully so each feature
/// can degrade independently across game versions.
///
/// Method lookups pin the parameter list to keep us bound to the intended
/// overload if the game introduces a new same-named method.
/// </summary>
internal static class GameReflection
{
    #region HyperbolicTargets

    public static readonly FieldInfo? TransferPlanner_sourceBody =
        AccessTools.Field(typeof(TransferPlanner), "_sourceBody");
    public static readonly FieldInfo? TransferPlanner_transferInfo =
        AccessTools.Field(typeof(TransferPlanner), "_transferInfo");
    public static readonly FieldInfo? TransferPlanner_selectedMinTime =
        AccessTools.Field(typeof(TransferPlanner), "_selectedMinTime");
    public static readonly FieldInfo? TransferPlanner_selectedMaxTime =
        AccessTools.Field(typeof(TransferPlanner), "_selectedMaxTime");
    public static readonly FieldInfo? TransferPlanner_selectedTimeUnit =
        AccessTools.Field(typeof(TransferPlanner), "_selectedTimeUnit");
    public static readonly FieldInfo? TransferPlanner_timeUnits =
        AccessTools.Field(typeof(TransferPlanner), "_timeUnits");
    public static readonly FieldInfo? TransferPlanner_selectedEntry =
        AccessTools.Field(typeof(TransferPlanner), "_selectedEntry");

    #endregion

    #region ManeuverTools

    public static readonly FieldInfo? TransferPlanner_transferType =
        AccessTools.Field(typeof(TransferPlanner), "_transferType");
    public static readonly FieldInfo? TransferPlanner_transferCalculated =
        AccessTools.Field(typeof(TransferPlanner), "_transferCalculated");
    public static readonly FieldInfo? TransferPlanner_transferBeingCalculated =
        AccessTools.Field(typeof(TransferPlanner), "_transferBeingCalculated");
    public static readonly FieldInfo? TransferPlanner_transferBurn =
        AccessTools.Field(typeof(TransferPlanner), "_transferBurn");
    public static readonly FieldInfo? TransferPlanner_correctionTime =
        AccessTools.Field(typeof(TransferPlanner), "_correctionTime");
    public static readonly FieldInfo? TransferPlanner_showPlanWindow =
        AccessTools.Field(typeof(TransferPlanner), "_showPlanWindow");
    public static readonly MethodInfo? TransferPlanner_SetTransferInfo =
        AccessTools.Method(typeof(TransferPlanner), "SetTransferInfo", Type.EmptyTypes);

    // "Preview Selected Transfer" checkbox in stock's plan window. Gates
    // OnPreRender's DrawSelectedTransfer call. Our Hohmann multi-pass
    // overlay piggybacks on the same toggle so the user only manages one
    // preview switch for both stock single-burn and multi-pass.
    public static readonly FieldInfo? TransferPlanner_displaySelectedTransfer =
        AccessTools.Field(typeof(TransferPlanner), "_displaySelectedTransfer");

    #endregion

    #region MultiPass

    public static readonly MethodInfo? Vehicle_UpdateFromTaskResults =
        AccessTools.Method(typeof(Vehicle), nameof(Vehicle.UpdateFromTaskResults),
            new Type[]
            {
                typeof(VehicleUpdateData).MakeByRefType(),
                typeof(BubbleOrigin).MakeByRefType(),
                typeof(Vehicle),
                typeof(ReadOnlySpan<Vehicle>),
                typeof(Brutal.Numerics.double3),
                typeof(Brutal.Numerics.double3),
            });

    // UncompressedSave is the concrete path that calls Universe.DeserializeSave;
    // VehicleSave.Load is per-vehicle, not world-state. We use UncompressedSave.Id
    // as the save-game discriminator for registry scoping.
    public static readonly MethodInfo? UncompressedSave_Load =
        AccessTools.Method(typeof(UncompressedSave), nameof(UncompressedSave.Load),
            Type.EmptyTypes);
    public static readonly MethodInfo? UncompressedSave_Write =
        AccessTools.Method(typeof(UncompressedSave), nameof(UncompressedSave.Write),
            Type.EmptyTypes);

    // Drop registry entries when their vehicle is destroyed; otherwise
    // a recycled vehicle id could pick up an orphaned execution.
    public static readonly MethodInfo? Vehicle_Dispose =
        AccessTools.Method(typeof(Vehicle), nameof(Vehicle.Dispose),
            Type.EmptyTypes);

    #endregion

    #region RcsTranslation

    // The gauge button stores its bound enum privately; the RCS gauge
    // patches need it to recognize the BurnMode/Auto button instance.
    public static readonly FieldInfo? GaugeButtonFlightComputer_enumValue =
        AccessTools.Field(typeof(GaugeButtonFlightComputer), "_enumValue");

    // Private tooltip hook for the Auto button; replaced with the RCS
    // explanation when a burn resolves to RCS execution.
    public static readonly MethodInfo? Vehicle_Hovered_BurnMode =
        AccessTools.Method(typeof(Vehicle), "Hovered",
            new Type[] { typeof(FlightComputerBurnMode) });

    // The 4980 flight burn editor draws through this static gauge-canvas
    // host; the RCS burn panel postfixes it. Validated so a game-side rework
    // degrades the panel gracefully instead of aborting the mod's patching.
    public static readonly MethodInfo? BurnCanvasHost_Draw =
        AccessTools.Method(typeof(BurnCanvasHost), "Draw",
            new Type[] { typeof(GaugeCanvas), typeof(Brutal.Numerics.float2), typeof(Brutal.Numerics.float2) });

    #endregion

    #region Validation

    public static bool ValidateHyperbolicTargets()
    {
        var targets = new (string name, object? target)[]
        {
            ("TransferPlanner._sourceBody",       TransferPlanner_sourceBody),
            ("TransferPlanner._transferInfo",      TransferPlanner_transferInfo),
            ("TransferPlanner._selectedMinTime",   TransferPlanner_selectedMinTime),
            ("TransferPlanner._selectedMaxTime",   TransferPlanner_selectedMaxTime),
            ("TransferPlanner._selectedTimeUnit",  TransferPlanner_selectedTimeUnit),
            ("TransferPlanner._timeUnits",         TransferPlanner_timeUnits),
            ("TransferPlanner._selectedEntry",     TransferPlanner_selectedEntry),
        };
        return ValidateTargets("HyperbolicTargets", targets);
    }

    public static bool ValidateManeuverTools()
    {
        var targets = new (string name, object? target)[]
        {
            ("TransferPlanner._sourceBody",              TransferPlanner_sourceBody),
            ("TransferPlanner._transferInfo",             TransferPlanner_transferInfo),
            ("TransferPlanner._selectedEntry",            TransferPlanner_selectedEntry),
            ("TransferPlanner._transferType",             TransferPlanner_transferType),
            ("TransferPlanner._transferCalculated",       TransferPlanner_transferCalculated),
            ("TransferPlanner._transferBeingCalculated",  TransferPlanner_transferBeingCalculated),
            ("TransferPlanner._transferBurn",             TransferPlanner_transferBurn),
            ("TransferPlanner._correctionTime",           TransferPlanner_correctionTime),
            ("TransferPlanner._showPlanWindow",           TransferPlanner_showPlanWindow),
            ("TransferPlanner.SetTransferInfo",            TransferPlanner_SetTransferInfo),
            ("TransferPlanner._displaySelectedTransfer",   TransferPlanner_displaySelectedTransfer),
        };
        return ValidateTargets("ManeuverTools", targets);
    }

    /// <summary>Separate from ManeuverTools so a missing
    /// UpdateFromTaskResults disables only multi-pass execution, leaving
    /// the maneuver quick-tools functional. Without UncompressedSave
    /// hooks we cannot scope registry entries to a save game.
    ///
    /// The stock plan-window fields multi-pass also reads are not listed here:
    /// <c>Mod.OnFullyLoaded</c> nests this block inside
    /// <see cref="ValidateManeuverTools"/>, which already covers them.</summary>
    public static bool ValidateMultiPass()
    {
        var targets = new (string name, object? target)[]
        {
            ("Vehicle.UpdateFromTaskResults", Vehicle_UpdateFromTaskResults),
            ("UncompressedSave.Load",         UncompressedSave_Load),
            ("UncompressedSave.Write",        UncompressedSave_Write),
            ("Vehicle.Dispose",               Vehicle_Dispose),
        };
        return ValidateTargets("MultiPass", targets);
    }

    /// <summary>Shares the save/load and per-tick hooks with MultiPass on
    /// purpose: both features scope their registries by save id and drive
    /// their state machines from Vehicle.UpdateFromTaskResults.</summary>
    public static bool ValidateRcsTranslation()
    {
        var targets = new (string name, object? target)[]
        {
            ("Vehicle.UpdateFromTaskResults",              Vehicle_UpdateFromTaskResults),
            ("UncompressedSave.Load",                      UncompressedSave_Load),
            ("UncompressedSave.Write",                     UncompressedSave_Write),
            ("Vehicle.Dispose",                            Vehicle_Dispose),
            ("GaugeButtonFlightComputer._enumValue",       GaugeButtonFlightComputer_enumValue),
            ("Vehicle.Hovered(FlightComputerBurnMode)",    Vehicle_Hovered_BurnMode),
            ("BurnCanvasHost.Draw",                        BurnCanvasHost_Draw),
        };
        return ValidateTargets("RcsTranslation", targets);
    }

    private static bool ValidateTargets(string feature, (string name, object? target)[] targets)
    {
        bool allOk = true;
        foreach (var (name, target) in targets)
        {
            if (target == null)
            {
                DefaultCategory.Log.Error(
                    $"[AFC] {feature}: {name} not found - game version may have changed.");
                allOk = false;
            }
        }
        return allOk;
    }

    #endregion
}
