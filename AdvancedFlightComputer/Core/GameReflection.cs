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
            ("TransferPlanner.SetTransferInfo",          TransferPlanner_SetTransferInfo),
        };
        return ValidateTargets("ManeuverTools", targets);
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
