using System;
using System.Reflection;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Every string-keyed handle into game internals, resolved once at assembly load. A feature
/// validates the handles it needs before it patches anything, so a game-side rename disables that
/// feature and leaves the others running. Method lookups pin the parameter list so a new
/// same-named overload cannot be picked up by accident.
///
/// The <see cref="UsedByAttribute"/> on each handle is what the validation reads, and a handle
/// without one fails every validation, so a key cannot be added without being validated for the
/// features that use it. Validators ignore <c>SoftAnchor</c> handles because each one controls
/// only its own patch.
/// </summary>
internal static class GameReflection
{
    [Flags]
    private enum Feature
    {
        HyperbolicTargets = 1,
        ManeuverTools = 2,
        MultiPass = 4,
        RcsTranslation = 8,
        Core = 16,
        PlanWindow = 32,

        /// <summary>Handles that control one optional patch. Each patch checks its own handle.</summary>
        SoftAnchor = 64,

        AutoStage = 128,
    }

    [AttributeUsage(AttributeTargets.Field)]
    private sealed class UsedByAttribute(Feature features) : Attribute
    {
        public Feature Features { get; } = features;
    }

    #region TransferPlanner

    [UsedBy(Feature.HyperbolicTargets | Feature.ManeuverTools | Feature.PlanWindow)]
    public static readonly FieldInfo? TransferPlanner_sourceBody =
        AccessTools.Field(typeof(TransferPlanner), "_sourceBody");

    [UsedBy(Feature.HyperbolicTargets | Feature.PlanWindow)]
    public static readonly FieldInfo? TransferPlanner_transferInfo =
        AccessTools.Field(typeof(TransferPlanner), "_transferInfo");

    [UsedBy(Feature.PlanWindow)]
    public static readonly FieldInfo? TransferPlanner_selectedEntry =
        AccessTools.Field(typeof(TransferPlanner), "_selectedEntry");

    // The transfer-window time bounds HyperbolicTargets writes as inputs.
    [UsedBy(Feature.HyperbolicTargets)]
    public static readonly FieldInfo? TransferPlanner_selectedMinTime =
        AccessTools.Field(typeof(TransferPlanner), "_selectedMinTime");

    [UsedBy(Feature.HyperbolicTargets)]
    public static readonly FieldInfo? TransferPlanner_selectedMaxTime =
        AccessTools.Field(typeof(TransferPlanner), "_selectedMaxTime");

    [UsedBy(Feature.HyperbolicTargets)]
    public static readonly FieldInfo? TransferPlanner_selectedTimeUnit =
        AccessTools.Field(typeof(TransferPlanner), "_selectedTimeUnit");

    [UsedBy(Feature.HyperbolicTargets)]
    public static readonly FieldInfo? TransferPlanner_timeUnits =
        AccessTools.Field(typeof(TransferPlanner), "_timeUnits");

    [UsedBy(Feature.ManeuverTools | Feature.PlanWindow)]
    public static readonly FieldInfo? TransferPlanner_transferType =
        AccessTools.Field(typeof(TransferPlanner), "_transferType");

    [UsedBy(Feature.ManeuverTools | Feature.PlanWindow)]
    public static readonly FieldInfo? TransferPlanner_transferCalculated =
        AccessTools.Field(typeof(TransferPlanner), "_transferCalculated");

    [UsedBy(Feature.ManeuverTools | Feature.PlanWindow)]
    public static readonly FieldInfo? TransferPlanner_transferBeingCalculated =
        AccessTools.Field(typeof(TransferPlanner), "_transferBeingCalculated");

    [UsedBy(Feature.PlanWindow)]
    public static readonly FieldInfo? TransferPlanner_transferBurn =
        AccessTools.Field(typeof(TransferPlanner), "_transferBurn");

    [UsedBy(Feature.PlanWindow)]
    public static readonly FieldInfo? TransferPlanner_showPlanWindow =
        AccessTools.Field(typeof(TransferPlanner), "_showPlanWindow");

    // Stock's "Preview Selected Transfer" checkbox. The Hohmann multi-pass overlay rides the same
    // toggle, so the user manages one preview switch for stock's single burn and the passes.
    [UsedBy(Feature.PlanWindow)]
    public static readonly FieldInfo? TransferPlanner_displaySelectedTransfer =
        AccessTools.Field(typeof(TransferPlanner), "_displaySelectedTransfer");

    // HyperbolicTargets binds its finalizer to the same method by name.
    [UsedBy(Feature.HyperbolicTargets | Feature.PlanWindow)]
    public static readonly MethodInfo? TransferPlanner_SetTransferInfo =
        AccessTools.Method(typeof(TransferPlanner), "SetTransferInfo", Type.EmptyTypes);

    // Typed accessors over the handles above for the plan-window state StockPlanner reads per
    // frame, one delegate each and no boxing per read. An accessor is null when its handle is
    // null or the field no longer holds the expected type, so the validation reports it like any
    // other missing handle.
    [UsedBy(Feature.HyperbolicTargets | Feature.ManeuverTools | Feature.PlanWindow)]
    public static readonly AccessTools.FieldRef<TransferObject>? TransferPlanner_sourceBodyRef =
        StaticFieldRef<TransferObject>(TransferPlanner_sourceBody);

    [UsedBy(Feature.HyperbolicTargets | Feature.PlanWindow)]
    public static readonly AccessTools.FieldRef<OrbitalTransfers.TransferInfo?>? TransferPlanner_transferInfoRef =
        StaticFieldRef<OrbitalTransfers.TransferInfo?>(TransferPlanner_transferInfo);

    [UsedBy(Feature.PlanWindow)]
    public static readonly AccessTools.FieldRef<OrbitalTransfers.PorkChopEntry?>? TransferPlanner_selectedEntryRef =
        StaticFieldRef<OrbitalTransfers.PorkChopEntry?>(TransferPlanner_selectedEntry);

    [UsedBy(Feature.ManeuverTools | Feature.PlanWindow)]
    public static readonly AccessTools.FieldRef<TransferType>? TransferPlanner_transferTypeRef =
        StaticFieldRef<TransferType>(TransferPlanner_transferType);

    [UsedBy(Feature.ManeuverTools | Feature.PlanWindow)]
    public static readonly AccessTools.FieldRef<bool>? TransferPlanner_transferCalculatedRef =
        StaticFieldRef<bool>(TransferPlanner_transferCalculated);

    [UsedBy(Feature.ManeuverTools | Feature.PlanWindow)]
    public static readonly AccessTools.FieldRef<bool>? TransferPlanner_transferBeingCalculatedRef =
        StaticFieldRef<bool>(TransferPlanner_transferBeingCalculated);

    [UsedBy(Feature.PlanWindow)]
    public static readonly AccessTools.FieldRef<Burn?>? TransferPlanner_transferBurnRef =
        StaticFieldRef<Burn?>(TransferPlanner_transferBurn);

    [UsedBy(Feature.PlanWindow)]
    public static readonly AccessTools.FieldRef<bool>? TransferPlanner_showPlanWindowRef =
        StaticFieldRef<bool>(TransferPlanner_showPlanWindow);

    [UsedBy(Feature.PlanWindow)]
    public static readonly AccessTools.FieldRef<bool>? TransferPlanner_displaySelectedTransferRef =
        StaticFieldRef<bool>(TransferPlanner_displaySelectedTransfer);

    #endregion

    #region Optional patch anchors

    // This guard prevents the stock search from using a second body with a NaN Period.
    [UsedBy(Feature.SoftAnchor)]
    public static readonly MethodInfo? PatchedConic_FindClosestApproaches =
        AccessTools.Method(typeof(PatchedConic), "FindClosestApproaches", new[]
        {
            typeof(Span<Encounter>),
            typeof(int).MakeByRefType(),
            typeof(IOrbiter),
            typeof(UniverseTime).MakeByRefType(),
        });

    // Stock draws the selected-transfer lines and markers through different viewport interfaces.
    [UsedBy(Feature.SoftAnchor)]
    public static readonly MethodInfo? TransferPlanner_DrawSelectedTransfer =
        AccessTools.Method(typeof(TransferPlanner), "DrawSelectedTransfer",
            new[] { typeof(IViewport) });

    [UsedBy(Feature.SoftAnchor)]
    public static readonly MethodInfo? TransferPlanner_DrawSelectedTransferUi =
        AccessTools.Method(typeof(TransferPlanner), "DrawSelectedTransferUi",
            new[] { typeof(IGameViewport) });

    #endregion

    #region Save, tick and vehicle lifetime

    // The per-frame tick MultiPass and RcsTranslation drive their state machines from. It runs on
    // the main thread after the solver results are applied to every vehicle and before
    // InputEvents.ApplyInputEvents, so a driver reads fresh FlightComputer state and can still queue
    // burn mutations for the same frame's drain. Neither per-vehicle half of the apply is a sound
    // host, because Vehicle.UpdateFromTaskResultsUnsynchronized runs one worker per physics bubble
    // and Vehicle.UpdateFromTaskResultsSynchronized is aggressively inlined, which a Harmony detour
    // on the callee cannot survive.
    [UsedBy(Feature.Core | Feature.MultiPass | Feature.RcsTranslation | Feature.AutoStage)]
    public static readonly MethodInfo? Universe_ApplyVehicleSolvers =
        AccessTools.Method(typeof(Universe), nameof(Universe.ApplyVehicleSolvers), Type.EmptyTypes);

    // UncompressedSave is the concrete path that calls Universe.DeserializeSave, and its Id is the
    // save-game discriminator the registries scope their entries by.
    [UsedBy(Feature.Core | Feature.MultiPass | Feature.RcsTranslation)]
    public static readonly MethodInfo? UncompressedSave_Load =
        AccessTools.Method(typeof(UncompressedSave), nameof(UncompressedSave.Load), Type.EmptyTypes);

    [UsedBy(Feature.Core | Feature.MultiPass | Feature.RcsTranslation)]
    public static readonly MethodInfo? UncompressedSave_Write =
        AccessTools.Method(typeof(UncompressedSave), nameof(UncompressedSave.Write), Type.EmptyTypes);

    // Registry entries drop with their vehicle, or a recycled vehicle id could pick up an orphaned
    // execution.
    [UsedBy(Feature.Core | Feature.MultiPass | Feature.RcsTranslation | Feature.AutoStage)]
    public static readonly MethodInfo? Vehicle_Dispose =
        AccessTools.Method(typeof(Vehicle), nameof(Vehicle.Dispose), new[] { typeof(bool) });

    #endregion

    #region AutoStage

    // The gauge button resolves its bound enum by Type.Name from this list, so the AUTOSTAGE
    // button's own enum type is appended to it at immediate load.
    [UsedBy(Feature.AutoStage)]
    public static readonly FieldInfo? GaugeButtonFlightComputer_EnumTypes =
        AccessTools.Field(typeof(GaugeButtonFlightComputer), "EnumTypes");

    // Closed over System.Enum, because GaugeButtonFlightComputer.PackData calls them on its boxed value.
    [UsedBy(Feature.AutoStage)]
    public static readonly MethodInfo? Vehicle_IsSet_Enum =
        GenericVehicleMethod(nameof(Vehicle.IsSet), parameterCount: 2)?.MakeGenericMethod(typeof(Enum));

    [UsedBy(Feature.AutoStage)]
    public static readonly MethodInfo? Vehicle_IsFlightComputerDisabled_Enum =
        GenericVehicleMethod(nameof(Vehicle.IsFlightComputerDisabled), parameterCount: 1)?.MakeGenericMethod(typeof(Enum));

    // Private setter. The public SetActiveSequence rewrites Activated list-wide and resets caches,
    // which stock's own activation does not.
    [UsedBy(Feature.AutoStage)]
    public static readonly PropertyInfo? SequenceList_ActiveSequence =
        AccessTools.Property(typeof(SequenceList), nameof(SequenceList.ActiveSequence));

    // Stock brackets its activation loop with this flag, and ResetCaches early-returns on it.
    [UsedBy(Feature.AutoStage)]
    public static readonly FieldInfo? SequenceList_updatingSequence =
        AccessTools.Field(typeof(SequenceList), "_updatingSequence");

    // Which settings page the nav rail has open. The enum is private to GameSettings, so its Mods
    // member is resolved as a boxed value once and compared by equality.
    [UsedBy(Feature.AutoStage)]
    public static readonly FieldInfo? GameSettings_openTab =
        AccessTools.Field(typeof(GameSettings), "_openTab");

    [UsedBy(Feature.AutoStage)]
    public static readonly object? GameSettings_openTab_Mods =
        GameSettings_openTab?.FieldType is { IsEnum: true } tab && Enum.TryParse(tab, "Mods", out object? mods) ? mods : null;

    // Only the delay tables in the settings page list the part library; the page draws without them.
    [UsedBy(Feature.SoftAnchor)]
    public static readonly FieldInfo? ModLibrary_AllParts =
        AccessTools.Field(typeof(ModLibrary), "AllParts");

    private static MethodInfo? GenericVehicleMethod(string name, int parameterCount)
    {
        foreach (MethodInfo method in typeof(Vehicle).GetMethods())
        {
            if (method.Name == name && method.IsGenericMethodDefinition
                && method.GetParameters().Length == parameterCount)
                return method;
        }
        return null;
    }

    #endregion

    #region RcsTranslation

    // The gauge button keeps its bound enum private, and the RCS gauge patches need it to recognize
    // the BurnMode button instance.
    [UsedBy(Feature.RcsTranslation)]
    public static readonly FieldInfo? GaugeButtonFlightComputer_enumValue =
        AccessTools.Field(typeof(GaugeButtonFlightComputer), "_enumValue");

    // Private tooltip hook for the Auto button, replaced with the RCS explanation when a burn
    // resolves to RCS execution.
    [UsedBy(Feature.RcsTranslation)]
    public static readonly MethodInfo? Vehicle_Hovered_BurnMode =
        AccessTools.Method(typeof(Vehicle), "Hovered", new[] { typeof(FlightComputerBurnMode) });

    // The flight burn editor draws through this static gauge-canvas host, which the RCS burn panel
    // postfixes.
    [UsedBy(Feature.RcsTranslation)]
    public static readonly MethodInfo? BurnCanvasHost_Draw =
        AccessTools.Method(typeof(BurnCanvasHost), "Draw",
            new[] { typeof(GaugeCanvas), typeof(Brutal.Numerics.float2), typeof(Brutal.Numerics.float2) });

    #endregion

    #region Validation

    public static bool ValidateCore() => Validate(Feature.Core);

    public static bool ValidateHyperbolicTargets() => Validate(Feature.HyperbolicTargets);

    public static bool ValidateManeuverTools() => Validate(Feature.ManeuverTools);

    public static bool ValidatePlanWindow() => Validate(Feature.PlanWindow);

    /// <summary>The plan-window handles MultiPass also reads are not tagged for it because
    /// <c>Mod</c> enables MultiPass only after the PlanWindow gate validates them.</summary>
    public static bool ValidateMultiPass() => Validate(Feature.MultiPass);

    public static bool ValidateRcsTranslation() => Validate(Feature.RcsTranslation);

    public static bool ValidateAutoStage() => Validate(Feature.AutoStage);

    private static bool Validate(Feature feature)
    {
        bool allOk = true;
        HashSet<string>? reported = null;
        foreach (FieldInfo handle in typeof(GameReflection).GetFields(BindingFlags.Public | BindingFlags.Static))
        {
            if (handle.GetCustomAttribute<UsedByAttribute>() is not { } tag)
            {
                DefaultCategory.Log.Error($"[AFC] {handle.Name} carries no UsedBy tag, so no feature validates it.");
                allOk = false;
                continue;
            }
            if ((tag.Features & feature) == 0 || handle.GetValue(null) != null)
                continue;

            // A handle and its typed twin name the same field, so both get one line.
            string name = DisplayName(handle);
            if ((reported ??= new HashSet<string>()).Add(name))
                DefaultCategory.Log.Error($"[AFC] {feature}: {name} not found - game version may have changed.");
            allOk = false;
        }
        return allOk;
    }

    // "TransferPlanner_sourceBody" and "TransferPlanner_sourceBodyRef" both read as
    // "TransferPlanner._sourceBody". A method handle keeps its name, so "Vehicle_Hovered_BurnMode"
    // reads as "Vehicle.Hovered_BurnMode".
    private static string DisplayName(FieldInfo handle)
    {
        string name = handle.Name;
        int split = name.IndexOf('_');
        if (split < 0)
            return name;
        string owner = name[..split];
        string member = name[(split + 1)..];
        if (handle.FieldType == typeof(MethodInfo))
            return $"{owner}.{member}";
        if (member.EndsWith("Ref", StringComparison.Ordinal))
            member = member[..^3];
        return $"{owner}._{member}";
    }

    private static AccessTools.FieldRef<F>? StaticFieldRef<F>(FieldInfo? field)
    {
        if (field == null)
            return null;
        try
        {
            return AccessTools.StaticFieldRefAccess<F>(field);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Error(
                $"[AFC] {field.DeclaringType?.Name}.{field.Name} is not a {typeof(F).Name}: {ex.Message}");
            return null;
        }
    }

    #endregion
}
