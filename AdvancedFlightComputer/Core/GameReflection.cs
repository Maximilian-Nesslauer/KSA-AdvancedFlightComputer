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
        MultiPass = 4,
        RcsTranslation = 8,
        Core = 16,

        /// <summary>Handles that control one optional patch. Each patch checks its own handle.</summary>
        SoftAnchor = 64,
        GuidanceDiagnostics = 128,
        Guidance = 256,
        AutoStage = 512,
        AutoRemove = 1024,
        SettingsPage = 2048,
    }

    [AttributeUsage(AttributeTargets.Field)]
    private sealed class UsedByAttribute(Feature features) : Attribute
    {
        public Feature Features { get; } = features;
    }

    #region TransferPlanner

    // The unbound-target support writes stock planner state from its SetTransferInfo finalizer and lists its comets for stock DESTINATION. The AFC console has its own transfer session and reads none of these.
    [UsedBy(Feature.HyperbolicTargets)]
    public static readonly FieldInfo? TransferPlanner_sourceBody =
        AccessTools.Field(typeof(TransferPlanner), "_sourceBody");

    [UsedBy(Feature.HyperbolicTargets)]
    public static readonly FieldInfo? TransferPlanner_transferInfo =
        AccessTools.Field(typeof(TransferPlanner), "_transferInfo");

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

    // HyperbolicTargets binds its finalizer to the same method by name.
    [UsedBy(Feature.HyperbolicTargets)]
    public static readonly MethodInfo? TransferPlanner_SetTransferInfo =
        AccessTools.Method(typeof(TransferPlanner), "SetTransferInfo", Type.EmptyTypes);

    // Typed accessors over the handles above, one delegate each and no boxing per access.
    // An accessor is null when its handle is null or the field no longer holds the expected type, so the validation reports it like any other missing handle.
    [UsedBy(Feature.HyperbolicTargets)]
    public static readonly AccessTools.FieldRef<TransferObject>? TransferPlanner_sourceBodyRef =
        StaticFieldRef<TransferObject>(TransferPlanner_sourceBody);

    [UsedBy(Feature.HyperbolicTargets)]
    public static readonly AccessTools.FieldRef<OrbitalTransfers.TransferInfo?>? TransferPlanner_transferInfoRef =
        StaticFieldRef<OrbitalTransfers.TransferInfo?>(TransferPlanner_transferInfo);

    [UsedBy(Feature.HyperbolicTargets)]
    public static readonly AccessTools.FieldRef<UniverseTime>? TransferPlanner_selectedMinTimeRef =
        StaticFieldRef<UniverseTime>(TransferPlanner_selectedMinTime);

    [UsedBy(Feature.HyperbolicTargets)]
    public static readonly AccessTools.FieldRef<UniverseTime>? TransferPlanner_selectedMaxTimeRef =
        StaticFieldRef<UniverseTime>(TransferPlanner_selectedMaxTime);

    [UsedBy(Feature.HyperbolicTargets)]
    public static readonly AccessTools.FieldRef<TimeObject>? TransferPlanner_selectedTimeUnitRef =
        StaticFieldRef<TimeObject>(TransferPlanner_selectedTimeUnit);

    [UsedBy(Feature.HyperbolicTargets)]
    public static readonly AccessTools.FieldRef<List<TimeObject>>? TransferPlanner_timeUnitsRef =
        StaticFieldRef<List<TimeObject>>(TransferPlanner_timeUnits);

    // The planner key routing sits on the public setter. It is soft: without it the console frame check routes the key alone.
    [UsedBy(Feature.SoftAnchor)]
    public static readonly MethodInfo? TransferPlanner_set_ShowPlanWindow =
        AccessTools.PropertySetter(typeof(TransferPlanner), nameof(TransferPlanner.ShowPlanWindow));

    #endregion

    #region Optional patch anchors

    // This guard prevents the stock search from using a second body with a NaN Period.
    // The last parameter is the out searchedUntil, which reflection sees as a by-ref type.
    [UsedBy(Feature.SoftAnchor)]
    public static readonly MethodInfo? PatchedConic_FindClosestApproaches =
        AccessTools.Method(typeof(PatchedConic), "FindClosestApproaches", new[]
        {
            typeof(Span<Encounter>),
            typeof(int).MakeByRefType(),
            typeof(IOrbiter),
            typeof(UniverseTime).MakeByRefType(),
            typeof(UniverseTime).MakeByRefType(),
        });

    // Re-patched after the hook, see GuidanceFeature.KeepMenuHookCall.
    [UsedBy(Feature.SoftAnchor)]
    public static readonly MethodInfo? Program_DrawMenuBar =
        AccessTools.Method(typeof(Program), nameof(Program.DrawMenuBar),
            new[] { typeof(IGameViewport), typeof(int) });

    #endregion

    #region Save, tick and vehicle lifetime

    // The per-frame tick MultiPass, RcsTranslation and AutoStage drive their state machines from.
    // It runs on the main thread after the solver results are applied to every vehicle and before InputEvents.ApplyInputEvents, so a driver reads fresh FlightComputer state and can still queue burn mutations for the same frame's drain.
    // AutoStage takes back a worker's FlightComputer.AutoBurnStoppedOutOfPropellant here, before the input drain and before FlightComputer.RaisePendingAlerts acts on it later in Program.PrepareFrame. The RCS executor raises that flag and FlightComputer.AutoBurnCompleted here for its own burns, after AutoStage has run.
    // Neither per-vehicle half of the apply is a sound host, because Vehicle.UpdateFromTaskResultsUnsynchronized runs one worker per physics bubble and Vehicle.UpdateFromTaskResultsSynchronized is aggressively inlined, which a Harmony detour on the callee cannot survive.
    [UsedBy(Feature.Core | Feature.MultiPass | Feature.RcsTranslation | Feature.AutoStage)]
    public static readonly MethodInfo? Universe_ApplyVehicleSolvers =
        AccessTools.Method(typeof(Universe), nameof(Universe.ApplyVehicleSolvers), Type.EmptyTypes);

    // Stock acts on the end of an Auto burn here, and of an RCS burn too, because the RCS executor raises the same flags. Program.PrepareFrame calls it for every vehicle after InputEvents.ApplyInputEvents, and FlightComputer.EndAutoBurn removes the finished or stopped burn there whenever another burn follows it. A propellant stop with a later stage keeps the burn loaded, stops the warp and warns instead.
    // MultiPass and AutoRemove act on the outcome in a postfix, so they never remove a burn stock is about to remove as well.
    [UsedBy(Feature.Core | Feature.MultiPass | Feature.RcsTranslation | Feature.AutoRemove)]
    public static readonly MethodInfo? FlightComputer_RaisePendingAlerts =
        AccessTools.Method(typeof(FlightComputer), nameof(FlightComputer.RaisePendingAlerts), new[] { typeof(Vehicle) });

    // UncompressedSave is the concrete path that calls Universe.DeserializeSave, and its Id is the save-game discriminator the registries scope their entries by.
    [UsedBy(Feature.Core | Feature.MultiPass | Feature.RcsTranslation)]
    public static readonly MethodInfo? UncompressedSave_Load =
        AccessTools.Method(typeof(UncompressedSave), nameof(UncompressedSave.Load), Type.EmptyTypes);

    [UsedBy(Feature.Core | Feature.MultiPass | Feature.RcsTranslation)]
    public static readonly MethodInfo? UncompressedSave_Write =
        AccessTools.Method(typeof(UncompressedSave), nameof(UncompressedSave.Write), Type.EmptyTypes);

    // Registry entries drop with their vehicle, or a recycled vehicle id could pick up an orphaned execution.
    [UsedBy(Feature.Core | Feature.MultiPass | Feature.RcsTranslation | Feature.AutoStage)]
    public static readonly MethodInfo? Vehicle_Dispose =
        AccessTools.Method(typeof(Vehicle), nameof(Vehicle.Dispose), new[] { typeof(bool) });

    // ID-keyed registries must follow a successful vehicle rename.
    [UsedBy(Feature.Core | Feature.MultiPass | Feature.RcsTranslation)]
    public static readonly MethodInfo? Vehicle_SetName =
        AccessTools.Method(typeof(Vehicle), nameof(Vehicle.SetName), new[] { typeof(string) });

    #endregion

    #region AutoStage

    // The gauge button resolves its bound enum by Type.Name from this list, so the AUTOSTAGE button's own enum type is appended to it at immediate load.
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

    // Private setter.
    // The public SetActiveSequence rewrites Activated list-wide and resets caches, which stock's own activation does not.
    [UsedBy(Feature.AutoStage)]
    public static readonly PropertyInfo? SequenceList_ActiveSequence =
        AccessTools.Property(typeof(SequenceList), nameof(SequenceList.ActiveSequence));

    // Stock brackets its activation loop with this flag, and ResetCaches early-returns on it.
    [UsedBy(Feature.AutoStage)]
    public static readonly FieldInfo? SequenceList_updatingSequence =
        AccessTools.Field(typeof(SequenceList), "_updatingSequence");

    // Which settings page the nav rail has open, read by the shared Mods settings section host.
    // The enum is private to GameSettings, so its Mods member is resolved as a boxed value once and compared by equality.
    // Its own feature, so a settings-window rename costs the sections only.
    [UsedBy(Feature.SettingsPage)]
    public static readonly FieldInfo? GameSettings_openTab =
        AccessTools.Field(typeof(GameSettings), "_openTab");

    [UsedBy(Feature.SettingsPage)]
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

    // The gauge button keeps its bound enum private, and the RCS and AutoStage gauge patches need it to recognize their button instances.
    [UsedBy(Feature.RcsTranslation | Feature.AutoStage)]
    public static readonly FieldInfo? GaugeButtonFlightComputer_enumValue =
        AccessTools.Field(typeof(GaugeButtonFlightComputer), "_enumValue");

    // Read for every flight computer button in every frame, so it is a delegate instead of FieldInfo.GetValue.
    [UsedBy(Feature.AutoStage)]
    public static readonly AccessTools.FieldRef<GaugeButtonFlightComputer, Enum>? GaugeButtonFlightComputer_enumValueRef =
        InstanceFieldRef<GaugeButtonFlightComputer, Enum>(GaugeButtonFlightComputer_enumValue);

    // Private tooltip hook for the Auto button, replaced with the RCS explanation when a burn resolves to RCS execution.
    [UsedBy(Feature.RcsTranslation)]
    public static readonly MethodInfo? Vehicle_Hovered_BurnMode =
        AccessTools.Method(typeof(Vehicle), "Hovered", new[] { typeof(FlightComputerBurnMode) });

    // The flight burn editor draws through this static gauge-canvas host, which the RCS burn panel postfixes.
    [UsedBy(Feature.RcsTranslation)]
    public static readonly MethodInfo? BurnCanvasHost_Draw =
        AccessTools.Method(typeof(BurnCanvasHost), "Draw",
            new[] { typeof(GaugeCanvas), typeof(Brutal.Numerics.float2), typeof(Brutal.Numerics.float2) });

    #endregion

    #region Guidance

    // The per-vehicle guidance step runs as a prefix here.
    // Universe.PrepareVehicleWorkers calls it after InputEvents.ApplyInputEvents drained the player's input and before VehicleUpdateState.PrepareFromVehicle snapshots the flight computer and the manual inputs for the worker, so it is the last main-thread site before the snapshot and the only one after the drain.
    // The rest of the method still runs after the prefix and clears EngineOn while BurnMode is Auto, which is why guidance holds the mode in Manual.
    [UsedBy(Feature.Guidance)]
    public static readonly MethodInfo? Vehicle_PrepareWorker =
        AccessTools.Method(typeof(Vehicle), nameof(Vehicle.PrepareWorker), new[] { typeof(SimStep) });

    // Stock builds AttitudeTarget here and reads it in UpdateAttitudeError right after, so a postfix can add the commanded turning rate without changing where the craft points.
    // It is a private method with one call site, so an inlining attribute added on a game update would leave this handle resolving and the postfix silent. afc-guidance-driver checks that the rate arrives.
    [UsedBy(Feature.Guidance)]
    public static readonly MethodInfo? FlightComputer_UpdateAttitudeTarget =
        AccessTools.Method(typeof(FlightComputer), "UpdateAttitudeTarget",
            new[] { typeof(FlightComputerNavigation).MakeByRefType() });

    // Vehicle.SerializeSave calls this for the save record of the craft. A postfix gives the record the player's RCS mode while guidance holds RCS off. afc-guidance-landing-gimbal checks that it arrives.
    // A soft anchor, because guidance flies without the postfix and only a save made during a landing burn needs it.
    [UsedBy(Feature.SoftAnchor)]
    public static readonly MethodInfo? FlightComputer_SerializeSave =
        AccessTools.Method(typeof(FlightComputer), nameof(FlightComputer.SerializeSave), Type.EmptyTypes);

    // The engine master switch and throttle that the player's ignite and shutdown actions write.
    [UsedBy(Feature.Guidance)]
    public static readonly FieldInfo? Vehicle_manualControlInputs =
        AccessTools.Field(typeof(Vehicle), "_manualControlInputs");

    [UsedBy(Feature.Guidance)]
    public static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs>? Vehicle_manualControlInputsRef =
        InstanceFieldRef<Vehicle, ManualControlInputs>(Vehicle_manualControlInputs);

    #endregion

    #region Validation

    // The driver's only off switch is in this menu, so the driver validates the hook as well.
    [UsedBy(Feature.GuidanceDiagnostics | Feature.Guidance)]
    public static readonly MethodInfo? Program_DrawProgramMenusHook =
        AccessTools.Method(typeof(Program), nameof(Program.DrawProgramMenusHook), Type.EmptyTypes);

    public static bool ValidateGuidanceDiagnostics() => Validate(Feature.GuidanceDiagnostics);

    public static bool ValidateGuidance() => Validate(Feature.Guidance);

    public static bool ValidateCore() => Validate(Feature.Core);

    public static bool ValidateHyperbolicTargets() => Validate(Feature.HyperbolicTargets);

    public static bool ValidateMultiPass() => Validate(Feature.MultiPass);

    public static bool ValidateRcsTranslation() => Validate(Feature.RcsTranslation);

    public static bool ValidateAutoStage() => Validate(Feature.AutoStage);

    public static bool ValidateAutoRemove() => Validate(Feature.AutoRemove);

    public static bool ValidateSettingsPage() => Validate(Feature.SettingsPage);

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

    // "TransferPlanner_sourceBody" and "TransferPlanner_sourceBodyRef" both read as "TransferPlanner._sourceBody".
    // A method handle keeps its name, so "Vehicle_Hovered_BurnMode" reads as "Vehicle.Hovered_BurnMode".
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

    private static AccessTools.FieldRef<T, F>? InstanceFieldRef<T, F>(FieldInfo? field) where T : class
    {
        if (field == null)
            return null;
        try
        {
            return AccessTools.FieldRefAccess<T, F>(field);
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
