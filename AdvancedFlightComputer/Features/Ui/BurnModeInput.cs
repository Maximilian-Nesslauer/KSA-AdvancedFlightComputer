using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.RcsTranslation;
using KSA;

namespace AdvancedFlightComputer.Features.Ui;

/// <summary>The node commands of the console, sent through the stock input buffers that InputEvents.ApplyInputEvents drains with the solvers joined.</summary>
internal static class BurnModeInput
{
    /// <summary>Flies the loaded node of the controlled craft: stock AUTO, or the RCS executor when the node resolves to RCS.</summary>
    public static void FlyWithAuto(Vehicle vehicle)
    {
        if (!ReferenceEquals(vehicle, Program.ControlledVehicle))
            return;
        InputEvents.FlightComputerInputBuffer.Add(new InputEvents.FlightComputerInputData
        {
            Vehicle = vehicle,
            Toggle = false,
            EnumValue = FlightComputerBurnMode.Auto,
        });
    }

    /// <summary>
    /// Why flying a node with stock AUTO is refused, or empty. While guidance holds the craft, the Auto burn mode counts as
    /// a player takeover of the engine and releases guidance (GuidanceWindow.ApplyAutopilot), so the verb
    /// is refused instead of stopping the running mode without a word.
    /// </summary>
    public static string FlyRefusal(Vehicle vehicle)
        => VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.Guidance ? "Guidance flies this craft" : "";

    /// <summary>
    /// Why flying the next node is refused, or empty. Every place that offers the verb for the next node
    /// asks this, so RCS, ORBIT, TRANSFER and the head drawer refuse the same node for the same reason.
    /// </summary>
    public static string FlyNextRefusal(Vehicle vehicle)
    {
        if (SharedVehicleHooks.RcsEnabled && NextNode(vehicle) is Burn node
            && RcsExecutor.ResolveMode(vehicle, OptionsOf(vehicle, node)) == RcsExecutionMode.Rcs)
        {
            string rcs = RcsRefusal(vehicle, node);
            if (rcs.Length > 0)
                return rcs;
        }
        return FlyRefusal(vehicle);
    }

    /// <summary>Why the RCS executor cannot fly <paramref name="node"/>, or empty. The estimate uses the current craft and the node's attitude option.</summary>
    public static string RcsRefusal(Vehicle vehicle, Burn node)
    {
        RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec);
        RcsBurnOptions? options = exec?.FindOptions(node.Time.Seconds(), node.DeltaVVlf.Length());
        bool rcsOnly = options?.Mode == RcsExecutionMode.Rcs;
        if (!CanTranslate(vehicle))
            return RcsRefusal(false, rcsOnly, false, 0.0, 0.0);
        bool estimates = RcsBurnPreview.TryGetEstimates(node, vehicle, vehicle.FlightComputer, exec, out RcsEstimates est, out _);
        double need = estimates ? est.RequiredPropellantKg(options?.Attitude ?? RcsAttitudeStrategy.Auto) : 0.0;
        return RcsRefusal(true, rcsOnly, estimates, need, RcsExecutor.AvailablePropellantCached(vehicle));
    }

    public static string RcsRefusal(bool canTranslate, bool rcsOnly, bool estimates, double needKg, double availableKg)
        => !canTranslate ? (rcsOnly ? "No RCS translation" : "No usable engine, no RCS translation")
            : estimates && needKg > availableKg ? "Propellant short"
            : "";

    public static bool CanTranslate(Vehicle vehicle)
        => SharedVehicleHooks.RcsEnabled && vehicle.Parts.Modules.Get<ThrusterController>().Length > 0
           && RcsExecutor.ProbeCached(vehicle).HasAnyTranslation;

    private static RcsBurnOptions? OptionsOf(Vehicle vehicle, Burn node)
    {
        RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec);
        return exec?.FindOptions(node.Time.Seconds(), node.DeltaVVlf.Length());
    }

    /// <summary>Sets the burn mode back to MANUAL. The node stays in the plan.</summary>
    public static void StopAuto(Vehicle vehicle)
    {
        if (!ReferenceEquals(vehicle, Program.ControlledVehicle))
            return;
        InputEvents.FlightComputerInputBuffer.Add(new InputEvents.FlightComputerInputData
        {
            Vehicle = vehicle,
            Toggle = false,
            EnumValue = FlightComputerBurnMode.Manual,
        });
    }

    /// <summary>Whether WARP of the stock BURN gauge works now on the controlled craft: Vehicle.IsFlightComputerDisabled holds it for a craft out of control and from the ignition on.</summary>
    public static bool WarpOffered(Vehicle vehicle)
        => ReferenceEquals(vehicle, Program.ControlledVehicle) && !vehicle.IsFlightComputerDisabled(FlightComputerAction.WarpToNextBurn);

    /// <summary>The stock warp to the ignition of the loaded burn runs now.</summary>
    public static bool WarpingToBurn(Vehicle vehicle)
        => Universe.IsAutoWarpActive && vehicle.FlightComputer.Burn is BurnTarget burn && Universe.AutoWarpTime == burn.IgnitionTime;

    /// <summary>The same event the WARP button of the stock BURN gauge sends in GaugeButtonFlightComputer.OnReleased.</summary>
    public static void ToggleWarpToBurn(Vehicle vehicle)
    {
        if (!ReferenceEquals(vehicle, Program.ControlledVehicle))
            return;
        InputEvents.FlightComputerInputBuffer.Add(new InputEvents.FlightComputerInputData
        {
            Vehicle = vehicle,
            Toggle = true,
            EnumValue = FlightComputerAction.WarpToNextBurn,
        });
    }

    public static void DeleteNode(Vehicle vehicle, Burn burn)
    {
        if (vehicle?.FlightComputer == null || burn == null)
            return;
        InputEvents.BurnUpdateBuffer.Add(new InputEvents.BurnUpdateData
        {
            Burn = burn,
            FlightComputer = vehicle.FlightComputer,
            DeleteBurn = true,
        });
    }

    /// <summary>The node the flight computer flies next, or null.</summary>
    public static Burn? NextNode(Vehicle vehicle) => vehicle?.FlightComputer?.BurnPlan.FindFirstExecutableBurn();

    /// <summary>Stock AUTO flies a loaded node, and the RCS executor does not hold it.</summary>
    public static bool StockAutoOnNode(Vehicle vehicle)
    {
        FlightComputer? fc = vehicle?.FlightComputer;
        return fc != null && fc.BurnMode == FlightComputerBurnMode.Auto && fc.Burn != null
               && VehicleControlOwnership.HolderOf(vehicle!) != ControlClaimant.RcsTranslation;
    }

    /// <summary>The seconds to the ignition of <paramref name="node"/>, which stock times half a burn before the node (FlightComputer.TimeBurn).</summary>
    public static double SecondsToIgnition(Vehicle vehicle, Burn node)
    {
        UniverseTime now = Universe.GetElapsedTime();
        return vehicle.FlightComputer?.Burn is BurnTarget loaded && StockBurnIdentity.IsLoaded(loaded, node) && !loaded.IgnitionTime.IsEndOfTime()
            ? (loaded.IgnitionTime - now).Seconds()
            : (node.Time - now).Seconds();
    }

    public static bool BurnFlies(Vehicle vehicle)
        => StockAutoOnNode(vehicle) || VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.RcsTranslation;

    public static bool ResolvesToRcs(Vehicle vehicle)
    {
        if (!SharedVehicleHooks.RcsEnabled || NextNode(vehicle) is not Burn burn)
            return false;
        RcsBurnOptions? options = OptionsOf(vehicle, burn);
        return Logic.BurnList.RcsTakes(false, options?.Mode == RcsExecutionMode.Rcs,
            RcsExecutor.ResolveMode(vehicle, options) == RcsExecutionMode.Rcs, CanTranslate(vehicle));
    }
}
