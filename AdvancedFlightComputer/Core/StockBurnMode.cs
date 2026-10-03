using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Where AFC's controllers take stock's burn mode and give it back. A controller that flies a craft holds the mode in Manual, because stock Auto takes the attitude near ignition and <c>Vehicle.PrepareWorker</c> clears the engine switch on every step while it is set. An Auto the hold replaced is given back when the controller lets go.
/// The writes mirror what <c>Vehicle.SetEnum</c> does for the burn mode, the navball frame and the refusal on a craft without control included, but they do not go through it, because AFC intercepts that path to start and cancel RCS burns.
/// </summary>
internal static class StockBurnMode
{
    /// <summary>Selects Manual and returns true when the mode was Auto, so the caller knows to give it back.</summary>
    internal static bool HoldManual(Vehicle vehicle)
    {
        if (vehicle.FlightComputer.BurnMode != FlightComputerBurnMode.Auto)
            return false;
        Write(vehicle, FlightComputerBurnMode.Manual);
        return true;
    }

    /// <summary>
    /// Gives an Auto back while the mode still reads Manual and <paramref name="heldTarget"/> is still the loaded burn target. Stock writes Manual itself when a burn is loaded, unloaded or completed, and Auto on a burn the player never armed would start the stock autopilot on it.
    /// A caller that cannot keep the target, because its hold outlives a save, passes the loaded one.
    /// Stock writes Manual too when a started Auto burn finds every active engine dry, and raises <c>FlightComputer.AutoBurnStoppedOutOfPropellant</c>, after which <c>FlightComputer.RaisePendingAlerts</c> ends the burn and removes it when another burn follows. An Auto given back to such a burn would stop on the next worker step, so it is not given back: the burn stays planned in Manual and stock's own stop warning says why.
    /// Only a caller that takes that stop back itself on every frame, as AutoStage does while new engines light, passes <paramref name="takesStopBack"/>.
    /// </summary>
    internal static bool GiveBackAuto(Vehicle vehicle, BurnTarget? heldTarget, bool takesStopBack = false)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (fc.BurnMode != FlightComputerBurnMode.Manual || heldTarget == null || !ReferenceEquals(fc.Burn, heldTarget)
            || !vehicle.IsControllable)
            return false;
        if (!takesStopBack && WouldStopAtOnce(vehicle, heldTarget))
        {
            TimedAlert.CreateWarning(ReferenceEquals(vehicle, Program.ControlledVehicle)
                ? LStrings.AlertAutoBurnStopped.Format()
                : LStrings.AlertAutoBurnStoppedVehicle.Format<string>(vehicle.Id));
            return false;
        }
        Write(vehicle, FlightComputerBurnMode.Auto);
        return true;
    }

    /// <summary>
    /// The propellant stop of <c>FlightComputer.ComputeControl</c>, read on the committed engine states the next worker step starts from: the burn has started or reached its ignition time, at least one engine is active, and no active engine reports propellant.
    /// </summary>
    internal static bool WouldStopAtOnce(Vehicle vehicle, BurnTarget target)
    {
        if (!target.Throttle.HasValue && Universe.GetElapsedTime() < target.IgnitionTime)
            return false;
        if (vehicle.Parts?.States == null
            || !ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>
                .TryGetFrom(vehicle.Parts.States, out var engines))
            return false;
        bool anyActive = false;
        foreach (var engine in engines.ModulesAndStates)
        {
            if (!engine.Module.IsActive)
                continue;
            if (engine.State.IsPropellantAvailable)
                return false;
            anyActive = true;
        }
        return anyActive;
    }

    // Vehicle.SetEnum pairs the mode with the navball frame, BurnBody for Auto and the vehicle region's own frame otherwise.
    private static void Write(Vehicle vehicle, FlightComputerBurnMode mode)
    {
        vehicle.SetNavBallFrame(mode == FlightComputerBurnMode.Auto
            ? VehicleReferenceFrame.BurnBody
            : vehicle.VehicleRegion.GetVehicleReferenceFrame());
        vehicle.FlightComputer.BurnMode = mode;
    }
}
