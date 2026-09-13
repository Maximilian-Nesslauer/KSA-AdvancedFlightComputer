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
    /// </summary>
    internal static bool GiveBackAuto(Vehicle vehicle, BurnTarget? heldTarget)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (fc.BurnMode != FlightComputerBurnMode.Manual || !ReferenceEquals(fc.Burn, heldTarget)
            || !vehicle.IsControllable)
            return false;
        Write(vehicle, FlightComputerBurnMode.Auto);
        return true;
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
