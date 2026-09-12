using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.HarnessTests.Framework;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

internal static class AutoStageFlightSupport
{
    // Near vacuum, the same spawn the harness flight test uses.
    public const double SpawnAltitudeM = 500_000.0;

    // The save from the game's Vehicles folder, on a circular orbit above the home body.
    public static Vehicle SpawnFromSave(TestContext t, string saveId, string id, out Astronomical homeBody)
    {
        CelestialSystem system = t.System;
        if (system.HomeBody is not IParentBody home || home is not Astronomical body)
            throw new InvalidOperationException("the loaded system has no home body to orbit.");
        homeBody = body;
        Orbit orbit = VehicleSpawner.CircularCci(home, body.MeanRadius + SpawnAltitudeM, Universe.GetElapsedTime());
        return VehicleSpawner.SpawnFromSave(saveId, system, home, id, orbit);
    }

    // Manual throttle with the flight computer holding prograde, so the burn only raises the orbit.
    // The g-load throttle cap of an auto burn does not exist on a manual one, so a test that flies
    // through several stagings has to throttle back or the game destroys the vehicle.
    public static void HoldPrograde(Vehicle vehicle, float throttle = 1f)
    {
        vehicle.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
        vehicle.FlightComputer.TrackTarget(FlightComputerAttitudeTrackTarget.Prograde);
        TestSupport.SetManualControlInputs(vehicle, throttle, engineOn: true);
    }

    // Staging is edge-triggered on propellant depletion, so the first stage is lit the way a
    // player lights it. The activation lands through the input queue on the next step.
    public static void IgniteFirstStage(Vehicle vehicle, SimDriver driver)
    {
        if (StagingHelpers.HasActiveEngineWithPropellant(vehicle))
            return;
        vehicle.Parts.SequenceList.ActivateNextSequence(vehicle);
        driver.Step(1.0);
    }

    // Through the gauge path, so the toggle patch is part of every flight.
    public static bool Arm(TestContext t, Vehicle vehicle)
    {
        vehicle.ToggleEnum(AfcAutoStageToggle.Enabled);
        return t.Check("ToggleEnum(AfcAutoStageToggle) arms the detector", StagingDetector.Active);
    }

    public static void CleanupAfterFlight(CelestialSystem system, HashSet<string> preexisting)
    {
        Program.ControlledVehicle = null;
        PhysicsBubble._forceOffRails = false;
        TestSupport.DespawnNewVehicles(system, preexisting);
    }
}
