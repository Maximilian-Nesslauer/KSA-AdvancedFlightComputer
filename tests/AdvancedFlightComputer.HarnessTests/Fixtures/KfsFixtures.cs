using HeadlessHarness.Harness;
using KSA;
using KSA.Kfs;
using KSA.Kfs.Host;
using KSA.Kfs.Host.Workbench;

namespace AdvancedFlightComputer.HarnessTests.Fixtures;

internal static class KfsFixtures
{
    internal static KfsVehicleHost QueuePilot(Vehicle vehicle)
    {
        var host = KfsVehicleHost.For(vehicle);
        var bench = new Workbench(KfsIntegration.Tables, Path.GetTempPath(), TextWriter.Null)
        {
            ActiveVehicle = () => host,
        };
        const string program = "#pragma kfs tick_hz 10\n#pragma kfs task step rate_hz=10\nvoid step(void) {}\n";
        var build = bench.BuildText("afc-pilot.c", program, host.Computer.Part, host.Layout);
        if (!build.Ok)
            throw new InvalidOperationException("kFS pilot build failed: " + string.Join("; ", build.Diagnostics));
        host.Computer.Enqueue(computer =>
        {
            var loaded = computer.Load(build.Image!, host.Layout);
            if (!loaded.Ok)
                throw new InvalidOperationException("kFS pilot load failed: " + loaded);
        });
        if (!bench.SelectTarget("vehicle"))
            throw new InvalidOperationException("kFS workbench could not select the test vehicle.");
        bench.Start();
        return host;
    }

    internal static DispatchResult DispatchPilot(KfsVehicleHost host)
        => host.Computer.Dispatch((long)Universe.GetElapsedTime().Nanoseconds,
            () => new HostSample(NavTruth.Zero, FlightDataValues.Zero, VehiclePropsValues.Empty(host.Thrusters.Length)));

    internal static void LoseControl(Vehicle vehicle, SimDriver driver)
    {
        vehicle.EndAllCrewMissions();
        foreach (Control control in vehicle.Parts.Modules.Get<Control>())
            foreach (PowerConsumer consumer in control.Parent.Modules.Get<PowerConsumer>())
                consumer.Mode = PowerSwitchMode.Off;
        driver.Step(0.02, 2);
    }
}
