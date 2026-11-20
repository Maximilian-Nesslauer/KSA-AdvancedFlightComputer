using AdvancedFlightComputer.Features.AeroControl;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Diagnostic, 2026-11-20 flight 195001: fin 4 sat on its -30 deg stop for 90 % of the glide while fin 2 barely moved. Each fin's
// torque at the flown condition (Mach 0.36, 6.9 kPa, 2.3 deg off engine first, direction unknown, so eight directions), in the
// model axes the controller allocates in, and the allocator on the flight's mean demand.
public sealed class GuidanceFinCheckTest : AfcTest
{
    public override string Name => "afc-guidance-fin-check";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        using KsaeroScope ksaero = KsaeroScope.Apply();
        if (!ksaero.Active)
        {
            t.Skip($"KSAero is not loaded ({ksaero.Status}).");
            return;
        }
        SimDriver driver = t.Session.CreateDriver();
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        bool railsBefore = PhysicsBubble._forceOffRails;
        try
        {
            Vehicle craft = VehicleSpawner.SpawnFromSave("2stage_KSAero", t.System, home, "HarnessFinCheck",
                OrbitFixtures.CircularAt(home, 500_000.0, driver.Elapsed));
            PhysicsBubble._forceOffRails = true;
            driver.Step(0.05, 10);
            var surfaces = new List<ControlSurface>();
            ControlSurfaces.Collect(craft, surfaces);
            if (!t.Check("the craft is swept", KsaAeroSweep.TryBuild(craft, home, Universe.GetElapsedSeconds(), out KsaAeroSweep.Result aero, out string error), error))
                return;
            KsaFrameBridge.BodyAxes(craft, out double3 mx, out double3 my, out double3 mz);
            t.Info($"model axes in asmb: x {mx}, y {my}, z {mz}");
            for (int i = 0; i < surfaces.Count; i++)
                t.Info($"fin{i + 1} '{surfaces[i].Name}' at {surfaces[i].Part.PositionVehicleAsmb}, range {surfaces[i].MinDeg}..{surfaces[i].MaxDeg}, {surfaces[i].RateDegS} deg/s");

            {
                // The aero-control test's condition, dumped for an offline repro: Mach 0.3, 6 kPa, engine first, no limiter, asmb axes.
                var dumpUsed = new List<ControlSurface>();
                AeroTorqueAllocator? dump = KsaAeroTorqueControl.Build(aero, surfaces, 0.3, 6000.0, 0.0, 0.0, 1.0, double3.Zero,
                    new double3(1, 0, 0), new double3(0, 1, 0), new double3(0, 0, 1), dumpUsed);
                for (int i = 0; dump != null && i < dump.Count; i++)
                {
                    var parts = new List<string>();
                    foreach (double d in new[] { -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0 })
                    {
                        dump.At(i, d, out double tx, out double ty, out double tz, out _, out _, out _);
                        parts.Add($"{tx:F1}, {ty:F1}, {tz:F1}");
                    }
                    t.Info($"DUMP fin{i}: {string.Join(", ", parts)}");
                }
            }
            const double Mach = 0.362, Q = 6940.0, Divisor = 1.0 + 0.114e-3 * 6940.0, Off = 2.3 * Math.PI / 180.0;
            var used = new List<ControlSurface>();
            for (int h = -1; h < 8; h++)
            {
                double alpha = h < 0 ? 0.0 : Off * Math.Cos(h * Math.PI / 4), beta = h < 0 ? 0.0 : Off * Math.Sin(h * Math.PI / 4);
                AeroTorqueAllocator? a = KsaAeroTorqueControl.Build(aero, surfaces, Mach, Q, alpha, beta, Divisor, double3.Zero, mx, my, mz, used);
                if (a == null)
                {
                    t.Info("no allocator");
                    return;
                }
                t.Info($"alpha {alpha * 180 / Math.PI:F2} beta {beta * 180 / Math.PI:F2} deg: torque in model x/y/z, kN m");
                for (int i = 0; i < a.Count; i++)
                {
                    string row = $"  fin{i + 1}:";
                    foreach (double d in new[] { -30.0, -15.0, 15.0, 30.0 })
                    {
                        a.At(i, d, out double tx, out double ty, out double tz, out _, out _, out _);
                        row += $"  {d,4:F0}: {tx / 1e3,6:F1} {ty / 1e3,6:F1} {tz / 1e3,5:F1}";
                    }
                    t.Info(row);
                }
                if (h >= 0)
                    continue;
                // The flight's mean demand, model axes, from rest, without and with the flight's deflections as the start.
                var priority = new double[] { 1.0, 1.0, 0.3 };
                var d0 = new double[a.Count];
                AeroTorqueAllocator.Result r = a.Allocate(21_600.0, -42_200.0, 7_000.0, priority, d0);
                t.Info($"mean demand 21.6/-42.2/7.0 kN m from rest: deflections {string.Join("/", d0.Select(x => x.ToString("F1")))}, gives {r.TorqueX / 1e3:F1}/{r.TorqueY / 1e3:F1}/{r.TorqueZ / 1e3:F1}, short {r.ShortfallNm / 1e3:F1} kN m");
                for (int i = 0; i < a.Count; i++)
                {
                    var probe = new double[a.Count];
                    probe[i] = 30.0;
                    AeroTorqueAllocator.Result p1 = a.Evaluate(probe, 0, 0, 0);
                    probe[i] = -30.0;
                    AeroTorqueAllocator.Result p2 = a.Evaluate(probe, 0, 0, 0);
                    t.Info($"  fin{i + 1} alone +30: {p1.TorqueX / 1e3:F1}/{p1.TorqueY / 1e3:F1}/{p1.TorqueZ / 1e3:F1}, -30: {p2.TorqueX / 1e3:F1}/{p2.TorqueY / 1e3:F1}/{p2.TorqueZ / 1e3:F1} kN m");
                }
            }
        }
        finally
        {
            PhysicsBubble._forceOffRails = railsBefore;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }
}
