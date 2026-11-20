using AdvancedFlightComputer.Features.AeroControl;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The aero controller on KSAero's all-moving fins (Features/AeroControl), on the player's 2stage_KSAero: four 2 m fins on the
// booster. KSAero registers the fins with AFC's control-surface registry when its own load hook runs (KsaeroScope runs it), the
// aero sweep turns each fin in its private mesh to every 15 deg of its range and records what it does to the body table, and
// the controller reads those tables at a flight condition for the torque the fins can give and the deflections for a demand.
// The fins themselves are commanded through the registry and move at their rate. Skipped where the save or KSAero is missing.
public sealed class GuidanceAeroControlTest : AfcTest
{
    public override string Name => "afc-guidance-aero-control";

    private const string Save = "2stage_KSAero";

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
        if (!t.Check("KSAero registered its fins with AFC", ControlSurfaces.Any))
            return;

        SimDriver driver = t.Session.CreateDriver();
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        bool railsBefore = PhysicsBubble._forceOffRails;
        try
        {
            Vehicle craft;
            try
            {
                craft = VehicleSpawner.SpawnFromSave(Save, t.System, home, "HarnessAeroControl",
                    OrbitFixtures.CircularAt(home, 500_000.0, driver.Elapsed));
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"{Save}: {e.Message}");
                return;
            }
            PhysicsBubble._forceOffRails = true;
            driver.Step(0.05, 10);

            var surfaces = new List<ControlSurface>();
            ControlSurfaces.Collect(craft, surfaces);
            if (!t.Check($"the craft's fins are listed: {surfaces.Count}", surfaces.Count == 4))
                return;
            t.Check("every fin is at rest before anything commands it", surfaces.All(s => Math.Abs(s.DeflectionDeg) < 1e-9));

            // The sweep, with the fins.
            var clock = System.Diagnostics.Stopwatch.StartNew();
            if (!t.Check("the craft is swept", KsaAeroSweep.TryBuild(craft, home, Universe.GetElapsedSeconds(), out KsaAeroSweep.Result aero, out string error), error))
                return;
            t.Info($"sweep with fins: {clock.Elapsed.TotalMilliseconds:F0} ms, of which fins {aero.Surfaces.Sum(s => s.SweepMillis):F0} ms");
            if (!t.Check($"every fin has a table: {aero.Surfaces.Length}", aero.Surfaces.Length == 4))
                return;
            foreach (KsaAeroSweep.SurfaceTable table in aero.Surfaces)
            {
                t.Check($"'{table.Name}' swept at {string.Join("/", table.DeflectionDeg)} deg",
                    table.DeflectionDeg.SequenceEqual(new[] { -30.0, -15.0, 0.0, 15.0, 30.0 }));
                // At Mach 0.4, flying engine first: the fin's moment at its two stops, and at rest none.
                int zero = Array.IndexOf(table.DeflectionDeg, 0.0);
                table.Moment[zero].Coefficients(0.4, 0.0, 0.0, out double l0, out double m0, out double n0);
                table.Moment[0].Coefficients(0.4, 0.0, 0.0, out double ln, out double mn, out double nn);
                table.Moment[^1].Coefficients(0.4, 0.0, 0.0, out double lp, out double mp, out double np);
                double3 neg = new(ln, mn, nn), pos = new(lp, mp, np);
                t.Info($"'{table.Name}': dC_l/m/n at -30 deg {ln:0.00000}/{mn:0.00000}/{nn:0.00000}, at +30 deg {lp:0.00000}/{mp:0.00000}/{np:0.00000}");
                t.Check($"'{table.Name}': nothing at rest", Math.Abs(l0) + Math.Abs(m0) + Math.Abs(n0) == 0.0);
                t.Check($"'{table.Name}': a moment at its stops", pos.Length() > 1e-5 && neg.Length() > 1e-5, $"{pos.Length():0.00000}/{neg.Length():0.00000}");
                // Engine first at no angle a fin is alone in the flow either way it turns: the two stops turn the craft opposite ways.
                t.Check($"'{table.Name}': its stops turn the craft opposite ways", double3.Dot(pos, neg) < 0.0,
                    $"cos {double3.Dot(pos, neg) / Math.Max(pos.Length() * neg.Length(), 1e-30):F2}");
            }

            // On its own thread, in two passes: the body table first, which is what a caller waits on, then the fins into the same result.
            var jobClock = System.Diagnostics.Stopwatch.StartNew();
            if (t.Check("a sweep starts on its own thread", KsaAeroSweep.TryStart(craft, home, Universe.GetElapsedSeconds(), out KsaAeroSweep.Job job, out string startError), startError))
            {
                while (!job.Done && jobClock.Elapsed.TotalSeconds < 60.0)
                    Thread.Sleep(10);
                double bodyMs = jobClock.Elapsed.TotalMilliseconds;
                job.TryTake(out KsaAeroSweep.Result early, out string jobError);
                t.Check($"the body table comes first: {bodyMs:F0} ms, with {early?.Surfaces.Length ?? -1} fin tables yet",
                    early?.BodyAero != null && early.Surfaces.Length < 4, jobError);
                while (early != null && early.Surfaces.Length < 4 && jobClock.Elapsed.TotalSeconds < 120.0)
                    Thread.Sleep(20);
                t.Check($"the fins follow into the same result: {jobClock.Elapsed.TotalMilliseconds:F0} ms", early?.Surfaces.Length == 4);
                KsaAeroSweep.WriteLog();
            }

            // The controller at a landing burn's air, engine first: Mach 0.3, 6 kPa, the tables' own centre of mass, no limiter. The long axis is asmb x.
            double3 ax = new(1, 0, 0), ay = new(0, 1, 0), az = new(0, 0, 1);
            var used = new List<ControlSurface>();
            AeroTorqueAllocator? allocator = KsaAeroTorqueControl.Build(aero, surfaces, 0.3, 6000.0, 0.0, 0.0, 1.0, double3.Zero, ax, ay, az, used);
            if (!t.Check("the controller builds for the fins", allocator != null && used.Count == 4))
                return;
            string[] names = { "roll (x)", "pitch (y)", "yaw (z)" };
            var reach = new double[3];
            for (int a = 0; a < 3; a++)
            {
                AeroTorqueAllocator.AxisAuthority plus = allocator!.Characterise(a, 1), minus = allocator.Characterise(a, -1);
                reach[a] = Math.Min(plus.MaxTorqueNm, minus.MaxTorqueNm);
                t.Info($"{names[a]}: {plus.MaxTorqueNm / 1e3:F1} / {minus.MaxTorqueNm / 1e3:F1} kN m, side force {Math.Sqrt(plus.ForcePerNmX * plus.ForcePerNmX + plus.ForcePerNmY * plus.ForcePerNmY + plus.ForcePerNmZ * plus.ForcePerNmZ):F3} N per N m");
                t.Check($"{names[a]}: torque both ways", plus.MaxTorqueNm > 100.0 && minus.MaxTorqueNm > 100.0);
            }
            var deflection = new double[4];
            var even = new double[] { 1.0, 1.0, 1.0 };
            AeroTorqueAllocator.Result half = allocator!.Allocate(0.0, 0.5 * reach[1], 0.0, even, deflection);
            t.Check($"half the pitch reach is met: short {half.ShortfallNm:F1} N m of {0.5 * reach[1]:F0}, deflections {string.Join("/", deflection.Select(d => d.ToString("F1")))}",
                half.ShortfallNm < 0.02 * reach[1]);
            AeroTorqueAllocator.Result none = allocator.Allocate(0.0, 0.0, 0.0, even, deflection);
            t.Check($"no demand moves no fin: {string.Join("/", deflection.Select(d => d.ToString("G3")))}", none.Deflected == 0 && none.ShortfallNm < 1e-6);
            AeroTorqueAllocator? still = KsaAeroTorqueControl.Build(aero, surfaces, 0.3, 0.0, 0.0, 0.0, 1.0, double3.Zero, ax, ay, az, used);
            t.Check("no air, no torque", still == null || still.Characterise(1, 1).MaxTorqueNm < 1e-9);

            // The fins themselves: commanded through the registry, they move at their rate (30 deg/s), and go home on release.
            surfaces[0].Command(15.0);
            driver.Step(0.05, 20);
            t.Check($"a commanded fin reaches 15 deg in a second: {surfaces[0].DeflectionDeg:F1}", Math.Abs(surfaces[0].DeflectionDeg - 15.0) < 0.5);
            KsaAeroTorqueControl.Release(craft);
            driver.Step(0.05, 20);
            t.Check($"released, it goes home: {surfaces[0].DeflectionDeg:F1}", Math.Abs(surfaces[0].DeflectionDeg) < 0.5);
        }
        finally
        {
            PhysicsBubble._forceOffRails = railsBefore;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }
}
