using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// LOCAL DIAGNOSTIC: for each choice of a craft's engines, the moment they make about the centre of mass with the gimbals at rest,
// and what their gimbals can hold against it, measured four ways: the landing check's (KsaThrustMoment.LateralAgainstGimbals,
// through KsaTvcAllocator.Solve), the allocator's own per-axis caps, the min-norm allocation along the needed direction with no
// per-axis cap, and the main chambers alone. Prints only; it fails nothing.
public sealed class GuidanceEngineMomentDiagTest : AfcTest
{
    public override string Name => "afc-guidance-engine-moment-diag";

    private static readonly string[] DefaultSaves = { "2stage_KSAero_Gimbal10", "2stage_KSAero" };

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(DefaultSaves);
        if (saves.Count == 0)
        {
            t.Skip($"none of '{string.Join("', '", DefaultSaves)}' is in the game's Vehicles folder.");
            return;
        }

        foreach (string save in saves)
        {
            HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
            try
            {
                Vehicle craft;
                try
                {
                    craft = VehicleSpawner.SpawnFromSave(save, t.System, home, "HarnessMomentDiag",
                        OrbitFixtures.CircularAt(home, 500_000.0, Universe.GetElapsedTime()));
                }
                catch (InvalidOperationException e)
                {
                    t.Skip($"'{save}': {e.Message}");
                    continue;
                }
                craft.Parts.EnsureDerived(DerivedData.All);
                Measure(t, save, craft);
            }
            finally
            {
                TestSupport.DespawnNewVehicles(t.System, preexisting);
            }
        }
        t.Check("diagnostic ran", true);
    }

    private static void Measure(TestContext t, string save, Vehicle craft)
    {
        // The three engines of the booster's triple engine plate.
        EngineController[] all = craft.Parts.Modules.Get<EngineController>().ToArray();
        EngineController[] booster = all.Where(e => e.Parent.FullPart.TreeParent?.Template.Id.Contains("Triple") == true).ToArray();
        float3 c = craft.CenterOfMassAsmbF;
        t.Info($"'{save}': {all.Length} controllers, booster set {booster.Length} x {booster.FirstOrDefault()?.Parent.FullPart.Template.Id}, CoM {c.X:F2} {c.Y:F2} {c.Z:F2}");
        if (booster.Length == 0)
            return;
        foreach (GimbalController gc in craft.Parts.Modules.Get<GimbalController>())
            if (gc.Data.Nozzles.Count > 0 && gc.Data.Nozzles[0].Rocket?.Controller is EngineController owner && booster.Contains(owner))
            {
                double3 p = KsaTvcAllocator.GimbalPivot(gc);
                t.Info($"  gimbal {gc.InstanceId} of {owner.Parent.FullPart.InstanceId}: {gc.Data.Nozzles.Count} nozzle(s), max thrust {gc.Data.MaximumThrust / 1e3:F1} kN, angles {gc.Gimbal.AxisY.MaxAngle * 180 / Math.PI:F1}/{gc.Gimbal.AxisZ.MaxAngle * 180 / Math.PI:F1} deg, pivot {p.X:F2} {p.Y:F2} {p.Z:F2}");
            }

        for (int mask = 1; mask < 1 << booster.Length; mask++)
        {
            EngineController[] set = booster.Where((_, i) => (mask & (1 << i)) != 0).ToArray();
            string name = string.Join("+", set.Select(e => e.Parent.FullPart.InstanceId));
            (double3 moment, double3 force) = KsaThrustMoment.Rest(craft, set);
            double3 axis = force / Math.Max(force.Length(), 1e-9);
            double3 lateral = moment - axis * double3.Dot(moment, axis);
            (double lateralNm, double checkNm) = KsaThrustMoment.LateralAgainstGimbals(craft, set);

            List<GimbalController> gimbals = GimbalsOf(craft, set, mainOnly: false);
            List<GimbalController> mains = GimbalsOf(craft, set, mainOnly: true);
            double3 against = lateralNm > 1e-6 ? -lateral / lateralNm : new double3(0, 1, 0);
            double free = MinNormAlong(gimbals, c, against, out string saturating);
            double mainFree = MinNormAlong(mains, c, against, out _);

            var thrusts = gimbals.Select(g => (double)g.Data.MaximumThrust).ToArray();
            var commands = new double[2 * gimbals.Count];
            TvcAllocationResult r = KsaTvcAllocator.Solve(gimbals.ToArray(), thrusts, c, against * 1e9, commands);
            t.Info($"  {name}: thrust {force.Length() / 1e3:F0} kN, lateral moment {lateralNm / 1e3:F0} kN m (per N {lateralNm / force.Length():F3} m), direction {against.X:F2} {against.Y:F2} {against.Z:F2}");
            t.Info($"    check (Solve) {checkNm / 1e3:F0} kN m = {lateralNm / Math.Max(checkNm, 1):P0}; per-axis caps {r.MaxTorque.X / 1e3:F0} {r.MaxTorque.Y / 1e3:F0} {r.MaxTorque.Z / 1e3:F0} kN m; saturation scale {r.SaturationScale:F3}, weak share {r.WeakAxisScale:F3}");
            t.Info($"    min-norm along the direction, no per-axis cap: {free / 1e3:F0} kN m = {lateralNm / Math.Max(free, 1):P0} (first to saturate: {saturating}); main chambers alone {mainFree / 1e3:F0} kN m = {lateralNm / Math.Max(mainFree, 1):P0}");
        }
    }

    private static List<GimbalController> GimbalsOf(Vehicle craft, EngineController[] set, bool mainOnly)
    {
        var list = new List<GimbalController>();
        foreach (GimbalController gc in craft.Parts.Modules.Get<GimbalController>())
            foreach (RocketNozzle nozzle in gc.Data.Nozzles)
                if (nozzle.Rocket?.Controller is EngineController owner && set.Contains(owner)
                    && (!mainOnly || gc.Gimbal.AxisZ.MaxAngle > 0f))
                {
                    list.Add(gc);
                    break;
                }
        return list;
    }

    // The min-norm allocation of a unit torque along the direction, scaled until the first gimbal reaches its range: the torque it gives along the direction.
    private static double MinNormAlong(List<GimbalController> gimbals, float3 com, double3 d, out string saturating)
    {
        saturating = "none";
        int cols = 2 * gimbals.Count;
        if (cols == 0)
            return 0.0;
        var bT = new double[3 * cols];
        var bF = new double[3 * cols];
        for (int i = 0; i < gimbals.Count; i++)
            KsaTvcAllocator.FillColumns(gimbals[i], com, gimbals[i].Data.MaximumThrust, bT, bF, i);
        var g = new double[9];
        for (int k = 0; k < cols; k++)
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 3; b++)
                    g[3 * a + b] += bT[3 * k + a] * bT[3 * k + b];
        double trace = g[0] + g[4] + g[8];
        g[0] += 1e-6 * trace; g[4] += 1e-6 * trace; g[8] += 1e-6 * trace;
        double[] inv = Invert(g);
        double wx = inv[0] * d.X + inv[1] * d.Y + inv[2] * d.Z;
        double wy = inv[3] * d.X + inv[4] * d.Y + inv[5] * d.Z;
        double wz = inv[6] * d.X + inv[7] * d.Y + inv[8] * d.Z;
        double peak = 0.0;
        double3 torque = default;
        for (int k = 0; k < cols; k++)
        {
            double u = bT[3 * k] * wx + bT[3 * k + 1] * wy + bT[3 * k + 2] * wz;
            if (Math.Abs(u) > peak)
            {
                peak = Math.Abs(u);
                saturating = $"gimbal {gimbals[k / 2].InstanceId} axis {k % 2}";
            }
            torque += new double3(bT[3 * k], bT[3 * k + 1], bT[3 * k + 2]) * u;
        }
        return peak > 0.0 ? double3.Dot(torque, d) / peak : 0.0;
    }

    private static double[] Invert(double[] m)
    {
        double a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5], g = m[6], h = m[7], i = m[8];
        double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        return
        [
            (e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det,
            (f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det,
            (d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det,
        ];
    }
}
