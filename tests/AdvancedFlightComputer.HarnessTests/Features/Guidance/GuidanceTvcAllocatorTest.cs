using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The oracle is the stock thrust geometry: TvcAllocation.ComputeJacobian for the allocator columns
// and TvcAllocation.ComputeTorqueAsmb for the torque a command delivers. Both turn the thrust point
// about the gimbal pivot. The synthetic gimbals are never registered with the simulation. The
// spawned craft checks the same columns on real engines, that the 6-DOF model arm gives the
// allocator's lateral gain, and that an engine off the gimbals stays in the thrust the model's
// torque is taken per newton of.
public sealed class GuidanceTvcAllocatorTest : AfcTest
{
    public override string Name => "afc-guidance-tvc-allocator";

    private static readonly string[] DefaultSaves = { "Test Vehicle 2", "Test Vehicle 1" };

    private const float Thrust = 1.0e6f;
    private const float MaxAngle = 0.105f;
    private const float PivotArm = 8f;
    private const float NozzleOffset = 2f;

    // The central difference over half the gimbal range costs (MaxAngle/2)^2/6 of slope, about 5e-4 here.
    private const double ColumnTolerance = 1e-3;

    protected override void Execute(TestContext t)
    {
        ColumnsUseThePivotArm(t);
        DeliveredTorqueMatchesTheDemand(t);
        ColumnsCoverALateralNozzleOffset(t);
        RealEnginesMatchStock(t);
        EnginesOffTheGimbalsCountInTheModelThrust(t);
        TheRollModelTakesACraftWithoutRollTorque(t);
        ASaturatedGimbalLeavesTheRestToTheOthers(t);
        TheCoupledRollGoesWithTheLateralTorque(t);
    }

    // One main gimbal 1 m off the axis, and a gimbal with a hundredth of its thrust beside it that stands for the verniers. A sideways deflection of the main one turns the craft about Y and rolls it by 1/PivotArm of that. Asked for Y torque at zero roll, the small gimbal has to take the roll out and its stops end the allocation early; asked with that roll named as coupled, the main gimbal gives its full travel.
    private static void TheCoupledRollGoesWithTheLateralTorque(TestContext t)
    {
        var com = new float3(0f, 0f, 0f);
        GimbalController main = Synthetic(floatQuat.Identity, new float3(-PivotArm, 1f, 0f), new float3(-NozzleOffset, 0f, 0f));
        GimbalController small = Synthetic(floatQuat.Identity, new float3(-PivotArm, 0f, 1.2f), new float3(-NozzleOffset, 0f, 0f));
        GimbalController[] gimbals = [main, small];
        double[] thrusts = [Thrust, 0.01 * Thrust];
        Columns(main, com, Thrust, out double3[] mainTorque, out _);
        // The main gimbal's column about Y, with the roll that comes with it.
        double3 column = Math.Abs(mainTorque[0].Y) > Math.Abs(mainTorque[1].Y) ? mainTorque[0] : mainTorque[1];
        double full = Math.Abs(column.Y), rollPerY = column.X / column.Y;
        double size = 10.0 * full;
        var unit = new double3(0, 1, 0);

        // The zero-roll figure the plan's cone was sized with: the min-norm solution scaled to its first stop.
        var zeroRoll = new double[4];
        TvcAllocationResult held = KsaTvcAllocator.Solve(gimbals, thrusts, com, unit * size, zeroRoll, redistribute: false);
        var coupledCommands = new double[4];
        var roll = new double3(rollPerY * size, 0, 0);
        TvcAllocationResult coupled = KsaTvcAllocator.Solve(gimbals, thrusts, com, unit * size + roll, coupledCommands, coupled: roll);

        double heldGot = held.AchievedTorque.Y, coupledGot = coupled.AchievedTorque.Y;
        t.Check("at zero roll the small gimbal's stops end the allocation early", heldGot < 0.5 * full,
            $"{heldGot:E3} of {full:E3} N m");
        int mainAxis = Math.Abs(mainTorque[0].Y) > Math.Abs(mainTorque[1].Y) ? 0 : 1;
        t.Check("with its roll named as coupled, the main gimbal goes to its stop", Math.Abs(coupledCommands[mainAxis]) >= 1.0 - 1e-6 && coupledGot >= (1.0 - 2e-3) * full,
            $"main command {coupledCommands[mainAxis]:F4}, {coupledGot:E3} of the main gimbal's {full:E3} N m");
        t.CheckRel("and the roll is the one that comes with it", coupled.AchievedTorque.X, rollPerY * coupledGot, 2e-3, 1.0);
        t.Check("every command stays inside its stop", coupledCommands.All(c => Math.Abs(c) <= 1.0 + 1e-9),
            string.Join(", ", coupledCommands.Select(c => c.ToString("F4"))));
    }

    // Two gimbals on the centre line, one with twice the other's range: the min-norm solution gives the wide one the larger command, so it reaches its stop first. Scaled to that, the narrow one gives only half of what it has; redistributed, it carries on to its own stop and the craft gets both gimbals' full torque, in the demanded direction, inside every stop.
    private static void ASaturatedGimbalLeavesTheRestToTheOthers(TestContext t)
    {
        var com = new float3(0f, 0f, 0f);
        GimbalController wide = Synthetic(floatQuat.Identity, com - new float3(PivotArm, 0f, 0f), new float3(-NozzleOffset, 0f, 0f), 2f * MaxAngle);
        GimbalController narrow = Synthetic(floatQuat.Identity, com - new float3(PivotArm, 0f, 0f), new float3(-NozzleOffset, 0f, 0f), MaxAngle);
        GimbalController[] gimbals = [wide, narrow];
        double[] thrusts = [Thrust, Thrust];
        Columns(wide, com, Thrust, out double3[] wideTorque, out _);
        Columns(narrow, com, Thrust, out double3[] narrowTorque, out _);
        // Far more than both can give, about Y alone.
        double3 unit = double3.Normalize(wideTorque[0]);
        double3 demand = unit * (10.0 * (wideTorque[0].Length() + narrowTorque[0].Length()));

        var scaledCommands = new double[4];
        TvcAllocationResult scaled = KsaTvcAllocator.Solve(gimbals, thrusts, com, demand, scaledCommands, redistribute: false);
        var commands = new double[4];
        TvcAllocationResult shared = KsaTvcAllocator.Solve(gimbals, thrusts, com, demand, commands);

        double both = wideTorque[0].Length() + narrowTorque[0].Length();
        double scaledGot = double3.Dot(scaled.AchievedTorque, unit), sharedGot = double3.Dot(shared.AchievedTorque, unit);
        t.CheckRel("scaled to the first stop, the narrow gimbal gives half of its torque", scaledGot, wideTorque[0].Length() + 0.5 * narrowTorque[0].Length(), 1e-3);
        t.CheckRel("redistributed, both gimbals give their full torque", sharedGot, both, 1e-3);
        double off = (shared.AchievedTorque - unit * sharedGot).Length();
        t.Check("the redistributed torque keeps the demanded direction", off <= 1e-6 * both, $"off the direction {off:E2} N m");
        bool inside = commands.All(c => Math.Abs(c) <= 1.0 + 1e-9);
        t.Check("every command stays inside its stop", inside, string.Join(", ", commands.Select(c => c.ToString("F4"))));
        t.Check("the reported share is all the demand the gimbals can give", Math.Abs(shared.SaturationScale - sharedGot / demand.Length()) < 1e-6,
            $"scale {shared.SaturationScale:F4}, given {sharedGot / demand.Length():F4}");
    }

    // A craft whose gimbals give no roll torque at all is planned with the roll left free and a roll bound above zero, and the choice of the first plan of a landing stands while the roll inertia falls with the propellant.
    private static void TheRollModelTakesACraftWithoutRollTorque(TestContext t)
    {
        Ksa6DofSetup.RollModel(0.0, 800.0, null, out bool freeAtZero, out double tauAtZero);
        t.Check("no roll torque at all leaves the roll free with a roll bound above zero",
            freeAtZero && tauAtZero > 0.0 && tauAtZero < Ksa6DofSetup.RollAuthorityFloorRadS2 * 800.0, $"free {freeAtZero}, bound {tauAtZero:G3} N m");

        Ksa6DofSetup.RollModel(26800.0, 60000.0, null, out bool freeWithTorque, out double tauWithTorque);
        t.Check("gimbals with roll torque keep the roll in the plan and their torque", !freeWithTorque && tauWithTorque == 26800.0,
            $"free {freeWithTorque}, bound {tauWithTorque:F0} N m");

        // 1.5 N m is under the floor at 800 kg m^2 (1.6 N m) and over it at 700 kg m^2 (1.4 N m).
        Ksa6DofSetup.RollModel(1.5, 800.0, null, out bool first, out _);
        Ksa6DofSetup.RollModel(1.5, 700.0, null, out bool fresh, out _);
        Ksa6DofSetup.RollModel(1.5, 700.0, first, out bool kept, out _);
        Ksa6DofSetup.RollModel(1.5, 800.0, false, out bool keptPinned, out _);
        t.Check("a later plan of the same landing keeps the choice of the first one", first && !fresh && kept && !keptPinned,
            $"first {first}, a fresh choice at the lower inertia {fresh}, kept {kept}, kept pinned {keptPinned}");
    }

    // Thrust along vehicle +X, the pivot PivotArm behind the centre of mass and the thrust point
    // NozzleOffset further back on the thrust axis.
    private static GimbalController AxialGimbal(out float3 com)
    {
        com = new float3(1f, -3f, 5f);
        return Synthetic(floatQuat.Identity, com - new float3(PivotArm, 0f, 0f),
            new float3(-NozzleOffset, 0f, 0f));
    }

    private static void ColumnsUseThePivotArm(TestContext t)
    {
        GimbalController gc = AxialGimbal(out float3 com);
        Columns(gc, com, Thrust, out double3[] torque, out double3[] force);
        CompareWithStockJacobian(t, "axial nozzle", gc, com, Thrust, torque);

        double pivotGain = PivotArm * Thrust * MaxAngle;
        for (int axis = 0; axis < 2; axis++)
        {
            t.CheckRel($"axial nozzle: axis {axis} torque per unit command is the pivot arm gain",
                torque[axis].Length(), pivotGain, ColumnTolerance);
            t.CheckRel($"axial nozzle: axis {axis} force per unit command",
                force[axis].Length(), Thrust * MaxAngle, ColumnTolerance);
        }
    }

    private static void DeliveredTorqueMatchesTheDemand(TestContext t)
    {
        GimbalController gc = AxialGimbal(out float3 com);
        Columns(gc, com, Thrust, out double3[] torque, out _);
        double3 demand = torque[0] * 0.3 + torque[1] * 0.2;

        var commands = new double[2];
        TvcAllocationResult result = KsaTvcAllocator.Solve(
            new[] { gc }, new double[] { Thrust }, com, demand, commands);
        t.Check("the demand is inside capability", result.SaturationScale == 1.0, $"scale {result.SaturationScale}");

        float2 angles = new((float)(commands[0] * MaxAngle), (float)(commands[1] * MaxAngle));
        float3 deliveredF = TvcAllocation.ComputeTorqueAsmb(gc, Thrust, angles, com);
        double3 delivered = new(deliveredF.X, deliveredF.Y, deliveredF.Z);
        double error = (delivered - demand).Length() / demand.Length();
        t.Check("the stock torque at the commanded angles matches the demand",
            error < 2e-3, $"relative error {error:E2}, demand {Fmt(demand)}, stock {Fmt(delivered)}");
        error = (result.AchievedTorque - demand).Length() / demand.Length();
        t.Check("the reported torque matches the demand", error < 1e-4, $"relative error {error:E2}");

        // The planned capability is the linear model at full command, compared with stock at full
        // deflection. With thrust along +X, a gimbal Y deflection turns the craft about Y and a Z
        // deflection about Z. The linearization costs MaxAngle^2/8 of torque, about 1.4e-3 here.
        float3 fullY = TvcAllocation.ComputeTorqueAsmb(gc, Thrust, new float2(MaxAngle, 0f), com);
        float3 fullZ = TvcAllocation.ComputeTorqueAsmb(gc, Thrust, new float2(0f, MaxAngle), com);
        t.CheckRel("capability about Y matches the stock full-deflection torque",
            Math.Abs(result.MaxTorque.Y), Math.Abs(fullY.Y), 3e-3);
        t.CheckRel("capability about Z matches the stock full-deflection torque",
            Math.Abs(result.MaxTorque.Z), Math.Abs(fullZ.Z), 3e-3);
    }

    // A rotated gimbal frame, a pivot off the centre line and a thrust point off the thrust axis,
    // so the columns also carry the term from the thrust point turning about the pivot.
    private static void ColumnsCoverALateralNozzleOffset(TestContext t)
    {
        doubleQuat rotation = doubleQuat.CreateFromAxisAngle(double3.Normalize(new double3(1, 2, 3)), 0.7);
        var com = new float3(0.5f, 2f, -1f);
        GimbalController gc = Synthetic(floatQuat.Pack(rotation), com + new float3(-6f, 1.5f, 0.8f),
            new float3(-NozzleOffset, 0.4f, -0.3f));
        Columns(gc, com, Thrust, out double3[] torque, out _);
        CompareWithStockJacobian(t, "offset nozzle", gc, com, Thrust, torque);
    }

    private static void RealEnginesMatchStock(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(DefaultSaves);
        if (saves.Count == 0)
        {
            t.Skip($"none of '{string.Join("', '", DefaultSaves)}' is in the game's Vehicles folder.");
            return;
        }

        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        try
        {
            Vehicle craft;
            try
            {
                craft = VehicleSpawner.SpawnFromSave(saves[0], t.System, home, "HarnessTvcAllocator",
                    OrbitFixtures.CircularAt(home, 500_000.0, Universe.GetElapsedTime()));
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"'{saves[0]}': {e.Message}");
                return;
            }
            craft.Parts.EnsureDerived(DerivedData.All);

            GimbalController[] gimbals = KsaGimbalControl.PlanningGimbals(craft);
            float3 com = craft.CenterOfMassAsmbF;
            GimbalController? main = null;
            foreach (GimbalController gc in gimbals)
            {
                if (gc.Data.MaximumThrust <= 0f)
                    continue;
                Columns(gc, com, gc.Data.MaximumThrust, out double3[] torque, out _);
                CompareWithStockJacobian(t, $"'{saves[0]}' gimbal {gc.InstanceId}", gc, com,
                    gc.Data.MaximumThrust, torque);
                if (main == null || gc.Data.MaximumThrust > main.Data.MaximumThrust)
                    main = gc;
            }
            if (main == null)
            {
                t.Skip($"'{saves[0]}' has no gimballed engine with thrust.");
                return;
            }

            ModelArmMatchesTheAllocator(t, craft, main, com);
        }
        finally
        {
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    // The 6-DOF model takes its torque per newton of the thrust of every engine, because its thrust is that
    // of all of them. One engine of a spawned craft is taken off its gimbal, as a fixed engine beside
    // gimballed ones is: the thrust on the gimbals falls by that engine, and the thrust the model divides
    // by must stay what it was.
    private static void EnginesOffTheGimbalsCountInTheModelThrust(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(DefaultSaves);
        if (saves.Count == 0)
        {
            t.Skip($"none of '{string.Join("', '", DefaultSaves)}' is in the game's Vehicles folder.");
            return;
        }

        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        GimbalController? taken = null;
        try
        {
            Vehicle craft;
            try
            {
                craft = VehicleSpawner.SpawnFromSave(saves[0], t.System, home, "HarnessGimbalShare",
                    OrbitFixtures.CircularAt(home, 500_000.0, Universe.GetElapsedTime()));
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"'{saves[0]}': {e.Message}");
                return;
            }
            craft.Parts.EnsureDerived(DerivedData.All);

            GimbalController[] gimbals = KsaGimbalControl.PlanningGimbals(craft);
            int withThrust = 0;
            foreach (GimbalController gc in gimbals)
                if (gc.Data.MaximumThrust > 0f && gc.Data.Nozzles.Count > 0)
                    withThrust++;
            if (withThrust < 2)
            {
                t.Skip($"'{saves[0]}' has {withThrust} gimballed engine(s) with thrust, and the check takes one of two off its gimbal.");
                return;
            }

            double onGimbals = Ksa6DofSetup.PlanningThrust(craft);
            double allEngines = Ksa6DofSetup.PlanningEngineThrust(craft);
            t.Check("no thrust is lost between the gimbals and the engines",
                allEngines >= onGimbals * (1.0 - 1e-3),
                $"gimbals {onGimbals / 1000.0:F1} kN, all engines {allEngines / 1000.0:F1} kN");

            foreach (GimbalController gc in gimbals)
                if (gc.Data.MaximumThrust > 0f && gc.Data.Nozzles.Count > 0)
                {
                    taken = gc;
                    break;
                }
            double removed = taken!.Data.MaximumThrust;
            taken.Data.Nozzles.Clear();
            taken.Data.MaximumThrust = 0f;

            double onGimbalsAfter = Ksa6DofSetup.PlanningThrust(craft);
            double allEnginesAfter = Ksa6DofSetup.PlanningEngineThrust(craft);
            t.Info($"'{saves[0]}': {onGimbals / 1000.0:F1} kN on gimbals of {allEngines / 1000.0:F1} kN, then "
                 + $"{onGimbalsAfter / 1000.0:F1} kN of {allEnginesAfter / 1000.0:F1} kN with one engine of {removed / 1000.0:F1} kN off its gimbal");
            t.CheckRel("the thrust on the gimbals falls by the engine taken off", onGimbalsAfter, onGimbals - removed, 1e-3);
            t.CheckRel("the thrust of every engine stays", allEnginesAfter, allEngines, 2e-3);
        }
        finally
        {
            taken?.RecomputeStaticData();
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    // The model's lateral torque is LArm times the lateral thrust, so its gain per unit command is
    // LArm * |Bf|. The allocator's is the part of its torque column across the thrust axis. The two
    // agree when the thrust point lies on the thrust axis through the pivot.
    private static void ModelArmMatchesTheAllocator(TestContext t, Vehicle craft, GimbalController main, float3 com)
    {
        float3 offset = main.Data.ThrustOffsetGimbal;
        float3 dir = main.Data.InitialDirGimbal;
        float3 lateral = offset - float3.Dot(offset, dir) * dir;
        double arm = Ksa6DofSetup.EngineArm(craft);
        if (lateral.Length() > 1e-3 * Math.Max(arm, 1.0))
        {
            t.Info($"main gimbal thrust point sits {lateral.Length():F3} m off its thrust axis, so the model arm is not compared.");
            return;
        }

        KsaFrameBridge.BodyAxes(craft, out _, out _, out double3 mz);
        Columns(main, com, main.Data.MaximumThrust, out double3[] torque, out double3[] force);
        float3 pivot = main.Data.PivotPosVehicleAsmb - com;
        t.Info($"main gimbal: model arm {arm:F3} m, pivot {Fmt(new double3(pivot.X, pivot.Y, pivot.Z))}, "
             + $"nozzle offset {offset.Length():F3} m, thrust {main.Data.MaximumThrust / 1000.0:F1} kN");
        for (int axis = 0; axis < 2; axis++)
        {
            if (force[axis].Length() <= 0.0)
                continue;
            double3 across = torque[axis] - mz * double3.Dot(torque[axis], mz);
            t.CheckRel($"main gimbal axis {axis}: the model arm gives the allocator's lateral gain",
                arm * force[axis].Length(), across.Length(), 2e-3);
        }
    }

    private static void CompareWithStockJacobian(TestContext t, string label, GimbalController gc,
                                                 float3 com, float thrust, double3[] torque)
    {
        var stock = new float3[2];
        TvcAllocation.ComputeJacobian(new[] { gc }, new[] { thrust }, new[] { float2.Zero }, com, stock);
        float[] maxAngle = { gc.Gimbal.AxisY.MaxAngle, gc.Gimbal.AxisZ.MaxAngle };
        double scale = Math.Max(torque[0].Length(), torque[1].Length());
        for (int axis = 0; axis < 2; axis++)
        {
            var expected = new double3(stock[axis].X, stock[axis].Y, stock[axis].Z) * maxAngle[axis];
            double error = (torque[axis] - expected).Length();
            // A real engine can gimbal further than the synthetic one, so the slope error scales with its range.
            double halfRange = 0.5 * maxAngle[axis];
            double tolerance = Math.Max(ColumnTolerance, 1e-4 + halfRange * halfRange / 4.0);
            t.Check($"{label}: axis {axis} column is the stock Jacobian times MaxAngle",
                error <= tolerance * Math.Max(scale, 1e-9),
                $"allocator {Fmt(torque[axis])}, stock {Fmt(expected)}");
        }
    }

    private static void Columns(GimbalController gc, float3 com, double thrust,
                                out double3[] torque, out double3[] force)
    {
        var bTorque = new double[6];
        var bForce = new double[6];
        KsaTvcAllocator.FillColumns(gc, com, thrust, bTorque, bForce, 0);
        torque = new[] { new double3(bTorque[0], bTorque[1], bTorque[2]), new double3(bTorque[3], bTorque[4], bTorque[5]) };
        force = new[] { new double3(bForce[0], bForce[1], bForce[2]), new double3(bForce[3], bForce[4], bForce[5]) };
    }

    // The fields GimbalController.RecomputeStaticData fills, with the rest thrust along gimbal +X.
    private static GimbalController Synthetic(floatQuat gimbalToVehicle, float3 pivot, float3 offsetGimbal, float maxAngle = MaxAngle)
    {
        Gimbal gimbal = Uninitialized<Gimbal>();
        gimbal.AxisY = new GimbalAxis { MaxAngle = maxAngle };
        gimbal.AxisZ = new GimbalAxis { MaxAngle = maxAngle };
        GimbalController gc = Uninitialized<GimbalController>();
        gc.Gimbal = gimbal;
        gc.Data = new GimbalControllerData();
        gc.Data.Gimbal2VehicleAsmb = gimbalToVehicle;
        gc.Data.PivotPosVehicleAsmb = pivot;
        gc.Data.InitialDirGimbal = float3.UnitX;
        gc.Data.ThrustOffsetGimbal = offsetGimbal;
        gc.Data.MaximumThrust = Thrust;
        return gc;
    }

    private static string Fmt(double3 v) => $"({v.X:E4}, {v.Y:E4}, {v.Z:E4})";

    // Constructors are skipped, so finalizers must not access the incomplete state.
    private static T Uninitialized<T>() where T : class
    {
        var value = (T)RuntimeHelpers.GetUninitializedObject(typeof(T));
        GC.SuppressFinalize(value);
        return value;
    }
}
