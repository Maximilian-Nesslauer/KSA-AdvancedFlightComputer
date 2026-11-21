using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.HarnessTests.Fixtures;

// A free-flying light probe from stock parts: a small tank, a battery, a probe core, one scaled engine and four RCS
// blocks on two attitude tanks. The RCS scale sets how far one minimum pulse turns the probe.
internal static class LightProbeFixture
{
    private const string Probe = "CoreCommandC_Prefab_ProbeCoreSmallA";
    private const string Bank = "CoreElectricalA_Prefab_InlineBattery1WBankV1A";
    private const string Engine = "CorePropulsionA_Prefab_EngineA4";
    private const string Fuel = "CoreFuelTankA_Prefab_LF1W1HA";
    private const string AttitudeFuel = "CoreFuelTankA_Prefab_RadialMPQuarterWA";
    private const string Thruster = "CorePropulsionB_Prefab_RCSSmallFiveHornV1A";
    internal const int EngineSequence = 1;
    // DeLavalNozzle.SetScale scales the thrust with the square of the part scale. At this scale the probe takes about
    // 7 s for 30 m/s, longer than the shortest burn the engine can aim.
    private const double EngineScale = 0.125;
    private const double RcsMountOffset = 0.122;
    private const double HalfLength = 0.5;
    private const double Radius = 0.5;
    private const double MomentArm = 0.31;

    // At full size one minimum pulse of these blocks turns the probe by up to 0.3 rad/s, far inside the stock RCS
    // turnaround band. A scale of 0.15 cuts the torque about 45 times.
    internal const double FullRcs = 1.0;
    internal const double FineRcs = 0.15;

    internal static VehicleSaveData Design(double rcsScale)
        => new() { RootPartInstance = Stack(rcsScale), ActiveSequence = EngineSequence };

    private static PartInstance Stack(double rcsScale)
    {
        uint id = 0;
        doubleQuat rotation = doubleQuat.Identity;
        PartInstance tank = Part(Fuel, double3.Zero, rotation, 1, ref id);
        tank.Components.Add(Tank("Tank", ConsumerRole.Engine, 160, "H2(l)", "O2(l)", 5.5));
        PartInstance bank = Part(Bank, new double3(HalfLength + 0.124, 0, 0), rotation, 1, ref id);
        Join(tank, 0, bank, 1);
        PartInstance probe = Part(Probe, new double3(HalfLength + 0.4981, 0, 0), rotation, 1, ref id);
        Join(bank, 0, probe, 0);
        PartInstance engine = Part(Engine, new double3(-HalfLength - 1.0027 * EngineScale, 0, 0), rotation, EngineScale, ref id);
        engine.Components.Add(new EngineController.SaveData { InstanceOf = "LR91-AJ-3", ActiveInStage = true, Sequence = EngineSequence });
        Join(tank, 1, engine, 0);

        foreach (double side in new[] { -1.0, 1.0 })
        {
            doubleQuat tangentRotation = Roll(side * Math.PI / 2);
            double3 tangent = new double3(0, 0, -1).Transform(tangentRotation);
            PartInstance attitudeTank = Part(AttitudeFuel, tangent * (Radius + 0.01) + new double3(0.01, 0, 0), tangentRotation, 1, ref id);
            attitudeTank.Components.Add(Tank("Tank1", ConsumerRole.Thruster, 4, "MMH(l)", "N2O4(l)", 1.6));
            Surface(tank, attitudeTank, null);
            foreach (double x in new[] { -MomentArm, MomentArm })
            {
                PartInstance jet = Part(Thruster, tangent * (Radius + RcsMountOffset * rcsScale) + new double3(x, 0, 0), tangentRotation, rcsScale, ref id);
                Surface(tank, jet, 0);
            }
        }
        return tank;
    }

    private static PartInstance Part(string template, double3 position, doubleQuat rotation, double scale, ref uint id)
        => new()
        {
            InstanceOf = template,
            LocalInstanceId = ++id,
            Stage = 0,
            Transform = new() { PositionValue = position, RotationValue = rotation, ScaleValue = new(scale, scale, scale) }
        };

    private static Tank.SaveData Tank(string key, ConsumerRole role, double mass, string fuel, string oxidizer, double ratio)
    {
        double fraction = 1 / (1 + ratio);
        return new()
        {
            InstanceOf = key,
            RoleAffinity = role,
            Moles = [Mole(fuel, mass, fraction), Mole(oxidizer, mass, 1 - fraction)]
        };
    }

    private static MoleResourceData Mole(string substance, double mass, double fraction)
        => new() { SubstancePhaseId = substance, Mass = new(mass * fraction), MassFraction = new(fraction) };

    private static void Join(PartInstance parent, int parentIndex, PartInstance child, int childIndex)
    {
        AddChild(parent, child);
        Connector(parent, parentIndex, child.LocalInstanceId);
        Connector(child, childIndex, parent.LocalInstanceId);
    }

    private static void Surface(PartInstance parent, PartInstance child, int? index)
    {
        AddChild(parent, child);
        if (index.HasValue)
            Connector(child, index.Value, parent.LocalInstanceId);
    }

    private static void Connector(PartInstance part, int index, uint other)
        => (part.Connectors ??= []).Add(new() { Index = index, ConnectedLocalInstanceId = other });

    private static void AddChild(PartInstance parent, PartInstance child) => (parent.Children ??= []).Add(child);

    private static doubleQuat Roll(double radians) => QuaternionEx.CreateFromXyzRadians(new double3(radians, 0, 0));
}
