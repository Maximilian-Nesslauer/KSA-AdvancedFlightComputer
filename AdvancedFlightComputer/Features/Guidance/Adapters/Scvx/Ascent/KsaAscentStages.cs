#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using System.Linq;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// The convex ascent's stages from the live vehicle, solids and all (issue #73): the drain model's phases, each split into the solid motors burning through it and its liquid engines, and each solid's burn sampled from the game's own model. <see cref="AscentPhasePlan"/> turns them into stages.
///
/// Main thread only: it reads the part tree and the mole states.
/// </summary>
internal static class KsaAscentStages
{
    private const double G0 = 9.80665;

    /// <summary>The phases and motors that went into a plan, for the log.</summary>
    internal sealed class Gathered
    {
        public List<AscentPhase> Phases = new();
        public List<AscentSolidMotor> Motors = new();
        /// <summary>Per motor: the burn the game's own thrust profile reports for its grain, s, for comparison with the simulated one.</summary>
        public List<double> ProfileSeconds = new();
    }

    /// <param name="model">The staging model, built with <paramref name="stageParts"/>.</param>
    /// <param name="pressureGrid">The planner's back pressures, Pa, or null for an airless body.</param>
    public static Gathered Gather(Vehicle vehicle, UpfgVehicle model, Dictionary<UpfgStage, HashSet<Part>> stageParts,
                                  double[] pressureGrid)
    {
        var result = new Gathered();
        PartTree tree = vehicle.Parts;
        double[] grid = pressureGrid ?? new[] { 0.0 };
        var motorIndex = new Dictionary<SolidMotor, int>();
        var liquidIndex = new Dictionary<Part, int>();

        foreach (UpfgStage st in model.Stages)
        {
            stageParts.TryGetValue(st, out HashSet<Part> parts);
            parts ??= new HashSet<Part>();
            var solids = new List<int>();
            var liquids = new List<Part>();
            foreach (Part part in parts)
            {
                bool solid = false;
                foreach (EngineController engine in part.Modules.Get<EngineController>())
                {
                    if (engine.Cores == null)
                        continue;
                    foreach (RocketCore core in engine.Cores)
                    {
                        if (core is not SolidMotor motor)
                            continue;
                        solid = true;
                        if (!motor.Stack.IsValid || motor.Rocket?.Nozzles == null)
                            continue;
                        if (!motorIndex.TryGetValue(motor, out int index))
                        {
                            AscentSolidMotor burn = SimulateBurn(tree, motor, grid);
                            if (burn == null)
                                continue;
                            index = result.Motors.Count;
                            motorIndex[motor] = index;
                            result.Motors.Add(burn);
                            result.ProfileSeconds.Add(motor.VacuumThrustProfile.TotalBurnSeconds);
                        }
                        if (!solids.Contains(index))
                            solids.Add(index);
                    }
                }
                if (!solid)
                    liquids.Add(part);
            }

            double stageFlow = st.Thrust / (st.Isp * G0);
            double duration = stageFlow > 0.0 ? (st.MassTotal - st.MassDry) / stageFlow : 0.0;
            var ids = liquids.Select(p => liquidIndex.TryGetValue(p, out int id) ? id : liquidIndex[p] = liquidIndex.Count).ToArray();

            if (solids.Count == 0)
            {
                // A liquid stage as it has always been planned: the drain model's own thrust, and its table.
                result.Phases.Add(new AscentPhase
                {
                    StartMass = st.MassTotal,
                    EndMass = st.MassDry,
                    Duration = duration,
                    LiquidThrust = st.Thrust,
                    LiquidMassFlow = stageFlow,
                    LiquidThrustAtPressure = st.ThrustAtPressure,
                    Throttleable = st.Throttleable,
                    LiquidEngines = ids,
                });
                continue;
            }

            // Solids and liquids together: the liquids from their own engines at each grid pressure, the solids from their simulated burns.
            double liquidFlow = 0.0;
            double[] liquidTable = null;
            double liquidThrust = 0.0;
            if (liquids.Count > 0)
            {
                var thrustAt = new double[grid.Length];
                for (int g = 0; g < grid.Length; g++)
                {
                    float3 sum = float3.Zero;
                    double flow = 0.0;
                    foreach (Part part in liquids)
                    {
                        (float3 thrustVec, double massFlow) = KsaVehicleAdapter.PartDesignPerformance(tree, part, grid[g]);
                        sum += thrustVec;
                        flow += massFlow;
                    }
                    thrustAt[g] = sum.Length();
                    if (g == 0)
                        liquidFlow = flow;
                }
                liquidThrust = thrustAt[0];
                if (pressureGrid != null && thrustAt.All(t => t > 0.0))
                    liquidTable = thrustAt;
                if (!(liquidThrust > 0.0) || !(liquidFlow > 0.0))
                {
                    liquidThrust = 0.0;
                    liquidFlow = 0.0;
                    liquidTable = null;
                    ids = Array.Empty<int>();
                }
            }
            result.Phases.Add(new AscentPhase
            {
                StartMass = st.MassTotal,
                EndMass = st.MassDry,
                Duration = duration,
                LiquidThrust = liquidThrust,
                LiquidMassFlow = liquidFlow,
                LiquidThrustAtPressure = liquidTable,
                Throttleable = true,
                Solids = solids.ToArray(),
                LiquidEngines = ids,
            });
        }
        return result;
    }

    /// <summary>
    /// A solid motor's burn from its grain as it is now, stepped through the same functions the game flies it by each frame (SolidMotor.UpdateState and ConsumePropellant): the burning area of each segment's grain, the chamber conditions that area settles at, each nozzle's performance at those conditions and every grid pressure, and the grain consumed by each segment's share of the area. It ends where the game's would: below half the minimum burn pressure once lit, or out of burnable grain. Null if the motor cannot burn at all.
    ///
    /// Stepped at the midpoint (RK2) about 600 times over the burn, which follows the grain's regression far more closely than the drain model's mean pacing does, and costs a few milliseconds.
    /// </summary>
    internal static AscentSolidMotor SimulateBurn(PartTree tree, SolidMotor motor, double[] grid)
    {
        if (tree?.Moles == null || tree.RocketNozzles == null || tree.RocketCores == null
            || !motor.Stack.IsValid || motor.Rocket?.Nozzles == null || motor.Stack.Segments.Length == 0)
            return null;

        SolidGrainSegment[] segments = motor.Stack.Segments;
        ReadOnlySpan<MoleState> moles = tree.Moles.States;
        var mass = new double[segments.Length];
        for (int s = 0; s < segments.Length; s++)
            mass[s] = segments[s].Grain != null ? moles[segments[s].Grain.StatesIdx].Mass : 0.0;

        RocketCoreState live = tree.RocketCores.States[motor.StatesIdx];
        bool lit = live.Throttle > 0f;
        float warm = lit && live.Conditions.Core.Pressure > 0f ? live.Conditions.Core.Pressure : motor.MaxChamberPressure;

        double usable = Usable(segments, mass);
        if (!(usable > 0.0))
            return null;
        // The game's own reckoning of the full grain's burn, to size the step; the grain over the ignition flow if it has none.
        double estimate = motor.VacuumThrustProfile.TotalBurnSeconds;
        if (!(estimate > 0.0))
        {
            if (!Evaluate(tree, motor, segments, mass, ref warm, out double flow0, null, null) || !(flow0 > 0.0))
                return null;
            estimate = usable / flow0;
        }
        double dt = Math.Clamp(estimate / 600.0, 1e-3, 0.5);

        int np = grid.Length;
        var time = new List<double>();
        var flows = new List<double>();
        var thrust = new List<double>();
        var row = new double[np];
        var mid = new double[segments.Length];
        double t = 0.0;
        double quench = lit ? 0.5 * motor.MinimumBurnPressure : motor.MinimumBurnPressure;
        for (int step = 0; step < 20000; step++)
        {
            float pressure = warm;
            if (!Evaluate(tree, motor, segments, mass, ref pressure, out double flow, grid, row) || pressure < quench || !(flow > 0.0))
                break;
            warm = pressure;
            quench = 0.5 * motor.MinimumBurnPressure;
            time.Add(t);
            flows.Add(flow);
            thrust.AddRange(row);

            usable = Usable(segments, mass);
            // The game stops a motor once a frame's burn would take the last of its grain; here the last step is cut to exactly that grain.
            if (usable <= flow * dt || usable <= motor.InitialBurnableGrainMass * 1e-4)
            {
                double last = Math.Max(usable / flow, 1e-4);
                time.Add(t + last);
                flows.Add(flow);
                thrust.AddRange(row);
                break;
            }

            // Midpoint: the flow half a step on, from the grain consumed at the start's flow.
            Consume(segments, mass, flow * 0.5 * dt, mid);
            float midPressure = warm;
            if (!Evaluate(tree, motor, segments, mid, ref midPressure, out double midFlow, null, null) || !(midFlow > 0.0))
                midFlow = flow;
            Consume(segments, mass, midFlow * dt, mass);
            t += dt;
        }

        if (time.Count < 2)
            return null;
        return new AscentSolidMotor
        {
            Name = motor.TemplateId,
            Time = time.ToArray(),
            MassFlow = flows.ToArray(),
            Thrust = thrust.ToArray(),
        };
    }

    // Burnable grain left, kg: above each segment's unburnable residue.
    private static double Usable(SolidGrainSegment[] segments, double[] mass)
    {
        double sum = 0.0;
        for (int s = 0; s < segments.Length; s++)
            sum += Math.Max(0.0, mass[s] - segments[s].UnburnableGrainMass);
        return sum;
    }

    // The motor at a grain state: chamber pressure (warm-started from and returned through pressure), mass flow, and the summed thrust at each grid pressure into row if asked for.
    private static bool Evaluate(PartTree tree, SolidMotor motor, SolidGrainSegment[] segments, double[] mass,
                                 ref float pressure, out double massFlow, double[] grid, double[] row)
    {
        massFlow = 0.0;
        float area = 0f;
        for (int s = 0; s < segments.Length; s++)
            area += segments[s].ComputeBurningArea((float)mass[s]);
        if (!(area > 0f))
            return false;
        RocketCoreConditions conditions = motor.SolveConditionsForArea(area, pressure);
        pressure = conditions.Core.Pressure;
        if (!(pressure > 0f))
            return false;
        var nozzles = tree.RocketNozzles.GetModulesAndStates(motor.Rocket.Nozzles.AsSpan());
        foreach (var nozzle in nozzles)
            massFlow += nozzle.Module.ComputePerformance(in conditions, 0f).GetRocketPerformance().MassFlowRate;
        if (row != null)
            for (int g = 0; g < grid.Length; g++)
            {
                float3 sum = float3.Zero;
                foreach (var nozzle in tree.RocketNozzles.GetModulesAndStates(motor.Rocket.Nozzles.AsSpan()))
                    sum += nozzle.Module.ComputePerformance(in conditions, (float)grid[g]).GetRocketPerformance().TotalThrust
                           * nozzle.State.ThrustDirectionVehicleAsmb;
                row[g] = sum.Length();
            }
        return double.IsFinite(massFlow);
    }

    // SolidMotor.ConsumePropellant: each segment gives up its share of the burning area.
    private static void Consume(SolidGrainSegment[] segments, double[] from, double burned, double[] into)
    {
        float area = 0f;
        var share = new float[segments.Length];
        for (int s = 0; s < segments.Length; s++)
        {
            share[s] = segments[s].ComputeBurningArea((float)from[s]);
            area += share[s];
        }
        for (int s = 0; s < segments.Length; s++)
            into[s] = area > 0f ? Math.Max(0.0, from[s] - burned * share[s] / area) : from[s];
    }
}
