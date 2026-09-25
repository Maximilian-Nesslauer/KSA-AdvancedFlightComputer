using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// Solid motors in the convex ascent (issue #73), offline, on vehicles whose solids burn KSA's thrust curves.
///
/// The curves are thrust over the burn from an offline mirror of KSA's SolidMotor (the grain tables, the chamber-pressure balance and the nozzle, from the game's content), for the regressive grain the 2stage save's boosters carry and the neutral grain that is the SRB-E's default. The body, the air and the pad are the game's: the 2stage problem as the game wrote it out on 2026-09-24.
///
///   1. 2stage: two boosters, then two liquid stages. Planned with the booster burn averaged, as before #73, and with its curve. The curve's stage must burn exactly its grain in exactly its burn.
///   2. A core lit with two boosters that outlasts them: the core burns on into the next stage on the same load.
///   3. A core that would run dry forty seconds before its boosters at full throttle: planned throttled down to outlast them (#32).
///   4. The same core with a 90 % floor, which cannot: held at full thrust, the boosters burning on alone.
///   5. A problem with solids survives the JSON dump the game writes.
/// </summary>
internal static class AscentSolidCheck
{
    private const double Mu = 3.98601877170e14;
    private const double Re = 6371000.0;
    private const double OmegaE = 7.29211585454431e-5;
    private const double DragArea = 24.73;
    private const double HeldFloor = 0.99;

    // Thrust over mean against the share of the burn, from the mirror.
    private static readonly (double T, double F)[] Regressive = [(0.0, 1.51), (0.22, 1.50), (0.50, 1.25), (0.75, 0.49), (0.87, 0.29), (1.0, 0.20)];
    private static readonly (double T, double F)[] Neutral = [(0.0, 0.94), (0.73, 1.34), (0.80, 0.90), (0.90, 0.44), (1.0, 0.16)];

    private static readonly double[] Grid = Enumerable.Range(0, 17).Select(g => 101325.0 * g / 16).ToArray();

    private static int _fails;

    private static void Check(string name, bool ok, string detail = "")
    {
        Console.WriteLine($"  {(ok ? "PASS" : "FAIL")}  {name,-58} {detail}");
        if (!ok) _fails++;
    }

    /// <summary>A motor burning a shape: mass flow and thrust both follow it (chamber pressure drives both), and the air costs p Ae.</summary>
    private static AscentSolidMotor Motor(string name, (double T, double F)[] shape, double burn, double grain, double veVac, double exitArea)
    {
        const int n = 201;
        double mean = 0.0;
        for (int i = 0; i < shape.Length - 1; i++)
            mean += 0.5 * (shape[i].F + shape[i + 1].F) * (shape[i + 1].T - shape[i].T);
        var time = new double[n];
        var flow = new double[n];
        var thrust = new double[n * Grid.Length];
        for (int j = 0; j < n; j++)
        {
            double s = (double)j / (n - 1);
            int k = 0;
            while (k < shape.Length - 2 && shape[k + 1].T < s) k++;
            double f = shape[k].F + (shape[k + 1].F - shape[k].F) * (s - shape[k].T) / (shape[k + 1].T - shape[k].T);
            time[j] = burn * s;
            flow[j] = grain / burn * f / mean;
            for (int g = 0; g < Grid.Length; g++)
                thrust[j * Grid.Length + g] = Math.Max(0.0, veVac * flow[j] - Grid[g] * exitArea);
        }
        return new AscentSolidMotor { Name = name, Time = time, MassFlow = flow, Thrust = thrust };
    }

    private static double[] LiquidTable(double vac, double sea) => Grid.Select(p => vac + (sea - vac) * p / 101325.0).ToArray();

    private static AscentProblem Problem(AscentStage[] stages, double m0)
    {
        double rt = Re + 200e3;
        return new AscentProblem
        {
            Mu = Mu,
            BodyRadius = Re,
            Omega = OmegaE,
            R0 = [1649437.7496963653, -5344473.417754894, 3050570.207341944],
            V0 = [392.3141591534744, 111.89019953419796, 4.788190206557868],
            M0 = m0,
            Stages = stages,
            Atmosphere = new KsaAscentAtmosphere(new ExponentialAtmosphere(1.225, 101325.0, 8000.0, 1.4), null, 3.857),
            GroundRadius = Re,
            TargetRadius = rt,
            TargetSpeed = Math.Sqrt(Mu / rt),
            PlaneNormal = [-0.7762039895743317, 0.10808700950724297, 0.6211477802783103],
            QMax = 80000.0,
            QAlphaMax = 7000.0,
        };
    }

    /// <summary>Solves, and as the planner does (AscentPlanJob), plans again once to a max-q the vehicle can hold if no seed got under it.</summary>
    private static AscentSolution Solve(string what, AscentProblem p, bool verbose)
    {
        AscentSolution s = SolveOnce(what, p, verbose);
        if (s.Converged || !double.IsFinite(s.SeedLeastMaxQ) || !(s.SeedLeastMaxQ > 0.98 * p.QMax))
            return s;
        double held = Math.Ceiling(1.15 * s.SeedLeastMaxQ / 5000.0) * 5000.0;
        return SolveOnce($"{what}, max q relaxed to {held / 1000.0:F0} kPa", new AscentProblem
        {
            Mu = p.Mu, BodyRadius = p.BodyRadius, Omega = p.Omega, R0 = p.R0, V0 = p.V0, M0 = p.M0, Stages = p.Stages,
            Atmosphere = p.Atmosphere, GroundRadius = p.GroundRadius, TargetRadius = p.TargetRadius, TargetSpeed = p.TargetSpeed,
            TargetRadialRate = p.TargetRadialRate, PlaneNormal = p.PlaneNormal, QMax = held, QAlphaMax = p.QAlphaMax,
        }, verbose);
    }

    private static AscentSolution SolveOnce(string what, AscentProblem p, bool verbose)
    {
        AscentSolution s = AscentScvx.Solve(p, it =>
        {
            if (verbose)
                Console.WriteLine($"    iter {it.Index}: rho={it.Rho:+0.00;-0.00} {it.Status,-7} tr={it.TrustRegion:F3} "
                    + $"sig={string.Join("/", it.SigmaSeconds.Select(x => x.ToString("F1")))}s pred={it.Predicted:E1} "
                    + $"defect={it.DefectNorm:E2} path={it.PathViolation:E1} term={it.TerminalViolation:E1} J={it.Cost:E5}");
        });
        Console.WriteLine($"  {what}: {s.Message}, {s.Iterations} iterations, {s.SolveSeconds:F1} s; "
            + (s.Nodes > 0
                ? $"{s.FinalMass / 1000.0:F2} t to orbit, burns {string.Join(" / ", s.BurnTime.Select(b => b.ToString("F1")))} s, "
                  + $"max q {s.DynamicPressure.Max() / 1000.0:F1} kPa (seed's least {s.SeedLeastMaxQ / 1000.0:F1}), max q-alpha {s.QAlpha.Max():F0}, "
                  + $"defects {s.MaxDefectPosition:F1} m / {s.MaxDefectVelocity:F3} m/s / {s.MaxDefectMass:F1} kg"
                : ""));
        return s;
    }

    /// <summary>Mass burned over a stage, kg: its first node's mass less its last's.</summary>
    private static double Burned(AscentSolution s, int stage)
        => s.Mass[Array.IndexOf(s.NodeStage, stage)] - s.Mass[Array.LastIndexOf(s.NodeStage, stage)];

    /// <summary>The throttle |u| over a stage: least and mean.</summary>
    private static (double Min, double Mean) Throttle(AscentSolution s, int stage)
    {
        double min = double.PositiveInfinity, sum = 0.0;
        int n = 0;
        for (int k = 0; k < s.Nodes; k++)
        {
            if (s.NodeStage[k] != stage) continue;
            double u = Math.Sqrt(s.Throttle[k * 3] * s.Throttle[k * 3] + s.Throttle[k * 3 + 1] * s.Throttle[k * 3 + 1] + s.Throttle[k * 3 + 2] * s.Throttle[k * 3 + 2]);
            min = Math.Min(min, u);
            sum += u;
            n++;
        }
        return (min, sum / n);
    }

    internal static int Run(bool verbose)
    {
        _fails = 0;
        Console.WriteLine("ASCENT SCVX WITH SOLIDS: thrust curves, pinned burns, shared liquid loads (#73, #32)");

        TwoStage(verbose);
        TwoStageAsFlown(verbose);
        CoreOutlasts(verbose);
        CoreThrottledDown(verbose, 0.4);
        CoreThrottledDown(verbose, 0.9);
        RoundTrip();

        Console.WriteLine();
        Console.WriteLine(_fails == 0 ? "ALL PASS" : $"{_fails} FAILED");
        return _fails == 0 ? 0 : 1;
    }

    // ---- 1. 2stage: boosters, then two liquid stages.
    private static void TwoStage(bool verbose)
    {
        Console.WriteLine();
        Console.WriteLine("2stage: two regressive boosters, then two liquid stages");
        const double m0 = 634.0e3, grain = 437.6e3, burn = 68.9, casings = 43.7e3;
        double ve = 17738.714e3 / 6353.6, exitArea = (17738.714e3 - 16446.598e3) / 101325.0 / 2.0;
        AscentStage upper1 = new(955.5e3, 222.9, 74.5e3, 8.1e3, DragArea, Grid, LiquidTable(955.5e3, 723.7e3));
        AscentStage upper2 = new(952.3e3, 222.7, 62.4e3, 0.0, DragArea, Grid, LiquidTable(952.3e3, 721.7e3));

        // As the planner had it before #73: the boosters at their mean, held at full throttle.
        var mean = new AscentStage(17738.714e3, 6353.6, grain, casings, DragArea, Grid, LiquidTable(17738.714e3, 16446.598e3), HeldFloor);
        AscentSolution before = Solve("mean thrust", Problem([mean, upper1, upper2], m0), verbose);

        var motors = new[] { Motor("left", Regressive, burn, grain / 2, ve, exitArea), Motor("right", Regressive, burn, grain / 2, ve, exitArea) };
        var phases = new[]
        {
            new AscentPhase { StartMass = m0, EndMass = m0 - grain, Duration = burn, Solids = [0, 1] },
            new AscentPhase { StartMass = m0 - grain - casings, EndMass = m0 - grain - casings - 74.5e3, Duration = 74.5e3 / 222.9,
                              LiquidThrust = 955.5e3, LiquidMassFlow = 222.9, LiquidThrustAtPressure = LiquidTable(955.5e3, 723.7e3), LiquidEngines = [1] },
            new AscentPhase { StartMass = m0 - grain - casings - 74.5e3 - 8.1e3, EndMass = m0 - grain - casings - 74.5e3 - 8.1e3 - 62.4e3, Duration = 62.4e3 / 222.7,
                              LiquidThrust = 952.3e3, LiquidMassFlow = 222.7, LiquidThrustAtPressure = LiquidTable(952.3e3, 721.7e3), LiquidEngines = [2] },
        };
        AscentPhasePlan.Result plan = AscentPhasePlan.Build(phases, motors, Grid, m0, DragArea, 0.99, HeldFloor);
        foreach (string d in plan.Describe) Console.WriteLine("    " + d);
        AscentSolution after = Solve("thrust curve", Problem(plan.Stages, m0), verbose);

        Check("mean-thrust plan converges", before.Converged, before.Message);
        Check("thrust-curve plan converges", after.Converged, after.Message);
        if (after.Nodes == 0) return;
        Check("the boosters' stage burns exactly their burn", Math.Abs(after.BurnTime[0] - burn) < 1e-6, $"{after.BurnTime[0]:F6} s");
        // To the solver's own mass defect: the defect gate is 1e-4 of the lift-off mass per interval, 63 kg here.
        Check("and exactly their grain", Math.Abs(Burned(after, 0) - grain) < 1e-4 * m0, $"{Burned(after, 0) / 1000.0:F3} t of {grain / 1000.0:F1}");
        double lift = after.SolidThrust[0], meanVac = grain / burn * ve;
        Check("lift-off thrust follows the curve, not the mean", lift > 1.35 * meanVac, $"{lift / 1000.0:F0} kN against {meanVac / 1000.0:F0} kN mean in vacuum");
        Check("the curve's burnout thrust is the tail", after.SolidThrust[Array.LastIndexOf(after.NodeStage, 0)] < 0.3 * meanVac,
              $"{after.SolidThrust[Array.LastIndexOf(after.NodeStage, 0)] / 1000.0:F0} kN");
        Check("dynamically feasible", after.MaxDefectPosition < 35.0 && after.MaxDefectVelocity < 0.8 && after.MaxDefectMass < 20.0,
              $"{after.MaxDefectPosition:F2} m, {after.MaxDefectVelocity:F4} m/s, {after.MaxDefectMass:F1} kg");
        Check("inserts", Math.Abs(after.TerminalResidual[0]) < 150.0 && Math.Abs(after.TerminalResidual[1]) < 0.2,
              $"{after.TerminalResidual[0]:F1} m, {after.TerminalResidual[1]:F3} m/s");
        if (before.Nodes > 0)
            Console.WriteLine($"  to orbit: {before.FinalMass / 1000.0:F2} t with the mean, {after.FinalMass / 1000.0:F2} t with the curve; "
                + $"altitude at booster burnout {(Radius(before, 0) - Re) / 1000.0:F1} km against {(Radius(after, 0) - Re) / 1000.0:F1} km, "
                + $"speed {Speed(before, 0):F0} against {Speed(after, 0):F0} m/s");
    }

    // ---- 1b. 2stage as the game lays it out: the second stage's engine lit with the boosters and burning on after them, at 35 kPa and a 40 % floor. On 2026-09-25 the harness flew this and the plan converged at 35 kPa by pointing the thrust 44 deg below the horizon against the airflow to bleed off the boosters' speed - sin(alpha) is zero again at 180 deg - which no speed-indexed profile can fly. The plan must never thrust against the airflow; the planner then relaxes max-q as it should.
    private static void TwoStageAsFlown(bool verbose)
    {
        Console.WriteLine();
        Console.WriteLine("2stage as the game lays it out: the second stage lit with the boosters, 35 kPa, 40% floor");
        const double m0 = 614.2e3, grain = 422.26e3, burn = 68.9, casings = 43.7e3, upperFlow = 222.9, upperLoad = 74.5e3 + 15.35e3;
        double ve = 17738.714e3 / 6353.6, exitArea = (17738.714e3 - 16446.598e3) / 101325.0 / 2.0;
        var motors = new[] { Motor("left", Regressive, burn, grain / 2, ve, exitArea), Motor("right", Regressive, burn, grain / 2, ve, exitArea) };
        double end0 = m0 - grain - upperFlow * burn, start1 = end0 - casings, end1 = start1 - (upperLoad - upperFlow * burn), start2 = end1 - 8.1e3;
        var phases = new[]
        {
            new AscentPhase { StartMass = m0, EndMass = end0, Duration = burn, Solids = [0, 1],
                              LiquidThrust = 955.5e3, LiquidMassFlow = upperFlow, LiquidThrustAtPressure = LiquidTable(955.5e3, 723.7e3), LiquidEngines = [1] },
            new AscentPhase { StartMass = start1, EndMass = end1, Duration = (upperLoad - upperFlow * burn) / upperFlow,
                              LiquidThrust = 955.5e3, LiquidMassFlow = upperFlow, LiquidThrustAtPressure = LiquidTable(955.5e3, 723.7e3), LiquidEngines = [1] },
            new AscentPhase { StartMass = start2, EndMass = start2 - 44.9e3, Duration = 44.9e3 / 222.7,
                              LiquidThrust = 952.3e3, LiquidMassFlow = 222.7, LiquidThrustAtPressure = LiquidTable(952.3e3, 721.7e3), LiquidEngines = [2] },
        };
        AscentPhasePlan.Result plan = AscentPhasePlan.Build(phases, motors, Grid, m0, DragArea, 0.4, HeldFloor);
        foreach (string d in plan.Describe) Console.WriteLine("    " + d);
        AscentProblem p = Problem(plan.Stages, m0);
        p = new AscentProblem
        {
            Mu = p.Mu, BodyRadius = p.BodyRadius, Omega = p.Omega, R0 = p.R0, V0 = p.V0, M0 = p.M0, Stages = p.Stages,
            Atmosphere = p.Atmosphere, GroundRadius = p.GroundRadius, TargetRadius = p.TargetRadius, TargetSpeed = p.TargetSpeed,
            TargetRadialRate = p.TargetRadialRate, PlaneNormal = p.PlaneNormal, QMax = 35000.0, QAlphaMax = 3500.0,
        };
        p.Settings.ThrottleMin = 0.4;
        AscentSolution s = Solve("plan", p, verbose);
        Check("converges", s.Converged, s.Message);
        if (s.Nodes == 0) return;
        // The least cos(alpha) wherever the air loads the vehicle.
        double least = 1.0, omega = OmegaE;
        for (int k = 0; k < s.Nodes; k++)
        {
            if (s.DynamicPressure[k] < 100.0) continue;
            double rx = s.Position[k * 3], ry = s.Position[k * 3 + 1];
            double ax = s.Velocity[k * 3] + omega * ry, ay = s.Velocity[k * 3 + 1] - omega * rx, az = s.Velocity[k * 3 + 2];
            double ux = s.Throttle[k * 3], uy = s.Throttle[k * 3 + 1], uz = s.Throttle[k * 3 + 2];
            double c = (ax * ux + ay * uy + az * uz) / (Math.Sqrt(ax * ax + ay * ay + az * az) * Math.Sqrt(ux * ux + uy * uy + uz * uz));
            least = Math.Min(least, c);
        }
        Check("thrust never points against the airflow where q loads the vehicle", least > 0.0, $"least cos(alpha) {least:F3}");
        Check("the boosters' stage lasts their burn", Math.Abs(s.BurnTime[0] - burn) < 1e-6, $"{s.BurnTime[0]:F3} s");
    }

    private static double Radius(AscentSolution s, int stage)
    {
        int k = Array.LastIndexOf(s.NodeStage, stage);
        return Math.Sqrt(s.Position[k * 3] * s.Position[k * 3] + s.Position[k * 3 + 1] * s.Position[k * 3 + 1] + s.Position[k * 3 + 2] * s.Position[k * 3 + 2]);
    }

    private static double Speed(AscentSolution s, int stage)
    {
        int k = Array.LastIndexOf(s.NodeStage, stage);
        return Math.Sqrt(s.Velocity[k * 3] * s.Velocity[k * 3] + s.Velocity[k * 3 + 1] * s.Velocity[k * 3 + 1] + s.Velocity[k * 3 + 2] * s.Velocity[k * 3 + 2]);
    }

    // The core, the boosters and the upper stage the next two cases share.
    private const double CoreThrust = 2896e3, CoreSea = 2450e3, CoreFlow = 891.6, CoreDry = 8.3e3;
    private const double UpperThrust = 952.3e3, UpperSea = 721.7e3, UpperFlow = 222.7, UpperLoad = 55.4e3, UpperDry = 8.0e3;
    private const double BoosterVe = 2600.0, BoosterExit = 1.0, BoosterCasing = 4.0e3;

    // ---- 2. A core that outlasts its boosters.
    private static void CoreOutlasts(bool verbose)
    {
        Console.WriteLine();
        Console.WriteLine("a core lit with two neutral boosters, outlasting them, then an upper stage");
        const double burn = 60.0, grain = 60e3, load = 122.6e3;
        double m0 = load + CoreDry + grain + 2 * BoosterCasing + UpperLoad + UpperDry;
        var motors = new[] { Motor("left", Neutral, burn, grain / 2, BoosterVe, BoosterExit), Motor("right", Neutral, burn, grain / 2, BoosterVe, BoosterExit) };
        double coreBurned = CoreFlow * burn;
        double end0 = m0 - grain - coreBurned, start1 = end0 - 2 * BoosterCasing;
        double end1 = start1 - (load - coreBurned), start2 = end1 - CoreDry;
        var phases = new[]
        {
            new AscentPhase { StartMass = m0, EndMass = end0, Duration = burn, Solids = [0, 1],
                              LiquidThrust = CoreThrust, LiquidMassFlow = CoreFlow, LiquidThrustAtPressure = LiquidTable(CoreThrust, CoreSea), LiquidEngines = [7] },
            new AscentPhase { StartMass = start1, EndMass = end1, Duration = (load - coreBurned) / CoreFlow,
                              LiquidThrust = CoreThrust, LiquidMassFlow = CoreFlow, LiquidThrustAtPressure = LiquidTable(CoreThrust, CoreSea), LiquidEngines = [7] },
            new AscentPhase { StartMass = start2, EndMass = start2 - UpperLoad, Duration = UpperLoad / UpperFlow,
                              LiquidThrust = UpperThrust, LiquidMassFlow = UpperFlow, LiquidThrustAtPressure = LiquidTable(UpperThrust, UpperSea), LiquidEngines = [8] },
        };
        AscentPhasePlan.Result plan = AscentPhasePlan.Build(phases, motors, Grid, m0, DragArea, 0.4, HeldFloor);
        foreach (string d in plan.Describe) Console.WriteLine("    " + d);
        Check("the core burns on from the boosters' stage", plan.Stages[0].LiquidCarriesOver && !plan.Stages[1].LiquidCarriesOver);
        AscentSolution s = Solve("plan", Problem(plan.Stages, m0), verbose);
        Check("converges", s.Converged, s.Message);
        if (s.Nodes == 0) return;
        Check("the boosters' stage lasts their burn", Math.Abs(s.BurnTime[0] - burn) < 1e-6, $"{s.BurnTime[0]:F3} s");
        double liquid = Burned(s, 0) - grain + Burned(s, 1);
        Check("the core burns its whole load across the two stages", Math.Abs(liquid - load) < 20.0, $"{liquid / 1000.0:F3} t of {load / 1000.0:F1}");
        (double min, double avg) = Throttle(s, 0);
        Console.WriteLine($"  core throttle with the boosters: least {min:P0}, mean {avg:P0}; alone: mean {Throttle(s, 1).Mean:P0}");
        Check("dynamically feasible", s.MaxDefectPosition < 35.0 && s.MaxDefectVelocity < 0.8 && s.MaxDefectMass < 20.0,
              $"{s.MaxDefectPosition:F2} m, {s.MaxDefectVelocity:F4} m/s, {s.MaxDefectMass:F1} kg");
    }

    // ---- 3 and 4. A core that runs dry before its boosters at full throttle.
    private static void CoreThrottledDown(bool verbose, double floor)
    {
        Console.WriteLine();
        Console.WriteLine($"a core that runs dry 40 s before its boosters at full throttle, {floor:P0} floor");
        const double burn = 100.0, grain = 100e3, coreFull = 60.0;
        double load = CoreFlow * coreFull;
        double m0 = load + CoreDry + grain + 2 * BoosterCasing + UpperLoad + UpperDry;
        var motors = new[] { Motor("left", Neutral, burn, grain / 2, BoosterVe, BoosterExit), Motor("right", Neutral, burn, grain / 2, BoosterVe, BoosterExit) };
        // The drain model: boosters and core until the core is dry, the boosters alone, then everything below the upper stage goes.
        double withCore = grain * coreFull / burn;
        double end0 = m0 - load - withCore, end1 = end0 - (grain - withCore), start2 = end1 - CoreDry - 2 * BoosterCasing;
        var phases = new[]
        {
            new AscentPhase { StartMass = m0, EndMass = end0, Duration = coreFull, Solids = [0, 1],
                              LiquidThrust = CoreThrust, LiquidMassFlow = CoreFlow, LiquidThrustAtPressure = LiquidTable(CoreThrust, CoreSea), LiquidEngines = [7] },
            new AscentPhase { StartMass = end0, EndMass = end1, Duration = burn - coreFull, Solids = [0, 1] },
            new AscentPhase { StartMass = start2, EndMass = start2 - UpperLoad, Duration = UpperLoad / UpperFlow,
                              LiquidThrust = UpperThrust, LiquidMassFlow = UpperFlow, LiquidThrustAtPressure = LiquidTable(UpperThrust, UpperSea), LiquidEngines = [8] },
        };
        AscentPhasePlan.Result plan = AscentPhasePlan.Build(phases, motors, Grid, m0, DragArea, floor, HeldFloor);
        foreach (string d in plan.Describe) Console.WriteLine("    " + d);
        foreach (string n in plan.Notes) Console.WriteLine("    note: " + n);
        bool expectDown = load / (CoreFlow * floor) >= AscentPhasePlan.OutlastMargin * burn;
        Check(expectDown ? "planned throttled down to outlast the boosters" : "held at full thrust: the floor is too high to outlast them",
              plan.CoreThrottledDown == expectDown, $"{plan.Stages.Length} stages");
        AscentSolution s = Solve("plan", Problem(plan.Stages, m0), verbose);
        Check("converges", s.Converged, s.Message);
        if (s.Nodes == 0) return;
        if (expectDown)
        {
            Check("the boosters' stage lasts their whole burn", Math.Abs(s.BurnTime[0] - burn) < 1e-6, $"{s.BurnTime[0]:F3} s");
            double liquid = Burned(s, 0) - grain + Burned(s, 1);
            Check("the core burns its whole load across the two stages", Math.Abs(liquid - load) < 20.0, $"{liquid / 1000.0:F3} t of {load / 1000.0:F1}");
            (double min, double avg) = Throttle(s, 0);
            Check("the core is throttled below what would empty it with the boosters", avg < coreFull / burn + 1e-3,
                  $"mean {avg:P0}, least {min:P0}; it would empty at {coreFull / burn:P0}");
            Check("and burns on after them", s.BurnTime[1] > 0.5, $"{s.BurnTime[1]:F1} s alone");
        }
        else
        {
            Check("the core's stage lasts the drain model's phase", Math.Abs(s.BurnTime[0] - coreFull) < 1e-6, $"{s.BurnTime[0]:F3} s");
            Check("the boosters burn on alone for the rest", Math.Abs(s.BurnTime[1] - (burn - coreFull)) < 1e-6, $"{s.BurnTime[1]:F3} s");
            Check("the core is held at full thrust", Throttle(s, 0).Min > 0.985, $"least {Throttle(s, 0).Min:P1}");
        }
        Check("dynamically feasible", s.MaxDefectPosition < 35.0 && s.MaxDefectVelocity < 0.8 && s.MaxDefectMass < 20.0,
              $"{s.MaxDefectPosition:F2} m, {s.MaxDefectVelocity:F4} m/s, {s.MaxDefectMass:F1} kg");
        Check("inserts", Math.Abs(s.TerminalResidual[0]) < 150.0 && Math.Abs(s.TerminalResidual[1]) < 0.2,
              $"{s.TerminalResidual[0]:F1} m, {s.TerminalResidual[1]:F3} m/s");
    }

    // ---- 5. The dump.
    private static void RoundTrip()
    {
        Console.WriteLine();
        Console.WriteLine("the problem file carries the solids");
        var motors = new[] { Motor("one", Regressive, 50.0, 20e3, 2500.0, 0.5) };
        var phases = new[]
        {
            new AscentPhase { StartMass = 100e3, EndMass = 60e3, Duration = 50.0, Solids = [0],
                              LiquidThrust = CoreThrust, LiquidMassFlow = 400.0, LiquidThrustAtPressure = LiquidTable(CoreThrust, CoreSea), LiquidEngines = [7] },
            new AscentPhase { StartMass = 58e3, EndMass = 38e3, Duration = 50.0,
                              LiquidThrust = CoreThrust, LiquidMassFlow = 400.0, LiquidThrustAtPressure = LiquidTable(CoreThrust, CoreSea), LiquidEngines = [7] },
        };
        AscentStage[] stages = AscentPhasePlan.Build(phases, motors, Grid, 100e3, DragArea, 0.4, HeldFloor).Stages;
        AscentProblem p = Problem(stages, 100e3);
        AscentProblem q = AscentProblemFile.FromJson(AscentProblemFile.ToJson(p));
        AscentStage a = p.Stages[0], b = q.Stages[0];
        Check("the solid table, the pinned burn and the carry-over survive",
              b.Solid != null && b.Solid.Time.SequenceEqual(a.Solid!.Time) && b.Solid.Thrust.SequenceEqual(a.Solid.Thrust)
              && b.Solid.MassFlow.SequenceEqual(a.Solid.MassFlow) && b.FixedBurnTime == a.FixedBurnTime && b.LiquidCarriesOver && !q.Stages[1].IsPinned);
    }
}
