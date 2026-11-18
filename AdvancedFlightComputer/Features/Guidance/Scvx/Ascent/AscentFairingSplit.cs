namespace AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>A timed mass-only event inside one continuous liquid burn.</summary>
public static class AscentFairingSplit
{
    public static bool TryBuild(AscentProblem problem, AscentSolution path, int stage, double elapsed, double massKg,
        out AscentProblem split, out AscentSolution seed, out string reason)
    {
        split = problem;
        seed = path;
        if ((uint)stage >= (uint)problem.Stages.Length || (uint)stage >= (uint)path.BurnTime.Length)
        {
            reason = "The fairing gate has no engine phase on the solved path.";
            return false;
        }
        if (problem.Recoveries.Count > 0)
        {
            reason = "A fairing split with optimized booster reserves is unsupported.";
            return false;
        }
        AscentStage old = problem.Stages[stage];
        if (!old.HasLiquid || old.Solid != null || old.IsPinned || old.LiquidCarriesOver
            || (stage > 0 && problem.Stages[stage - 1].LiquidCarriesOver))
        {
            reason = "The gate is in a solid, pinned or shared-load phase.";
            return false;
        }
        double remaining = path.BurnTime[stage] - elapsed;
        if (!(elapsed > 1.0) || !(remaining > 1.0))
        {
            reason = "The gate is too close to an engine phase boundary.";
            return false;
        }

        int first = Array.IndexOf(path.NodeStage, stage), last = Array.LastIndexOf(path.NodeStage, stage);
        double dropAt = path.Time[first] + elapsed;
        (int lo, int hi, double blend) = Bracket(path.Time, first, last, dropAt);
        double used = path.Mass[first] - Mix(path.Mass[lo], path.Mass[hi], blend);
        if (!(used > 1.0) || !(old.PropellantMass - used > 1.0) || !(massKg > 0.0)
            || !(problem.FinalMassFloor - massKg > 0.0))
        {
            reason = "The gate leaves too little propellant or retained mass for a timed split.";
            return false;
        }

        // The pinned prefix and its free continuation share one fuel constraint.
        // The fairing leaves at the new boundary, and the real stage drop stays at its end.
        var prefix = new AscentStage(old.Thrust, old.MassFlow, used, massKg, old.DragArea,
            old.PressureGrid, old.ThrustAtPressure, old.ThrottleMin, fixedBurnTime: elapsed,
            liquidCarriesOver: true, seedThrottle: old.SeedThrottle, accelerationLimit: old.AccelerationLimit,
            physicalThrottleMin: old.PhysicalThrottleMin);
        var suffix = old.With(old.PropellantMass - used, old.JettisonMass, false);
        AscentStage[] stages = [.. problem.Stages[..stage], prefix, suffix, .. problem.Stages[(stage + 1)..]];
        split = new AscentProblem
        {
            Mu = problem.Mu, BodyRadius = problem.BodyRadius, Omega = problem.Omega,
            R0 = problem.R0, V0 = problem.V0, M0 = problem.M0, Stages = stages,
            Atmosphere = problem.Atmosphere, GroundRadius = problem.GroundRadius,
            TargetRadius = problem.TargetRadius, TargetSpeed = problem.TargetSpeed, TargetRadialRate = problem.TargetRadialRate,
            PlaneNormal = problem.PlaneNormal, QMax = problem.QMax, QAlphaMax = problem.QAlphaMax,
            PlaneTurnMin = problem.PlaneTurnMin, PlaneTurnMax = problem.PlaneTurnMax,
            LiftoffDirection = problem.LiftoffDirection, PadRoll = problem.PadRoll, Settings = problem.Settings,
        };
        seed = SplitSeed(path, stage, elapsed, remaining, massKg, Math.Max(3, problem.Settings.NodesPerStage));
        reason = "";
        return true;
    }

    private static AscentSolution SplitSeed(AscentSolution old, int split, double elapsed, double remaining, double massKg, int per)
    {
        double[] burn = [.. old.BurnTime[..split], elapsed, remaining, .. old.BurnTime[(split + 1)..]];
        int nodes = burn.Length * per;
        var nodeStage = new int[nodes];
        var time = new double[nodes];
        var mass = new double[nodes];
        var position = new double[3 * nodes];
        var velocity = new double[3 * nodes];
        var throttle = new double[3 * nodes];
        for (int stage = 0; stage < burn.Length; stage++)
        {
            int source = stage <= split ? stage : stage - 1;
            int first = Array.IndexOf(old.NodeStage, source), last = Array.LastIndexOf(old.NodeStage, source);
            double from = old.Time[first] + (stage == split + 1 ? elapsed : 0.0);
            for (int j = 0; j < per; j++)
            {
                int k = stage * per + j;
                nodeStage[k] = stage;
                double at = from + burn[stage] * j / (per - 1);
                time[k] = at;
                (int lo, int hi, double f) = Bracket(old.Time, first, last, at);
                mass[k] = Mix(old.Mass[lo], old.Mass[hi], f) - (stage > split ? massKg : 0.0);
                for (int axis = 0; axis < 3; axis++)
                {
                    position[3 * k + axis] = Mix(old.Position[3 * lo + axis], old.Position[3 * hi + axis], f);
                    velocity[3 * k + axis] = Mix(old.Velocity[3 * lo + axis], old.Velocity[3 * hi + axis], f);
                    throttle[3 * k + axis] = Mix(old.Throttle[3 * lo + axis], old.Throttle[3 * hi + axis], f);
                }
            }
        }

        // WarmStart uses the reference state, controls and burn times from this mesh.
        return new AscentSolution
        {
            Status = AscentStatus.IterationLimit, Message = "fairing split seed",
            Nodes = nodes, NodeStage = nodeStage, Time = time, Mass = mass, Position = position, Velocity = velocity,
            Throttle = throttle, BurnTime = burn, KickDeg = old.KickDeg, SeedNote = old.SeedNote,
            SeedLeastMaxQ = old.SeedLeastMaxQ, PlaneTurn = old.PlaneTurn,
            Thrust = [], DynamicPressure = [], QAlpha = [], StageDeltaV = [], TerminalResidual = [],
            MaxDefectPosition = double.NaN, MaxDefectVelocity = double.NaN, MaxDefectMass = double.NaN,
            Iterations = 0, Accepted = 0, SolveSeconds = 0.0, SeedSeconds = 0.0, Trace = [],
        };
    }

    private static (int Lo, int Hi, double T) Bracket(double[] times, int first, int last, double at)
    {
        if (at <= times[first]) return (first, first, 0.0);
        if (at >= times[last]) return (last, last, 0.0);
        int hi = Array.BinarySearch(times, first, last - first + 1, at);
        if (hi >= 0) return (hi, hi, 0.0);
        hi = ~hi;
        return (hi - 1, hi, (at - times[hi - 1]) / (times[hi] - times[hi - 1]));
    }

    private static double Mix(double a, double b, double t) => a + t * (b - a);
}
