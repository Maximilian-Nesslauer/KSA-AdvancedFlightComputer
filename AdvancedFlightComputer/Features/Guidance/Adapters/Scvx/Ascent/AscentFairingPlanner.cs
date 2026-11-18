using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;

namespace AdvancedFlightComputer.Features.Guidance;

public sealed record AscentFairingRequest(int Row, (uint Id, double Mass)[] Parts, double ClearanceDatum,
    double AltitudeFloorM, double QCeilingPa, bool Atmospheric)
{
    internal double MassKg => Parts.Sum(part => part.Mass);
}

public sealed record PlannedFairingRelease(int Row, int Phase, double Time, double Radius, double Q, double MassKg);

internal static class AscentFairingPlanner
{
    private const double GateSampleSeconds = 0.25;
    private const double MaxGateDelaySeconds = 1.0;
    private const int MaxAttempts = 3;

    internal static PlannedFairingRelease? Refine(AscentFairingRequest request,
        AscentProblem baselineProblem, AscentSolution baseline,
        Func<AscentProblem, AscentSolution, AscentSolution> solve,
        out AscentProblem problem, out AscentSolution solution, out string reason)
    {
        problem = baselineProblem;
        solution = baseline;
        if (!request.Atmospheric || !(request.ClearanceDatum > 0.0) || !double.IsFinite(request.ClearanceDatum))
        {
            reason = "Airless or unresolved terrain clearance uses the live gate.";
            return null;
        }

        var initial = Scan(baseline, request, -1).First;
        if (initial is not { } initialGate)
        {
            reason = "The solved path does not reach the fairing gate.";
            return null;
        }

        int phase = initialGate.Phase;
        int first = Array.IndexOf(baseline.NodeStage, phase);
        double elapsed = initialGate.Time - baseline.Time[first];
        for (int attempt = 0; attempt < MaxAttempts; attempt++)
        {
            if (!AscentFairingSplit.TryBuild(baselineProblem, baseline, phase, elapsed, request.MassKg,
                out AscentProblem split, out AscentSolution seed, out reason))
                return null;
            AscentSolution candidate = solve(split, seed);
            if (!candidate.Usable)
            {
                reason = "The timed fairing solve did not produce a flyable plan.";
                return null;
            }

            int end = Array.LastIndexOf(candidate.NodeStage, phase);
            double time = candidate.Time[end];
            var (firstGate, atRelease) = Scan(candidate, request, end);
            if (firstGate is not { } qualified || (qualified.Phase != phase && qualified.Phase != phase + 1))
            {
                reason = "The fairing gate moved outside its engine phase in the timed solve.";
                return null;
            }
            double delay = time - qualified.Time;
            if (atRelease && delay >= -1e-7 && delay <= MaxGateDelaySeconds)
            {
                problem = split;
                solution = candidate;
                return new PlannedFairingRelease(request.Row, phase, time, Radius(candidate.Position, end),
                    candidate.DynamicPressure[end], request.MassKg);
            }

            // Rebuild from the unmodified problem, so no retry can deduct the mass twice.
            elapsed += qualified.Time - time + GateSampleSeconds;
        }

        reason = "The timed solve could not align the mass drop with the gate.";
        return null;
    }

    private readonly record struct GatePoint(int Phase, double Time);

    private static (GatePoint? First, bool AtRelease) Scan(AscentSolution path, AscentFairingRequest request, int release)
    {
        var gate = new FairingReleaseGate();
        GatePoint? first = null;
        bool qualifiedAtRelease = false;
        for (int k = 0; k < path.Nodes; k++)
        {
            double t = path.Time[k];
            int previous = Math.Max(0, k - 1);
            double start = path.Time[previous];
            double dt = t - start;
            if (k > 0 && !(dt > 0.0))
                continue;
            int samples = Math.Max(1, (int)Math.Ceiling(dt / GateSampleSeconds));
            double q0 = path.DynamicPressure[previous], dq = path.DynamicPressure[k] - q0;
            for (int j = 1; j <= samples; j++)
            {
                double fraction = (double)j / samples;
                double now = j == samples ? t : start + dt * fraction;
                double r2 = 0.0, rv = 0.0;
                for (int axis = 0; axis < 3; axis++)
                {
                    int from = 3 * previous + axis, to = 3 * k + axis;
                    double r = Mix(path.Position[from], path.Position[to], fraction);
                    double v = Mix(path.Velocity[from], path.Velocity[to], fraction);
                    r2 += r * r;
                    rv += r * v;
                }
                bool qualified = gate.Sample(now, Math.Max(q0 + fraction * dq, 0.0), Math.Sqrt(r2) - request.ClearanceDatum,
                    rv > 0.0, request.Atmospheric, request.AltitudeFloorM, request.QCeilingPa);
                if (qualified)
                    first ??= new GatePoint(path.NodeStage[k], now);
                if (k == release && j == samples)
                    qualifiedAtRelease = qualified;
            }
        }
        return (first, qualifiedAtRelease);
    }

    private static double Radius(double[] position, int k)
    {
        int i = 3 * k;
        return Math.Sqrt(position[i] * position[i] + position[i + 1] * position[i + 1] + position[i + 2] * position[i + 2]);
    }

    private static double Mix(double a, double b, double t) => a + (b - a) * t;
}
