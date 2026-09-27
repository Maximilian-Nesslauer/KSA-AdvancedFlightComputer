using AdvancedFlightComputer.Guidance.Scvx.SixDof;

internal static class PlanStateInterpolationCheck
{
    private const int NX = Dynamics6Dof.NX;

    internal static int Run()
    {
        bool ok = CheckConstantAcceleration() && CheckHighSpeedReplan();
        Console.WriteLine(ok ? "PASS - plan state interpolation" : "FAIL - plan state interpolation");
        return ok ? 0 : 1;
    }

    private static bool CheckConstantAcceleration()
    {
        const double dt = 2.0;
        double[] p0 = [-10.0, 5.0, 100.0];
        double[] v0 = [3.0, -2.0, -5.0];
        double[] acceleration = [4.0, 0.5, -1.0];
        var states = new double[2 * NX];
        for (int i = 0; i < 3; i++)
        {
            states[i] = p0[i];
            states[Dynamics6Dof.IV + i] = v0[i];
            states[NX + i] = p0[i] + v0[i] * dt + 0.5 * acceleration[i] * dt * dt;
            states[NX + Dynamics6Dof.IV + i] = v0[i] + acceleration[i] * dt;
        }

        Span<double> position = stackalloc double[3];
        Span<double> velocity = stackalloc double[3];
        foreach (double fraction in new[] { 0.0, 0.25, 0.5, 0.75, 1.0 })
        {
            PlanStateInterpolation.SamplePositionVelocity(states, 0, 1, fraction, dt,
                position, velocity);
            double time = fraction * dt;
            for (int i = 0; i < 3; i++)
            {
                double expectedPosition = p0[i] + v0[i] * time
                    + 0.5 * acceleration[i] * time * time;
                double expectedVelocity = v0[i] + acceleration[i] * time;
                if (Math.Abs(position[i] - expectedPosition) > 1e-9
                    || Math.Abs(velocity[i] - expectedVelocity) > 1e-9)
                    return false;
            }
        }
        PlanStateInterpolation.SamplePositionVelocity(states, 1, 1, 0.0, dt,
            position, velocity);
        for (int i = 0; i < 3; i++)
            if (Math.Abs(position[i] - states[NX + i]) > 1e-9
                || Math.Abs(velocity[i] - states[NX + Dynamics6Dof.IV + i]) > 1e-9)
                return false;
        return true;
    }

    private static bool CheckHighSpeedReplan()
    {
        double[][] nodes =
        [
            [-20443.5928, 2446.81036, 3361.85302, 1152.61313, -138.996834, -148.500183],
            [-19288.0622, 2307.66233, 3211.9958, 1121.14939, -135.208341, -146.404315],
            [-18163.4729, 2172.11321, 3064.37575, 1089.47958, -131.436089, -143.823358],
        ];
        var states = new double[nodes.Length * NX];
        for (int k = 0; k < nodes.Length; k++)
            for (int i = 0; i < 6; i++)
                states[k * NX + i] = nodes[k][i];

        double[] observedPosition = [-20264.3911, 2425.20023, 3338.73689];
        double[] observedVelocity = [1147.80491, -138.413231, -148.241092];
        double oldDt = 49.8384618 / 49.0;
        double elapsed = 0.15580172;
        double newDt = (49.8384618 - elapsed) / 49.0;
        Span<double> startPosition = stackalloc double[3];
        Span<double> startVelocity = stackalloc double[3];
        PlanStateInterpolation.SamplePositionVelocity(states, 0, 1, elapsed / oldDt,
            oldDt, startPosition, startVelocity);

        double linearDriftSquared = 0.0, curvedDriftSquared = 0.0;
        for (int i = 0; i < 3; i++)
        {
            double linear = states[i] + elapsed / oldDt * (states[NX + i] - states[i]);
            linearDriftSquared += (linear - observedPosition[i]) * (linear - observedPosition[i]);
            curvedDriftSquared += (startPosition[i] - observedPosition[i])
                * (startPosition[i] - observedPosition[i]);
        }

        double nodeTime = elapsed + newDt;
        double node = nodeTime / oldDt;
        int left = (int)Math.Floor(node);
        Span<double> nextPosition = stackalloc double[3];
        Span<double> nextVelocity = stackalloc double[3];
        PlanStateInterpolation.SamplePositionVelocity(states, left, left + 1,
            node - left, oldDt, nextPosition, nextVelocity);
        double curvedDefectSquared = 0.0, linearDefectSquared = 0.0;
        for (int i = 0; i < 3; i++)
        {
            double curved = nextPosition[i] - observedPosition[i]
                - 0.5 * newDt * (observedVelocity[i] + nextVelocity[i]);
            curvedDefectSquared += curved * curved;

            double linearPosition = states[left * NX + i] + (node - left)
                * (states[(left + 1) * NX + i] - states[left * NX + i]);
            double linearVelocity = states[left * NX + Dynamics6Dof.IV + i] + (node - left)
                * (states[(left + 1) * NX + Dynamics6Dof.IV + i]
                   - states[left * NX + Dynamics6Dof.IV + i]);
            double linear = linearPosition - observedPosition[i]
                - 0.5 * newDt * (observedVelocity[i] + linearVelocity);
            linearDefectSquared += linear * linear;
        }

        Console.WriteLine($"High-speed position drift: linear {Math.Sqrt(linearDriftSquared):F3} m, "
            + $"cubic {Math.Sqrt(curvedDriftSquared):F3} m.");
        Console.WriteLine($"First-interval position defect: linear {Math.Sqrt(linearDefectSquared):F3} m, "
            + $"cubic {Math.Sqrt(curvedDefectSquared):F3} m.");
        return Math.Sqrt(linearDriftSquared) > 2.0
            && Math.Sqrt(curvedDriftSquared) < 0.1
            && Math.Sqrt(linearDefectSquared) > 2.0
            && Math.Sqrt(curvedDefectSquared) < 1.0;
    }
}
