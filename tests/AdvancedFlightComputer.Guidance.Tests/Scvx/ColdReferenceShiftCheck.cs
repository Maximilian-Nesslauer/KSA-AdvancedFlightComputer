using AdvancedFlightComputer.Guidance.Scvx.SixDof;

internal static class ColdReferenceShiftCheck
{
    internal static int Run()
    {
        const int nodes = 40;
        const double sigma = 20;
        const double thrust = 2_000_000;
        double step = sigma / (nodes - 1);
        var dynamics = new Dynamics6Dof.Params { Gz = -1.62 };
        var states = new double[nodes * Dynamics6Dof.NX];
        var controls = new double[nodes * Dynamics6Dof.NUBase];
        states[0] = 28000;
        states[1] = 150;
        states[2] = 5400;
        states[3] = 1300;
        states[5] = -175;
        states[6] = 1;
        states[Dynamics6Dof.IM] = 175000;
        double flow = thrust / (dynamics.Isp * dynamics.G0);
        for (int k = 0; k < nodes; k++)
        {
            controls[k * Dynamics6Dof.NUBase + Dynamics6Dof.IT] = thrust;
            if (k == nodes - 1) break;
            int offset = k * Dynamics6Dof.NX;
            int next = (k + 1) * Dynamics6Dof.NX;
            double mass = states[offset + Dynamics6Dof.IM];
            double nextMass = mass - flow * step;
            double nextVz = states[offset + 5]
                + 0.5 * step * (2 * dynamics.Gz + thrust / mass + thrust / nextMass);
            states[next] = states[offset] + states[offset + 3] * step;
            states[next + 1] = states[offset + 1];
            states[next + 2] = states[offset + 2] + 0.5 * step * (states[offset + 5] + nextVz);
            states[next + 3] = states[offset + 3];
            states[next + 5] = nextVz;
            states[next + 6] = 1;
            states[next + Dynamics6Dof.IM] = nextMass;
        }

        double original = WorstDefect(states, controls, dynamics, nodes, step);
        double[] observed = states[..Dynamics6Dof.NX].ToArray();
        observed[0] -= 20;
        observed[1] += 10;
        observed[2] -= 175;
        observed[3] -= 25;
        observed[5] -= 2;

        double[] firstNodeOnly = (double[])states.Clone();
        observed.CopyTo(firstNodeOnly, 0);
        double falseDefect = WorstDefect(firstNodeOnly, controls, dynamics, nodes, step);

        ColdReferenceShift.Apply(states, nodes, sigma, observed);
        double shifted = WorstDefect(states, controls, dynamics, nodes, step);
        bool anchored = states.AsSpan(0, Dynamics6Dof.NX).SequenceEqual(observed);
        double[] afterFuelUse = states[..Dynamics6Dof.NX].ToArray();
        afterFuelUse[Dynamics6Dof.IM] -= 1000;
        ColdReferenceShift.Apply(states, nodes, sigma, afterFuelUse);
        double shiftedMass = WorstDefect(states, controls, dynamics, nodes, step);
        bool passed = original < 1e-8 && falseDefect > 100 && shifted < 1e-8
            && shiftedMass > 1e-4 && shiftedMass < 1 && anchored;
        Console.WriteLine($"Cold reference shift: original defect {original:G3}, first-node defect {falseDefect:G3}, shifted defect {shifted:G3}, fuel-use defect {shiftedMass:G3}, anchored {anchored}.");

        bool blendPassed = CheckAttitudeAndRateBlend();
        passed &= blendPassed;
        Console.WriteLine(passed ? "PASS - cold reference shift" : "FAIL - cold reference shift");
        return passed ? 0 : 1;
    }

    private static bool CheckAttitudeAndRateBlend()
    {
        const int nodes = 20, nx = Dynamics6Dof.NX, iq = Dynamics6Dof.IQ, iw = Dynamics6Dof.IW;
        var states = new double[nodes * nx];
        for (int k = 0; k < nodes; k++)
        {
            states[k * nx + iq] = 1.0;
            states[k * nx + Dynamics6Dof.IM] = 50_000.0;
        }

        double[] observed = states[..nx].ToArray();
        double half = 0.5 * 5.0 * Math.PI / 180.0;
        observed[iq] = Math.Cos(half);
        observed[iq + 1] = Math.Sin(half);
        observed[iw] = 0.08;
        observed[iw + 1] = -0.05;
        observed[iw + 2] = 0.03;

        ColdReferenceShift.Apply(states, nodes, 10.0, observed);

        bool anchored = states.AsSpan(0, nx).SequenceEqual(observed);
        double dot01 = Math.Abs(states[iq] * states[nx + iq] + states[iq + 1] * states[nx + iq + 1]
                              + states[iq + 2] * states[nx + iq + 2] + states[iq + 3] * states[nx + iq + 3]);
        double jumpDeg = 2.0 * Math.Acos(Math.Clamp(dot01, 0.0, 1.0)) * 180.0 / Math.PI;
        double rateJump = Math.Abs(states[nx + iw] - states[iw]);

        double maxNormErr = 0.0;
        for (int k = 0; k < nodes; k++)
        {
            int o = k * nx + iq;
            double n = Math.Sqrt(states[o] * states[o] + states[o + 1] * states[o + 1]
                               + states[o + 2] * states[o + 2] + states[o + 3] * states[o + 3]);
            maxNormErr = Math.Max(maxNormErr, Math.Abs(n - 1.0));
        }

        int last = (nodes - 1) * nx;
        bool terminalUntouched = Math.Abs(states[last + iq] - 1.0) < 1e-12
            && Math.Abs(states[last + iq + 1]) < 1e-12
            && Math.Abs(states[last + iw]) < 1e-12;

        // Pure antipodal sign flip (q -> -q) must not rotate interior nodes so branch alignment sees the flip cleanly.
        var anti = new double[nodes * nx];
        for (int k = 0; k < nodes; k++)
        {
            anti[k * nx + iq] = 1.0;
            anti[k * nx + Dynamics6Dof.IM] = 50_000.0;
        }
        double[] obsAnti = anti[..nx].ToArray();
        obsAnti[iq] = -1.0;
        ColdReferenceShift.Apply(anti, nodes, 10.0, obsAnti);
        bool antipodalSafe = anti[iq] == -1.0 && Math.Abs(anti[nx + iq] - 1.0) < 1e-12;

        bool ok = anchored && jumpDeg < 1.05 && rateJump < 0.02 && maxNormErr < 1e-12
            && terminalUntouched && antipodalSafe;
        Console.WriteLine($"Attitude/rate blend: interval-0 jump {jumpDeg:F2} deg (vs 5.00), rate jump {rateJump:F3} rad/s (vs 0.080), |q| err {maxNormErr:E1}, terminal untouched {terminalUntouched}, antipodal safe {antipodalSafe}.");
        return ok;
    }

    private static double WorstDefect(double[] states, double[] controls, Dynamics6Dof.Params dynamics,
        int nodes, double step)
    {
        const int nx = Dynamics6Dof.NX;
        const int nu = Dynamics6Dof.NUBase;
        Span<double> first = stackalloc double[nx];
        Span<double> second = stackalloc double[nx];
        double worst = 0;
        for (int k = 0; k < nodes - 1; k++)
        {
            Dynamics6Dof.Eval(states.AsSpan(k * nx, nx), controls.AsSpan(k * nu, nu), dynamics, first);
            Dynamics6Dof.Eval(states.AsSpan((k + 1) * nx, nx), controls.AsSpan((k + 1) * nu, nu), dynamics, second);
            for (int i = 0; i < nx; i++)
            {
                double defect = states[(k + 1) * nx + i] - states[k * nx + i]
                    - 0.5 * step * (first[i] + second[i]);
                worst = Math.Max(worst, Math.Abs(defect));
            }
        }
        return worst;
    }
}
