using AdvancedFlightComputer.Features.RcsTorque;

/// <summary>
/// --rcs-refusal: the belly flop's RCS programme that came back unsolved in flight (2026-11-22 23:49, the game log's "RCS refused a torque"): 48 thrusters, 17420 N m of pure y asked for. Rebuilt from the logged columns, solved again, and checked against the allocator's own reach. Exit 1 while any demand here a thruster set can plainly meet comes back unsolved or short.
/// </summary>
internal static class RcsRefusalCheck
{
    static readonly double[,] Thrusters =
    {
        { -46579.4, -0.150068, -5976.22, 1.26699 },
        { -45900.5, -0.147559, -5976.15, 1.26699 },
        { 45900.5, 0.191695, 5976.15, 1.26699 },
        { 46579.4, 0.194203, 5976.22, 1.26699 },
        { -46239.9, -0.148813, -5976.19, 1.26699 },
        { 46239.9, 0.192949, 5976.19, 1.26699 },
        { 0.241673, -46579.9, -6062.74, 1.26699 },
        { 0.246661, -45901.1, -6062.74, 1.26699 },
        { -0.291471, 45901.1, 6062.74, 1.26699 },
        { -0.286483, 46579.9, 6062.74, 1.26699 },
        { 0.244167, -46240.5, -6062.74, 1.26699 },
        { -0.288977, 46240.5, 6062.74, 1.26699 },
        { 46580.2, -0.194212, -5969.55, 1.26699 },
        { 45901.3, -0.191703, -5969.62, 1.26699 },
        { -45901.3, 0.147548, 5969.62, 1.26699 },
        { -46580.2, 0.150057, 5969.55, 1.26699 },
        { 46240.7, -0.192957, -5969.58, 1.26699 },
        { -46240.7, 0.148802, 5969.58, 1.26699 },
        { 0.610082, 46579.9, -5883.4, 1.26699 },
        { 0.610082, 45901.1, -5883.4, 1.26699 },
        { -0.566598, -45901.1, 5883.4, 1.26699 },
        { -0.566598, -46579.9, 5883.4, 1.26699 },
        { 0.610082, 46240.5, -5883.4, 1.26699 },
        { -0.566598, -46240.5, 5883.4, 1.26699 },
        { 50672.8, 0.248834, -2806.93, 1.26699 },
        { 51300.4, 0.251322, -2548.2, 1.26699 },
        { -51300.4, -0.0917115, 2548.2, 1.26699 },
        { -50672.8, -0.0891983, 2806.93, 1.26699 },
        { 50986.6, 0.250078, -2677.57, 1.26699 },
        { -50986.6, -0.0904549, 2677.57, 1.26699 },
        { 0.544489, 50672.5, -2900.59, 1.26699 },
        { 0.523618, 51300.2, -2641.92, 1.26699 },
        { -0.686372, -51300.2, 2641.92, 1.26699 },
        { -0.707265, -50672.5, 2900.59, 1.26699 },
        { 0.534054, 50986.3, -2771.26, 1.26699 },
        { -0.696819, -50986.3, 2771.26, 1.26699 },
        { -50672.2, 0.0892086, -2814.92, 1.26699 },
        { -51299.9, 0.0917218, -2556.31, 1.26699 },
        { 51299.9, -0.251317, 2556.31, 1.26699 },
        { 50672.2, -0.24883, 2814.92, 1.26699 },
        { -50986.1, 0.0904652, -2685.61, 1.26699 },
        { 50986.1, -0.250074, 2685.61, 1.26699 },
        { 0.353104, -50672.5, -2721.26, 1.26699 },
        { 0.327247, -51300.2, -2462.59, 1.26699 },
        { -0.165271, 51300.2, 2462.59, 1.26699 },
        { -0.191148, 50672.5, 2721.26, 1.26699 },
        { 0.340175, -50986.3, -2591.92, 1.26699 },
        { -0.17821, 50986.3, 2591.92, 1.26699 }
    };

    public static int Run()
    {
        int n = Thrusters.GetLength(0);
        var alloc = new RcsTorqueAllocator(n);
        for (int i = 0; i < n; i++)
        {
            for (int r = 0; r < 3; r++)
                alloc.Torque[3 * i + r] = Thrusters[i, r];
            alloc.Flow[i] = Thrusters[i, 3];
        }
        int bad = 0;
        foreach (var (name, d, p) in new (string, double[], double[])[]
        {
            ("logged: +y 17420, priority 0.05 1 0.05", new[] { 0.0, 17420.0, 0.0 }, new[] { 0.05, 1, 0.05 }),
            ("+y 17420, priority 1 1 0.3", new[] { 0.0, 17420.0, 0.0 }, new[] { 1.0, 1.0, 0.3 }),
            ("-y 17420, priority 1 1 0.3", new[] { 0.0, -17420.0, 0.0 }, new[] { 1.0, 1.0, 0.3 }),
            ("+x 17420, priority 1 1 0.3", new[] { 17420.0, 0.0, 0.0 }, new[] { 1.0, 1.0, 0.3 }),
            ("+y 583000, priority 0.05 1 0.05", new[] { 0.0, 583000.0, 0.0 }, new[] { 0.05, 1.0, 0.05 }),
            ("+z 50000, priority 0.05 0.05 1", new[] { 0.0, 0.0, 50000.0 }, new[] { 0.05, 0.05, 1.0 }),
            ("-x -y 300000 each, priority 1 1 0.3", new[] { -300000.0, -300000.0, 0.0 }, new[] { 1.0, 1.0, 0.3 }),
        })
        {
            var duty = new double[n];
            var res = alloc.Allocate(d[0], d[1], d[2], p, duty);
            double asked = Math.Sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            double along = res.Solved ? (res.TorqueX * d[0] + res.TorqueY * d[1] + res.TorqueZ * d[2]) / asked : 0.0;
            bool ok = res.Solved && along >= 0.95 * asked;
            if (!ok) bad++;
            Console.WriteLine(FormattableString.Invariant($"  {(ok ? "ok  " : "FAIL")} {name,-40} solved {res.Solved}, gives {res.TorqueX,9:F0} {res.TorqueY,9:F0} {res.TorqueZ,9:F0} N m, {res.Firing} firing, {res.FlowKgS:F2} kg/s"));
        }
        for (int axis = 0; axis < 3; axis++)
            foreach (int sign in new[] { 1, -1 })
                Console.WriteLine(FormattableString.Invariant($"  reach {"xyz"[axis]}{(sign > 0 ? "+" : "-")}: {alloc.Characterise(axis, sign).MaxTorqueNm,9:F0} N m"));
        // The flight's columns had more digits than the log's six: the same thrusters, each torque nudged by a part in 1e5, and demands of every size about each axis and some mixed, as the phase plane asks.
        var rng = new Random(11);
        int tries = 0, unsolved = 0, shorted = 0;
        string first = null;
        for (int trial = 0; trial < 400; trial++)
        {
            var jig = new RcsTorqueAllocator(n);
            for (int i = 0; i < n; i++)
            {
                for (int r = 0; r < 3; r++)
                    jig.Torque[3 * i + r] = Thrusters[i, r] * (1 + 1e-5 * (2 * rng.NextDouble() - 1)) + 1e-3 * (2 * rng.NextDouble() - 1);
                jig.Flow[i] = Thrusters[i, 3];
            }
            for (int k = 0; k < 10; k++)
            {
                int axis = rng.Next(3);
                double mag = Math.Pow(10, 2 + 3.7 * rng.NextDouble()) * (rng.Next(2) == 0 ? 1 : -1);
                var d = new double[3];
                d[axis] = mag;
                if (rng.Next(4) == 0)
                {
                    int other = (axis + 1) % 3;
                    d[other] = (other == 2 ? 103000.0 : 583000.0) * 0.4 * (2 * rng.NextDouble() - 1);
                }
                var p = new[] { 0.05, 0.05, 0.05 };
                for (int r = 0; r < 3; r++) if (d[r] != 0) p[r] = r == 2 && rng.Next(2) == 0 ? 0.3 : 1.0;
                double reach = axis == 2 ? 103000 : 583000;
                if (Math.Abs(mag) > 0.5 * reach) continue;
                tries++;
                var duty = new double[n];
                var res = jig.Allocate(d[0], d[1], d[2], p, duty);
                double asked = Math.Sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
                double along = res.Solved ? (res.TorqueX * d[0] + res.TorqueY * d[1] + res.TorqueZ * d[2]) / asked : 0.0;
                if (!res.Solved) unsolved++;
                else if (along < 0.95 * asked) shorted++;
                if ((!res.Solved || along < 0.95 * asked) && first == null)
                    first = FormattableString.Invariant($"trial {trial}: demand {d[0]:F1} {d[1]:F1} {d[2]:F1}, priority {p[0]} {p[1]} {p[2]}, solved {res.Solved}, gave {res.TorqueX:F0} {res.TorqueY:F0} {res.TorqueZ:F0}");
            }
        }
        Console.WriteLine($"  fuzz: {tries} demands within reach, {unsolved} unsolved, {shorted} short of 95 %{(first != null ? "; first: " + first : "")}");
        bad += unsolved + shorted;
        Console.WriteLine(bad == 0 ? "PASS rcs-refusal" : $"FAIL rcs-refusal ({bad})");
        return bad == 0 ? 0 : 1;
    }
}
