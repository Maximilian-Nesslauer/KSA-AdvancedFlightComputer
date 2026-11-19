using AdvancedFlightComputer.Features.RcsTranslation;

namespace AdvancedFlightComputer.Features.RcsTorque;

/// <summary>
/// Turns a torque demand into the cheapest set of RCS thruster duties that gives it, for any layout. Pure arithmetic over the thrusters' full-thrust wrenches, so it knows nothing of the game and can be checked offline.
///
/// THE PROBLEM, one linear programme per call, in the caller's frame:
///     minimise    sum_i flow_i d_i + sum_a price_a (e+_a + e-_a)
///     subject to  sum_i torque_i d_i + e+ - e- = demand      (three rows)
///                 d_i + s_i = 1                                (one row per thruster)
///                 d, s, e+, e- >= 0
/// d_i is the fraction of the control period thruster i fires. The side force sum_i force_i d_i is left free: thrusters at one end of a booster cannot turn it without pushing it sideways, and the caller models that push rather than forbidding it. The shortfall e is priced far above any thruster, so it is used only for the part of a demand no duty pattern can give, and its per-axis prices say which axis gives way first.
///
/// This is the jet-select LP of Bergmann and Weiler (AIAA 1983-2209), as RcsLpSolver already runs it for translation, with the roles of force and torque swapped and an upper bound on each duty.
///
/// The same LP measures the layout for a planner (<see cref="Characterise"/>): the largest pure torque about each axis each way, and what a newton metre of it costs in propellant and side force.
/// </summary>
internal sealed class RcsTorqueAllocator
{
    /// <summary>How far above the dearest thruster a newton metre of shortfall is priced, before the per-axis weights.</summary>
    private const double ShortfallPriceFactor = 1.0e3;

    /// <summary>Duties under this are dropped: they would ask for a puff below any thruster's minimum pulse, and their torque is small.</summary>
    private const double DutyFloor = 1.0e-4;

    public int Count { get; }

    /// <summary>Full-thrust force of each thruster, N, three per thruster, in the caller's frame.</summary>
    public double[] Force { get; }

    /// <summary>Full-thrust torque about the centre of mass, N m, three per thruster, in the same frame.</summary>
    public double[] Torque { get; }

    /// <summary>Propellant flow at full thrust, kg/s, one per thruster. Zero leaves a thruster out.</summary>
    public double[] Flow { get; }

    public RcsTorqueAllocator(int count)
    {
        Count = count;
        Force = new double[3 * count];
        Torque = new double[3 * count];
        Flow = new double[count];
    }

    /// <summary>The cheapest duties for a demand, and what they give.</summary>
    public readonly record struct Result(
        bool Solved,
        double TorqueX, double TorqueY, double TorqueZ,
        double ForceX, double ForceY, double ForceZ,
        double FlowKgS,
        double ShortfallNm,
        int Firing);

    /// <summary>What one axis can do, one way: the largest pure torque about it (the other two held at zero), and per newton metre of a moderate demand, the propellant flow and the side force that come with it.</summary>
    public readonly record struct AxisAuthority(
        double MaxTorqueNm,
        double FlowPerNm,
        double ForcePerNmX, double ForcePerNmY, double ForcePerNmZ);

    private bool Usable(int i)
    {
        if (!(Flow[i] > 0.0) || !double.IsFinite(Flow[i]))
            return false;
        double t = Math.Abs(Torque[3 * i]) + Math.Abs(Torque[3 * i + 1]) + Math.Abs(Torque[3 * i + 2]);
        return t > 0.0 && double.IsFinite(t);
    }

    /// <summary>The dearest thruster's propellant per newton metre of the torque it gives, the reference the shortfall is priced against.</summary>
    private double DearestFlowPerNm()
    {
        double worst = 0.0;
        for (int i = 0; i < Count; i++)
        {
            if (!Usable(i))
                continue;
            double t = Math.Sqrt(Torque[3 * i] * Torque[3 * i] + Torque[3 * i + 1] * Torque[3 * i + 1]
                                 + Torque[3 * i + 2] * Torque[3 * i + 2]);
            worst = Math.Max(worst, Flow[i] / t);
        }
        return worst > 0.0 ? worst : 1.0;
    }

    /// <summary>
    /// The cheapest duties, written into <paramref name="duty"/> (Count long), for a torque demand in N m. <paramref name="priority"/> weights each axis's shortfall: the axis with the smallest weight gives way first when the demand is out of reach.
    /// </summary>
    public Result Allocate(double demandX, double demandY, double demandZ,
                           ReadOnlySpan<double> priority, Span<double> duty)
    {
        duty.Clear();
        int n = Count;
        // Columns: n duties, n bound slacks, six shortfalls. Rows: three torque, n bounds.
        int rows = 3 + n, cols = 2 * n + 6;
        var a = new double[rows * cols];
        var cost = new double[cols];
        var b = new double[rows];
        b[0] = demandX; b[1] = demandY; b[2] = demandZ;

        for (int i = 0; i < n; i++)
        {
            bool usable = Usable(i);
            for (int r = 0; r < 3; r++)
                a[i * rows + r] = usable ? Torque[3 * i + r] : 0.0;
            a[i * rows + 3 + i] = 1.0;
            a[(n + i) * rows + 3 + i] = 1.0;
            b[3 + i] = usable ? 1.0 : 0.0;
            cost[i] = usable ? Flow[i] : 0.0;
        }

        double price = ShortfallPriceFactor * DearestFlowPerNm();
        for (int r = 0; r < 3; r++)
        {
            double w = priority.Length > r && priority[r] > 0.0 ? priority[r] : 1.0;
            a[(2 * n + 2 * r) * rows + r] = 1.0;
            a[(2 * n + 2 * r + 1) * rows + r] = -1.0;
            cost[2 * n + 2 * r] = w * price;
            cost[2 * n + 2 * r + 1] = w * price;
        }

        double[]? x = RcsLpSolver.Solve(rows, cols, a, cost, b);
        if (x == null)
            return default;

        double tx = 0, ty = 0, tz = 0, fx = 0, fy = 0, fz = 0, flow = 0;
        int firing = 0;
        for (int i = 0; i < n; i++)
        {
            double d = Math.Clamp(x[i], 0.0, 1.0);
            if (d < DutyFloor)
                d = 0.0;
            duty[i] = d;
            if (d <= 0.0)
                continue;
            firing++;
            tx += Torque[3 * i] * d; ty += Torque[3 * i + 1] * d; tz += Torque[3 * i + 2] * d;
            fx += Force[3 * i] * d; fy += Force[3 * i + 1] * d; fz += Force[3 * i + 2] * d;
            flow += Flow[i] * d;
        }
        double sx = demandX - tx, sy = demandY - ty, sz = demandZ - tz;
        return new Result(true, tx, ty, tz, fx, fy, fz, flow, Math.Sqrt(sx * sx + sy * sy + sz * sz), firing);
    }

    /// <summary>
    /// What the layout can do about axis <paramref name="axis"/> (0, 1, 2 of the frame) in the direction <paramref name="sign"/>: the largest torque with the other two axes held at zero, and the propellant and side force per newton metre of the cheapest pattern at half of it.
    /// </summary>
    public AxisAuthority Characterise(int axis, int sign)
    {
        int n = Count;
        // Columns: n duties, n bound slacks, the torque t. Rows: three torque, n bounds. Maximise t, with a whisper of propellant cost so a tie goes to the cheaper pattern.
        int rows = 3 + n, cols = 2 * n + 1;
        var a = new double[rows * cols];
        var cost = new double[cols];
        var b = new double[rows];
        double tie = 1.0e-6 / DearestFlowPerNm();
        for (int i = 0; i < n; i++)
        {
            bool usable = Usable(i);
            for (int r = 0; r < 3; r++)
                a[i * rows + r] = usable ? Torque[3 * i + r] : 0.0;
            a[i * rows + 3 + i] = 1.0;
            a[(n + i) * rows + 3 + i] = 1.0;
            b[3 + i] = usable ? 1.0 : 0.0;
            cost[i] = usable ? tie * Flow[i] : 0.0;
        }
        a[2 * n * rows + axis] = -sign;
        cost[2 * n] = -1.0;

        double[]? x = RcsLpSolver.Solve(rows, cols, a, cost, b);
        double max = x != null ? Math.Max(x[2 * n], 0.0) : 0.0;
        if (!(max > 0.0) || !double.IsFinite(max))
            return default;

        Span<double> demand = stackalloc double[3];
        demand[axis] = 0.5 * sign * max;
        Span<double> duty = n <= 256 ? stackalloc double[n] : new double[n];
        Span<double> even = stackalloc double[3] { 1.0, 1.0, 1.0 };
        Result half = Allocate(demand[0], demand[1], demand[2], even, duty);
        double given = 0.5 * max;
        if (!half.Solved || half.ShortfallNm > 1e-6 * given)
            return new AxisAuthority(max, 0.0, 0.0, 0.0, 0.0);
        return new AxisAuthority(max, half.FlowKgS / given, half.ForceX / given, half.ForceY / given, half.ForceZ / given);
    }
}
