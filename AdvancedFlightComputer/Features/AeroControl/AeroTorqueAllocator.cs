using AdvancedFlightComputer.Features.RcsTranslation;

namespace AdvancedFlightComputer.Features.AeroControl;

/// <summary>
/// Turns a torque demand into the control-surface deflections that give it, for the flight condition the surfaces were evaluated at. Pure arithmetic over each surface's torque and force at its swept deflections, so it knows nothing of the game and can be checked offline. The aero counterpart of RcsTorqueAllocator, with the same two questions: what deflections give this torque (<see cref="Allocate"/>), and how much torque can the surfaces give about each axis each way (<see cref="Characterise"/>).
///
/// THE CURVES. Each surface comes as its torque and force at a handful of deflections (KsaAeroSweep.SurfaceTable read at the flight condition: 0 and every 15 deg to its limits), zero at rest, straight between. The surfaces are added together: each was swept alone.
///
/// THE PROBLEM, one linear programme per call, in the caller's frame. Each surface's deflection is split into segments outward from rest - 0 to 15, 15 to 30, and the same the other way - each a variable between 0 and its width, giving its segment's slope of torque per degree:
///     minimise    sum_s c |s| + sum_a price_a (e+_a + e-_a)
///     subject to  sum_s slope_s s + e+ - e- = demand      (three rows)
///                 s + slack_s = width_s                     (one row per segment)
///                 s, slack, e+, e- >= 0
/// The shortfall e is priced far above any degree, so the surfaces give all they can before any of the demand goes unmet, and its per-axis prices say which axis gives way first. A small price per degree (c) makes the answer unique and keeps surfaces home that the demand does not need.
///  The segments model the curve exactly where each surface's effectiveness falls off with deflection (outer segments flatter), as stock aero's does as a fin stalls. The deflection is the segments added up from rest whatever order the programme filled them in, and the torque reported is the curve's at that deflection, so where an outer segment is the steeper the result is still true, if not the best.
///  The side force comes with the torque and is left free, as for the RCS: a surface cannot turn a craft without pushing it, and the caller models the push.
/// </summary>
internal sealed class AeroTorqueAllocator
{
    /// <summary>The price of a degree of deflection, as a share of the torque the most effective degree gives: small enough that any shortfall is worth a degree, large enough to keep idle surfaces home.</summary>
    private const double DegreePriceShare = 1.0e-3;

    /// <summary>Deflections under this, degrees, are the solver's round-off and are dropped: far below any actuator's step (KSAero's is 0.1 deg).</summary>
    private const double DeflectionFloorDeg = 1.0e-3;

    /// <summary>The price of a degree moved, as a share of the torque the most effective degree gives (Allocate from where the surfaces are): half a degree's price away from rest, so a surface holds still unless moving buys torque, and still goes home when nothing needs it.</summary>
    private const double ChangePriceShare = 0.5e-3;

    private readonly double[][] _deflection;
    private readonly double[][] _torque;
    private readonly double[][] _force;

    public int Count { get; }

    public AeroTorqueAllocator(int count)
    {
        Count = count;
        _deflection = new double[count][];
        _torque = new double[count][];
        _force = new double[count][];
        for (int i = 0; i < count; i++)
        {
            _deflection[i] = [0.0];
            _torque[i] = new double[3];
            _force[i] = new double[3];
        }
    }

    /// <summary>
    /// Surface <paramref name="i"/>'s curve: its deflections, degrees, ascending, with 0 among them, and at each the torque (N m) and force (N) it adds, three per deflection, in the caller's frame.
    /// </summary>
    public void SetSurface(int i, ReadOnlySpan<double> deflectionDeg, ReadOnlySpan<double> torque, ReadOnlySpan<double> force)
    {
        int n = deflectionDeg.Length;
        if (n == 0 || torque.Length < 3 * n || force.Length < 3 * n)
            throw new ArgumentException("A surface needs a torque and a force at every deflection.");
        _deflection[i] = deflectionDeg.ToArray();
        _torque[i] = torque[..(3 * n)].ToArray();
        _force[i] = force[..(3 * n)].ToArray();
    }

    /// <summary>What a set of deflections gives, and how short of the demand it falls.</summary>
    public readonly record struct Result(
        bool Solved,
        double TorqueX, double TorqueY, double TorqueZ,
        double ForceX, double ForceY, double ForceZ,
        double ShortfallNm,
        int Deflected);

    /// <summary>What the surfaces can do about one axis, one way: the largest pure torque (the other two held at zero), and the force per newton metre of the deflections that give half of it.</summary>
    public readonly record struct AxisAuthority(
        double MaxTorqueNm,
        double ForcePerNmX, double ForcePerNmY, double ForcePerNmZ);

    // One segment: surface, the deflection it starts at, its signed width (degrees, negative outward on the negative side), and its slope of torque per degree of its own travel.
    private readonly record struct Segment(int Surface, double From, double Width, double SlopeX, double SlopeY, double SlopeZ);

    private List<Segment> Segments()
    {
        var segments = new List<Segment>();
        for (int i = 0; i < Count; i++)
        {
            double[] d = _deflection[i];
            double[] t = _torque[i];
            int zero = Array.IndexOf(d, 0.0);
            if (zero < 0)
                continue;
            for (int k = zero; k + 1 < d.Length; k++)
                AddSegment(segments, i, d, t, k, k + 1);
            for (int k = zero; k - 1 >= 0; k--)
                AddSegment(segments, i, d, t, k, k - 1);
        }
        return segments;
    }

    private static void AddSegment(List<Segment> into, int surface, double[] d, double[] t, int from, int to)
    {
        double width = Math.Abs(d[to] - d[from]);
        if (!(width > 0.0))
            return;
        double sx = (t[3 * to] - t[3 * from]) / width;
        double sy = (t[3 * to + 1] - t[3 * from + 1]) / width;
        double sz = (t[3 * to + 2] - t[3 * from + 2]) / width;
        if (!double.IsFinite(sx) || !double.IsFinite(sy) || !double.IsFinite(sz))
            return;
        into.Add(new Segment(surface, d[from], d[to] - d[from], sx, sy, sz));
    }

    private static double DegreePrice(List<Segment> segments)
    {
        double best = 0.0;
        foreach (Segment s in segments)
            best = Math.Max(best, Math.Sqrt(s.SlopeX * s.SlopeX + s.SlopeY * s.SlopeY + s.SlopeZ * s.SlopeZ));
        return DegreePriceShare * (best > 0.0 ? best : 1.0);
    }

    /// <summary>
    /// The deflections for a torque demand in N m, written into <paramref name="deflectionDeg"/> (Count long). <paramref name="priority"/> weights each axis's shortfall: the axis with the smallest weight gives way first when the demand is out of reach. Every surface from rest over its whole range.
    /// </summary>
    public Result Allocate(double demandX, double demandY, double demandZ,
                           ReadOnlySpan<double> priority, Span<double> deflectionDeg)
        => Allocate(demandX, demandY, demandZ, priority, deflectionDeg, default, default, default, default, default);

    /// <summary>
    /// The deflections for a torque demand in N m, as <see cref="Allocate(double, double, double, ReadOnlySpan{double}, Span{double})"/>, from where each surface is now and as far as it can go this step, alongside a backup that shares the demand.
    ///
    /// FROM WHERE THEY ARE. <paramref name="currentDeg"/> is each surface's deflection now (empty: all at rest), and <paramref name="maxStepDeg"/> how far each can move before the next call, its rate times the step (empty or not positive: anywhere in its range). A surface is only ever asked for a deflection it reaches by the next call, so what this reports it gives is what it does give then, and whatever shares the demand is told the truth. The curve is split at its swept deflections, at rest and at the reach, outward from the present deflection each way.
    ///
    /// STICKY. Each degree moved costs <see cref="ChangePriceShare"/> of the best slope, on top of the price of a degree away from rest (<see cref="DegreePriceShare"/>, earned back moving toward rest): a surface stays where it is unless moving buys torque, and a demand that four surfaces can meet many ways does not swap between them from one call to the next. A surface the demand does not need goes home at its rate, since a degree toward rest is worth more than a degree moved costs.
    ///
    /// THE BACKUP (optional): <paramref name="backupTorque"/> (three per column) and <paramref name="backupCost"/>, columns between 0 and 1 that give torque at a price in the shortfall's units (one per N m at priority 1) - the RCS thrusters at full duty, priced by their propellant. The surfaces and the backup share the demand in one programme, so the surfaces take whatever axis the backup finds dear, rather than what a fixed priority leaves them. Their duties go to <paramref name="backupDuty"/>; the result reports the surfaces' torque alone.
    /// </summary>
    public Result Allocate(double demandX, double demandY, double demandZ,
                           ReadOnlySpan<double> priority, Span<double> deflectionDeg,
                           ReadOnlySpan<double> currentDeg, ReadOnlySpan<double> maxStepDeg,
                           ReadOnlySpan<double> backupTorque, ReadOnlySpan<double> backupCost, Span<double> backupDuty)
    {
        deflectionDeg[..Count].Clear();
        int m = backupCost.Length;
        if (backupTorque.Length < 3 * m)
            throw new ArgumentException("The backup needs three torques per column.");
        if (backupDuty.Length >= m)
            backupDuty[..m].Clear();

        // The present deflections, and how far each surface reaches from them by the next call.
        var current = new double[Count];
        var lo = new double[Count];
        var hi = new double[Count];
        for (int i = 0; i < Count; i++)
        {
            double[] d = _deflection[i];
            current[i] = Math.Clamp(currentDeg.Length > i && double.IsFinite(currentDeg[i]) ? currentDeg[i] : 0.0, d[0], d[^1]);
            double step = maxStepDeg.Length > i && maxStepDeg[i] > 0.0 && double.IsFinite(maxStepDeg[i]) ? maxStepDeg[i] : double.PositiveInfinity;
            lo[i] = Math.Max(d[0], current[i] - step);
            hi[i] = Math.Min(d[^1], current[i] + step);
        }
        double degree = DegreePrice(Segments());
        double change = degree * ChangePriceShare / DegreePriceShare;

        // REFINED FROM ITS OWN ANSWER. The first pass is a linear programme over each curve's pieces from the present deflection, straightened where they steepen (AddDirection), exact wherever the curves bend the usual way. Where they do not, its answer can count on torque the curves do not give, and from a deflection on a bend it can even fill the steep piece one way and the shallow one back, moving nowhere for torque it does not get (2026-11-20: 2stage_KSAero's fins at a 45 deg range, the drag of a deflected fin growing faster outward, 48 of 51 kN m of pitch with 12 of roll and 14 of yaw nobody asked for). So it is refined: passes with one straight slope per surface across a window round the best answer yet - the same slope either way, nothing to play off - kept when the true cost, off the curves, falls, and with the window halved when it does not.
        var start = (double[])current.Clone();
        var best = new double[Count];
        var bestDuty = new double[m];
        var trial = new double[Count];
        var trialDuty = new double[m];
        double bestCost = double.PositiveInfinity, bestMiss = double.PositiveInfinity;
        double size = Math.Sqrt(demandX * demandX + demandY * demandY + demandZ * demandZ);
        double window = RefineWindowDeg;
        for (int pass = 0; pass < MaxPasses && window > RefineWindowMinDeg; pass++)
        {
            bool pieces = pass == 0;
            if (!SolvePass(demandX, demandY, demandZ, priority, pieces ? current : best, current, lo, hi, degree, change,
                           backupTorque, backupCost, trial, trialDuty, out double plannedX, out double plannedY, out double plannedZ,
                           pieces ? 0.0 : window))
            {
                if (pieces)
                    return default;
                window *= 0.5;
                continue;
            }
            Result r = Evaluate(trial, 0.0, 0.0, 0.0);
            double cost = TrueCost(demandX, demandY, demandZ, priority, r, trial, current, degree, change, backupTorque, backupCost, trialDuty);
            double miss = Math.Sqrt((plannedX - r.TorqueX) * (plannedX - r.TorqueX) + (plannedY - r.TorqueY) * (plannedY - r.TorqueY)
                                    + (plannedZ - r.TorqueZ) * (plannedZ - r.TorqueZ));
            if (double.IsPositiveInfinity(bestCost) || cost < bestCost - 1e-9 * (1.0 + Math.Abs(bestCost)))
            {
                bestCost = cost;
                bestMiss = miss;
                Array.Copy(trial, best, Count);
                Array.Copy(trialDuty, bestDuty, m);
            }
            else if (!pieces)
                window *= 0.5;
            // Done once the best answer's plan is what the curves give.
            if (bestMiss <= 1.0 + 5e-3 * size)
                break;
        }
        best.CopyTo(deflectionDeg);
        if (backupDuty.Length >= m)
            bestDuty.CopyTo(backupDuty);
        return Evaluate(deflectionDeg, demandX, demandY, demandZ, solved: true);
    }

    /// <summary>The refinement in Allocate: at most this many passes; the first window either way of the best answer, and the smallest, degrees.</summary>
    private const int MaxPasses = 12;
    private const double RefineWindowDeg = 10.0, RefineWindowMinDeg = 0.05;

    /// <summary>
    /// One pass of Allocate: the linear programme over the pieces from <paramref name="start"/>, within [lo, hi], each degree priced against <paramref name="current"/> (change) and against rest (degree). The deflections into <paramref name="deflection"/>, the backup's duties into <paramref name="duty"/>, and the surfaces' torque the programme counted on.
    /// </summary>
    private bool SolvePass(double demandX, double demandY, double demandZ, ReadOnlySpan<double> priority,
                           double[] start, double[] current, double[] lo, double[] hi, double degree, double change,
                           ReadOnlySpan<double> backupTorque, ReadOnlySpan<double> backupCost,
                           double[] deflection, double[] duty, out double plannedX, out double plannedY, out double plannedZ,
                           double windowDeg = 0.0)
    {
        int m = backupCost.Length;
        double baseX = 0.0, baseY = 0.0, baseZ = 0.0;
        for (int i = 0; i < Count; i++)
        {
            At(i, start[i], out double tx, out double ty, out double tz, out _, out _, out _);
            baseX += tx; baseY += ty; baseZ += tz;
        }
        plannedX = baseX; plannedY = baseY; plannedZ = baseZ;
        List<Segment> segments = windowDeg > 0.0 ? SecantsWithin(start, lo, hi, windowDeg) : SegmentsWithin(start, lo, hi);
        int n = segments.Count;

        // Columns: n segments, n slacks, m backups, m slacks, six shortfalls. Rows: three torque, n widths, m bounds.
        int rows = 3 + n + m, cols = 2 * n + 2 * m + 6;
        var a = new double[rows * cols];
        var cost = new double[cols];
        var b = new double[rows];
        b[0] = demandX - baseX; b[1] = demandY - baseY; b[2] = demandZ - baseZ;
        for (int j = 0; j < n; j++)
        {
            Segment s = segments[j];
            a[j * rows] = s.SlopeX; a[j * rows + 1] = s.SlopeY; a[j * rows + 2] = s.SlopeZ;
            a[j * rows + 3 + j] = 1.0;
            a[(n + j) * rows + 3 + j] = 1.0;
            b[3 + j] = Math.Abs(s.Width);
            // Away from rest the segment's far end is further out than its near end, and the segments never cross rest; likewise for where the surface is now, which a refinement pass may have moved away from.
            bool outward = Math.Abs(s.From + s.Width) > Math.Abs(s.From);
            bool away = Math.Abs(s.From + s.Width - current[s.Surface]) > Math.Abs(s.From - current[s.Surface]);
            cost[j] = (away ? change : -change) + (outward ? degree : -degree);
        }
        for (int k = 0; k < m; k++)
        {
            int col = 2 * n + k, slack = 2 * n + m + k, row = 3 + n + k;
            bool usable = double.IsFinite(backupCost[k]) && backupCost[k] >= 0.0;
            for (int r = 0; r < 3; r++)
                a[col * rows + r] = usable && double.IsFinite(backupTorque[3 * k + r]) ? backupTorque[3 * k + r] : 0.0;
            a[col * rows + row] = 1.0;
            a[slack * rows + row] = 1.0;
            b[row] = usable ? 1.0 : 0.0;
            cost[col] = usable ? backupCost[k] : 0.0;
        }
        int sf = 2 * n + 2 * m;
        for (int r = 0; r < 3; r++)
        {
            double w = priority.Length > r && priority[r] > 0.0 ? priority[r] : 1.0;
            a[(sf + 2 * r) * rows + r] = 1.0;
            a[(sf + 2 * r + 1) * rows + r] = -1.0;
            cost[sf + 2 * r] = w;
            cost[sf + 2 * r + 1] = w;
        }

        double[]? x = RcsLpSolver.Solve(rows, cols, a, cost, b);
        if (x == null)
            return false;
        Array.Copy(start, deflection, Count);
        for (int j = 0; j < n; j++)
        {
            Segment s = segments[j];
            double travel = Math.Clamp(x[j], 0.0, Math.Abs(s.Width));
            deflection[s.Surface] += Math.Sign(s.Width) * travel;
            plannedX += s.SlopeX * travel; plannedY += s.SlopeY * travel; plannedZ += s.SlopeZ * travel;
        }
        for (int i = 0; i < Count; i++)
        {
            deflection[i] = Math.Clamp(deflection[i], lo[i], hi[i]);
            if (Math.Abs(deflection[i]) < DeflectionFloorDeg)
                deflection[i] = 0.0;
        }
        for (int k = 0; k < m; k++)
            duty[k] = Math.Clamp(x[2 * n + k], 0.0, 1.0);
        return true;
    }

    /// <summary>What a set of deflections and duties really costs, off the curves, in the programme's own prices.</summary>
    private double TrueCost(double demandX, double demandY, double demandZ, ReadOnlySpan<double> priority, Result surfaces,
                            double[] deflection, double[] current, double degree, double change,
                            ReadOnlySpan<double> backupTorque, ReadOnlySpan<double> backupCost, double[] duty)
    {
        double tx = surfaces.TorqueX, ty = surfaces.TorqueY, tz = surfaces.TorqueZ, cost = 0.0;
        for (int k = 0; k < backupCost.Length; k++)
        {
            if (!(double.IsFinite(backupCost[k]) && backupCost[k] >= 0.0))
                continue;
            tx += backupTorque[3 * k] * duty[k]; ty += backupTorque[3 * k + 1] * duty[k]; tz += backupTorque[3 * k + 2] * duty[k];
            cost += backupCost[k] * duty[k];
        }
        double w0 = priority.Length > 0 && priority[0] > 0.0 ? priority[0] : 1.0;
        double w1 = priority.Length > 1 && priority[1] > 0.0 ? priority[1] : 1.0;
        double w2 = priority.Length > 2 && priority[2] > 0.0 ? priority[2] : 1.0;
        cost += w0 * Math.Abs(demandX - tx) + w1 * Math.Abs(demandY - ty) + w2 * Math.Abs(demandZ - tz);
        for (int i = 0; i < Count; i++)
            cost += degree * Math.Abs(deflection[i]) + change * Math.Abs(deflection[i] - current[i]);
        return cost;
    }

    /// <summary>
    /// The segments each surface can move along from <paramref name="start"/> within [lo, hi], its reach by the next call, split at its swept deflections and at rest, each starting at the end nearer the start.
    /// </summary>
    private List<Segment> SegmentsWithin(double[] start, double[] los, double[] his)
    {
        var segments = new List<Segment>();
        var points = new List<double>();
        for (int i = 0; i < Count; i++)
        {
            double[] d = _deflection[i];
            double lo = los[i], hi = his[i];
            double c = Math.Clamp(start[i], lo, hi);
            points.Clear();
            points.Add(c); points.Add(lo); points.Add(hi);
            foreach (double p in d)
                if (p > lo && p < hi)
                    points.Add(p);
            if (0.0 > lo && 0.0 < hi)
                points.Add(0.0);
            points.Sort();
            int ci = points.IndexOf(c);
            var up = new List<double>();
            for (int k = ci; k < points.Count; k++)
                up.Add(points[k]);
            var down = new List<double>();
            for (int k = ci; k >= 0; k--)
                down.Add(points[k]);
            AddDirection(segments, i, up);
            AddDirection(segments, i, down);
        }
        return segments;
    }

    /// <summary>
    /// The refinement's pieces: for each surface one straight slope, the curve's chord across a window either way of <paramref name="start"/> within [lo, hi], used for a move up or down alike.
    /// </summary>
    private List<Segment> SecantsWithin(double[] start, double[] los, double[] his, double windowDeg)
    {
        var segments = new List<Segment>();
        for (int i = 0; i < Count; i++)
        {
            double c = Math.Clamp(start[i], los[i], his[i]);
            double a = Math.Max(los[i], c - windowDeg), b = Math.Min(his[i], c + windowDeg);
            if (!(b - a > 1e-9))
                continue;
            At(i, a, out double ax, out double ay, out double az, out _, out _, out _);
            At(i, b, out double bx, out double by, out double bz, out _, out _, out _);
            double w = b - a;
            double sx = (bx - ax) / w, sy = (by - ay) / w, sz = (bz - az) / w;
            if (!double.IsFinite(sx) || !double.IsFinite(sy) || !double.IsFinite(sz))
                continue;
            if (b - c > 1e-9)
                segments.Add(new Segment(i, c, b - c, sx, sy, sz));
            if (c - a > 1e-9)
                segments.Add(new Segment(i, c, a - c, -sx, -sy, -sz));
        }
        return segments;
    }

    /// <summary>
    /// One direction's pieces from the start outward (<paramref name="path"/>, start first), STRAIGHTENED WHERE THEY STEEPEN: a piece steeper than the one inside it is merged with it into one straight piece across both, and again with the one inside that, so the pieces given to the programme only get shallower outward. A programme then fills them inside first, as a real surface must travel, instead of taking a steep outer piece alone and adding up to a deflection that gives less than it counted on. The straight piece is a chord of the curve, so it promises a little more than the curve gives on its way out; the next refinement pass, from where this one ended, takes up the difference.
    /// </summary>
    private void AddDirection(List<Segment> into, int surface, List<double> path)
    {
        var pieces = new List<(double From, double To)>();
        for (int k = 0; k + 1 < path.Count; k++)
        {
            if (!(Math.Abs(path[k + 1] - path[k]) > 1e-9))
                continue;
            (double From, double To) piece = (path[k], path[k + 1]);
            while (pieces.Count > 0 && SlopeNorm(surface, piece.From, piece.To) > SlopeNorm(surface, pieces[^1].From, pieces[^1].To) * (1.0 + 1e-9))
            {
                piece = (pieces[^1].From, piece.To);
                pieces.RemoveAt(pieces.Count - 1);
            }
            pieces.Add(piece);
        }
        foreach ((double from, double to) in pieces)
            AddSegmentAt(into, surface, from, to);
    }

    private double SlopeNorm(int surface, double from, double to)
    {
        At(surface, from, out double ax, out double ay, out double az, out _, out _, out _);
        At(surface, to, out double bx, out double by, out double bz, out _, out _, out _);
        double w = Math.Abs(to - from);
        return w > 0.0 ? Math.Sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay) + (bz - az) * (bz - az)) / w : 0.0;
    }

    private void AddSegmentAt(List<Segment> into, int surface, double from, double to)
    {
        double width = Math.Abs(to - from);
        if (!(width > 1e-9))
            return;
        At(surface, from, out double ax, out double ay, out double az, out _, out _, out _);
        At(surface, to, out double bx, out double by, out double bz, out _, out _, out _);
        double sx = (bx - ax) / width, sy = (by - ay) / width, sz = (bz - az) / width;
        if (!double.IsFinite(sx) || !double.IsFinite(sy) || !double.IsFinite(sz))
            return;
        into.Add(new Segment(surface, from, to - from, sx, sy, sz));
    }

    /// <summary>
    /// What the surfaces can do about axis <paramref name="axis"/> (0, 1, 2 of the frame) in the direction <paramref name="sign"/>: the largest torque with the other two axes held at zero, and the force per newton metre of the deflections for half of it.
    /// </summary>
    public AxisAuthority Characterise(int axis, int sign)
    {
        // From rest over each surface's whole range, on the straightened pieces Allocate uses (AddDirection), so the reach is one the surfaces can travel to, inside piece first.
        var rest = new double[Count];
        var los = new double[Count];
        var his = new double[Count];
        for (int i = 0; i < Count; i++)
        {
            los[i] = _deflection[i][0];
            his[i] = _deflection[i][^1];
        }
        List<Segment> segments = SegmentsWithin(rest, los, his);
        int n = segments.Count;
        if (n == 0)
            return default;
        // Columns: n segments, n slacks, the torque t. Rows: three torque, n widths. Maximise t, with a whisper of deflection price so a tie goes to less deflection.
        int rows = 3 + n, cols = 2 * n + 1;
        var a = new double[rows * cols];
        var cost = new double[cols];
        var b = new double[rows];
        double tie = 1.0e-3 * DegreePrice(segments);
        for (int j = 0; j < n; j++)
        {
            Segment s = segments[j];
            a[j * rows] = s.SlopeX; a[j * rows + 1] = s.SlopeY; a[j * rows + 2] = s.SlopeZ;
            a[j * rows + 3 + j] = 1.0;
            a[(n + j) * rows + 3 + j] = 1.0;
            b[3 + j] = Math.Abs(s.Width);
            cost[j] = tie;
        }
        a[2 * n * rows + axis] = -sign;
        cost[2 * n] = -1.0;

        double[]? x = RcsLpSolver.Solve(rows, cols, a, cost, b);
        double max = x != null ? Math.Max(x[2 * n], 0.0) : 0.0;
        if (!(max > 0.0) || !double.IsFinite(max))
            return default;

        Span<double> demand = stackalloc double[3];
        demand[axis] = 0.5 * sign * max;
        Span<double> deflection = Count <= 64 ? stackalloc double[Count] : new double[Count];
        Span<double> even = stackalloc double[3] { 1.0, 1.0, 1.0 };
        Result half = Allocate(demand[0], demand[1], demand[2], even, deflection);
        double given = 0.5 * max;
        if (!half.Solved || half.ShortfallNm > 0.05 * given)
            return new AxisAuthority(max, 0.0, 0.0, 0.0);
        return new AxisAuthority(max, half.ForceX / given, half.ForceY / given, half.ForceZ / given);
    }

    /// <summary>Surface <paramref name="i"/>'s torque and force at a deflection, read straight between its swept deflections and held at its ends.</summary>
    public void At(int i, double deflectionDeg, out double tx, out double ty, out double tz, out double fx, out double fy, out double fz)
    {
        double[] d = _deflection[i];
        double[] t = _torque[i];
        double[] f = _force[i];
        int n = d.Length;
        double x = Math.Clamp(deflectionDeg, d[0], d[n - 1]);
        int k = 0;
        while (k + 2 < n && x > d[k + 1])
            k++;
        int k1 = Math.Min(k + 1, n - 1);
        double w = d[k1] > d[k] ? (x - d[k]) / (d[k1] - d[k]) : 0.0;
        tx = t[3 * k] + w * (t[3 * k1] - t[3 * k]);
        ty = t[3 * k + 1] + w * (t[3 * k1 + 1] - t[3 * k + 1]);
        tz = t[3 * k + 2] + w * (t[3 * k1 + 2] - t[3 * k + 2]);
        fx = f[3 * k] + w * (f[3 * k1] - f[3 * k]);
        fy = f[3 * k + 1] + w * (f[3 * k1 + 1] - f[3 * k + 1]);
        fz = f[3 * k + 2] + w * (f[3 * k1 + 2] - f[3 * k + 2]);
    }

    /// <summary>What a set of deflections gives, off each surface's curve, against a demand.</summary>
    public Result Evaluate(ReadOnlySpan<double> deflectionDeg, double demandX, double demandY, double demandZ, bool solved = true)
    {
        double tx = 0, ty = 0, tz = 0, fx = 0, fy = 0, fz = 0;
        int deflected = 0;
        for (int i = 0; i < Count; i++)
        {
            if (deflectionDeg[i] != 0.0)
                deflected++;
            At(i, deflectionDeg[i], out double ax, out double ay, out double az, out double bx, out double by, out double bz);
            tx += ax; ty += ay; tz += az;
            fx += bx; fy += by; fz += bz;
        }
        double sx = demandX - tx, sy = demandY - ty, sz = demandZ - tz;
        return new Result(solved, tx, ty, tz, fx, fy, fz, Math.Sqrt(sx * sx + sy * sy + sz * sz), deflected);
    }
}
