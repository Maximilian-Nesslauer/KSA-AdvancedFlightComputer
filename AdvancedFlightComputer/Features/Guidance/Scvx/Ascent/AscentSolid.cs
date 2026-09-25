namespace AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// The solid motors burning through one planned stage, as a table against the stage's own clock: their summed mass flow, and their summed thrust at each of a few back pressures. See issue #73.
///
/// A TABLE, NOT THE BURN MODEL. KSA's burn model is a chain of piecewise-linear float tables (grain perimeter against web depth, combustion products against chamber pressure), a fixed-point solve for chamber pressure and a flow-separation branch in the nozzle. Its slope jumps at every table point, and nothing the plan decides reaches it: the chamber-pressure solve runs the nozzle at zero back pressure, since the throat is choked, so the grain burns back as a fixed function of time since ignition. Only the nozzle exit sees the air. So the game samples its own model once, and the dynamics differentiate this table's interpolant in back pressure only.
///
/// THE STAGE'S DURATION IS PINNED to <see cref="AscentStage.FixedBurnTime"/> whenever a solid burns in it, so each node sits at a fixed second of the table and time never needs differentiating. That is also why the table's slope jumps (a star grain's points burning out, boost turning to sustain) cost nothing: they are only ever read at the nodes and interpolated between them for the seed.
///
/// PRESSURE IS AN AXIS, NOT A FACTOR on a time shape: an overexpanded nozzle separates at low chamber pressure, so how much thrust the air costs changes over the burn (by 7 to 36 % of the mean at sea level across KSA's five grains). One column, at zero, is an airless body.
/// </summary>
public sealed class AscentSolidBurn
{
    /// <param name="time">Seconds from the stage's start, strictly increasing from 0.</param>
    /// <param name="massFlow">Summed mass flow at each time, kg/s.</param>
    /// <param name="pressureGrid">Strictly increasing back pressures, Pa; a single pressure for a table with no air.</param>
    /// <param name="thrust">Summed thrust, N, row-major: one row per time, one column per grid pressure.</param>
    public AscentSolidBurn(double[] time, double[] massFlow, double[] pressureGrid, double[] thrust)
    {
        ArgumentNullException.ThrowIfNull(time);
        ArgumentNullException.ThrowIfNull(massFlow);
        ArgumentNullException.ThrowIfNull(pressureGrid);
        ArgumentNullException.ThrowIfNull(thrust);
        if (time.Length < 2 || time[0] != 0.0)
            throw new ArgumentException("At least two samples, starting at zero.", nameof(time));
        for (int j = 1; j < time.Length; j++)
            if (!(time[j] > time[j - 1]) || !double.IsFinite(time[j]))
                throw new ArgumentException("Times must be finite and strictly increasing.", nameof(time));
        if (massFlow.Length != time.Length)
            throw new ArgumentException("One mass flow per time.", nameof(massFlow));
        foreach (double m in massFlow)
            if (!(m >= 0.0) || !double.IsFinite(m))
                throw new ArgumentException("Mass flows must be finite and non-negative.", nameof(massFlow));
        if (pressureGrid.Length < 1)
            throw new ArgumentException("At least one pressure.", nameof(pressureGrid));
        for (int g = 1; g < pressureGrid.Length; g++)
            if (!(pressureGrid[g] > pressureGrid[g - 1]))
                throw new ArgumentException("Pressures must be strictly increasing.", nameof(pressureGrid));
        if (thrust.Length != time.Length * pressureGrid.Length)
            throw new ArgumentException("One thrust per time and pressure.", nameof(thrust));
        foreach (double f in thrust)
            if (!(f >= 0.0) || !double.IsFinite(f))
                throw new ArgumentException("Thrusts must be finite and non-negative.", nameof(thrust));

        Time = (double[])time.Clone();
        MassFlow = (double[])massFlow.Clone();
        PressureGrid = (double[])pressureGrid.Clone();
        Thrust = (double[])thrust.Clone();
    }

    public double[] Time { get; }
    public double[] MassFlow { get; }
    public double[] PressureGrid { get; }
    public double[] Thrust { get; }

    public int Pressures => PressureGrid.Length;

    /// <summary>Seconds the table covers.</summary>
    public double Duration => Time[^1];

    /// <summary>Propellant burned between two stage times, kg: the trapezoid over the table, which is how the table was sampled.</summary>
    public double Propellant(double from, double to)
    {
        from = Math.Clamp(from, 0.0, Duration);
        to = Math.Clamp(to, 0.0, Duration);
        if (!(to > from))
            return 0.0;
        double sum = 0.0;
        for (int j = 0; j < Time.Length - 1; j++)
        {
            double a = Math.Max(Time[j], from), b = Math.Min(Time[j + 1], to);
            if (!(b > a))
                continue;
            sum += 0.5 * (MassFlowAt(a) + MassFlowAt(b)) * (b - a);
        }
        return sum;
    }

    /// <summary>Mass flow at a stage time, kg/s, linear between samples and held past the ends.</summary>
    public double MassFlowAt(double t)
    {
        Locate(t, out int lo, out int hi, out double f);
        return MassFlow[lo] + f * (MassFlow[hi] - MassFlow[lo]);
    }

    /// <summary>Thrust at each grid pressure at a stage time, N, linear between samples.</summary>
    public void ThrustRow(double t, Span<double> row)
    {
        Locate(t, out int lo, out int hi, out double f);
        int np = Pressures;
        for (int g = 0; g < np; g++)
            row[g] = Thrust[lo * np + g] + f * (Thrust[hi * np + g] - Thrust[lo * np + g]);
    }

    /// <summary>Mean thrust at the first grid pressure over the whole table, N: the reference the dynamics scale their regularisation by.</summary>
    public double MeanThrust()
    {
        int np = Pressures;
        double sum = 0.0;
        for (int j = 0; j < Time.Length - 1; j++)
            sum += 0.5 * (Thrust[j * np] + Thrust[(j + 1) * np]) * (Time[j + 1] - Time[j]);
        return sum / Duration;
    }

    /// <summary>Total impulse at the first grid pressure over a stretch of stage time, N s.</summary>
    public double Impulse(double from, double to)
    {
        from = Math.Clamp(from, 0.0, Duration);
        to = Math.Clamp(to, 0.0, Duration);
        int np = Pressures;
        double sum = 0.0;
        Span<double> ra = stackalloc double[np];
        Span<double> rb = stackalloc double[np];
        for (int j = 0; j < Time.Length - 1; j++)
        {
            double a = Math.Max(Time[j], from), b = Math.Min(Time[j + 1], to);
            if (!(b > a))
                continue;
            ThrustRow(a, ra);
            ThrustRow(b, rb);
            sum += 0.5 * (ra[0] + rb[0]) * (b - a);
        }
        return sum;
    }

    private void Locate(double t, out int lo, out int hi, out double f)
    {
        int n = Time.Length;
        if (t <= 0.0) { lo = hi = 0; f = 0.0; return; }
        if (t >= Time[n - 1]) { lo = hi = n - 1; f = 0.0; return; }
        hi = Array.BinarySearch(Time, t);
        if (hi >= 0) { lo = hi; f = 0.0; return; }
        hi = ~hi;
        lo = hi - 1;
        f = (t - Time[lo]) / (Time[hi] - Time[lo]);
    }
}
