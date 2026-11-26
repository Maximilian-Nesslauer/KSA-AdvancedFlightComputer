namespace AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;

/// <summary>
/// The angle-of-attack schedule an orbital entry flies: at each Mach number, the angle where the pitch moment is zero and restoring (C_m falls through zero as alpha rises), so the vehicle sits in pitch equilibrium and its controls only hold it there. Drag and lift over drag are then read along that schedule, which is all the planner needs (<see cref="EntryModel"/>).
///
/// Alpha is measured from nose-first (0) towards belly-first (90). Where a Mach row has no restoring zero inside the allowed band, the schedule takes the angle of least pitch moment in the band instead and marks the row untrimmed: the controls then hold the smallest moment there is to hold (the user's rule, 2026-11-21: it need not be an exact trim, only the minimum-moment point).
/// </summary>
public sealed class EntryTrim
{
    public required double[] Mach { get; init; }
    public required double[] AlphaDeg { get; init; }
    public required double[] Cd { get; init; }
    public required double[] LiftToDrag { get; init; }
    public required bool[] Trimmed { get; init; }

    /// <summary>The pitch moment coefficient left at each row's angle: zero where trimmed, the least in the band otherwise.</summary>
    public double[] ResidualCm { get; init; } = [];

    public int TrimmedRows => Trimmed.Count(t => t);

    /// <summary>
    /// Build from one belly axis's sweep, flat arrays indexed [mach * alphas + alpha].
    /// </summary>
    public static EntryTrim Build(double[] machGrid, double[] alphaDeg, double[] cd, double[] cl, double[] cm,
                                  double alphaMinDeg, double alphaMaxDeg, double preferredDeg)
    {
        int nm = machGrid.Length, na = alphaDeg.Length;
        var aOut = new double[nm];
        var cdOut = new double[nm];
        var ldOut = new double[nm];
        var trimmed = new bool[nm];
        var residual = new double[nm];
        for (int i = 0; i < nm; i++)
        {
            double best = double.NaN;
            for (int j = 0; j < na - 1; j++)
            {
                double a0 = alphaDeg[j], a1 = alphaDeg[j + 1];
                double m0 = cm[i * na + j], m1 = cm[i * na + j + 1];
                if (!(m0 > 0 && m1 <= 0)) continue;          // restoring: the moment falls through zero
                double a = a0 + (a1 - a0) * m0 / (m0 - m1);
                if (a < alphaMinDeg || a > alphaMaxDeg) continue;
                if (double.IsNaN(best) || Math.Abs(a - preferredDeg) < Math.Abs(best - preferredDeg)) best = a;
            }
            trimmed[i] = !double.IsNaN(best);
            double at = trimmed[i] ? best : LeastMoment(alphaDeg, cm, i, na, alphaMinDeg, alphaMaxDeg, preferredDeg);
            residual[i] = trimmed[i] ? 0.0 : Row(alphaDeg, cm, i, na, at);
            aOut[i] = at;
            double cdi = Row(alphaDeg, cd, i, na, at);
            double cli = Row(alphaDeg, cl, i, na, at);
            cdOut[i] = Math.Max(cdi, 1e-4);
            ldOut[i] = cli / cdOut[i];
        }
        return new EntryTrim { Mach = (double[])machGrid.Clone(), AlphaDeg = aOut, Cd = cdOut, LiftToDrag = ldOut, Trimmed = trimmed, ResidualCm = residual };
    }

    /// <summary>The angle of least |C_m| in the band, read every half degree; a near tie goes to the angle nearer the preferred one.</summary>
    static double LeastMoment(double[] alphaDeg, double[] cm, int i, int na, double lo, double hi, double preferredDeg)
    {
        double best = Math.Clamp(preferredDeg, lo, hi), bestScore = double.PositiveInfinity;
        for (double a = lo; a <= hi + 1e-9; a += 0.5)
        {
            double score = Math.Abs(Row(alphaDeg, cm, i, na, a)) + 1e-6 * Math.Abs(a - preferredDeg);
            if (score < bestScore) { bestScore = score; best = a; }
        }
        return best;
    }

    /// <summary>
    /// The schedule at given angles, one per Mach row (degrees from nose-first), with drag and lift over drag read from the same sweep: the flap-aware trim (GuidanceWindow.RetrimWithFlaps) picks the angles, this reads the rest. <paramref name="trimmed"/> marks the rows whose controls can hold the moment there.
    /// </summary>
    public static EntryTrim AtAngles(double[] machGrid, double[] alphaDeg, double[] cd, double[] cl, double[] cm, double[] anglesDeg, bool[] trimmed, double[] residualCm)
    {
        int nm = machGrid.Length, na = alphaDeg.Length;
        var cdOut = new double[nm];
        var ldOut = new double[nm];
        for (int i = 0; i < nm; i++)
        {
            cdOut[i] = Math.Max(Row(alphaDeg, cd, i, na, anglesDeg[i]), 1e-4);
            ldOut[i] = Row(alphaDeg, cl, i, na, anglesDeg[i]) / cdOut[i];
        }
        return new EntryTrim
        {
            Mach = (double[])machGrid.Clone(), AlphaDeg = (double[])anglesDeg.Clone(), Cd = cdOut, LiftToDrag = ldOut,
            Trimmed = (bool[])trimmed.Clone(), ResidualCm = (double[])residualCm.Clone(),
        };
    }

    /// <summary>A fixed angle of attack at every Mach, from the same sweep.</summary>
    public static EntryTrim Fixed(double[] machGrid, double[] alphaDeg, double[] cd, double[] cl, double fixedDeg)
    {
        int nm = machGrid.Length, na = alphaDeg.Length;
        var aOut = new double[nm];
        var cdOut = new double[nm];
        var ldOut = new double[nm];
        for (int i = 0; i < nm; i++)
        {
            aOut[i] = fixedDeg;
            cdOut[i] = Math.Max(Row(alphaDeg, cd, i, na, fixedDeg), 1e-4);
            ldOut[i] = Row(alphaDeg, cl, i, na, fixedDeg) / cdOut[i];
        }
        return new EntryTrim { Mach = (double[])machGrid.Clone(), AlphaDeg = aOut, Cd = cdOut, LiftToDrag = ldOut, Trimmed = new bool[nm], ResidualCm = new double[nm] };
    }

    static double Row(double[] alphaDeg, double[] table, int i, int na, double a)
    {
        if (a <= alphaDeg[0]) return table[i * na];
        if (a >= alphaDeg[na - 1]) return table[i * na + na - 1];
        int j = 0;
        while (j < na - 2 && alphaDeg[j + 1] < a) j++;
        double f = (a - alphaDeg[j]) / (alphaDeg[j + 1] - alphaDeg[j]);
        return table[i * na + j] + (table[i * na + j + 1] - table[i * na + j]) * f;
    }

    double ByMach(double[] values, double mach)
    {
        int n = Mach.Length;
        if (mach <= Mach[0]) return values[0];
        if (mach >= Mach[n - 1]) return values[n - 1];
        int i = 0;
        while (i < n - 2 && Mach[i + 1] < mach) i++;
        double f = (mach - Mach[i]) / (Mach[i + 1] - Mach[i]);
        return values[i] + (values[i + 1] - values[i]) * f;
    }

    public double AlphaAt(double mach) => ByMach(AlphaDeg, mach);
    public double CdAt(double mach) => ByMach(Cd, mach);
    public double LiftToDragAt(double mach) => ByMach(LiftToDrag, mach);
}
