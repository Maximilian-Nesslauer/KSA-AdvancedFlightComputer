namespace AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;

/// <summary>
/// A solved entry plan on its energy grid, with everything the tracker and the TRAJ display read from it.
///
/// Node arrays are in the planner's normalised units except where the name says otherwise: <see cref="DragMs2"/>, <see cref="SpeedMs"/>, <see cref="AltitudeM"/>, <see cref="HdotMs"/> and the corridor ranges are SI.
/// </summary>
public sealed class EntryPlan
{
    public required double[] E { get; init; }
    public required double[] R { get; init; }
    public required double[] Gamma { get; init; }
    /// <summary>Range to go, rad of arc.</summary>
    public required double[] S { get; init; }
    /// <summary>Vertical lift over drag.</summary>
    public required double[] U { get; init; }

    public required double[] DragMs2 { get; init; }
    public required double[] SpeedMs { get; init; }
    public required double[] AltitudeM { get; init; }
    public required double[] HdotMs { get; init; }
    public required double[] Mach { get; init; }
    public required double[] LiftToDrag { get; init; }

    /// <summary>Drag at the altitude floor: the most the path limits allow, m/s^2.</summary>
    public required double[] DragMaxMs2 { get; init; }
    /// <summary>Drag on the equilibrium glide at the lateral-reserve bank: the least the vehicle can hold, m/s^2.</summary>
    public required double[] DragMinMs2 { get; init; }
    public required double[] FloorAltitudeM { get; init; }

    /// <summary>
    /// Range still reachable from each node to the hand-over, m: flying the most drag (shortest) and the least (longest). These are the TRAJ display's high- and low-energy lines.
    /// </summary>
    public required double[] RangeShortM { get; init; }
    public required double[] RangeLongM { get; init; }

    public required double BodyRadius { get; init; }

    public bool Converged { get; init; }
    public int Iterations { get; init; }
    public double MaxDefect { get; init; }
    public double TerminalRangeErrorM { get; init; }
    public double SolveMs { get; init; }
    public bool Cold { get; init; }
    public string Note { get; init; } = "";

    public int Count => E.Length;
    public double E0 => E[0];
    public double EF => E[^1];

    public readonly record struct Sample(
        double U, double DragMs2, double HdotMs, double R, double Gamma, double S,
        double SpeedMs, double AltitudeM, double LiftToDrag, double BankRefDeg, double Fraction,
        double RangeShortM, double RangeLongM, double DragMaxMs2, double DragMinMs2);

    /// <summary>The plan at an energy, interpolated linearly between nodes and clamped to its ends.</summary>
    public Sample At(double e)
    {
        int n = Count;
        int k;
        double f;
        if (e <= E[0]) { k = 0; f = 0; }
        else if (e >= E[n - 1]) { k = n - 2; f = 1; }
        else
        {
            int lo = 0, hi = n - 1;
            while (hi - lo > 1)
            {
                int mid = (lo + hi) >> 1;
                if (E[mid] <= e) lo = mid; else hi = mid;
            }
            k = lo;
            f = (e - E[k]) / (E[k + 1] - E[k]);
        }
        double L(double[] a) => a[k] + (a[k + 1] - a[k]) * f;
        double u = L(U), ld = L(LiftToDrag);
        double bank = ld > 1e-6 ? Math.Acos(Math.Clamp(u / ld, -1.0, 1.0)) * 180.0 / Math.PI : 0.0;
        double frac = (e - E[0]) / Math.Max(E[n - 1] - E[0], 1e-12);
        return new Sample(u, L(DragMs2), L(HdotMs), L(R), L(Gamma), L(S), L(SpeedMs), L(AltitudeM), ld, bank, Math.Clamp(frac, 0, 1),
                          L(RangeShortM), L(RangeLongM), L(DragMaxMs2), L(DragMinMs2));
    }
}
