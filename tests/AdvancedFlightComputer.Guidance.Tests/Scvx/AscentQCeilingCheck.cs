using AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// The relaxed max q a planner retries at stays under a ceiling of 98 % of the 200 kPa at which the game breaks a vehicle up. The numbers are those of a 108 g single stage under a 24.4 g limit (#120): its steepest seed reaches 235.7 kPa, so the rule without a ceiling retried at 275 kPa, and a plan solved there peaked at 209.7 kPa. Under the ceiling the rule is the same as without one.
/// </summary>
internal static class AscentQCeilingCheck
{
    private const double Breakup = 200000.0;
    private const double Ceiling = 0.98 * Breakup;
    private const double SeedLeastMaxQ = 235700.0;

    private static int _fails;

    private static void Check(string name, bool ok, string detail = "")
    {
        Console.WriteLine($"  {(ok ? "PASS" : "FAIL")}  {name,-66} {detail}");
        if (!ok) _fails++;
    }

    /// <summary>The rule as it was without a ceiling, Pa.</summary>
    private static double Uncapped(double seed) => Math.Ceiling(AscentMaxQ.RelaxFactor * seed / AscentMaxQ.Step) * AscentMaxQ.Step;

    internal static int Run(bool verbose)
    {
        _fails = 0;
        Console.WriteLine("ASCENT MAX-Q CEILING: a relaxed retry stays under the break-up dynamic pressure (#120)");

        double highest = AscentMaxQ.Highest(Ceiling);
        Check("the highest max q is the ceiling rounded down to a step", highest == 195000.0, $"{highest / 1000.0:F0} kPa");

        double capped = AscentMaxQ.Relaxed(SeedLeastMaxQ, Ceiling);
        double open = AscentMaxQ.Relaxed(SeedLeastMaxQ, double.PositiveInfinity);
        Check("without a ceiling the retry passes the break-up", open == 275000.0 && open == Uncapped(SeedLeastMaxQ), $"{open / 1000.0:F0} kPa");
        Check("with one it stays under the ceiling", capped == 195000.0 && capped < Ceiling, $"{capped / 1000.0:F0} kPa against {Ceiling / 1000.0:F0}");
        Check("a solve already at the highest max q is not retried", !(capped > highest), $"retry {capped / 1000.0:F0} kPa, tried {highest / 1000.0:F0}");
        Check("a 1 kPa solve whose seed reaches 24 kPa is retried at 30 kPa", AscentMaxQ.Relaxed(24000.0, Ceiling) == 30000.0,
              $"{AscentMaxQ.Relaxed(24000.0, Ceiling) / 1000.0:F0} kPa");

        int same = 0, cut = 0, wrong = 0;
        for (int i = 10; i <= 4000; i++)
        {
            double seed = i * 100.0;
            double got = AscentMaxQ.Relaxed(seed, Ceiling);
            double want = Math.Min(Uncapped(seed), highest);
            if (got != want || got > highest)
                wrong++;
            else if (Uncapped(seed) <= highest)
                same++;
            else
                cut++;
            if (verbose && i % 250 == 0)
                Console.WriteLine($"    seed {seed / 1000.0,6:F1} kPa: retry {got / 1000.0:F0} kPa (without a ceiling {Uncapped(seed) / 1000.0:F0})");
        }
        Check("from 1 to 400 kPa: the old retry under the ceiling, the ceiling above", wrong == 0 && same > 0 && cut > 0,
              $"{same} unchanged, {cut} held at {highest / 1000.0:F0} kPa, {wrong} wrong");

        Console.WriteLine();
        Console.WriteLine(_fails == 0 ? "ALL PASS" : $"{_fails} FAILED");
        return _fails == 0 ? 0 : 1;
    }
}
