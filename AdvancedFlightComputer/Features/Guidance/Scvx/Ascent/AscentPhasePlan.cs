namespace AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// One solid motor's burn from its grain as it is now, sampled from the game's own per-frame model: mass flow, and thrust at each of the planner's back pressures. Time 0 is now, which for a motor not yet lit is its ignition.
/// </summary>
public sealed class AscentSolidMotor
{
    public required string Name { get; init; }
    /// <summary>Seconds from now, strictly increasing from 0, ending where the motor goes out.</summary>
    public required double[] Time { get; init; }
    public required double[] MassFlow { get; init; }
    /// <summary>Thrust, N, row-major: one row per time, one column per grid pressure.</summary>
    public required double[] Thrust { get; init; }
    public double BurnTime => Time[^1];
}

/// <summary>
/// One phase of the game's staging model: a stretch over which the same engines burn, as the drain simulation lays it out at full throttle. A sequence is split into phases wherever an engine starts or stops, so solids burning out under a still-burning core end one.
/// </summary>
public sealed class AscentPhase
{
    /// <summary>Mass at the phase's start and end in the drain model, kg; the gap to the next phase's start is what separates between them.</summary>
    public required double StartMass { get; init; }
    public required double EndMass { get; init; }
    /// <summary>The phase's length at full throttle, s.</summary>
    public required double Duration { get; init; }
    /// <summary>Full-throttle thrust and mass flow of the phase's liquid engines; zero for solids alone.</summary>
    public double LiquidThrust { get; init; }
    public double LiquidMassFlow { get; init; }
    /// <summary>Full-throttle liquid thrust at each grid pressure, N, or null for constant thrust.</summary>
    public double[]? LiquidThrustAtPressure { get; init; }
    /// <summary>The liquid engines can throttle.</summary>
    public bool Throttleable { get; init; } = true;
    /// <summary>The solid motors burning through the phase, as indices into the motor list.</summary>
    public int[] Solids { get; init; } = [];
    /// <summary>The liquid engines, as identifiers: the same set in two phases in a row is one engine set burning on.</summary>
    public int[] LiquidEngines { get; init; } = [];

    public bool HasLiquid => LiquidMassFlow > 0.0;
}

/// <summary>
/// The game's phases turned into the planner's stages, solids and all (issue #73).
///
/// EACH PHASE IS A STAGE. One with solids in it has its burn time pinned, so every node sits at a fixed second of its solids' burn:
///   - Solids that burn out end the stage, and its duration is theirs.
///   - A liquid core lit with them burns on into the next stage on the same load (<see cref="AscentStage.LiquidCarriesOver"/>), and its throttle is the planner's to choose all the way through.
///
/// A CORE THAT WOULD RUN DRY BEFORE ITS BOOSTERS at full throttle (the drain model's phase ends with the core, and the boosters burn on alone) is planned throttled down to outlast them, if it can at its throttle floor (issue #32). The two phases are laid down the other way round: boosters and core until the boosters burn out, then the core alone on what is left. The planner then picks how hard the core burns while the boosters do. Nothing separates at the boosters' burnout in that case - the vehicle was built to drop them with the core - so the boosters' casings ride along until the core is dry.
///
/// Otherwise the core is held at full thrust through the drain model's phase, whose length is then pinned too, and the boosters burn on alone in the next stage from where they were.
/// </summary>
public static class AscentPhasePlan
{
    /// <summary>A core is planned to outlast its boosters only if it can at this margin above its throttle floor.</summary>
    public const double OutlastMargin = 1.15;

    /// <summary>The seed burns a throttled-down core at this fraction of what would just empty it over the boosters' burn, so the stage after has some load to start from.</summary>
    public const double SeedLoadFraction = 0.9;

    /// <summary>Samples per stage table.</summary>
    private const int TableSamples = 257;

    public sealed class Result
    {
        public required AscentStage[] Stages { get; init; }
        /// <summary>Each stage's mass at its start, kg, down the plan's own chain.</summary>
        public required double[] StartMass { get; init; }
        /// <summary>One line per stage, for the log.</summary>
        public required string[] Describe { get; init; }
        /// <summary>What was decided about the solids, for the log and the panel.</summary>
        public required List<string> Notes { get; init; }
        /// <summary>A core was planned throttled down to outlast its boosters.</summary>
        public bool CoreThrottledDown { get; init; }
    }

    /// <param name="phases">The drain model's phases, in order; the first starts at <paramref name="m0"/>.</param>
    /// <param name="motors">Every solid motor the phases name.</param>
    /// <param name="pressureGrid">The back pressures the liquid and solid thrust tables are on, Pa, or null for an airless body (the solids' tables then have one column, at zero).</param>
    /// <param name="m0">The live mass, kg.</param>
    /// <param name="throttleFloor">The liquid engines' throttle floor.</param>
    /// <param name="heldFloor">The floor a stage that cannot throttle is held at: the script's 99 %.</param>
    /// <param name="reserveKg">Liquid kept back from the first load, kg: the booster reserve.</param>
    public static Result Build(IReadOnlyList<AscentPhase> phases, IReadOnlyList<AscentSolidMotor> motors,
                               double[]? pressureGrid, double m0, double dragArea,
                               double throttleFloor, double heldFloor, double reserveKg = 0.0)
    {
        double[] grid = pressureGrid ?? [0.0];
        var stages = new List<AscentStage>();
        var describe = new List<string>();
        var notes = new List<string>();
        var elapsed = new double[motors.Count];
        bool throttledDown = false;

        for (int i = 0; i < phases.Count; i++)
        {
            AscentPhase ph = phases[i];
            AscentPhase? next = i + 1 < phases.Count ? phases[i + 1] : null;
            double jettison = next != null ? Math.Max(0.0, ph.EndMass - next.StartMass) : 0.0;
            double floor = ph.Throttleable ? throttleFloor : heldFloor;
            double start = i == 0 ? m0 : ph.StartMass;

            if (ph.Solids.Length == 0)
            {
                stages.Add(new AscentStage(ph.LiquidThrust, ph.LiquidMassFlow, start - ph.EndMass, jettison, dragArea,
                                           pressureGrid, ph.LiquidThrustAtPressure, floor));
                describe.Add($"liquid {ph.LiquidThrust / 1000.0:F0} kN, {ph.LiquidMassFlow:F1} kg/s, {(start - ph.EndMass) / 1000.0:F2} t");
                continue;
            }

            double solidsLeft = double.PositiveInfinity;
            bool anyEnds = next == null;
            foreach (int m in ph.Solids)
            {
                solidsLeft = Math.Min(solidsLeft, motors[m].BurnTime - elapsed[m]);
                if (next != null && Array.IndexOf(next.Solids, m) < 0)
                    anyEnds = true;
            }
            solidsLeft = Math.Max(solidsLeft, 1e-3);

            if (anyEnds || !ph.HasLiquid)
            {
                // Solids that burn out end the stage. Solids alone that all burn on end where the drain model's phase does: something else starts there.
                double sigma = anyEnds ? Math.Min(solidsLeft, SolidsEnd(ph, next, motors, elapsed)) : Math.Min(ph.Duration, solidsLeft);
                AscentSolidBurn table = Table(ph.Solids, motors, elapsed, sigma, grid);
                double grain = table.Propellant(0.0, sigma);
                bool carries = ph.HasLiquid && next != null && next.HasLiquid && SameEngines(ph.LiquidEngines, next.LiquidEngines);
                double liquid = ph.HasLiquid ? ph.LiquidMassFlow * sigma : 0.0;
                if (Math.Abs(sigma - ph.Duration) > 0.05 * ph.Duration + 1.0)
                    notes.Add($"stage {stages.Count + 1}: the solids burn {sigma:F1} s against the drain model's {ph.Duration:F1} s");
                stages.Add(new AscentStage(ph.LiquidThrust, ph.LiquidMassFlow, grain + liquid, jettison, dragArea,
                                           pressureGrid, ph.LiquidThrustAtPressure, ph.HasLiquid ? floor : heldFloor,
                                           table, sigma, carries));
                describe.Add(Describe(ph, table, sigma, grain, liquid, carries ? "burns on" : null));
                Advance(ph.Solids, elapsed, sigma);
                continue;
            }

            // The liquid runs dry first and the solids burn on: see if the core can be throttled down to outlast them.
            double load = ph.LiquidMassFlow * ph.Duration;
            bool boostersAlone = next != null && !next.HasLiquid && SameEngines(ph.Solids, next.Solids)
                                 && (i + 2 >= phases.Count || !phases[i + 2].Solids.Any(s => Array.IndexOf(ph.Solids, s) >= 0));
            double lastsAtFloor = load / (ph.LiquidMassFlow * throttleFloor);
            if (ph.Throttleable && boostersAlone && throttleFloor < heldFloor && lastsAtFloor >= OutlastMargin * solidsLeft)
            {
                double sigma = solidsLeft;
                double seed = Math.Clamp(SeedLoadFraction * load / (ph.LiquidMassFlow * sigma), throttleFloor, 1.0);
                double burned = ph.LiquidMassFlow * seed * sigma;
                AscentSolidBurn table = Table(ph.Solids, motors, elapsed, sigma, grid);
                double grain = table.Propellant(0.0, sigma);
                // Nothing separates at the boosters' burnout: the drain model had none between the two phases, and whatever it drops after the boosters goes after the core now.
                double drop = Math.Max(0.0, ph.EndMass - next!.StartMass);
                double after = i + 2 < phases.Count ? Math.Max(0.0, next.EndMass - phases[i + 2].StartMass) : 0.0;
                stages.Add(new AscentStage(ph.LiquidThrust, ph.LiquidMassFlow, grain + burned, drop, dragArea,
                                           pressureGrid, ph.LiquidThrustAtPressure, throttleFloor, table, sigma,
                                           liquidCarriesOver: true, seedThrottle: seed));
                describe.Add(Describe(ph, table, sigma, grain, burned, "throttled to outlast them"));
                stages.Add(new AscentStage(ph.LiquidThrust, ph.LiquidMassFlow, load - burned, after, dragArea,
                                           pressureGrid, ph.LiquidThrustAtPressure, throttleFloor));
                describe.Add($"liquid {ph.LiquidThrust / 1000.0:F0} kN, {ph.LiquidMassFlow:F1} kg/s, {(load - burned) / 1000.0:F2} t left after the boosters");
                notes.Add($"the core would run dry {solidsLeft - ph.Duration:F0} s before its boosters at full throttle, so it is throttled to outlast them "
                        + $"(it can for {lastsAtFloor:F0} s at its {throttleFloor:P0} floor)");
                throttledDown = true;
                Advance(ph.Solids, elapsed, sigma);
                i++;    // the boosters' phase alone is the stage just laid down
                continue;
            }

            // Held at full thrust for the drain model's phase; the solids burn on in the next.
            {
                double sigma = Math.Min(ph.Duration, solidsLeft);
                AscentSolidBurn table = Table(ph.Solids, motors, elapsed, sigma, grid);
                double grain = table.Propellant(0.0, sigma);
                double liquid = ph.LiquidMassFlow * sigma;
                stages.Add(new AscentStage(ph.LiquidThrust, ph.LiquidMassFlow, grain + liquid, jettison, dragArea,
                                           pressureGrid, ph.LiquidThrustAtPressure, heldFloor, table, sigma));
                describe.Add(Describe(ph, table, sigma, grain, liquid, "held at full thrust, the solids burn on"));
                if (ph.Throttleable)
                    notes.Add(boostersAlone
                        ? $"the core runs dry {solidsLeft - ph.Duration:F0} s before its boosters and cannot outlast them at its {throttleFloor:P0} floor ({lastsAtFloor:F0} s), so it is held at full thrust"
                        : "the core runs dry before its boosters, which burn on with other engines; it is held at full thrust");
                Advance(ph.Solids, elapsed, sigma);
            }
        }

        ApplyReserve(stages, reserveKg, notes);

        var startMass = new double[stages.Count];
        double mass = m0;
        for (int i = 0; i < stages.Count; i++)
        {
            startMass[i] = mass;
            mass -= stages[i].PropellantMass + stages[i].JettisonMass;
        }
        return new Result
        {
            Stages = stages.ToArray(),
            StartMass = startMass,
            Describe = describe.ToArray(),
            Notes = notes,
            CoreThrottledDown = throttledDown,
        };
    }

    /// <summary>
    /// The first <paramref name="count"/> stages as a stack of their own: the last drops nothing, and if its liquid would have burned on into the next, it may now burn all of the load that was left for that stage, and no longer carries over.
    /// </summary>
    public static AscentStage[] Truncate(AscentStage[] all, int count)
    {
        count = Math.Clamp(count, 1, all.Length);
        var list = all.Take(count).ToArray();
        AscentStage last = list[^1];
        double prop = last.PropellantMass;
        if (last.LiquidCarriesOver)
            for (int j = count; j < all.Length; j++)
            {
                AscentStage s = all[j];
                prop += s.PropellantMass - (s.Solid?.Propellant(0.0, s.FixedBurnTime) ?? 0.0);
                if (!s.LiquidCarriesOver)
                    break;
            }
        list[^1] = last.With(prop, 0.0, false);
        return list;
    }

    // Where a stage that ends at a burnout ends: the first of its solids to go out that the next phase does not carry on.
    private static double SolidsEnd(AscentPhase ph, AscentPhase? next, IReadOnlyList<AscentSolidMotor> motors, double[] elapsed)
    {
        double end = double.PositiveInfinity;
        foreach (int m in ph.Solids)
            if (next == null || Array.IndexOf(next.Solids, m) < 0)
                end = Math.Min(end, motors[m].BurnTime - elapsed[m]);
        return Math.Max(end, 1e-3);
    }

    /// <summary>The motors' summed burn over the stage's own clock: each read from where it has got to.</summary>
    private static AscentSolidBurn Table(int[] solids, IReadOnlyList<AscentSolidMotor> motors, double[] elapsed,
                                         double seconds, double[] grid)
    {
        int np = grid.Length;
        var time = new double[TableSamples];
        var flow = new double[TableSamples];
        var thrust = new double[TableSamples * np];
        for (int j = 0; j < TableSamples; j++)
        {
            double t = seconds * j / (TableSamples - 1);
            time[j] = t;
            foreach (int m in solids)
            {
                AscentSolidMotor motor = motors[m];
                Sample(motor, elapsed[m] + t, np, out double mf, thrust.AsSpan(j * np, np));
                flow[j] += mf;
            }
        }
        return new AscentSolidBurn(time, flow, grid, thrust);
    }

    // Adds a motor's thrust row at a time into row, and returns its mass flow; linear between its samples, nothing once it has gone out.
    private static void Sample(AscentSolidMotor motor, double t, int np, out double massFlow, Span<double> row)
    {
        double[] times = motor.Time;
        int n = times.Length;
        int lo, hi;
        double f;
        if (t > times[n - 1] * (1.0 + 1e-9) + 1e-9)
        {
            massFlow = 0.0;
            return;
        }
        if (t <= 0.0) { lo = hi = 0; f = 0.0; }
        else if (t >= times[n - 1]) { lo = hi = n - 1; f = 0.0; }
        else
        {
            hi = Array.BinarySearch(times, t);
            if (hi >= 0) { lo = hi; f = 0.0; }
            else
            {
                hi = ~hi;
                lo = hi - 1;
                f = (t - times[lo]) / (times[hi] - times[lo]);
            }
        }
        massFlow = motor.MassFlow[lo] + f * (motor.MassFlow[hi] - motor.MassFlow[lo]);
        for (int g = 0; g < np; g++)
            row[g] += motor.Thrust[lo * np + g] + f * (motor.Thrust[hi * np + g] - motor.Thrust[lo * np + g]);
    }

    private static void Advance(int[] solids, double[] elapsed, double seconds)
    {
        foreach (int m in solids)
            elapsed[m] += seconds;
    }

    private static bool SameEngines(int[] a, int[] b)
        => a.Length == b.Length && a.All(x => Array.IndexOf(b, x) >= 0);

    /// <summary>
    /// Keeps the booster reserve out of the first liquid load: the stage that ends it burns that much less and drops it instead. Nothing to keep back from solids.
    /// </summary>
    private static void ApplyReserve(List<AscentStage> stages, double reserveKg, List<string> notes)
    {
        if (!(reserveKg > 0.0) || stages.Count == 0)
            return;
        int end = 0;
        while (end < stages.Count - 1 && stages[end].LiquidCarriesOver) end++;
        AscentStage s = stages[end];
        double solid = s.Solid?.Propellant(0.0, s.FixedBurnTime) ?? 0.0;
        double liquid = s.PropellantMass - solid;
        if (!s.HasLiquid || s.IsPinned || !(liquid > 0.0))
        {
            notes.Add("the booster reserve is not applied: the first stage has no liquid load that its own burn-out ends");
            return;
        }
        // The same guard UPFG keeps: never past 98 % of the load.
        double kept = Math.Min(reserveKg, 0.98 * liquid);
        stages[end] = s.With(s.PropellantMass - kept, s.JettisonMass + kept, s.LiquidCarriesOver);
    }

    private static string Describe(AscentPhase ph, AscentSolidBurn table, double sigma, double grain, double liquid, string? core)
    {
        double meanFlow = grain / sigma;
        double first = table.MassFlow[0], last = table.MassFlowAt(sigma);
        string solids = $"{ph.Solids.Length} solid(s) for {sigma:F1} s: {grain / 1000.0:F2} t, flow {first:F0} -> {last:F0} kg/s (mean {meanFlow:F0}), "
                      + $"thrust {table.Thrust[0] / 1000.0:F0} -> {table.Thrust[(table.Time.Length - 1) * table.Pressures] / 1000.0:F0} kN in vacuum";
        return ph.HasLiquid
            ? $"{solids}; liquid {ph.LiquidThrust / 1000.0:F0} kN, {ph.LiquidMassFlow:F1} kg/s, {liquid / 1000.0:F2} t{(core != null ? " (" + core + ")" : "")}"
            : solids;
    }
}
