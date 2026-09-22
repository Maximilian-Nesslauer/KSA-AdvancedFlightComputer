#nullable disable

using System;
using System.Collections.Generic;
using Brutal.Numerics;

namespace AdvancedFlightComputer.Features.Guidance.Upfg;

// WHERE A FREE INSERTION COSTS THE LEAST.
//
// With the argument of periapsis free, every point of the target ellipse is an equally good place to arrive: the orbit is the same whichever one the burn ends at, so periapsis is a choice rather than a requirement.
// It is the cheapest choice only for a burn that arrives level.
// A vehicle still climbing when it gets there pays to flatten out at exactly the lowest point of the ellipse, where a few degrees further round the ellipse is itself climbing.
// How much that is worth depends on the ellipse and on the vehicle, so it is measured rather than assumed.
//
// THE ANCHOR.
// By default the search works from periapsis, over the climbing side only: from periapsis (or the floor crossing) to MaxNuDeg past it.
// Insertions before periapsis were tried and left out: on a GTO ascent the search chased them into a flight that ran dry.
// With the apoapsis anchor it works on the climbing side of apoapsis instead, from MaxNuDeg before it to ApoapsisLeadDeg before it, and never at or past it: the burn ends still climbing, so the vehicle coasts up to apoapsis and can circularise there.
// That is for a vehicle whose burn is better ended high - a weak one on a long burn.
// A continuous burn can end high; for most vehicles it is simply dearer, and far from where a burn would naturally end UPFG's prices stop meaning much, so the anchor is the player's choice rather than something the search wanders to.
//
// THE COST IS BURN TIME.
// UPFG's tgo is the time the stage list still has to burn to reach the insertion, gravity and steering losses included, worked through the real thrust, Isp, masses, g-limit and staging.
// The engines run at fixed thrust, so the propellant an insertion costs rises with the time it takes to fly.
//
// A GRID, NOT A LINE SEARCH.
// Every SearchIntervalS the search prices insertions CoarseStepDeg apart across its whole range - skipping any under the floor - then prices FineStepDeg points either side of the cheapest, and puts the goal at the cheapest of everything, refined by the parabola through it and its neighbours.
// A local search that walks from where it is can only see the slope under it: it stalls against an insertion that does not price, and cannot know a cheaper basin lies further along.
// The grid sees the whole range every time.
// The insertion actually flown slews toward the goal at NuRateDegS, so the target the steering is solved for never jumps.
//
// A PRICE IS A CONVERGED SOLUTION.
// Re-solved again and again from one instant, UPFG settles on a tgo that depends only on that instant and the insertion - but slowly on a long burn, over hundreds of solves, and not steadily: it can creep along for a few hundred solves and then drop within fifty.
// A price read before that depends on where the solver started.
// An earlier version compared probes after a fixed 40 solves and read its own trajectory into the costs: from a frozen state its goal walked to the end of its range.
// So each probe is solved until tgo has stayed within SettleTgoS for a whole SettleWindowSolves (TryPrice).
// One that has not by MaxProbeSolves, or that blows out past DivergedVgoFactor, prices nothing - some insertions have no fixed point at all from a given instant, periapsis among them on a long burn from orbit behind a weak upper stage - and the grid simply has a gap there.
// The probes are warm-started from their grid neighbours, whose converged solutions are a far closer start than the live one, and MaxSearchSolves caps a search regardless.
//
// THE INSERTION IS DECIDED EARLY.
// Where to arrive is a question about the whole burn, and it has a meaningful answer only while most of the burn is still to fly: late on, the trajectory is committed to the insertion it has been steering for, and moving it then spends more on the change than the new insertion could save.
// So the goal freezes once FreezeVgoFraction of the dV the first search saw has been flown, or once tgo is under FreezeTgoS, whichever comes first.
//
// THE OTHER GUARDS:
// - A move has to save MinSavingS of burn. On an ordinary low orbit the whole cost curve is a fraction of a second deep, and following noise that shallow would only stir the steering.
// - A near-circular target is left alone: every point of a circle is the same insertion.
//
// The caller bounds how often a search runs in wall-clock time as well (see Ascent.cs): the interval here is sim time, which warp compresses.
public sealed class UpfgInsertionSearch
{
    /// <summary>Sim seconds between searches.</summary>
    public const double SearchIntervalS = 10.0;

    /// <summary>Below this tgo the goal stays where it is to cutoff, s.</summary>
    public const double FreezeTgoS = 60.0;

    /// <summary>The goal also freezes once vgo has fallen to this fraction of what the first search saw.</summary>
    public const double FreezeVgoFraction = 0.5;

    /// <summary>How far the search looks: past periapsis, or before apoapsis, deg.</summary>
    public const double MaxNuDeg = 45.0;

    /// <summary>With the apoapsis anchor, the latest the burn may end, deg before apoapsis - the coast that leaves to circularise.</summary>
    public const double ApoapsisLeadDeg = 5.0;

    /// <summary>Spacing of the grid across the whole range, deg.</summary>
    public const double CoarseStepDeg = 5.0;

    /// <summary>Spacing of the finer points either side of the cheapest coarse one, deg.</summary>
    public const double FineStepDeg = 2.5;

    /// <summary>How far either side of the cheapest coarse point the fine points go, deg.</summary>
    public const double FineSpanDeg = 2.5;

    /// <summary>Most the flown insertion moves per second of sim time, deg/s.</summary>
    public const double NuRateDegS = 0.5;

    /// <summary>Smallest burn time a move of the goal has to save, s.</summary>
    public const double MinSavingS = 0.2;

    /// <summary>Solves over which a probe's tgo must stay within SettleTgoS to count as converged.</summary>
    public const int SettleWindowSolves = 50;

    /// <summary>How far tgo may range over a settle window and still count as converged, s.</summary>
    public const double SettleTgoS = 0.01;

    /// <summary>Most solves one probe gets to converge. The slowest seen from a cold start took about a thousand.</summary>
    public const int MaxProbeSolves = 3000;

    /// <summary>Most solves one search spends across its whole grid; past it the grid is left with gaps.</summary>
    public const int MaxSearchSolves = 40000;

    /// <summary>
    /// A probe whose vgo passes this multiple of the solution it started from is abandoned there and then. An insertion with no fixed point from this instant does not wander: re-solved, its vgo creeps up and then blows out to several times what it started at before collapsing and starting again.
    /// </summary>
    public const double DivergedVgoFactor = 2.0;

    // The live vgo when the first successful search of this flight ran, NaN before one has.
    private double _firstSearchVgo = double.NaN;

    /// <summary>True when the search works before apoapsis rather than past periapsis (see SetAnchor).</summary>
    public bool AtApoapsis { get; private set; }

    /// <summary>The apsis the search measures from: zero for periapsis, pi for apoapsis.</summary>
    public double AnchorNu => AtApoapsis ? Math.PI : 0.0;

    /// <summary>Where the insertion goes before any search, and with optimisation off: periapsis, or ApoapsisLeadDeg before apoapsis.</summary>
    public double DefaultNu => AtApoapsis ? Math.PI - DegToRad(ApoapsisLeadDeg) : 0.0;

    /// <summary>The insertion true anomaly to fly now, rad in (-pi, pi], slewing toward <see cref="GoalNu"/>.</summary>
    public double Nu { get; private set; }

    /// <summary>Where the last search put the cheapest insertion it could see, rad in (-pi, pi]: zero is periapsis, pi apoapsis.</summary>
    public double GoalNu { get; private set; }

    /// <summary>Burn time the goal saves over inserting at DefaultNu (periapsis or its floor crossing, or ApoapsisLeadDeg before apoapsis), s, by the last search's own prices. NaN before one has run, when that insertion did not price, and while optimisation is off.</summary>
    public double SavingS { get; private set; } = double.NaN;

    /// <summary>The same saving as thrust dV, m/s, from the same probes' vgo - for the readout.</summary>
    public double SavingMs { get; private set; } = double.NaN;

    /// <summary>Sim time of the last search, NegativeInfinity before the first.</summary>
    public double LastSearchTime { get; private set; } = double.NegativeInfinity;

    /// <summary>Solves the last search spent across its probes - its cost, for the log.</summary>
    public int LastSearchSolves { get; private set; }

    /// <summary>Insertions the last search priced, and how many it tried.</summary>
    public int LastSearchPriced { get; private set; }
    public int LastSearchPoints { get; private set; }

    /// <summary>True once the goal has stopped moving for the rest of the flight (see the class notes).</summary>
    public bool Frozen { get; private set; }

    public void Reset()
    {
        Nu = DefaultNu;
        GoalNu = DefaultNu;
        SavingS = double.NaN;
        SavingMs = double.NaN;
        LastSearchTime = double.NegativeInfinity;
        LastSearchSolves = 0;
        LastSearchPriced = 0;
        LastSearchPoints = 0;
        Frozen = false;
        _firstSearchVgo = double.NaN;
    }

    /// <summary>
    /// Which apsis the search works from. Called before every solve, so the target built from <see cref="Nu"/> already has it. A change moves the flown insertion straight to the new DefaultNu rather than slewing half way round the ellipse, and the next search starts from there.
    /// </summary>
    public void SetAnchor(bool apoapsis)
    {
        if (apoapsis == AtApoapsis)
            return;
        AtApoapsis = apoapsis;
        Nu = DefaultNu;
        GoalNu = DefaultNu;
        SavingS = double.NaN;
        SavingMs = double.NaN;
        LastSearchTime = double.NegativeInfinity;
    }

    /// <summary>
    /// One guidance cycle, run straight after the live solve with the same state and stage model, so the probes start from a solution of exactly this instant. <paramref name="enabled"/> is whether a free insertion is being optimised at all - off, the goal returns to the anchor and there is no saving to report. <paramref name="allowed"/> is whether a search may run right now (converged, flying closed loop, and within the caller's wall-clock budget). The slew toward the goal runs either way, so a search that stops still lands the insertion where the last one pointed.
    /// </summary>
    public void Step(double time, double dt, bool enabled, bool allowed, UpfgGuidance live, UpfgTarget target,
                     double3 r, double3 v, double mass, double mu, UpfgVehicle vehicle)
    {
        if (!enabled)
        {
            GoalNu = DefaultNu;
            SavingS = double.NaN;
            SavingMs = double.NaN;
        }
        else
        {
            if (!Frozen && live.Converged
                && (live.Tgo < FreezeTgoS || live.VgoMag < FreezeVgoFraction * _firstSearchVgo))
                Frozen = true;
            if (allowed && !Frozen && target.Ecc >= UpfgTarget.MinArgPeEccentricity
                && time - LastSearchTime >= SearchIntervalS)
            {
                LastSearchTime = time;
                if (Search(live, target, r, v, mass, mu, vehicle) && double.IsNaN(_firstSearchVgo))
                    _firstSearchVgo = live.VgoMag;
            }
        }

        double maxMove = DegToRad(NuRateDegS) * Math.Max(dt, 0.0);
        Nu = UpfgTarget.WrapPi(Nu + Math.Clamp(UpfgTarget.WrapPi(GoalNu - Nu), -maxMove, maxMove));
    }

    // One priced insertion, by its offset from the anchor.
    private readonly struct Priced
    {
        public readonly double Offset;
        public readonly double Tgo;
        public readonly double Vgo;

        public Priced(double offset, double tgo, double vgo)
        {
            Offset = offset;
            Tgo = tgo;
            Vgo = vgo;
        }
    }

    // One search over the anchor's range: true when at least one insertion priced.
    private bool Search(UpfgGuidance live, UpfgTarget target, double3 r, double3 v, double mass, double mu,
                        UpfgVehicle vehicle)
    {
        LastSearchSolves = 0;
        LastSearchPriced = 0;
        LastSearchPoints = 0;

        // Offsets from the apsis, rad. From periapsis the range is the climbing side, starting at the floor crossing; from apoapsis it is the climbing side too, ending ApoapsisLeadDeg short of it, where only an ellipse almost wholly under the floor has anything to skip.
        double floor = target.FloorAnomaly;
        double reach = DegToRad(MaxNuDeg);
        double coarse = DegToRad(CoarseStepDeg);
        double fine = DegToRad(FineStepDeg);
        double anchor = AnchorNu;
        double lowest = AtApoapsis ? -reach : floor;
        double highest = AtApoapsis ? -DegToRad(ApoapsisLeadDeg) : reach;
        bool Usable(double offset) => offset >= lowest - 1e-9 && offset <= highest + 1e-9
                                      && Math.Abs(UpfgTarget.WrapPi(anchor + offset)) >= floor - 1e-9;
        if (highest - lowest < coarse)
            return false;

        var coarseOffsets = new List<double>();
        if (AtApoapsis)
            coarseOffsets.Add(highest);
        else
            coarseOffsets.Add(floor);
        int steps = (int)Math.Round(MaxNuDeg / CoarseStepDeg);
        for (int k = AtApoapsis ? -steps : 0; k <= steps; k++)
        {
            double offset = k * coarse;
            if (!AtApoapsis && offset < floor + 0.25 * coarse && !(floor == 0.0 && k == 0))
                continue;
            if (Usable(offset) && !coarseOffsets.Contains(offset))
                coarseOffsets.Add(offset);
        }
        coarseOffsets.Sort();
        if (coarseOffsets.Count == 0)
            return false;

        var priced = new List<Priced>();

        UpfgGuidance PriceAt(UpfgGuidance warm, double offset)
        {
            LastSearchPoints++;
            if (LastSearchSolves >= MaxSearchSolves)
                return null;
            bool ok = TryPrice(warm, target, UpfgTarget.WrapPi(anchor + offset), r, v, mass, mu, vehicle,
                out double cost, out UpfgGuidance probe, out int solves);
            LastSearchSolves += solves;
            if (!ok)
                return null;
            priced.Add(new Priced(offset, cost, probe.VgoMag));
            LastSearchPriced++;
            return probe;
        }

        // The coarse grid, walked out both ways from the point nearest the live insertion, each probe warm-started from the last one that priced on its side.
        double goalOffset = UpfgTarget.WrapPi(GoalNu - anchor);
        int start = 0;
        for (int i = 1; i < coarseOffsets.Count; i++)
            if (Math.Abs(coarseOffsets[i] - goalOffset) < Math.Abs(coarseOffsets[start] - goalOffset))
                start = i;
        var coarseProbes = new UpfgGuidance[coarseOffsets.Count];
        coarseProbes[start] = PriceAt(live, coarseOffsets[start]);
        UpfgGuidance from = coarseProbes[start] ?? live;
        for (int i = start + 1; i < coarseOffsets.Count; i++)
            from = (coarseProbes[i] = PriceAt(from, coarseOffsets[i])) ?? from;
        from = coarseProbes[start] ?? live;
        for (int i = start - 1; i >= 0; i--)
            from = (coarseProbes[i] = PriceAt(from, coarseOffsets[i])) ?? from;

        if (priced.Count == 0)
        {
            SavingS = SavingMs = double.NaN;
            return false;
        }

        // Fine points either side of the cheapest coarse point, walked outward from it, inside the range.
        int bestCoarse = -1;
        for (int i = 0; i < coarseOffsets.Count; i++)
            if (coarseProbes[i] != null && (bestCoarse < 0 || coarseProbes[i].Tgo < coarseProbes[bestCoarse].Tgo))
                bestCoarse = i;
        int fineCount = (int)Math.Round(FineSpanDeg / FineStepDeg);
        foreach (int sign in new[] { 1, -1 })
        {
            from = coarseProbes[bestCoarse];
            for (int k = 1; k <= fineCount; k++)
            {
                double offset = coarseOffsets[bestCoarse] + sign * k * fine;
                if (!Usable(offset))
                    break;
                bool known = false;
                foreach (double c in coarseOffsets)
                    known |= Math.Abs(c - offset) < 1e-9;
                if (!known)
                    from = PriceAt(from, offset) ?? from;
            }
        }

        priced.Sort((a, b) => a.Offset.CompareTo(b.Offset));
        int best = 0;
        for (int i = 1; i < priced.Count; i++)
            if (priced[i].Tgo < priced[best].Tgo)
                best = i;

        // Between points: the vertex of the parabola through the cheapest and its neighbours, when both are close and it curves up.
        double bestOffset = priced[best].Offset, bestTgo = priced[best].Tgo;
        if (best > 0 && best < priced.Count - 1
            && priced[best].Offset - priced[best - 1].Offset <= fine + 1e-9
            && priced[best + 1].Offset - priced[best].Offset <= fine + 1e-9)
        {
            double x0 = priced[best - 1].Offset - priced[best].Offset, y0 = priced[best - 1].Tgo;
            double x2 = priced[best + 1].Offset - priced[best].Offset, y2 = priced[best + 1].Tgo;
            double y1 = priced[best].Tgo;
            double denom = x0 * x2 * (x0 - x2);
            double a = (x2 * (y0 - y1) - x0 * (y2 - y1)) / denom;
            double b = (x0 * x0 * (y2 - y1) - x2 * x2 * (y0 - y1)) / denom;
            if (a > 0.0)
            {
                double shift = Math.Clamp(-b / (2.0 * a), x0, x2);
                bestOffset += shift;
                bestTgo = y1 + shift * (b + a * shift);
            }
        }

        // The goal moves only for MinSavingS. Where it is now is priced off the points either side of it, no further apart than the coarse grid; without both it may not fly at all, and it moves regardless.
        double atGoal = double.NaN;
        for (int i = 0; i < priced.Count - 1; i++)
        {
            double o0 = priced[i].Offset, o1 = priced[i + 1].Offset;
            if (goalOffset < o0 - 1e-9 || goalOffset > o1 + 1e-9 || o1 - o0 > coarse + 1e-9)
                continue;
            double f = o1 - o0 > 1e-12 ? (goalOffset - o0) / (o1 - o0) : 0.0;
            atGoal = priced[i].Tgo + f * (priced[i + 1].Tgo - priced[i].Tgo);
            break;
        }
        if (!(atGoal - bestTgo < MinSavingS))
            GoalNu = UpfgTarget.WrapPi(anchor + bestOffset);

        // The saving is against the anchor itself - periapsis or its floor crossing, or apoapsis - from the priced point nearest the goal, so it is two converged probes rather than a fitted guess.
        double newGoalOffset = UpfgTarget.WrapPi(GoalNu - anchor);
        double referenceOffset = AtApoapsis ? highest : floor;
        int nearest = 0, reference = -1;
        for (int i = 0; i < priced.Count; i++)
        {
            if (Math.Abs(priced[i].Offset - newGoalOffset) < Math.Abs(priced[nearest].Offset - newGoalOffset))
                nearest = i;
            if (Math.Abs(priced[i].Offset - referenceOffset) < 1e-9)
                reference = i;
        }
        SavingS = reference >= 0 ? priced[reference].Tgo - priced[nearest].Tgo : double.NaN;
        SavingMs = reference >= 0 ? priced[reference].Vgo - priced[nearest].Vgo : double.NaN;
        return true;
    }

    /// <summary>
    /// What inserting at true anomaly <paramref name="nu"/> costs from this instant: the tgo, in seconds of burn, that a copy of <paramref name="from"/> converges on when solved for that free insertion again and again from (r, v, mass). The copy is returned too - its VgoMag is the same price as thrust dV - and serves as a warm start for pricing a neighbouring insertion. False when it went non-finite, blew out past DivergedVgoFactor or had not converged within MaxProbeSolves, which prices nothing. <paramref name="from"/> is only read.
    /// </summary>
    public static bool TryPrice(UpfgGuidance from, UpfgTarget target, double nu, double3 r, double3 v, double mass,
                                double mu, UpfgVehicle vehicle, out double cost, out UpfgGuidance probe, out int solves)
    {
        cost = double.NaN;
        probe = from.Clone();
        UpfgTarget pinned = target.WithFreeInsertion(nu);
        double runaway = DivergedVgoFactor * from.VgoMag;
        // Converged means tgo stayed inside SettleTgoS for every solve of a window, not that two samples a window apart agree: a solution cycling with a period that divides the window reads the same at both ends.
        double low = double.PositiveInfinity, high = double.NegativeInfinity;
        for (solves = 1; solves <= MaxProbeSolves; solves++)
        {
            probe.Step(r, v, mass, mu, pinned, vehicle, 1, 0.0);
            if (!double.IsFinite(probe.Tgo) || !double.IsFinite(probe.VgoMag) || probe.Tgo <= 0.0
                || (runaway > 0.0 && probe.VgoMag > runaway))
                return false;
            low = Math.Min(low, probe.Tgo);
            high = Math.Max(high, probe.Tgo);
            if (solves % SettleWindowSolves != 0)
                continue;
            if (high - low <= SettleTgoS)
            {
                cost = probe.Tgo;
                return true;
            }
            low = double.PositiveInfinity;
            high = double.NegativeInfinity;
        }
        solves = MaxProbeSolves;
        return false;
    }

    private static double DegToRad(double d) => d * Math.PI / 180.0;
}
