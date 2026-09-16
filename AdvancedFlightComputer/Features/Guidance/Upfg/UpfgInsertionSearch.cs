#nullable disable

using System;
using Brutal.Numerics;

namespace AdvancedFlightComputer.Features.Guidance.Upfg;

// WHERE A FREE INSERTION COSTS THE LEAST dV.
//
// With the argument of periapsis free, every point of the target ellipse is an equally good place to arrive: the orbit is the same whichever one the burn ends at, so periapsis is a choice rather than a requirement. It is the cheapest choice only for a burn that arrives level. A vehicle still climbing when it gets there pays to flatten out at exactly the lowest point of the ellipse, where a few degrees further round the ellipse is itself climbing. How much that is worth depends on the ellipse - an eccentric one gains flight-path angle quickly just past periapsis while its radius barely moves - and on the vehicle, because a long burn arrives climbing. So it is measured rather than assumed.
//
// UPFG already prices an insertion. Its velocity-to-go is the thrust dV the stage list still has to produce, gravity and steering losses included, worked through the real thrust, Isp, masses, g-limit and staging - and for a fixed stage list the propellant it costs rises with it. The search is therefore a one-dimensional minimisation of vgo over the insertion's true anomaly: every SearchIntervalS it prices insertions a probe step either side of the goal, fits a parabola through the three costs and moves the goal toward the parabola's lowest point. The insertion actually flown slews toward the goal at NuRateDegS, so the target the steering is solved for never jumps.
//
// A PRICE IS A CONVERGED SOLUTION. Re-solved again and again from one instant, UPFG settles on a vgo that depends only on that instant and the insertion - but slowly on a long burn, over hundreds of solves, and not steadily: it can creep along for a few hundred solves and then drop by over a hundred m/s within fifty. A price read before that point depends on where the solver started, and an earlier version of this search, which compared probes after a fixed 40 solves, read its own trajectory into the costs: every insertion a little further on than the one being flown looked cheaper, and from a frozen state the goal walked to the end of its range while claiming savings of 10 km/s. So each probe is solved until vgo and tgo have stopped moving over a whole SettleWindowSolves (TryPrice), and a probe that has not by MaxProbeSolves, or that blows out past DivergedVgoFactor, prices nothing - some insertions have no fixed point at all from a given instant, periapsis among them on a long burn from orbit behind a weak upper stage, and the search steps away from those rather than stopping on them. A solve is a few microseconds, so this costs milliseconds, and the probes are warm-started from each other - the neighbour's converged solution is a far closer start than the live one - which keeps it there.
//
// THE INSERTION IS DECIDED EARLY. Where to arrive is a question about the whole burn, and it has a meaningful answer only while most of the burn is still to fly: late on, the trajectory is committed to the insertion it has been steering for, and moving it then spends more on the change than the new insertion could save. So the goal freezes once FreezeVgoFraction of the dV the first search saw has been flown, or once tgo is under FreezeTgoS, whichever comes first.
//
// THE OTHER GUARDS, each for a reason measured in closed-loop simulation of this solver:
//  - Climbing side only, from periapsis to MaxNuDeg past it. Arriving before periapsis means arriving descending, and burns from orbit that had to do that diverged.
//  - A move has to be worth MinSavingMs. On an ordinary low orbit the whole cost curve is a few m/s deep, and following noise that shallow would only stir the steering.
//  - A near-circular target is left alone: every point of a circle is the same insertion.
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

    /// <summary>Furthest past periapsis the search looks, deg.</summary>
    public const double MaxNuDeg = 45.0;

    /// <summary>Spacing of the three probes, deg.</summary>
    public const double ProbeStepDeg = 2.5;

    /// <summary>Most one search may move the goal, deg - how far past its probes the parabola is trusted.</summary>
    public const double MaxGoalStepDeg = 5.0;

    /// <summary>Most the flown insertion moves per second of sim time, deg/s.</summary>
    public const double NuRateDegS = 0.5;

    /// <summary>Smallest predicted saving a move has to buy, m/s.</summary>
    public const double MinSavingMs = 2.0;

    /// <summary>Solves over which a probe's vgo and tgo must hold still to count as converged.</summary>
    public const int SettleWindowSolves = 50;

    /// <summary>How far vgo may move over a settle window and still count as converged, m/s.</summary>
    public const double SettleVgoMs = 0.1;

    /// <summary>How far tgo may move over a settle window and still count as converged, s.</summary>
    public const double SettleTgoS = 0.01;

    /// <summary>Most solves one probe gets to converge. The slowest seen from a cold start took about a thousand.</summary>
    public const int MaxProbeSolves = 3000;

    /// <summary>
    /// A probe whose vgo passes this multiple of the solution it started from is abandoned there and then. An insertion with no fixed point from this instant does not wander: re-solved, its vgo creeps up and then blows out to several times what it started at before collapsing and starting again, and nothing a step of the search away legitimately costs double.
    /// </summary>
    public const double DivergedVgoFactor = 2.0;

    // The live vgo when the first successful search of this flight ran, NaN before one has.
    private double _firstSearchVgo = double.NaN;

    /// <summary>The insertion true anomaly to fly now, rad, slewing toward <see cref="GoalNu"/>.</summary>
    public double Nu { get; private set; }

    /// <summary>Where the last search put the cheapest insertion it could see, rad.</summary>
    public double GoalNu { get; private set; }

    /// <summary>What the goal saves over inserting at periapsis (or the floor), m/s, by the last successful search's own costs. NaN before one has run, and while optimisation is off.</summary>
    public double SavingMs { get; private set; } = double.NaN;

    /// <summary>Sim time of the last search, NegativeInfinity before the first.</summary>
    public double LastSearchTime { get; private set; } = double.NegativeInfinity;

    /// <summary>Solves the last search spent across its probes - its cost, for the log.</summary>
    public int LastSearchSolves { get; private set; }

    /// <summary>True once the goal has stopped moving for the rest of the flight (see the class notes).</summary>
    public bool Frozen { get; private set; }

    public void Reset()
    {
        Nu = 0.0;
        GoalNu = 0.0;
        SavingMs = double.NaN;
        LastSearchTime = double.NegativeInfinity;
        LastSearchSolves = 0;
        Frozen = false;
        _firstSearchVgo = double.NaN;
    }

    /// <summary>
    /// One guidance cycle, run straight after the live solve with the same state and stage model, so the probes start from a solution of exactly this instant. <paramref name="enabled"/> is whether a free insertion is being optimised at all - off, the goal returns to periapsis and there is no saving to report. <paramref name="allowed"/> is whether a search may run right now (converged, flying closed loop, and within the caller's wall-clock budget). The slew toward the goal runs either way, so a search that stops still lands the insertion where the last one pointed.
    /// </summary>
    public void Step(double time, double dt, bool enabled, bool allowed, UpfgGuidance live, UpfgTarget target,
                     double3 r, double3 v, double mass, double mu, UpfgVehicle vehicle)
    {
        if (!enabled)
        {
            GoalNu = 0.0;
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
        Nu += Math.Clamp(GoalNu - Nu, -maxMove, maxMove);
    }

    // One search: true when every probe it needed priced.
    private bool Search(UpfgGuidance live, UpfgTarget target, double3 r, double3 v, double mass, double mu,
                        UpfgVehicle vehicle)
    {
        LastSearchSolves = 0;
        double step = DegToRad(ProbeStepDeg);

        // The floor can put the lowest insertion past periapsis, and below it every candidate is the same insertion.
        double lowest = target.FloorAnomaly;
        double highest = DegToRad(MaxNuDeg);
        if (highest - lowest < 2.0 * step)
            return false;

        double mid = Math.Clamp(GoalNu, lowest + step, highest - step);
        if (!Price(live, target, mid, r, v, mass, mu, vehicle, out double here, out UpfgGuidance hereProbe))
            return false;
        bool belowPriced = Price(hereProbe, target, mid - step, r, v, mass, mu, vehicle, out double below, out UpfgGuidance belowProbe);
        bool abovePriced = Price(hereProbe, target, mid + step, r, v, mass, mu, vehicle, out double above, out _);

        // A neighbour with no converged solution is an insertion UPFG diverges on from here - on a long burn from orbit, periapsis itself can be one. There is no parabola to fit then, only the insertions that did price, and the goal goes to the cheapest of them: always when it was sitting on the one that failed, otherwise only for the usual saving.
        if (!belowPriced || !abovePriced)
        {
            double bestNu = mid, bestCost = here;
            if (belowPriced && below < bestCost)
                (bestNu, bestCost) = (mid - step, below);
            if (abovePriced && above < bestCost)
                (bestNu, bestCost) = (mid + step, above);
            bool goalFailed = (!belowPriced && GoalNu <= mid - step + 1e-9) || (!abovePriced && GoalNu >= mid + step - 1e-9);
            double atGoal = GoalNu == mid ? here : GoalNu == mid - step && belowPriced ? below : double.NaN;
            if (goalFailed || !(atGoal - bestCost < MinSavingMs))
                GoalNu = bestNu;
            SavingMs = double.NaN;
            return true;
        }

        // The parabola through the three costs, in probe steps about mid.
        double a = 0.5 * (below + above) - here;
        double b = 0.5 * (above - below);
        double CostAt(double x) => here + x * (b + a * x);

        double reach = MaxGoalStepDeg / ProbeStepDeg;
        double from = (GoalNu - mid) / step;
        double to;
        if (a > 1e-9)
            to = -b / (2.0 * a);
        else
            to = b < 0.0 ? from + reach : b > 0.0 ? from - reach : from;   // no bottom in sight: downhill
        to = Math.Clamp(to, from - reach, from + reach);
        to = Math.Clamp(to, (lowest - mid) / step, (highest - mid) / step);

        if (CostAt(from) - CostAt(to) >= MinSavingMs)
            GoalNu = mid + to * step;

        double goalCost = CostAt((GoalNu - mid) / step);
        if (mid - step <= lowest + 1e-9)
            SavingMs = below - goalCost;
        else if (Price(belowProbe, target, lowest, r, v, mass, mu, vehicle, out double atLowest, out _))
            SavingMs = atLowest - goalCost;
        return true;
    }

    private bool Price(UpfgGuidance from, UpfgTarget target, double nu, double3 r, double3 v, double mass, double mu,
                       UpfgVehicle vehicle, out double cost, out UpfgGuidance probe)
    {
        bool priced = TryPrice(from, target, nu, r, v, mass, mu, vehicle, out cost, out probe, out int solves);
        LastSearchSolves += solves;
        return priced;
    }

    /// <summary>
    /// What inserting at true anomaly <paramref name="nu"/> costs from this instant: the vgo a copy of <paramref name="from"/> converges on when solved for that free insertion again and again from (r, v, mass), in m/s. The copy is returned too, as a warm start for pricing a neighbouring insertion. False when it went non-finite, blew out past DivergedVgoFactor or had not converged within MaxProbeSolves, which prices nothing. <paramref name="from"/> is only read.
    /// </summary>
    public static bool TryPrice(UpfgGuidance from, UpfgTarget target, double nu, double3 r, double3 v, double mass,
                                double mu, UpfgVehicle vehicle, out double cost, out UpfgGuidance probe, out int solves)
    {
        cost = double.NaN;
        probe = from.Clone();
        UpfgTarget pinned = target.WithFreeInsertion(nu);
        double runaway = DivergedVgoFactor * from.VgoMag;
        double windowVgo = double.NaN, windowTgo = double.NaN;
        for (solves = 1; solves <= MaxProbeSolves; solves++)
        {
            probe.Step(r, v, mass, mu, pinned, vehicle, 1, 0.0);
            if (!double.IsFinite(probe.Tgo) || !double.IsFinite(probe.VgoMag) || probe.Tgo <= 0.0
                || (runaway > 0.0 && probe.VgoMag > runaway))
                return false;
            if (solves % SettleWindowSolves != 0)
                continue;
            if (Math.Abs(probe.VgoMag - windowVgo) <= SettleVgoMs && Math.Abs(probe.Tgo - windowTgo) <= SettleTgoS)
            {
                cost = probe.VgoMag;
                return true;
            }
            windowVgo = probe.VgoMag;
            windowTgo = probe.Tgo;
        }
        solves = MaxProbeSolves;
        return false;
    }

    private static double DegToRad(double d) => d * Math.PI / 180.0;
}
