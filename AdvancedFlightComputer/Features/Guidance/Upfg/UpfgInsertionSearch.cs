#nullable disable

using System;
using Brutal.Numerics;

namespace AdvancedFlightComputer.Features.Guidance.Upfg;

// WHERE A FREE INSERTION COSTS THE LEAST dV.
//
// With the argument of periapsis free, every point of the target ellipse is an equally good place to arrive: the orbit is the same whichever one the burn ends at, so periapsis is a choice rather than a requirement. It is the cheapest choice only for a burn that arrives level. A vehicle still climbing when it gets there pays to flatten out at exactly the lowest point of the ellipse, where a few degrees further round the ellipse is itself climbing. How much that is worth depends on the ellipse - an eccentric one gains flight-path angle quickly just past periapsis while its radius barely moves - and on the vehicle, because a long burn arrives climbing. So it is measured rather than assumed.
//
// UPFG already prices an insertion. Its velocity-to-go is the thrust dV the stage list still has to produce, gravity and steering losses included, worked through the real thrust, Isp, masses, g-limit and staging - and for a fixed stage list the propellant it costs rises with it. The search is therefore a one-dimensional minimisation of vgo over the insertion's true anomaly: every SearchIntervalS it runs copies of the live solver toward insertions a probe step either side of the goal, fits a parabola through the three costs and moves the goal toward the parabola's lowest point. The insertion actually flown slews toward the goal at NuRateDegS, so the target the steering is solved for never jumps.
//
// THE PROBES ARE COMPARED, NOT CONVERGED. The live solution is one step of a recursive filter tracking a moving vehicle, not a fixed point of this instant, and a copy re-solved again and again from the same state goes on drifting for dozens of solves - by over a hundred m/s on a long burn. Every probe starts from the same live solution and gets the same ProbeSolves, so that drift is common to all of them and falls out of the comparison. Only differences are ever used: the parabola, the saving threshold and the readout alike.
//
// THE INSERTION IS DECIDED EARLY. Where to arrive is a question about the whole burn, and it has a meaningful answer only while most of the burn is still to fly. Later the trajectory is committed to the insertion it has been steering for: from there every other insertion looks dear except one a little further on, so a search left running chases the trajectory outward - in closed-loop simulation it walked a settled goal from 13 to 20 degrees over the last third of a burn, and the burn diverged. So the goal freezes once FreezeVgoFraction of the dV the first search saw has been flown, or once tgo is under FreezeTgoS, whichever comes first.
//
// THE OTHER GUARDS, each for a reason measured in closed-loop simulation of this solver:
//  - Climbing side only, from periapsis to MaxNuDeg past it. Arriving before periapsis means arriving descending, and burns from orbit that had to do that diverged.
//  - A move has to be worth MinSavingMs. On an ordinary low orbit the whole cost curve is a few m/s deep, and following noise that shallow would only stir the steering.
//  - A near-circular target is left alone: every point of a circle is the same insertion.
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

    /// <summary>
    /// Solves each probe gets. Enough for a candidate's step away from the live target to settle into the drift every probe shares - after that the difference between two probes barely moves, even while each keeps drifting.
    /// </summary>
    public const int ProbeSolves = 40;

    // The live vgo when the first search of this flight ran, NaN before it has.
    private double _firstSearchVgo = double.NaN;

    /// <summary>The insertion true anomaly to fly now, rad, slewing toward <see cref="GoalNu"/>.</summary>
    public double Nu { get; private set; }

    /// <summary>Where the last search put the cheapest insertion it could see, rad.</summary>
    public double GoalNu { get; private set; }

    /// <summary>What the goal saves over inserting at periapsis (or the floor), m/s, by the last search's own costs. NaN before one has run.</summary>
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
    /// One guidance cycle, run straight after the live solve with the same state and stage model, so the probes start from a solution of exactly this instant. <paramref name="enabled"/> is whether a free insertion is being optimised at all - off, the goal returns to periapsis. <paramref name="allowed"/> is whether the live solution can be searched from right now (converged, flying closed loop). The slew toward the goal runs either way, so a search that stops still lands the insertion where the last one pointed.
    /// </summary>
    public void Step(double time, double dt, bool enabled, bool allowed, UpfgGuidance live, UpfgTarget target,
                     double3 r, double3 v, double mass, double mu, UpfgVehicle vehicle)
    {
        if (!enabled)
        {
            GoalNu = 0.0;
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
                if (double.IsNaN(_firstSearchVgo))
                    _firstSearchVgo = live.VgoMag;
                Search(live, target, r, v, mass, mu, vehicle);
            }
        }

        double maxMove = DegToRad(NuRateDegS) * Math.Max(dt, 0.0);
        Nu += Math.Clamp(GoalNu - Nu, -maxMove, maxMove);
    }

    private void Search(UpfgGuidance live, UpfgTarget target, double3 r, double3 v, double mass, double mu,
                        UpfgVehicle vehicle)
    {
        LastSearchSolves = 0;
        double step = DegToRad(ProbeStepDeg);

        // The floor can put the lowest insertion past periapsis, and below it every candidate is the same insertion.
        double lowest = target.TrueAnomalyAtRadius(target.MinCutoffRadius);
        double highest = DegToRad(MaxNuDeg);
        if (highest - lowest < 2.0 * step)
            return;

        double mid = Math.Clamp(GoalNu, lowest + step, highest - step);
        if (!TryCost(live, target, mid - step, r, v, mass, mu, vehicle, out double below)
            || !TryCost(live, target, mid, r, v, mass, mu, vehicle, out double here)
            || !TryCost(live, target, mid + step, r, v, mass, mu, vehicle, out double above))
            return;

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
        else if (TryCost(live, target, lowest, r, v, mass, mu, vehicle, out double atLowest))
            SavingMs = atLowest - goalCost;
    }

    // vgo after a copy of the live solver has spent ProbeSolves solves on the insertion at true anomaly nu. False when the copy went non-finite, which resets it and reads as a zero tgo.
    private bool TryCost(UpfgGuidance live, UpfgTarget target, double nu, double3 r, double3 v, double mass,
                         double mu, UpfgVehicle vehicle, out double cost)
    {
        cost = double.NaN;
        UpfgGuidance probe = live.Clone();
        UpfgTarget pinned = target.WithFreeInsertion(nu);
        for (int i = 0; i < ProbeSolves; i++)
        {
            probe.Step(r, v, mass, mu, pinned, vehicle, 1, 0.0);
            LastSearchSolves++;
            if (!double.IsFinite(probe.Tgo) || !double.IsFinite(probe.VgoMag) || probe.Tgo <= 0.0)
                return false;
        }
        cost = probe.VgoMag;
        return true;
    }

    private static double DegToRad(double d) => d * Math.PI / 180.0;
}
