using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.MissionPlanner;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Transfer;

/// <summary>
/// The correction burn of a craft on its way to one of its parent's moons, aimed at a periapsis there.
/// FlybyTargeting aims the burn at the requested periapsis, and the game's own patched-conic propagation (HohmannFlybyUI.BuildPreview) has to confirm the encounter before the plan is offered.
/// The delta v is refined for the finite burn as the flyby preview refines it, so the burn that is created is the burn that was checked.
/// The periapsis at the moon moves by hundreds of kilometres for each tenth of a m/s at a departure from a low parking orbit, so no flown departure keeps it: TryPlanCorrection aims the flown orbit again on the way.
/// </summary>
internal static class MoonInjectionPlanner
{
    // The aim is corrected until the propagated periapsis altitude is within this fraction of the requested one, or within the floor for a low flyby.
    private const int PeriapsisCorrections = 4;
    private const double PeriapsisTolerance = 0.03;
    private const double PeriapsisToleranceFloorM = 2000.0;

    /// <summary>The planned injection or correction: the burn time and the finite-burn delta v in VLF, as Burn.Create takes them, with what the propagation predicts. A correction that keeps the side the craft already passes on has no side.</summary>
    internal readonly record struct MoonInjection(
        UniverseTime BurnTime,
        double3 DvVlf,
        double ImpulsiveDvMs,
        UniverseTime Arrival,
        double PredictedPeriapsisRadius,
        FlybySide? Side);

    // Aims one departure past the moon on each side and keeps the cheapest side whose encounter the game's propagation confirms.
    // The two-body aim misses the requested periapsis by the moon's pull before its sphere of influence and by the finite burn, so the requested radius is corrected by the propagated miss, a few secant steps at most.
    private static bool TryAim(Vehicle vehicle, Celestial moon, IParentBody moonBody, UniverseTime start, UniverseTime transit,
        double rp, double surface, out MoonInjection plan, ref string why)
    {
        plan = default;
        bool found = false;
        double tolerance = Math.Max(PeriapsisToleranceFloorM, PeriapsisTolerance * (rp - moonBody.MeanRadius));
        foreach (FlybySide side in Enum.GetValues<FlybySide>())
        {
            double aim = rp, lastAim = double.NaN, lastPredicted = double.NaN;
            for (int pass = 0; pass <= PeriapsisCorrections; pass++)
            {
                FlybyTargeting.FlybyOutcome outcome = FlybyTargeting.ComputeFlybyDeparture(vehicle, moon, start, transit, aim, side);
                if (outcome.Result is not FlybyTargeting.FlybyResult departure)
                    break;
                PatchedConic? prePatch = vehicle.FlightPlan.TryFindPatch(departure.BurnTime);
                if (prePatch == null)
                {
                    why = "NO FLIGHT-PLAN PATCH AT INJECTION";
                    break;
                }
                FlybyPrediction prediction = HohmannFlybyUI.BuildPreview(vehicle, moon, moonBody, departure, out _);
                if (prediction.Status != FlybyPredictionStatus.Available || prediction.PeriapsisRadius is not double predicted)
                {
                    if (prediction.Reason.Length > 0)
                        why = prediction.Reason.ToUpperInvariant().TrimEnd('.');
                    break;
                }
                if (predicted > surface)
                {
                    // A side that reaches the requested periapsis beats one that does not, and between two that do, or two that do not, the cheaper wins.
                    double impulsive = departure.DvVlf.Length();
                    bool hits = Math.Abs(predicted - rp) <= tolerance;
                    bool planHits = found && Math.Abs(plan.PredictedPeriapsisRadius - rp) <= tolerance;
                    if (!found || (hits && !planHits) || (hits == planHits && impulsive < plan.ImpulsiveDvMs))
                    {
                        double3 dvVlf = HohmannFlybyUI.RefineDeparture(vehicle, prePatch, departure);
                        plan = new MoonInjection(departure.BurnTime, dvVlf, impulsive,
                            departure.Arrival ?? start + transit, predicted, side);
                        found = true;
                    }
                }
                else
                    why = "PREDICTED PERIAPSIS BELOW THE SURFACE";
                if (Math.Abs(predicted - rp) <= tolerance)
                    break;
                // A secant step once two aims are known, a unit step before that.
                double slope = double.IsNaN(lastAim) || Math.Abs(predicted - lastPredicted) < 1.0 ? 1.0 : (aim - lastAim) / (predicted - lastPredicted);
                lastAim = aim;
                lastPredicted = predicted;
                aim = Math.Max(surface * 1.01, aim + (rp - predicted) * Math.Clamp(slope, 0.2, 5.0));
            }
        }
        return found;
    }

    // A correction that costs more than this is not a correction of a flown injection any more, m/s.
    private const double MaxCorrectionDvMs = 250.0;

    // The least time between a correction and the arrival, s. Closer than this the moon's pull bends the path that the two-body aim treats as straight.
    private const double MinCorrectionFlightS = 3600.0;

    // The finite-difference step of the correction's gradient, m/s, and the most secant steps on its size.
    private const double GradientStepMs = 0.05;
    private const int NudgeIterations = 6;

    // Samples of the closest-approach scan that seeds a correction's arrival time when the flight plan holds no encounter.
    private const int ApproachSamples = 400;

    /// <summary>The periapsis radius at <paramref name="moonBody"/> on the craft's own flight plan, which is the game's patched-conic propagation of the orbit as flown, m. NaN when the plan holds no encounter.</summary>
    public static double PredictedPeriapsisRadius(Vehicle vehicle, IParentBody moonBody)
        => FlybyPrediction.FromPlan(vehicle.FlightPlan, moonBody).PeriapsisRadius ?? double.NaN;

    /// <summary>The time the craft's flight plan enters the sphere of influence of <paramref name="moonBody"/>, or null when the plan holds no encounter.</summary>
    public static UniverseTime? SphereEntryTime(Vehicle vehicle, IParentBody moonBody)
    {
        foreach (PatchedConic patch in vehicle.FlightPlan.Patches)
            if (patch.Orbit.Parent?.Id == moonBody.Id)
                return patch.StartTime;
        return null;
    }

    /// <summary>
    /// Plans one burn at <paramref name="burnTime"/> that aims the craft's orbit toward <paramref name="moon"/> at <paramref name="periapsisAltitudeM"/> again. The arrival time stays the one of the orbit as flown, so the burn only moves the aim point.
    /// With an encounter on the flight plan it is the smallest burn that moves the periapsis (TryNudge). Without one it aims anew (TryAim). False with the reason in <paramref name="refusal"/>.
    /// </summary>
    public static bool TryPlanCorrection(Vehicle vehicle, Celestial moon, double periapsisAltitudeM, UniverseTime burnTime,
        out MoonInjection plan, out string refusal)
    {
        plan = default;
        refusal = "";
        Orbit orbit = vehicle.Orbit;
        if (moon is not IParentBody moonBody || moon.Orbit?.Parent is not IParentBody home
            || orbit?.Parent == null || orbit.Parent.Id != home.Id)
        {
            refusal = "CRAFT IS NOT ON THE WAY TO THE MOON";
            return false;
        }
        if (ArrivalOnFlownOrbit(vehicle, orbit, moon, moonBody, burnTime) is not UniverseTime arrival
            || !((arrival - burnTime).Seconds() >= MinCorrectionFlightS))
        {
            refusal = "NO APPROACH LEFT TO CORRECT";
            return false;
        }

        double rp = moonBody.MeanRadius + periapsisAltitudeM;
        string why = "NO ENCOUNTER IN PREDICTION";
        if (!TryNudge(vehicle, moon, moonBody, burnTime, rp, out plan)
            && !TryAim(vehicle, moon, moonBody, burnTime, arrival - burnTime, rp, moonBody.GetNearSurfaceRadius(), out plan, ref why))
        {
            refusal = why;
            return false;
        }
        if (plan.DvVlf.Length() > MaxCorrectionDvMs)
        {
            refusal = $"THE CORRECTION WOULD COST {plan.DvVlf.Length():F0} M/S";
            plan = default;
            return false;
        }
        return true;
    }

    // The smallest burn at burnTime that brings the periapsis of the encounter on the flight plan to rp, with the arrival time free.
    // The miss is measured as an impact parameter, b = rp sqrt((e + 1) / (e - 1)) on the hyperbola the game propagates, because b is close to linear in the burn while the periapsis radius flattens out toward the centre of the body.
    // The gradient of the miss over the three VLF axes comes from central differences of the game's own propagation. The burn along the gradient is the minimum-norm one, and its size is found by secant steps on the same propagation.
    private static bool TryNudge(Vehicle vehicle, Celestial moon, IParentBody moonBody, UniverseTime burnTime, double rp, out MoonInjection plan)
    {
        plan = default;
        if (vehicle.FlightPlan.TryFindPatch(burnTime) is not PatchedConic prePatch
            || !TryMiss(vehicle.FlightPlan, moonBody, rp, out double miss, out _, out _))
            return false;

        double3 gradient = default;
        for (int axis = 0; axis < 3; axis++)
        {
            double3 step = new double3(axis == 0 ? GradientStepMs : 0.0, axis == 1 ? GradientStepMs : 0.0, axis == 2 ? GradientStepMs : 0.0);
            if (!TryBurnMiss(step, out double plus, out _, out _) || !TryBurnMiss(-step, out double minus, out _, out _))
                return false;
            double slope = (plus - minus) / (2.0 * GradientStepMs);
            gradient += new double3(axis == 0 ? slope : 0.0, axis == 1 ? slope : 0.0, axis == 2 ? slope : 0.0);
        }
        double steepest = gradient.Length();
        if (!(steepest > 0.0) || !double.IsFinite(steepest))
            return false;
        double3 direction = gradient / steepest;

        double tolerance = Math.Max(PeriapsisToleranceFloorM, PeriapsisTolerance * (rp - moonBody.MeanRadius));
        double size = 0.0, lastSize = double.NaN, lastMiss = double.NaN;
        for (int pass = 0; pass < NudgeIterations; pass++)
        {
            double slope = double.IsNaN(lastSize) || Math.Abs(miss - lastMiss) < 1.0 ? steepest : (miss - lastMiss) / (size - lastSize);
            if (!(Math.Abs(slope) > 0.0))
                return false;
            lastSize = size;
            lastMiss = miss;
            size = Math.Clamp(size - miss / slope, -MaxCorrectionDvMs, MaxCorrectionDvMs);
            if (size == lastSize)
                return false;
            if (!TryBurnMiss(direction * size, out miss, out double periapsis, out UniverseTime arrival))
                return false;
            if (Math.Abs(periapsis - rp) <= tolerance)
            {
                double3 dvVlf = HohmannFlybyUI.RefineDeparture(vehicle, prePatch, Departure(direction * size));
                plan = new MoonInjection(burnTime, dvVlf, Math.Abs(size), arrival, periapsis, null);
                return true;
            }
        }
        return false;

        // The burn as a departure without an arrival goal, so the preview and the node keep the conic of the impulsive burn (FiniteBurnGoal.Transfer).
        FlybyTargeting.FlybyResult Departure(double3 dvVlf)
            => new(dvVlf, FiniteBurns.VlfToCci(prePatch.Orbit, burnTime, dvVlf), burnTime, double.NaN, double.NaN, rp,
                IsCrossParent: false, PlannerVInfMs: double.NaN, PlannerApoTargetMeters: double.NaN);

        bool TryBurnMiss(double3 dvVlf, out double burnMiss, out double periapsis, out UniverseTime arrival)
        {
            burnMiss = periapsis = double.NaN;
            arrival = default;
            return HohmannFlybyUI.BuildPreview(vehicle, moon, moonBody, Departure(dvVlf), out FlightPlan? preview).Status == FlybyPredictionStatus.Available
                && preview != null && TryMiss(preview, moonBody, rp, out burnMiss, out periapsis, out arrival);
        }
    }

    // The miss of the first encounter with the moon on a plan: its impact parameter minus the one that gives the periapsis rp at the same approach speed, m.
    private static bool TryMiss(FlightPlan plan, IParentBody moonBody, double rp, out double miss, out double periapsis, out UniverseTime arrival)
    {
        miss = periapsis = double.NaN;
        arrival = default;
        foreach (PatchedConic patch in plan.Patches)
        {
            if (patch.Orbit.Parent?.Id != moonBody.Id)
                continue;
            double ecc = patch.Orbit.Eccentricity;
            periapsis = patch.Orbit.Periapsis;
            if (!(ecc > 1.0) || !(periapsis > 0.0))
                return false;
            double excessSquared = moonBody.Mu * (ecc - 1.0) / periapsis;
            miss = periapsis * Math.Sqrt((ecc + 1.0) / (ecc - 1.0)) - Math.Sqrt(rp * rp + 2.0 * moonBody.Mu * rp / excessSquared);
            arrival = patch.Orbit.TimeAtPeriapsis;
            return double.IsFinite(miss);
        }
        return false;
    }

    // The arrival the correction keeps: the periapsis of the encounter on the flight plan, or without an encounter the closest approach of the flown orbit to the moon within one revolution.
    private static UniverseTime? ArrivalOnFlownOrbit(Vehicle vehicle, Orbit orbit, Celestial moon, IParentBody moonBody, UniverseTime from)
    {
        foreach (PatchedConic patch in vehicle.FlightPlan.Patches)
            if (patch.Orbit.Parent?.Id == moonBody.Id)
                return patch.Orbit.Eccentricity >= 1.0 ? patch.Orbit.TimeAtPeriapsis : patch.StartTime;

        double span = 2.0 * LunarTransferGeometry.HohmannTime(orbit.Parent.Mu, orbit.Periapsis, moon.Orbit.SemiMajorAxis);
        if (orbit.Eccentricity < 1.0 && orbit.Period > 0.0)
            span = Math.Min(span, orbit.Period);
        double step = span / ApproachSamples;
        if (!(step > 0.0))
            return null;
        double best = double.NaN, bestDistance = double.MaxValue;
        for (int i = 1; i <= ApproachSamples; i++)
        {
            double distance = Distance(i * step);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                best = i * step;
            }
        }
        if (double.IsNaN(best))
            return null;
        double refined = MathEx.BrentMin(Distance, Math.Max(step, best - step), Math.Min(span, best + step), 1e-06);
        return from + (double.IsFinite(refined) ? refined : best);

        // Seconds from the burn rather than absolute sim seconds, so the search keeps its precision late in a save.
        double Distance(double dt)
        {
            UniverseTime t = from + dt;
            double d = (orbit.GetStateVectorsAt(t).PositionCci - moon.Orbit.GetStateVectorsAt(t).PositionCci).Length();
            return double.IsFinite(d) ? d : double.MaxValue;
        }
    }
}
