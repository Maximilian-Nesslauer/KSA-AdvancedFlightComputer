using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.MissionPlanner;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Transfer;

/// <summary>
/// The injection burn from a parking orbit to a flyby of one of its parent's moons, chosen without a porkchop window.
/// A grid of Lambert solves (OrbitalTransfers.SuperiorLambert, aimed at the moon's centre) over the departures of the next parking revolutions and the flight times around the Hohmann time finds the cheap departures.
/// FlybyTargeting then aims each, cheapest first, at the requested periapsis, and the game's own patched-conic propagation (HohmannFlybyUI.BuildPreview) has to confirm the encounter before the plan is offered.
/// The delta v is refined for the finite burn as the flyby preview refines it, so the burn that is created is the burn that was checked.
/// The periapsis at the moon moves by hundreds of kilometres for each tenth of a m/s at an injection from a low parking orbit, so no flown injection keeps it: TryPlanCorrection aims the flown orbit again on the way, and TryPlanCapture plans the burn into an orbit of the moon at the periapsis.
/// </summary>
internal static class MoonInjectionPlanner
{
    // The coarse grid covers this many parking revolutions from the earliest departure. The phase to the moon comes round once a revolution, so one and a half always holds the best departure.
    private const double SearchRevolutions = 1.5;
    private const int StartSamples = 144;
    private const int FlightSamples = 24;

    // Flight times as fractions of the Hohmann time. A transfer of exactly 180 degrees has no plane, so the cheapest one stops just short of the Hohmann time.
    private const double ShortestFlight = 0.45;
    private const double LongestFlight = 1.05;

    // The fine grid spans one coarse cell either side of a coarse candidate.
    private const int FineSamples = 16;
    private const int MaxCandidates = 6;

    // The most an injection may cost against the ideal Hohmann burn. A moon's eccentric orbit and a few degrees out of plane stay well inside it.
    private const double MaxDvOverHohmann = 1.25;

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

    /// <summary>
    /// Plans the injection from <paramref name="vehicle"/>'s orbit to a flyby of <paramref name="moon"/> at <paramref name="periapsisAltitudeM"/> above its mean radius, with the burn no earlier than <paramref name="notBefore"/>.
    /// False with the reason in <paramref name="refusal"/> when the craft does not orbit the moon's parent on a closed orbit, or no departure leads to an encounter.
    /// </summary>
    public static bool TryPlan(Vehicle vehicle, Celestial moon, double periapsisAltitudeM, UniverseTime notBefore,
        out MoonInjection plan, out string refusal)
    {
        plan = default;
        refusal = "";
        Orbit parking = vehicle.Orbit;
        if (moon is not IParentBody moonBody || moon.Orbit?.Parent is not IParentBody home)
        {
            refusal = "NO MOON TO FLY TO";
            return false;
        }
        if (parking?.Parent == null || parking.Parent.Id != home.Id)
        {
            refusal = $"CRAFT DOES NOT ORBIT {home.Id.ToUpperInvariant()}";
            return false;
        }
        if (!(parking.Eccentricity < 1.0) || !(parking.Period > 0.0))
        {
            refusal = "NO CLOSED PARKING ORBIT";
            return false;
        }

        double mu = home.Mu;
        double parkingSma = parking.SemiMajorAxis;
        double hohmann = LunarTransferGeometry.HohmannTime(mu, parkingSma, moon.Orbit.SemiMajorAxis);
        double start0 = notBefore.Seconds();
        double startSpan = SearchRevolutions * parking.Period;
        double flightMin = ShortestFlight * hohmann;
        double flightSpan = (LongestFlight - ShortestFlight) * hohmann;

        // The coarse grid keeps the cheapest flight time of every departure. Each local minimum over the departures is a candidate, cheapest first: the cheapest one can still miss the sphere of influence when the moon lies far out of the parking plane, because the plane of a transfer near 180 degrees is badly conditioned.
        double startStep = startSpan / StartSamples, flightStep = flightSpan / FlightSamples;
        var costs = new double[StartSamples + 1];
        var flights = new double[StartSamples + 1];
        for (int i = 0; i <= StartSamples; i++)
        {
            costs[i] = double.MaxValue;
            for (int j = 0; j <= FlightSamples; j++)
            {
                double flight = flightMin + j * flightStep;
                double cost = LambertCost(start0 + i * startStep, flight);
                if (cost < costs[i])
                {
                    costs[i] = cost;
                    flights[i] = flight;
                }
            }
        }
        var candidates = new List<int>();
        for (int i = 0; i <= StartSamples; i++)
            if (costs[i] < double.MaxValue
                && (i == 0 || costs[i] <= costs[i - 1]) && (i == StartSamples || costs[i] <= costs[i + 1]))
                candidates.Add(i);
        candidates.Sort((a, b) => costs[a].CompareTo(costs[b]));
        if (candidates.Count == 0)
        {
            refusal = "NO TRANSFER SOLUTION";
            return false;
        }

        // A parking plane far from the moon's position at arrival still has a Lambert solution, but its delta v is a plane change of several km/s that no craft is built for. Such a plan is refused with the angle that causes it rather than offered.
        double limit = MaxDvOverHohmann * HohmannDv(mu, parkingSma, moon.Orbit.SemiMajorAxis);
        int cheapest = candidates[0];
        if (costs[cheapest] > limit)
        {
            double3 normal = parking.GetOrbitNormalCci();
            double3 moonThen = moon.Orbit.GetStateVectorsAt(new UniverseTime(start0 + cheapest * startStep + flights[cheapest])).PositionCci;
            double outOfPlaneDeg = Math.Asin(Math.Clamp(double3.Dot(normal, double3.Normalize(moonThen)), -1.0, 1.0)) * 180.0 / Math.PI;
            refusal = $"{moon.Id.ToUpperInvariant()} IS {Math.Abs(outOfPlaneDeg):F0} DEG OUT OF THE PARKING PLANE AT ARRIVAL, THE INJECTION WOULD COST {costs[cheapest]:F0} M/S. "
                + $"LAUNCH INTO A PLANE THAT HOLDS {moon.Id.ToUpperInvariant()}";
            return false;
        }

        double rp = moonBody.MeanRadius + periapsisAltitudeM;
        double surface = moonBody.GetNearSurfaceRadius();
        string why = "NO ENCOUNTER IN PREDICTION";
        for (int c = 0; c < Math.Min(candidates.Count, MaxCandidates); c++)
        {
            int i = candidates[c];
            if (costs[i] > limit)
                break;
            double bestCost = double.MaxValue, bestStart = double.NaN, bestFlight = double.NaN;
            for (int a = -FineSamples; a <= FineSamples; a++)
                for (int b = -FineSamples; b <= FineSamples; b++)
                {
                    double startS = Math.Max(start0, start0 + i * startStep + a * startStep / FineSamples);
                    double flightS = flights[i] + b * flightStep / FineSamples;
                    double cost = LambertCost(startS, flightS);
                    if (cost < bestCost)
                    {
                        bestCost = cost;
                        bestStart = startS;
                        bestFlight = flightS;
                    }
                }
            if (double.IsNaN(bestStart))
                continue;
            if (TryAim(vehicle, moon, moonBody, new UniverseTime(bestStart), new UniverseTime(bestFlight), rp, surface, out plan, ref why))
                return true;
        }
        refusal = why;
        return false;

        double LambertCost(double startS, double flightS)
        {
            if (!(flightS > 0.0))
                return double.MaxValue;
            StateVectors from = parking.GetStateVectorsAt(new UniverseTime(startS));
            double3 to = moon.Orbit.GetStateVectorsAt(new UniverseTime(startS + flightS)).PositionCci;
            OrbitalTransfers.SuperiorLambert(mu, from.PositionCci, to, new UniverseTime(flightS), out double3 departure, out _);
            double cost = (departure - from.VelocityCci).Length();
            return double.IsFinite(cost) ? cost : double.MaxValue;
        }
    }

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

    // The largest apoapsis of a capture with less delta v than the circular orbit needs, as a fraction of the moon's sphere of influence.
    private const double MaxCaptureApoapsisOverSphere = 0.5;

    // The share of the craft's remaining delta v a capture may plan with.
    private const double CaptureBudgetShare = 0.97;

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
    /// With an encounter on the flight plan it is the smallest burn that moves the periapsis (TryNudge). Without one it aims anew as the injection does. False with the reason in <paramref name="refusal"/>.
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

    /// <summary>The capture burn at the periapsis of a flyby: the burn time and the finite-burn delta v in VLF, as Burn.Create takes them, with the orbit the game's propagation of the burn predicts.</summary>
    internal readonly record struct MoonCapture(
        UniverseTime BurnTime,
        double3 DvVlf,
        double PeriapsisRadius,
        double ApoapsisRadius,
        bool Circular);

    /// <summary>
    /// Plans the retrograde burn at the periapsis of the craft's flyby orbit around <paramref name="moonBody"/> that leaves it in a circular orbit at that periapsis.
    /// With less than that in <paramref name="availableDvMs"/> the burn takes what the craft has and leaves an ellipse, as long as its apoapsis stays well inside the sphere of influence.
    /// False with the reason in <paramref name="refusal"/>.
    /// </summary>
    public static bool TryPlanCapture(Vehicle vehicle, IParentBody moonBody, double availableDvMs, UniverseTime notBefore,
        out MoonCapture plan, out string refusal)
    {
        plan = default;
        refusal = "";
        Orbit orbit = vehicle.Orbit;
        string moonName = moonBody.Id.ToUpperInvariant();
        if (orbit?.Parent == null || orbit.Parent.Id != moonBody.Id)
        {
            refusal = $"CRAFT IS NOT IN THE SPHERE OF {moonName}";
            return false;
        }
        if (orbit.Eccentricity < 1.0)
        {
            refusal = $"CRAFT ALREADY ORBITS {moonName}";
            return false;
        }
        // Orbit.GetNextPeriapsisTime gives the one periapsis of an open orbit, also when it has passed.
        if (orbit.GetNextPeriapsisTime(notBefore) is not UniverseTime burnTime || burnTime <= notBefore)
        {
            refusal = "THE PERIAPSIS IS TOO CLOSE OR HAS PASSED";
            return false;
        }
        double rp = orbit.Periapsis;
        if (!(rp > moonBody.GetNearSurfaceRadius()))
        {
            refusal = "THE PERIAPSIS IS BELOW THE SURFACE";
            return false;
        }
        PatchedConic? prePatch = vehicle.FlightPlan.TryFindPatch(burnTime);
        if (prePatch == null || prePatch.Orbit.Parent?.Id != moonBody.Id)
        {
            refusal = "NO FLIGHT-PLAN PATCH AT THE PERIAPSIS";
            return false;
        }

        StateVectors atPeriapsis = orbit.GetStateVectorsAt(burnTime);
        double speed = atPeriapsis.VelocityCci.Length();
        double mu = moonBody.Mu;
        double forCircle = speed - Math.Sqrt(mu / rp);
        double dv = forCircle;
        bool circular = true;
        if (!(availableDvMs * CaptureBudgetShare >= forCircle))
        {
            circular = false;
            dv = availableDvMs * CaptureBudgetShare;
            double after = speed - dv;
            double energy = 0.5 * after * after - mu / rp;
            double apoapsis = energy < 0.0 ? -mu / energy - rp : double.PositiveInfinity;
            if (!(dv > 0.0) || !(apoapsis <= MaxCaptureApoapsisOverSphere * moonBody.SphereOfInfluence))
            {
                refusal = $"A CAPTURE AT {moonName} NEEDS {forCircle:F0} M/S FOR A CIRCLE, THE CRAFT HAS {Math.Max(availableDvMs, 0.0):F0} M/S";
                return false;
            }
        }

        // Retrograde at the periapsis, in the VLF frame of that point.
        double3 dvCci = atPeriapsis.VelocityCci * (-dv / speed);
        double3 dvVlf = dvCci.Transform(atPeriapsis.GetVlf2ParentCci().OrIdentity().Inverse());
        dvVlf = FiniteBurns.Refine(orbit, burnTime, dvVlf, FiniteBurns.FullThrottlePhases(vehicle),
            circular ? FiniteBurnGoal.Circular : FiniteBurnGoal.Apsis).DvVlf;
        // A refinement can ask for more than the craft has when the burn is long. The budget wins, and the propagation below says what orbit that leaves.
        if (!circular && dvVlf.Length() > dv)
            dvVlf *= dv / dvVlf.Length();

        if (!TryPropagateBurn(vehicle, prePatch, burnTime, dvVlf, out Orbit? captured) || captured == null
            || !(captured.Eccentricity < 1.0) || !(captured.Periapsis > moonBody.GetNearSurfaceRadius())
            || !(captured.Apoapsis <= moonBody.SphereOfInfluence))
        {
            refusal = "THE PREDICTED ORBIT AFTER THE CAPTURE BURN IS NOT A SAFE ORBIT";
            return false;
        }
        plan = new MoonCapture(burnTime, dvVlf, captured.Periapsis, captured.Apoapsis, circular);
        return true;
    }

    // The orbit after the burn as the game propagates it: the finite burn of FlightPlan.TryAddFiniteBurn for a craft with thrust, as the flyby preview builds it, and the impulsive patch of FlightPlan.CalculateBurnPatch otherwise.
    private static bool TryPropagateBurn(Vehicle vehicle, PatchedConic prePatch, UniverseTime burnTime, double3 dvVlf, out Orbit? after)
    {
        after = null;
        try
        {
            FlightPlan plan = FlightPlan.CreateUninitialized(vehicle.Hash);
            plan.ImpactClearanceMargin = vehicle.BoundingSphereRadiusBody;
            PatchedConic? piece = FiniteBurns.TryAddFiniteBurn(plan, prePatch, burnTime, dvVlf,
                FiniteBurns.FullThrottlePhases(vehicle), out bool impacted);
            if (impacted)
                return false;
            piece ??= plan.CalculateBurnPatch(prePatch, prePatch.Orbit.GetTimeSincePeriapsisThisOrbit(burnTime), dvVlf, burnTime);
            after = piece?.Orbit;
            return after != null;
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("moon-capture-preview:" + ex.GetType().Name,
                $"[AFC] Capture preview: vehicle='{vehicle.Id}' burn={burnTime.Seconds():F1}s: {ex}");
            return false;
        }
    }

    /// <summary>The delta v of the first burn of the Hohmann transfer from a circular orbit of radius <paramref name="r1"/> out to <paramref name="r2"/>, m/s.</summary>
    public static double HohmannDv(double mu, double r1, double r2)
        => Math.Sqrt(mu / r1) * (Math.Sqrt(2.0 * r2 / (r1 + r2)) - 1.0);
}
