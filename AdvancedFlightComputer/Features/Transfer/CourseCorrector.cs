using System;
using System.Collections.Generic;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Transfer;

/// <summary>
/// The course check and the correction burn of a craft on its way to a moon of its parent, after a departure burn that aimed at a periapsis there.
/// No flown injection from a low parking orbit keeps that periapsis, because it moves by hundreds of kilometres for each tenth of a m/s of the burn, so every departure AFC plans is followed by this check.
/// The check reads the craft's own flight plan, and the correction comes from MoonInjectionPlanner.TryPlanCorrection. Who flies the node is the caller's choice.
/// </summary>
internal static class CourseCorrector
{
    /// <summary>The game rebuilds the flight plan over several frames after a burn. The course is read this long after one, s.</summary>
    internal const double SettleS = 30.0;

    /// <summary>The fewest steps between a burn and the reading of the course, for a caller that also counts them.</summary>
    internal const int SettleSteps = 30;

    /// <summary>The most corrections after one departure. The second one flies what the execution of the first left over.</summary>
    internal const int MaxCorrections = 2;

    // The flown periapsis is on course within this fraction of the requested altitude, or within the floor for a low one.
    private const double PeriapsisTolerance = 0.10;
    private const double PeriapsisToleranceFloorM = 10_000.0;

    // Where a correction may be placed: half an hour after the check that asks for it, or at one of these fractions of the time left to the moon's sphere.
    // An early correction is the cheapest, but a burn of a few cm/s is lost in what the turn to the burn attitude itself adds to the orbit, so the earliest place whose burn is at least LeastDvMs is taken, and the last place when none is.
    private const double FirstCorrectionS = 1800.0;
    private static readonly double[] CorrectionShares = { 0.25, 0.5, 0.75 };
    private const double LeastDvMs = 1.0;

    /// <summary>Whether a periapsis radius at <paramref name="moonBody"/> is close enough to the requested altitude above its mean radius. False for NaN, which stands for no encounter.</summary>
    public static bool OnCourse(double periapsisRadius, IParentBody moonBody, double altitudeM)
        => Math.Abs(periapsisRadius - (moonBody.MeanRadius + altitudeM)) <= Math.Max(PeriapsisToleranceFloorM, PeriapsisTolerance * altitudeM);

    /// <summary>True while the burn plan is not free for a node of the caller: a node is planned, a burn is loaded, or stock Auto is selected.</summary>
    public static bool BurnPlanned(Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        return fc.BurnPlan.BurnCount > 0 || fc.Burn != null || fc.BurnMode == FlightComputerBurnMode.Auto;
    }

    /// <summary>
    /// Plans the correction at the earliest place whose burn is large enough to be flown well, or at the last place that has a plan.
    /// Returns the refusal of the first place when no place has one, and an empty text with the plan otherwise.
    /// </summary>
    public static string TryPlan(Vehicle vehicle, Celestial moon, IParentBody moonBody, double altitudeM, double now,
        out MoonInjectionPlanner.MoonInjection plan)
    {
        plan = default;
        var leads = new List<double> { FirstCorrectionS };
        if (MoonInjectionPlanner.SphereEntryTime(vehicle, moonBody) is UniverseTime entry)
            foreach (double share in CorrectionShares)
                if (share * (entry.Seconds() - now) > leads[^1])
                    leads.Add(share * (entry.Seconds() - now));
        string refusal = "";
        bool found = false;
        foreach (double lead in leads)
        {
            if (!MoonInjectionPlanner.TryPlanCorrection(vehicle, moon, altitudeM, new UniverseTime(now + lead),
                    out MoonInjectionPlanner.MoonInjection candidate, out string why))
            {
                if (refusal.Length == 0)
                    refusal = why;
                continue;
            }
            plan = candidate;
            found = true;
            if (candidate.DvVlf.Length() >= LeastDvMs)
                break;
        }
        return found ? "" : refusal;
    }

    /// <summary>
    /// Adds the node of a planned burn to an empty burn plan and checks that stock loaded it. FlightComputer.AddBurn loads the first executable burn in Manual, so the node flies only when somebody selects Auto for it.
    /// Call it on the main thread while no vehicle worker runs. Returns the refusal, empty when the node is loaded.
    /// </summary>
    public static string TryAddNode(Vehicle vehicle, UniverseTime burnTime, double3 dvVlf, out Burn? node)
    {
        node = null;
        if (BurnPlanned(vehicle))
            return "A BURN IS ALREADY PLANNED";
        if (!vehicle.IsControllable)
            return "THE CRAFT HAS NO CONTROL";
        PatchedConic? patch = vehicle.FlightPlan.TryFindPatch(burnTime);
        if (patch == null)
            return "NO FLIGHT-PLAN PATCH AT THE BURN";
        FlightComputer fc = vehicle.FlightComputer;
        Burn added = Burn.Create(patch.Orbit.GetPointAt(burnTime), burnTime.Seconds(), dvVlf, patch, vehicle);
        added.IsGizmoActive = false;
        fc.AddBurn(added);
        if (!ReferenceEquals(fc.BurnPlan.FindFirstExecutableBurn(), added) || fc.Burn == null)
        {
            if (fc.BurnPlan.TryGetBurn(added))
                fc.RemoveBurn(added);
            return "STOCK COULD NOT LOAD THE BURN";
        }
        node = added;
        return "";
    }
}
