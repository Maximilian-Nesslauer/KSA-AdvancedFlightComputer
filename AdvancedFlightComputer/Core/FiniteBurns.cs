using System;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Core;

internal enum FiniteBurnGoalKind
{
    // Reach the conic that the impulsive burn would give. FiniteBurnRefiner.RefineTransfer with no arrival does this, and it matches the exit velocity instead when that conic leaves the SOI.
    SameOrbit,
    Circular,
    // Reach the plane of PlaneTarget, or of the impulsive post-burn orbit when it is null. This can move the burn time.
    Plane,
    // Reach the exit velocity of the impulsive transfer, or its position at Arrival when it stays inside the SOI.
    Transfer,
    // Reach the radius of the apsis opposite the burn on the impulsive orbit, with the burn direction kept. A circular impulsive orbit uses the Circular goal.
    Apsis,
    // Reach the semi-major axis, and so the period, of the impulsive orbit, with the burn direction kept.
    Period,
}

internal readonly record struct FiniteBurnGoal(
    FiniteBurnGoalKind Kind, Orbit? PlaneTarget = null, UniverseTime? Arrival = null)
{
    public static FiniteBurnGoal SameOrbit => new(FiniteBurnGoalKind.SameOrbit);
    public static FiniteBurnGoal Circular => new(FiniteBurnGoalKind.Circular);
    public static FiniteBurnGoal Plane(Orbit? target) => new(FiniteBurnGoalKind.Plane, target);
    public static FiniteBurnGoal Transfer(UniverseTime? arrival) => new(FiniteBurnGoalKind.Transfer, null, arrival);
    public static FiniteBurnGoal Apsis => new(FiniteBurnGoalKind.Apsis);
    public static FiniteBurnGoal Period => new(FiniteBurnGoalKind.Period);
}

// Corrects an impulsive burn for the time the engines need to deliver it, with the stock FiniteBurnRefiner and the phases stock uses in BurnContextMenu.DrawBurnActions and TransferPlanner.
// The burn keeps its impulse time as the burn midpoint, and the delta v stays in the VLF frame at that time, as Burn.Create expects.
// Phases without thrust leave the burn impulsive, which is also what the refiner and FlightPlan.TryAddFiniteBurn do.
internal static class FiniteBurns
{
    // The circular tolerance of OrbitManeuvers.ComputeCircularize.
    private const double CircularEccentricity = 0.001;
    // The iteration limits and the finite difference step of FiniteBurnRefiner.
    private const int MaxIterations = 12;
    private const int MaxHalvings = 8;
    private const double ScaleStep = 1e-4;
    private const double RadiusToleranceM = 0.1;

    // FlightComputer.GetFullThrottlePhases returns a span over a buffer that the flight computer reuses, so read it on the main thread and do not keep it.
    public static ReadOnlySpan<FiniteBurnPhase> FullThrottlePhases(Vehicle vehicle)
        => vehicle.FlightComputer.GetFullThrottlePhases();

    public static bool HasThrust(ReadOnlySpan<FiniteBurnPhase> phases)
        => !phases.IsEmpty && phases[0].Thrust > 0.0;

    // A cached preview depends on the phases, so a cache key adds this value. It changes when the thrust of the first phase or the phase count changes, and the mass does not enter it because the caches key the mass themselves.
    public static int PhaseSignature(Vehicle vehicle)
    {
        ReadOnlySpan<FiniteBurnPhase> phases = FullThrottlePhases(vehicle);
        return HasThrust(phases) ? HashCode.Combine(phases.Length, (long)Math.Round(phases[0].Thrust)) : 0;
    }

    public static (UniverseTime BurnTime, double3 DvVlf) Refine(
        Orbit orbit, UniverseTime burnTime, double3 dvVlf,
        ReadOnlySpan<FiniteBurnPhase> phases, in FiniteBurnGoal goal, UniverseTime? notBefore = null)
    {
        if (!HasThrust(phases) || !IsFinite(dvVlf) || dvVlf.LengthSquared() <= 0.0)
            return (burnTime, dvVlf);

        UniverseTime refinedTime = burnTime;
        double3 refinedDv;
        switch (goal.Kind)
        {
            case FiniteBurnGoalKind.Circular:
                refinedDv = FiniteBurnRefiner.RefineToCircular(orbit, burnTime, dvVlf, phases);
                break;
            case FiniteBurnGoalKind.Plane:
                (refinedTime, refinedDv) = FiniteBurnRefiner.RefineToPlane(
                    orbit, burnTime, dvVlf, phases,
                    goal.PlaneTarget ?? InstantOrbit(orbit, burnTime, dvVlf));
                // RefineToPlane moves the burn earlier to center it on the node. A node closer than that keeps the impulsive burn, so the burn never starts before the current time or the end of the previous burn.
                if (notBefore.HasValue && refinedTime < notBefore.Value)
                    return (burnTime, dvVlf);
                break;
            case FiniteBurnGoalKind.Transfer:
                refinedDv = FiniteBurnRefiner.RefineTransfer(orbit, burnTime, dvVlf, phases, goal.Arrival);
                break;
            case FiniteBurnGoalKind.Apsis:
            case FiniteBurnGoalKind.Period:
                refinedDv = RefineConicSize(orbit, burnTime, dvVlf, phases, goal.Kind);
                break;
            default:
                refinedDv = FiniteBurnRefiner.RefineTransfer(orbit, burnTime, dvVlf, phases, null);
                break;
        }

        // The refiner stops at its best iterate, but a degenerate goal can still give a value that Burn.Create must not receive.
        if (!IsFinite(refinedDv) || !double.IsFinite(refinedTime.Seconds()))
            return (burnTime, dvVlf);
        return (refinedTime, refinedDv);
    }

    public static double3 VlfToCci(Orbit orbit, UniverseTime burnTime, double3 dvVlf)
        => dvVlf.Transform(orbit.GetStateVectorsAt(burnTime).GetVlf2ParentCci().OrIdentity());

    // Appends the burn with FlightPlan.TryAddFiniteBurn, as TransferPlanner builds its transfer preview, so a preview shows the burn arc and the Ignition, Staging and Cutoff markers that stock draws for the burn after Create.
    // Returns the patch that carries the conic after cutoff, or null when the phases have no thrust and nothing was added. The caller then adds an impulsive burn patch.
    public static PatchedConic? TryAddFiniteBurn(
        FlightPlan plan, PatchedConic prePatch, UniverseTime burnTime, double3 dvVlf,
        ReadOnlySpan<FiniteBurnPhase> phases, out bool impacted)
    {
        impacted = false;
        if (prePatch.PrimaryBody == null)
            return null;
        if (!plan.TryAddFiniteBurn(burnTime, dvVlf, prePatch, phases, out impacted))
            return null;
        return LastBurnPiece(plan);
    }

    // A burn that crosses an SOI is split into several pieces, and only the last one ends at cutoff.
    public static PatchedConic? LastBurnPiece(FlightPlan plan)
    {
        for (int i = plan.Patches.Count - 1; i >= 0; i--)
        {
            if (plan.Patches[i].BurnTrajectory.HasValue)
                return plan.Patches[i];
        }
        return null;
    }

    // An apsis burn sets one radius, but the refiner goals match a full state, which a long burn cannot reach from the same burn point.
    // FiniteBurnRefiner.RefineTransfer would match the velocity of the impulsive conic at a cutoff radius that differs from it, which changes the energy and so the apsis it was meant to set.
    // This keeps the burn direction and solves for the delta v magnitude alone, with a Newton iteration on a finite difference slope and step halving, as FiniteBurnRefiner.Refine does.
    private static double3 RefineConicSize(
        Orbit orbit, UniverseTime burnTime, double3 dvVlf,
        ReadOnlySpan<FiniteBurnPhase> phases, FiniteBurnGoalKind kind)
    {
        double mu = orbit.Parent.Mu;
        StateVectors sv = orbit.GetStateVectorsAt(burnTime);
        double3 instantVelocity = sv.VelocityCci + dvVlf.Transform(sv.GetVlf2ParentCci().OrIdentity());
        if (!TryConicSize(mu, sv.PositionCci, instantVelocity, out double instantSma, out double instantEcc))
            return FiniteBurnRefiner.RefineTransfer(orbit, burnTime, dvVlf, phases, null);

        SizeTarget target;
        if (kind == FiniteBurnGoalKind.Period)
        {
            target = new SizeTarget(SizeQuantity.SemiMajorAxis, instantSma);
        }
        else
        {
            if (instantEcc < CircularEccentricity)
                return FiniteBurnRefiner.RefineToCircular(orbit, burnTime, dvVlf, phases);
            double burnRadius = sv.PositionCci.Length();
            double apoapsis = instantSma * (1.0 + instantEcc);
            double periapsis = instantSma * (1.0 - instantEcc);
            target = Math.Abs(apoapsis - burnRadius) >= Math.Abs(periapsis - burnRadius)
                ? new SizeTarget(SizeQuantity.Apoapsis, apoapsis)
                : new SizeTarget(SizeQuantity.Periapsis, periapsis);
        }

        double tolerance = Math.Max(RadiusToleranceM, 1e-12 * target.Value);
        double scale = 1.0;
        double miss = SizeMiss(orbit, burnTime, dvVlf, scale, phases, mu, target);
        for (int i = 0; i < MaxIterations && Math.Abs(miss) > tolerance; i++)
        {
            double slope = (SizeMiss(orbit, burnTime, dvVlf, scale + ScaleStep, phases, mu, target) - miss) / ScaleStep;
            if (!double.IsFinite(slope) || slope == 0.0)
                break;
            double step = -miss / slope;
            bool improved = false;
            for (int j = 0; j < MaxHalvings; j++)
            {
                double trial = SizeMiss(orbit, burnTime, dvVlf, scale + step, phases, mu, target);
                if (Math.Abs(trial) < Math.Abs(miss))
                {
                    scale += step;
                    miss = trial;
                    improved = true;
                    break;
                }
                step *= 0.5;
            }
            if (!improved)
                break;
        }
        return dvVlf * scale;
    }

    private enum SizeQuantity { SemiMajorAxis, Apoapsis, Periapsis }

    private readonly record struct SizeTarget(SizeQuantity Quantity, double Value);

    // Infinity marks a trial that leaves the bound orbit, so the step halving rejects it.
    private static double SizeMiss(
        Orbit orbit, UniverseTime burnTime, double3 dvVlf, double scale,
        ReadOnlySpan<FiniteBurnPhase> phases, double mu, SizeTarget target)
    {
        FiniteBurnResult arc = FiniteBurnPredictor.Predict(orbit, burnTime, dvVlf * scale, phases);
        if (!TryConicSize(mu, arc.PositionCci, arc.VelocityCci, out double sma, out double ecc))
            return double.PositiveInfinity;
        double value = target.Quantity switch
        {
            SizeQuantity.Apoapsis => sma * (1.0 + ecc),
            SizeQuantity.Periapsis => sma * (1.0 - ecc),
            _ => sma,
        };
        return value - target.Value;
    }

    // Semi-major axis from the vis-viva equation and eccentricity from the angular momentum, for a bound conic only.
    private static bool TryConicSize(double mu, double3 positionCci, double3 velocityCci, out double sma, out double ecc)
    {
        sma = double.NaN;
        ecc = double.NaN;
        double r = positionCci.Length();
        if (!(r > 0.0) || !(mu > 0.0))
            return false;
        double inverseSma = 2.0 / r - velocityCci.LengthSquared() / mu;
        if (!(inverseSma > 0.0))
            return false;
        sma = 1.0 / inverseSma;
        double h = double3.Cross(positionCci, velocityCci).Length();
        ecc = Math.Sqrt(Math.Max(0.0, 1.0 - h * h / (mu * sma)));
        return double.IsFinite(sma) && double.IsFinite(ecc);
    }

    private static Orbit InstantOrbit(Orbit orbit, UniverseTime burnTime, double3 dvVlf)
    {
        StateVectors sv = orbit.GetStateVectorsAt(burnTime);
        return Orbit.CreateFromStateCci(orbit.Parent, burnTime, sv.PositionCci,
            sv.VelocityCci + dvVlf.Transform(sv.GetVlf2ParentCci().OrIdentity()), orbit.OrbitLineColor);
    }

    private static bool IsFinite(double3 v)
        => double.IsFinite(v.X) && double.IsFinite(v.Y) && double.IsFinite(v.Z);
}
