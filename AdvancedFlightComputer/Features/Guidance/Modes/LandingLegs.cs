#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;
using Brutal.Numerics;
using KSA;

// The legs of a landing: a landing that guidance flies deploys every landing leg of the craft once, when its time to the touchdown comes down to the deploy time of the legs plus LegsDeployBufferS (LegsRules.Decide). The time to the touchdown is the one of the plan that flies the landing (LegsToGo).
// The plans need nothing from this: the vehicle height every landing plans with is measured with the legs deployed (KsaVehicleHeight), so the plan end and the contact point stay where they are when the legs come down.
public static partial class GuidanceWindow
{
    /// <summary>
    /// One step of the leg deploy for <paramref name="vehicle"/>, from the Vehicle.PrepareWorker prefix after its guidance step, so the goal time written here reaches the worker step that animates the legs (KeyframeAnimationModule.UpdateModules). It writes only while guidance holds the claim on the craft for a landing. A fault costs the deploy and is reported once, never the guidance step.
    /// </summary>
    internal static void StepLandingLegs(Vehicle vehicle)
    {
        if (vehicle == null || vehicle.IsDisposed || !VehicleAutopilotState.TryGet(vehicle, out VehicleAutopilotState state))
            return;
        try
        {
            StepLandingLegs(vehicle, state);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"guidance-legs-{vehicle.Id}:{ex.GetType().Name}",
                $"[AFC] The landing leg deploy failed on '{vehicle.Id}': {ex}");
        }
    }

    internal static void StepLandingLegs(Vehicle vehicle, VehicleAutopilotState state)
    {
        bool flies = LegsLandingFlies(state);
        bool holds = VehicleControlOwnership.Holds(vehicle, ControlClaimant.Guidance);
        // The clocks are read only where they can matter, so a craft that flies no landing pays nothing.
        double toGo = double.NaN, height = double.NaN, legsS = double.NaN, deployAt = double.NaN;
        LegsClock clock = LegsClock.None;
        if (flies && holds && !state.LegsFired && state.LegsAutoDeploy)
        {
            height = LegsHeightM(vehicle, state);
            toGo = LegsToGo(vehicle, state, height, Universe.GetElapsedSeconds(), out clock);
            legsS = KsaLandingLegs.DeploySeconds(vehicle);
            deployAt = LegsRules.DeployAtS(legsS, state.LegsDeployBufferS);
        }
        switch (LegsRules.Decide(state.LegsAutoDeploy, flies, holds, state.LegsFired, vehicle.Situation.HasAnyContact(), toGo, deployAt))
        {
            case LegsStep.Deploy:
                int legs = KsaLandingLegs.Count(vehicle);
                state.LegsCommanded = KsaLandingLegs.DeployAll(vehicle);
                state.LegsFired = true;
                state.LegsFiredToGoS = toGo;
                state.LegsFiredAtM = height;
                if (legs > 0)
                    GuidanceLog.Info(vehicle, FormattableString.Invariant(
                        $"landing legs: {state.LegsCommanded} of {legs} commanded to deploy at {toGo:F1} s to go on the {LegsRules.ClockName(clock)}, {height:F1} m over the ground (deploy time {legsS:F1} s, buffer {state.LegsDeployBufferS:F1} s)."));
                break;
            case LegsStep.Rearm:
                state.LegsFired = false;
                state.LegsFiredToGoS = double.NaN;
                state.LegsFiredAtM = double.NaN;
                state.LegsCommanded = 0;
                break;
        }
    }

    /// <summary>A landing flies the craft: the landing machine from the deorbit to the hover, touchdown guidance and its standby, entry guidance, or a boostback.</summary>
    internal static bool LegsLandingFlies(VehicleAutopilotState s)
        => s.Active || s.EngagePending || s.SixDofStandby
            || (s.LandingPhase != LandingPhase.Idle && s.LandingPhase != LandingPhase.Done)
            || s.ThreeDofPhase is ThreeDofPhase.Glide or ThreeDofPhase.Burn
            || (s.BoostbackPhase != BoostbackPhase.Idle && s.BoostbackPhase != BoostbackPhase.Done);

    /// <summary>
    /// The time from <paramref name="now"/> to the touchdown of the landing that flies <paramref name="vehicle"/>, s, and through <paramref name="clock"/> the clock it was read from (LegsRules.ToGo):
    ///  - touchdown guidance: the end of its published plan (Ksa6DofPlan.SolveTime plus Sigma), where the legs meet the ground;
    ///  - entry guidance in its glide or burn: the end of its plan (Plan3Dof.EndTime), at rest on the site;
    ///  - the G-FOLD descent: its planned arrival (GfoldArrivalTime), on the ground, or at the gate of the vertical approach, which comes before the touchdown. While a new search runs the arrival is a placeholder, so it is not read;
    ///  - touchdown guidance on standby: the end of the plan it would take over with;
    ///  - a boostback glide: the predicted impact of its last update (GlideTgo), or the fall time when that is shorter;
    ///  - without any of these: the fall time of the legs to the ground (LegsRules.FallSeconds), which is never longer than the height over the descent rate.
    /// A plan whose end is behind now gives no time, so the next clock answers.
    /// </summary>
    internal static double LegsToGo(Vehicle vehicle, VehicleAutopilotState s, double legsHeightM, double now, out LegsClock clock)
    {
        LegsClocks clocks = LegsClocksOf(s, s.Guidance?.Published, LegsFallSeconds(vehicle, legsHeightM), now);
        return LegsRules.ToGo(in clocks, out clock);
    }

    /// <summary>The time to go that each clock of LegsToGo gives at <paramref name="now"/>, from the modes of <paramref name="s"/>, the published plan <paramref name="touchdown"/> of touchdown guidance and the fall time <paramref name="fall"/>.</summary>
    internal static LegsClocks LegsClocksOf(VehicleAutopilotState s, Ksa6DofPlan touchdown, double fall, double now)
    {
        Plan3Dof entry = s.ThreeDofPhase is ThreeDofPhase.Glide or ThreeDofPhase.Burn ? s.ThreeDofPlan : null;
        // The glide holds its time to impact at GlideMinTgo or more, so near the ground the fall time, which is the shorter one there, is what reaches the deploy time.
        double impact = s.BoostbackPhase == BoostbackPhase.Glide ? LegsRules.PlanToGo(s.GlideLastUpdate + s.GlideTgo, now) : double.NaN;
        if (double.IsFinite(impact) && double.IsFinite(fall))
            impact = Math.Min(impact, fall);
        return new LegsClocks(
            TouchdownPlanS: s.Active && touchdown != null ? LegsRules.PlanToGo(touchdown.SolveTime + touchdown.Sigma, now) : double.NaN,
            EntryPlanS: entry != null ? LegsRules.PlanToGo(entry.EndTime, now) : double.NaN,
            GfoldPlanS: s.LandingPhase == LandingPhase.GfoldDescent && s.GfoldPlan != null && !s.GfoldForceSearch
                ? LegsRules.PlanToGo(s.GfoldArrivalTime, now) : double.NaN,
            StandbyPlanS: s.SixDofStandby && touchdown != null ? LegsRules.PlanToGo(touchdown.SolveTime + touchdown.Sigma, now) : double.NaN,
            ImpactS: impact,
            FallS: fall);
    }

    /// <summary>The fall time of the legs from <paramref name="legsHeightM"/> to the ground at the descent rate of <paramref name="vehicle"/> and the gravity of its body, s (LegsRules.FallSeconds).</summary>
    private static double LegsFallSeconds(Vehicle vehicle, double legsHeightM)
    {
        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return double.NaN;
        double3 r = orbit.StateVectors.PositionCci;
        double radius = r.Length();
        if (!(radius > 0.0))
            return double.NaN;
        // The radial part of the inertial velocity is the radial part of the velocity over the turning ground too.
        double sink = -double3.Dot(orbit.StateVectors.VelocityCci, r / radius);
        return LegsRules.FallSeconds(legsHeightM, sink, parent.Mu / (radius * radius));
    }

    /// <summary>The height of the legs over the terrain under the craft, the height of the head and of TerminalHeight, with the vehicle height of <paramref name="s"/>.</summary>
    internal static double LegsHeightM(Vehicle vehicle, VehicleAutopilotState s)
    {
        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return double.NaN;
        double3 r = orbit.StateVectors.PositionCci;
        double3 dirCcf = double3.Normalize(r).Transform(parent.GetCci2Ccf());
        double terrain = (parent as Celestial)?.GetTerrainHeightFromDirCcf(dirCcf) ?? 0.0;
        if (!double.IsFinite(terrain))
            terrain = 0.0;
        return r.Length() - (parent.MeanRadius + terrain) - s.VehicleHeightM;
    }
}
