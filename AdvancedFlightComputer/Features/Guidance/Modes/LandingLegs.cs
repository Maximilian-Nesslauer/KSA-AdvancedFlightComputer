#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Numerics;
using KSA;

// The legs of a landing: a landing that guidance flies deploys every landing leg of the craft once, when the legs come below LegsDeployHeightM over the ground under the craft (LegsRules.Decide).
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
        // The height is read only where it can matter, so a craft that flies no landing pays nothing.
        double legsHeight = flies && holds && !state.LegsFired && state.LegsAutoDeploy ? LegsHeightM(vehicle, state) : double.NaN;
        switch (LegsRules.Decide(state.LegsAutoDeploy, flies, holds, state.LegsFired, vehicle.Situation.HasAnyContact(), legsHeight, state.LegsDeployHeightM))
        {
            case LegsStep.Deploy:
                int legs = KsaLandingLegs.Count(vehicle);
                state.LegsCommanded = KsaLandingLegs.DeployAll(vehicle);
                state.LegsFired = true;
                state.LegsFiredAtM = legsHeight;
                if (legs > 0)
                    GuidanceLog.Info(vehicle, FormattableString.Invariant(
                        $"landing legs: {state.LegsCommanded} of {legs} commanded to deploy at {legsHeight:F1} m over the ground (setting {state.LegsDeployHeightM:F0} m)."));
                break;
            case LegsStep.Rearm:
                state.LegsFired = false;
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
