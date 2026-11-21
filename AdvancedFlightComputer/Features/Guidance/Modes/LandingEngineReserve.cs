using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

// The landing group's part of the booster reserve. The reserve formula stays as it is, and a selected group only changes the thrust and exhaust velocity it is given.
public static partial class GuidanceWindow
{
    /// <summary>The sea-level exhaust velocity of the selected landing group, with its full thrust in <paramref name="thrustN"/>. NaN when the group does not resolve on the booster, cannot draw from the booster's reserve, or cannot throttle as low as the booster.</summary>
    private static double LandingGroupSeaLevelVe(Vehicle vehicle, ReturnableStage booster, Part root, HashSet<Part> leaving,
                                                 LandingEngineGroup group, double seaLevelPressure, out double thrustN)
        => LandingGroupSeaLevelVe(vehicle, booster, root, leaving, group, seaLevelPressure, out thrustN, out _);

    private static double LandingGroupSeaLevelVe(Vehicle vehicle, ReturnableStage booster, Part root, HashSet<Part> leaving,
                                                 LandingEngineGroup group, double seaLevelPressure, out double thrustN, out string problem)
    {
        thrustN = double.NaN;
        problem = "";
        if (!ResolveBoosterLandingEngines(vehicle, booster, root, leaving, group, out EngineController[] engines))
        {
            problem = engines.Length > 0 && !ThrottlesAsLowAsBooster(engines, leaving)
                ? "CANNOT THROTTLE AS LOW AS THE OTHER ENGINES OF THE BOOSTER"
                : "HAS A MISSING, UNSUPPLIED OR UNSUPPORTED ENGINE";
            return double.NaN;
        }
        float3 force = float3.Zero;
        double flow = 0.0;
        foreach (EngineController engine in engines)
        {
            RocketControllerData perf = RocketControllerData.ComputeFromRockets(engine.Rockets, (float)seaLevelPressure, 1f);
            force += perf.ThrustMax;
            flow += perf.MassFlowRateMax;
        }
        thrustN = force.Length();
        return flow > 0.0 ? thrustN / flow : double.NaN;
    }

    private static bool ResolveBoosterLandingEngines(Vehicle vehicle, ReturnableStage booster, Part root, HashSet<Part> leaving,
                                                      LandingEngineGroup group, out EngineController[] engines)
    {
        engines = [];
        return root != null && leaving != null
            && _s.ReturnableStages.Count(stage => stage.EngineGroupScope == booster.EngineGroupScope) == 1
            && LandingEngineGroups.Resolve(group, LandingEngineGroups.Controllers(vehicle, root, leaving), out engines)
            && engines.All(engine => engine.Rockets.Length > 0 && engine.Rockets.All(KsaEnginePerf.FollowsThrottle))
            && !SplitsLandingGimbal(vehicle, engines)
            && ThrottlesAsLowAsBooster(engines, leaving)
            && LandingEngineGroups.HasRetainedFeeds(vehicle, engines, leaving);
    }

    /// <summary>
    /// True when no engine of the group has a higher minimum throttle than the lowest of the booster's engines.
    /// After separation the landing models take that lowest value as the booster's floor (PartTree.EngineThrottleMin counts every installed controller), so the same refusal the glide makes is made here, before launch.
    /// </summary>
    private static bool ThrottlesAsLowAsBooster(EngineController[] engines, HashSet<Part> leaving)
    {
        float floor = 1f;
        foreach (Part part in leaving)
            foreach (EngineController engine in part.SubtreeModules.Get<EngineController>())
                floor = Math.Min(floor, engine.MinimumThrottle);
        return engines.All(engine => engine.MinimumThrottle <= floor + 1e-4f);
    }

    /// <summary>
    /// True when every returning booster on the row of <paramref name="stage"/> has the same dry mass, propellant, launch flow and landing engines (thrust and exhaust velocity at <paramref name="seaLevelPressure"/>), because the row arms one reserve in flight (RefreshReserve takes the first booster's) and gives each booster the same share.
    /// </summary>
    private static bool SharesACommonBoosterReserve(Vehicle vehicle, ReturnableStage stage, HashSet<Part> reference,
                                                    double seaLevelPressure, double landingThrustN, double landingVe)
    {
        var expected = BoosterShare(vehicle, reference, stage.SequenceNumber);
        foreach (ReturnableStage other in _s.ReturnableStages)
        {
            if (other == stage || other.SequenceIndex != stage.SequenceIndex)
                continue;
            Part root = FindPart(vehicle, other.RootId);
            if (root == null)
                return false;
            HashSet<Part> leaving = PartsLeavingWith(vehicle, root, other.SequenceIndex);
            var share = BoosterShare(vehicle, leaving, stage.SequenceNumber);
            if (!Close(expected.DryKg, share.DryKg) || !Close(expected.PropellantKg, share.PropellantKg)
                || !Close(expected.LaunchFlow, share.LaunchFlow))
                return false;
            if (!other.ComesHome)
                continue;
            LandingEngineGroup? group = other.EngineGroupScope.Length > 0 ? _s.GetEngineGroups(other.EngineGroupScope).Chosen : null;
            double thrust;
            double ve = group != null
                ? LandingGroupSeaLevelVe(vehicle, other, root, leaving, group, seaLevelPressure, out thrust)
                : BoosterSeaLevelVe(vehicle, leaving, seaLevelPressure, out thrust);
            if (!Close(landingThrustN, thrust) || !Close(landingVe, ve))
                return false;
        }
        return true;

        static bool Close(double a, double b) => Math.Abs(a - b) <= 0.005 * Math.Max(a, b);
    }

    private static (double DryKg, double PropellantKg, double LaunchFlow) BoosterShare(
        Vehicle vehicle, HashSet<Part> parts, int separationSequence)
    {
        ReadOnlySpan<MoleState> moles = vehicle.Parts.Moles.States;
        double dry = 0.0, propellant = 0.0, flow = 0.0;
        foreach (Part part in parts)
        {
            foreach (InertMass mass in part.SubtreeModules.Get<InertMass>())
                dry += mass.MassPropertiesAsmb.Props.Mass;
            foreach (Tank tank in part.SubtreeModules.Get<Tank>())
            {
                double kg = tank.ComputeSubstanceMass(moles);
                if (tank.RoleAffinity == ConsumerRole.Engine)
                    propellant += kg;
                else
                    dry += kg;
            }
            foreach (EngineController engine in part.SubtreeModules.Get<EngineController>())
                if (engine.Sequence > 0 && engine.Sequence < separationSequence)
                    flow += engine.VacuumData.MassFlowRateMax;
        }
        return (dry, propellant, flow);
    }

    /// <summary>Why the booster landing groups no longer match the plan, or an empty string when they all still resolve.</summary>
    private static string LandingGroupsChangedSincePlan(Vehicle vehicle)
    {
        foreach (ReturnableStage booster in _s.ReturnableStages)
        {
            if (!booster.ComesHome || booster.EngineGroupScope.Length == 0
                || _s.GetEngineGroups(booster.EngineGroupScope).Chosen is not { } group)
                continue;
            Part root = FindPart(vehicle, booster.RootId);
            if (root == null || !ResolveBoosterLandingEngines(vehicle, booster, root,
                    PartsLeavingWith(vehicle, root, booster.SequenceIndex), group, out _))
                return $"'{booster.Name}': THE LANDING GROUP OR ITS FUEL SUPPLY CHANGED SINCE PLANNING.";
        }
        return "";
    }
}
