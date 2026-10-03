using AdvancedFlightComputer.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

// In every worker frame FlightComputer.ComputeControl runs before Rocket.UpdateRockets, and an engine switched on since the last worker step still carries the committed EngineControllerState of an inactive engine, with IsPropellantAvailable false.
// During an Auto burn that first tick commands the new engine no throttle in FlightComputer.CommandEngineThrottles, and when every active engine is new it drops the burn to Manual and raises AutoBurnStoppedOutOfPropellant, so the frame after a staging has no thrust and the burn depends on the stop being taken back.
// The prefix writes the value Rocket.UpdateRockets writes on its first pass over the engine, from the same mole and core states, into the committed state the worker starts from.
// Vehicle.PrepareWorker runs on the main thread after PartTree.FlushDirtyResourceManagers and before the vehicle solvers are queued, so the write needs no channel to the worker and changes no wakeup.
// WasActive stays false, so Rocket.RocketsNeedUpdate still runs that first pass, which then owns the value.
[HarmonyPatch(typeof(Vehicle), nameof(Vehicle.PrepareWorker), new[] { typeof(SimStep) })]
internal static class NewEngineSeedPatch
{
    static void Prefix(Vehicle __instance) => Seed(__instance);

    // Only during an Auto burn, the one mode in which the stale tick ends what the player set up.
    internal static int Seed(Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (vehicle.IsDisposed || fc.BurnMode != FlightComputerBurnMode.Auto || fc.Burn == null)
            return 0;
        PartTree? tree = vehicle.Parts;
        if (tree?.States == null
            || !ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>
                .TryGetFrom(tree.States, out var engines))
            return 0;

        int seeded = 0;
        bool derived = false;
        foreach (var engine in engines.ModulesAndStates)
        {
            if (!engine.Module.IsActive || engine.State.WasActive)
                continue;
            if (!derived)
            {
                tree.EnsureDerived(DerivedData.SolidMotorStacks);
                derived = true;
            }
            bool fueled = VehiclePropellant.IsFueled(engine.Module, tree.Moles.States, tree.RocketCores.States, out _, out _);
            if (fueled == engine.State.IsPropellantAvailable)
                continue;
            engines.GetModuleAndAllMutableStatesForInitialization(engine.Module).State.IsPropellantAvailable = fueled;
            if (fueled)
                seeded++;
        }

        if (seeded > 0 && DebugConfig.AutoStage)
            DefaultCategory.Log.Debug($"[AFC] AutoStage reports {seeded} newly lit engine(s) on '{vehicle.Id}' as fueled for the first worker tick.");
        return seeded;
    }
}
