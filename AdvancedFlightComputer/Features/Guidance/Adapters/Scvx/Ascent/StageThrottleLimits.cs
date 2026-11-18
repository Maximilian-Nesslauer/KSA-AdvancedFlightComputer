using System.Reflection;
using System.Reflection.Emit;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>Reads the rocket cores contributing to each stock drain phase while building a convex ascent model.</summary>
[HarmonyPatch]
internal static class StageThrottleLimits
{
    internal readonly record struct Phase(double Minimum, bool FollowsThrottle, bool HasSolids);

    [ThreadStatic]
    private static Dictionary<List<SequencePhaseInfo>, List<Phase>>? _capture;

    internal static UpfgVehicle BuildForConvex(Vehicle vehicle, float pressure, double[]? grid,
                                              Dictionary<UpfgStage, HashSet<Part>> stageParts,
                                              FairingDrainSnapshot? fairing = null)
    {
        var previous = _capture;
        _capture = new();
        try
        {
            vehicle.Parts.PerformanceSequences.RecomputeForFlight(pressure);
            return KsaVehicleAdapter.BuildWithFairing(vehicle, pressure, grid, stageParts, fairing);
        }
        finally
        {
            _capture = previous;
        }
    }

    internal static Phase? Read(List<SequencePhaseInfo> phases, int index)
        => _capture != null && _capture.TryGetValue(phases, out var limits) && index < limits.Count
            ? limits[index] : null;

    internal static void ApplyPatch(Harmony harmony)
    {
        if (GameReflection.SequencePerformanceList_RunDrainSimulation == null
            || GameReflection.SequencePerformanceList_activeCores == null
            || GameReflection.SequencePerformanceList_activeCoreCount == null)
        {
            DefaultCategory.Log.Warning("[AFC] Convex ascent uses part-level throttle limits because stock drain metadata is unavailable.");
            return;
        }
        harmony.CreateClassProcessor(typeof(StageThrottleLimits)).Patch();
    }

    private static MethodBase? TargetMethod() => GameReflection.SequencePerformanceList_RunDrainSimulation;

    private static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
    {
        var codes = instructions.ToList();
        MethodInfo add = AccessTools.DeclaredMethod(typeof(List<SequencePhaseInfo>), nameof(List<SequencePhaseInfo>.Add));
        int index = codes.FindIndex(code => code.Calls(add));
        if (index < 0 || codes.FindLastIndex(code => code.Calls(add)) != index)
        {
            DefaultCategory.Log.Warning("[AFC] Convex ascent uses part-level throttle limits because the stock drain phase anchor changed.");
            return codes;
        }

        // The drain loop has just published a phase and each retained core still has its phase activity flag.
        codes.InsertRange(index + 1,
        [
            new CodeInstruction(OpCodes.Ldarg_0),
            new CodeInstruction(OpCodes.Ldarg_2),
            new CodeInstruction(OpCodes.Call, AccessTools.DeclaredMethod(typeof(StageThrottleLimits), nameof(RecordPhase))),
        ]);
        return codes;
    }

    private static void RecordPhase(SequencePerformanceList performance, List<SequencePhaseInfo> phases)
    {
        if (_capture == null)
            return;
        var cores = (List<CoreDrainState>)GameReflection.SequencePerformanceList_activeCores!.GetValue(performance)!;
        int count = (int)GameReflection.SequencePerformanceList_activeCoreCount!.GetValue(performance)!;
        double floor = 0.0;
        bool followsThrottle = true;
        bool hasSolids = false;
        for (int i = 0; i < count; i++)
        {
            CoreDrainState state = cores[i];
            if (!state.Active || state.Core.Rocket is not { } rocket)
                continue;
            if (rocket is SolidRocket)
            {
                hasSolids = true;
                continue;
            }
            floor = Math.Max(floor, rocket.MinimumThrottle);
            followsThrottle &= KsaEnginePerf.FollowsThrottle(rocket);
        }
        if (phases.Count == 1 || !_capture.TryGetValue(phases, out var limits))
            _capture[phases] = limits = new();
        limits.Add(new Phase(floor, followsThrottle, hasSolids));
    }
}
