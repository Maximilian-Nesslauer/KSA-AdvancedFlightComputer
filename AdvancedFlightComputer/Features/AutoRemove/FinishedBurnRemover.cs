#if DEBUG
using System.Diagnostics;
#endif
using AutoRemoveFinishedBurns.Core;
using Brutal.Logging;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AutoRemoveFinishedBurns.Features;

/// <summary>
/// Watches the controlled vehicle's burn mode and drops an auto-burn from the
/// plan on the tick the flight computer reports it finished.
///
/// The hook is the main-thread solver-apply pass, not the per-vehicle apply:
/// Vehicle.UpdateFromTaskResultsUnsynchronized runs one physics bubble per
/// worker thread, and BurnPlan is an unlocked list, so removing a burn from
/// there would race the UI and the input-event drain. The synchronized
/// per-vehicle half is on the main thread but aggressively inlined into its
/// caller, which a Harmony detour cannot intercept once the JIT inlines it.
/// Universe.ApplyVehicleSolvers is the enclosing pass, runs after every bubble
/// has applied, and still lands before InputEvents.ApplyInputEvents.
/// </summary>
[HarmonyPatch(typeof(Universe), nameof(Universe.ApplyVehicleSolvers), new Type[0])]
static class BurnRemovalPatch
{
    private static Vehicle? _sampledVehicle;
    private static FlightComputerBurnMode _sampledMode;

    public static void Reset()
    {
        _sampledVehicle = null;
        _sampledMode = default;
    }

    static void Postfix()
    {
#if DEBUG
        long perfStart = DebugConfig.Performance ? Stopwatch.GetTimestamp() : 0;
#endif
        Vehicle? vehicle = Program.ControlledVehicle;
        if (vehicle == null)
        {
            Reset();
        }
        else
        {
            try
            {
                RemoveFinishedBurn(vehicle);
            }
            catch (Exception ex)
            {
                LogHelper.ErrorOnce("Postfix:" + ex.GetType().Name,
                    $"[AutoRemoveFinishedBurns] vehicle='{vehicle.Id}' postfix threw: {ex}");
            }
        }
#if DEBUG
        if (DebugConfig.Performance)
            PerfTracker.Record("BurnRemovalPatch.Postfix",
                Stopwatch.GetTimestamp() - perfStart);
#endif
    }

    private static void RemoveFinishedBurn(Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        FlightComputerBurnMode currentMode = fc.BurnMode;
        bool sameVehicle = ReferenceEquals(_sampledVehicle, vehicle);
        FlightComputerBurnMode previousMode = _sampledMode;

        // The sample advances even while the mod is switched off, so turning it
        // back on cannot act on a transition it never observed.
        _sampledVehicle = vehicle;
        _sampledMode = currentMode;

        if (!sameVehicle) return;
        if (!Config.Enabled) return;
        if (previousMode != FlightComputerBurnMode.Auto) return;
        if (currentMode != FlightComputerBurnMode.Manual) return;

        BurnTarget? target = fc.Burn;
        if (target == null) return;

        // A node with no delta-V is not a maneuver and cannot finish, yet its
        // DeltaVToGoCci is zero as well, so the reversal test below would read
        // it as finished. Stock creates such nodes (TargetTrackWindow), and
        // adding one ahead of the running burn produces the Auto -> Manual
        // transition on its own, because FlightComputer.AddBurn unloads the
        // burn it replaces.
        if (target.DeltaVTargetCci.LengthSquared() <= 0f) return;

        // Out-of-fuel also flips Auto -> Manual but leaves DeltaVToGoCci
        // pointing the same way as DeltaVTargetCci (dot > 0). Reversal
        // (dot <= 0) is the discriminator for actual completion. This
        // mirrors stock FlightComputer.UpdateBurnTarget which sets
        // BurnMode = Manual on the same dot-product check.
        if (float3.Dot(target.DeltaVToGoCci, target.DeltaVTargetCci) > 0f) return;

        // Parent departure burns stay in the plan without ever being loaded, so
        // the entry that just finished is the first executable one rather than
        // the first one, the same burn stock's own removal path resolves.
        Burn? finished = fc.BurnPlan.FindFirstExecutableBurn();
        if (finished == null) return;

        if (DebugConfig.Detection)
        {
            float dvToGo = target.DeltaVToGoCci.Length();
            float dvTarget = target.DeltaVTargetCci.Length();
            DefaultCategory.Log.Debug(
                $"[AutoRemoveFinishedBurns] vehicle='{vehicle.Id}' " +
                $"auto-burn finished (dvToGo={dvToGo:F2}m/s, dvTarget={dvTarget:F2}m/s); " +
                "removing from plan.");
        }

        fc.RemoveBurn(finished);
    }
}
