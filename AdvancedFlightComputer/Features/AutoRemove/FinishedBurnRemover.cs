using System.Diagnostics;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.AutoRemove;

// Watches the controlled vehicle's burn mode on the shared tick and drops an auto-burn from the
// plan on the tick the flight computer reports it finished. An RCS burn never leaves Manual, so
// its completion arrives through RcsBurnCompletions instead.
internal static class FinishedBurnRemover
{
    private static Vehicle? _sampledVehicle;
    private static FlightComputerBurnMode _sampledMode;

    public static void Reset()
    {
        _sampledVehicle = null;
        _sampledMode = default;
    }

    internal static void Tick()
    {
#if DEBUG
        long perfStart = DebugConfig.Performance ? Stopwatch.GetTimestamp() : 0;
#endif
        Vehicle? vehicle = Program.ControlledVehicle;
        if (vehicle == null || vehicle.IsDisposed)
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
                LogHelper.WarnOnce("autoremove-tick:" + ex.GetType().Name,
                    $"[AFC] AutoRemove tick threw for '{vehicle.Id}': {ex}");
            }
        }
#if DEBUG
        if (DebugConfig.Performance)
            PerfTracker.Record("FinishedBurnRemover.Tick", Stopwatch.GetTimestamp() - perfStart);
#endif
    }

    private static void RemoveFinishedBurn(Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        FlightComputerBurnMode currentMode = fc.BurnMode;
        bool sameVehicle = ReferenceEquals(_sampledVehicle, vehicle);
        FlightComputerBurnMode previousMode = _sampledMode;

        // The sample advances while the feature is switched off, so switching it back on cannot
        // act on a transition it never observed.
        _sampledVehicle = vehicle;
        _sampledMode = currentMode;

        if (!sameVehicle || !AutoRemoveConfig.Enabled)
            return;
        if (previousMode != FlightComputerBurnMode.Auto || currentMode != FlightComputerBurnMode.Manual)
            return;

        BurnTarget? target = fc.Burn;
        if (target == null)
            return;

        // A node with no delta-V cannot finish, yet its DeltaVToGoCci is zero too, so the reversal
        // test would read it as finished. Stock creates such nodes, and adding one ahead of the
        // running burn produces the Auto to Manual transition on its own.
        if (target.DeltaVTargetCci.LengthSquared() <= 0f)
            return;

        // Out of fuel also flips Auto to Manual but leaves DeltaVToGoCci pointing along the target.
        // The reversal is the same check FlightComputer.UpdateBurnTarget drops to Manual on.
        if (float3.Dot(target.DeltaVToGoCci, target.DeltaVTargetCci) > 0f)
            return;

        // Parent departure burns stay in the plan without ever being loaded, so the finished entry
        // is the first executable one, the same burn stock's own removal resolves.
        Burn? finished = fc.BurnPlan.FindFirstExecutableBurn();
        if (finished == null)
            return;

        if (DebugConfig.AutoRemove)
            DefaultCategory.Log.Debug(
                $"[AFC] AutoRemove: '{vehicle.Id}' finished an auto-burn (dvToGo={target.DeltaVToGoCci.Length():F2}m/s, " +
                $"dvTarget={target.DeltaVTargetCci.Length():F2}m/s); removing it from the plan.");

        fc.RemoveBurn(finished);
    }

    // Same policy for an RCS burn: only while enabled, only on the controlled vehicle, and only
    // while the burn is still in the plan. The raiser isolates each subscriber, so no catch here.
    // Burn equality is by time and delta-V, so the plan's own instance is resolved first and the
    // removed object is the disposed one, as on the tick path.
    internal static void OnRcsBurnCompleted(Vehicle vehicle, Burn burn)
    {
        if (!AutoRemoveConfig.Enabled || Program.ControlledVehicle != vehicle)
            return;
        FlightComputer fc = vehicle.FlightComputer;
        int index = fc.BurnPlan.TryGetBurnIndex(burn);
        if (index < 0 || !fc.BurnPlan.TryGetBurn(index, out Burn? planBurn) || planBurn == null)
            return;

        if (DebugConfig.AutoRemove)
            DefaultCategory.Log.Debug(
                $"[AFC] AutoRemove: '{vehicle.Id}' finished an RCS burn (dv={planBurn.DeltaVVlf.Length():F2}m/s); removing it from the plan.");

        fc.RemoveBurn(planBurn);
    }
}
