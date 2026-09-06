using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

// Initial and later passes use the same scheduling path.
internal static class MultiPassCommitter
{
    public static string? TryCommitNext(Vehicle vehicle, MultiPassExecution exec)
    {
        var plan = exec.Intent.RecomputePass(
            vehicle, exec.PassIndex, exec.PassCountTotal, exec.Mode);
        if (plan.Pass == null)
            return plan.FailureReason ?? "intent returned no pass";

        PassPreview preview = plan.Pass.Value;
        Burn? burn = QueueAddBurn(vehicle, preview.BurnTime, preview.DvVlf);
        if (burn == null)
            return $"no flight-plan patch at t={preview.BurnTime.Seconds():F0}s";

        exec.AssignCurrentBurn(burn);
        return null;
    }

    // Buffer additions with other input changes. No burn can be created if the flight plan has no patch at the requested time.
    public static Burn? QueueAddBurn(
        Vehicle source, UniverseTime burnTime, double3 dvVlf, FlightPlan? chainPlan = null)
    {
        // A supplied chain plan anchors the maneuver after another pending burn. Normal pass commits use the live plan because the vehicle has already flown the previous pass and its prediction can be stale.
        PatchedConic? patch = chainPlan?.TryFindPatch(burnTime)
                              ?? source.FlightPlan.TryFindPatch(burnTime);
        if (patch == null)
        {
            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPassCommitter.QueueAddBurn: vehicle='{source.Id}' " +
                    $"no patch for t={burnTime.Seconds():F1}s; cannot create burn");
            return null;
        }

        OrbitPointCce point = patch.Orbit.GetPointAt(burnTime);
        Burn burn = Burn.Create(point, burnTime.Seconds(), dvVlf, patch, source);
        burn.IsGizmoActive = false;

        InputEvents.BurnUpdateBuffer.Add(new InputEvents.BurnUpdateData
        {
            Burn = burn,
            FlightComputer = source.FlightComputer,
            AddBurn = true,
        });

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPassCommitter.QueueAddBurn: vehicle='{source.Id}' " +
                $"queued burn t={burnTime.Seconds():F1}s dv={dvVlf.Length():F2}m/s " +
                $"on patch (orbit Pe={patch.Orbit.Periapsis:F0} Ap={patch.Orbit.Apoapsis:F0})");
        return burn;
    }
}
