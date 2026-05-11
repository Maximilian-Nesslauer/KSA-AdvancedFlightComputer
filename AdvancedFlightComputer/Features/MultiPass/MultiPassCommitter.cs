using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Shared "plan the next pass and put it in the BurnPlan" mechanics
/// for both <see cref="PassCompletionPatch"/> (after a pass completes)
/// and StartMultiPass (the user-driven first pass).
/// </summary>
internal static class MultiPassCommitter
{
    /// <summary>
    /// Plans pass <c>exec.PassIndex</c> via the intent, queues the
    /// burn through <see cref="InputEvents.BurnUpdateBuffer"/>, and
    /// attaches it to <paramref name="exec"/>. Returns null on
    /// success, or a short failure reason.
    /// </summary>
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

    /// <summary>
    /// Builds a Burn at <paramref name="burnTime"/> and queues it via
    /// <see cref="InputEvents.BurnUpdateBuffer"/> (stock pattern: the
    /// BurnPlan mutation applies at the next frame boundary, sequenced
    /// with any other queued deletes / updates). Returns the Burn so
    /// the caller can hold a reference, or null if no patch covers
    /// <paramref name="burnTime"/>.
    /// </summary>
    public static Burn? QueueAddBurn(Vehicle source, SimTime burnTime, double3 dvVlf)
    {
        PatchedConic? patch = source.FlightPlan.TryFindPatch(burnTime);
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
