using System.Runtime.CompilerServices;
using KSA;

namespace AdvancedFlightComputer.Core;

internal enum ControlClaimant
{
    None,
    RcsTranslation,
    Guidance,
}

/// <summary>
/// Tracks one AFC claimant per Vehicle object, so renaming a vehicle does not change its claim.
/// New vehicles start unclaimed, and claims are not saved.
/// Callers must acquire a claim before writing control and release it after cleanup succeeds.
/// Access is restricted to the main thread.
/// </summary>
internal static class VehicleControlOwnership
{
    private static readonly ConditionalWeakTable<Vehicle, Claim> _claims = new();

    private sealed class Claim
    {
        internal ControlClaimant Claimant;

        // Lets ID-keyed features detect a rename while the claim follows the vehicle object.
        internal string? VehicleId;
    }

    internal static ControlClaimant HolderOf(Vehicle vehicle)
        => vehicle != null && _claims.TryGetValue(vehicle, out Claim? claim)
            ? claim.Claimant
            : ControlClaimant.None;

    internal static bool Holds(Vehicle vehicle, ControlClaimant claimant)
        => claimant != ControlClaimant.None && HolderOf(vehicle) == claimant;

    /// <summary>Allows the current holder to claim again, but refuses a different claimant.</summary>
    internal static bool TryClaim(Vehicle vehicle, ControlClaimant claimant, out ControlClaimant holder)
    {
        holder = HolderOf(vehicle);
        if (vehicle == null || claimant == ControlClaimant.None)
            return false;
        if (holder == claimant)
            return true;
        if (holder != ControlClaimant.None)
            return false;

        Claim claim = _claims.GetOrCreateValue(vehicle);
        claim.Claimant = claimant;
        claim.VehicleId = vehicle.Id;
        holder = claimant;
        return true;
    }

    /// <summary>The id the vehicle had when its claim was taken, or null when it is unclaimed.</summary>
    internal static string? ClaimedId(Vehicle vehicle)
        => vehicle != null && _claims.TryGetValue(vehicle, out Claim? claim) ? claim.VehicleId : null;

    /// <summary>Releases only the caller's claim.</summary>
    internal static void Release(Vehicle vehicle, ControlClaimant claimant)
    {
        if (vehicle != null && Holds(vehicle, claimant))
            _claims.Remove(vehicle);
    }

    internal static void ReleaseAll(Vehicle vehicle)
    {
        if (vehicle != null)
            _claims.Remove(vehicle);
    }

    internal static void Clear() => _claims.Clear();

    internal static string Describe(ControlClaimant claimant) => claimant switch
    {
        ControlClaimant.RcsTranslation => "an RCS burn",
        ControlClaimant.Guidance => "guidance",
        _ => "another feature",
    };
}
