using System;
using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// The vehicles the per-frame drivers walk. Both of them hang off
/// <see cref="Universe.ApplyVehicleSolvers"/>, which carries no vehicle
/// argument, so each has to enumerate the world itself.
///
/// Deliberately the live registry rather than <see cref="Program.VehiclesInFrame"/>:
/// that cache is filled from this very collection by
/// <c>Program.RefreshVehiclesInFrame</c>, which runs AFTER the solver apply in the
/// game's frame (so it is one frame stale here) and never runs at all under the
/// headless harness, whose SimDriver drives the solvers directly instead of going
/// through <c>Program.PrepareFrame</c>. Reading the registry gives the drivers one
/// rule that holds in both.
/// </summary>
internal static class LoadedVehicles
{
    /// <summary>A disposed vehicle is normally gone from here already, because
    /// <c>Vehicle.Dispose(bool)</c> deregisters it from the system. It sets
    /// IsDisposed first though, so callers still test the flag rather than rely
    /// on the two staying in that order.</summary>
    public static ReadOnlySpan<Astronomical> All
    {
        get
        {
            CelestialSystem? system = Universe.CurrentSystem;
            return system == null ? default : system.All.AsSpan();
        }
    }
}
