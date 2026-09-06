using System;
using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// The vehicles the per-frame drivers walk. They hang off <see cref="Universe.ApplyVehicleSolvers"/>,
/// which carries no vehicle argument, so the shared hook enumerates the world.
///
/// This is the live registry rather than <see cref="Program.VehiclesInFrame"/>, because
/// <c>Program.PrepareFrame</c> refreshes that cache only after the solver apply, which makes it one
/// frame stale here, and the headless harness drives the solvers without PrepareFrame, which
/// leaves it never refreshed at all.
/// </summary>
internal static class LoadedVehicles
{
    /// <summary><c>Vehicle.Dispose(bool)</c> sets IsDisposed before it deregisters, so callers test
    /// the flag rather than rely on a disposed vehicle being gone from here.</summary>
    public static ReadOnlySpan<Astronomical> All
    {
        get
        {
            CelestialSystem? system = Universe.CurrentSystem;
            return system == null ? default : system.All.AsSpan();
        }
    }
}
