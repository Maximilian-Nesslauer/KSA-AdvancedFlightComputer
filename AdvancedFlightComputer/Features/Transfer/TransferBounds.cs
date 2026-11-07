using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.HyperbolicTargets;
using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.Transfer;

/// <summary>
/// The transfer window and the destination list of the TRANSFER page, following stock's planner. The
/// bounds follow the private TransferPlanner.SetTransferInfo: the transfer departs from the source's
/// parent when that parent orbits something, else from the craft itself; the Hohmann time of flight
/// sizes the window from max(Hohmann - target period, Hohmann / 2) to that plus min(2 x target period,
/// Hohmann). A target on an unbound orbit has no period, so its window is the Hohmann estimate times
/// the ratios the unbound-target support uses for stock's planner (HyperbolicTargets). The destination
/// list follows TransferPlanner.PopulateWithTargets for this source, plus the unbound bodies with an SOI.
/// </summary>
internal static class TransferBounds
{
    public static OrbitalTransfers.TransferInfo? Build(Vehicle source, IOrbiter destination)
    {
        try
        {
            IOrbiter? orbiter = source.Parent as IOrbiter;
            var info = new OrbitalTransfers.TransferInfo(orbiter ?? source, destination, source, usePorkChopData: true);
            Orbit origin = orbiter == null ? info.Starting : orbiter.Orbit;
            if (OrbitalTransfers.SameSoiTransfer(info))
                origin = info.Starting;
            info.HohmannTimeOfFlight = OrbitalTransfers.HohmannFlight(origin, destination.Orbit);
            double hohmann = info.HohmannTimeOfFlight.Seconds();
            if (!(hohmann > 0.0))
                return null;

            double min, max;
            if (destination.Orbit.IsBound())
            {
                double period = destination.Orbit.Period;
                min = Math.Max(hohmann - period, hohmann / 2.0);
                max = min + Math.Min(2.0 * period, hohmann);
            }
            else
            {
                min = hohmann * HyperbolicTargets.HyperbolicTargets.MinTofRatio;
                max = hohmann * HyperbolicTargets.HyperbolicTargets.MaxTofRatio;
            }
            if (!double.IsFinite(min) || !double.IsFinite(max) || !(max > min))
                return null;
            info.MinTransferTimeOfFlight = new UniverseTime(min);
            info.MaxTransferTimeOfFlight = new UniverseTime(max);
            return info;
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("transfer-bounds:" + ex.GetType().Name,
                $"[AFC] Transfer window for '{source.Id}' to '{(destination as Astronomical)?.Id ?? "?"}' failed: {ex.Message}");
            return null;
        }
    }

    /// <summary>The destinations stock's Hohmann plan offers this source, plus the unbound bodies with an SOI around the transfer frame.</summary>
    public static void Destinations(Vehicle source, List<TransferObject> list)
    {
        list.Clear();
        foreach (Vehicle vehicle in Program.VehiclesInFrame)
        {
            if (vehicle.Id != source.Id && vehicle.Parent == source.Parent)
                list.Add(new TransferObject(vehicle));
        }

        ReadOnlySpan<Astronomical> all = Universe.CurrentSystem!.All.AsSpan();
        IParentBody? parent = source.Parent;
        if (parent == null)
            return;
        for (int i = 0; i < all.Length; i++)
        {
            Astronomical astronomical = all[i];
            if (astronomical is Vehicle || astronomical is FixedStar || astronomical is not IOrbiter orbiter)
                continue;
            if (astronomical.System.Id != source.System.Id || orbiter.Id == source.Id || orbiter.Id == parent.Id)
                continue;
            if (!InSameNeighbourhood(astronomical, orbiter, parent))
                continue;
            if ((orbiter.Orbit != null && orbiter.Orbit.Eccentricity >= 1.0) || double.IsNaN(orbiter.SphereOfInfluence) || orbiter.SphereOfInfluence <= 0.0)
                continue;
            list.Add(new TransferObject(astronomical));
        }
        HyperbolicTargets.HyperbolicTargets.AddUnboundTargets(source, list);
    }

    // The neighbourhood rule of TransferPlanner.PopulateWithPlanets for a source around parent.
    private static bool InSameNeighbourhood(Astronomical astronomical, IOrbiter orbiter, IParentBody parent)
    {
        Celestial? celestial = parent as Celestial;
        StellarBody? stellarBody = parent as StellarBody;
        if (parent is OrbitingStar orbitingStar)
            return orbiter.Parent == orbitingStar || orbiter.Parent == orbitingStar.Parent;
        if (parent.IsMoon())
            return orbiter.Parent?.Id == celestial?.Parent.Id;
        if (!astronomical.IsMoon())
            return orbiter.Parent?.Id == (celestial != null ? celestial.Parent.Id : stellarBody?.Id);
        return orbiter.Parent?.Id == parent.Id;
    }
}
