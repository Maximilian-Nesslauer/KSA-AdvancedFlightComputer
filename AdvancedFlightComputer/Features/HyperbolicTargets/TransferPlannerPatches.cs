using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.HyperbolicTargets;

/// <summary>
/// PopulateWithPlanets filters out eccentricity >= 1. We let the stock logic
/// run, then append hyperbolic bodies into the span.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.PopulateWithPlanets),
    new Type[] { typeof(Span<TransferObject>), typeof(int), typeof(bool) },
    new ArgumentType[] { ArgumentType.Normal, ArgumentType.Ref, ArgumentType.Normal })]
internal static class Patch_PopulateWithPlanets
{
    static void Postfix(Span<TransferObject> list, ref int count, bool getAll)
    {
        if (getAll) return;

        try
        {
            var sourceBody = (TransferObject)GameReflection.TransferPlanner_sourceBody!.GetValue(null)!;
            if (sourceBody.Body is not Vehicle source) return;

            var star = HyperbolicTargets.GetParentStar(source);
            if (star == null) return;

            ReadOnlySpan<Astronomical> all = Universe.CurrentSystem!.All.AsSpan();
            for (int i = 0; i < all.Length; i++)
            {
                if (all[i] is not Celestial celestial) continue;
                if (celestial.Orbit == null || celestial.Orbit.Eccentricity < 1.0) continue;
                if (celestial.Id == source.Id || celestial.Id == source.Parent?.Id) continue;
                if (celestial.Parent != star) continue;

                if (celestial.SphereOfInfluence <= 0.0 || double.IsNaN(celestial.SphereOfInfluence))
                {
                    LogHelper.WarnOnce($"soi-missing-{celestial.Id}",
                        $"[AFC] {celestial.Id} has no SOI, XML patch may be missing");
                    continue;
                }

                if (count >= list.Length) break;
                list[count++] = new TransferObject(celestial);
            }
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] PopulateWithPlanets postfix: {ex.Message}");
        }
    }
}

/// <summary>
/// HohmannFlight uses (Apoapsis + Periapsis) / 2 as transfer SMA, but
/// hyperbolic orbits have negative Apoapsis (from negative SMA), producing
/// NaN via sqrt of a negative number. We substitute the target's current
/// distance from its parent as the "destination radius".
/// </summary>
[HarmonyPatch(typeof(OrbitalTransfers), nameof(OrbitalTransfers.HohmannFlight))]
internal static class Patch_HohmannFlight
{
    static bool Prefix(Orbit origin, Orbit destination, ref SimTime __result)
    {
        if (origin.Eccentricity < 1.0 && destination.Eccentricity < 1.0)
            return true;

        double r1 = origin.Eccentricity < 1.0
            ? origin.SemiMajorAxis
            : origin.Periapsis;

        double r2;
        if (destination.Eccentricity >= 1.0)
        {
            var now = Universe.GetElapsedSimTime();
            double currentDist = destination.GetStateVectorsAt(now).PositionCci.Length();
            r2 = Math.Max(currentDist, destination.Periapsis);
        }
        else
        {
            r2 = destination.Periapsis;
        }

        double transferSma = (r1 + r2) * 0.5;
        if (transferSma <= 0.0)
            transferSma = Math.Max(r1, r2);

        double tof = Math.PI * Math.Sqrt(transferSma * transferSma * transferSma / origin.Mu);
        __result = new SimTime(tof);
        return false;
    }
}

/// <summary>
/// SetTransferInfo derives Min/MaxTransferTimeOfFlight from the target's
/// orbital Period, which is NaN for unbound orbits. We replace the NaN
/// values with ratios of our Hohmann estimate; stock's own time-unit
/// auto-pick already runs against the patched (finite) Hohmann ToF, so
/// no extra unit selection is needed here.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), "SetTransferInfo", new Type[0])]
internal static class Patch_SetTransferInfo
{
    static void Postfix()
    {
        try
        {
            var info = GameReflection.TransferPlanner_transferInfo!.GetValue(null)
                as OrbitalTransfers.TransferInfo;
            if (info?.Target?.Orbit == null) return;
            if (info.Target.Orbit.Eccentricity < 1.0) return;

            SimTime hohmann = info.HohmannTimeOfFlight;
            if (double.IsNaN(hohmann.Seconds()) || hohmann.Seconds() <= 0.0)
                return;

            info.MinTransferTimeOfFlight = hohmann * HyperbolicTargets.MinTofRatio;
            info.MaxTransferTimeOfFlight = hohmann * HyperbolicTargets.MaxTofRatio;

            GameReflection.TransferPlanner_selectedMinTime!
                .SetValue(null, new SimTime(info.MinTransferTimeOfFlight.Seconds()));
            GameReflection.TransferPlanner_selectedMaxTime!
                .SetValue(null, new SimTime(info.MaxTransferTimeOfFlight.Seconds()));
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] SetTransferInfo postfix: {ex.Message}");
        }
    }
}

/// <summary>
/// AlignmentTime uses synodic period (infinite for hyperbolic targets).
/// For a hyperbolic flyby the cheapest intercept is near the target's
/// periapsis (closest to the Sun, slowest, longest dwell in the inner
/// system), so we depart roughly hohmann_tof before that.
/// </summary>
[HarmonyPatch(typeof(OrbitalTransfers), nameof(OrbitalTransfers.AlignmentTime))]
internal static class Patch_AlignmentTime
{
    static bool Prefix(OrbitalTransfers.TransferInfo transferInfo,
                       SimTime startTime,
                       ref SimTime __result)
    {
        if (transferInfo.Target?.Orbit == null || transferInfo.Target.Orbit.Eccentricity < 1.0)
            return true;

        SimTime tPeri = transferInfo.Target.Orbit.TimeAtPeriapsis;
        SimTime hohmannToF = transferInfo.HohmannTimeOfFlight;
        SimTime ideal = new SimTime(tPeri.Seconds() - hohmannToF.Seconds());

        __result = SimTime.Max(ideal, startTime);
        return false;
    }
}
