using System;
using System.Collections.Generic;
using System.Reflection;
using Brutal.Logging;
using Brutal.Numerics;
using CommunityToolkit.HighPerformance.Buffers;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features;

/// <summary>
/// Enables the Transfer Planner to target objects on hyperbolic orbits
/// (Oumuamua, 2I/Borisov, 3I/ATLAS, etc.).
///
/// The Lambert solver handles any orbit geometry, but surrounding code
/// assumes targets have a finite Period, positive SMA, and meaningful SOI.
/// HyperbolicBodies.xml injects Mass and SOI via KittenExtensions. The
/// Harmony patches here fix everything else: target filtering, time-of-flight
/// estimation, departure alignment, encounter detection, and orbit rendering.
/// </summary>
static class HyperbolicTargets
{
    /// <summary>
    /// Margin from the exact asymptote angle (acos(-1/e)) to avoid
    /// non-finite positions at the mathematical limit.
    /// </summary>
    private const double AsymptoteMargin = 0.999;

    /// <summary>Number of points generated for orbit rendering lines.</summary>
    private const int OrbitPointCount = 2000;

    /// <summary>Min transfer ToF as fraction of Hohmann estimate.</summary>
    private const double MinTofRatio = 0.3;

    /// <summary>Max transfer ToF as fraction of Hohmann estimate.</summary>
    private const double MaxTofRatio = 4.0;

    private static readonly FieldInfo? f_sourceBody =
        AccessTools.Field(typeof(TransferPlanner), "_sourceBody");
    private static readonly FieldInfo? f_transferInfo =
        AccessTools.Field(typeof(TransferPlanner), "_transferInfo");
    private static readonly FieldInfo? f_selectedMinTime =
        AccessTools.Field(typeof(TransferPlanner), "_selectedMinTime");
    private static readonly FieldInfo? f_selectedMaxTime =
        AccessTools.Field(typeof(TransferPlanner), "_selectedMaxTime");
    private static readonly FieldInfo? f_selectedTimeUnit =
        AccessTools.Field(typeof(TransferPlanner), "_selectedTimeUnit");
    private static readonly FieldInfo? f_timeUnits =
        AccessTools.Field(typeof(TransferPlanner), "_timeUnits");
    private static readonly FieldInfo? f_selectedEntry =
        AccessTools.Field(typeof(TransferPlanner), "_selectedEntry");

    /// <summary>
    /// Checks that all reflection targets resolve. Returns false if any are
    /// null (game version changed and renamed a field). Called from Mod.cs
    /// before PatchAll - if this fails, patches are not applied.
    /// </summary>
    public static bool ValidateReflectionTargets()
    {
        var fields = new (string name, FieldInfo? field)[]
        {
            ("_sourceBody",       f_sourceBody),
            ("_transferInfo",     f_transferInfo),
            ("_selectedMinTime",  f_selectedMinTime),
            ("_selectedMaxTime",  f_selectedMaxTime),
            ("_selectedTimeUnit", f_selectedTimeUnit),
            ("_timeUnits",        f_timeUnits),
            ("_selectedEntry",    f_selectedEntry),
        };

        bool allOk = true;
        foreach (var (name, field) in fields)
        {
            if (field == null)
            {
                DefaultCategory.Log.Error(
                    $"[AFC] TransferPlanner.{name} not found - game version may have changed.");
                allOk = false;
            }
        }
        return allOk;
    }

    /// <summary>
    /// Walks up the parent chain to find the star the vehicle orbits.
    /// Returns null if no star is found (shouldn't happen in normal gameplay).
    /// </summary>
    private static StellarBody? GetParentStar(Vehicle source)
    {
        IParentBody? current = source.Parent;
        while (current != null)
        {
            if (current is StellarBody star)
                return star;
            current = (current as Celestial)?.Parent;
        }
        return null;
    }

    #region Patch Group 1: Target list & transfer planning

    /// <summary>
    /// The original GetPlanetList filters out eccentricity >= 1. We let it
    /// run for bound bodies, then append hyperbolic ones it skipped.
    /// </summary>
    [HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.GetPlanetList))]
    static class Patch_GetPlanetList
    {
        static void Postfix(bool getAll, ref List<TransferObject> __result)
        {
            if (getAll) return;

            try
            {
                var sourceBody = (TransferObject)f_sourceBody!.GetValue(null)!;
                if (sourceBody.Body is not Vehicle source) return;

                var star = GetParentStar(source);
                if (star == null) return;

                var existingIds = new HashSet<string>();
                foreach (var entry in __result)
                    if (entry.Body != null)
                        existingIds.Add(entry.Body.Id);

                foreach (Astronomical astro in Universe.CurrentSystem!.All.GetList())
                {
                    if (astro is not Celestial celestial) continue;
                    if (astro is StellarBody) continue;
                    if (celestial.Orbit == null || celestial.Orbit.Eccentricity < 1.0) continue;
                    if (existingIds.Contains(celestial.Id)) continue;

                    if (celestial.Parent != star) continue;

                    if (celestial.SphereOfInfluence <= 0.0 || double.IsNaN(celestial.SphereOfInfluence))
                    {
                        DefaultCategory.Log.Warning(
                            $"[AFC] {celestial.Id} has no SOI, XML patch may be missing");
                        continue;
                    }

                    __result.Add(new TransferObject(celestial));
                }
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning($"[AFC] GetPlanetList postfix: {ex.Message}");
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
    static class Patch_HohmannFlight
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
    /// values with ratios of our Hohmann estimate.
    /// </summary>
    [HarmonyPatch(typeof(TransferPlanner), "SetTransferInfo")]
    static class Patch_SetTransferInfo
    {
        static void Postfix()
        {
            try
            {
                var info = f_transferInfo!.GetValue(null) as OrbitalTransfers.TransferInfo;
                if (info?.Target?.Orbit == null) return;
                if (info.Target.Orbit.Eccentricity < 1.0) return;

                SimTime hohmann = info.HohmannTimeOfFlight;
                if (double.IsNaN(hohmann.Seconds()) || hohmann.Seconds() <= 0.0)
                    return;

                info.MinTransferTimeOfFlight = hohmann * MinTofRatio;
                info.MaxTransferTimeOfFlight = hohmann * MaxTofRatio;

                f_selectedMinTime!.SetValue(null, new SimTime(info.MinTransferTimeOfFlight.Seconds()));
                f_selectedMaxTime!.SetValue(null, new SimTime(info.MaxTransferTimeOfFlight.Seconds()));

                // Pick a time unit where the Hohmann ToF displays readably
                // (whole-number part <= 3 digits, e.g. "183 days" not "4392 hours").
                var timeUnits = f_timeUnits!.GetValue(null) as List<TimeObject>;
                if (timeUnits == null) return;

                foreach (var unit in timeUnits)
                {
                    string formatted = TimeSpanReference
                        .FromSeconds(hohmann.Seconds())
                        .ToNearest(unit.Unit);
                    int wholeDigits = formatted.IndexOfAny(['.', ' ']);
                    if (wholeDigits < 0) wholeDigits = formatted.Length;
                    if (wholeDigits <= 3)
                    {
                        f_selectedTimeUnit!.SetValue(null, unit);
                        break;
                    }
                }
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning($"[AFC] SetTransferInfo postfix: {ex.Message}");
            }
        }
    }

    /// <summary>
    /// AlignmentTime uses synodic period (infinite for hyperbolic targets).
    /// We return periapsis_time - hohmann_tof as the departure window start.
    /// </summary>
    [HarmonyPatch(typeof(OrbitalTransfers), nameof(OrbitalTransfers.AlignmentTime))]
    static class Patch_AlignmentTime
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

    #endregion

    #region Patch Group 2: Encounter detection for hyperbolic targets

    /// <summary>
    /// For hyperbolic targets, replaces TryFindIntercept entirely: builds
    /// the flight plan from the Lambert-optimal dV and computes the actual
    /// patched-conic closest approach distance for honest reporting.
    ///
    /// The game's RefineBurnTask sweeps dV +/-5% and checks each trajectory
    /// for SOI intercepts via patched conic propagation. This fails for
    /// hyperbolic targets because FindClosestApproaches uses the target's
    /// Period (NaN for unbound orbits) as the time limit, causing it to
    /// bail out immediately. Even with a finite time limit, the patched
    /// conic trajectory diverges ~0.3 AU from the Lambert solution at
    /// arrival because the impulsive approximation breaks down at extreme
    /// dV (~26 km/s).
    ///
    /// Instead, we build the flight plan with the Lambert-optimal dV
    /// directly and sweep the trajectory at 1-day intervals to compute
    /// an honest closest-approach distance so users know mid-course
    /// corrections will be needed.
    ///
    /// For bound targets, runs the original dV sweep + encounter detection.
    /// </summary>
    [HarmonyPatch(typeof(RefineBurnTask), nameof(RefineBurnTask.TryFindIntercept))]
    static class Patch_TryFindIntercept
    {
        static bool Prefix(
            OrbitalTransfers.TransferInfo transferInfo,
            ref OrbitalTransfers.PorkChopEntry selectedEntry,
            ref bool __result)
        {
            if (transferInfo.Target.Orbit.Eccentricity < 1.0)
                return true;

            try
            {
                double3 dv = selectedEntry.TransferData.TransferDvVlf;
                FlightPlan flightPlan = FlightPlan.CreateUninitialized(transferInfo.Vehicle.Hash);

                if (OrbitalTransfers.BuildFlightPlan(ref flightPlan, transferInfo,
                        selectedEntry.TransferData.Start, dv,
                        out var closestPoint, out var _))
                {
                    selectedEntry.FlightPlan = flightPlan;
                    selectedEntry.TransferData.Point = closestPoint;

                    double patchedConicDist = FindPatchedConicClosestApproach(
                        flightPlan, transferInfo.Target,
                        selectedEntry.TransferData.Transit.Seconds());
                    selectedEntry.TransferData.ClosestApproachDistance = patchedConicDist;
                    __result = true;

                    if (Mod.DebugMode)
                    {
                        DefaultCategory.Log.Debug(
                            $"[AFC] RefineBurnTask: hyperbolic target, " +
                            $"Lambert dV={dv.Length():F1} m/s, " +
                            $"patched conic miss={patchedConicDist / 1000:F0} km");
                    }
                }
                else
                {
                    __result = false;
                }

                return false;
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning($"[AFC] TryFindIntercept prefix: {ex.Message}");
                return true;
            }
        }

        /// <summary>
        /// Sweeps the heliocentric patch at 1-day intervals to find the
        /// minimum distance between the spacecraft and the target.
        /// </summary>
        private static double FindPatchedConicClosestApproach(
            FlightPlan flightPlan, IOrbiter target, double transitSeconds)
        {
            double minDist = double.MaxValue;
            Orbit targetOrbit = target.Orbit;

            foreach (var patch in flightPlan.Patches)
            {
                if (patch.PrimaryBody?.Hash != targetOrbit.Parent.Hash)
                    continue;

                double start = patch.StartTime.Seconds();
                double end = Math.Min(start + transitSeconds * 2.0, patch.EndTime.Seconds());
                double step = 86400.0; // 1 day

                for (double t = start; t <= end; t += step)
                {
                    var posShip = patch.Orbit.GetStateVectorsAt(new SimTime(t)).PositionCci;
                    var posTarget = targetOrbit.GetStateVectorsAt(new SimTime(t)).PositionCci;
                    double dist = (posTarget - posShip).Length();
                    minDist = Math.Min(minDist, dist);
                }
            }

            return minDist;
        }
    }

    #endregion

    #region Patch Group 3: Orbit rendering

    /// <summary>
    /// Workaround for https://forums.ahwoo.com/threads/781/
    /// Remove once fixed upstream.
    ///
    /// GenerateSpacedPoints creates orbit line points between a start and end
    /// true anomaly. For bound orbits, -Pi..+Pi covers the full ellipse. For
    /// hyperbolic orbits, Pi exceeds the asymptote (acos(-1/e)), producing
    /// non-finite positions that the renderer draws as lines through the origin.
    /// We clamp to the asymptote instead.
    /// </summary>
    [HarmonyPatch(typeof(UpdateTaskUtils), "GenerateSpacedPoints",
        [typeof(PatchedConic), typeof(Orbit)])]
    static class Patch_ClipPointGeneration
    {
        static bool Prefix(PatchedConic? patch, Orbit o,
            ref MemoryOwner<OrbitPointCce> __result)
        {
            if (o.IsBound()) return true;

            // Only intervene when the original would fall back to -Pi..+Pi.
            // Escape burns and encounter patches have valid bounds, leave them.
            bool hasValidBounds = patch != null
                && patch.EndTrueAnomaly.IsNotNaN()
                && patch.EndTransition != PatchTransition.Final;
            if (hasValidBounds) return true;

            try
            {
                var orb2ParentCce = o.GetOrb2ParentCce();
                var points = MemoryOwner<OrbitPointCce>.Allocate(OrbitPointCount);

                double asymptote = Math.Acos(-1.0 / o.Eccentricity) * AsymptoteMargin;
                double startTa = -asymptote;
                double endTa = asymptote;

                if (patch != null
                    && patch.StartTrueAnomaly.IsNotNegativePi()
                    && patch.StartTrueAnomaly.IsNotZero())
                {
                    startTa = patch.StartTrueAnomaly.Value();
                }

                double step = (endTa - startTa) / (OrbitPointCount - 1);
                for (int i = 0; i < OrbitPointCount; i++)
                {
                    var ta = new TrueAnomaly(startTa + i * step);
                    EccentricAnomaly ea = o.GetEccentricAnomaly(ta);
                    if (double.IsFinite(ea.Value()))
                    {
                        SimTime timeFromPe = o.GetTimeFromPeTo(ea);
                        SimTime remaining = o.GetRemainingTimeTo(ea);
                        double3 posCce = o.GetPositionOrb(ea).Transform(orb2ParentCce);
                        points.Span[i] = new OrbitPointCce(posCce, timeFromPe, remaining, ta);
                    }
                    else
                    {
                        // Repeat last valid point to avoid default-zero gaps
                        if (i > 0) points.Span[i] = points.Span[i - 1];
                    }
                }

                __result = points;
                return false;
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning($"[AFC] ClipPointGeneration prefix: {ex.Message}");
                return true;
            }
        }
    }

    #endregion

    #region Diagnostics

    /// <summary>
    /// Logs transfer data when a porkchop entry is selected (debug only).
    /// </summary>
    [HarmonyPatch(typeof(TransferPlanner), "SetSelectedTransfer")]
    static class Patch_DiagnosticLog
    {
        static void Postfix(bool __result)
        {
            if (!__result || !Mod.DebugMode) return;

            try
            {
                var entry = f_selectedEntry!.GetValue(null) as OrbitalTransfers.PorkChopEntry;
                if (entry?.FlightPlan == null) return;

                var data = entry.TransferData;
                var patches = entry.FlightPlan.Patches;
                double dvMag = data.TransferDvVlf.Length();

                DefaultCategory.Log.Info(
                    $"[AFC] Transfer: dV={dvMag:F1} m/s, {patches.Count} patches, " +
                    $"transit={data.Transit.Seconds():F0}s, " +
                    $"closest={data.ClosestApproachDistance:E2}m");

                for (int i = 0; i < patches.Count; i++)
                {
                    var p = patches[i];
                    var o = p.Orbit;
                    double startTaDeg = p.StartTrueAnomaly.Value() * (180.0 / Math.PI);
                    double endTaDeg = p.EndTrueAnomaly.Value() * (180.0 / Math.PI);

                    DefaultCategory.Log.Debug(
                        $"[AFC]   [{i}] e={o.Eccentricity:F4} parent={p.PrimaryBody?.Id ?? "?"} " +
                        $"{p.StartTransition}->{p.EndTransition} " +
                        $"TA={startTaDeg:F1}..{endTaDeg:F1}deg");

                    if (o.Eccentricity >= 1.0)
                    {
                        double asymDeg = Math.Acos(-1.0 / o.Eccentricity) * (180.0 / Math.PI);
                        DefaultCategory.Log.Debug(
                            $"[AFC]       asymptote=+/-{asymDeg:F1}deg " +
                            $"points={(!o.IsMissingPoints() ? "ok" : "MISSING")}");
                    }
                }
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning($"[AFC] DiagnosticLog: {ex.Message}");
            }
        }
    }

    #endregion
}
