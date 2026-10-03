using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.HyperbolicTargets;

/// <summary>
/// Adds unbound bodies that stock excludes from the transfer target list. A body still needs a
/// positive, non-NaN sphere of influence. Stock drops <c>Eccentricity &gt;= 1.0</c>, while the
/// handling patches use <see cref="Orbit.IsBound"/> so they also cover the narrow parabolic band
/// whose Period is NaN. Only bodies orbiting the transfer's own frame are listed, see
/// <see cref="HyperbolicTargets.TransferFrame"/>, so from a moon orbit a heliocentric comet stays
/// out, because a plan around the planet could not reach it.
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
            if (StockPlanner.SourceVehicle is not Vehicle source) return;
            if (HyperbolicTargets.TransferFrame(source) is not IParentBody frame) return;

            ReadOnlySpan<Astronomical> all = Universe.CurrentSystem!.All.AsSpan();
            for (int i = 0; i < all.Length; i++)
            {
                Astronomical body = all[i];
                if (body is Vehicle || body is not IOrbiter orbiter) continue;
                if (orbiter.Orbit == null || orbiter.Orbit.Eccentricity < 1.0) continue;
                if (orbiter.Parent != frame) continue;
                if (orbiter.Id == source.Id || orbiter.Id == source.Parent?.Id) continue;

                if (orbiter.SphereOfInfluence <= 0.0 || double.IsNaN(orbiter.SphereOfInfluence))
                {
                    LogHelper.WarnOnce($"soi-missing-{orbiter.Id}",
                        $"[AFC] {orbiter.Id} has no SOI, XML patch may be missing");
                    continue;
                }

                if (count >= list.Length) break;
                list[count++] = new TransferObject(body);
            }
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("populate-with-planets:" + ex.GetType().Name,
                $"[AFC] PopulateWithPlanets postfix: {ex}");
        }
    }
}

/// <summary>
/// Stock's <c>HohmannFlight</c> takes (Apoapsis + Periapsis) / 2 as each end's
/// radius, and Apoapsis is NaN on an unbound orbit. An unbound end uses its
/// periapsis instead, which is finite and does not move with time, so the
/// porkchop search gets a stable baseline. A bound end keeps the semi major axis
/// stock would use, and the gravitational parameter is the origin parent's, as in
/// stock, because <see cref="Orbit.Mu"/> can carry a central-mass override.
/// </summary>
[HarmonyPatch(typeof(OrbitalTransfers), nameof(OrbitalTransfers.HohmannFlight),
    new Type[] { typeof(Orbit), typeof(Orbit) })]
internal static class Patch_HohmannFlight
{
    static bool Prefix(Orbit origin, Orbit destination, ref UniverseTime __result)
    {
        if (origin.IsBound() && destination.IsBound())
            return true;

        double r1 = origin.IsBound() ? origin.SemiMajorAxis : origin.Periapsis;
        double r2 = destination.IsBound() ? destination.SemiMajorAxis : destination.Periapsis;
        double transferSma = (r1 + r2) * 0.5;
        double mu = origin.Parent.Mu;
        double tof = Math.PI * Math.Sqrt(transferSma * transferSma * transferSma / mu);

        // UniverseTime throws on NaN, and running the original instead would build one from the NaN apoapsis.
        // Zero is what every consumer of the estimate already tests for.
        // This runs on the porkchop worker as well as the draw thread, so the dedup set is the only shared state touched here.
        if (!double.IsFinite(tof))
        {
            LogHelper.WarnOnce(
                $"hohmann-tof-degenerate-{origin.Parent?.Id ?? "?"}",
                $"[AFC] HohmannFlight: degenerate geometry (r1={r1:E3}m r2={r2:E3}m " +
                $"mu={mu:E3}); reporting no transfer estimate.");
            __result = UniverseTime.Zero;
            return false;
        }
        __result = new UniverseTime(tof);
        return false;
    }
}

/// <summary>
/// Replaces the transfer window stock derives from the target's Period, which
/// is NaN on an unbound orbit, with ratios of the Hohmann estimate.
///
/// A finalizer rather than a postfix, because <c>UniverseTime.NanosecondsFromSeconds</c>
/// throws on NaN and <c>TransferPlanner.SetTransferInfo</c> subtracts the NaN Period
/// from a UniverseTime, so the original never completes for these targets and a
/// postfix would never run. By the time it throws, stock has already stored the new
/// TransferInfo with the patched Hohmann estimate, which is everything the window
/// needs. Only the lines after the throw are replayed here, namely the window, the
/// two selected time fields and the time unit pick.
///
/// <c>SetTransferInfo</c> is called from <c>TransferPlanner.DrawPlanWindow</c>
/// only, so this runs on the draw thread and may raise the player alert.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), "SetTransferInfo", new Type[0])]
internal static class Patch_SetTransferInfo
{
    private static readonly HashSet<string> _alertedTargets = new();

    public static void Reset() => _alertedTargets.Clear();

    static Exception? Finalizer(Exception? __exception)
    {
        try
        {
            OrbitalTransfers.TransferInfo? info = StockPlanner.TransferInfo;
            if (info?.Target is not IOrbiter target || target.Orbit == null || target.Orbit.IsBound())
                return __exception;
            if (__exception != null && __exception is not ArgumentException)
                return __exception;

            double hohmannSec = info.HohmannTimeOfFlight.Seconds();
            if (!(hohmannSec > 0.0))
            {
                // No estimate to size a window from.
                // A null TransferInfo is stock's own "nothing selected" state, so the window degrades instead of throwing out of the draw every frame.
                if (GameReflection.TransferPlanner_transferInfoRef is { } infoRef)
                    infoRef() = null;
                LogHelper.WarnOnce(
                    $"transfer-window-degenerate-{target.Id}",
                    "[AFC] SetTransferInfo: no Hohmann estimate for the unbound target; " +
                    "transfer cleared.");
                return null;
            }

            info.MinTransferTimeOfFlight = new UniverseTime(hohmannSec * HyperbolicTargets.MinTofRatio);
            info.MaxTransferTimeOfFlight = new UniverseTime(hohmannSec * HyperbolicTargets.MaxTofRatio);
            GameReflection.TransferPlanner_selectedMinTimeRef!() = info.MinTransferTimeOfFlight;
            GameReflection.TransferPlanner_selectedMaxTimeRef!() = info.MaxTransferTimeOfFlight;
            SelectTimeUnit(hohmannSec);

            MaybeAlertDepartureWindowPast(target, info.HohmannTimeOfFlight);
            return null;
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("set-transfer-info:" + ex.GetType().Name,
                $"[AFC] SetTransferInfo finalizer for target " +
                $"'{(StockPlanner.TransferInfo?.Target as Astronomical)?.Id ?? "?"}': {ex}");
            return __exception;
        }
    }

    /// <summary>Stock's own unit pick, replayed because it sits after the line
    /// that throws. It takes the first unit whose formatted estimate has at most
    /// three integer digits.</summary>
    private static void SelectTimeUnit(double hohmannSec)
    {
        if (StockPlanner.TimeUnits is not { } units)
            return;
        foreach (TimeObject unit in units)
        {
            ReadOnlySpan<char> text = TimeSpanReference.AllocateNearestString(unit.Unit, hohmannSec);
            int end = text.IndexOfAny(' ', '.');
            int integerDigits = end < 0 ? text.Length : end;
            if (integerDigits <= 3)
            {
                StockPlanner.SelectedTimeUnit = unit;
                return;
            }
        }
    }

    /// <summary>Says once per target that the ideal departure, one transfer time
    /// before the target's periapsis, is already behind the sim time, which is
    /// why the porkchop window looks degenerate. <see cref="Patch_AlignmentTime"/>
    /// evaluates the same condition but also runs on the porkchop worker, and
    /// <c>TimedAlert.Create</c> mutates an unsynchronized list the draw thread walks.</summary>
    private static void MaybeAlertDepartureWindowPast(IOrbiter target, UniverseTime hohmannToF)
    {
        UniverseTime ideal = target.Orbit.TimeAtPeriapsis - hohmannToF;
        if (!(ideal < Universe.GetElapsedTime())) return;

        string targetId = target.Id;
        if (!_alertedTargets.Add(targetId)) return;

        // An unbound target has one periapsis passage, and TransferTask.Run sweeps from the current sim time whatever AlignmentTime returned, so this does not promise a later window.
        TimedAlert.Create(
            $"{targetId}: the ideal departure, one transfer time before its periapsis, " +
            "has already passed. The porkchop window starts from the current time.",
            Color.Yellow, 6.0);
    }
}

/// <summary>
/// Stock's <c>AlignmentTime</c> works from the synodic period, which is infinite
/// against an unbound target. The cheapest intercept of a comet is near its
/// periapsis, so the departure is placed one Hohmann time before that.
///
/// Runs on two threads. <c>TransferTask.Run</c> calls it from the ThreadPool
/// whenever the transfer source is not the vehicle, and the plan window's
/// "Show Parent/Target Alignment" block calls it every frame on the draw thread.
/// Nothing here may touch state that assumes one thread, which is why the player
/// alert lives in <see cref="Patch_SetTransferInfo"/>.
/// </summary>
[HarmonyPatch(typeof(OrbitalTransfers), nameof(OrbitalTransfers.AlignmentTime),
    new Type[] { typeof(OrbitalTransfers.TransferInfo), typeof(UniverseTime), typeof(double) })]
internal static class Patch_AlignmentTime
{
    static bool Prefix(OrbitalTransfers.TransferInfo transferInfo,
                       UniverseTime startTime,
                       double offsetDegrees,
                       ref UniverseTime __result)
    {
        try
        {
            // Celestial targets only.
            // Stock lists vehicles as targets with no eccentricity filter, and a vehicle on an escape trajectory in the same SOI has nothing to do with a heliocentric periapsis model.
            if (transferInfo.Target is not Celestial
                || transferInfo.Target.Orbit == null
                || transferInfo.Target.Orbit.IsBound())
                return true;

            UniverseTime tPeri = transferInfo.Target.Orbit.TimeAtPeriapsis;
            UniverseTime hohmannToF = transferInfo.HohmannTimeOfFlight;
            if (!(hohmannToF.Seconds() > 0.0))
            {
                // The alignment block builds a fresh TransferInfo per frame, and only SetTransferInfo ever assigns HohmannTimeOfFlight, so derive it the same way or the lead time this patch exists for vanishes.
                hohmannToF = OrbitalTransfers.HohmannFlight(
                    transferInfo.Source.Orbit, transferInfo.Target.Orbit);
                if (!(hohmannToF.Seconds() > 0.0)) return true;
            }

            UniverseTime ideal = tPeri - hohmannToF;

            if (offsetDegrees != 0.0)
            {
                // The periapsis model has no phase angle, so it cannot apply this offset.
                LogHelper.WarnOnce(
                    $"alignment-offset-{(transferInfo.Target as Astronomical)?.Id ?? "?"}",
                    $"[AFC] AlignmentTime: the {offsetDegrees:F1} degree phase offset does not " +
                    "apply to an unbound target. Departing one Hohmann time before periapsis.");
            }

            if (ideal < startTime)
            {
                string targetId = (transferInfo.Target as Astronomical)?.Id ?? "?";
                LogHelper.WarnOnce($"alignment-past-{targetId}",
                    $"[AFC] {targetId}: ideal departure is already past (periapsis at sim " +
                    $"t={tPeri.Seconds():F0}s minus Hohmann ToF {hohmannToF.Seconds():F0}s); " +
                    "alignment time clamped to startTime.");
            }

            __result = UniverseTime.Max(ideal, startTime);
            return false;
        }
        catch (Exception ex)
        {
            // TransferTask.Run rethrows this on a ThreadPool thread.
            // Log each fault type once because the plan window can call this on every frame.
            LogHelper.WarnOnce("alignment-time:" + ex.GetType().Name,
                $"[AFC] AlignmentTime prefix for target " +
                $"'{(transferInfo?.Target as Astronomical)?.Id ?? "?"}' at sim " +
                $"t={startTime.Seconds():F0}s: {ex}");
            return true;
        }
    }
}
