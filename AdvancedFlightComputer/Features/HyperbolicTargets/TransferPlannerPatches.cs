using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
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
            if (StockPlanner.SourceVehicle is not Vehicle source) return;

            var star = HyperbolicTargets.GetParentStar(source);
            if (star == null) return;

            // Only offer hyperbolic targets when the source frame is
            // heliocentric: the vehicle orbits the star directly, or a planet
            // that orbits the star. From a moon orbit stock shows only sibling
            // bodies, so a heliocentric comet target there would just produce a
            // nonsensical plan.
            IParentBody? sourceParent = source.Parent;
            if (sourceParent != star && (sourceParent as Celestial)?.Parent != star)
                return;

            ReadOnlySpan<Astronomical> all = Universe.CurrentSystem!.All.AsSpan();
            for (int i = 0; i < all.Length; i++)
            {
                if (all[i] is not Celestial celestial) continue;
                // Deliberately e < 1.0 and NOT IsBound(): this is the exact
                // complement of stock's own filter, so no body is listed twice.
                // The handling guards below use IsBound() instead, so a
                // game-parabolic body just under e = 1.0 (which stock lists but
                // NaNs on) is still taken over.
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
            DefaultCategory.Log.Warning($"[AFC] PopulateWithPlanets postfix: {ex}");
        }
    }
}

/// <summary>
/// HohmannFlight derives the transfer ellipse SMA from (Apoapsis + Periapsis) / 2.
/// For unbound orbits OrbitData sets Apoapsis to NaN, the NaN propagates through
/// the sqrt, and the time-of-flight estimate becomes NaN. Each unbound end gets
/// its Periapsis substituted (finite and time-invariant, a stable baseline for
/// the porkchop search); a bound end keeps the semi-major axis stock would use.
///
/// Unbound is tested with IsBound(), not e &gt;= 1.0: the game classifies the
/// band |e - 1| &lt;= 1e-6 as parabolic (SMA infinite, Apoapsis NaN), so an
/// eccentricity compare would hand a game-parabolic orbit just under 1.0 back
/// to stock's NaN math. Same reasoning for the guards in the other patches of
/// this feature.
/// </summary>
[HarmonyPatch(typeof(OrbitalTransfers), nameof(OrbitalTransfers.HohmannFlight))]
internal static class Patch_HohmannFlight
{
    static bool Prefix(Orbit origin, Orbit destination, ref UniverseTime __result)
    {
        if (origin.IsBound() && destination.IsBound())
            return true;

        double r1 = origin.IsBound()
            ? origin.SemiMajorAxis
            : origin.Periapsis;
        double r2 = destination.IsBound()
            ? destination.SemiMajorAxis
            : destination.Periapsis;

        double transferSma = (r1 + r2) * 0.5;
        if (transferSma <= 0.0)
            transferSma = Math.Max(r1, r2);

        double tof = Math.PI * Math.Sqrt(transferSma * transferSma * transferSma / origin.Mu);
        // A degenerate radius or mu still reaches here (the transferSma <= 0.0
        // test above is false for NaN), and UniverseTime rejects NaN outright.
        // Running the original is NOT an escape: stock takes its
        // (Apoapsis + Periapsis) / 2 branch for any orbit above e = 0.01, and
        // Apoapsis is NaN on the unbound orbit this prefix exists for, so it
        // would construct from NaN one frame deeper. Zero is the sentinel every
        // consumer of this estimate already tests for (both AFC patches gate on
        // "> 0.0" before using it), so it degrades instead of throwing.
        //
        // Worth the care because the porkchop worker reaches this: stock's
        // TransferTask.Run is a ThreadPool work item and calls AlignmentTime,
        // whose AFC prefix calls HohmannFlight on its fallback path.
        if (!double.IsFinite(tof))
        {
            LogHelper.WarnOnce(
                $"hohmann-tof-degenerate-{origin.Parent?.Id ?? "?"}",
                $"[AFC] HohmannFlight: degenerate geometry (r1={r1:E3}m r2={r2:E3}m " +
                $"mu={origin.Mu:E3}); reporting no transfer estimate.");
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
            if (info?.Target?.Orbit == null || info.Target.Orbit.IsBound())
                return __exception;
            if (__exception != null && __exception is not ArgumentException)
                return __exception;

            double hohmannSec = info.HohmannTimeOfFlight.Seconds();
            if (!(hohmannSec > 0.0))
            {
                // No estimate to size a window from. A null TransferInfo is stock's
                // own "nothing selected" state, so the window degrades instead of
                // throwing out of the draw every frame.
                if (GameReflection.TransferPlanner_transferInfoRef is { } infoRef)
                    infoRef() = null;
                LogHelper.WarnOnce(
                    $"transfer-window-degenerate-{(info.Target as Astronomical)?.Id ?? "?"}",
                    "[AFC] SetTransferInfo: no Hohmann estimate for the unbound target; " +
                    "transfer cleared.");
                return null;
            }

            info.MinTransferTimeOfFlight = new UniverseTime(hohmannSec * HyperbolicTargets.MinTofRatio);
            info.MaxTransferTimeOfFlight = new UniverseTime(hohmannSec * HyperbolicTargets.MaxTofRatio);
            GameReflection.TransferPlanner_selectedMinTime!.SetValue(null, info.MinTransferTimeOfFlight);
            GameReflection.TransferPlanner_selectedMaxTime!.SetValue(null, info.MaxTransferTimeOfFlight);
            SelectTimeUnit(hohmannSec);

            MaybeAlertDepartureWindowPast(info, info.HohmannTimeOfFlight);
            return null;
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] SetTransferInfo finalizer: {ex}");
            return __exception;
        }
    }

    /// <summary>Stock's own unit pick, replayed because it sits after the line
    /// that throws. It takes the first unit whose formatted estimate has at most
    /// three integer digits.</summary>
    private static void SelectTimeUnit(double hohmannSec)
    {
        if (GameReflection.TransferPlanner_timeUnits!.GetValue(null) is not List<TimeObject> units)
            return;
        foreach (TimeObject unit in units)
        {
            ReadOnlySpan<char> text = TimeSpanReference.AllocateNearestString(unit.Unit, hohmannSec);
            int end = text.IndexOfAny(' ', '.');
            int integerDigits = end < 0 ? text.Length : end;
            if (integerDigits <= 3)
            {
                GameReflection.TransferPlanner_selectedTimeUnit!.SetValue(null, unit);
                return;
            }
        }
    }

    /// <summary>Says once per target that the ideal departure, one transfer time
    /// before the target's periapsis, is already behind the sim time, which is
    /// why the porkchop window looks degenerate. <see cref="Patch_AlignmentTime"/>
    /// evaluates the same condition but also runs on the porkchop worker, and
    /// <c>TimedAlert.Create</c> mutates an unsynchronized list the draw thread walks.</summary>
    private static void MaybeAlertDepartureWindowPast(
        OrbitalTransfers.TransferInfo info, UniverseTime hohmannToF)
    {
        UniverseTime ideal = info.Target.Orbit.TimeAtPeriapsis - hohmannToF;
        if (!(ideal < Universe.GetElapsedTime())) return;

        string targetId = (info.Target as Astronomical)?.Id ?? "?";
        if (!_alertedTargets.Add(targetId)) return;

        // An unbound target has one periapsis passage, and TransferTask.Run sweeps
        // from the current sim time whatever AlignmentTime returned, so this does
        // not promise a later window.
        TimedAlert.Create(
            $"{targetId}: the ideal departure, one transfer time before its periapsis, " +
            "has already passed. The porkchop window starts from the current time.",
            Color.Yellow, 6.0);
    }
}

/// <summary>
/// AlignmentTime uses synodic period (infinite for hyperbolic targets).
/// For a hyperbolic flyby the cheapest intercept is near the target's
/// periapsis (closest to the Sun, slowest, longest dwell in the inner
/// system), so we depart roughly hohmann_tof before that.
///
/// This prefix runs on two threads. <c>TransferTask</c>'s constructor queues its
/// Run on the ThreadPool, and Run calls AlignmentTime whenever
/// <c>TransferInfo.Source</c> is not a Vehicle, which
/// <c>TransferPlanner.SetTransferInfo</c> makes true for any vehicle parked at a
/// Celestial; <c>TransferPlanner.DrawPlanWindow</c>'s "Show Parent/Target
/// Alignment" block calls it again every frame on the draw thread. So nothing
/// here may touch state that assumes one thread. In particular the user-facing
/// alert lives in <see cref="Patch_SetTransferInfo"/> instead:
/// <c>TimedAlert.Create</c> appends to and sorts a static list that
/// <c>Alert.DrawAll</c> walks and removes from on the draw thread, with no
/// synchronization on either side. What is left is the computation, the
/// <c>__result</c> write, and <see cref="LogHelper"/>'s locked dedup set.
/// </summary>
[HarmonyPatch(typeof(OrbitalTransfers), nameof(OrbitalTransfers.AlignmentTime))]
internal static class Patch_AlignmentTime
{
    static bool Prefix(OrbitalTransfers.TransferInfo transferInfo,
                       UniverseTime startTime,
                       ref UniverseTime __result)
    {
        try
        {
            // Celestial targets only. Stock's PopulateWithVehiclesAsTargets applies
            // no eccentricity filter, so a vehicle in the same SOI that is on an
            // escape trajectory would otherwise be routed through a heliocentric
            // "depart before the target's periapsis" model that says nothing about
            // it.
            if (transferInfo.Target is not Celestial
                || transferInfo.Target.Orbit == null
                || transferInfo.Target.Orbit.IsBound())
                return true;

            UniverseTime tPeri = transferInfo.Target.Orbit.TimeAtPeriapsis;
            UniverseTime hohmannToF = transferInfo.HohmannTimeOfFlight;
            if (!(hohmannToF.Seconds() > 0.0))
            {
                // The "Show Parent/Target Alignment" block news up a TransferInfo per
                // frame, and no TransferInfo constructor assigns HohmannTimeOfFlight -
                // only TransferPlanner.SetTransferInfo does. Left at zero the lead time
                // this patch exists to apply would vanish and the alignment marker
                // would sit at the target's periapsis instead of one transfer time
                // before it, so derive it the same way SetTransferInfo does.
                hohmannToF = OrbitalTransfers.HohmannFlight(
                    transferInfo.Source.Orbit, transferInfo.Target.Orbit);
                if (!(hohmannToF.Seconds() > 0.0)) return true;
            }

            UniverseTime ideal = tPeri - hohmannToF;

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
            // TransferTask.Run rethrows anything that is not an
            // OperationCanceledException, and it is a ThreadPool work item, so an
            // exception escaping here would be unhandled on a pool thread.
            DefaultCategory.Log.Warning($"[AFC] AlignmentTime prefix: {ex}");
            return true;
        }
    }
}
