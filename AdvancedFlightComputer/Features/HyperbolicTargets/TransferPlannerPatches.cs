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
    static bool Prefix(Orbit origin, Orbit destination, ref SimTime __result)
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
    /// <summary>Targets already alerted about this mod load, so re-selecting a
    /// destination does not re-announce the same thing.</summary>
    private static readonly HashSet<string> _alertedTargets = new();

    public static void Reset() => _alertedTargets.Clear();

    static void Postfix()
    {
        try
        {
            OrbitalTransfers.TransferInfo? info = StockPlanner.TransferInfo;
            if (info?.Target?.Orbit == null) return;
            if (info.Target.Orbit.IsBound()) return;

            SimTime hohmann = info.HohmannTimeOfFlight;
            if (double.IsNaN(hohmann.Seconds()) || hohmann.Seconds() <= 0.0)
                return;

            info.MinTransferTimeOfFlight = hohmann * HyperbolicTargets.MinTofRatio;
            info.MaxTransferTimeOfFlight = hohmann * HyperbolicTargets.MaxTofRatio;

            GameReflection.TransferPlanner_selectedMinTime!
                .SetValue(null, new SimTime(info.MinTransferTimeOfFlight.Seconds()));
            GameReflection.TransferPlanner_selectedMaxTime!
                .SetValue(null, new SimTime(info.MaxTransferTimeOfFlight.Seconds()));

            MaybeAlertDepartureWindowPast(info, hohmann);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] SetTransferInfo postfix: {ex}");
        }
    }

    /// <summary>Says once per target that the ideal departure - one transfer time
    /// before the target's periapsis - is already behind the current sim time,
    /// which is why the porkchop window looks degenerate.
    ///
    /// This lives here rather than in <see cref="Patch_AlignmentTime"/>, which
    /// evaluates the same condition but also runs on the porkchop worker thread.
    /// <c>TransferPlanner.SetTransferInfo</c> is only ever called from
    /// <c>TransferPlanner.DrawPlanWindow</c>, so this postfix is draw-thread only
    /// and both <c>TimedAlert.Create</c> and the plain dedup set above are safe
    /// here.</summary>
    private static void MaybeAlertDepartureWindowPast(
        OrbitalTransfers.TransferInfo info, SimTime hohmannToF)
    {
        SimTime tPeri = info.Target.Orbit.TimeAtPeriapsis;
        SimTime ideal = new SimTime(tPeri.Seconds() - hohmannToF.Seconds());
        if (!(ideal < Universe.GetElapsedSimTime())) return;

        string targetId = (info.Target as Astronomical)?.Id ?? "?";
        if (!_alertedTargets.Add(targetId)) return;

        // Deliberately does not claim the transfer falls back to a later window:
        // an unbound target has a single periapsis passage, and TransferTask.Run
        // builds its start sweep from the current sim time regardless of what
        // AlignmentTime returned.
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
                       SimTime startTime,
                       ref SimTime __result)
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

            SimTime tPeri = transferInfo.Target.Orbit.TimeAtPeriapsis;
            SimTime hohmannToF = transferInfo.HohmannTimeOfFlight;
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

            SimTime ideal = new SimTime(tPeri.Seconds() - hohmannToF.Seconds());

            if (ideal < startTime)
            {
                string targetId = (transferInfo.Target as Astronomical)?.Id ?? "?";
                LogHelper.WarnOnce($"alignment-past-{targetId}",
                    $"[AFC] {targetId}: ideal departure is already past (periapsis at sim " +
                    $"t={tPeri.Seconds():F0}s minus Hohmann ToF {hohmannToF.Seconds():F0}s); " +
                    "alignment time clamped to startTime.");
            }

            __result = SimTime.Max(ideal, startTime);
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
