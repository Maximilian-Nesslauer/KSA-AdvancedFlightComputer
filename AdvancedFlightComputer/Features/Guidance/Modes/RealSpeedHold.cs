#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System.Collections.Generic;
using KSA;

/// <summary>
/// Holds the simulation at 1x while a craft's guidance plans against the clock, and gives the player's speed back after the last hold ends.
/// The speed is global and several craft can hold at once, so the holders share one remembered speed. A holder that stops asking without a release, because its craft was destroyed or left the step loop, counts as gone after <see cref="StaleSeconds"/> of sim time; sim time stands still while the game is paused, so a pause never ends a hold. A holder that goes without a release also drops the remembered speed, because a later hold that changed nothing must not give back the speed of a hold that ended long ago.
/// </summary>
internal static class RealSpeedHold
{
    private const double StaleSeconds = 5.0;

    private static readonly Dictionary<Vehicle, double> Holders = new(ReferenceEqualityComparer.Instance);

    // The speed the player had when a hold lowered it, NaN when there is nothing to give back. An auto warp that a hold stops is not remembered, because a fixed speed is not what the warp would have done.
    private static double _playerSpeed = double.NaN;

    public static bool Holding(Vehicle vehicle) => Holders.ContainsKey(vehicle);

    public static double RememberedSpeed => _playerSpeed;

    /// <summary>Keeps the simulation at 1x for <paramref name="vehicle"/>. Returns the speed it lowered from on this call, NaN when it changed nothing. A speed that the player raises during the hold is lowered again and becomes the speed given back.</summary>
    public static double Hold(Vehicle vehicle, double simNow)
    {
        PruneStale(simNow);
        Holders[vehicle] = simNow;
        bool warp = Universe.IsAutoWarpActive;
        double speed = Universe.GetSimulationSpeed();
        if (!warp && !(speed > 1.0))
            return double.NaN;
        if (!warp)
            _playerSpeed = speed;
        Universe.ResetSimulationSpeed();
        return speed;
    }

    /// <summary>
    /// Ends the hold of <paramref name="vehicle"/>. When no hold is left, the remembered speed goes back, but only while the simulation still runs at the 1x the hold set: a pause, a warp or a speed the player chose since then stays as it is. With <paramref name="giveBack"/> false the remembered speed is dropped, for a craft that goes on into a phase that needs real speed. Returns the speed given back, NaN when none.
    /// </summary>
    public static double Release(Vehicle vehicle, double simNow, bool giveBack)
    {
        bool held = Holders.Remove(vehicle);
        if (!giveBack && held)
            _playerSpeed = double.NaN;
        PruneStale(simNow);
        if (Holders.Count > 0 || double.IsNaN(_playerSpeed))
            return double.NaN;
        double speed = _playerSpeed;
        _playerSpeed = double.NaN;
        if (Universe.IsAutoWarpActive || Universe.GetSimulationSpeed() != 1.0)
            return double.NaN;
        Universe.SetSimulationSpeed(speed);
        return speed;
    }

    /// <summary>Drops the hold of <paramref name="vehicle"/> without a give-back, for a craft that leaves the step loop without a release: it was destroyed, or guidance was switched off.</summary>
    public static void Forget(Vehicle vehicle)
    {
        if (Holders.Remove(vehicle))
            _playerSpeed = double.NaN;
    }

    /// <summary>Forgets every hold without changing the speed, for a save load or a guidance shutdown.</summary>
    public static void Clear()
    {
        Holders.Clear();
        _playerSpeed = double.NaN;
    }

    private static void PruneStale(double simNow)
    {
        if (Holders.Count == 0)
            return;
        List<Vehicle> stale = null;
        foreach (KeyValuePair<Vehicle, double> holder in Holders)
        {
            if (holder.Key.IsDisposed || simNow - holder.Value > StaleSeconds || simNow < holder.Value)
                (stale ??= new List<Vehicle>()).Add(holder.Key);
        }
        if (stale == null)
            return;
        foreach (Vehicle vehicle in stale)
            Holders.Remove(vehicle);
        _playerSpeed = double.NaN;
    }
}

public static partial class GuidanceWindow
{
    /// <summary>
    /// Runs from ApplyAutopilot ahead of the launch planner and the deorbit planning. A launch plan is solved for a lift-off instant and is refused when the craft lifts off more than <see cref="PlanLaunchToleranceS"/> later, and a solve takes seconds of wall time, so above 1x the plan is stale when it lands. The deorbit search is captured from the state at one instant, so it runs only at 1x. Both therefore hold the simulation at 1x for as long as they run. The braking burn, the powered descent and the touchdown phases set 1x themselves and must not get the old speed back, so a hold that ends into one of them drops it. The same applies to the coasts to a planned burn and to a launch armed for a window: they run to a fixed instant, and at a high fixed speed one frame step can carry the craft past it (Universe.GetJobSimStep scales the step with the speed). For these, the warp that guidance offers stops in time.
    /// A launch plan is collected only by StepAscentPlanner, which an unfocused craft that flies nothing does not reach, so a pending launch holds only while its craft is in focus. Otherwise a craft left during its solve would keep the whole game at 1x.
    /// </summary>
    private static void StepRealSpeedHold(Vehicle vehicle, bool focused)
    {
        string why = _s.ConvexLaunchPending && focused ? "solves the launch plan"
            : _s.LandingPhase == LandingPhase.DeorbitPlanning ? "plans the deorbit burn"
            : "";
        double now = SimNow();
        if (why.Length > 0)
        {
            double lowered = RealSpeedHold.Hold(vehicle, now);
            if (!double.IsNaN(lowered))
                GuidanceLog.Info(vehicle, $"simulation speed set from {lowered:F1}x to 1x while guidance {why}.");
            return;
        }
        if (!RealSpeedHold.Holding(vehicle))
            return;
        bool realSpeedNext = _s.Active || _s.EngagePending || _s.LaunchArmed
            || _s.LandingPhase is LandingPhase.Coast or LandingPhase.DeorbitCoast or LandingPhase.Prep or LandingPhase.Burn or LandingPhase.GfoldDescent
                or LandingPhase.TerminalHover or LandingPhase.TerminalCoast or LandingPhase.TerminalBrake;
        double restored = RealSpeedHold.Release(vehicle, now, giveBack: !realSpeedNext);
        if (!double.IsNaN(restored))
            GuidanceLog.Info(vehicle, $"simulation speed set back to {restored:F1}x.");
    }
}
