using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>What the cut supervisor does this step.</summary>
public enum CutSupervisorAction
{
    /// <summary>Not armed: no landing descent seen yet, the engine is off, or the craft is down.</summary>
    Idle,
    /// <summary>Armed, above the watch window or keeping up its sink: nothing to do.</summary>
    Watching,
    /// <summary>In the watch window and short of its sink: the shortfall is building.</summary>
    Building,
    /// <summary>Cut this step: the engine goes off.</summary>
    Cut,
    /// <summary>Cut earlier: the engine stays off, whatever guidance asks.</summary>
    HoldOff,
}

/// <summary>
/// The cut supervisor's settings: the sink it expects near the ground, the shortfall that cuts at the cut height (less below it), the highest legs it cuts from, the height it starts counting at, and the descent that arms it.
/// </summary>
public readonly record struct CutSupervisorSettings(double TargetSinkMs, double ShortfallCutM, double MaxCutHeightM, double WatchHeightM, double ArmSinkMs = 1.0)
{
    /// <summary>The smallest shortfall that cuts, m, however low the legs. 0.05 since 2026-11-27: at 0.1 a Starship sinking 0.2-0.4 m/s with its legs 0.45 m up (14:28) took half a second to cut while it slid sideways at 3 m/s.</summary>
    public const double MinShortfallM = 0.05;

    /// <summary>The shortfall that cuts with the legs <paramref name="legsHeightM"/> above the ground: <see cref="ShortfallCutM"/> at the cut height, in proportion below it, never under <see cref="MinShortfallM"/>.</summary>
    public double ShortfallToCutM(double legsHeightM)
        => MaxCutHeightM > 0.0
            ? Math.Max(MinShortfallM, ShortfallCutM * Math.Clamp(legsHeightM, 0.0, MaxCutHeightM) / MaxCutHeightM)
            : MinShortfallM;

    /// <summary>The most the shortfall may hold, m: twice the cut height's threshold, so a long stall above the cut height does not stay owed once the craft is coming down again.</summary>
    public double ShortfallCapM => 2.0 * Math.Max(ShortfallCutM, MinShortfallM);
}

/// <summary>The cut supervisor's memory between steps, one per craft.</summary>
public struct CutSupervisorState
{
    /// <summary>Seen descending faster than the arming sink with the engine lit: a landing descent, not a craft on the pad.</summary>
    public bool Armed;
    /// <summary>Has cut; latched until the craft is released.</summary>
    public bool Cut;
    /// <summary>The cut came at the plan's end, not from the shortfall.</summary>
    public bool CutAtPlanEnd;
    /// <summary>The height not descended, m: the integral of the target sink less the actual sink inside the watch window, never below zero, capped at the settings' ShortfallCapM.</summary>
    public double ShortfallM;
}

/// <summary>
/// THE CUT SUPERVISOR (#335): a watchdog beside touchdown guidance, not part of it, that cuts the engine when a landing has stopped coming down just above the ground - the failure of every one-engine Starship landing on 2026-11-27, whose plans brought it to rest 0.5-2.6 m up and then drifted, cut and relit, or toppled.
///  It reads only the legs' height above the ground under the craft (a radio altitude) and the vertical speed. Armed once the craft has descended faster than <see cref="CutSupervisorSettings.ArmSinkMs"/> with the engine lit, it integrates, while the legs are within <see cref="CutSupervisorSettings.WatchHeightM"/> of the ground, the target sink less the actual sink: the height it has failed to descend. A slight stall builds it slowly, a hover at a steady rate, a climb fast, and sinking faster than the target pays it back, never below zero.
///  It cuts once the legs are within <see cref="CutSupervisorSettings.MaxCutHeightM"/> and the shortfall reaches <see cref="CutSupervisorSettings.ShortfallToCutM"/>: the full <see cref="CutSupervisorSettings.ShortfallCutM"/> at the cut height, less in proportion lower down, so a stall at 0.5 m is cut in a quarter of the time of one at 2 m - and only while the craft is still short of its sink at that moment. WATCHING ABOVE THE CUT HEIGHT (2026-11-27, 10:50: a 3.5 s hover with the legs 2.4-2.6 m up, just over a 2 m window, counted nothing): a stall up there builds its shortfall, and is cut as soon as it sinks into the cut height. The engine stays off from then on. A free fall from 2 m ends at about 6.3 m/s, which the user chose over a craft left hovering.
///  Pure: no game types, so UiLogic tests it.
/// </summary>
public static class CutSupervisor
{
    /// <param name="planEnded">Touchdown guidance has reached the end of its plan without contact and flies the rest to contact. Within the cut height that is a cut at once: the plan said the craft was down. On 2026-11-27 (14:27) a Starship reached its plan's end with the legs 0.47 m up, sliding at 4.6 m/s, and the shortfall took another 0.2 s to build.</param>
    public static CutSupervisorAction Step(ref CutSupervisorState state, in CutSupervisorSettings settings,
                                           double legsHeightM, double verticalSpeedMs, double dt, bool engineLit, bool contact,
                                           bool planEnded = false)
    {
        if (state.Cut)
            return CutSupervisorAction.HoldOff;
        if (contact)
        {
            state.ShortfallM = 0.0;
            return CutSupervisorAction.Idle;
        }
        if (!engineLit || !double.IsFinite(verticalSpeedMs))
            return state.Armed ? CutSupervisorAction.Watching : CutSupervisorAction.Idle;
        if (!state.Armed)
        {
            if (verticalSpeedMs < -settings.ArmSinkMs)
                state.Armed = true;
            else
                return CutSupervisorAction.Idle;
        }

        // Above the watch window - or no height - nothing builds and nothing carries over.
        double watch = Math.Max(settings.WatchHeightM, settings.MaxCutHeightM);
        if (!double.IsFinite(legsHeightM) || legsHeightM > watch)
        {
            state.ShortfallM = 0.0;
            return CutSupervisorAction.Watching;
        }

        // AT THE PLAN'S END, inside the cut height: down, as far as the plan knows.
        if (planEnded && legsHeightM <= settings.MaxCutHeightM)
        {
            state.Cut = true;
            state.CutAtPlanEnd = true;
            return CutSupervisorAction.Cut;
        }

        double step = Math.Clamp(dt, 0.0, 0.25);
        double sink = -verticalSpeedMs;
        state.ShortfallM = Math.Clamp(state.ShortfallM + (settings.TargetSinkMs - sink) * step, 0.0, settings.ShortfallCapM);
        // STALLED NOW as well as owed: a stall the craft has come out of - sinking at least the target again - is paid back, not cut. Replayed on 2026-11-27 10:50, a 3.5 s hover 2.5 m up that guidance then brought down at 0.9 m/s to a soft landing would otherwise have been cut 2 m up.
        if (legsHeightM <= settings.MaxCutHeightM && sink < settings.TargetSinkMs && state.ShortfallM >= settings.ShortfallToCutM(legsHeightM))
        {
            state.Cut = true;
            return CutSupervisorAction.Cut;
        }
        return sink < settings.TargetSinkMs ? CutSupervisorAction.Building : CutSupervisorAction.Watching;
    }

    /// <summary>The speed a free fall from <paramref name="legsHeightM"/> reaches the ground at, m/s, starting at <paramref name="verticalSpeedMs"/> (up positive).</summary>
    public static double ImpactSpeedMs(double legsHeightM, double verticalSpeedMs, double gravity)
    {
        double sink = Math.Max(-verticalSpeedMs, 0.0);
        double up = Math.Max(verticalSpeedMs, 0.0);
        // A climb first rises up^2/2g further.
        double fall = Math.Max(legsHeightM, 0.0) + up * up / (2.0 * gravity);
        return Math.Sqrt(sink * sink + 2.0 * gravity * fall);
    }
}
