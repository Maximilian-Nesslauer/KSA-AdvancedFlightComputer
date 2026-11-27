using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>What the cut supervisor does this step.</summary>
public enum CutSupervisorAction
{
    /// <summary>Not armed: no landing descent seen yet, the engine is off, or the craft is down.</summary>
    Idle,
    /// <summary>Armed, outside the cut window or keeping up its sink: nothing to do.</summary>
    Watching,
    /// <summary>In the window and short of its sink: the shortfall is building.</summary>
    Building,
    /// <summary>Cut this step: the engine goes off.</summary>
    Cut,
    /// <summary>Cut earlier: the engine stays off, whatever guidance asks.</summary>
    HoldOff,
}

/// <summary>The cut supervisor's settings: the sink it expects near the ground, the shortfall it cuts at, the highest legs it cuts from, and the descent that arms it.</summary>
public readonly record struct CutSupervisorSettings(double TargetSinkMs, double ShortfallCutM, double MaxCutHeightM, double ArmSinkMs = 1.0);

/// <summary>The cut supervisor's memory between steps, one per craft.</summary>
public struct CutSupervisorState
{
    /// <summary>Seen descending faster than the arming sink with the engine lit: a landing descent, not a craft on the pad.</summary>
    public bool Armed;
    /// <summary>Has cut; latched until the craft is released.</summary>
    public bool Cut;
    /// <summary>The height not descended, m: the integral of the target sink less the actual sink, inside the cut window, never below zero.</summary>
    public double ShortfallM;
}

/// <summary>
/// THE CUT SUPERVISOR (#335): a watchdog beside touchdown guidance, not part of it, that cuts the engine when a landing has stopped coming down just above the ground - the failure of every one-engine Starship landing on 2026-11-27, whose plans brought it to rest 0.5-2 m up and then drifted, cut and relit, or toppled.
///  It reads only the legs' height above the ground under the craft (a radio altitude) and the vertical speed. Armed once the craft has descended faster than <see cref="CutSupervisorSettings.ArmSinkMs"/> with the engine lit, it integrates, while the legs are within <see cref="CutSupervisorSettings.MaxCutHeightM"/> of the ground, the target sink less the actual sink: the height it has failed to descend. A slight stall builds it slowly, a hover at a steady rate, a climb fast, and sinking faster than the target takes it back down, never below zero. At <see cref="CutSupervisorSettings.ShortfallCutM"/> it cuts, and the engine stays off from then on: a free fall from the window's top, 2 m, ends at about 6.3 m/s, which the user chose over a craft left hovering.
///  Pure: no game types, so UiLogic tests it.
/// </summary>
public static class CutSupervisor
{
    public static CutSupervisorAction Step(ref CutSupervisorState state, in CutSupervisorSettings settings,
                                           double legsHeightM, double verticalSpeedMs, double dt, bool engineLit, bool contact)
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

        // Outside the window - too high, or no height - nothing builds and nothing carries over.
        if (!double.IsFinite(legsHeightM) || legsHeightM > settings.MaxCutHeightM)
        {
            state.ShortfallM = 0.0;
            return CutSupervisorAction.Watching;
        }

        double step = Math.Clamp(dt, 0.0, 0.25);
        double sink = -verticalSpeedMs;
        state.ShortfallM = Math.Max(state.ShortfallM + (settings.TargetSinkMs - sink) * step, 0.0);
        if (state.ShortfallM >= settings.ShortfallCutM)
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
