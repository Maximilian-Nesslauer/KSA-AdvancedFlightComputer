#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Features.Ui.Logic;
using KSA;

// Orbital entry guidance (Modes/OrbitalEntry.cs, with bellyflop as its last phase) in the console, through the mode registry: the live check, the head readout and status, the commit of LAND > ENTRY with ORBITAL ENTRY chosen, and the one stop. A DEORBIT ONLY burn hands the craft to it with no commit (OrbitalEntryAfterDeorbit), so the live check reads the state and not the page.
public static partial class GuidanceWindow
{
    private static GuidanceModeHooks OrbitalEntryModeHooks() => new(
        GuidanceModeId.OrbitalEntry,
        IsLive: s => OrbitalEntryLiveFor(s) || s.OrbitalEntry.EngagePending,
        PhaseLabel: OrbitalEntryPhaseLabel,
        Commit: (vehicle, orbit, parent) => RequestOrbitalEntry(vehicle),
        Stop: StopOrbitalEntryFromConsole,
        Readout: OrbitalEntryReadout,
        StopText: s => OrbitalEntryGuidance.StopText(OrbitalEntryStageOf(s)));

    private static OrbitalEntryStage OrbitalEntryStageOf(VehicleAutopilotState s) => s.OrbitalEntry.Phase switch
    {
        OrbitalEntryPhase.Sweeping => OrbitalEntryStage.Sweeping,
        OrbitalEntryPhase.Entry => OrbitalEntryStage.Entry,
        OrbitalEntryPhase.Holding => OrbitalEntryStage.Holding,
        OrbitalEntryPhase.Bellyflop => OrbitalEntryStage.Bellyflop,
        _ => OrbitalEntryStage.Requested,
    };

    private static string OrbitalEntryPhaseLabel(VehicleAutopilotState s) => s.OrbitalEntry.Phase switch
    {
        OrbitalEntryPhase.Entry => "ENTRY",
        OrbitalEntryPhase.Holding => "HOLD",
        OrbitalEntryPhase.Bellyflop => "BELLYFLOP",
        _ => "SWEEP",
    };

    /// <summary>
    /// The commit: the next guidance step takes the request ahead of the claim (EngageOrbitalEntry), from wherever the craft is, so a stopped entry is picked up again from its own state. The claim ends the mode it takes over.
    /// </summary>
    private static void RequestOrbitalEntry(Vehicle vehicle)
    {
        var st = _s.OrbitalEntry;
        st.StopPending = false;
        st.EngagePending = true;
        _entryTrajClosed = false;
    }

    // A request that has not engaged is only cancelled. A live entry ends as its own stop does (StopOrbitalEntry): its attitude let go, the engine as it is.
    private static void StopOrbitalEntryFromConsole(Vehicle vehicle)
    {
        _s.OrbitalEntry.EngagePending = false;
        if (OrbitalEntryLive)
            StopOrbitalEntry(vehicle, "ORBITAL ENTRY STOPPED.");
    }

    // The countdown: the plan's fall to the ground under bellyflop, else none, because the entry plans in energy and not in time. The numbers are the landing's: site, height and speed.
    private static ModeLiveReadout OrbitalEntryReadout(Vehicle vehicle, VehicleAutopilotState s)
    {
        var st = s.OrbitalEntry;
        HeadValue[] numbers = LandingNumbers(vehicle, site: true);
        if (st.Phase == OrbitalEntryPhase.Bellyflop && st.Bellyflop.Plan is { } fall)
        {
            double left = Math.Max(fall.EndTime - SimNow(), 0.0);
            return new ModeLiveReadout(left, "IMPACT T-", numbers, false, PlanWords.OnPlan, HeightShown: true, ImpactShown: true);
        }
        bool waiting = st.Phase is OrbitalEntryPhase.Sweeping || st.EngagePending || (st.Phase == OrbitalEntryPhase.Entry && st.Plan == null);
        PlanWord word = st.Phase != OrbitalEntryPhase.Entry ? default
            : st.Plan == null ? PlanWords.Planning
            : st.Plan.Converged ? PlanWords.OnPlan
            : PlanWords.LastPlan;
        return new ModeLiveReadout(double.NaN, "", numbers, waiting, word, HeightShown: true);
    }

    /// <summary>Sim seconds a refusal or an end of orbital entry stays in the head while no other landing mode flies.</summary>
    private const double OrbitalEntryEndedTextSeconds = 30.0;

    private sealed class OrbitalEntryEnd
    {
        public string Text;
        public double Time = double.NaN;
    }

    private static readonly ConditionalWeakTable<VehicleAutopilotState, OrbitalEntryEnd> _orbitalEntryEnds = new();

    /// <summary>The head lines of orbital entry: its status while it flies, and a warning it left (a refusal) for a while after, until another landing mode flies.</summary>
    internal static void AddOrbitalEntryStatusLines(List<StatusLine> lines)
    {
        var st = _s.OrbitalEntry;
        OrbitalEntryEnd end = _orbitalEntryEnds.GetValue(_s, _ => new OrbitalEntryEnd());
        if (OrbitalEntryLiveFor(_s))
        {
            end.Text = null;
            if (st.Status.Length > 0)
                lines.Add(new StatusLine(st.StatusWarn ? Severity.Warning : Severity.Info, OrbitalEntryText(st.Status), ""));
            return;
        }
        if (st.EngagePending || !st.StatusWarn || st.Status.Length == 0)
            return;
        double now = SimNow();
        if (!string.Equals(end.Text, st.Status, StringComparison.Ordinal))
        {
            end.Text = st.Status;
            end.Time = now;
        }
        LiveLanding other = LandSolvers.Live(_s.Active || _s.EngagePending, ThreeDofLive,
            _s.LandingPhase == LandingPhase.TerminalHover, LandingMachineLive, BoostbackLive);
        if (other != LiveLanding.None || !(Math.Abs(now - end.Time) <= OrbitalEntryEndedTextSeconds))
            return;
        lines.Add(new StatusLine(Severity.Warning, OrbitalEntryText(st.Status), ""));
    }

    private static string OrbitalEntryText(string text)
        => text.StartsWith("ORBITAL ENTRY", StringComparison.OrdinalIgnoreCase) || text.StartsWith("BELLYFLOP", StringComparison.OrdinalIgnoreCase)
            ? text
            : "ORBITAL ENTRY: " + text;
}
