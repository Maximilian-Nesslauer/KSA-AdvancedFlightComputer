#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Features.Ui.Logic;
using KSA;

// Entry guidance (Modes/ThreeDof.cs) in the console, through the mode registry: the live check, the head readout, the commit of LAND > LAND NOW with ENTRY chosen, the one stop and its DIAG view. A boostback glide hands the craft to entry guidance with no commit (StepBoostback), so the live check reads the state and not the page.
public static partial class GuidanceWindow
{
    private static GuidanceModeHooks EntryModeHooks() => new(
        GuidanceModeId.EntryHere,
        IsLive: EntryLive,
        PhaseLabel: EntryPhaseLabel,
        Commit: (vehicle, orbit, parent) => RequestEntryGuidance(vehicle),
        Stop: StopEntryGuidance,
        Diagnostics: new ModeDiagnostics(GuidanceModeId.EntryHere, "ENTRY", DrawEntryDiagnostics),
        Readout: EntryReadout,
        StopText: s => EntryGuidance.StopText(EntryStageOf(s), s.ControlAcquired));

    /// <summary>Entry guidance flies the craft of <paramref name="s"/>, or its engage request waits for the next step.</summary>
    internal static bool EntryLive(VehicleAutopilotState s)
        => s.ThreeDofPhase is ThreeDofPhase.Glide or ThreeDofPhase.Burn || s.ThreeDofEngagePending;

    private static EntryStage EntryStageOf(VehicleAutopilotState s) => s.ThreeDofPhase switch
    {
        ThreeDofPhase.Burn => EntryStage.Burn,
        ThreeDofPhase.Glide => EntryStage.Glide,
        _ => EntryStage.Requested,
    };

    private static string EntryPhaseLabel(VehicleAutopilotState s) => s.ThreeDofPhase switch
    {
        ThreeDofPhase.Burn => "LANDING BURN",
        ThreeDofPhase.Glide => "GLIDE",
        _ => "START",
    };

    /// <summary>
    /// Entry guidance glides with no model, because the aero sweep of the craft found no atmosphere. Step3Dof builds the model only from a sweep with an atmosphere, so it never plans, and the glide branch holds the engine off on every step to the ground.
    /// </summary>
    internal static bool EntryHasNoAir(VehicleAutopilotState s)
        => s.ThreeDofPhase == ThreeDofPhase.Glide && s.ThreeDofNominal == null && s.Aero is { Atmosphere: null };

    /// <summary>The body of the craft has no atmosphere, which is the case EntryHasNoAir reports once entry guidance flies (Astronomical.GetPhysicalAtmosphere is null).</summary>
    internal static bool EntryBodyAirless(Vehicle vehicle)
        => vehicle?.Orbit?.Parent is IParentBody parent && !HasPhysicalAtmosphere(parent);

    /// <summary>Sim seconds an ended entry guidance warning stays in the head while no other landing mode flies.</summary>
    private const double EntryEndedTextSeconds = 30.0;

    private sealed class EntryEnd
    {
        public string Text;
        public double Time = double.NaN;
        public bool Superseded;
    }

    // The last end of entry guidance the console saw, per craft. The mode keeps its last text after the end, so the head needs its own limit for it.
    private static readonly ConditionalWeakTable<VehicleAutopilotState, EntryEnd> _entryEnds = new();

    /// <summary>
    /// The head lines of entry guidance. While it flies, its status and the no-air error. After it ends, only a warning, and only until another landing mode flies or EntryEndedTextSeconds pass, because no other step clears the text. A hand-over to touchdown guidance ends the warning when touchdown guidance takes the burn.
    /// </summary>
    internal static void AddEntryStatusLines(List<StatusLine> lines)
    {
        EntryEnd end = _entryEnds.GetValue(_s, _ => new EntryEnd());
        if (EntryLive(_s))
        {
            end.Text = null;
            end.Superseded = false;
            // A pending request still carries the text of the last run, so only a live mode shows its text.
            if (!ThreeDofLive)
                return;
            if (_s.ThreeDofStatus.Length > 0 && _s.ThreeDofSeverity != Severity.Diag)
                lines.Add(new StatusLine(_s.ThreeDofSeverity, EntryText(_s.ThreeDofStatus), ""));
            if (EntryHasNoAir(_s))
                lines.Add(new StatusLine(Severity.Error, EntryGuidance.NoAirText, ""));
            return;
        }

        if (_s.ThreeDofStatus.Length == 0)
            return;
        Severity severity = _s.ThreeDofSeverity;
        if (severity < Severity.Warning)
            return;
        double now = SimNow();
        if (!string.Equals(end.Text, _s.ThreeDofStatus, StringComparison.Ordinal))
        {
            end.Text = _s.ThreeDofStatus;
            end.Time = now;
            end.Superseded = false;
        }
        LiveLanding other = LandSolvers.Live(_s.Active || _s.EngagePending, false,
            _s.LandingPhase == LandingPhase.TerminalHover, LandingMachineLive, BoostbackLive);
        if (other != LiveLanding.None)
            end.Superseded = true;
        if (end.Superseded || !(Math.Abs(now - end.Time) <= EntryEndedTextSeconds))
            return;
        lines.Add(new StatusLine(severity, EntryText(_s.ThreeDofStatus), ""));
    }

    private static string EntryText(string text)
        => text.StartsWith("ENTRY GUIDANCE", StringComparison.OrdinalIgnoreCase) ? text : "ENTRY GUIDANCE: " + text;

    // The countdown that matters in each phase: the planned ignition in the glide, the burn left in the burn, and the boostback's time to impact while the first plan of the glide is solved. Only the boostback glide writes GlideTgo, so a burn without a plan shows no number rather than a frozen one.
    private static ModeLiveReadout EntryReadout(Vehicle vehicle, VehicleAutopilotState s)
    {
        double now = SimNow();
        var plan = s.ThreeDofPlan;
        HeadValue[] numbers = LandingNumbers(vehicle, site: true);
        PlanWord word = PlanWords.Of(plan != null, EntryNoPlanLasts(s), s.ThreeDofRefusing);
        if (s.ThreeDofPhase == ThreeDofPhase.Burn && plan != null)
            return new ModeLiveReadout(Math.Max(plan.EndTime - now, 0.0), "BURN LEFT", numbers, false, word, HeightShown: true, ImpactShown: true);
        if (s.ThreeDofPhase == ThreeDofPhase.Glide && plan != null && plan.IgnitionTime > now)
            return new ModeLiveReadout(plan.IgnitionTime - now, "TO IGNITION", numbers, false, word, HeightShown: true);
        bool waiting = s.ThreeDofPhase != ThreeDofPhase.Glide || plan == null;
        bool glideTgo = s.ThreeDofPhase == ThreeDofPhase.Glide && plan == null && double.IsFinite(s.GlideTgo);
        return new ModeLiveReadout(glideTgo ? s.GlideTgo : double.NaN, "IMPACT T-", numbers, waiting, word, HeightShown: true, ImpactShown: glideTgo);
    }

    // Entry guidance stays without a plan past the wait of the head (LastingProblem), or it never gets one on this body.
    private static bool EntryNoPlanLasts(VehicleAutopilotState s) => EntryHasNoAir(s) || NoPlanHolds(s);

    /// <summary>
    /// The take-over of entry guidance that has no plan, for the head: the same start as the verb of TOUCHDOWN on LAND NOW. Or null while the mode has a plan or may still get one. It asks no question, because this glide holds the engine off to the ground and the seconds count. The take-over on the page keeps its question.
    /// </summary>
    internal static CommitAction EntryTakeOver(Vehicle vehicle)
    {
        if (vehicle == null || !ThreeDofLive || _s.ThreeDofPlan != null || !EntryNoPlanLasts(_s))
            return null;
        return new CommitAction(PageText.TakeOverTouchdown, "", PageText.TakeOverTouchdownTip, () =>
        {
            Use(vehicle);
            RunCommit(vehicle, GuidanceModeId.SixDofHere);
        }, Danger: false, Lead: true);
    }

    // The engage request is taken by the next guidance step, ahead of the control claim (ApplyAutopilot), and the claim of Engage3Dof ends the mode it takes over. A deorbit that stock AUTO flies is the exception, because the step returns from StepStockDeorbit before it reads the request, so the request would wait for the end of the deorbit. The take-over ends that deorbit here, with the same abort as its stop. The landing turns AFC MAY USE ENGINES on, because entry guidance throttles the engine.
    private static void RequestEntryGuidance(Vehicle vehicle)
    {
        if (IsStockDeorbitPhase(_s.LandingPhase))
            StopGuidance(vehicle, StopTarget.Landing);
        _s.AutoStage = true;
        _s.ThreeDofEngagePending = true;
    }

    // A request that has not engaged is only cancelled. A live entry guidance ends through Disengage3Dof, which leaves a lit burn running and cuts the engine in the glide.
    private static void StopEntryGuidance(Vehicle vehicle)
    {
        _s.ThreeDofEngagePending = false;
        if (ThreeDofLive)
            Disengage3Dof("ENTRY GUIDANCE STOPPED");
    }
}
