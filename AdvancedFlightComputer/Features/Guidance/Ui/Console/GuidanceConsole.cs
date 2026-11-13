#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

// What the console shell asks of guidance, in one place: binding the frame to the subject craft, the head readout, the warp offer, the status lines and cues, the commit and the stop dispatch, and the windows drawn after the console. The shell lives in Features/Ui/Console and never touches guidance state itself.
public static partial class GuidanceWindow
{
    /// <summary>The cue target that opens the head drawer instead of a page.</summary>
    internal const string DrawerTarget = "DRAWER";

    /// <summary>Set by the shell each frame: a page that reads guidance state is on screen. The guidance step keeps an idle craft's stage model current while this holds.</summary>
    internal static bool GuidanceUiVisible;

    /// <summary>A cue target ("PAGE", "PAGE/SUBPAGE" or the drawer) a page button asks the shell to open. The shell reads and clears it after the window.</summary>
    internal static string PendingNavigation = "";

    /// <summary>A note about a commit that changed a craft switch on its own, for the shell to show once.</summary>
    internal static string PendingCommitNote = "";

    // A stop for a craft that is not the one on screen is applied by that craft's own step, after its lock checks, instead of from the draw.
    private static readonly List<(Vehicle Vehicle, StopTarget Target)> _stopRequests = new();

    internal static void BeginConsoleFrame(Vehicle vehicle)
    {
        EnsureAfcModes();
        Use(vehicle);
    }

    internal static void OnSubjectChanged()
    {
        _retargetArmed = false;
        _retargetStageId = 0;
        _footerPicksSite = false;
        _refusedLine = "";
    }

    /// <summary>Forgets the stops queued for other craft, because a save load replaces every craft they name.</summary>
    internal static void ResetWorldRequests() => _stopRequests.Clear();

    internal static void ResetConsoleState()
    {
        GuidanceUiVisible = false;
        PendingNavigation = "";
        PendingCommitNote = "";
        _followedLandWay = null;
        _siteTyping = false;
        _footerPicksSite = false;
        _refusedLine = "";
        PageShownProbe = null;
        ClearLandFact();
        _stopRequests.Clear();
        TierScope.Reset();
        GuidanceRegion.Reset();
        ResetModeDiagnostics();
        ResetModeHooks();
    }

    #region Reads for the shell

    internal static bool RetargetArmed => _retargetArmed;

    /// <summary>PICK SITE in a picture header: arms a click on the map or the world that moves the landing site. A stage target keeps its own arming in its row.</summary>
    internal static void TogglePickSite()
    {
        bool armedForSite = _retargetArmed && _retargetStageId == 0;
        _retargetStageId = 0;
        _retargetArmed = !armedForSite;
    }

    internal static bool PickSiteArmed => _retargetArmed && _retargetStageId == 0;

    /// <summary>
    /// The solver of LAND > LAND NOW: entry guidance where the page set it for the situation of the craft (ReadLandForm), else touchdown guidance. A live G-FOLD descent is the landing machine's, so the page follows what flies. Touchdown guidance does not take a live descent over from this page, so TOUCHDOWN is held while one flies.
    /// </summary>
    internal static LandSolver LandSolverRoute
        => LandingMachineLive && !_s.UseThreeDofLanding ? LandSolver.Gfold : LandSolvers.Shown(LandSolvers.Chosen(_s.UseThreeDofLanding, _s.UseSixDofLanding), false);

    private static LandSolver LandDrawnSolver
        => LandSolvers.SolverFor(LandSolvers.Live(_s.Active || _s.EngagePending, ThreeDofLive, _s.LandingPhase == LandingPhase.TerminalHover, LandingMachineLive, BoostbackLive)) is LandSolver live
            ? live
            : LandSolverRoute;

    internal static bool NoStageHome => _s.ReturnableStages.Count == 0;

    /// <summary>The running ascent reached its terminal cutoff, as opposed to an abort.</summary>
    internal static bool AscentCutoffDone => _s.CutoffDone;

    internal static string BoostbackCommitHintText() => BoostbackCommitHint();

    /// <summary>
    /// The state of the plan, which is the answer row of the launch verb. A calculation that runs says so as something AFC does now, and the picture shows how far it is. A calculation that found no plan says why, and so does a plan that cannot fly, because the verb then calculates first. Only a launch that can go now says ready.
    /// </summary>
    internal static string LaunchStatusText(Vehicle vehicle, out bool pending)
    {
        pending = false;
        Orbit orbit = vehicle?.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return "";
        if (_s.AscentPlanJob != null || _s.AscentPlanRequested)
        {
            pending = true;
            return PageText.PlanCalculating;
        }
        AscentPlan plan = _s.AscentPlan;
        return PageText.LaunchAnswer(LaunchPlanWhy(vehicle, orbit, parent), _s.AscentPlanStatus, plan?.Usable == true, _s.FlyConvexAscent,
            plan?.Solution?.FinalMass ?? double.NaN, plan?.StagesPlanned ?? 0, plan?.StagesAvailable ?? 0);
    }

    private static string LaunchPlanWhy(Vehicle vehicle, Orbit orbit, IParentBody parent)
        => _s.FlyConvexAscent && !_s.Running && !ConvexPlanFlyable(vehicle, orbit, parent, ExecuteLaunchInstant(), out string why) ? why : "";

    internal static bool LaunchWaits(out bool atWindow)
    {
        atWindow = _s.ConvexLaunchAtWindow;
        return _s.ConvexLaunchPending;
    }

    internal static void CancelLaunchPlan(Vehicle vehicle)
    {
        Use(vehicle);
        _s.AscentPlanJob?.Cancel();
        _s.AscentPlanRequested = false;
        _s.ConvexLaunchPending = false;
    }

    private static readonly ConditionalWeakTable<VehicleAutopilotState, StrongBox<FlownAscent>> _flownAscents = new();

    private static readonly ConditionalWeakTable<VehicleAutopilotState, StrongBox<double>> _climbEnds = new();

    private static double ClimbEndTime() => _climbEnds.TryGetValue(_s, out StrongBox<double> end) ? end.Value : double.PositiveInfinity;

    /// <summary>The ascent of the bound craft ended in this frame, so the orbit it left and its goal are kept for <see cref="FlownLine"/>. A stop on the pad or in a fall leaves no orbit to keep, and a flight by hand after it is no ascent of AFC. Only a cutoff ends the climb on the picture, because the craft can climb on by hand after a stop.</summary>
    internal static void NoteAscentEnded(in SituationFacts f)
    {
        if (_s.CutoffDone)
            _climbEnds.AddOrUpdate(_s, new StrongBox<double>(SimNow()));
        if (f.Where is CraftSituation.Orbit or CraftSituation.CoastingUp)
            _flownAscents.AddOrUpdate(_s, new StrongBox<FlownAscent>(new FlownAscent(f.Body, f.PeKm, f.ApKm, _s.PeKm, _s.ApKm)));
    }

    internal static void ForgetFlownAscent()
    {
        _flownAscents.Remove(_s);
        _climbEnds.Remove(_s);
    }

    internal static string FlownLine(in SituationFacts f)
        => _flownAscents.TryGetValue(_s, out StrongBox<FlownAscent> flown) ? flown.Value.Line(in f) : "";

    /// <summary>The first word of <see cref="FlownLine"/> while the ascent reached its goal, so that word takes the good colour, or empty.</summary>
    internal static string FlownGoodWord()
        => _flownAscents.TryGetValue(_s, out StrongBox<FlownAscent> flown) && PageText.GoalReached(flown.Value.PeKm, flown.Value.ApKm, flown.Value.GoalPeKm, flown.Value.GoalApKm) ? PageText.FlownWord : "";

    /// <summary>Whether CIRCULARIZE is the next step after the ascent the console just saw end (NextStep.CircularizeFits). With no record of it the orbit decides alone.</summary>
    internal static bool CircularizeFitsFlown()
        => !_flownAscents.TryGetValue(_s, out StrongBox<FlownAscent> flown) || NextStep.CircularizeFits(flown.Value.PeKm, flown.Value.ApKm, flown.Value.GoalPeKm, flown.Value.GoalApKm);

    internal static bool GoalOrbitHolds(in SituationFacts f)
        => _flownAscents.TryGetValue(_s, out StrongBox<FlownAscent> flown)
            && !NextStep.CircularizeFits(flown.Value.PeKm, flown.Value.ApKm, flown.Value.GoalPeKm, flown.Value.GoalApKm)
            && PageText.GoalReached(f.PeKm, f.ApKm, flown.Value.GoalPeKm, flown.Value.GoalApKm);

    /// <summary>This burn is the stock burn of the deorbit of the bound craft, so a delete of it ends the landing (StepStockDeorbit).</summary>
    internal static bool IsDeorbitBurn(Burn burn) => _s.DeorbitNodeClaimed && ReferenceEquals(_s.DeorbitNode, burn);

    internal static bool BelowBrakingStart(Vehicle vehicle)
    {
        Orbit orbit = vehicle?.Orbit;
        IParentBody parent = orbit?.Parent;
        return parent != null && orbit.StateVectors.PositionCci.Length() - parent.MeanRadius < _s.BrakingAltitudeKm * 1000.0;
    }

    // The worst fact the LAND form found while it drew this frame, for its footer. A danger replaces a warning, and the first of each kind stays.
    private static string _landPageFact = "";
    private static bool _landFactIsDanger;
    private static bool _landFactCalm;
    private static ConsolePage? _landFactNext;
    private static LandSolver? _landFactSolver;

    private static void ClearLandFact()
    {
        _landPageFact = "";
        _landFactIsDanger = false;
        _landFactCalm = false;
        _landFactNext = null;
        _landFactSolver = null;
        _landHoverFlies = true;
        _landHoverKnown = false;
    }

    private static void NoteLandFact(string fact, bool danger, ConsolePage? next = null, LandSolver? nextSolver = null, bool calm = false)
    {
        if (_landPageFact.Length > 0 && (_landFactIsDanger || !danger))
            return;
        _landPageFact = fact;
        _landFactIsDanger = danger;
        _landFactNext = next;
        _landFactSolver = nextSolver;
        _landFactCalm = calm;
    }

    /// <summary>
    /// The LAUNCH footer note that guidance will arm AutoStage. AutoSequence arms it only while guidance
    /// may steer and stage, with the AutoStage feature loaded, and only when the craft is not armed yet.
    /// </summary>
    internal static string LaunchFlownByNote(Vehicle vehicle)
        => vehicle != null && _s.Engage && _s.AutoStage && SharedVehicleHooks.AutoStageEnabled
           && !AutoStage.StagingDetector.IsArmed(vehicle)
            ? ". AutoStage arms"
            : "";

    /// <summary>The live values the footer outcome of a guidance verb names, read from the bound craft. The target is the craft's stock target, which ARM waits for.</summary>
    internal static PageText.OutcomeFacts CommitOutcomeFacts(Vehicle vehicle)
        => new(
            AutoStageArms: LaunchFlownByNote(vehicle).Length > 0,
            Target: _s.TargetId.Length > 0 ? _s.TargetId : _s.PlaneTarget?.Label ?? "",
            Solver: LandSolvers.Label(LandSolvers.Shown(LandSolvers.Chosen(false, _s.UseSixDofLanding), LandingMachineLive)),
            HandToTouchdown: _s.ThreeDofSixDofHandover, TouchdownLastS: _s.ThreeDofSixDofTimeS,
            HoverFromM: _s.GfoldHoverHandoffAltM,
            MayFly: _s.Engage && _s.AutoStage,
            HoverFlies: _landHoverFlies,
            HoverKnown: _landHoverKnown);

    #endregion

    #region Head warp offer

    /// <summary>Guidance asks for a time warp: to a launch window or a planned burn, or to the deorbit burn stock AUTO flies.</summary>
    internal static bool HeadWarpPending => (_warpPromptActive && !Universe.IsAutoWarpActive) || StockDeorbitWarpOffered;

    private static bool StockDeorbitWarpOffered
        => _s.LandingPhase == LandingPhase.DeorbitBurn && _s.DeorbitAutoArmed && !Universe.IsAutoWarpActive
            && (_s.DeorbitTarget == null || _s.DeorbitTarget.IgnitionTime.Seconds() > SimNow());

    internal static float HeadWarpWidth()
    {
        float gap = ImGui.GetStyle().ItemSpacing.X;
        float w = ConsoleUi.OwnerChipWidth(Owner.Stock, "STOCK") + ConsoleUi.ChipToButtonGap + ConsoleWidgets.ButtonWidth("WARP".AsSpan());
        if (!_warpPromptActive)
            return w;
        w += gap + ConsoleWidgets.ButtonWidth("1X".AsSpan());
        return _warpLabel == LaunchWindowWarpLabel && !_s.Running ? w + gap + ConsoleWidgets.ButtonWidth("NOW".AsSpan()) : w;
    }

    private static bool HeadWarpButton(string id, bool lit)
        => lit ? ConsoleWidgets.PrimaryButton("WARP".AsSpan(), id.AsSpan(), default) : ConsoleWidgets.Button("WARP".AsSpan(), id.AsSpan(), default);

    /// <summary>
    /// The one warp offer of the head, on one row: WARP, 1X and, for a launch window, NOW, then what it warps to and when, where <paramref name="room"/> has space for that. The deorbit burn that stock AUTO flies gets the same row, and its WARP sets the request the stock deorbit step answers (StepStockDeorbitWarp). With <paramref name="lit"/> WARP has the gold fill, because a wait of hours has no other action.
    /// </summary>
    internal static void DrawHeadWarp(Vehicle vehicle, float room, bool lit)
    {
        Use(vehicle);
        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;

        float2 origin = ImGui.GetCursorScreenPos();
        if (_warpPromptActive)
        {
            double wait = _warpTargetSimSec - SimNow();
            if (wait <= 1.0 || Universe.IsAutoWarpActive)
            {
                _warpPromptActive = false;
                return;
            }
            // The buttons drive the stock time warp, so they stand after a STOCK chip.
            ConsoleUi.OwnerChip(Owner.Stock, "STOCK");
            ImGui.SameLine(0f, ConsoleUi.ChipToButtonGap);
            string target = "TO " + _warpLabel + ", T-" + FormatHelper.FormatDuration(wait);
            if (HeadWarpButton("AfcHeadWarp", lit))
            {
                // The draw overlaps the vehicle solver, so the warp goes through the input buffer that Program.PrepareFrame drains with the solvers joined, as stock's own warp does.
                InputEvents.AutoWarpBuffer.Add(new InputEvents.AutoWarpData
                {
                    StopWarp = false,
                    WarpToTime = Universe.GetElapsedTime() + wait,
                });
                _warpPromptActive = false;
            }
            if (ImGui.IsItemHovered())
                ConsoleUi.Tooltip(target.AsSpan());
            ImGui.SameLine();
            if (ConsoleWidgets.Button("1X".AsSpan(), "AfcHeadWarpStay".AsSpan(), default))
            {
                _warpPromptActive = false;
                _warpDeclinedLabel = _warpLabel;
            }
            if (ImGui.IsItemHovered())
                ConsoleUi.Tooltip("Holds 1x. No new prompt during this wait.".AsSpan());

            // Offered to warp to a launch window, the other answer is not to wait for it at all, for a window just missed whose next one is most of a revolution away (see LaunchNowAscent).
            if (_warpLabel == LaunchWindowWarpLabel && !_s.Running)
            {
                ImGui.SameLine();
                if (ConsoleWidgets.Button("NOW".AsSpan(), "AfcHeadLaunchNow".AsSpan(), default))
                {
                    ChaseStatus status = TryChaseOrbit(vehicle, orbit, parent, parent.MeanRadius, out ChasePlan plan);
                    if (status == ChaseStatus.Ok || status == ChaseStatus.PlaneUnreachable)
                        LaunchNowAscent(vehicle, orbit, parent, in plan);
                }
                if (ImGui.IsItemHovered())
                    ConsoleUi.Tooltip((!_s.FlyConvexAscent || _s.BackupAscent ? PageText.LaunchNowGravityTurnTip : PageText.LaunchNowTip).AsSpan());
            }
            DrawWarpLine(origin.X + room, target);
            return;
        }

        if (StockDeorbitWarpOffered)
        {
            ConsoleUi.OwnerChip(Owner.Stock, "STOCK");
            ImGui.SameLine(0f, ConsoleUi.ChipToButtonGap);
            if (HeadWarpButton("AfcHeadDeorbitWarp", lit))
                _s.DeorbitWarpRequested = true;
            if (ImGui.IsItemHovered())
                ConsoleUi.Tooltip("Warps to the STOCK AUTO turn before the deorbit burn. AFC lands after the burn.".AsSpan());
            DrawWarpLine(origin.X + room, "TO DEORBIT BURN");
        }
    }

    private static void DrawWarpLine(float right, string text)
    {
        ImGui.SameLine();
        float2 at = ImGui.GetCursorScreenPos();
        ConsoleStyle.PushValueFont();
        if (ImGui.CalcTextSize(text).X <= right - at.X)
            ConsoleUi.FitText(at, right - at.X, ConsoleStyle.FrameHeightPx, text, ConsoleStyle.ApplyAlpha(ConsoleStyle.TextLabelU32));
        ConsoleStyle.PopFont();
    }

    #endregion

    #region Status lines and cues

    /// <summary>Every status source of the bound craft, in the order the selector breaks ties by.</summary>
    internal static void BuildStatusLines(Vehicle vehicle, List<StatusLine> lines)
    {
        if (OwnerThreadViolation.Length > 0)
            lines.Add(new StatusLine(Severity.Error, "THREADING: " + OwnerThreadViolation, ""));
        if (StepFaultLasts)
            lines.Add(new StatusLine(Severity.Error, "ERROR: " + _s.GuidanceError, ""));
        if (_s.ReleaseError.Length > 0)
            lines.Add(new StatusLine(Severity.Error, _s.ReleaseError, ""));
        string failure = ReleaseFailure(vehicle);
        if (failure.Length > 0 && failure != _s.ReleaseError)
            lines.Add(new StatusLine(Severity.Error, failure, ""));
        if (_s.ConvexLaunchFailure.Length > 0)
            lines.Add(new StatusLine(Severity.Warning, _s.ConvexLaunchFailure, ConsolePages.Key(ConsolePage.Launch)));
        AddStatusLine(lines, _s.BoostbackSeverity, _s.BoostbackStatus);
        AddStatusLine(lines, _s.LandingSeverity, _s.LandingStatus);
        AddEntryStatusLines(lines);
        AddStatusLine(lines, _s.ErrorSeverity, _s.Error);
        AddStatusLine(lines, NoThrustLasts() ? Severity.Warning : _s.StatusSeverity, _s.Status);
    }

    // No thrust is normal for a moment at a staging, so the step writes it as a plain note. When it lasts, such as after the last stage, the pilot must see it.
    private static bool NoThrustLasts()
        => _lastingProblems.GetValue(_s, _ => new LastingProblems()).NoThrust.Lasts(_s.Status == NoThrustStatus ? NoThrustStatus : "", SimNow(), GuidanceClock.NowMs);

    private static void AddStatusLine(List<StatusLine> lines, Severity severity, string text)
    {
        if (text.Length > 0 && severity != Severity.Diag)
            lines.Add(new StatusLine(severity, text, ""));
    }

    /// <summary>
    /// Turns the focus requests the landing modes raise into cues, and clears them. The console never changes page by itself; the cue offers a link instead. The flags stay on the state, so no step code changes.
    /// </summary>
    internal static void ConsumeCues(List<StatusLine> cues)
    {
        FollowLiveLanding(cues);
        if (_s.GfoldCuePending)
        {
            cues.Add(new StatusLine(Severity.Cue, "AFC: POWERED DESCENT",
                ConsolePages.CueTarget(ConsolePage.Land, ConsolePages.EnumKey(LandWay.FromHere))));
            _s.GfoldCuePending = false;
        }
        if (_s.TermCuePending)
        {
            cues.Add(new StatusLine(Severity.Cue, "AFC: HOVER",
                ConsolePages.CueTarget(ConsolePage.Land, ConsolePages.EnumKey(LandWay.Hover))));
            _s.TermCuePending = false;
        }
    }

    // The landing mode that flew each craft when the console last read it. Per craft, so a switch of the craft on screen is not read as a hand-over.
    private static readonly ConditionalWeakTable<VehicleAutopilotState, StrongBox<LiveLanding>> _lastLiveLanding = new();

    /// <summary>
    /// On a change of the mode that flies the landing (boostback, entry guidance, touchdown guidance, G-FOLD, hover), sets the solver choice to that mode, so the LAND page, its commit and its stop act on what flies, and raises a cue for the modes that raise none of their own. A hand-over from entry guidance to touchdown guidance is the case that needs it most: without it the page would still show entry guidance.
    /// </summary>
    private static void FollowLiveLanding(List<StatusLine> cues)
    {
        // EngagePending counts as touchdown guidance, because the hand-over from entry guidance (ThreeDofToSixDof) sets only that flag and Active follows on the next step. Without it the console reads a frame with no landing mode between the two, and the cue of the hand-over is lost.
        LiveLanding live = LandSolvers.Live(_s.Active || _s.EngagePending, ThreeDofLive,
            _s.LandingPhase == LandingPhase.TerminalHover, LandingMachineLive, BoostbackLive);
        StrongBox<LiveLanding> last = _lastLiveLanding.GetValue(_s, _ => new StrongBox<LiveLanding>(LiveLanding.None));
        if (live == last.Value)
            return;
        if (LandSolvers.SolverFor(live) is LandSolver solver)
            (_s.UseThreeDofLanding, _s.UseSixDofLanding) = LandSolvers.Apply(solver, _s.UseSixDofLanding);
        if (LandWays.Follow(last.Value, live) is LandWay way)
            _followedLandWay = way;
        if (LandSolvers.CueFor(last.Value, live) is StatusLine cue)
            cues.Add(cue);
        last.Value = live;
    }

    // The LAND way that matches a hand-over of the landing, for the shell to read once. UI state only.
    private static LandWay? _followedLandWay;

    /// <summary>The way LAND follows after a hand-over of the landing (LandWays.Follow), once, or null.</summary>
    internal static LandWay? ConsumeFollowedLandWay()
    {
        LandWay? way = _followedLandWay;
        _followedLandWay = null;
        return way;
    }

    #endregion

    #region Failure band

    private sealed class LastingProblem
    {
        private string _what = "";
        private double _since = double.NaN;
        private long _sinceRealMs;

        /// <summary>The last call of <see cref="Lasts"/> said yes.</summary>
        public bool Holds { get; private set; }

        public bool Lasts(string what, double now, long realMs)
        {
            if (what != _what)
            {
                _what = what;
                _since = now;
                _sinceRealMs = realMs;
            }
            Holds = what.Length > 0 && StatusSelector.NoPlanSustained(now - _since, (realMs - _sinceRealMs) / 1000.0);
            return Holds;
        }
    }

    private sealed class LastingProblems
    {
        public readonly LastingProblem Release = new(), NoPlan = new(), NoThrust = new();
    }

    private static readonly ConditionalWeakTable<VehicleAutopilotState, LastingProblems> _lastingProblems = new();

    private static bool NoPlanHolds(VehicleAutopilotState s) => _lastingProblems.TryGetValue(s, out LastingProblems problems) && problems.NoPlan.Holds;

    private static bool StepFaultLasts => StatusSelector.StepFaultLasts(
        _s.Running || _s.LandingPhase is LandingPhase.Prep or LandingPhase.Burn, _s.GuidanceError, _s.FailStreak);

    internal static void ReadFailures(List<CraftFailure> failures)
    {
        if (_s.FailureWhat.Length > 0)
            failures.Add(new CraftFailure(_s.FailureWhat, _s.FailureDetail, _s.FailureDetail, _s.FailureSerial));
        if (StepFaultLasts)
            failures.Add(new CraftFailure("GUIDANCE FAULT (REPEATED)", _s.GuidanceError));

        LastingProblems problems = _lastingProblems.GetValue(_s, _ => new LastingProblems());
        double now = SimNow();
        long realMs = GuidanceClock.NowMs;
        string release = _s.ReleaseError.Length > 0 ? "RELEASE FAILED" : "";
        if (problems.Release.Lasts(release, now, realMs))
            failures.Add(new CraftFailure(release, _s.ReleaseError, _s.ReleaseError));

        // Step3Dof never plans on a body with no air, so this one needs no wait.
        bool noAir = EntryHasNoAir(_s);
        if (noAir)
            failures.Add(new CraftFailure("NO ENTRY PLAN", EntryGuidance.NoAirText));
        string detail = "";
        string noPlan = noAir ? "" : NoPlanNow(out detail);
        if (problems.NoPlan.Lasts(noPlan, now, realMs))
            failures.Add(new CraftFailure(noPlan, detail));
    }

    // A state that is normal when it is short, because the first plan takes a few seconds.
    private static string NoPlanNow(out string detail)
    {
        detail = "";
        if (ThreeDofLive && _s.ThreeDofPlan == null)
        {
            detail = _s.ThreeDofStatus;
            return "NO ENTRY PLAN";
        }
        if (_s.Active && (_s.Guidance == null || !_s.Guidance.HasPlan))
        {
            detail = _s.Error;
            return "TOUCHDOWN: NO PLAN";
        }
        if (_s.LandingPhase == LandingPhase.GfoldDescent && _s.GfoldPlan == null)
        {
            detail = _s.LandingStatus;
            return "G-FOLD: NO PLAN";
        }
        if (_s.BoostbackPhase == BoostbackPhase.Rotation && !_s.BoostbackHasPlan && _s.BoostbackPlanError.Length > 0)
        {
            detail = _s.BoostbackPlanError;
            return "BOOSTBACK: NO PLAN";
        }
        return "";
    }

    /// <summary>What the engine of <paramref name="vehicle"/> does now. EngineOn is the master switch and IsAnyEngineActive only says that a stage is active, so a lit engine needs both, a throttle and propellant.</summary>
    internal static EngineNow EngineNowOf(Vehicle vehicle)
    {
        if (vehicle.FlightComputer?.BurnMode == FlightComputerBurnMode.Auto)
            return EngineNow.StockAuto;
        ref ManualControlInputs inputs = ref ManualInputs(vehicle);
        return inputs.EngineOn && inputs.EngineThrottle > 0f && vehicle.IsAnyEngineActive() && vehicle.IsAnyEnginePropellantAvailable()
            ? EngineNow.On
            : EngineNow.Off;
    }

    /// <summary>The height of the legs above the ground under <paramref name="vehicle"/>, the one height of the head, or NaN with no body to read.</summary>
    internal static double GroundHeightM(Vehicle vehicle)
    {
        Orbit orbit = vehicle?.Orbit;
        IParentBody parent = orbit?.Parent;
        return parent != null ? TerminalHeight(orbit, parent, parent.MeanRadius) : double.NaN;
    }

    /// <summary>The seconds to impact, or NaN when none is known. A glide that flies knows its time with the air, and the stock flight plan ends the patch that hits the ground with PatchTransition.Impact.</summary>
    internal static double ImpactSeconds(Vehicle vehicle, bool guidanceBound)
    {
        bool gliding = guidanceBound && (_s.BoostbackPhase == BoostbackPhase.Glide || (ThreeDofLive && _s.ThreeDofPlan == null));
        if (gliding && double.IsFinite(_s.GlideTgo))
            return _s.GlideTgo;
        foreach (PatchedConic patch in vehicle.FlightPlan.Patches)
        {
            if (patch.EndTransition == PatchTransition.Impact)
                return (patch.EndTime - Universe.GetElapsedTime()).Seconds();
        }
        return double.NaN;
    }

    #endregion

    #region Commit and stop

    internal static CommitGates CommitGatesFor(Vehicle vehicle, GuidanceModeId mode, in SituationFacts situation)
    {
        bool hoverPage = mode == GuidanceModeId.Hover;
        Orbit orbit = vehicle?.Orbit;
        IParentBody parent = orbit?.Parent;
        bool hasTarget = false;
        bool twrLow = false;
        bool plansFirst = false;
        string hoverProblem = "";
        string noSite = "";
        string farSite = "";
        if (parent != null)
        {
            if (NoSitePicked(parent))
                noSite = PageText.NoSiteAnswer(SituationVerbs.VerbName(mode), _s.SiteLatDeg, _s.SiteLonDeg);
            else if (mode is GuidanceModeId.SixDofHere or GuidanceModeId.GfoldHere)
            {
                double siteM = AngleBetween(orbit.StateVectors.PositionCci, SiteDirCciAt(parent, 0)) * parent.MeanRadius;
                double impact = ImpactSeconds(vehicle, guidanceBound: true);
                if (FormRules.SiteOutOfReach(siteM, impact, vehicle.GetSurfaceSpeed()))
                    farSite = PageText.SiteFar;
            }
            hasTarget = HasLaunchTarget
                && TryChaseOrbit(vehicle, orbit, parent, parent.MeanRadius, out _) == ChaseStatus.Ok;
            plansFirst = mode == GuidanceModeId.AscentUpfg && LaunchPlanWhy(vehicle, orbit, parent).Length > 0;
            // Only with a lit engine, because the TWR counts active engines and TerminalHoverAvailable also accepts a craft whose engine the next staging lights.
            if (hoverPage && KsaEnginePerf.SupportsThrottleControl(vehicle))
                twrLow = TerminalTwr(vehicle, orbit, parent.Mu) < 1.0;
            if (hoverPage)
                hoverProblem = TerminalHoverProblem(vehicle);
        }
        GuidanceModeDescriptor gravityTurn = GuidanceModes.Of(GuidanceModeId.AscentGravityTurn);
        string gravityTurnRefusal = gravityTurn == null ? CommitResolver.GravityTurnUnavailable
            : gravityTurn.Available ? ""
            : gravityTurn.UnavailableReason;
        return new CommitGates(
            HasTarget: hasTarget,
            LandingSwitchesOff: LandingSwitchesOff,
            DeorbitRefusal: LandingSwitchesOff ? "" : DeorbitSettingsRefusal(),
            GravityTurnRefusal: gravityTurnRefusal,
            HoverTwrBelowOne: twrLow,
            PlansFirst: plansFirst,
            BackupChosen: mode == GuidanceModeId.AscentUpfg && BackupLaunchChosen,
            Where: situation.Where,
            HasAir: situation.HasAir,
            BelowBrakingStart: BelowBrakingStart(vehicle),
            GfoldChosen: !_s.UseSixDofLanding,
            HoverProblem: hoverProblem,
            PageFact: mode is GuidanceModeId.SixDofHere or GuidanceModeId.GfoldHere or GuidanceModeId.DeorbitLand ? _landPageFact : "",
            NoSiteNote: noSite,
            FarSiteNote: farSite,
            PageFactNext: _landFactNext,
            PageFactSolver: _landFactSolver,
            PageFactCalm: _landFactCalm,
            NoStageHome: NoStageHome);
    }

    // The backup choice counts only while the failure it answered is still shown.
    private static bool BackupLaunchChosen => _s.ConvexBackupChosen && _s.ConvexLaunchFailure.Length > 0;

    /// <summary>
    /// Acts on a commit click. A commit that takes the craft from another live guidance mode asks first, in the footer (TakeOverAsk). The confirmed action runs on a later frame, so it checks again that guidance is still on and the mod still loaded, and that the craft is still the one on screen.
    /// </summary>
    internal static void Commit(Vehicle vehicle, GuidanceModeId mode, CommitView view, StopTarget running,
        GuidanceModeId? registeredRunning = null)
    {
        if (vehicle == null || !view.Enabled)
            return;

        if (!view.NeedsTakeOver)
        {
            Use(vehicle);
            RunCommit(vehicle, mode);
            return;
        }

        Vehicle craft = vehicle;
        TakeOverAsk.Raise(view.Verb, CommitResolver.ModeName(running, registeredRunning),
            view.EndsOnly ? CommitResolver.EndsOnlyMessage(running, registeredRunning) : CommitResolver.TakeOverMessage(running, view.Verb, registeredRunning), () =>
        {
            if (!SharedVehicleHooks.GuidanceEnabled || !ModActive)
                return;
            if (craft.IsDisposed || !ReferenceEquals(Program.ControlledVehicle, craft))
                return;
            Use(craft);
            RunCommit(craft, mode);
        }, DateTime.UtcNow);
    }

    // One guidance entry point per built-in mode. A mode that registered later commits through its hooks (RegisterMode). The gravity turn has neither yet: its descriptor is unavailable, so its commit stays disabled, and a mode that becomes available without an entry point is refused with a note.
    private static void RunCommit(Vehicle vehicle, GuidanceModeId mode)
    {
        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;

        _s.ClearStatusLines();

        bool engage = _s.Engage;
        bool autoStage = _s.AutoStage;
        switch (mode)
        {
            case GuidanceModeId.AscentUpfg:
                // The launch window step keeps the chase orbit current, at most 250 ms behind. The commit takes the chase as it stands now, so a target picked just before the click is the one the launch flies.
                RefreshMoonLaunchPlane(vehicle, parent);
                if (HasLaunchTarget)
                {
                    ChaseStatus chase = TryChaseOrbit(vehicle, orbit, parent, parent.MeanRadius, out ChasePlan plan);
                    _s.ChaseDriven = chase == ChaseStatus.Ok || chase == ChaseStatus.PlaneUnreachable;
                    if (_s.ChaseDriven)
                        ApplyChaseOrbit(in plan);
                }
                if (BackupLaunchChosen)
                    LaunchBackupAscent(vehicle, orbit, parent);
                else
                    ExecuteAscent(vehicle, orbit, parent);
                break;
            case GuidanceModeId.Boostback:
                ExecuteBoostback(vehicle, orbit, parent);
                break;
            case GuidanceModeId.DeorbitLand:
                ExecuteLanding(vehicle);
                break;
            case GuidanceModeId.GfoldHere:
                StartGfoldNow(vehicle);
                break;
            case GuidanceModeId.SixDofHere:
                Engage6Dof(vehicle);
                break;
            case GuidanceModeId.Hover:
                StartTerminalHover(vehicle);
                break;
            default:
                if (HooksOf(mode) is GuidanceModeHooks hooks)
                {
                    hooks.Commit(vehicle, orbit, parent);
                    break;
                }
                // A mode that registered as available before its entry point is wired here would otherwise be a LAUNCH click that does nothing.
                PendingCommitNote = (GuidanceModes.Of(mode)?.MethodLabel ?? mode.ToString().ToUpperInvariant())
                    + " HAS NO ENTRY POINT. NOT STARTED.";
                GuidanceLog.Info(vehicle, $"commit for {mode} refused: no entry point in RunCommit and no registered hooks.");
                return;
        }

        NoteRefusal(vehicle, mode);

        // StartGfoldNow and StartTerminalHover turn both switches on, because a powered descent that may not steer or throttle is not a descent. Said once, so the change is not silent.
        if ((!engage && _s.Engage) || (!autoStage && _s.AutoStage))
            PendingCommitNote = "ON FOR LANDING: " + GuidanceSettingSpecs.Engage.Label + " + "
                + GuidanceSettingSpecs.AutoStage.Label;
    }

    private static string _refusedLine = "";
    private static GuidanceModeId _refusedMode;

    private static void NoteRefusal(Vehicle vehicle, GuidanceModeId mode)
    {
        CraftActivity activity = ReadMode(vehicle, _s).Activity;
        _refusedMode = mode;
        _refusedLine = FlightView.ModeFlies(mode, in activity) ? ""
            : _s.LandingSeverity == Severity.Warning ? _s.LandingStatus
            : _s.BoostbackSeverity == Severity.Warning ? _s.BoostbackStatus
            : _s.StatusSeverity == Severity.Warning ? _s.Status
            : _s.ErrorSeverity == Severity.Warning ? _s.Error
            : "";
    }

    internal static string RefusedLine => _refusedLine;

    /// <summary>The answer row of <paramref name="mode"/> after a press that it refused, or empty. It stands as long as the line of that refusal stands (StatusSelector.ShowsInHead).</summary>
    internal static string RefusedAnswer(GuidanceModeId mode, IReadOnlyList<StatusLine> lines, string fitText)
    {
        if (_refusedLine.Length == 0 || mode != _refusedMode)
            return "";
        for (int i = 0; i < lines.Count; i++)
        {
            if (lines[i].Text == _refusedLine)
                return PageText.RefusedAnswer(fitText, _refusedLine);
        }
        return "";
    }

    /// <summary>The guidance part of the stop dispatch: the abort entry point of each mode.</summary>
    internal static void StopGuidance(Vehicle vehicle, StopTarget target)
    {
        if (vehicle == null)
            return;
        Use(vehicle);
        switch (target)
        {
            case StopTarget.SixDof:
                Disengage6Dof(vehicle);
                break;
            case StopTarget.Landing:
            case StopTarget.Hover:
            {
                bool stockDeorbit = IsStockDeorbitPhase(_s.LandingPhase) && _s.DeorbitNodeClaimed;
                bool engineOff = Ui.Console.ConsoleFrame.EngineKnownOff(vehicle);
                bool burnDeleted = StockDeorbitNodeUnchanged;
                AbortLanding();
                if (stockDeorbit)
                    _s.LandingStatus = PageText.LandingStopped(engineOff, burnDeleted);
                break;
            }
            case StopTarget.Boostback:
                AbortBoostback();
                break;
            case StopTarget.Ascent:
            case StopTarget.LaunchArmed:
                AbortAscent(vehicle);
                break;
            case StopTarget.Registered:
                LiveRegisteredMode(_s)?.Stop(vehicle);
                break;
        }
    }

    /// <summary>A stop for a craft that is not on screen, applied at the start of that craft's next guidance step.</summary>
    internal static void RequestStop(Vehicle vehicle, StopTarget target)
    {
        if (vehicle == null || !StopResolver.IsGuidance(target))
            return;
        _stopRequests.RemoveAll(r => ReferenceEquals(r.Vehicle, vehicle));
        _stopRequests.Add((vehicle, target));
    }

    /// <summary>Applies the stop requests of <paramref name="vehicle"/>. Called from its guidance step on the main thread, before the mode steps.</summary>
    internal static void DrainRequests(Vehicle vehicle)
    {
        for (int i = _stopRequests.Count - 1; i >= 0; i--)
        {
            if (!ReferenceEquals(_stopRequests[i].Vehicle, vehicle))
                continue;
            StopTarget target = _stopRequests[i].Target;
            _stopRequests.RemoveAt(i);
            StopGuidance(vehicle, target);
        }
    }

    internal static void ReleaseGuidance(Vehicle vehicle)
    {
        if (vehicle == null)
            return;
        Use(vehicle);
        // An entry guidance request waits for the next step, which the release would not stop.
        _s.ThreeDofEngagePending = false;
        ResetFlightComputer();
    }

    #endregion

    /// <summary>Overlays and the retarget click, after the console window has ended.</summary>
    internal static void DrawTrailing(IGameViewport viewport, Vehicle vehicle)
    {
        if (vehicle?.Orbit?.Parent == null)
            return;
        DrawTrailingWindows(viewport, vehicle);
    }

    #region Folds

    /// <summary>Draws <paramref name="draw"/> in the Expert scope. A row of that tier draws in no other scope, so a part of a fold holds its Expert rows only through this.</summary>
    private static Action InExpert(Action draw) => () =>
    {
        using (TierScope.Push(UiTier.Expert))
            draw();
    };

    /// <summary>The first part of a GNC fold with a changed value: a reset of the changed values of the fold, which asks first. It stays out of the label, because the label counts the values.</summary>
    private static FoldPart ResetPart(string id, Vehicle vehicle, SettingSpec[] doubles, BoolSpec[] bools, int changed)
        => new("", changed > 0, () =>
        {
            string lockReason = ResetLockReason(doubles, bools);
            bool clicked;
            using (new ImGuiDisabledScope(lockReason.Length > 0))
                clicked = ConsoleUi.ButtonRight(PageText.ResetFold, "AfcResetFold" + id);
            if (lockReason.Length > 0 && ImGui.IsItemHovered(ImGuiHoveredFlags.AllowWhenDisabled))
                ConsoleUi.Tooltip(lockReason.AsSpan());
            if (clicked && lockReason.Length == 0)
                ResetPopup.Create(ResetFor(vehicle, doubles, bools), PageText.ResetFold, PageText.ResetFoldQuestion(changed));
        });

    /// <summary>
    /// A reset action for the confirm popup, which runs it on a later frame. So it binds the craft again, does nothing when guidance went off or the mod unloaded in the meantime, and leaves alone a value that got locked since the click.
    /// </summary>
    private static Action ResetFor(Vehicle vehicle, SettingSpec[] doubles, BoolSpec[] bools)
        => () =>
        {
            if (!SharedVehicleHooks.GuidanceEnabled || !ModActive)
                return;
            if (vehicle == null || vehicle.IsDisposed || !VehicleAutopilotState.TryGet(vehicle, out VehicleAutopilotState state))
                return;
            Use(vehicle);
            SettingAccess.ResetToDefaults(state, doubles, bools, SettingLocked);
        };

    #endregion

    #region Reset locks

    // The values a committed deorbit reads (CurrentDeorbitSettings, EffectiveGLimit). The spec rows of these keys are disabled while it flies (GaugeRow and its kin ask SettingLockReason), because a changed value makes the deorbit refuse itself (DeorbitRequestCurrent), and a reset leaves them alone for the same reason.
    private static readonly HashSet<string> DeorbitLockedKeys = new(StringComparer.Ordinal)
    {
        Specs.SiteLatDeg.Key, Specs.SiteLonDeg.Key, Specs.BrakingAltitudeKm.Key, Specs.DeorbitArrivalAngleEnabled.Key,
        Specs.DeorbitArrivalDescentDeg.Key, Specs.DownrangeFactor.Key, Specs.AimAltKm.Key, Specs.DescentRate.Key,
        Specs.GateUprangeKm.Key, Specs.GfoldHandoffTgo.Key, Specs.LandingVerticalGateM.Key,
        Specs.VehicleHeightM.Key, Specs.VehicleHeightAuto.Key, Specs.GLimitEnabled.Key, Specs.GLimitG.Key,
    };

    // The limits and the orbit of the ascent problem, greyed while it is solved (DrawLimitsFold, DrawConvexAscentSection, DrawOrbitRow, DrawPlaneRow), because a plan solved with other values no longer matches the launch.
    private static readonly HashSet<string> AscentPlanLockedKeys = new(StringComparer.Ordinal)
    {
        Specs.ConvexQMaxKpa.Key, Specs.ConvexQAlphaMax.Key, Specs.ConvexThrottleMinPct.Key,
        Specs.PeKm.Key, Specs.ApKm.Key, Specs.IncDeg.Key, Specs.LanDeg.Key,
    };

    private static string SettingLockReason(string key)
    {
        if (DeorbitTargetLocked && DeorbitLockedKeys.Contains(key))
            return PageText.DeorbitLockedSetting;
        if ((_s.AscentPlanJob != null || _s.AscentPlanRequested) && AscentPlanLockedKeys.Contains(key))
            return "LOCKED: ASCENT PLAN SOLVING.";
        return "";
    }

    private static bool SettingLocked(string key) => SettingLockReason(key).Length > 0;

    /// <summary>Why a reset of these values cannot run now: the lock of a changed value it would write, or empty when it can run.</summary>
    private static string ResetLockReason(SettingSpec[] doubles, BoolSpec[] bools)
    {
        if (doubles != null)
            foreach (SettingSpec spec in doubles)
            {
                string reason = SettingLockReason(spec.Key);
                if (reason.Length > 0 && SettingAccess.TryGetDouble(_s, spec.Key, out double value) && SettingMath.IsChanged(spec, value))
                    return reason;
            }
        if (bools != null)
            foreach (BoolSpec spec in bools)
            {
                string reason = SettingLockReason(spec.Key);
                if (reason.Length > 0 && SettingAccess.TryGetBool(_s, spec.Key, out bool value) && SettingMath.IsChanged(spec, value))
                    return reason;
            }
        return "";
    }

    #endregion
}
