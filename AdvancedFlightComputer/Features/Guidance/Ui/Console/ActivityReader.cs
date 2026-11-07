#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.Features.Ui;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Numerics;
using KSA;

/// <summary>How the console colours a mode line.</summary>
internal enum ModeTone { Idle, Flying, Waiting, Error }

/// <summary>What the console head and the drawer rows say about one craft. <see cref="Numbers"/> are the values of the numbers row, which holds what fits (HeadNumbers). <see cref="NumbersCard"/> is the card of the numbers, with what is no headline. <see cref="NumbersHoldOrbit"/> says that the numbers name apoapsis and periapsis, so the situation line does not, and <see cref="HeightShown"/> and <see cref="ImpactShown"/> say the same of a fall. <see cref="PhaseList"/> is the chain of a mode whose phases depend on what it flies, or null for the phases of its descriptor. <see cref="Word"/> is the status word, where a value says one. <see cref="PhaseBesideName"/> says that the phase is one plain fact with no chain, so it stands beside the name and the head has no row for it.</summary>
internal readonly record struct ModeReadout(CraftActivity Activity, string ModeName, string Method, string Phase,
    ModeTone Tone, double TgoSeconds, string TgoLabel, IReadOnlyList<HeadValue> Numbers, string NumbersCard = "", bool NumbersHoldOrbit = false, IReadOnlyList<string> PhaseList = null,
    PlanWord Word = default, bool HeightShown = false, bool ImpactShown = false, bool PhaseBesideName = false);

public static partial class GuidanceWindow
{
    /// <summary>
    /// Reads what AFC does on <paramref name="vehicle"/> from that craft's own guidance state, so it works for any loaded craft and not only for the one the console is bound to. A craft without guidance state reports only its multi-pass and RCS activity.
    /// </summary>
    internal static ModeReadout ReadMode(Vehicle vehicle)
    {
        VehicleAutopilotState s = null;
        if (vehicle != null && SharedVehicleHooks.GuidanceEnabled)
            VehicleAutopilotState.TryGet(vehicle, out s);
        return ReadMode(vehicle, s);
    }

    internal static ModeReadout ReadMode(Vehicle vehicle, VehicleAutopilotState s)
    {
        bool multiPass = vehicle != null && SharedVehicleHooks.MultiPassEnabled && MultiPassRegistry.Has(vehicle.Id);
        bool rcs = vehicle != null && VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.RcsTranslation;
        // BurnModeInput.StopAuto acts only on the controlled craft, so only that craft offers the stop of stock AUTO.
        bool stockAuto = vehicle != null && ReferenceEquals(vehicle, Program.ControlledVehicle) && BurnModeInput.StockAutoOnNode(vehicle);

        if (s == null)
        {
            var bare = new CraftActivity { MultiPassActive = multiPass, RcsFlying = rcs, StockAutoOnNode = stockAuto };
            return OtherModes(vehicle, bare);
        }

        bool machine = s.LandingPhase == LandingPhase.GfoldDescent
            || s.LandingPhase == LandingPhase.TerminalCoast || s.LandingPhase == LandingPhase.TerminalBrake;
        // Entry guidance is a registered mode, so its live check and its phase-dependent stop texts come from its hooks.
        GuidanceModeHooks registered = LiveRegisteredMode(s);
        (string stopOutcome, string stopDetail) = RegisteredStopText(registered, s);
        var activity = new CraftActivity
        {
            AscentRunning = s.Running,
            LaunchArmed = s.LaunchArmed,
            ConvexLaunchPending = s.ConvexLaunchPending,
            BoostbackLive = s.BoostbackPhase != BoostbackPhase.Idle && s.BoostbackPhase != BoostbackPhase.Done,
            DescentLive = s.LandingPhase != LandingPhase.Idle && s.LandingPhase != LandingPhase.Done,
            LandingMachineLive = machine,
            TerminalBurnActive = s.LandingPhase == LandingPhase.TerminalCoast || s.LandingPhase == LandingPhase.TerminalBrake,
            HoverLive = s.LandingPhase == LandingPhase.TerminalHover,
            SixDofEngaged = s.Active || s.EngagePending,
            LandingAirborne = machine || s.LandingPhase == LandingPhase.TerminalHover,
            ControlAcquired = s.ControlAcquired,
            HoldsNoControl = !s.ControlAcquired && !s.Active && !s.EngagePending
                && (!s.Engage || (!s.Running && (s.LaunchArmed || s.ConvexLaunchPending || LandingWaits(s.LandingPhase)))),
            EngineOff =Ui.Console.ConsoleFrame.EngineKnownOff(vehicle),
            EnginePercent = Ui.Console.ConsoleFrame.EnginePercent(vehicle),
            LandingStockAuto = s.DeorbitNodeClaimed && vehicle?.FlightComputer?.BurnMode == FlightComputerBurnMode.Auto,
            MultiPassActive = multiPass,
            RcsFlying = rcs,
            StockAutoOnNode = stockAuto,
            ReleaseWouldAct = HoldsReleasableState(s) || s.ArmedStaging,
            RegisteredModeLive = registered?.Id,
            RegisteredStopOutcome = stopOutcome,
            RegisteredStopDetail = stopDetail,
        };

        // The phase labels read the ambient state, so it points at this craft's state while they run and goes back afterwards. Nothing here writes the state.
        VehicleAutopilotState previous = _s;
        _s = s;
        try
        {
            return DescribeGuidance(vehicle, activity);
        }
        finally
        {
            _s = previous;
        }
    }

    private static ModeReadout DescribeGuidance(Vehicle vehicle, CraftActivity a)
    {
        bool error = StepFaultLasts || _s.ReleaseError.Length > 0;
        ModeTone Tone(bool flying) => error ? ModeTone.Error : flying ? ModeTone.Flying : ModeTone.Waiting;

        switch (StopResolver.Resolve(in a))
        {
            case StopTarget.SixDof:
            {
                string phase = !_s.Active ? "START" : _s.Converging ? "PLANNING" : "TOUCHDOWN";
                // With a plan the time is the end of the plan. Without one it is the fall the stock flight plan sees.
                Ksa6DofGuidance g = _s.Guidance;
                bool hasPlan = g != null && g.HasPlan;
                var plan = hasPlan ? g.Published : null;
                double planLeft = plan != null ? Math.Max(0.0, plan.Sigma - (SimNow() - plan.SolveTime)) : double.NaN;
                bool planned = double.IsFinite(planLeft);
                PlanWord word = !_s.Active ? default
                    : _s.Converging ? PlanWords.Planning
                    : hasPlan && _s.SixDofTrack ? PlanWords.Tracking(g.TrackingError.PositionM)
                    : PlanWords.Of(hasPlan, NoPlanHolds(_s));
                return new ModeReadout(a, "TOUCHDOWN", "", phase,
                    Tone(_s.Active && !_s.Converging), planned ? planLeft : ImpactSeconds(vehicle, guidanceBound: false), planned ? PageText.LandsIn : "IMPACT T-",
                    LandingNumbers(vehicle, site: true), PageText.HoverNumbersCard, Word: word, HeightShown: true, ImpactShown: true);
            }
            case StopTarget.Landing:
            {
                string label = DescentPhaseLabel();
                bool burnWaits = _s.LandingPhase == LandingPhase.DeorbitBurn && _s.DeorbitTarget != null && _s.DeorbitTarget.IgnitionTime.Seconds() > SimNow();
                string phase = a.LandingMachineLive ? label : GuidanceModes.DeorbitStepOf(label, burnWaits);
                bool waiting = _s.LandingPhase is LandingPhase.DeorbitPlanning or LandingPhase.TransferPlanning
                    or LandingPhase.DeorbitCoast or LandingPhase.TransferCoast or LandingPhase.Coast
                    or LandingPhase.DeorbitNodePending;
                (double tgo, string tgoLabel) = DescentCountdown(vehicle);
                bool braking = (_s.LandingPhase is LandingPhase.Prep or LandingPhase.Burn) && _s.Upfg.Tgo > 0.0;
                bool deorbitBurn = IsStockDeorbitPhase(_s.LandingPhase);
                bool hoverFlies = !_s.GfoldHoverRefused && HoverFliesByThrust(vehicle);
                string method = _s.UseSixDofLanding ? "TOUCHDOWN" : "G-FOLD";
                // A direct approach holds a braking plan and no stock burn (DeorbitPlanner.DirectSteps).
                bool stockBurn = _s.DeorbitPlan != null || _s.DirectBrakingPlan == null;
                return new ModeReadout(a, "LANDING",
                    method, phase, Tone(!waiting), tgo, tgoLabel,
                    DescentNumbers(vehicle, a.LandingMachineLive), braking ? PageText.BrakingToGo(_s.Upfg.VgoMag) : deorbitBurn ? "" : PageText.HoverNumbersCard,
                    PhaseList: a.LandingMachineLive ? GuidanceModes.GfoldPhasesFor(hoverFlies, _s.LandingPhase == LandingPhase.TerminalCoast) : GuidanceModes.DeorbitStepsFor(stockBurn, method, hoverFlies),
                    Word: LandingWord(in a), HeightShown: !deorbitBurn, ImpactShown: a.LandingMachineLive);
            }
            case StopTarget.Boostback:
            {
                string phase = BoostbackPhaseName(_s.BoostbackPhase).ToUpperInvariant();
                double miss = vehicle?.Orbit?.Parent is IParentBody body ? ImpactMissDistance(body) : double.NaN;
                string missText = double.IsFinite(miss) ? "MISS " + Fmt.KmFine(miss / 1000.0) : "";
                PlanTone missTone = PlanWords.MissTone(miss / 1000.0, flying: true);
                IReadOnlyList<string> chain = GuidanceModes.BoostbackPhasesFor(_s.ThreeDofFromGlide);
                switch (_s.BoostbackPhase)
                {
                    case BoostbackPhase.Glide:
                    {
                        double entryIn = _s.ThreeDofFromGlide && double.IsFinite(_s.GlideTgo) ? _s.GlideTgo - _s.ThreeDofEngageTgoS : double.NaN;
                        double glideMissKm = Math.Sqrt(_s.GlideMissDownM * _s.GlideMissDownM + _s.GlideMissCrossM * _s.GlideMissCrossM) / 1000.0;
                        return new ModeReadout(a, "BOOSTBACK", "", phase, Tone(true), _s.GlideTgo, "IMPACT T-",
                            PageText.GlideNumbers(_s.GlideMissDownM, _s.GlideMissCrossM, entryIn), PageText.GlideNumbersCard, PhaseList: chain,
                            Word: PlanWords.Miss(glideMissKm), ImpactShown: true);
                    }
                    case BoostbackPhase.Boostback:
                        return new ModeReadout(a, "BOOSTBACK", "", phase, Tone(true), _s.BoostbackTgo, "BURN LEFT",
                            MissAnd(missText, missTone, new HeadValue("DV TO GO " + Fmt.DvWhole(_s.BoostbackDvGo))), _s.HasImpact ? PageText.ImpactCard(_s.Impact.TimeOfFlight.V) : "", PhaseList: chain,
                            Word: PlanWords.Of(_s.BoostbackHasPlan, noPlanLasts: true, lastPlan: _s.BoostbackPlanError.Length > 0));
                    case BoostbackPhase.Rotation:
                    {
                        double errorDeg = vehicle?.FlightComputer != null ? vehicle.FlightComputer.ErrorAngles.Length() * 180.0 / Math.PI : double.NaN;
                        HeadValue[] attitude = double.IsFinite(errorDeg) ? [new HeadValue("TURN " + Fmt.Deg(errorDeg), 1)] : [];
                        PlanWord word = !_s.BoostbackHasPlan ? PlanWords.Of(false, NoPlanHolds(_s))
                            : errorDeg <= BoostbackAlignDeg ? PlanWords.OnPlan : PlanWords.Turning;
                        return new ModeReadout(a, "BOOSTBACK", "", phase, Tone(true), _s.HasImpact ? _s.Impact.TimeOfFlight.V : double.NaN, "IMPACT T-",
                            MissAnd(missText, missTone, attitude), PhaseList: chain, Word: word, ImpactShown: true);
                    }
                    default:
                        return new ModeReadout(a, "BOOSTBACK", "", phase, Tone(true), _s.HasImpact ? _s.Impact.TimeOfFlight.V : double.NaN, "IMPACT T-",
                            MissAnd(missText, missTone), PhaseList: chain, ImpactShown: true);
                }
            }
            case StopTarget.Hover:
                return new ModeReadout(a, "HOVER", "", "", Tone(true), double.NaN, "", HoverNumbers(vehicle), PageText.HoverNumbersCard, HeightShown: true, ImpactShown: true);
            // An armed launch and a launch waiting on its plan are the ascent before lift-off, with the ARMED and PLANNING phases.
            case StopTarget.Ascent:
            case StopTarget.LaunchArmed:
            {
                AscentMethodUi ascent = CurrentAscentMethodUi();
                string method = ascent.Id == AscentMethod.CatsUpfg ? "UPFG" : ascent.Label;
                string phase = ascent.PhaseLabel();
                bool flying = _s.Running;
                bool solved = _s.Upfg.Tgo > 0.0 || _s.Phase == AscentPhase.Terminal;
                // An armed launch counts to the lift-off the calculated climb chose near the window, as StepLaunchWindow fires it.
                double tgo = flying ? (solved ? AscentTgoSeconds() : double.NaN) : _s.LaunchArmed ? _s.LaunchTargetTime - SimNow() + (ExecuteLaunchInstant() - WindowInstant()) : double.NaN;
                string card = flying && _s.Upfg.Tgo > 0.0
                    ? PageText.AscentToGo(_s.Upfg.VgoMag, _s.Upfg.Rgo.Length() / 1000.0)
                    : "";
                bool planFlies = flying ? _s.FlyingPlan != null : _s.FlyConvexAscent && !BackupLaunchChosen;
                PlanWord word = flying ? AscentWord() : a.ControlAcquired ? default : PlanWords.Yours(_s.LaunchArmed ? "WINDOW" : "LIFTOFF");
                return new ModeReadout(a, "ASCENT", method, phase,
                    Tone(flying), tgo, flying ? "CUTOFF T-" : "WINDOW T-", flying ? AscentNumbers(vehicle) : Array.Empty<HeadValue>(), card, NumbersHoldOrbit: flying,
                    PhaseList: ascent.Id == AscentMethod.CatsUpfg ? GuidanceModes.AscentPhasesFor(planFlies) : null, Word: word);
            }
            case StopTarget.Registered when a.RegisteredModeLive is GuidanceModeId id && HooksOf(id) is GuidanceModeHooks hooks:
            {
                string phase;
                try
                {
                    phase = hooks.PhaseLabel(_s) ?? "";
                }
                catch (Exception ex)
                {
                    phase = "";
                    LogHelper.WarnOnce($"afc-mode-phase-{id}:{ex.GetType().Name}",
                        $"[AFC] The phase label of guidance mode {id} failed: {ex.Message}");
                }
                ModeLiveReadout live = new(double.NaN, "", Array.Empty<HeadValue>(), false);
                if (hooks.Readout != null)
                {
                    try
                    {
                        live = hooks.Readout(vehicle, _s);
                    }
                    catch (Exception ex)
                    {
                        LogHelper.WarnOnce($"afc-mode-readout-{id}:{ex.GetType().Name}",
                            $"[AFC] The head readout of guidance mode {id} failed: {ex.Message}");
                    }
                }
                return new ModeReadout(a, GuidanceModes.Of(id)?.MethodLabel ?? id.ToString().ToUpperInvariant(), "", phase,
                    Tone(!live.Waiting), live.Seconds, live.SecondsLabel ?? "", live.Numbers ?? Array.Empty<HeadValue>(), live.HeightShown ? PageText.HoverNumbersCard : "",
                    Word: live.Word, HeightShown: live.HeightShown, ImpactShown: live.ImpactShown);
            }
            default:
                return OtherModes(vehicle, a);
        }
    }

    private static ModeReadout OtherModes(Vehicle vehicle, CraftActivity a)
    {
        if (a.RcsFlying)
            return RcsReadout(vehicle, a);
        if (a.MultiPassActive && vehicle != null && MultiPassRegistry.TryGet(vehicle.Id, out MultiPassExecution exec))
        {
            string phase = $"PASS {exec.PassIndex + 1} OF {exec.PassCountTotal}";
            Burn burn = exec.CurrentBurn;
            double next = burn != null ? burn.Time.Seconds() - SimNow() : double.NaN;
            return new ModeReadout(a, "PASSES", "", phase,
                ModeTone.Waiting, next > 0.0 ? next : double.NaN, "IN", burn != null ? [new HeadValue("BURN " + Fmt.Dv(burn.DeltaVVlf.Length()))] : Array.Empty<HeadValue>(), PhaseBesideName: true);
        }
        return new ModeReadout(a, "", "", "", ModeTone.Idle, double.NaN, "", Array.Empty<HeadValue>());
    }

    // The RCS burn as the head monitors it: WAITING with the time to ignition, then ALIGNING or FIRING, and the delta-v still to go.
    private static ModeReadout RcsReadout(Vehicle vehicle, CraftActivity a)
    {
        FlightComputer fc = vehicle?.FlightComputer;
        BurnTarget bt = fc?.Burn;
        bool found = RcsExecRegistry.TryGet(vehicle?.Id, out RcsExecution exec);
        IReadOnlyList<string> chain = GuidanceModes.RcsPhasesFor(found && exec.ResolvedStrategy == RcsAttitudeStrategy.Align);
        if (bt == null)
            return new ModeReadout(a, "RCS BURN", "", "WAITING", ModeTone.Waiting, double.NaN, "", Array.Empty<HeadValue>(), PhaseList: chain);
        double toIgnition = (bt.IgnitionTime - Universe.GetElapsedTime()).Seconds();
        double toGo = bt.DeltaVToGoCci.Length();
        HeadValue[] numbers = [new HeadValue("TO GO " + Fmt.Dv(toGo))];
        if (toIgnition > 0.0)
            return new ModeReadout(a, "RCS BURN", "", "WAITING", ModeTone.Waiting, toIgnition, "TO IGNITION", numbers, PhaseList: chain);
        bool aligning = found && exec.ResolvedStrategy == RcsAttitudeStrategy.Align && RcsExecutor.OutsideAlignGate(fc);
        return new ModeReadout(a, "RCS BURN", "", aligning ? "ALIGNING" : "FIRING", ModeTone.Flying, double.NaN, "", numbers, PhaseList: chain);
    }

    private static HeadValue[] MissAnd(string missText, PlanTone tone, params HeadValue[] others)
        => missText.Length > 0 ? [new HeadValue(missText, Tone: tone), .. others] : others;

    private static HeadValue[] LandingNumbers(Vehicle vehicle, bool site)
    {
        Orbit orbit = vehicle?.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return [];
        double3 r = orbit.StateVectors.PositionCci;
        double3 vSrf = orbit.StateVectors.VelocityCci - double3.Cross(parent.GetAngularVelocityCci(), r);
        var alt = new HeadValue("ALT " + Fmt.Height(TerminalHeight(orbit, parent, parent.MeanRadius)));
        var vs = new HeadValue("VS " + Fmt.Speed(double3.Dot(vSrf, double3.Normalize(r))));
        return site && !NoSitePicked(parent)
            ? [alt, vs, new HeadValue("SITE " + Fmt.Distance(AngleBetween(r, SiteDirCciAt(parent, 0)) * parent.MeanRadius), 1)]
            : [alt, vs];
    }

    private static HeadValue[] HoverNumbers(Vehicle vehicle)
    {
        Orbit orbit = vehicle?.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return [];
        return [.. LandingNumbers(vehicle, site: false), new HeadValue("MIN TWR " + Fmt.Ratio(TerminalMinThrottleTwr(vehicle, orbit, parent.Mu)), 2)];
    }

    private static HeadValue[] AscentNumbers(Vehicle vehicle)
    {
        Orbit orbit = vehicle?.Orbit;
        return orbit == null ? [] : PageText.AscentNumbers(orbit.IsBound(), orbit.ApoapsisAltitude / 1000.0, Math.Max(_s.PeKm, _s.ApKm), orbit.PeriapsisAltitude / 1000.0, Math.Min(_s.PeKm, _s.ApKm), AscentMarginMs());
    }

    // The delta-v on board less what the last solve of UPFG still asks for, NaN before a solve. It is an estimate, because the velocity to go assumes the stage model it reads.
    private static double AscentMarginMs()
    {
        var stages = _s.UpfgVehicle;
        if (stages == null || stages.Stages.Count == 0 || !(_s.Upfg.Tgo > 0.0))
            return double.NaN;
        double total = 0.0;
        for (int i = 0; i < stages.Stages.Count; i++)
        {
            StageFigures(stages.Stages[i], out float dv, out _);
            total += dv;
        }
        return total - _s.Upfg.VgoMag;
    }

    private static PlanWord AscentWord() => _s.Phase switch
    {
        AscentPhase.ClosedLoop or AscentPhase.Terminal => _s.Upfg.Converged ? PlanWords.OnPlan : PlanWords.Of(false, StepFaultLasts),
        AscentPhase.Profile => PlanWords.FlyingPlan(_s.ConvexHandoverAltKm),
        _ => PlanWords.FlyingTurn,
    };

    private static PlanWord LandingWord(in CraftActivity a) => _s.LandingPhase switch
    {
        LandingPhase.GfoldDescent => PlanWords.Of(_s.GfoldPlan != null, NoPlanHolds(_s), _s.GfoldFailStreak > 0),
        LandingPhase.Burn => _s.Upfg.Converged ? PlanWords.OnPlan : PlanWords.Of(false, StepFaultLasts),
        LandingPhase.Prep => PlanWords.Planning,
        LandingPhase.Coast when !a.ControlAcquired => PlanWords.Yours("BURN"),
        _ => default,
    };

    // Up to the end of the deorbit burn the size of that burn says more than the height of an orbit. Once stock AUTO burns, the value counts down what the burn still has to deliver.
    private static HeadValue[] DescentNumbers(Vehicle vehicle, bool poweredDescent)
    {
        if (!IsStockDeorbitPhase(_s.LandingPhase) || _s.DeorbitPlan == null)
            return LandingNumbers(vehicle, poweredDescent);
        BurnTarget target = _s.DeorbitTarget;
        bool burning = _s.LandingPhase == LandingPhase.DeorbitBurn && target != null && target.IgnitionTime.Seconds() <= SimNow();
        return [new HeadValue("BURN " + Fmt.Dv(burning ? target.DeltaVToGoCci.Length() : _s.DeorbitPlan.DeltaV.Length()))];
    }

    // The countdown of a landing with its own word, because each phase counts to something else. A coast with none shows the time to impact.
    private static (double Seconds, string Label) DescentCountdown(Vehicle vehicle)
    {
        double tgo = DescentTgoSeconds();
        switch (_s.LandingPhase)
        {
            case LandingPhase.DeorbitPlanning:
            case LandingPhase.TransferPlanning:
            case LandingPhase.DeorbitCoast:
            case LandingPhase.TransferCoast:
            case LandingPhase.Coast:
            case LandingPhase.DeorbitNodePending:
            case LandingPhase.Prep:
                return (tgo, "IN");
            case LandingPhase.DeorbitBurn:
                double toIgnition = _s.DeorbitTarget != null ? _s.DeorbitTarget.IgnitionTime.Seconds() - SimNow() : double.NaN;
                return toIgnition > 0.0 ? (toIgnition, "IN") : (tgo, "BURN LEFT");
            case LandingPhase.Burn:
            case LandingPhase.TerminalBrake:
                return (tgo, "BURN LEFT");
            case LandingPhase.GfoldDescent:
                return (tgo, PageText.LandsIn);
            case LandingPhase.TerminalCoast:
                return (ImpactSeconds(vehicle, guidanceBound: false), "IMPACT T-");
            default:
                return (double.NaN, "");
        }
    }
}
