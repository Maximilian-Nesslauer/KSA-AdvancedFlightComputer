#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;

public static partial class GuidanceWindow
{
    // Public because VehicleAutopilotState holds a vehicle's phase: every craft runs this machine on its own, so the phase is a field on the flight computer rather than one global "the landing".
    // Every landing starts in DeorbitPlanning. A transfer continues through the stock node phases to TransferCoast, a direct approach continues at Coast, and both brake from Prep.
    public enum LandingPhase { Idle,
        DeorbitPlanning, DeorbitCoast, DeorbitNodePending, DeorbitBurn, TransferPlanning, TransferCoast,
        Coast, Prep, Burn, GfoldDescent, TerminalHover, TerminalCoast, TerminalBrake, Done }

    // The site, the approach shaping (downrange factor, gate altitude/uprange, sink rate) and the whole pass scan live on the vehicle - see VehicleAutopilotState.
    private const double PrepLeadTime = 30.0;      // converge + point before ignition

    // Upcoming site passes are computed in closed form - see Guidance/SitePasses.cs.
    private const int PassesToShow = 5;

    /// <summary>
    /// Skip straight to G-FOLD from the current state, or restart it after a failure.
    /// Engages the autopilot and auto engines, because a powered descent that is not
    /// allowed to steer or throttle is not a descent.
    /// </summary>
    private static void StartGfoldNow(Vehicle vehicle)
        => BeginGfoldDescent(vehicle, SimNow(), engineLit: false, "G-FOLD STARTED FROM CURRENT STATE");

    // A lit engine keeps its throttle as the first G-FOLD command, so a handover from another mode does not blip the engine.
    private static void BeginGfoldDescent(Vehicle vehicle, double now, bool engineLit, string status)
    {
        ResetLandingEngineWait();
        ClaimVehicle(GuidanceMode.Landing, vehicle);   // the descent takes the vehicle over
        _s.Engage = true;
        _s.AutoStage = true;
        _s.LandingPhase = LandingPhase.GfoldDescent;
        ResetGfoldTrace();   // fresh flown path and a fresh axis latch
        _s.GfoldHandoffTime = now;
        _s.GfoldLastSolveTime = double.NegativeInfinity;
        _s.GfoldPlan = null;
        _s.GfoldFailStreak = 0;
        _s.GfoldTrackInit = false;
        _s.GfoldEngineOn = engineLit;
        if (engineLit)
            _s.GfoldThrottle = vehicle.GetManualThrottle();
        _s.GfoldRelightRequestTime = double.NaN;
        _s.GfoldHoverRefused = false;
        _s.HasCommand = false;
        _s.LandingStatus = status;
    }

    // An abort in the air hands the craft back with the engine as it is, because a cut there drops the craft. The coast and the deorbit burn keep the cut, which stops the burn and costs a coasting craft nothing.
    private static void AbortLanding()
    {
        ResetLandingEngineWait();
        _s.DeorbitPlanner?.Dispose();
        _s.DeorbitPlanner = null;
        if (!_s.ControlAcquired && !_s.DeorbitNodeClaimed) ClearDeorbitPlanState();
        bool airborne = _s.LandingPhase == LandingPhase.GfoldDescent
            || _s.LandingPhase == LandingPhase.TerminalHover || TerminalBurnActive;
        _s.LandingPhase = LandingPhase.Done;
        if (airborne)
        {
            _s.ReleaseWithoutEngineCut = true;
            _s.LandingStatus = "LANDING STOPPED. ENGINE UNCHANGED.";
        }
        else
        {
            _s.LandingCutPending = true;
            _s.LandingStatus = "LANDING STOPPED";
        }
    }

    private static bool CraftOnWayDown(Vehicle vehicle)
        => Ui.Console.ConsoleFrame.ReadFacts(vehicle).Where is Ui.Logic.CraftSituation.Unknown or Ui.Logic.CraftSituation.CoastingUp
            or Ui.Logic.CraftSituation.FallingAir or Ui.Logic.CraftSituation.FallingNoAir;

    private static void EndLandingBadly(Vehicle vehicle, string what, string text)
    {
        if (CraftOnWayDown(vehicle))
            _s.FailLanding(what, text);
        else
            _s.WarnLanding(text);
    }

    private static void EndTouchdownBadly(Vehicle vehicle, string what, string text)
    {
        if (CraftOnWayDown(vehicle))
            _s.FailError(what, text);
        else
            _s.WarnError(text);
    }

    // These phases wait without control, and a release keeps them queued.
    private static bool LandingWaits(LandingPhase phase) =>
        phase is LandingPhase.Coast or LandingPhase.DeorbitPlanning or LandingPhase.DeorbitCoast;

    // Stock Auto flies the deorbit node in these phases while guidance holds only the claim.
    private static bool IsStockDeorbitPhase(LandingPhase phase) =>
        phase is LandingPhase.DeorbitCoast or LandingPhase.DeorbitNodePending or LandingPhase.DeorbitBurn;

    private static bool TerminalBurnActive => _s.LandingPhase is LandingPhase.TerminalCoast or LandingPhase.TerminalBrake;

    // G-FOLD or its landing burn is flying. A 6-DOF descent can fall back to G-FOLD, so while this holds the LAND page belongs to G-FOLD whatever solver is selected.
    private static bool LandingMachineLive => _s.LandingPhase == LandingPhase.GfoldDescent || TerminalBurnActive;

    private static bool LandingSwitchesOff => !_s.Engage || !_s.AutoStage;

    private const string LandingSwitchesChangedStatus = "LANDING STOPPED: CONTROL SWITCHES CHANGED";

    // These phases use AFC's manual engine and attitude control.
    // The direct coast waits until Prep, and the stock node takes a separate claim without forcing Manual.
    private static bool LandingCommands(LandingPhase phase) =>
        phase is LandingPhase.TransferPlanning or LandingPhase.TransferCoast or LandingPhase.Prep
            or LandingPhase.Burn or LandingPhase.GfoldDescent or LandingPhase.TerminalHover
            or LandingPhase.TerminalCoast or LandingPhase.TerminalBrake;

    // Runs from ApplyAutopilot ahead of the claim, so the step that turns the coast into Prep is the step that claims the craft.
    private static void StepLandingCoast()
    {
        if (_s.LandingPhase == LandingPhase.Coast && SimNow() >= _s.BurnStartTime - PrepLeadTime)
            EnterBrakingPrep();
    }

    private static void EnterBrakingPrep()
    {
        if (Universe.IsAutoWarpActive || Universe.GetSimulationSpeed() > 1.0)
            Universe.ResetSimulationSpeed();
        _s.Upfg.Reset();
        _s.LandingPhase = LandingPhase.Prep;
        _s.LandingStatus = "PLANNING BRAKING BURN";
    }

    // The phases that a ground contact ends. Touchdown arming carries across a change between two of them.
    private static bool IsPoweredDescentPhase(LandingPhase phase) =>
        phase is LandingPhase.DeorbitBurn or LandingPhase.TransferCoast or LandingPhase.Burn
            or LandingPhase.GfoldDescent or LandingPhase.TerminalHover or LandingPhase.TerminalCoast or LandingPhase.TerminalBrake;

    // KSA's own contact switch.
    // The physics step raises a terrain-contact flag on the vehicle whenever ANY part of it makes a Bepu contact with the terrain or launch-pad collider (ConstraintSim.DetectTerrainContact), and ocean entry sets the matching ocean flag - so this fires on the legs, or on whatever else reaches the ground first, without us guessing at leg geometry.
    //
    // This replaces trusting the altitude estimate to notice touchdown.
    // That estimate is terrain-height sampling minus an assumed vehicle height, and when it reads low the hover controller keeps flying a vehicle that is already on the ground.
    private static bool HasTouchedDown(Vehicle vehicle) => vehicle.Situation.HasAnyContact();


    // Per-frame landing state machine, run for this vehicle from ApplyAutopilot (the PrepareWorker prefix) whether or not it is the one on screen.
    private static void StepLanding(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                    double mu, double bodyRadius)
    {
        if (_s.LandingPhase == LandingPhase.Idle || _s.LandingPhase == LandingPhase.Done)
            return;

        double now = SimNow();

        // Touchdown, from the physics rather than from geometry.
        // Checked before any powered phase steps, so the engine is cut on the frame contact is reported instead of the controller fighting the ground.
        //
        // Armed only once the vehicle has actually been off the ground: a vehicle sitting on the pad already reports terrain contact (the launch-pad collider counts), so without this, taking over with terminal hover from the ground would cut the engine on its first step.
        // Preserve arming across descent transitions because contact can arrive on the transition step.
        bool contact = HasTouchedDown(vehicle);
        if (_s.LandingPhase != _s.TouchdownPrevPhase)
        {
            bool continuesDescent = IsPoweredDescentPhase(_s.TouchdownPrevPhase) && IsPoweredDescentPhase(_s.LandingPhase);
            _s.TouchdownPrevPhase = _s.LandingPhase;
            if (!continuesDescent) _s.LandingTouchdownArmed = false;
        }
        if (!contact)
            _s.LandingTouchdownArmed = true;

        if (contact && _s.LandingTouchdownArmed && IsPoweredDescentPhase(_s.LandingPhase))
        {
            ResetLandingEngineWait();
            _s.GfoldThrottle = 0.0;
            _s.HasCommand = false;
            _s.LandingPhase = LandingPhase.Done;
            _s.LandingCutPending = true;
            _s.LandingStatus = FormattableString.Invariant($"TOUCHDOWN: CONTACT, ENGINE CUT ({_s.GfoldSpeedMs:F1} m/s)");
            return;
        }

        if (StepDeorbit(vehicle, orbit, parent, now)) return;

        if ((_s.DeorbitRequest != null || TerminalBurnActive) && LandingSwitchesOff)
        {
            AbortLanding();
            _s.WarnLanding(LandingSwitchesChangedStatus);
            return;
        }

        if (TerminalBurnActive)
        {
            StepTerminalBurn(vehicle, orbit, parent, now);
            return;
        }

        if (_s.LandingPhase == LandingPhase.GfoldDescent)
        {
            StepGfoldDescent(vehicle, orbit, parent, bodyRadius, now);
            return;
        }

        if (_s.LandingPhase == LandingPhase.TerminalHover)
        {
            StepTerminalHover(vehicle, orbit, parent, mu, bodyRadius, now);
            return;
        }

        // The coast turned into Prep in StepLandingCoast, ahead of the claim, so a coast that reaches this step is still waiting.
        if (_s.LandingPhase == LandingPhase.Coast)
            return;

        string engineRefusal = BrakingEngineRefusal(vehicle);
        if (engineRefusal.Length > 0) { RefuseDeorbit(vehicle, engineRefusal); return; }
        if (!PrepareLandingEngines(vehicle, parent, now, requireAirless: false)) return;

        // Prep / Burn: run Mode-3 guidance on the live vehicle.
        // The landing target is re-derived every step: the site rotates with the body, and the plane is whatever we are actually flying in.
        try
        {
            UpfgVehicle live = BuildUpfgVehicle(vehicle);
            if (live != null)
            {
                double gLimit = EffectiveAccelLimitG(vehicle);
                if (double.IsFinite(gLimit))
                    ApplyGLimit(live, gLimit);
                _s.UpfgVehicle = live;

                double3 r = orbit.StateVectors.PositionCci;
                double3 v = orbit.StateVectors.VelocityCci;
                double3 planeNormal = double3.Normalize(double3.Cross(r, v));
                // The gate: above the site (terrain-referenced, not the mean sphere) and uprange of it along the approach (rotating a position about +h moves it downrange, so uprange is the negative rotation).
                double gateRadius = bodyRadius + SiteTerrainHeight(parent) + LandingBrakeGateAltitude;
                double3 gateDir = RotateAbout(SiteDirCciAt(parent, 0), planeNormal,
                    -_s.GateUprangeKm * 1000.0 / bodyRadius);
                var target = new UpfgTarget
                {
                    Radius = gateRadius,
                    Velocity = 0,                 // no forward speed at the gate
                    DescentRate = _s.DescentRate,   // arrive sinking, not stopped
                    Fpa = 0,
                    Normal = planeNormal,
                    Rdes = gateDir * gateRadius,
                };
#if DEBUG
                using (new AdvancedFlightComputer.Core.PerfTracker.Scope("Guidance.DeorbitSolve"))
#endif
                {
                    _s.Upfg.Step(r, v, vehicle.TotalMass, mu, target, live, 3);
                }
                _s.CommandDir = _s.Upfg.Steering;
                _s.HasCommand = _s.CommandDir.Length() > 0.5;

                // The same sample the ascent logs, so a burn that dives or overshoots can be read after the flight.
                if (GuidanceLog.Enabled && now - _s.LastGuidanceLogTime >= GuidanceLogIntervalS)
                {
                    _s.LastGuidanceLogTime = now;
                    double3 up = double3.Normalize(r);
                    double siteDistanceKm = AngleBetween(r, SiteDirCciAt(parent, 0)) * bodyRadius / 1000.0;
                    GuidanceLog.Debug(vehicle, $"UPFG {_s.LandingPhase}: tgo {_s.Upfg.Tgo:F1} s, vgo {_s.Upfg.VgoMag:F0} m/s, converged {_s.Upfg.Converged}"
                        + $", throttle {_s.Upfg.Throttle:F2}, steer pitch {PitchOf(up, _s.Upfg.Steering):F1} deg, command pitch {PitchOf(up, _s.CommandDir):F1} deg"
                        + $", alt {(r.Length() - bodyRadius - SiteTerrainHeight(parent)) / 1000.0:F1} km over the site, {siteDistanceKm:F1} km to it"
                        + $", speed {v.Length():F0} m/s, sink {-double3.Dot(v, up):F0} m/s, mass {vehicle.TotalMass / 1000.0:F1} t, model {live.Stages.Count} stage(s).");
                }
            }
            _s.FailStreak = 0;
            _s.GuidanceError = "";
        }
        catch (Exception e)
        {
            _s.FailStreak++;
            _s.GuidanceError = e.Message;
            if (_s.FailStreak > MaxFailStreak)
            {
                if (_s.LandingPhase != LandingPhase.Done)
                    GuidanceLog.Info(vehicle, $"landing stopped after {_s.FailStreak} failed steps: {e}");
                _s.LandingPhase = LandingPhase.Done;
                _s.LandingCutPending = true;
                EndLandingBadly(vehicle, "LANDING STOPPED: REPEATED FAULTS", "GUIDANCE FAULT (REPEATED). LANDING STOPPED.");
            }
        }

        if (_s.LandingPhase == LandingPhase.Prep && now >= _s.BurnStartTime)
        {
            if (!CheckBrakingIgnition(vehicle, orbit, now)) return;
            _s.LandingPhase = LandingPhase.Burn;
        }

        if (_s.LandingPhase == LandingPhase.Burn)
            StepBrakingHandover(vehicle, parent, now);
    }

    // With no plan from the standby this long before the gate, the braking burn hands over anyway. The craft is then slow and near the gate, which G-FOLD plans from in one solve.
    private const double BrakingLastHandoverTgoS = 5.0;

    // Oldest a standby plan may be and still take the braking burn, s. Longer than at the end of an entry burn, because one cold iteration under the braking burn can take a second and the craft is still far from the ground.
    private const double BrakingStandbyMaxPlanAgeS = 2.0;

    /// <summary>
    /// Whether G-FOLD can fly this craft down, and if not, why. G-FOLD plans a point mass on an airless body and steers the thrust through the flight computer, so it needs liquid engines that throttle and shut down, and an engine on a gimbal to hold the thrust direction through the burn. A craft that turns only with RCS is refused, because RCS torque alone is not known to hold the thrust axis of a lit engine.
    /// </summary>
    private static bool GfoldCanFly(Vehicle vehicle, IParentBody parent, out string why)
    {
        why = KsaGfold.RefusalReason(vehicle, parent);
        if (why.Length == 0 && Ksa6DofSetup.GimbalLimitDeg(vehicle) <= 0.0)
            why = "G-FOLD needs a gimballed engine to hold the thrust direction.";
        return why.Length == 0;
    }

    /// <summary>
    /// Decides the descent solver of a landing from orbit before anything burns, because the braking gate of the deorbit depends on it: touchdown guidance, or G-FOLD for a craft that touchdown guidance cannot plan for. Returns the refusal when neither can fly the craft. A craft with no active engine cannot be measured yet, so it keeps touchdown guidance, and the braking burn decides again at its hand-over.
    /// </summary>
    private static string ChooseLandingSolver(Vehicle vehicle)
    {
        _s.UseSixDofLanding = true;
        if (!KsaEnginePerf.SupportsThrottleControl(vehicle) || Can6DofTake(vehicle, out string touchdown))
            return "";
        if (!GfoldCanFly(vehicle, vehicle.Orbit.Parent, out string gfold))
            return Ui.Logic.PageText.LandingRefused(touchdown, gfold);
        _s.UseSixDofLanding = false;
        GuidanceLog.Info(vehicle, $"Touchdown guidance cannot plan this craft ({touchdown}). G-FOLD flies the descent.");
        return "";
    }

    /// <summary>
    /// The hand-over of the braking burn to the powered descent, from GfoldHandoffTgo before the gate. Touchdown guidance solves on standby while the burn keeps flying the craft, and takes it once it has a fresh plan, so the craft always has live guidance (SixDofStandby.cs). A craft that touchdown guidance cannot plan for goes to G-FOLD at once. A craft whose standby has no plan when the burn reaches its last seconds goes to G-FOLD when G-FOLD can fly it, and to a cold touchdown engage when it cannot.
    /// </summary>
    private static void StepBrakingHandover(Vehicle vehicle, IParentBody parent, double now)
    {
        if (!_s.SixDofStandby && !(_s.Upfg.Converged && _s.Upfg.Tgo <= _s.GfoldHandoffTgo))
            return;

        string sixDofRefusal = null;
        if (_s.UseSixDofLanding)
        {
            // The standby runs the spread cold solve. The two options that replace it plan in one step at the engage, so the burn hands over directly.
            bool direct = _s.SixDofFixedTime || _s.SixDofGfoldSeed;
            bool last = _s.Upfg.Tgo <= BrakingLastHandoverTgoS;
            double3 siteCci = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
            double[] x = KsaFrameBridge.ToModelState(vehicle, KsaFrameBridge.BuildSiteFrame(siteCci));
            double seed = Math.Max(_s.Upfg.Tgo, SixDofStandbyMinSeedS);
            if (!_s.SixDofStandby && !Can6DofTake(vehicle, out sixDofRefusal))
            {
                // The craft is lighter and may have other engines lit than at the commit, so G-FOLD is asked again here. With no solver for the craft, the refused touchdown engage releases it with the burn's engine command and says so.
                if (!GfoldCanFly(vehicle, parent, out _))
                {
                    HandBrakingBurnToTouchdown(vehicle, "HANDOVER: TOUCHDOWN");
                    return;
                }
                _s.UseSixDofLanding = false;
            }
            else if (direct)
            {
                HandBrakingBurnToTouchdown(vehicle, "HANDOVER: TOUCHDOWN");
                return;
            }
            else if (!_s.SixDofStandby)
            {
                // A standby that faulted waits out its retry time, but not past the last seconds of the burn.
                if (now >= _s.SixDofStandbyRetryAt && StartSixDofStandby(vehicle, parent, siteCci, x, now, seed))
                    _s.LandingStatus = "BRAKING. TOUCHDOWN PLANS ON STANDBY.";
                if (!last || _s.SixDofStandby)
                    return;
            }
            else
            {
                StepSixDofStandby(vehicle, parent, siteCci, x, now, seed);
                if (SixDofStandbyReady(now, BrakingStandbyMaxPlanAgeS))
                {
                    HandBrakingBurnToTouchdown(vehicle, "HANDOVER: TOUCHDOWN");
                    return;
                }
                if (!last)
                    return;
            }

            if (_s.UseSixDofLanding)
            {
                // The last seconds of the burn with no plan from the standby.
                if (!GfoldCanFly(vehicle, parent, out _))
                {
                    HandBrakingBurnToTouchdown(vehicle, "HANDOVER: TOUCHDOWN, NO PLAN YET");
                    return;
                }
                sixDofRefusal = "no plan before the end of the braking burn";
                _s.UseSixDofLanding = false;
            }
        }

        StopSixDofStandby();
        _s.LandingPhase = LandingPhase.GfoldDescent;
        ResetGfoldTrace();   // fresh flown path and a fresh axis latch
        _s.GfoldHandoffTime = now;
        _s.GfoldLastSolveTime = double.NegativeInfinity;
        _s.GfoldPlan = null;
        _s.GfoldApproach = GfoldApproach.BrakeAtGate;
        _s.GfoldFailStreak = 0;
        _s.GfoldTrackInit = false;
        // The engine is lit from the burn, so the tracker's coast hysteresis starts from a lit engine and switches it off only when the plan really coasts.
        _s.GfoldEngineOn = true;
        _s.GfoldRelightRequestTime = double.NaN;
        _s.GfoldHoverRefused = false;
        // This step already writes the descent's engine command, and G-FOLD has no throttle until its first plan on the next step, so the burn's throttle carries over. Left at zero, the braking burn stops for a step, and for as long as the first solves find no plan. The tracker does not smooth from the carried value, because GfoldTrackInit is reset above.
        _s.GfoldThrottle = _s.Upfg.Throttle;
        _s.LandingStatus = sixDofRefusal == null
            ? "HANDOVER: G-FOLD"
            : $"HANDOVER: G-FOLD. TOUCHDOWN: NO SOLUTION ({sixDofRefusal}).";
        if (sixDofRefusal != null)
            _s.WarnLanding(_s.LandingStatus);
        _s.GfoldCuePending = true;   // cue the LAND NOW way of LAND
    }

    // 6-DOF is exclusive, because it drives attitude through the TVC allocator and not the flight computer, so the landing machine lets go. Engage6Dof is that release (ClaimVehicle) and the request in one, and the next guidance step consumes the request. It is reached from the sim step, so the release at the end of the same step sees the queued request and keeps the craft for it (ApplyAutopilot), and the engine holds the burn's command until the engage runs. A standby that is running is promoted by that engage with the plan it has.
    private static void HandBrakingBurnToTouchdown(Vehicle vehicle, string status)
    {
        Engage6Dof(vehicle, fromBrakingBurn: true);
        _s.LandingStatus = status;
        _s.GfoldCuePending = true;   // cue the LAND NOW way of LAND
    }

    // ----- Landing site geometry -----

    // The site's body-fixed (CCF) direction.
    // KSA's own convention: lat = asin(z), lon = atan2(y,x) in CCF.
    private static double3 SiteDirCcf() => SiteDirCcf(_s.SiteLatDeg, _s.SiteLonDeg);

    internal static double3 SiteDirCcf(double latDeg, double lonDeg)
    {
        double lat = UpfgTarget.DegToRad(latDeg);
        double lon = UpfgTarget.DegToRad(lonDeg);
        return new double3(
            Math.Cos(lat) * Math.Cos(lon),
            Math.Cos(lat) * Math.Sin(lon),
            Math.Sin(lat));
    }

    // The site's CCI direction dtFuture seconds from now.
    // A body spins about its own CCI Z axis (per IParentBody.GetAngularVelocityCci), so the future position is the current one carried around Z.
    private static double3 SiteDirCciAt(IParentBody parent, double dtFuture)
    {
        double3 dirNow = SiteDirCcf().Transform(parent.GetCcf2Cci());
        return RotZ(dirNow, parent.GetAngularVelocity() * dtFuture);
    }

    // Terrain height of the site above the body's mean-radius sphere, sampled from KSA's own heightmap (the game places surface objects at MeanRadius + this).
    // The site is fixed in CCF, so the value is cached and only re-sampled when the inputs (or the body) change.

    private static double SiteTerrainHeight(IParentBody parent)
    {
        if (_s.SiteLatDeg != _s.SiteTerrainCacheLat || _s.SiteLonDeg != _s.SiteTerrainCacheLon
            || !ReferenceEquals(parent, _s.SiteTerrainCacheBody))
        {
            _s.SiteTerrainHeightM = (parent as Celestial)?.GetTerrainHeightFromDirCcf(SiteDirCcf()) ?? 0.0;
            if (!double.IsFinite(_s.SiteTerrainHeightM))
                _s.SiteTerrainHeightM = 0.0;
            _s.SiteTerrainCacheLat = _s.SiteLatDeg;
            _s.SiteTerrainCacheLon = _s.SiteLonDeg;
            _s.SiteTerrainCacheBody = parent;
        }
        return _s.SiteTerrainHeightM;
    }

    private const double SiteGroundRingStepM = 0.5;
    private const int SiteGroundRingPoints = 16;
    private const double SiteGroundMaxReachM = 30.0;

    // How far above the vehicle height the first contact at the site comes for these foot points. The terrain is sampled on rings around the site out to the widest foot point, once per site and reach, from the height function that the game builds its terrain collision mesh from (BubbleTerrainGrid.SampleRadius calls Celestial.GetTerrainHeightFromDirCcf).
    private static double SiteEarlyContact(IParentBody parent, TouchdownContact.FootPoint[] feet)
    {
        if (parent is not Celestial body || feet == null || feet.Length == 0)
            return 0.0;
        double reach = 0.0;
        foreach (var foot in feet)
            reach = Math.Max(reach, foot.RadiusM);
        reach = Math.Min(Math.Ceiling(reach), SiteGroundMaxReachM);
        if (!(reach > 0.0))
            return 0.0;

        if (_s.SiteGround == null || _s.SiteLatDeg != _s.SiteGroundCacheLat || _s.SiteLonDeg != _s.SiteGroundCacheLon
            || reach != _s.SiteGroundCacheReachM || !ReferenceEquals(parent, _s.SiteGroundCacheBody))
        {
            double3 up = SiteDirCcf();
            double3 east = double3.Cross(new double3(0, 0, 1), up);
            east = east.LengthSquared() > 1e-12 ? double3.Normalize(east) : new double3(1, 0, 0);
            double3 north = double3.Cross(up, east);
            double centre = body.GetTerrainHeightFromDirCcf(up);
            double radius = body.MeanRadius + centre;
            var highest = new double[(int)Math.Round(reach / SiteGroundRingStepM)];
            for (int ring = 0; ring < highest.Length; ring++)
            {
                double r = (ring + 1) * SiteGroundRingStepM;
                double high = double.NegativeInfinity;
                for (int i = 0; i < SiteGroundRingPoints; i++)
                {
                    double a = i * 2.0 * Math.PI / SiteGroundRingPoints;
                    double3 point = up * radius + (east * Math.Cos(a) + north * Math.Sin(a)) * r;
                    high = Math.Max(high, body.GetTerrainHeightFromDirCcf(double3.Normalize(point)) - centre);
                }
                highest[ring] = high;
            }
            _s.SiteGround = new TouchdownContact.GroundRise(SiteGroundRingStepM, highest);
            _s.SiteGroundCacheLat = _s.SiteLatDeg;
            _s.SiteGroundCacheLon = _s.SiteLonDeg;
            _s.SiteGroundCacheReachM = reach;
            _s.SiteGroundCacheBody = parent;
        }
        return TouchdownContact.EarlyContactM(feet, _s.SiteGround);
    }

}
