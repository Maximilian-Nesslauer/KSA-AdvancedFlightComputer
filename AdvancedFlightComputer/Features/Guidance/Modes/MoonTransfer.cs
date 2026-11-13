#nullable disable

using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MissionPlanner;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.Features.Transfer;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>Where the chain from a completed ascent to a moon stands.</summary>
public enum MoonChainStage
{
    Idle,
    /// <summary>The ascent is complete and still lets go of the craft.</summary>
    AfterAscent,
    /// <summary>Stock AUTO has the injection burn.</summary>
    Injection,
    /// <summary>A burn is done and the flown orbit is checked against the periapsis.</summary>
    Course,
    /// <summary>Stock AUTO has a correction burn.</summary>
    Correction,
    /// <summary>The craft coasts to the moon's sphere of influence for the capture.</summary>
    Coast,
    /// <summary>Stock AUTO has the capture burn.</summary>
    Capture,
}

// THEN on the LAUNCH page: the chain from a completed ascent to a moon. It sets the launch plane, plans the injection from the parking orbit, checks the flown orbit after each burn and plans a correction when the periapsis is off (CourseCorrector), and plans the capture burn when it was asked for.
// Guidance holds no claim for any of it. The ascent release has cut the engine and given the claim back before the first node is made, and stock AUTO flies every burn, so one owner has the craft at every moment.
// Each node is the player's like one made on the MANEUVER page: a node the player removes or changes, another planned burn, AUTO switched off or another controller on the craft stops the chain, and it never makes a second node to get its way.
// A stop that the burn plan did not cause (the pick on the LAUNCH page, another controller, guidance switched off, a fault) also selects Manual on a chain burn that has not started, so no burn of a stopped chain ignites later by itself.
public static partial class GuidanceWindow
{
    // The ascent release cuts the engine and hands the craft back within a few steps, and the game rebuilds the flight plan after a burn. Planning waits this long after either.
    private const double MoonSettleS = 10.0;

    // The ignition of the injection is never sooner than this after the plan, so stock Auto has the time to turn the craft. The ignition comes half a burn ahead of the node.
    private const double MoonBurnLeadS = 600.0;

    // An armed burn may read Manual this long before the chain takes it as switched off. Stock drops a started Auto burn to Manual when the active engines run dry, and AutoStage, when it is on, gives Auto back once the next stage burns.
    private const double MoonManualGraceS = 5.0;

    // The coast warp ends this long after the craft has entered the moon's sphere.
    private const double MoonSphereMarginS = 120.0;

    // An armed burn that has not completed this long after its node time plus its burn duration was left by stock.
    private const double MoonBurnOverdueS = 1800.0;

    // The plane of an inclination-controlled launch lies this far above the site's latitude, so the site passes under it twice a day and not only at one tangent point.
    private const double MoonPlaneLatitudeMarginDeg = 0.1;

    /// <summary>The moons a craft orbiting <paramref name="parent"/> can be sent to: its children that are bodies with a sphere of influence.</summary>
    internal static List<Celestial> MoonsOf(IParentBody parent)
    {
        var moons = new List<Celestial>();
        if (parent == null)
            return moons;
        foreach (IOrbiter child in parent.Children)
            if (child is Celestial moon && moon is IParentBody body && body.SphereOfInfluence > 0.0)
                moons.Add(moon);
        return moons;
    }

    private static Celestial FindMoon(IParentBody parent, string id)
    {
        foreach (Celestial moon in MoonsOf(parent))
            if (moon.Id == id)
                return moon;
        return null;
    }

    // The chain's moon by id from the whole system, because inside the moon's sphere the craft's parent is the moon itself.
    private static Celestial ChainMoon()
        => Universe.CurrentSystem != null && Universe.CurrentSystem.All.TryGet(_s.MoonChainMoonId, out Astronomical body)
            ? body as Celestial : null;

    #region Launch plane

    /// <summary>
    /// Keeps the plane target of a launch with THEN set: the moon's own orbit plane when the site passes under it, and otherwise the plane just above the site's latitude that holds the moon at the soonest arrival the launch can make.
    /// A picked target vessel wins, because its chase orbit is the player's explicit choice of plane. Without a moon there is no plane target.
    /// </summary>
    private static void RefreshMoonLaunchPlane(Vehicle vehicle, IParentBody parent)
    {
        if (_s.Running || _s.LaunchArmed || _s.MoonChain != MoonChainStage.Idle)
            return;
        Celestial moon = _s.TargetId.Length == 0 && _s.MoonTransferId.Length > 0 ? FindMoon(parent, _s.MoonTransferId) : null;
        _s.PlaneTarget = moon != null && TryMoonLaunchPlane(vehicle, moon, _s.PeKm, _s.ApKm, out AscentPlaneTarget target) ? target : null;
    }

    internal static bool TryMoonLaunchPlane(Vehicle vehicle, Celestial moon, double peKm, double apKm, out AscentPlaneTarget target)
    {
        target = null;
        IParentBody home = moon.Orbit.Parent;
        double now = SimNow();
        var inputs = new LunarLaunchInputs(
            LunarLaunchPlanner.EarliestArrival(moon, home.Mu, home.MeanRadius + peKm * 1000.0, now),
            PlaneControl.InPlane, 0.0, false, peKm, apKm);
        LunarLaunchPlan plan = LunarLaunchPlanner.Solve(vehicle, moon, now, inputs);
        if (plan.HasPlane && plan.SiteReachesPlane)
        {
            target = new AscentPlaneTarget(plan.IncDeg, plan.LanDeg, peKm, apKm, $"the orbit plane of {moon.Id}");
            return true;
        }

        // The site lies outside the moon's declination range. Of the two planes of the least inclination the site reaches, the one closer to the moon's own plane meets the moon at the smaller angle.
        LunarLaunchPlan best = null;
        foreach (bool southbound in new[] { false, true })
        {
            LunarLaunchInputs tilted = inputs with
            {
                Control = PlaneControl.Inclination,
                IncDeg = Math.Abs(plan.SiteLatDeg) + MoonPlaneLatitudeMarginDeg,
                Southbound = southbound,
            };
            double arrival = LunarLaunchPlanner.SoonestArrival(vehicle, moon, now, tilted);
            if (double.IsNaN(arrival))
                continue;
            LunarLaunchPlan candidate = LunarLaunchPlanner.Solve(vehicle, moon, now, tilted with { ArrivalTime = arrival });
            if (candidate.HasPlane && candidate.SiteReachesPlane
                && (best == null || candidate.TiltToMoonOrbitDeg < best.TiltToMoonOrbitDeg))
                best = candidate;
        }
        if (best == null)
            return false;
        target = new AscentPlaneTarget(best.IncDeg, best.LanDeg, peKm, apKm, $"the plane to {moon.Id}");
        return true;
    }

    #endregion

    #region Chain

    /// <summary>Called where the ascent completes. Only a craft with a moon picked starts the chain, with the moon, the periapsis and the capture choice as they stand then.</summary>
    private static void QueueMoonTransfer(Vehicle vehicle)
    {
        if (_s.MoonTransferId.Length == 0)
            return;
        ClearMoonChainBurn();
        _s.MoonChain = MoonChainStage.AfterAscent;
        _s.MoonChainMoonId = _s.MoonTransferId;
        _s.MoonChainPeKm = _s.MoonTransferPeKm;
        _s.MoonChainCapture = _s.MoonTransferCapture;
        _s.MoonChainAfter = SimNow() + MoonSettleS;
        _s.MoonChainCorrections = 0;
        _s.MoonChainStopRequested = false;
        // The launch plane has done its work, and the launch window step stays off while the chain runs.
        _s.PlaneTarget = null;
        GuidanceLog.Info(vehicle, $"ascent complete, the chain to {_s.MoonChainMoonId} waits for the release.");
    }

    /// <summary>
    /// One step of the chain. Runs in the Vehicle.PrepareWorker prefix, on the main thread before the vehicle workers start, as the stock deorbit node does (LoadStockDeorbitNode), so FlightComputer.AddBurn loads a node into this step's snapshot.
    /// </summary>
    private static void StepMoonTransfer(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        if (_s.MoonChain == MoonChainStage.Idle)
            return;
        // A fault ends the chain, because a stage that throws would plan again on every step.
        try
        {
            StepMoonChain(vehicle, orbit, parent);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"guidance-moon-chain-{vehicle.Id}:{ex.GetType().Name}",
                $"[AFC] The chain to {_s.MoonChainMoonId} failed on '{vehicle.Id}': {ex}");
            StopMoonChain(vehicle, "A FAULT IN THE PLAN, SEE THE LOG", disarm: true);
        }
    }

    private static void StepMoonChain(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        if (_s.MoonChainStopRequested)
        {
            StopMoonChain(vehicle, "STOPPED ON THE LAUNCH PAGE", disarm: true);
            return;
        }
        bool held = _s.Running || _s.LaunchArmed || _s.ControlAcquired || _s.FcResetPending
            || VehicleControlOwnership.HolderOf(vehicle) != ControlClaimant.None;
        if (_s.MoonChain == MoonChainStage.AfterAscent)
        {
            if (!held && SimNow() >= _s.MoonChainAfter)
                PlanMoonInjection(vehicle, orbit, parent);
            return;
        }
        if (held)
        {
            StopMoonChain(vehicle, "ANOTHER CONTROLLER HAS THE CRAFT", disarm: true);
            return;
        }
        Celestial moon = ChainMoon();
        if (moon is not IParentBody moonBody)
        {
            StopMoonChain(vehicle, $"{_s.MoonChainMoonId.ToUpperInvariant()} IS NOT IN THIS SYSTEM", disarm: true);
            return;
        }
        switch (_s.MoonChain)
        {
            case MoonChainStage.Injection:
            case MoonChainStage.Correction:
            case MoonChainStage.Capture:
                StepMoonBurn(vehicle, orbit, moon, moonBody);
                break;
            case MoonChainStage.Course:
                StepMoonCourse(vehicle, orbit, moon, moonBody);
                break;
            case MoonChainStage.Coast:
                StepMoonCoast(vehicle, orbit, moon, moonBody);
                break;
        }
    }

    private static void PlanMoonInjection(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        string moonName = _s.MoonChainMoonId.ToUpperInvariant();
        Celestial moon = FindMoon(parent, _s.MoonChainMoonId);
        string refusal = "";
        // IParentBody.GetAtmosphereRadius is zero for an airless body, so the mean radius is the floor there.
        double floor = Math.Max(parent.MeanRadius, parent.GetAtmosphereRadius());
        if (moon == null)
            refusal = $"{moonName} DOES NOT ORBIT {parent.Id.ToUpperInvariant()}";
        else if (orbit == null || !(orbit.Eccentricity < 1.0) || !(orbit.Periapsis > floor))
            refusal = "NO PARKING ORBIT ABOVE THE ATMOSPHERE";
        else if (CourseCorrector.BurnPlanned(vehicle))
            refusal = "A BURN IS ALREADY PLANNED";

        MoonInjectionPlanner.MoonInjection plan = default;
        double now = SimNow();
        if (refusal.Length == 0
            && !MoonInjectionPlanner.TryPlan(vehicle, moon, _s.MoonChainPeKm * 1000.0, new UniverseTime(now + MoonBurnLeadS), out plan, out string why))
            refusal = why;
        // A long burn ignites well ahead of its node. When the ignition of the first plan comes sooner than the lead, the node is planned again that much later.
        double halfBurn = refusal.Length == 0 ? 0.5 * MoonBurnSeconds(vehicle, plan.DvVlf.Length()) : 0.0;
        if (halfBurn > 0.0 && plan.BurnTime.Seconds() - halfBurn < now + MoonBurnLeadS
            && !MoonInjectionPlanner.TryPlan(vehicle, moon, _s.MoonChainPeKm * 1000.0, new UniverseTime(now + MoonBurnLeadS + halfBurn), out plan, out why))
            refusal = why;
        if (refusal.Length == 0)
            refusal = ArmMoonBurn(vehicle, plan.BurnTime, plan.DvVlf);
        if (refusal.Length > 0)
        {
            EndMoonChain(vehicle, $"{refusal}. NO INJECTION TO {moonName}.", warn: true);
            return;
        }
        _s.MoonChain = MoonChainStage.Injection;
        double minutes = (plan.BurnTime.Seconds() - SimNow()) / 60.0;
        double hours = (plan.Arrival.Seconds() - plan.BurnTime.Seconds()) / 3600.0;
        double peKm = (plan.PredictedPeriapsisRadius - ((IParentBody)moon).MeanRadius) / 1000.0;
        _s.Status = $"TO {moonName}: INJECTION {plan.DvVlf.Length():F0} M/S IN {MoonWait(plan.BurnTime.Seconds() - SimNow())}, STOCK AUTO FLIES IT. A CORRECTION FOLLOWS.";
        GuidanceLog.Info(vehicle, $"injection to {moon.Id}: node at {plan.BurnTime.Seconds():F1} s ({minutes:F1} min), "
            + $"delta-v {plan.DvVlf.Length():F1} m/s (impulsive {plan.ImpulsiveDvMs:F1}), side {plan.Side}, "
            + $"arrival after {hours:F1} h, predicted periapsis {peKm:F1} km, stock Auto armed.");
    }

    // The time the engines need for a burn at full throttle, from the phases stock plans its finite burns with (FiniteBurns.FullThrottlePhases). Zero without thrust.
    private static double MoonBurnSeconds(Vehicle vehicle, double dvMs)
    {
        double seconds = 0.0;
        foreach (FiniteBurnPhase phase in FiniteBurns.FullThrottlePhases(vehicle))
        {
            if (!(phase.Thrust > 0.0) || !(phase.MassFlowRate > 0.0) || !(phase.StartMass > 0.0))
                continue;
            double exhaust = phase.Thrust / phase.MassFlowRate;
            double needed = phase.StartMass * (1.0 - Math.Exp(-dvMs / exhaust)) / phase.MassFlowRate;
            if (needed <= phase.MaxSeconds)
                return seconds + needed;
            seconds += phase.MaxSeconds;
            dvMs -= exhaust * Math.Log(phase.StartMass / (phase.StartMass - phase.MassFlowRate * phase.MaxSeconds));
        }
        return seconds;
    }

    // CourseCorrector.TryAddNode loads the node in Manual. Vehicle.SetEnum then selects Auto as the burn mode button does, which MultiPass also uses for its next pass.
    // The RCS executor takes an Auto request for a burn that resolves to RCS, which is the case while no active engine has propellant, and leaves the mode in Manual. The chain flies no burn on thrusters, so it does not make that request.
    // Returns the refusal, empty when the node is loaded and armed.
    private static string ArmMoonBurn(Vehicle vehicle, UniverseTime burnTime, double3 dvVlf)
    {
        string refusal = CourseCorrector.TryAddNode(vehicle, burnTime, dvVlf, out Burn node);
        if (refusal.Length > 0)
            return refusal;
        FlightComputer fc = vehicle.FlightComputer;
        if (RcsExecutor.WouldExecuteRcs(vehicle))
            return "NO ACTIVE ENGINE HAS PROPELLANT, THE BURN STAYS PLANNED FOR THE BURN GAUGE";
        vehicle.SetEnum(FlightComputerBurnMode.Auto);
        if (fc.BurnMode != FlightComputerBurnMode.Auto)
            return "STOCK AUTO WAS REFUSED, THE BURN STAYS PLANNED FOR THE BURN GAUGE";
        _s.MoonChainNode = node;
        _s.MoonChainTarget = fc.Burn;
        _s.MoonChainNodeTime = node.ImpulseTime.Seconds();
        _s.MoonChainNodeDv = node.DeltaVVlf;
        _s.MoonChainManualSince = double.NaN;
        _s.MoonChainWarpAsked = false;
        return "";
    }

    // The armed burn, until stock reports it complete. FlightComputer.UpdateBurnTarget ends an Auto burn when the delta v to go no longer points along the target and writes Manual, and the kept BurnTarget still shows that after the node is gone, as the stock deorbit phase reads it (StockDeorbitComplete).
    private static void StepMoonBurn(Vehicle vehicle, Orbit orbit, Celestial moon, IParentBody moonBody)
    {
        FlightComputer fc = vehicle.FlightComputer;
        BurnTarget target = _s.MoonChainTarget;
        Burn node = _s.MoonChainNode;
        bool complete = target.DeltaVTargetCci.LengthSquared() > 0
            && float3.Dot(target.DeltaVToGoCci, target.DeltaVTargetCci) <= 0;
        if (complete)
        {
            if (fc.BurnMode == FlightComputerBurnMode.Manual)
                MoonBurnComplete(vehicle, orbit, moonBody);
            return;
        }

        double now = SimNow();
        string stop = "";
        if (!fc.BurnPlan.TryGetBurn(node))
            stop = "THE BURN WAS REMOVED";
        else if (Math.Abs(node.ImpulseTime.Seconds() - _s.MoonChainNodeTime) > 0.001 || (node.DeltaVVlf - _s.MoonChainNodeDv).Length() > 0.001)
            stop = "THE BURN WAS CHANGED";
        else if (fc.BurnPlan.BurnCount > 1 || !ReferenceEquals(fc.BurnPlan.FindFirstExecutableBurn(), node) || !ReferenceEquals(fc.Burn, target))
            stop = "ANOTHER BURN IS PLANNED";
        else if (now > _s.MoonChainNodeTime + Math.Max(0f, target.BurnDuration) + MoonBurnOverdueS)
            stop = "THE BURN DID NOT COMPLETE";
        else if (fc.BurnMode != FlightComputerBurnMode.Auto)
        {
            if (double.IsNaN(_s.MoonChainManualSince))
                _s.MoonChainManualSince = now;
            else if (now - _s.MoonChainManualSince > MoonManualGraceS)
                stop = "STOCK AUTO IS OFF (SWITCHED OFF, OR THE STAGE RAN DRY)";
        }
        else
            _s.MoonChainManualSince = double.NaN;
        if (stop.Length > 0)
        {
            StopMoonChain(vehicle, stop);
            return;
        }

        // A new BurnTarget has an EndOfTime ignition until a vehicle worker has updated it. The warp ends before stock Auto starts its turn, as the warp to the stock deorbit burn does.
        if (!_s.MoonChainWarpAsked && target.IgnitionTime != UniverseTime.EndOfTime && fc.BurnMode == FlightComputerBurnMode.Auto)
            AskMoonWarp(vehicle, StockDeorbitWarpTime(target));
    }

    private static void MoonBurnComplete(Vehicle vehicle, Orbit orbit, IParentBody moonBody)
    {
        MoonChainStage done = _s.MoonChain;
        double residual = _s.MoonChainTarget.DeltaVToGoCci.Length();
        // FlightComputer.EndAutoBurn removes a completed node only when another burn follows it. The chain plans its next burn only into an empty plan, so it removes its own completed node.
        FlightComputer fc = vehicle.FlightComputer;
        if (fc.BurnPlan.TryGetBurn(_s.MoonChainNode) && fc.BurnPlan.BurnCount == 1)
            fc.RemoveBurn(_s.MoonChainNode);
        ClearMoonChainBurn();
        GuidanceLog.Info(vehicle, $"chain to {moonBody.Id}: {done} burn complete, residual {residual:F3} m/s.");
        if (done == MoonChainStage.Capture)
        {
            double peKm = (orbit.Periapsis - moonBody.MeanRadius) / 1000.0;
            double apKm = (orbit.Apoapsis - moonBody.MeanRadius) / 1000.0;
            EndMoonChain(vehicle, orbit.Eccentricity < 1.0
                ? $"IN ORBIT AT {moonBody.Id.ToUpperInvariant()}: {peKm:F0} x {apKm:F0} KM."
                : $"CAPTURE BURN DONE, BUT THE ORBIT AT {moonBody.Id.ToUpperInvariant()} IS STILL OPEN.", warn: !(orbit.Eccentricity < 1.0));
            return;
        }
        _s.MoonChain = MoonChainStage.Course;
        _s.MoonChainAfter = SimNow() + CourseCorrector.SettleS;
        _s.Status = $"TO {moonBody.Id.ToUpperInvariant()}: BURN DONE, CHECKING THE COURSE.";
    }

    // After a burn: compare the periapsis on the craft's own flight plan with the request, and plan a correction when it is off.
    private static void StepMoonCourse(Vehicle vehicle, Orbit orbit, Celestial moon, IParentBody moonBody)
    {
        if (SimNow() < _s.MoonChainAfter)
            return;
        string moonName = moonBody.Id.ToUpperInvariant();
        if (CourseCorrector.BurnPlanned(vehicle))
        {
            StopMoonChain(vehicle, "A BURN IS PLANNED");
            return;
        }
        if (orbit.Parent.Id == moonBody.Id)
        {
            BeginMoonCoast(vehicle, moonBody);
            return;
        }

        double predicted = MoonInjectionPlanner.PredictedPeriapsisRadius(vehicle, moonBody);
        double predictedKm = (predicted - moonBody.MeanRadius) / 1000.0;
        string course = double.IsNaN(predicted) ? $"NO ENCOUNTER WITH {moonName}" : $"PERIAPSIS AT {moonName} {predictedKm:F0} KM";
        GuidanceLog.Info(vehicle, $"chain to {moonBody.Id}: course check after {_s.MoonChainCorrections} correction(s), "
            + (double.IsNaN(predicted) ? "no encounter on the flight plan." : $"periapsis {predictedKm:F1} km for {_s.MoonChainPeKm:F1} km requested."));
        if (CourseCorrector.OnCourse(predicted, moonBody, _s.MoonChainPeKm * 1000.0))
        {
            BeginMoonCoast(vehicle, moonBody);
            return;
        }
        if (_s.MoonChainCorrections >= CourseCorrector.MaxCorrections)
        {
            // A course that still clears the surface can be captured from. Anything else is the player's to fix.
            if (predicted > moonBody.GetNearSurfaceRadius())
                BeginMoonCoast(vehicle, moonBody, onCourse: false);
            else
                EndMoonChain(vehicle, $"{course} AFTER {_s.MoonChainCorrections} CORRECTIONS. THE CHAIN STOPS, CORRECT THE COURSE ON TRANSFER.", warn: true);
            return;
        }

        double now = SimNow();
        string refusal = CourseCorrector.TryPlan(vehicle, moon, moonBody, _s.MoonChainPeKm * 1000.0, now, out MoonInjectionPlanner.MoonInjection plan);
        if (refusal.Length == 0)
            refusal = ArmMoonBurn(vehicle, plan.BurnTime, plan.DvVlf);
        double lead = plan.BurnTime.Seconds() - now;
        if (refusal.Length > 0)
        {
            EndMoonChain(vehicle, $"{course}. NO CORRECTION: {refusal}.", warn: true);
            return;
        }
        _s.MoonChainCorrections++;
        _s.MoonChain = MoonChainStage.Correction;
        double newKm = (plan.PredictedPeriapsisRadius - moonBody.MeanRadius) / 1000.0;
        _s.Status = $"TO {moonName}: {course}. CORRECTION {plan.DvVlf.Length():F1} M/S IN {MoonWait(lead)}, STOCK AUTO FLIES IT.";
        GuidanceLog.Info(vehicle, $"chain to {moonBody.Id}: correction {_s.MoonChainCorrections} at {plan.BurnTime.Seconds():F1} s, "
            + $"delta-v {plan.DvVlf.Length():F2} m/s, {(plan.Side is { } side ? "aimed anew on side " + side : "side kept")}, predicted periapsis {newKm:F1} km, stock Auto armed.");
    }

    // The course is good enough. Without the capture the chain ends here, with the craft on its way.
    private static void BeginMoonCoast(Vehicle vehicle, IParentBody moonBody, bool onCourse = true)
    {
        string moonName = moonBody.Id.ToUpperInvariant();
        double predicted = vehicle.Orbit.Parent.Id == moonBody.Id
            ? vehicle.Orbit.Periapsis : MoonInjectionPlanner.PredictedPeriapsisRadius(vehicle, moonBody);
        double predictedKm = (predicted - moonBody.MeanRadius) / 1000.0;
        string course = onCourse ? $"ON COURSE, PERIAPSIS {predictedKm:F0} KM"
            : $"PERIAPSIS {predictedKm:F0} KM AFTER {_s.MoonChainCorrections} CORRECTIONS, NOT THE {_s.MoonChainPeKm:F0} KM ASKED FOR";
        if (!_s.MoonChainCapture)
        {
            EndMoonChain(vehicle, $"TO {moonName}: {course}.", warn: !onCourse);
            return;
        }
        _s.MoonChain = MoonChainStage.Coast;
        _s.MoonChainAfter = double.NaN;
        _s.MoonChainWarpAsked = false;
        _s.Status = $"TO {moonName}: {course}. THE CAPTURE BURN IS PLANNED IN ITS SPHERE.";
    }

    // The coast to the moon's sphere, and inside it the capture burn planned from the orbit as flown.
    private static void StepMoonCoast(Vehicle vehicle, Orbit orbit, Celestial moon, IParentBody moonBody)
    {
        string moonName = moonBody.Id.ToUpperInvariant();
        if (CourseCorrector.BurnPlanned(vehicle))
        {
            StopMoonChain(vehicle, "A BURN IS PLANNED");
            return;
        }
        double now = SimNow();
        if (orbit.Parent.Id != moonBody.Id)
        {
            if (MoonInjectionPlanner.SphereEntryTime(vehicle, moonBody) is not UniverseTime entry)
            {
                // The flight plan is rebuilt over several frames, so a missing encounter counts only once it lasts.
                if (double.IsNaN(_s.MoonChainAfter))
                    _s.MoonChainAfter = now + CourseCorrector.SettleS;
                else if (now >= _s.MoonChainAfter)
                    StopMoonChain(vehicle, $"NO ENCOUNTER WITH {moonName} ANY MORE");
                return;
            }
            _s.MoonChainAfter = double.NaN;
            if (!_s.MoonChainWarpAsked)
                AskMoonWarp(vehicle, entry.Seconds() + MoonSphereMarginS);
            return;
        }

        // Inside the sphere. The settle time starts at the first step here.
        if (double.IsNaN(_s.MoonChainAfter) || _s.MoonChainAfter > now + MoonSettleS)
            _s.MoonChainAfter = now + MoonSettleS;
        if (now < _s.MoonChainAfter)
            return;
        double available = vehicle.Parts?.PerformanceSequences?.TotalDeltaV ?? 0.0;
        string refusal = MoonInjectionPlanner.TryPlanCapture(vehicle, moonBody, available, new UniverseTime(now + MoonBurnLeadS),
            out MoonInjectionPlanner.MoonCapture plan, out string why) ? ArmMoonBurn(vehicle, plan.BurnTime, plan.DvVlf) : why;
        if (refusal.Length > 0)
        {
            EndMoonChain(vehicle, $"{refusal}. NO CAPTURE AT {moonName}.", warn: true);
            return;
        }
        _s.MoonChain = MoonChainStage.Capture;
        double peKm = (plan.PeriapsisRadius - moonBody.MeanRadius) / 1000.0;
        double apKm = (plan.ApoapsisRadius - moonBody.MeanRadius) / 1000.0;
        double hours = (plan.BurnTime.Seconds() - now) / 3600.0;
        _s.Status = $"AT {moonName}: CAPTURE {plan.DvVlf.Length():F0} M/S IN {MoonWait(plan.BurnTime.Seconds() - now)} TO {peKm:F0} x {apKm:F0} KM, STOCK AUTO FLIES IT."
            + (plan.Circular ? "" : " NOT ENOUGH DELTA-V FOR A CIRCLE.");
        GuidanceLog.Info(vehicle, $"chain to {moonBody.Id}: capture node at {plan.BurnTime.Seconds():F1} s, delta-v {plan.DvVlf.Length():F1} m/s "
            + $"of {available:F0} m/s left, predicted orbit {peKm:F1} x {apKm:F1} km, stock Auto armed.");
    }

    private static string MoonWait(double seconds)
        => seconds < 7200.0 ? $"{seconds / 60.0:F0} MIN" : $"{seconds / 3600.0:F1} H";

    // The chain asks for each warp once, and only for the craft in focus, because a warp moves the whole game. Universe.AutoWarpTo is the call the stock warp to the next burn ends in.
    // A warp of its own that still runs is replaced, and while any other warp runs the chain waits with its request.
    private static void AskMoonWarp(Vehicle vehicle, double toSimSec)
    {
        bool ownWarpRuns = double.IsFinite(_s.MoonChainWarpTime) && Universe.AutoWarpTime == new UniverseTime(_s.MoonChainWarpTime);
        if (Universe.IsAutoWarpActive && !ownWarpRuns)
            return;
        _s.MoonChainWarpAsked = true;
        if (!ReferenceEquals(vehicle, Program.ControlledVehicle) || !double.IsFinite(toSimSec) || toSimSec <= SimNow() + WarpLeadTime)
            return;
        Universe.AutoWarpTo(new UniverseTime(toSimSec));
        _s.MoonChainWarpTime = toSimSec;
    }

    private static void StopOwnedMoonWarp()
    {
        if (double.IsFinite(_s.MoonChainWarpTime) && Universe.IsAutoWarpActive
            && Universe.AutoWarpTime == new UniverseTime(_s.MoonChainWarpTime))
            Universe.AutoWarpStop(true);
        _s.MoonChainWarpTime = double.NaN;
    }

    private static void ClearMoonChainBurn()
    {
        _s.MoonChainNode = null;
        _s.MoonChainTarget = null;
        _s.MoonChainNodeTime = double.NaN;
        _s.MoonChainNodeDv = default;
        _s.MoonChainManualSince = double.NaN;
        _s.MoonChainWarpAsked = false;
        _s.MoonChainWarpTime = double.NaN;
    }

    // The player or another controller took over. The node and the engine stay as they are, because they are the player's, and so does the burn mode unless the stop asks for the disarm.
    private static void StopMoonChain(Vehicle vehicle, string reason, bool disarm = false)
    {
        string burn = disarm && DisarmMoonBurn(vehicle) ? " THE BURN STAYS PLANNED, AUTO IS OFF." : "";
        EndMoonChain(vehicle, $"CHAIN TO {_s.MoonChainMoonId.ToUpperInvariant()} STOPPED: {reason}.{burn}", warn: true);
    }

    // Takes the Auto of the chain's own loaded burn back, and says whether it did. A burn that has started stays with stock, because ending it is an engine command.
    // Guidance that took the craft holds the mode in Manual and would give the Auto back at its release (RestoreBurnMode), so that give-back is dropped instead.
    private static bool DisarmMoonBurn(Vehicle vehicle)
    {
        BurnTarget target = _s.MoonChainTarget;
        if (target == null || !ReferenceEquals(vehicle.FlightComputer.Burn, target))
            return false;
        if (_s.ForcedBurnManual && ReferenceEquals(_s.ForcedBurnTarget, target))
        {
            _s.ForcedBurnManual = false;
            _s.ForcedBurnTarget = null;
            return true;
        }
        bool started = target.Throttle.HasValue || SimNow() >= target.IgnitionTime.Seconds();
        return !started && StockBurnMode.HoldManual(vehicle);
    }

    // The one way out of the chain, finished or not. THEN goes back to its default with it, so the next ascent of this craft does not start another chain by itself.
    private static void EndMoonChain(Vehicle vehicle, string status, bool warn = false)
    {
        GuidanceLog.Info(vehicle, $"chain to {_s.MoonChainMoonId} ended in stage {_s.MoonChain}: {status}");
        StopOwnedMoonWarp();
        ClearMoonChainBurn();
        _s.MoonChain = MoonChainStage.Idle;
        _s.MoonChainStopRequested = false;
        _s.MoonChainAfter = double.NaN;
        _s.MoonTransferId = "";
        _s.MoonTransferCapture = false;
        _s.PlaneTarget = null;
        if (warn)
            _s.WarnStatus(status);
        else
            _s.Status = status;
    }

    #endregion
}
