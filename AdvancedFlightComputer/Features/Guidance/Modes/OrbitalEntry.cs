#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Threading;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;
using Brutal.Numerics;
using KSA;

/// <summary>
/// ORBITAL ENTRY GUIDANCE: from the air's first touch to a hand-over at about Mach 2, belly first at the pitch-trim angle of attack, banking to fly the planner's vertical lift over drag.
///
///  PLAN. The planner (Scvx/OrbitalEntry/EntryPlanner) chooses vertical L/D against energy to reach the target's range inside the heating, dynamic-pressure and load limits, with the least heat load. It runs on its own thread and is re-solved every OrbitalEntryResolveS from the craft's state, warm from the plan before.
///  TRACK. The tracker (EntryTracker) flies the plan with drag and sink-rate feedback, turns vertical L/D into bank from the measured L/D, reverses the bank's side when the heading error to the site leaves a band, and may bank further than the plan does (to the tracker's limit) to absorb what the plan did not foresee.
///  ATTITUDE. The angle of attack is the pitch-trim one at the current Mach, from a belly-first sweep of the craft taken at engage (KsaAeroSweep, bellyFirst). The nose goes at that angle above the flight path towards the lift, the belly axis faces the air, and the bank turns both about the velocity.
///  END. At the hand-over energy the craft holds its entry attitude, wings level, until the player takes it; how it flies out of the plan (a belly-flop landing or an aircraft-like approach) is for later.
/// </summary>
public static partial class GuidanceWindow
{
    public enum OrbitalEntryPhase { Idle, Sweeping, Entry, Holding, Bellyflop }

    /// <summary>After the hand-over BELLYFLOP flies (FLIP LANDING, on a craft with an engine to land on).</summary>
    internal static bool EntryFlips => _s.OrbitalEntryBellyflop && _s.OrbitalEntryFlip;

    /// <summary>After the hand-over RETRO BURN takes the craft (the 3-DOF), which turns it engine first.</summary>
    internal static bool EntryRetro => _s.OrbitalEntryBellyflop && !_s.OrbitalEntryFlip;

    internal static bool OrbitalEntryLive => _s.OrbitalEntry.Phase is OrbitalEntryPhase.Sweeping or OrbitalEntryPhase.Entry or OrbitalEntryPhase.Holding or OrbitalEntryPhase.Bellyflop;

    private static bool OrbitalEntryLiveFor(VehicleAutopilotState s)
        => s.OrbitalEntry.Phase is OrbitalEntryPhase.Sweeping or OrbitalEntryPhase.Entry or OrbitalEntryPhase.Holding or OrbitalEntryPhase.Bellyflop;

    private static readonly EntryPlanner _entryPlanner = new();

    /// <summary>The config the planner and tracker fly with, from the craft's settings.</summary>
    private static EntryConfig OrbitalEntryConfig() => new()
    {
        HeatingRateMax = _s.OrbitalEntryHeatMaxKWm2 * 1000.0,
        DynamicPressureMax = _s.OrbitalEntryQMaxKPa * 1000.0,
        LoadFactorMax = _s.OrbitalEntryGMax,
        BankMinDeg = _s.OrbitalEntryBankMinDeg,
        BankMaxDeg = _s.OrbitalEntryBankMaxDeg,
        OverbankDeg = Math.Max(_s.OrbitalEntryOverbankDeg, _s.OrbitalEntryBankMaxDeg),
        RollRateDegS = Math.Max(_s.OrbitalEntryRollRateDegS, 0.5),
        StartDrag = _s.OrbitalEntryStartDragMs2,
        HandoverAltitude = _s.OrbitalEntryHandoverAltKm * 1000.0,
        HandoverMach = _s.OrbitalEntryHandoverMach,
        TrackOmega = Math.Max(_s.OrbitalEntryTrackOmega, 0.01),
        GainDragFloor = Math.Max(_s.OrbitalEntryGainDragFloor, 0.1),
        MeasureSmoothS = Math.Max(_s.OrbitalEntryMeasureSmoothS, 0.0),
        TrackRollRateDegS = Math.Max(_s.OrbitalEntryTrackRollRateDegS, 0.5),
        CorridorWeight = Math.Max(_s.OrbitalEntryCorridorWeight, 0.0),
        ReversalBandDeg = Math.Max(_s.OrbitalEntryReversalBandDeg, 0.5),
        EngageInAtmosphere = _s.OrbitalEntryEngageInAir,
        CommandSmoothS = Math.Max(_s.OrbitalEntrySmoothS, 0.0),
    };

    /// <summary>Take the craft for orbital entry. Draw code only sets OrbitalEntry.EngagePending; this runs from the step, ahead of the claim.</summary>
    private static bool EngageOrbitalEntry(Vehicle vehicle, double now)
    {
        var st = _s.OrbitalEntry;
        if (!vehicle.IsControllable)
            return RefuseOrbitalEntry(vehicle, vehicle.NoControlTooltip.ToString());
        IParentBody parent = vehicle.Orbit?.Parent;
        if (parent?.GetPhysicalAtmosphere() == null)
            return RefuseOrbitalEntry(vehicle, "NO ATMOSPHERE.");
        if (vehicle.ComputeLiveThrust() > 0.0)
            return RefuseOrbitalEntry(vehicle, "AN ENGINE IS BURNING.");
        if (!KsaAeroSweep.TryStart(vehicle, parent, now, out KsaAeroSweep.Job job, out string error, bellyFirst: true, bellySide: EntryBellySideWanted()))
            return RefuseOrbitalEntry(vehicle, "AERO SWEEP FAILED: " + error.ToUpperInvariant() + ".");

        if (IsStockDeorbitPhase(_s.LandingPhase))
            StopGuidance(vehicle, StopTarget.Landing);
        ClaimVehicle(GuidanceMode.OrbitalEntry, vehicle);
        _s.Engage = true;
        _s.OrbitalEntryBellyflop = LandEngineOnCraft(vehicle);
        st.Reset();
        st.AeroJob = job;
        st.Tracker = new EntryTracker(OrbitalEntryConfig());
        st.LastStep = now;
        st.Phase = OrbitalEntryPhase.Sweeping;
        st.SetStatus("MEASURING THE CRAFT BELLY FIRST.");
        GuidanceLog.Info(vehicle, "Orbital entry guidance engaged; belly-first aero sweep started.");
        if (_s.EntryLogging)
        {
            EntryConfig c = OrbitalEntryConfig();
            string setup = FormattableString.Invariant(
                $"limits: q {c.DynamicPressureMax / 1000:F1} kPa, load {c.LoadFactorMax:F2} g, heating {c.HeatingRateMax / 1000:F0} kW/m2 (nose {_s.OrbitalEntryNoseRadiusM:F2} m); bank plan {c.BankMinDeg:F0}-{c.BankMaxDeg:F0}, tracker to {c.OverbankDeg:F0}, reversal band {c.ReversalBandDeg:F1} deg, corridor margin {c.CorridorWeight:F2}, roll {c.RollRateDegS:F1} deg/s; RCS {(_s.OrbitalEntryUseRcs ? FormattableString.Invariant($"on, phase plane band {_s.OrbitalEntryRcsDeadbandDeg:F1} deg") : "off")}; hand-over {c.HandoverAltitude / 1000:F1} km Mach {c.HandoverMach:F2}; start drag {c.StartDrag:F2}; re-solve {_s.OrbitalEntryResolveS:F2} s; site {_s.SiteLatDeg:F4} {_s.SiteLonDeg:F4}; mass {vehicle.TotalMass:F0} kg");
            if (OrbitalEntryLog.Start(st, vehicle.Id, (parent as Astronomical)?.Id ?? "", setup))
                GuidanceLog.Info(vehicle, "Orbital entry log: " + OrbitalEntryLog.Path);
        }
        return true;
    }

    private static bool RefuseOrbitalEntry(Vehicle vehicle, string reason)
    {
        string status = "ORBITAL ENTRY REFUSED: " + reason;
        if (_s.OrbitalEntry.Status != status)
            GuidanceLog.Info(vehicle, status);
        _s.OrbitalEntry.SetStatus(status, warn: true);
        return false;
    }

    /// <summary>Stop orbital entry: its solver and attitude let go, the engine left as it is.</summary>
    /// <summary>
    /// The hand-over to what follows the entry, other than bellyflop (which takes the craft at its own first plan, StepBellyflop).
    ///  RETRO BURN: the 3-DOF is asked for once, as the boostback's glide asks for it (ThreeDofEngagePending): its claim ends the entry, while it waits for its sweep or its landing engines (the engage asks again itself) the entry holds its attitude, and after a refusal the entry goes on holding. It turns the craft engine first itself.
    ///  PARACHUTES (a craft with no engine to land on): armed once, as the part menu's ARM does; each opens by itself at its deploy altitude, and the entry lets go of the craft once one has.
    /// </summary>
    private static void StepEntryHandOver(Vehicle vehicle, OrbitalEntryState st, double mach, double alt)
    {
        if (_s.OrbitalEntryBellyflop && !LandEngineOnCraft(vehicle))
            _s.OrbitalEntryBellyflop = false;
        if (EntryRetro)
        {
            if (st.RetroAsked)
                return;
            GuidanceLog.Info(vehicle, FormattableString.Invariant($"Orbital entry hands the craft to RETRO BURN at Mach {mach:F2}, {alt / 1000:F1} km, {st.RangeM / 1000:F1} km to go."));
            st.RetroAsked = true;
            _s.ThreeDofEngagePending = true;
            return;
        }
        if (_s.OrbitalEntryBellyflop)
            return;
        if (!st.ChutesArmed)
        {
            int count = KsaParachutes.Count(vehicle);
            if (count == 0)
                return;
            int armed = KsaParachutes.ArmAll(vehicle);
            st.ChutesArmed = true;
            GuidanceLog.Info(vehicle, FormattableString.Invariant($"Orbital entry hand-over: {armed} of {count} parachutes armed at Mach {mach:F2}, {alt / 1000:F1} km; each opens at its own deploy altitude."));
        }
        if (KsaParachutes.AnyOpen(vehicle))
        {
            StopOrbitalEntry(vehicle, FormattableString.Invariant($"PARACHUTES OPEN AT {alt / 1000:F1} KM. CONTROL RELEASED."));
            return;
        }
    }

    private static void StopOrbitalEntry(Vehicle vehicle, string why)
    {
        var st = _s.OrbitalEntry;
        if (_s.OwnAttitudeFlying && _s.OwnAttitudeVehicle is { } flown)
            StopOwnAttitude(flown);
        // Touchdown's standby for the flip goes with it.
        if (st.Bellyflop.TouchdownStandby)
        {
            StopSixDofStandby();
            _s.SixDofCoastPending = false;
        }
        OrbitalEntryLog.Stop(st);
        st.Reset();
        st.Phase = OrbitalEntryPhase.Idle;
        st.SetStatus(why);
        _s.HasCommand = false;
        _s.ReleaseWithoutEngineCut = true;
        if (vehicle != null)
            GuidanceLog.Info(vehicle, "Orbital entry guidance stopped: " + why);
    }

    /// <summary>The release path's reset: no log, no status change.</summary>
    private static void ResetOrbitalEntry()
    {
        OrbitalEntryLog.Stop(_s.OrbitalEntry);
        _s.OrbitalEntry.Reset();
        _s.OrbitalEntry.Phase = OrbitalEntryPhase.Idle;
        _s.OrbitalEntry.EngagePending = false;
    }

    /// <summary>One step of orbital entry guidance, from ApplyAutopilot.</summary>
    private static void StepOrbitalEntry(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        var st = _s.OrbitalEntry;
        if (!OrbitalEntryLive)
            return;
        if (st.StopPending)
        {
            StopOrbitalEntry(vehicle, "ORBITAL ENTRY STOPPED.");
            return;
        }
        double now = SimNow();
        double dt = double.IsNaN(st.LastStep) ? 0.0 : Math.Clamp(now - st.LastStep, 0.0, 1.0);
        st.LastStep = now;

        if (st.Phase == OrbitalEntryPhase.Sweeping)
        {
            _s.HasCommand = false;
            if (st.AeroJob == null || !st.AeroJob.TryTake(out KsaAeroSweep.Result aero, out string error))
                return;
            st.AeroJob = null;
            if (aero?.Belly == null || aero.Atmosphere == null)
            {
                StopOrbitalEntry(vehicle, "ORBITAL ENTRY REFUSED: NO BELLY-FIRST AERO (" + (error.Length > 0 ? error.ToUpperInvariant() : "NO AIR") + ").");
                return;
            }
            st.Aero = aero;
            if (!BuildOrbitalEntryTrim(st, out string why))
            {
                StopOrbitalEntry(vehicle, "ORBITAL ENTRY REFUSED: " + why);
                return;
            }
            st.Worker = new Ksa3DofWorker();
            st.Phase = OrbitalEntryPhase.Entry;
            // The belly-first body and surface tables, on the side the trim chose: the same sweep as the nose- and engine-first ones, so the attitude controller and the fins' and flaps' allocator read the air belly first as they read it the other ways. Shared with touchdown guidance (ResampleBoostbackAero): the craft's own sweep is taken where it already has them on this side, and this one becomes the craft's when it lands.
            _s.AeroBellySide = st.BellyIndex;
            KsaAeroSweep.Result shared = _s.Aero;
            if (shared?.SphereBodyAero != null && shared.BellyBodyAero != null && shared.BellySide == st.BellyIndex
                && ReferenceEquals(shared.Parent, parent) && !SweepStale(shared, vehicle))
            {
                st.Aero = shared;
                st.TablesReady = true;
                GuidanceLog.Info(vehicle, $"Orbital entry: belly-first tables on {st.BellyAxisName} taken from the craft's own sweep, {shared.Surfaces?.Length ?? 0} control surfaces.");
            }
            else if (KsaAeroSweep.TryStart(vehicle, parent, now, out KsaAeroSweep.Job tables, out string tablesError, bellyFirst: true, bellySide: st.BellyIndex))
                st.TablesJob = tables;
            else
                GuidanceLog.Info(vehicle, "Orbital entry: the belly-first tables could not be swept (" + tablesError + "); the attitude holds without them.");
            GuidanceLog.Info(vehicle, FormattableString.Invariant(
                $"Orbital entry: belly axis {st.BellyAxisName}, {st.Trim.TrimmedRows}/{st.Trim.Mach.Length} Mach rows trimmed, alpha {st.Trim.AlphaAt(20):F0} deg at Mach 20, {st.Trim.AlphaAt(5):F0} at Mach 5, L/D {st.Trim.LiftToDragAt(20):F2} / {st.Trim.LiftToDragAt(5):F2}."));
            var rows = new System.Text.StringBuilder();
            for (int i = 0; i < st.Trim.Mach.Length; i++)
            {
                string residualText = !st.Trim.Trimmed[i] && st.Trim.ResidualCm is { } resCm && i < resCm.Length
                    ? FormattableString.Invariant($" Cm {resCm[i]:F4}")
                    : "";
                rows.Append(FormattableString.Invariant($" M{st.Trim.Mach[i]:0.#}: {st.Trim.AlphaDeg[i]:F0}{(st.Trim.Trimmed[i] ? "" : "*")} Cd {st.Trim.Cd[i]:F2} L/D {st.Trim.LiftToDrag[i]:F2}{residualText};"));
            }
            GuidanceLog.Info(vehicle, "Orbital entry schedule (alpha deg from nose-first; * no trim, least moment instead, with the Cm left):" + rows);
        }

        // The belly-first tables, once swept, replace the first sweep's for the attitude controller. The trim stays the first sweep's.
        if (st.TablesJob != null && st.TablesJob.TryTake(out KsaAeroSweep.Result swept, out string sweptError))
        {
            st.TablesJob = null;
            if (swept?.BellyBodyAero != null)
            {
                st.Aero = swept;
                st.TablesReady = true;
                // The craft's sweep too, for touchdown guidance and whatever flies after (ResampleBoostbackAero).
                _s.Aero = swept;
                _s.AeroJob = null;
                GuidanceLog.Info(vehicle, FormattableString.Invariant(
                    $"Orbital entry: belly-first tables on {st.BellyAxisName} swept, {swept.Surfaces?.Length ?? 0} control surfaces so far."));
            }
            else
                GuidanceLog.Info(vehicle, "Orbital entry: the belly-first table sweep gave nothing (" + sweptError + ").");
        }

        // Once the flaps' belly-first tables are in, the schedule is redone with their reach - only while TRIM AoA is on: off, ENTRY AoA is flown throughout.
        if (_s.OrbitalEntryTrimAoa && st.TablesReady && !st.FlapTrimDone && st.Aero.Surfaces is { Length: > 0 })
            using (new AdvancedFlightComputer.Core.PerfTracker.Scope("Entry.RetrimWithFlaps"))
                RetrimWithFlaps(vehicle, st);

        // ---- where the craft is ----
        double3 r = orbit.StateVectors.PositionCci;
        double3 vRel = orbit.StateVectors.VelocityCci - double3.Cross(parent.GetAngularVelocityCci(), r);
        double rMag = r.Length(), vMag = vRel.Length();
        if (vMag < 1.0)
            return;
        double3 rHat = r / rMag, vHat = vRel / vMag;
        double R0 = parent.MeanRadius;
        double alt = rMag - R0;
        double gamma = Math.Asin(Math.Clamp(double3.Dot(vHat, rHat), -1.0, 1.0));
        double3 siteHat = double3.Normalize(SiteDirCciAt(parent, 0));
        double rangeRad = Math.Acos(Math.Clamp(double3.Dot(rHat, siteHat), -1.0, 1.0));
        double3 leftHat = double3.Cross(r, vRel).NormalizeOrZero();
        double crossRad = Math.Asin(Math.Clamp(double3.Dot(leftHat, siteHat), -1.0, 1.0));
        // The site's bearing off the ground track, + to the left: the angle the reversals are kept within.
        double3 trackHat = (vRel - double3.Dot(vRel, rHat) * rHat).NormalizeOrZero();
        double3 toSite = (siteHat - double3.Dot(siteHat, rHat) * rHat).NormalizeOrZero();
        double headingErr = Math.Atan2(double3.Dot(double3.Cross(trackHat, toSite), rHat), double3.Dot(trackHat, toSite));
        st.HeadingErrorDeg = headingErr * 180.0 / Math.PI;
        // Range to go ALONG the ground track, signed: negative once the site is behind. The planner is longitudinal, so it plans this, and the crossrange is the reversal band's.
        double3 headHat = (vHat - double3.Dot(vHat, rHat) * rHat).NormalizeOrZero();
        double downRad = Math.Atan2(double3.Dot(siteHat, headHat), double3.Dot(siteHat, rHat));
        double greatCircleRad = rangeRad;
        rangeRad = downRad;

        var atm = st.Aero.Atmosphere;
        double liveA = vehicle.PhysicsEnvironment.SpeedOfSound;
        double mach = liveA > 1.0 ? vMag / liveA : atm.Mach(vMag, alt);
        st.SpeedMs = vMag; st.AltitudeM = alt; st.RangeM = rangeRad * R0; st.CrossrangeM = crossRad * R0; st.GreatCircleM = greatCircleRad * R0;
        st.HdotMs = vMag * Math.Sin(gamma); st.Mach = mach;

        // ---- the air the craft is in: the game's own aero force on it now ----
        double drag = 0.0, ld = st.Trim.LiftToDragAt(mach);
        bool liveAero;
        double3 forceBody;
        st.AeroAccelKnown = false;
        using (new AdvancedFlightComputer.Core.PerfTracker.Scope("Entry.LiveAero"))
            liveAero = KsaAeroSweep.TryLiveAero(vehicle, out forceBody, out _, dt);
        if (liveAero)
        {
            double3 f = forceBody.Transform(KsaFrameBridge.BodyToCci(vehicle));
            double m = Math.Max(vehicle.TotalMass, 1.0);
            st.AeroAccelCci = f / m;
            st.AeroAccelKnown = true;
            drag = Math.Max(-double3.Dot(f, vHat) / m, 0.0);
            double liftAccel = (f + drag * m * vHat).Length() / m;
            if (drag > 0.3)
                ld = liftAccel / drag;
        }
        st.DragMs2 = drag; st.LiftToDrag = ld;

        EntryModel nominal = OrbitalEntryModel(vehicle, parent, st, 1.0, 1.0).WithRotationAt(parent, r, vRel);
        double modelDrag = nominal.DragAccel(alt, vMag);
        // The scales are of the entry's own model at its angle of attack: under bellyflop the craft flies belly first, which that model does not describe, so they stand still (bellyflop measures its own, BellyflopState.ForceScale).
        if (drag > 0.3 && modelDrag > 0.1 && st.Phase != OrbitalEntryPhase.Bellyflop)
        {
            double k = Math.Clamp(dt / 5.0, 0.0, 1.0);
            st.DensityScale += k * (Math.Clamp(drag / modelDrag, 0.5, 2.0) - st.DensityScale);
            double ldModel = st.Trim.LiftToDragAt(mach);
            if (Math.Abs(ldModel) > 0.05)
                st.LiftToDragScale += k * (Math.Clamp(ld / ldModel, 0.5, 1.5) - st.LiftToDragScale);
        }

        EntryConfig cfg = OrbitalEntryConfig();
        double e = EntryDynamics.Energy(rMag / R0, vMag / nominal.SpeedUnit);
        // The coast's own clock: the seconds to the plan's start, counted down between the predictions.
        if (!double.IsNaN(st.PredictedStartS))
            st.PredictedStartS = Math.Max(0, st.PredictedStartS - dt);
        st.Energy = e;
        double ef = EntryPlanner.FinalEnergy(nominal, cfg);

        // ---- plans: take what the worker published, ask for the next ----
        if (Interlocked.Exchange(ref st.Pending, null) is { } res)
        {
            st.Solves++;
            if (res.Plan != null)
                OrbitalEntryLog.Plan(st, st.Solves, now, res.Plan);
            string solveText = res.Plan == null ? "no plan: " + res.Why : FormattableString.Invariant(
                $"{(res.Plan.Converged ? "converged" : "REFUSED " + res.Why)}, {res.Plan.Iterations} it, {res.Plan.SolveMs:F0} ms, miss {res.Plan.TerminalRangeErrorM / 1000:F1} km, defect {res.Plan.MaxDefect:F3}, from {res.Plan.S[0] * R0 / 1000:F0} km to go at {res.Plan.SpeedMs[0]:F0} m/s, corridor {res.Plan.RangeShortM[0] / 1000:F0}-{res.Plan.RangeLongM[0] / 1000:F0} km{(res.Plan.Cold ? ", cold" : "")}");
            if (res.Plan == null || !res.Plan.Converged)
                GuidanceLog.Info(vehicle, $"Orbital entry solve {st.Solves}: {solveText}.");
            else
                GuidanceLog.Debug(vehicle, $"Orbital entry solve {st.Solves}: {solveText}.");
            if (res.Plan != null && (res.Plan.Converged || st.Plan == null))
            {
                st.Plan = res.Plan;
                st.PlanAt = now;
                st.LastSolveMs = res.Plan.SolveMs;
                if (!res.Plan.Converged)
                    st.LastRefusal = res.Why;
            }
            else
            {
                st.Refusals++;
                st.LastRefusal = res.Why.Length > 0 ? res.Why : "no plan";
            }
        }
        st.DragModelMs2 = modelDrag;
        if (st.Phase == OrbitalEntryPhase.Entry && e < ef && now >= st.NextSolveAt && st.Worker != null && !st.Worker.Busy)
        {
            using var _queue = new AdvancedFlightComputer.Core.PerfTracker.Scope("Entry.QueueSolve");
            // BELLYFLOP: the entry aims at the ground through bellyflop's predicted fall from the hand-over.
            if (EntryFlips)
            {
                UpdateBellyflopRangeTarget(vehicle, parent, st);
                cfg.TerminalRangeM = st.Bellyflop.TerminalRangeM;
                cfg.TerminalRangeShortM = st.Bellyflop.TerminalShortM;
                cfg.TerminalRangeLongM = st.Bellyflop.TerminalLongM;
            }
            // RETRO BURN: the hand-over short of the site by the ground the retro burn's glide covers, a fixed estimate (RETRO AIM SHORT).
            else if (EntryRetro)
                cfg.TerminalRangeM = Math.Max(_s.OrbitalEntryRetroAimShortKm, 0.0) * 1000.0;
            QueueOrbitalEntrySolve(vehicle, parent, st, cfg, nominal, e, rMag, vMag, gamma, rangeRad, drag, now);
        }

        // ---- hand-over ----
        if (st.Phase == OrbitalEntryPhase.Entry && e >= ef)
        {
            st.Phase = OrbitalEntryPhase.Holding;
            // Past the hand-over before the entry ever planned: engaged in the bellyflop's own air, so bellyflop takes the craft at its first plan (BELLYFLOP DIRECT).
            if (st.Solves == 0 && st.Plan == null && EntryFlips)
            {
                st.BellyflopDirect = true;
                GuidanceLog.Info(vehicle, FormattableString.Invariant(
                    $"BELLYFLOP DIRECT: engaged past the hand-over, at Mach {mach:F2} and {alt / 1000:F1} km, {st.RangeM / 1000:F1} km to go: no entry, bellyflop takes the craft at its first plan; the attitude it has is held until then."));
            }
            GuidanceLog.Info(vehicle, FormattableString.Invariant(
                $"Orbital entry hand-over: Mach {mach:F2} at {alt / 1000:F1} km, {st.RangeM / 1000:F1} km to go, {st.CrossrangeM / 1000:F1} km across{(EntryFlips ? FormattableString.Invariant($" (bellyflop's fall {st.Bellyflop.TerminalRangeM / 1000:F1} km)") : "")}."));
        }

        // ---- after the hand-over: RETRO BURN takes the craft, or the parachutes are armed ----
        if (st.Phase == OrbitalEntryPhase.Holding && !st.BellyflopDirect)
            StepEntryHandOver(vehicle, st, mach, alt);
        if (st.Phase == OrbitalEntryPhase.Idle)
            return;

        // ---- bellyflop: solving in the background, taking over at the hand-over ----
        using (new AdvancedFlightComputer.Core.PerfTracker.Scope("Entry.Bellyflop"))
            StepBellyflop(vehicle, parent, st, mach, now, dt);

        // ---- bank ----
        double bankDeg;
        if (st.Phase == OrbitalEntryPhase.Bellyflop)
        {
            // Bellyflop flies the craft: no bank, no tracker. Its status is its own (StepBellyflop).
            bankDeg = 0.0;
        }
        else if (st.Phase == OrbitalEntryPhase.Holding && st.BellyflopDirect)
        {
            bankDeg = 0.0;
            st.Tracker.Reset();
            st.SetStatus(FormattableString.Invariant($"BELLYFLOP DIRECT: MACH {mach:F1} AT {alt / 1000:F1} KM, {st.RangeM / 1000:F1} KM TO GO. HOLDING THE ATTITUDE UNTIL BELLYFLOP'S FIRST PLAN."));
        }
        else if (st.Phase == OrbitalEntryPhase.Holding)
        {
            // Wings level, still at the trim angle: the hand-over keeps the entry attitude and slews bank smoothly to zero.
            bankDeg = st.Tracker.SlewToLevel(dt);
            string next = st.ChutesArmed ? "PARACHUTES ARMED: THEY OPEN AT THEIR DEPLOY ALTITUDE."
                : st.RetroAsked ? "HOLDING ENTRY ATTITUDE UNTIL RETRO BURN TAKES THE CRAFT."
                : "HOLDING ENTRY ATTITUDE.";
            st.SetStatus(FormattableString.Invariant($"HAND-OVER: MACH {mach:F1} AT {alt / 1000:F1} KM, {st.RangeM / 1000:F0} KM TO GO. ") + next);
        }
        else
        {
            int side = st.Tracker.Side;
            using var _tracker = new AdvancedFlightComputer.Core.PerfTracker.Scope("Entry.Tracker");
            var outp = st.Tracker.Update(new EntryTracker.Input(dt, e, drag, st.HdotMs, ld, atm.ScaleHeightAt(Math.Max(alt, 0.0)),
                                                                headingErr, vMag / nominal.SpeedUnit, st.Plan, alt < atm.TopAltitude,
                                                                st.AttitudeOwn ? st.Attitude.AirAxisAccelRadS2 * 180.0 / Math.PI : double.NaN));
            if (side != 0 && outp.Side != side) st.Reversals++;
            st.Last = outp;
            bankDeg = outp.BankDeg;
            string status = outp.Status;
            // Where the site sits against the corridor at this energy: the TRAJ display's two lines.
            int reach = 0;
            // The range the reach is judged on: the site's from the craft now, or, in the coast, from where the plan takes hold.
            double reachRangeM = st.RangeM;
            bool coasting = st.Plan != null && e < st.Plan.E0;
            if (coasting)
            {
                // Before the plan's start the site's place in the corridor is the plan's own there: where the coast will deliver the craft, against the corridor at that energy.
                var p0 = st.Plan;
                double aheadAtStart = p0.S[0] * p0.BodyRadius;
                st.RangeShortM = p0.RangeShortM[0];
                st.RangeLongM = p0.RangeLongM[0];
                reachRangeM = aheadAtStart;
                reach = aheadAtStart < p0.RangeShortM[0] ? 1 : aheadAtStart > p0.RangeLongM[0] ? -1 : 0;
                status = FormattableString.Invariant(
                    $"COASTING TO THE AIR: THE PLAN TAKES HOLD IN {(double.IsNaN(st.PredictedStartS) ? 0 : st.PredictedStartS):F0} s, THE SITE {aheadAtStart / 1000:F0} KM AHEAD THERE (CORRIDOR {p0.RangeShortM[0] / 1000:F0}-{p0.RangeLongM[0] / 1000:F0}).");
            }
            else if (st.Plan != null)
            {
                var ps = st.Plan.At(e);
                st.RangeShortM = ps.RangeShortM;
                st.RangeLongM = ps.RangeLongM;
                reach = st.RangeM < ps.RangeShortM ? 1 : st.RangeM > ps.RangeLongM ? -1 : 0;
                if (reach != st.Reach)
                    GuidanceLog.Info(vehicle, FormattableString.Invariant(
                        $"Orbital entry reach: {(reach > 0 ? "HIGH ENERGY, the site is closer than the most drag can stop at" : reach < 0 ? "LOW ENERGY, the site is further than the craft can glide" : "inside the corridor")} ({st.RangeM / 1000:F0} km to go, corridor {ps.RangeShortM / 1000:F0}-{ps.RangeLongM / 1000:F0} km at {vMag:F0} m/s)."));
            }
            st.Reach = reach;
            // A converged plan that reaches the site answers the question the warning asks: the site is in reach, whatever the corridor's edges say (2026-11-25: "HIGH ENERGY: SITE -11384 KM SHORT" counted down through a good coast).
            bool planReaches = st.Plan != null && st.Plan.Converged && Math.Abs(st.Plan.TerminalRangeErrorM) <= cfg.GateRangeM;
            if (reach > 0 && !planReaches)
                status = FormattableString.Invariant($"HIGH ENERGY: SITE {(st.RangeShortM - reachRangeM) / 1000:F0} KM SHORT OF THE CORRIDOR. ") + status;
            else if (reach < 0 && !planReaches)
                status = FormattableString.Invariant($"LOW ENERGY: SITE {(reachRangeM - st.RangeLongM) / 1000:F0} KM PAST THE CORRIDOR. ") + status;
            // A heating limit the craft cannot hold explains the rest, so it goes first.
            bool heatOut = cfg.HeatingRateMax > 0 && double.IsFinite(st.HeatFloorKWm2) && cfg.HeatingRateMax / 1000.0 < st.HeatFloorKWm2;
            if (heatOut)
                status = FormattableString.Invariant($"MAX HEATING {cfg.HeatingRateMax / 1000:F0} CANNOT BE HELD: THIS CRAFT FLIES {st.HeatFloorKWm2:F0} kW/m^2 AT LEAST. ") + status;
            if (st.Plan == null)
                status = double.IsNaN(st.PredictedStartS) ? "PLANNING THE ENTRY." : FormattableString.Invariant($"PLANNING THE ENTRY. AIR IN {st.PredictedStartS:F0} s.");
            else if (!st.Plan.Converged)
                status += " (PLAN NOT CONVERGED: " + st.LastRefusal.ToUpperInvariant() + ")";
            // The attitude the plan assumes is not the one flown: say so first, it explains the rest.
            bool offAttitude = st.Tracker.InAir && Math.Abs(st.AlphaFlownDeg - st.AlphaDeg) > 10.0;
            if (offAttitude)
                status = FormattableString.Invariant($"ATTITUDE NOT HELD: AoA {st.AlphaFlownDeg:F0} DEG FLOWN, {st.AlphaDeg:F0} ASKED. ") + status;
            st.SetStatus(status, warn: offAttitude || heatOut || reach != 0 || outp.Overbanked || outp.LiftUpLimit || outp.LiftDownLimit || (st.Plan != null && !st.Plan.Converged));
        }

        // ---- attitude ----
        double alpha = st.Trim.AlphaAt(mach) * Math.PI / 180.0;
        st.AlphaDeg = alpha * 180.0 / Math.PI;
        double3 liftUp = (rHat - double3.Dot(rHat, vHat) * vHat).NormalizeOrZero();
        double3 right = double3.Cross(vHat, liftUp);
        double sigma = bankDeg * Math.PI / 180.0;
        double3 lift = Math.Cos(sigma) * liftUp + Math.Sin(sigma) * right;
        double3 nose = Math.Cos(alpha) * vHat + Math.Sin(alpha) * lift;
        double3 belly = Math.Sin(alpha) * vHat - Math.Cos(alpha) * lift;
        // Under bellyflop, its plan's pitch, bank and yaw at the craft's state.
        if (st.Phase == OrbitalEntryPhase.Bellyflop && BellyflopAttitude(vehicle, parent, st, now, out double3 bfNose, out double3 bfBelly))
        {
            nose = bfNose;
            belly = bfBelly;
        }
        // BELLYFLOP DIRECT, before bellyflop's first plan: the attitude the craft had when it got here, held still (the entry's angle of attack is 45 deg off a belly-first fall).
        else if (st.Phase == OrbitalEntryPhase.Holding && st.BellyflopDirect && st.BellyIndex >= 0)
        {
            if (!st.HoldLatched)
            {
                st.HoldNoseCci = KsaFrameBridge.BodyNoseCci(vehicle);
                st.HoldBellyCci = KsaAeroSweep.BellyAxes[st.BellyIndex].Transform(KsaFrameBridge.BodyToCci(vehicle));
                st.HoldLatched = true;
            }
            nose = st.HoldNoseCci;
            belly = st.HoldBellyCci;
        }
        st.RollRefCci = st.BellyIndex switch
        {
            0 => double3.Cross(-belly, nose),     // belly is body -Z: body z = -belly, y = z x x
            1 => double3.Cross(belly, nose),      // +Z
            2 => -belly,                          // -Y
            _ => belly,                           // +Y
        };
        _s.CommandDir = nose;
        // The target's own turning rate, fed forward: the bank slews at the roll rate, the AoA follows its Mach schedule and the airflow turns with the path, and with no rate the controller only learns of each after the error has built (a reversal lagged at a zero rate target until 2026-11-21).
        st.TargetRateCci = TargetRate(st, nose, st.RollRefCci, dt);
        if (st.Last.Phase == EntryTracker.Phase.Tracking && st.Last.DragRefMs2 > 0 && double.IsFinite(st.DragMs2))
        {
            double err = st.DragMs2 - st.Last.DragRefMs2;
            st.DragErrorF = double.IsFinite(st.DragErrorF) ? st.DragErrorF + Math.Clamp(dt / DragErrorSmoothS, 0.0, 1.0) * (err - st.DragErrorF) : err;
        }
        else st.DragErrorF = double.NaN;
        _s.CommandRate = st.TargetRateCci;
        _s.HasCommand = true;

        // ---- what the craft flies, off its own body axes ----
        doubleQuat body2Cci = KsaFrameBridge.BodyToCci(vehicle);
        double3 noseNow = double3.UnitX.Transform(body2Cci);
        st.AlphaFlownDeg = Math.Acos(Math.Clamp(double3.Dot(noseNow, vHat), -1.0, 1.0)) * 180.0 / Math.PI;
        if (st.BellyIndex >= 0)
        {
            double3 bellyNow = KsaAeroSweep.BellyAxes[st.BellyIndex].Transform(body2Cci);
            double3 liftNow = -(bellyNow - double3.Dot(bellyNow, vHat) * vHat).NormalizeOrZero();
            st.BankFlownDeg = Math.Atan2(double3.Dot(liftNow, right), double3.Dot(liftNow, liftUp)) * 180.0 / Math.PI;
        }

        if (OrbitalEntryLog.Due(st, now))
            using (new AdvancedFlightComputer.Core.PerfTracker.Scope("Entry.LogRow"))
                OrbitalEntryLogRow(st, now, e, ef, mach, gamma, nominal, vehicle);

        // ---- the TRAJ window's history ----
        if (now >= st.NextTraceAt)
        {
            st.NextTraceAt = now + 1.0;
            st.Trace.Add(new OrbitalEntryState.TrajSample(vMag, st.RangeM, drag, st.Last.DragRefMs2, bankDeg, alt, st.Mach));
            if (st.Trace.Count > 4000)
                st.Trace.RemoveRange(0, 1000);
        }
    }

    private static readonly System.Collections.Generic.List<double> _oeRow = new();

    private static void OrbitalEntryLogRow(OrbitalEntryState st, double now, double e, double ef, double mach, double gamma, EntryModel nominal, Vehicle vehicle)
    {
        var o = st.Last;
        var a = st.Attitude;
        EntryPlan p = st.Plan;
        double R0 = nominal.BodyRadius;
        var v = _oeRow;
        v.Clear();
        double q = 0.5 * vehicle.PhysicsEnvironment.AtmosphericDensity * st.SpeedMs * st.SpeedMs;
        EntryPlan.Sample? ps = p?.At(e);
        v.Add(now); v.Add((int)st.Phase); v.Add(st.AltitudeM); v.Add(st.SpeedMs); v.Add(mach); v.Add(gamma * 180 / Math.PI); v.Add(q);
        v.Add(e); v.Add(ef); v.Add(st.GreatCircleM / 1000); v.Add(st.RangeM / 1000); v.Add(st.CrossrangeM / 1000); v.Add(st.HeadingErrorDeg); v.Add(o.DeadbandRad * 180.0 / Math.PI);
        v.Add(st.RangeShortM / 1000); v.Add(st.RangeLongM / 1000);
        v.Add(ps.HasValue ? ps.Value.S * R0 / 1000 : double.NaN); v.Add(st.Reach);
        v.Add(st.DragMs2); v.Add(o.DragRefMs2); v.Add(ps.HasValue ? ps.Value.DragMaxMs2 : double.NaN); v.Add(ps.HasValue ? ps.Value.DragMinMs2 : double.NaN); v.Add(st.DragModelMs2);
        v.Add(st.HdotMs); v.Add(o.HdotRefMs);
        v.Add(st.LiftToDrag); v.Add(st.Trim.LiftToDragAt(mach)); v.Add(st.LiftToDragScale); v.Add(st.DensityScale);
        v.Add(ps.HasValue ? ps.Value.U : double.NaN); v.Add(o.UCmd); v.Add(o.BankRefDeg); v.Add(o.BankTargetDeg); v.Add(o.BankDeg); v.Add(st.BankFlownDeg);
        v.Add(o.Side); v.Add(o.Reversing ? 1 : 0); v.Add(o.Overbanked ? 1 : 0); v.Add(o.LiftUpLimit ? 1 : 0); v.Add(o.LiftDownLimit ? 1 : 0);
        v.Add(st.AlphaDeg); v.Add(st.AlphaFlownDeg); v.Add(a.ErrorDeg); v.Add(a.RateErrorDegS); v.Add(st.AttitudeOwn ? 1 : 0); v.Add(a.AirKnown ? 1 : 0);
        v.Add(a.DemandNm.X); v.Add(a.DemandNm.Y); v.Add(a.DemandNm.Z); v.Add(a.AirNm.X); v.Add(a.AirNm.Y); v.Add(a.AirNm.Z);
        v.Add(a.SurfacesNm.X); v.Add(a.SurfacesNm.Y); v.Add(a.SurfacesNm.Z); v.Add(a.RcsNm.X); v.Add(a.RcsNm.Y); v.Add(a.RcsNm.Z); v.Add(a.RcsAskedNm.X); v.Add(a.RcsAskedNm.Y); v.Add(a.RcsAskedNm.Z); v.Add(a.RcsFlowKgS);
        const double toDeg = 180.0 / Math.PI;
        v.Add(a.SpareAccelRadS2.X * toDeg); v.Add(a.SpareAccelRadS2.Y * toDeg); v.Add(a.SpareAccelRadS2.Z * toDeg); v.Add(a.AirAxisAccelRadS2 * toDeg); v.Add(a.Saturated ? 1 : 0); v.Add(a.RcsAxes); v.Add(a.RcsFiring);
        var bfp = st.Bellyflop.Plan;
        v.Add(st.Bellyflop.PitchDeg); v.Add(st.Bellyflop.BankDeg); v.Add(st.Bellyflop.YawDeg);
        v.Add(bfp?.MissM ?? double.NaN); v.Add(bfp != null ? bfp.EndTime - now : double.NaN); v.Add(st.Bellyflop.TerminalRangeM / 1000.0);
        v.Add(st.Bellyflop.ForceScale);
        v.Add(a.FinCommandDeg?.Length ?? 0);
        for (int i = 0; i < 4; i++) v.Add(a.FinCommandDeg != null && i < a.FinCommandDeg.Length ? a.FinCommandDeg[i] : double.NaN);
        for (int i = 0; i < 4; i++) v.Add(a.FinNowDeg != null && i < a.FinNowDeg.Length ? a.FinNowDeg[i] : double.NaN);
        v.Add(st.Solves); v.Add(p != null && p.Converged ? 1 : 0); v.Add(p != null ? now - st.PlanAt : double.NaN);
        v.Add(p != null ? p.TerminalRangeErrorM / 1000 : double.NaN); v.Add(p != null ? p.MaxDefect : double.NaN); v.Add(p != null ? p.Iterations : double.NaN); v.Add(st.LastSolveMs);
        v.Add(st.Solves); v.Add(st.Refusals); v.Add(st.TablesReady ? 1 : 0);
        st.AttitudeOwn = false;
        OrbitalEntryLog.Row(st, now, v, st.Status);
    }

    private static void QueueOrbitalEntrySolve(Vehicle vehicle, IParentBody parent, OrbitalEntryState st, EntryConfig cfg, EntryModel nominal,
                                               double e, double rMag, double vMag, double gamma, double rangeRad, double drag, double now)
    {
        EntryModel model = nominal.WithScales(st.DensityScale, st.LiftToDragScale);
        double R0 = model.BodyRadius;
        UpdateHeatFloor(vehicle, st, model, cfg, e);
        double e0;
        double[] x0;
        // The plan starts at the plan start drag: from the state as it is once there, before it from the predicted coast down to it.
        if (drag >= cfg.PlanStartDrag)
        {
            e0 = e;
            x0 = [rMag / R0, gamma, rangeRad];
            st.PredictedStartS = double.NaN;
            st.NextSolveAt = now + Math.Max(_s.OrbitalEntryResolveS, 0.25);
        }
        else
        {
            var start = EntryPlanner.PredictStart(model, cfg, rMag, vMag, gamma, rangeRad);
            if (start == null)
            {
                st.SetStatus("NOT ON AN ENTRY TRAJECTORY: THE CRAFT DOES NOT MEET THE AIR.", warn: true);
                st.NextSolveAt = now + 10.0;
                return;
            }
            e0 = start.Value.E;
            x0 = start.Value.X;
            st.PredictedStartS = start.Value.SecondsAhead;
            st.NextSolveAt = now + 10.0;
        }
        EntryPlan warm = st.Plan;
        st.Worker.TryRun(st, ct =>
        {
            st.Pending = _entryPlanner.Solve(model, cfg, e0, x0, warm);
        });
    }

    /// <summary>
    /// The least peak heating this craft can fly from here (EntryDynamics.LeastPeakHeating), for the status line's warning when MAX HEATING lies under it. Logged the first time, and again when it moves by more than 5 %.
    /// </summary>
    private static void UpdateHeatFloor(Vehicle vehicle, OrbitalEntryState st, EntryModel model, EntryConfig cfg, double e)
    {
        var (peak, speed) = new EntryDynamics(model).LeastPeakHeating(e, EntryPlanner.FinalEnergy(model, cfg), cfg);
        double kw = peak / 1000.0;
        bool log = !double.IsFinite(st.HeatFloorKWm2) || Math.Abs(kw - st.HeatFloorKWm2) > 0.05 * Math.Max(st.HeatFloorKWm2, 1.0);
        if (log)
            GuidanceLog.Info(vehicle, FormattableString.Invariant(
                $"Orbital entry: the least peak heating this craft can fly is {kw:F0} kW/m2 at {speed:F0} m/s (equilibrium glide at the plan's least bank, {cfg.BankMinDeg:F0} deg, on a {model.NoseRadius:F2} m nose); MAX HEATING {(cfg.HeatingRateMax > 0 ? FormattableString.Invariant($"{cfg.HeatingRateMax / 1000:F0} kW/m2{(cfg.HeatingRateMax < peak ? ", which cannot be held" : "")}") : "off")}."));
        (st.HeatFloorKWm2, st.HeatFloorSpeedMs) = (kw, speed);
    }

    /// <summary>
    /// The planner's model of this craft: the sampled atmosphere, the trim schedule's drag and L/D on the sweep's reference area, the craft's mass now. Immutable and thread-safe, so the worker can fly it.
    /// </summary>
    private static EntryModel OrbitalEntryModel(Vehicle vehicle, IParentBody parent, OrbitalEntryState st, double densityScale, double ldScale)
        => OrbitalEntryModel(parent, st.Aero, st.Trim, vehicle.TotalMass, densityScale, ldScale);

    /// <summary>The model with the body's rotation and the latitude and heading of a state (EntryModel.Omega).</summary>
    internal static EntryModel WithRotationAt(this EntryModel model, IParentBody parent, double3 r, double3 vRel)
    {
        var (omega, latitude, heading) = RotationAt(parent, r, vRel);
        return model.WithRotation(omega, latitude, heading);
    }

    /// <summary>The time the displayed drag error is smoothed over, s: the measured drag flutters with every twitch of the attitude.</summary>
    private const double DragErrorSmoothS = 3.0;

    /// <summary>The time over which the target's turning rate is smoothed, s: a step's jitter in the measured path should not reach the fins.</summary>
    private const double TargetRateSmoothS = 0.3;

    /// <summary>
    /// The attitude target's angular velocity, CCI rad/s, from this step's target frame against the last one's: for an orthonormal frame e_i turned by R, sum e_i x R e_i = 2 sin(theta) k. Smoothed over TargetRateSmoothS. Zero on the first step and after a jump of more than 10 deg in a step (a re-engage or a change of belly).
    /// </summary>
    private static double3 TargetRate(OrbitalEntryState st, double3 nose, double3 rollRef, double dt)
    {
        if (!(dt > 1e-6))
            return st.TargetRateCci;   // no time has passed: the same rate, and the same frame to difference from
        double3 x1 = nose.NormalizeOrZero();
        double3 y1 = (rollRef - double3.Dot(rollRef, x1) * x1).NormalizeOrZero();
        double3 x0 = st.PrevNoseCci, r0 = st.PrevRollRefCci;
        st.PrevNoseCci = x1;
        st.PrevRollRefCci = y1;
        if (x0.Length() < 0.5 || r0.Length() < 0.5 || x1.Length() < 0.5 || y1.Length() < 0.5)
            return st.TargetRateCci = default;
        double3 y0 = (r0 - double3.Dot(r0, x0) * x0).NormalizeOrZero();
        double3 z0 = double3.Cross(x0, y0), z1 = double3.Cross(x1, y1);
        double3 v = double3.Cross(x0, x1) + double3.Cross(y0, y1) + double3.Cross(z0, z1);
        double sinTheta = Math.Clamp(0.5 * v.Length(), 0.0, 1.0);
        double theta = Math.Asin(sinTheta);
        if (theta > 10.0 * Math.PI / 180.0)
            return st.TargetRateCci = default;
        double3 raw = sinTheta > 1e-12 ? double3.Normalize(v) * (theta / dt) : default;
        double k = Math.Clamp(dt / TargetRateSmoothS, 0.0, 1.0);
        return st.TargetRateCci + k * (raw - st.TargetRateCci);
    }

    /// <summary>The body's rotation, and the latitude and heading of a state (position, CCI; velocity over the ground, CCI), rad.</summary>
    internal static (double Omega, double Latitude, double Heading) RotationAt(IParentBody parent, double3 r, double3 vRel)
    {
        double3 spin = parent.GetAngularVelocityCci();
        double omega = spin.Length();
        if (!(omega > 0.0))
            return (0.0, 0.0, 0.0);
        double3 pole = spin / omega;
        double3 up = double3.Normalize(r);
        double latitude = Math.Asin(Math.Clamp(double3.Dot(up, pole), -1.0, 1.0));
        double3 east = double3.Cross(pole, up).NormalizeOrZero();
        double3 north = double3.Cross(up, east);
        double heading = Math.Atan2(double3.Dot(vRel, east), double3.Dot(vRel, north));
        return (omega, latitude, heading);
    }

    private static EntryModel OrbitalEntryModel(IParentBody parent, KsaAeroSweep.Result aero, EntryTrim trim, double massKg, double densityScale, double ldScale)
    {
        var atm = aero.Atmosphere;
        return new EntryModel
        {
            BodyRadius = parent.MeanRadius,
            Mu = parent.Mu,
            Density = h => atm.Density(h),
            SpeedOfSound = h => atm.SpeedOfSoundAt(h),
            AtmosphereTop = atm.TopAltitude,
            DragCoefficient = trim.CdAt,
            LiftToDrag = trim.LiftToDragAt,
            Mass = Math.Max(massKg, 1.0),
            ReferenceArea = aero.ReferenceArea,
            NoseRadius = Math.Max(_s.OrbitalEntryNoseRadiusM, 0.05),
            DensityScale = densityScale,
            LiftToDragScale = ldScale,
        };
    }

    /// <summary>The angle-of-attack schedule from the belly-first sweep: the chosen belly axis, or the one of the four with the most trimmed Mach rows and then the best lift.</summary>
    private static bool BuildOrbitalEntryTrim(OrbitalEntryState st, out string why)
    {
        if (!TryBuildEntryTrim(st.Aero, out EntryTrim trim, out int index, out why))
            return false;
        st.Trim = trim;
        st.BellyIndex = index;
        st.BellyAxisName = st.Aero.Belly[index].AxisName;
        st.TrimRows = st.Aero.Belly[index];
        return true;
    }

    /// <summary>
    /// THE SCHEDULE WITH THE FLAPS. At rest the flaps may leave no angle trimmed (2026-11-21: C_m -1.4 at every angle, so the least moment sat at the band's edge). With the belly-first body and surface tables in, each Mach row takes the angle where the body's pitch moment lies inside what the flaps can add to it either way. The angle with the most margin each side wins, nearest ENTRY AoA on a tie, and the least shortfall where none is inside. Drag and L/D stay the at-rest rows'.
    /// </summary>
    private static void RetrimWithFlaps(Vehicle vehicle, OrbitalEntryState st)
    {
        st.FlapTrimDone = true;
        KsaAeroSweep.Result aero = st.Aero;
        var rows = st.TrimRows;
        if (aero.BellyBodyMoment == null || rows == null || st.BellyIndex < 0)
            return;
        foreach (var t in aero.Surfaces)
            if (t.BellyMoment.Length != t.DeflectionDeg.Length)
                return;
        double3 b = KsaAeroSweep.BellyAxes[st.BellyIndex];
        double3 a = double3.Cross(b, new double3(1, 0, 0));
        var table = aero.BellyBodyMoment;
        double lo = Math.Max(_s.OrbitalEntryAoaMinDeg, 90.0 - KsaAeroSweep.BellyAlphaTableMaxDeg);
        double hi = Math.Min(_s.OrbitalEntryAoaMaxDeg, 90.0 - KsaAeroSweep.BellyAlphaTableMinDeg);
        if (!(hi > lo))
            return;
        int nm = rows.MachGrid.Length;
        var angles = new double[nm];
        var trimmed = new bool[nm];
        var residual = new double[nm];
        var text = new System.Text.StringBuilder();
        for (int i = 0; i < nm; i++)
        {
            double mach = Math.Clamp(rows.MachGrid[i], table.MachMin, table.MachMax);
            double bestScore = double.PositiveInfinity, bestA = lo, bestRes = 0, bestBody = 0, bestLo = 0, bestHi = 0;
            for (double ae = lo; ae <= hi + 1e-9; ae += 1.0)
            {
                double at = (90.0 - ae) * Math.PI / 180.0;
                table.Coefficients(mach, at, 0.0, out double cl, out double cm, out double cn);
                double body = cl * a.X + cm * a.Y + cn * a.Z;
                double down = 0, up = 0;
                foreach (var t in aero.Surfaces)
                {
                    double tMin = 0, tMax = 0;
                    foreach (var m in t.BellyMoment)
                    {
                        m.Coefficients(mach, at, 0.0, out double sl, out double sm, out double sn);
                        double d = sl * a.X + sm * a.Y + sn * a.Z;
                        tMin = Math.Min(tMin, d);
                        tMax = Math.Max(tMax, d);
                    }
                    down += tMin;
                    up += tMax;
                }
                double reachLo = body + down, reachHi = body + up;
                double res = reachLo > 0 ? reachLo : reachHi < 0 ? reachHi : 0.0;
                double width = Math.Max(reachHi - reachLo, 1e-9);
                double offCentre = Math.Abs(0.5 * (reachLo + reachHi)) / width;
                double score = Math.Abs(res) * 1e3 + offCentre + 1e-4 * Math.Abs(ae - _s.OrbitalEntryAoaDeg);
                if (score < bestScore)
                {
                    bestScore = score; bestA = ae; bestRes = res; bestBody = body; bestLo = down; bestHi = up;
                }
            }
            angles[i] = bestA;
            trimmed[i] = bestRes == 0.0;
            residual[i] = bestRes;
            text.Append(FormattableString.Invariant($" M{rows.MachGrid[i]:0.#}: {bestA:F0}{(trimmed[i] ? "" : "*")} body {bestBody:F3} flaps {bestLo:F3}..+{bestHi:F3};"));
        }
        st.Trim = EntryTrim.AtAngles(rows.MachGrid, rows.AlphaDeg, rows.Cd, rows.Cl, rows.Cm, angles, trimmed, residual);
        GuidanceLog.Info(vehicle, "Orbital entry schedule with the flaps (alpha deg from nose-first; * the flaps cannot hold it there; C_m about the pitch axis):" + text);
    }

    /// <summary>
    /// The belly side BELLY AXIS asks for (KsaAeroSweep.BellyAxes index): 0 is the craft's own belly, +Z (index 1): -Z had it upside down, and every flight set 2 by hand (2026-11-21); 1 to 4 name a side; 5 lets the sweep choose, -1 with <paramref name="chooseAny"/>, else the side chosen before if there is one.
    /// Every belly-first sweep is asked for on this side, trim rows and side tables together, so one sweep serves the aim, the entry and touchdown (2026-11-25: three sweeps of the Starship in a minute, each asking for a set the last had not swept).
    /// </summary>
    private static int EntryBellySideWanted(bool chooseAny = false) => _s.BellySideWanted(chooseAny);

    /// <summary>The trim schedule of a belly-first sweep, and the belly side it flies (KsaAeroSweep.BellyAxes index).</summary>
    private static bool TryBuildEntryTrim(KsaAeroSweep.Result aero, out EntryTrim trim, out int index, out string why)
    {
        why = "";
        trim = null;
        index = -1;
        var sweeps = aero.Belly;
        EntryTrim Make(KsaAeroSweep.BellySweep b) => _s.OrbitalEntryTrimAoa
            ? EntryTrim.Build(b.MachGrid, b.AlphaDeg, b.Cd, b.Cl, b.Cm, _s.OrbitalEntryAoaMinDeg, _s.OrbitalEntryAoaMaxDeg, _s.OrbitalEntryAoaDeg)
            : EntryTrim.Fixed(b.MachGrid, b.AlphaDeg, b.Cd, b.Cl, _s.OrbitalEntryAoaDeg);
        int best = -1;
        EntryTrim bestTrim = null;
        double bestScore = double.NegativeInfinity;
        for (int i = 0; i < sweeps.Length; i++)
        {
            int wanted = EntryBellySideWanted(chooseAny: true);
            if (wanted >= 0 && i != wanted)
                continue;
            EntryTrim t = Make(sweeps[i]);
            double meanLd = 0;
            for (int j = 0; j < t.Mach.Length; j++) meanLd += t.LiftToDrag[j];
            meanLd /= Math.Max(t.Mach.Length, 1);
            // The side with the least pitch moment left over the schedule, then the most lift.
            double residual = 0;
            foreach (double cmLeft in t.ResidualCm) residual += Math.Abs(cmLeft);
            double score = t.TrimmedRows * 10.0 - residual * 1000.0 + meanLd;
            if (score > bestScore) { bestScore = score; best = i; bestTrim = t; }
        }
        if (bestTrim == null)
        {
            why = "NO BELLY AXIS TO FLY.";
            return false;
        }
        if (bestTrim.LiftToDragAt(10) <= 0.02)
        {
            why = FormattableString.Invariant($"NO LIFT BELLY FIRST ON {sweeps[best].AxisName} (L/D {bestTrim.LiftToDragAt(10):F2} AT MACH 10).");
            return false;
        }
        trim = bestTrim;
        index = best;
        return true;
    }
}
