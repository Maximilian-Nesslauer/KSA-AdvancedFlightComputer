#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System.Collections.Generic;
using AdvancedFlightComputer.Guidance.Scvx.Bellyflop;
using AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;
using Brutal.Numerics;

/// <summary>
/// One craft's orbital entry: what the mode (Modes/OrbitalEntry.cs) keeps between steps and what the ENTRY TRAJ window reads. The settings themselves are fields on <see cref="VehicleAutopilotState"/> (OrbitalEntry*), so the console's setting rows and the craft's saved settings find them.
///
/// The planner runs on <see cref="Worker"/>; the step reads only the plan the worker published (<see cref="Plan"/>), which is immutable once set.
/// </summary>
public sealed class OrbitalEntryState
{
    public GuidanceWindow.OrbitalEntryPhase Phase;
    public bool EngagePending;
    public bool StopPending;

    /// <summary>The roll reference the attitude command flies with: body +y in CCI, from the bank and the belly axis.</summary>
    public double3 RollRefCci;

    /// <summary>Last step's attitude target (nose, roll reference) and the target's turning rate, CCI rad/s, filtered: the attitude controller's rate feedforward.</summary>
    public double3 PrevNoseCci, PrevRollRefCci, TargetRateCci;

    // ---- aero ----
    public KsaAeroSweep.Job AeroJob;
    /// <summary>The second sweep: the belly-first body and surface tables on the side the trim chose (KsaAeroSweep.TryStart's bellySide). Until it lands the attitude flies on the first sweep's tables, with no surfaces belly first.</summary>
    public KsaAeroSweep.Job TablesJob;
    public bool TablesReady;
    /// <summary>The first sweep's belly-first rows on the side flown: the drag, lift and moment the schedule reads, kept when the belly tables replace that sweep.</summary>
    public KsaAeroSweep.BellySweep TrimRows;
    /// <summary>The schedule has been redone with the flaps' reach (RetrimWithFlaps).</summary>
    public bool FlapTrimDone;
    public KsaAeroSweep.Result Aero;
    public EntryTrim Trim;
    public string BellyAxisName = "";
    public int BellyIndex = -1;

    // ---- bellyflop: the terminal descent from the hand-over (docs/guidance/bellyflop-design.md) ----
    public readonly BellyflopState Bellyflop = new();

    // ---- planning ----
    public Ksa3DofWorker Worker;
    public volatile EntryPlan Plan;
    public volatile EntryPlanner.Result Pending;
    public double NextSolveAt = double.NegativeInfinity;
    public int Solves, Refusals;
    public string LastRefusal = "";
    public double LastSolveMs;
    public double PredictedStartS = double.NaN;

    /// <summary>The least peak heating this craft can fly on this entry, kW/m^2, and the speed it peaks at (EntryDynamics.LeastPeakHeating), refreshed at each re-plan; NaN until the trim is built. A MAX HEATING under it cannot be held.</summary>
    public double HeatFloorKWm2 = double.NaN, HeatFloorSpeedMs = double.NaN;

    // ---- tracking ----
    public EntryTracker Tracker;
    public EntryTracker.Output Last;
    public double LastStep = double.NaN;
    public double DensityScale = 1.0, LiftToDragScale = 1.0;
    public double DragMs2, LiftToDrag, HdotMs, SpeedMs, AltitudeM, RangeM, CrossrangeM, Mach, AlphaDeg, Energy;
    /// <summary>The game's aero acceleration on the craft this step, CCI, m/s^2, for bellyflop's own force scale; <see cref="AeroAccelKnown"/> false where it could not be read.</summary>
    public double3 AeroAccelCci;
    public bool AeroAccelKnown;
    public int Reversals;

    /// <summary>The site against the corridor at this energy (EntryPlan's short and long ranges): 0 inside, +1 too close (high energy), -1 too far (low energy).</summary>
    public int Reach;
    public double RangeShortM = double.NaN, RangeLongM = double.NaN, GreatCircleM, BankFlownDeg, AlphaFlownDeg, DragModelMs2;
    public double PlanAt = double.NaN;
    public double HeadingErrorDeg;

    /// <summary>Measured drag less the reference, m/s^2, smoothed for the display (the tracker filters its own).</summary>
    public double DragErrorF = double.NaN;

    /// <summary>The ENTRY TRAJ window's fixed axes for this entry, built from its first plan; cleared by a reset so the next entry builds its own.</summary>
    public object TrajLayout;

    /// <summary>AFC's attitude controller's last report while it flew the entry (FlyOwnAttitude), for the log.</summary>
    internal AttitudeController.Report Attitude;
    internal bool AttitudeOwn;

    // ---- the TRAJ window's history: one sample a second ----
    public readonly List<TrajSample> Trace = new();
    public double NextTraceAt = double.NegativeInfinity;

    // ---- aiming the deorbit burn (OrbitalEntryAim), kept across the entry's own resets ----
    public enum AimSteps { None, Search, Check, Ready }
    public AimSteps AimStage;
    public KsaAeroSweep.Job AimJob;
    public KsaAeroSweep.Result AimAero;
    public EntryTrim AimTrim;
    public EntryModel AimModel;
    public Ksa3DofWorker AimWorker;
    public volatile EntryPlanner.Result AimCheck;
    public double AimDeparture, AimStartE, AimStartRangeM, AimCrossM, AimShortM, AimLongM, AimTargetM, AimShiftM;
    public double[] AimStartX;
    public int AimTries;

    // ---- status ----
    public string Status = "";
    public bool StatusWarn;

    public readonly record struct TrajSample(double SpeedMs, double RangeM, double DragMs2, double DragRefMs2, double BankDeg, double AltitudeM, double Mach);

    public void SetStatus(string text, bool warn = false)
    {
        Status = text;
        StatusWarn = warn;
    }

    public void Reset()
    {
        Bellyflop.Reset();
        Worker?.Dispose();
        Worker = null;
        Plan = null;
        Pending = null;
        Trim = null;
        AeroJob = null;
        TablesJob = null;
        TablesReady = false;
        TrimRows = null;
        FlapTrimDone = false;
        Aero = null;
        BellyAxisName = "";
        BellyIndex = -1;
        NextSolveAt = double.NegativeInfinity;
        Solves = Refusals = 0;
        LastRefusal = "";
        LastSolveMs = 0;
        PredictedStartS = double.NaN;
        HeatFloorKWm2 = HeatFloorSpeedMs = double.NaN;
        Tracker = null;
        Last = default;
        LastStep = double.NaN;
        DensityScale = LiftToDragScale = 1.0;
        Reversals = 0;
        Reach = 0;
        RangeShortM = RangeLongM = double.NaN;
        PlanAt = double.NaN;
        Attitude = default;
        AttitudeOwn = false;
        Trace.Clear();
        StopPending = false;
        RollRefCci = default;
        NextTraceAt = double.NegativeInfinity;
        TrajLayout = null;
        DragErrorF = double.NaN;
        PrevNoseCci = PrevRollRefCci = TargetRateCci = default;
    }
}

/// <summary>
/// Bellyflop's part of an orbital entry: solved in the background from BELLYFLOP FROM MACH on its own worker, flown from the hand-over. The step reads only what the worker published.
/// </summary>
public sealed class BellyflopState
{
    public Ksa3DofWorker Worker;
    public readonly BellyflopPlanner Planner = new();
    public volatile BellyflopPlan Plan;
    public volatile BellyflopPlanner.Result Pending;
    /// <summary>The model the published plan was solved with, for the attitude: it carries the bearing yaw is referenced to.</summary>
    public volatile BellyflopModel Model;
    /// <summary>The last plan whose dynamics closed, converged or not: the warm start of the next solve, so a plan refused only because the site is not yet in reach does not leave the next one to start cold.</summary>
    public volatile BellyflopPlan Warm;
    /// <summary>The measured aero force over bellyflop's own model of it at the angles flown, low-passed: the model's force scale. 1 until measured.</summary>
    public double ForceScale = 1.0;
    public bool ForceScaleKnown;
    public double NextSolveAt = double.NegativeInfinity;
    public int Solves, Refusals;
    /// <summary>Refusals since the last plan that converged: past BellyflopColdAfterRefusals the next solve starts cold, not from a plan it cannot follow.</summary>
    public int RefusedInRow;
    public string LastRefusal = "";
    /// <summary>The yaw reference, horizontal in the site frame: the bearing to the site, followed until bellyflop takes over and held from then.</summary>
    public double3 BearingLocal;
    /// <summary>The range to go the entry leaves for bellyflop at the hand-over, m: the predicted belly-first fall (EntryConfig.TerminalRangeM).</summary>
    public double TerminalRangeM;
    /// <summary>Pitch, bank and yaw flown now, deg (the airflow on the belly), for the log and the status.</summary>
    public double PitchDeg = double.NaN, BankDeg = double.NaN, YawDeg = double.NaN;

    public void Reset()
    {
        Worker?.Dispose();
        Worker = null;
        Plan = null;
        Pending = null;
        Model = null;
        Warm = null;
        ForceScale = 1.0;
        ForceScaleKnown = false;
        NextSolveAt = double.NegativeInfinity;
        Solves = Refusals = RefusedInRow = 0;
        LastRefusal = "";
        BearingLocal = default;
        TerminalRangeM = 0;
        PitchDeg = BankDeg = YawDeg = double.NaN;
    }
}
