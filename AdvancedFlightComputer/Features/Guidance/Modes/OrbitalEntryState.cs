#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System.Collections.Generic;
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

    // ---- aero ----
    public KsaAeroSweep.Job AeroJob;
    /// <summary>The second sweep: the belly-first body and surface tables on the side the trim chose (KsaAeroSweep.TryStart's bellySide). Until it lands the attitude flies on the first sweep's tables, with no surfaces belly first.</summary>
    public KsaAeroSweep.Job TablesJob;
    public bool TablesReady;
    public KsaAeroSweep.Result Aero;
    public EntryTrim Trim;
    public string BellyAxisName = "";
    public int BellyIndex = -1;

    // ---- planning ----
    public Ksa3DofWorker Worker;
    public volatile EntryPlan Plan;
    public volatile EntryPlanner.Result Pending;
    public double NextSolveAt = double.NegativeInfinity;
    public int Solves, Refusals;
    public string LastRefusal = "";
    public double LastSolveMs;
    public double PredictedStartS = double.NaN;

    // ---- tracking ----
    public EntryTracker Tracker;
    public EntryTracker.Output Last;
    public double LastStep = double.NaN;
    public double DensityScale = 1.0, LiftToDragScale = 1.0;
    public double DragMs2, LiftToDrag, HdotMs, SpeedMs, AltitudeM, RangeM, CrossrangeM, Mach, AlphaDeg, Energy;
    public int Reversals;

    // ---- the TRAJ window's history: one sample a second ----
    public readonly List<TrajSample> Trace = new();
    public double NextTraceAt = double.NegativeInfinity;

    // ---- status ----
    public string Status = "";
    public bool StatusWarn;

    public readonly record struct TrajSample(double SpeedMs, double RangeM, double DragMs2, double DragRefMs2, double BankDeg, double AltitudeM);

    public void SetStatus(string text, bool warn = false)
    {
        Status = text;
        StatusWarn = warn;
    }

    public void Reset()
    {
        Worker?.Dispose();
        Worker = null;
        Plan = null;
        Pending = null;
        Trim = null;
        AeroJob = null;
        TablesJob = null;
        TablesReady = false;
        Aero = null;
        BellyAxisName = "";
        BellyIndex = -1;
        NextSolveAt = double.NegativeInfinity;
        Solves = Refusals = 0;
        LastRefusal = "";
        LastSolveMs = 0;
        PredictedStartS = double.NaN;
        Tracker = null;
        Last = default;
        LastStep = double.NaN;
        DensityScale = LiftToDragScale = 1.0;
        Reversals = 0;
        Trace.Clear();
        StopPending = false;
        RollRefCci = default;
        NextTraceAt = double.NegativeInfinity;
    }
}
