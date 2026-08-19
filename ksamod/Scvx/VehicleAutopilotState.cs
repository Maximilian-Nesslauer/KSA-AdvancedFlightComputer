using System.Runtime.CompilerServices;
using Brutal.Numerics;
using PoweredGuidance.Upfg;
using KSA;

/// <summary>
/// Everything the autopilot remembers about ONE vehicle.
///
/// This used to be static fields on PoweredGuidanceWindow, which meant the mod could
/// fly exactly one vehicle at a time — and worse, that the state outlived the vehicle
/// it described. Loading a save replaced the Vehicle while the statics survived, so the
/// autopilot went on flying a plan built for a craft that no longer existed.
///
/// Keyed per vehicle instead, a booster can fly itself home while the player watches
/// the upper stage continue to orbit, and the window simply shows whichever vehicle is
/// focused.
///
/// SETTINGS DO NOT LIVE HERE. Tilt limit, node spacing target, replan cadence, the
/// threading toggle — those are the player's preferences and stay global, so changing
/// one affects every vehicle rather than only whichever happened to be focused when it
/// was set. What lives here is what a vehicle is DOING: whether guidance is engaged,
/// its plan, its worker, and the counters the escalation ladder runs on.
/// </summary>
public sealed class VehicleAutopilotState
{
    // --- engagement ---
    public Ksa6DofGuidance Guidance;
    public Ksa6DofSolveWorker Worker;
    public bool Active;
    public bool EngagePending;          // set by the draw, consumed by the step
    public string Error = "";

    // --- the MPC loop ---
    public double LastReplan;
    public bool DidSolve, SolveOk;      // did THIS cycle re-solve, and did it work
    public int RefusalRun;              // consecutive refused re-solves
    public int Recoveries;

    // --- cold solve ---
    public bool Converging;             // cold solve in progress, not yet flyable
    public int ColdFrames;
    public bool ColdResult;             // outcome of the last collected cold iteration

    // --- node ladder ---
    public int GateIndex = -1;          // -1 = above every gate
    public int GateChanges;
    public int RungFloor = int.MaxValue;
    public double RungFloorSpeed;
    public int BackedOffTo = -1;

    // --- actuation, for the readout and the bias estimator ---
    public double LastThrottle;
    public double DemandN, CapabilityN;
    public bool ThrustSaturated;
    public bool TouchdownArmed;

    // --- offset-free MPC: the residual acceleration estimate ---
    public double[] PrevV;
    public double PrevT;
    public double3 Bias;

    // --- diagnostics: how valid the diagonal-inertia approximation is ---
    public double OffDiag, Asym;

    /// <summary>
    /// Vehicle mass at the previous step, for spotting a staging event. Zero until the
    /// first step after engaging.
    ///
    /// Mass is the signal because it needs no part-tree API and cannot be fooled: over
    /// one sim step a burn removes a fraction of a percent even at full flow, while a
    /// separation removes a large slice at once. Anything past StagingMassDropFraction
    /// is a part tree change, not propellant.
    /// </summary>
    public double LastMass;

    // ------------------------------------------------------------------ UPFG / ascent
    //
    // The flight-computer path, which every non-6-DOF mode drives. These were static
    // too, so two vehicles would have shared one _running, one _status and one
    // command direction - the upper stage's guidance writing over the booster's.

    /// <summary>Guidance is driving the flight computer for this vehicle.</summary>
    public bool Running;
    public bool WasEngaged;
    public string GuidanceError = "";
    public string Status = "";
    public int FailStreak;

    /// <summary>The staged vehicle model UPFG plans against, rebuilt as the stack changes.</summary>
    public UpfgVehicle UpfgVehicle;

    /// <summary>Last commanded inertial direction, and whether there is one.</summary>
    public double3 CommandDir;
    public bool HasCommand;

    public bool CutoffDone;
    public bool StagingActive;
    public double LastSequenceTime = double.NegativeInfinity;

    // The stage model cache. VehicleStageModel used to carry the vehicle it was built
    // for, purely so a switch could invalidate it; the key is the vehicle now, so that
    // field is gone.
    public UpfgVehicle StageModel;
    public bool StageModelDirty = true;
    public long StageModelTick;

    /// <summary>A flight-computer reset the draw asked for, applied on the sim thread.</summary>
    public bool FcResetPending;

    // ------------------------------------------------------------------ ascent
    public PoweredGuidanceWindow.AscentPhase Phase = PoweredGuidanceWindow.AscentPhase.Vertical;
    public double TurnStartTime;
    public double3 FrozenDir;
    public double CutoffTime;
    public bool LanSeeded;

    /// <summary>
    /// True when the sim thread may touch this vehicle's guidance: solve on it, rebuild
    /// it, or replace it. While a job is in flight the worker owns it outright — only
    /// Published and Inputs may be crossed, and both are immutable.
    /// </summary>
    public bool Idle(bool threaded) => !threaded || Worker == null || !Worker.IsBusy;

    /// <summary>
    /// Per-vehicle state, held WEAKLY so a destroyed or unloaded vehicle takes its
    /// autopilot state with it. A Dictionary would keep every craft the player ever
    /// engaged alive for the session and would need explicit cleanup on scene changes —
    /// the exact bookkeeping that made a stale plan survive a save load in the first
    /// place. Nothing here needs to outlive its vehicle.
    /// </summary>
    private static readonly ConditionalWeakTable<Vehicle, VehicleAutopilotState> Table = new();

    /// <summary>State for this vehicle, created on first use.</summary>
    public static VehicleAutopilotState For(Vehicle vehicle) => Table.GetOrCreateValue(vehicle);

    /// <summary>
    /// State for this vehicle ONLY if it already has some. The autopilot hook runs for
    /// every vehicle on every sim step — thousands of calls a second under time warp —
    /// so the hot path must not allocate for craft that have never been engaged.
    /// </summary>
    public static bool TryGet(Vehicle vehicle, out VehicleAutopilotState state)
    {
        state = null;
        return vehicle != null && Table.TryGetValue(vehicle, out state);
    }
}
