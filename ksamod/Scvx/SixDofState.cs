using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;

/// <summary>
/// Everything the 6-DOF autopilot remembers about ONE vehicle.
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
public sealed class SixDofState
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
    private static readonly ConditionalWeakTable<Vehicle, SixDofState> Table = new();

    /// <summary>State for this vehicle, created on first use.</summary>
    public static SixDofState For(Vehicle vehicle) => Table.GetOrCreateValue(vehicle);

    /// <summary>
    /// State for this vehicle ONLY if it already has some. The autopilot hook runs for
    /// every vehicle on every sim step — thousands of calls a second under time warp —
    /// so the hot path must not allocate for craft that have never been engaged.
    /// </summary>
    public static bool TryGet(Vehicle vehicle, out SixDofState state)
    {
        state = null;
        return vehicle != null && Table.TryGetValue(vehicle, out state);
    }
}
