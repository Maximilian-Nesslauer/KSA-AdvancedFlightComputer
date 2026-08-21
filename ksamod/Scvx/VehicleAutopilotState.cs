using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using Gfold;
using PoweredGuidance.Upfg;
using KSA;

/// <summary>
/// ONE VEHICLE'S FLIGHT COMPUTER. Everything the autopilot knows about a single
/// craft: which mode is engaged, where it is going, how it is tuned, and every
/// filter, plan, timer and counter those modes run on.
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
/// EVERYTHING THAT SHAPES HOW A VEHICLE FLIES LIVES HERE, not just what it is doing.
/// An earlier split kept "settings" global on the theory that they were player
/// preferences — but a target orbit, a landing site, a pointing cone, a feedback gain
/// and a vehicle height are all properties of one craft's mission and one craft's
/// airframe, and sharing them meant focusing a second vehicle silently re-aimed the
/// first. The rule now is simply: if a vehicle's flight computer would know it, it is
/// a field on this class.
///
/// What stays static on PoweredGuidanceWindow is only what belongs to the PANEL rather
/// than to a craft: which overlay or popup is open, the warp confirmation dialog,
/// reflection handles, and scratch buffers reused within a single call.
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

    /// <summary>6-DOF touchdown arming. The landing machine has its own — see
    /// <see cref="LandingTouchdownArmed"/>; they arm on different events.</summary>
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

    // ------------------------------------------------------------------ shared
    //
    // The engage toggles are per vehicle because they gate whether THIS craft's
    // commands reach its flight computer. Shared, turning the autopilot off to hand one
    // vehicle back also stopped steering every other vehicle in flight.

    /// <summary>Commands reach the flight computer.</summary>
    public bool Engage = true;

    /// <summary>Fire sequences and drive the engine master switch automatically.</summary>
    public bool AutoStage = true;

    /// <summary>Vehicle-wide acceleration limit — an airframe/payload constraint.</summary>
    public bool GLimitEnabled;
    public double GLimitG = 4.0;

    /// <summary>
    /// This craft's UPFG solver, used by ascent (mode 1) and the deorbit burn (mode 3).
    ///
    /// ONE INSTANCE PER VEHICLE IS A CORRECTNESS REQUIREMENT, not tidiness. UPFG is a
    /// recursive filter: every call refines vgo, rd, rgrav and a warm-started conic
    /// state carried from the previous call, and it measures sensed acceleration as
    /// (v - vprev) against the velocity IT saw last. Shared between craft, the first
    /// step after a focus switch subtracted one vehicle's inertial velocity from
    /// another's — kilometres per second of phantom sensed dv straight into vgo — and
    /// the steering swung until it re-converged. That was the attitude twitch on every
    /// vehicle switch. Converged latches too, so the newly focused craft inherited the
    /// other's "converged" and could promote its own ascent phase on it.
    /// </summary>
    public readonly UpfgGuidance Upfg = new UpfgGuidance();

    /// <summary>
    /// Spent engines the auto-stager has already fired a sequence for. Per vehicle so
    /// one craft dropping its boosters cannot suppress another's staging.
    /// </summary>
    public readonly HashSet<uint> SpentStagedFor = new HashSet<uint>();

    // ------------------------------------------------------------------ UPFG / ascent
    //
    // The flight-computer path, which every non-6-DOF mode drives. These were static
    // too, so two vehicles would have shared one _running, one _status and one
    // command direction - the upper stage's guidance writing over the booster's.

    /// <summary>Guidance is driving the flight computer for this vehicle.</summary>
    public bool Running;

    /// <summary>
    /// EXECUTE was pressed while <see cref="AutoLaunch"/> was set: warp to the launch
    /// window and start guidance there. Separate from AutoLaunch because that is a
    /// MODE the user selects ahead of time, while this is the commit — arming used to
    /// happen the instant the checkbox was ticked, which meant the panel could start
    /// warping before anyone had asked it to launch.
    /// </summary>
    public bool LaunchArmed;

    /// <summary>
    /// Largest |rgo| and |vgo| seen since EXECUTE, so the ascent panel can draw each
    /// as a fraction of where it started rather than an unscaled absolute. Display
    /// only — nothing in the guidance loop reads them. Latched as a running maximum
    /// rather than sampled once, because the first solved frame is not reliably the
    /// largest: UPFG is still converging on it.
    /// </summary>
    public double RgoPeak, VgoPeak;
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

    /// <summary>
    /// This vehicle has already been handed back after the mod was switched off.
    ///
    /// Deactivation cannot be a synchronous sweep: the per-vehicle state lives in a
    /// ConditionalWeakTable with no enumeration, and the writes that release attitude
    /// and cut the engine are only legal from the PrepareWorker prefix. So the flag is
    /// set globally and each vehicle tears itself down the next time that prefix runs
    /// for it, which is every sim step. This marks the ones already done, so the
    /// steady-state cost of an inactive mod is one bool test per vehicle.
    /// </summary>
    public bool HandedBack;

    // ------------------------------------------------------------------ ascent
    public PoweredGuidanceWindow.AscentPhase Phase = PoweredGuidanceWindow.AscentPhase.Vertical;
    public double TurnStartTime;
    public double3 FrozenDir;
    public double CutoffTime;
    public bool LanSeeded;

    /// <summary>
    /// THIS VEHICLE'S TARGET ORBIT (altitudes km, angles deg). Defaults are an ISS
    /// launch: the 51.6 deg plane, inserting at a 200 km perigee with apogee at ISS
    /// altitude for the rendezvous transfer. LAN is seeded from the vehicle's own
    /// position the first time its panel is drawn.
    ///
    /// The seed flag was already per vehicle while these were not, so focusing a craft
    /// that had never been seeded rewrote the LAN out from under whichever vehicle was
    /// already flying to it.
    /// </summary>
    public double PeKm = 200.0;
    public double ApKm = 420.0;
    public double IncDeg = 51.6;
    public double LanDeg = 250.0;

    /// <summary>Launch-to-target: the vehicle to chase, and the chase orbit's offset.</summary>
    public string TargetId = "";
    public double ChaseOffsetKm = 20.0;
    public bool LaunchDescending;
    public bool AutoLaunch = true;

    /// <summary>
    /// Absolute sim time of the launch window, LATCHED when EXECUTE arms the launch.
    /// An absolute instant rather than a countdown because the countdown is derived
    /// from a wrapped angle: overshoot the window under time warp and it does not go
    /// negative, it jumps to a full rotation away. Comparing sim times survives a warp
    /// step of any size.
    /// </summary>
    public double LaunchTargetTime = double.NaN;

    /// <summary>Gravity-turn shaping: when the pitchover starts and how fast it ramps.</summary>
    public double TurnStartAltKm = 0.5;
    public double TurnRateDegS = 1.0;

    // ------------------------------------------------------------------ landing
    public PoweredGuidanceWindow.LandingPhase LandingPhase = PoweredGuidanceWindow.LandingPhase.Idle;

    /// <summary>
    /// This vehicle's landing site. Defaults to the Apollo 11 landmark as KSA itself
    /// defines it (Content/Core/Astronomicals.xml, Landmark Id="Apollo11" on the Moon).
    /// </summary>
    public double SiteLatDeg = 0.67408;
    public double SiteLonDeg = 23.47297;

    public double DownrangeFactor = 1.2;   // light the burn this x predicted distance out
    public double AimAltKm = 0.1;          // high-gate altitude above the site
    public double DescentRate = 20.0;      // sink rate at the gate
    public double GateUprangeKm = 0.0;     // gate offset against the approach direction
    public double BurnDownrangeKm;         // predicted braking distance
    public double BurnStartTime;           // sim time of ignition
    public string LandingStatus = "";
    public bool LandingCutPending;         // one-shot engine cut when the flow ends

    /// <summary>Touchdown arming for the landing state machine (6-DOF has its own).</summary>
    public bool LandingTouchdownArmed;
    public PoweredGuidanceWindow.LandingPhase TouchdownPrevPhase = PoweredGuidanceWindow.LandingPhase.Idle;

    /// <summary>Upcoming site passes (time from now, closest ground distance).</summary>
    /// <summary>
    /// Upcoming site passes: time from now, closest ground distance, and that distance
    /// SIGNED by which side of the ground track the site falls on. The sign is what
    /// lets the pass strip put a pass left or right of the site instead of piling
    /// every one of them on the same side.
    /// </summary>
    public readonly List<(double tSec, double minKm, double crossKm)> Passes = new();

    /// <summary>Ask the panel to show this vehicle's G-FOLD / Terminal sub-tab.</summary>
    public bool GfoldTabSelectPending;

    /// <summary>
    /// Which solver flies the powered descent: G-FOLD by default, or the 6-DOF
    /// successive-convexification one. Per vehicle rather than a panel-wide setting,
    /// because the deorbit handoff READS it to decide what to start — so it describes
    /// how this craft lands, not what the player last clicked.
    /// </summary>
    public bool UseSixDofLanding;
    public bool TermTabSelectPending;

    /// <summary>
    /// Terrain height of the site above the mean-radius sphere, cached against the
    /// inputs that produce it. Per vehicle because the site is.
    /// </summary>
    public double SiteTerrainCacheLat = double.NaN;
    public double SiteTerrainCacheLon = double.NaN;
    public object SiteTerrainCacheBody;
    public double SiteTerrainHeightM;

    // ------------------------------------------------------------------ G-FOLD
    public double GfoldGlideSlopeDeg = 1.0;
    public double GfoldPointingDeg = 90.0;
    public double GfoldVMaxMs = 500.0;
    public double GfoldIntervalS = 0.25;   // re-solve cadence
    public int GfoldNodes = 50;
    public double GfoldHoverHandoffAltM = 10.0;
    public double GfoldThrottleMin = 0.05;
    public double GfoldThrottleMax = 0.90;
    public double GfoldSlewReg = 0.05;

    /// <summary>
    /// Distance from this vehicle's CoM down to its landing legs — an airframe
    /// dimension, so emphatically per vehicle. Applied as an offset on the TARGET
    /// altitude (the CoM is planned to arrive this high above the pad), NOT by shifting
    /// the vehicle reference point.
    /// </summary>
    public double VehicleHeightM = 15.0;

    public double GfoldHandoffTgo = 40.0;  // hand UPFG braking over this long before the gate
    public double GfoldThrottle;
    public double GfoldHandoffTime;
    public double GfoldLastSolveTime = double.NegativeInfinity;
    public double GfoldAltM, GfoldSpeedMs;
    public EcosStatus GfoldStatus = EcosStatus.Optimal;
    public int GfoldFailStreak;
    public bool GfoldForceSearch;

    /// <summary>The committed descent plan and the tracker flying it.</summary>
    public GfoldTrajectory GfoldPlan;
    public double GfoldPlanStart;          // sim time of plan node 0
    public double GfoldArrivalTime;        // sim time of planned touchdown
    public double GfoldThrustMax = 1.0;    // engine vac thrust at plan time, N
    public double GfoldKp = 0.08;          // position feedback gain
    public double GfoldKd = 0.30;          // velocity feedback gain
    public double GfoldSmoothTau = 0.15;   // command low-pass time constant
    public double GfoldLastTrackTime;
    public bool GfoldTrackInit;
    public bool GfoldEngineOn;             // hysteretic engine state

    // --- G-FOLD side-view plot ---------------------------------------------
    // The flown path in the pad frame, as (horizontal range to pad, height above
    // touchdown) in metres. Sampled off the guidance step rather than the draw, so it
    // stays even under time warp and does not depend on which craft is on screen.
    public float2[] GfoldTrace;
    public int GfoldTraceCount;
    public double GfoldTraceLastTime = double.NegativeInfinity;

    /// <summary>
    /// The plot's latched axis extents, in metres. LATCHED rather than fitted every
    /// frame: a descent shrinks continuously, so a fitted plot zooms the whole way
    /// down and nothing on it holds still long enough to read.
    /// </summary>
    public double GfoldAxisRangeM, GfoldAxisAltM;
    public double GfoldAxisLockedAt = double.NegativeInfinity;

    /// <summary>
    /// How many times the plot's axes have been latched: 0 none, then start, half the
    /// flight remaining, a quarter remaining, and the final approach. Each stage fires
    /// once, in order.
    /// </summary>
    public int GfoldAxisStage;
    public double GfoldFlightTime0;

    // ------------------------------------------------------------------ terminal hover
    public double TermTouchdownRate = 0.5; // m/s, constant final descent
    public double TermConstAltM = 0.25;    // constant-rate zone height
    public double TermQuadK = 0.2;         // m^-1 s^-1: v = touch + k*(h-h0)^2
    public double TermMaxDescRate = 15.0;  // profile cap
    public double TermMaxTiltDeg = 20.0;   // thrust tilt limit off vertical
    public double TermKpV = 1.5, TermKiV = 0.0, TermKdV = 0.0;
    public double TermKpL = 0.10, TermKiL = 0.0, TermKdL = 0.0;
    public double TermNudgeStep = 0.5;     // m/s per key press / click

    /// <summary>Velocity setpoint offsets (m/s) the player nudges while hovering.</summary>
    public double TermSetE, TermSetN, TermSetUp;

    public PoweredGuidanceWindow.Pid TermPidUp, TermPidE, TermPidN;
    public double TermLastTime;
    public bool TermInit;

    // ------------------------------------------------------------------ 6-DOF SCvx
    public int SixDofNodes = 50;
    public double SixDofTiltDeg = 120.0;
    public double SixDofThrottleFloor = 0.40;
    public bool SixDofFloorAuto = true;    // track the vehicle's real minimum throttle
    public double SixDofSigmaSeed = 20.0;
    public double SixDofTargetAltM = 10.0;
    public double SixDofGlideSlopeDeg = 10.0;   // 0 = off; degrees above horizontal
    public bool SixDofVzEnabled = true;
    public double SixDofVzMaxMs;
    public bool SixDofHoverHandoff = true;
    public double SixDofHoverHandoffAltM = 30.0;
    public double SixDofReplanSec = 0.1;
    public double SixDofThrustFrac = 1.0;  // share of total thrust the burn uses
    public double SixDofRateDampShare = 0.002;
    public double SixDofControlSmooth = 0.05;
    public double SixDofProximal = 0.05;
    public bool SixDofFixedTime;
    public int SixDofSigmaSamples = 5;
    public double SixDofNodeDtTarget = 0.60;
    public bool SixDofNodeGates = true;
    public bool SixDofGfoldSeed;
    public bool SixDofSpreadCold = true;
    public double SixDofColdIntervalS = 0.25;
    public bool SixDofBiasEnabled = true;

    /// <summary>
    /// Solve this craft's re-plan on a worker thread. Per vehicle because the worker
    /// is: toggling it off disposes THIS vehicle's worker, and as one global flag it
    /// left every other craft's worker running while claiming threading was off.
    /// </summary>
    public bool SixDofThreaded = true;

    /// <summary>
    /// This craft asks to be recorded when it engages. The log itself is a single
    /// global sink and grants the first claimant — see SixDofLog.Start, which refuses
    /// a second owner rather than interleaving two craft into one CSV.
    /// </summary>
    public bool SixDofLogging = true;

    // ------------------------------------------------------------------ flown track
    //
    // The overlay's trace. Per vehicle so switching craft shows that craft's own flown
    // path rather than blanking a track the player was watching.
    //
    // Allocated lazily by RecordTrace, not here: at TraceCapacity samples this buffer
    // is ~57 kB, and state is created for any craft the panel merely draws. Every
    // other field on this class is a handful of bytes; this one is worth deferring
    // until the vehicle is actually being flown or watched.
    public double3[] Trace;
    public int TraceCount;
    public int TraceHead;                  // next write slot
    public double TraceLastTime = double.NegativeInfinity;
    public IParentBody TraceParent;

    // ------------------------------------------------------------------ gimbal probe
    public int GimbalMode;                 // 0 = off, 1 = direct, 2 = torque
    public float GimbalY;
    public float GimbalZ;
    public float GimbalRoll;
    public float GimbalPitch;
    public float GimbalYaw;

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
