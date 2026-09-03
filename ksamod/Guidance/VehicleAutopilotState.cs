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

    /// <summary>
    /// The TURNING RATE that command is moving at, rad/s in CCI — the "turning rate
    /// implied" by the steering law, published to the flight computer as the target's
    /// own rate (see KsaAttitudeRate). Without it the FC is told every guidance update
    /// is a stationary target and nulls the error instead of tracking the motion.
    /// </summary>
    public double3 CommandRate;

    /// <summary>
    /// Sim time of the last UPFG solve, and of the last ascent step. UPFG runs on a
    /// fixed GUIDANCE CYCLE rather than once per sim step (see
    /// PoweredGuidanceWindow.GuidanceCycle): it is a recursive once-per-cycle
    /// algorithm, and calling it every step wound its internal corrections up sixty
    /// times faster than they are damped for. NegativeInfinity means "never solved",
    /// which forces a solve on the first step after EXECUTE.
    ///
    /// The step time is separate because the phase machine and the commanded attitude
    /// still update every step; it supplies the interval the command slew limit
    /// integrates over.
    /// </summary>
    public double LastSolveTime = double.NegativeInfinity;
    public double LastStepTime;

    /// <summary>
    /// The roll the vehicle had when ascent guidance engaged, as an angle about the
    /// thrust axis from the target plane's normal, and whether it has been measured
    /// yet. Held for the whole ascent so the mod commands a thrust DIRECTION and
    /// nothing else — see PoweredGuidanceWindow.AscentRollRef.
    /// </summary>
    public double RollOffset;
    public bool RollLatched;

    /// <summary>
    /// FORCE A SPECIFIC ROLL instead of holding the one the vehicle lifted off with.
    /// The angle is about the thrust axis, measured in the same plane-referenced frame
    /// RollOffset uses: 0 puts the vehicle's body Y along the target plane's normal
    /// (wings level with the orbital plane) and positive turns right-handed about the
    /// thrust direction.
    ///
    /// Forcing it takes MORE than writing the angle into the commanded quaternion.
    /// KSA's UpdateAttitudeTrackError branches on RollMode.IsDecoupled(), and the
    /// default IS decoupled — in that branch the roll term is never computed and the
    /// target's roll is discarded, so the vehicle would keep whatever roll it had and
    /// the box would do nothing. Ticking this therefore also puts the flight computer
    /// into a roll-tracking mode; see PoweredGuidanceWindow.CommandAttitude.
    /// </summary>
    public bool ForceRoll;
    public double ForceRollDeg;

    public bool CutoffDone;
    public bool StagingActive;
    public double LastSequenceTime = double.NegativeInfinity;

    // --- ascent propellant reserve, for a returning booster ---

    /// <summary>
    /// dV to leave in the first stage for its own return, m/s. Zero switches it off.
    ///
    /// MEASURED AGAINST THE BOOSTER, NOT THE STACK, which is the whole point of
    /// expressing it as dV rather than as kilograms. The propellant a given dV costs is
    /// m_boosterDry * (exp(dv/ve) - 1) - the upper stage cancels out of the rocket
    /// equation entirely - and sizing it against the stack instead over-reserves by the
    /// ratio of the two masses. On a 20 t booster under a 40 t upper stage that is
    /// three times too much propellant and 2.6 times the dV asked for. See
    /// PoweredGuidanceWindow.ReservePropellantKg.
    /// </summary>
    public double AscentReserveDvMs;

    /// <summary>What that dV costs at the current staging geometry, kg. Recomputed with
    /// the stage model; zero when the reserve is not armed.</summary>
    public double ReserveKg;

    /// <summary>The booster's own dry mass, kg - the mass the next separation drops.
    /// Shown because it is the number the reserve is sized against and a wrong-looking
    /// reserve is nearly always a wrong-looking booster.</summary>
    public double ReserveBoosterDryKg;

    /// <summary>
    /// True when the next separation drops EVERY engine now producing thrust, so the
    /// vehicle that separates really is the booster doing the flying.
    ///
    /// This is what keeps the reserve off a strap-on stack. While boosters burn
    /// alongside a core, the next separation drops the boosters and the core keeps
    /// firing - reserving propellant there would stage the whole first stage early to
    /// leave fuel in casings that are about to be thrown away. Once the strap-ons are
    /// gone and the next separation is the core's own, this goes true and the reserve
    /// arms itself.
    /// </summary>
    public bool ReserveArmed;

    /// <summary>Why the reserve is not armed, empty when it is.</summary>
    public string ReserveNote = "";

    /// <summary>Set once the reserve has fired its staging, so it fires once.</summary>
    public bool ReserveStaged;

    /// <summary>
    /// Sim time until which this vehicle is a booster waiting for boostback to engage.
    ///
    /// Set when the sweep adopts a craft that separated under an ascent reserve, cleared
    /// when boostback takes or the window runs out. It exists because the engage cannot
    /// happen on the frame the adoption does - the part tree is still settling and the
    /// aero sweep has nothing to fit - so the attempt is retried, and something has to
    /// keep the sweep interested in a vehicle that is not yet flying anything.
    /// </summary>
    public double HandoverPendingUntil = double.NegativeInfinity;

    // The stage model cache. VehicleStageModel used to carry the vehicle it was built
    // for, purely so a switch could invalidate it; the key is the vehicle now, so that
    // field is gone.
    public UpfgVehicle StageModel;
    public bool StageModelDirty = true;
    public long StageModelTick;

    /// <summary>
    /// KSA's own total dV for the recompute the snapshot above was taken from — the
    /// figure behind the in-game stage menu. Kept alongside so the panel can show the
    /// two side by side when they disagree.
    /// </summary>
    public double StageModelKsaDv;

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

    /// <summary>
    /// Wall-clock tick of the last unarmed launch-window derivation, throttling the
    /// target search behind it. Not used once armed — the instant above is absolute by
    /// then, and re-deriving it is what would let a warp step skip the window.
    /// </summary>
    public long LaunchWindowTick;

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

    /// <summary>
    /// WALL-CLOCK CEILING PER SOLVE, seconds. The guard that stops a hard descent
    /// state from hanging the game.
    ///
    /// SCS ships with no time limit and a 100000-iteration cap, so a state it converges
    /// on slowly just runs — on the sim thread. And the caller's response to a failed
    /// solve is to run a 35-solve search, twice, so one slow state became minutes of
    /// frozen game, retried every GfoldIntervalS. A solve that exceeds this comes back
    /// as MaxIterations, which reads as "this time of flight does not work" — the same
    /// answer an infeasible one gives, and the search moves on instead of grinding.
    ///
    /// 40 ms is under three frames and roughly twice the 19.6 ms a converging solve
    /// costs at eps 1e-4, so it bounds the pathological case without truncating the
    /// normal one. It does NOT bound a search, which is tens of solves — that is the
    /// argument for moving the solve off this thread, not something a per-solve limit
    /// can fix.
    /// </summary>
    public double GfoldSolveTimeLimitS = 0.040;

    /// <summary>Wall-clock cost of the last descent solve, ms — the number the frame budget cares about.</summary>
    public double GfoldSolveMs;
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
    public ConicStatus GfoldStatus = ConicStatus.Optimal;
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
    public double SixDofTargetAltM = 3.0;
    public double SixDofGlideSlopeDeg = 10.0;   // 0 = off; degrees above horizontal
    public bool SixDofVzEnabled = true;
    public double SixDofVzMaxMs;
    public bool SixDofHoverHandoff = true;
    public double SixDofHoverHandoffAltM = 3.5;
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
    public bool SixDofLogging = false;

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

    // ---------------------------------------------------------------- boostback

    /// <summary>
    /// This vehicle's aero surrogate: the Cd(Mach, alpha) table sampled off KSA's own
    /// aerodynamics, and the atmosphere that goes with it. Null until the Boostback
    /// tab has sampled it.
    ///
    /// PER VEHICLE, not global, for the same reason everything else here is: the table
    /// is a property of one craft's bounding box, so sharing it would silently hand a
    /// booster's drag model to an upper stage. It is also why the sweep survives a tab
    /// switch - resampling on every draw would re-fit a spline sixty times a second
    /// to answer a question whose inputs only change when the part tree does.
    /// </summary>
    public KsaAeroSweep.Result Aero;

    /// <summary>Why the last sweep failed, empty if it did not.</summary>
    public string AeroError = "";

    /// <summary>
    /// Resample when the vehicle's geometry changes, not merely when it is looked at.
    ///
    /// KSA recomputes AerodynamicCdABody only in UpdateAfterPartTreeModification, so
    /// the table is exactly as stale as the bounding box is - which staging changes and
    /// nothing else does. Comparing the box the sweep was taken from against the live
    /// one catches that without hooking anything.
    /// </summary>
    public bool AeroStale(double3 liveExtents)
    {
        if (Aero == null) return true;
        double3 was = Aero.SampledExtents;
        double tol = 1e-3;
        return System.Math.Abs(was.X - liveExtents.X) > tol
            || System.Math.Abs(was.Y - liveExtents.Y) > tol
            || System.Math.Abs(was.Z - liveExtents.Z) > tol;
    }

    // --- impact prediction ---
    //
    // Per vehicle, not static, and for a reason that bit the older overlays: the
    // prediction is a property of ONE craft's state and drag model, so a booster and
    // the upper stage it just dropped must not share one. Cached rather than
    // recomputed per frame because a prediction is milliseconds, not microseconds.

    /// <summary>Last impact prediction, or default if there is none yet.</summary>
    public PoweredGuidance.Flight.ImpactPrediction Impact;

    /// <summary>Whether <see cref="Impact"/> holds anything at all.</summary>
    public bool HasImpact;

    /// <summary>The predictor's raw output buffer: four doubles per sample, x,y,z in
    /// CCI and the time from now at which the vehicle is there. Allocated once and
    /// reused - a per-frame array here would be several MB a second of garbage.</summary>
    public double[] ImpactPath;

    /// <summary>
    /// The same path converted to BODY-FIXED coordinates, once, at prediction time.
    ///
    /// Stored in CCF rather than CCI so the drawn track is glued to the ground. In CCI
    /// the correct de-rotation angle for each sample depends on how long ago the
    /// prediction was made, so a cached inertial path slides across the terrain
    /// between recalculations - about 90 m per 200 ms at Earth's equator - and then
    /// snaps back on the next one. In CCF nothing moves until the prediction actually
    /// changes.
    /// </summary>
    public double3[] ImpactPathCcf;

    /// <summary>Samples actually written to <see cref="ImpactPathCcf"/>.</summary>
    public int ImpactPathCount;

    /// <summary>Predicted impact point, body-fixed: the raw latest answer, and the
    /// smoothed one that is actually drawn. See the note on the smoothing in
    /// BoostbackOverlay - the raw point steps at the recalculation rate.</summary>
    public double3 ImpactCcfRaw, ImpactCcfShown;

    /// <summary>True once <see cref="ImpactCcfShown"/> holds something to blend from.</summary>
    public bool ImpactShownValid;

    /// <summary>Wall-clock tick of the last display blend, for a frame-rate
    /// independent smoothing factor.</summary>
    public long ImpactSmoothTick;

    /// <summary>
    /// Low-passed terrain height under the predicted impact, m.
    ///
    /// Low-passed because it is sampled at a point that MOVES: each recalculation
    /// looks up the height wherever the first pass landed, and over rough ground a
    /// few hundred metres of lateral movement can change it by hundreds of metres.
    /// Fed straight back into the target radius that produces a shallow-angle impact
    /// point, that is a multiplier - at a 20 degree descent a 500 m height change
    /// moves the impact 1.4 km sideways - and it was the largest single source of the
    /// marker jumping about.
    /// </summary>
    public double ImpactTerrainH;
    public bool ImpactTerrainValid;

    /// <summary>Scratch for the integrator, so a prediction allocates nothing.</summary>
    public PoweredGuidance.Numerics.Dual[] ImpactScratch;

    /// <summary>Wall-clock tick of the last prediction, throttling it. Wall clock and
    /// not sim time, for the same reason the launch-window scan uses it: under warp a
    /// sim-time gate would not throttle at all.</summary>
    public long ImpactTick;

    /// <summary>Impact latitude and longitude in degrees, body-fixed AT THE IMPACT
    /// TIME - not at the time of the prediction. The body turns under a coasting
    /// booster by roughly a degree per two minutes of flight, so the difference is
    /// tens of kilometres and it is the whole point of storing them separately.</summary>
    public double ImpactLatDeg, ImpactLonDeg;

    /// <summary>Downrange from the vehicle to the predicted impact, m.</summary>
    public double ImpactDownrangeM;

    // --- the shot boostback plan ---

    /// <summary>
    /// The optimised burn - pitch, yaw, turn rates and duration - from
    /// <see cref="PoweredGuidance.Flight.BoostbackShooter"/>. This is what the boostback phase
    /// FLIES, in place of the impulsive correction.
    ///
    /// WHY A PLAN RATHER THAN A DIRECTION. <see cref="SteerDv"/> answers "what is the
    /// smallest push, applied NOW". Over a real burn that answer points at the ground -
    /// measured 33 degrees below the horizon on the reference arc - because treating a
    /// fifty-second burn as an impulse hides the falling that happens during it. The
    /// shot integrates the powered arc instead, and comes out 25 degrees ABOVE the
    /// horizon for 18% less propellant; below the horizon the burn is not merely dear
    /// but impossible for this vehicle. Scvx.Console --shoot measures both.
    ///
    /// The impulsive correction is still computed, because it is what says whether
    /// there is any targeting work left at all - see SteerShape for that split.
    /// </summary>
    public PoweredGuidance.Flight.BurnParameters BoostbackPlan;

    /// <summary>
    /// The frame the plan's angles are measured in, captured at the solve.
    ///
    /// Stored rather than rebuilt, because rebuilding it from the current state would
    /// silently re-datum the angles: the frame's axes are local vertical and
    /// retrograde-horizontal, both of which turn through a burn, so the same pitch
    /// number means a different direction a few seconds later. That drift is exactly
    /// what the re-solve is for.
    /// </summary>
    public PoweredGuidance.Flight.BoostbackShooter.Frame BoostbackPlanFrame;

    /// <summary>
    /// Sim time the plan was SOLVED at. The steering law is a function of time since
    /// this and the duration is measured from it, so it is half of what the plan means
    /// - which is why it moves only when a solve succeeds. Advancing it on a failed
    /// attempt would silently rewind the burn clock by the interval and hand the law a
    /// tau that had already been flown.
    /// </summary>
    public double BoostbackPlanTime = double.NegativeInfinity;

    /// <summary>Sim time the last solve was ATTEMPTED at, successful or not. Separate
    /// from BoostbackPlanTime purely so a failing solve is still rate-limited.</summary>
    public double BoostbackPlanAttemptTime = double.NegativeInfinity;

    /// <summary>True once a plan has been solved and not invalidated.</summary>
    public bool BoostbackHasPlan;

    /// <summary>
    /// Set once the plan has HANDED OVER to the impulsive correction for the last few
    /// seconds of the burn - see PoweredGuidanceWindow.BoostbackTerminalS. Past this the
    /// plan is neither re-solved nor flown, and the plan clock is stale.
    /// </summary>
    public bool BoostbackTerminal;

    /// <summary>Why the last plan attempt failed, empty if it did not. The previous
    /// plan is kept when one fails, so this can be set while a good plan flies.</summary>
    public string BoostbackPlanError = "";

    /// <summary>Miss the plan converged to, m, and what it costs in propellant.</summary>
    public double BoostbackPlanMissM, BoostbackPlanPropellantKg;

    /// <summary>How long the last plan solve took, ms. Surfaced on the tab because the
    /// solve runs on the sim thread: this is the number that says whether the cadence
    /// is still affordable on the vehicle actually being flown. The reference arc
    /// measures 2.3 ms warm in Release and 24.6 in Debug, so read it against the build
    /// rather than against a remembered figure.</summary>
    public double BoostbackPlanSolveMs;

    /// <summary>Scratch for the shooter, kept per vehicle so the solve allocates
    /// nothing - same arrangement as ImpactScratch.</summary>
    public PoweredGuidance.Numerics.Dual[] BoostbackPlanScratch;

    // --- steering on the impact point ---

    /// <summary>
    /// The velocity correction that moves the predicted impact onto the landing site,
    /// in CCI, m/s. This is the damped Gauss-Newton step -J^+ m: the direction that
    /// NULLS the miss, which is not the same as the direction that reduces it
    /// fastest. See <see cref="SteerGreedy"/>.
    /// </summary>
    public double3 SteerDv;

    /// <summary>
    /// The greedy direction, -J^T m, normalised. Kept alongside the real correction
    /// because the two are easy to confuse and genuinely differ: they coincide only
    /// when the miss happens to lie along one of J's singular directions, and part
    /// company by ten degrees or so otherwise - which costs about a factor of two in
    /// residual per unit of dv. Drawn as the second arrow so the difference is
    /// visible rather than asserted.
    /// </summary>
    public double3 SteerGreedy;

    /// <summary>
    /// The FLIGHT-PATH-ANGLE shaping nudge, in CCI, m/s: a push along the free
    /// direction that changes the trajectory's shape without moving the impact point.
    ///
    /// Separate from <see cref="SteerDv"/> and not folded into it, because the two are
    /// used for different things and conflating them breaks the burn's termination.
    /// SteerDv is the TARGETING correction and its magnitude is what says how much
    /// targeting work is left - Boostback ends the burn when it drops below a floor. A
    /// shaping term added into that number would hold the magnitude up after the miss
    /// was nulled and the burn would never end on its own.
    ///
    /// So: <see cref="SteerCommand"/> is what to point along and how long to burn for,
    /// SteerDv alone is what says whether there is anything left to correct.
    /// </summary>
    public double3 SteerShape;

    /// <summary>What the vehicle should actually fly: the targeting correction plus
    /// the shaping nudge. Both have to be imparted, so this is also what sizes the
    /// burn.</summary>
    public double3 SteerCommand => SteerDv + SteerShape;

    /// <summary>How much vertical authority the free direction has, as the cosine
    /// between it and the local vertical. Near zero means shaping is not available on
    /// this geometry however much dv is spent - see ImpactSteering.FreeDirection.</summary>
    public double SteerFreeVertical;

    /// <summary>
    /// Pitch of the COMMANDED burn relative to the local horizon, degrees. Positive is
    /// above the horizon, negative is into the ground.
    ///
    /// Deliberately about the command and not the vehicle's flight path angle: a
    /// booster past apogee is descending regardless, and cancelling that is not the
    /// boostback's job. Thrusting further into it is what shaping prevents.
    /// </summary>
    public double SteerCmdPitchDeg;

    /// <summary>True when the requested pitch is above what the geometry can reach,
    /// so the command is short of <see cref="BoostbackPitchDeg"/>. Surfaced because a
    /// knob that silently fails to do what it says is worse than one that does
    /// nothing. See <see cref="SteerMaxPitchDeg"/>.</summary>
    public bool SteerPitchUnreachable;

    /// <summary>
    /// The highest pitch the free direction can reach, degrees - the pitch of the free
    /// direction itself.
    ///
    /// The commanded burn is (targeting dv + t * free direction), so as t grows the
    /// direction asymptotes to the free direction and the pitch asymptotes to ITS
    /// pitch. Everything below that is reachable and the cost runs to infinity at it.
    /// This is the real limit on the knob, and it is geometry rather than policy.
    /// </summary>
    public double SteerMaxPitchDeg;

    /// <summary>
    /// MINIMUM PITCH ABOVE THE HORIZON for the boostback burn, degrees. The one knob
    /// for flight-path-angle shaping, and it is honoured whatever it costs.
    ///
    /// Zero means "never thrust below the horizon". Positive lofts the burn, which is
    /// what buys flight time for a low-thrust vehicle. Set it very negative to switch
    /// shaping off entirely.
    ///
    /// THERE IS NO dV CAP, deliberately. The commanded burn is the targeting
    /// correction plus some multiple of the free direction, and moving along that
    /// direction does not change where the vehicle lands - so any pitch is reachable
    /// by simply moving further along it. The only limit is geometric: the command
    /// asymptotes to the free direction, so pitches at or above
    /// <see cref="SteerMaxPitchDeg"/> cost unbounded dV and are refused. An earlier
    /// version capped the spend at 60 m/s, which silently pinned the achievable pitch
    /// near -15 degrees whatever this was set to.
    ///
    /// It does still fade out with the burn without a cap, because the dv needed for a
    /// given pitch is PROPORTIONAL to the targeting correction - the ratio depends only
    /// on the angle and the geometry. So as the miss is nulled the shaping goes with it.
    ///
    /// A FLOOR, not a setpoint: shaping only ever raises the command, never pushes a
    /// burn that is already pointing high enough back down. There is no correct value
    /// to derive - the paper this follows (Jo, Han and Ahn) likewise picks its
    /// equivalent threshold by offline trajectory optimisation and says so. Tune it
    /// against flight results, watching total dV rather than the angle.
    /// </summary>
    public double BoostbackPitchDeg;

    /// <summary>Miss distance the correction was computed against, m.</summary>
    public double SteerMissM;

    /// <summary>True when the two above hold a usable answer.</summary>
    public bool HasSteer;

    /// <summary>Wall-clock tick of the last Jacobian, throttling it separately from
    /// the prediction: three seeded sweeps cost about four times one prediction.</summary>
    public long SteerTick;

    // --- the boostback state machine ---
    //
    // Separation -> Rotation -> Boostback -> EntryOrient, per vehicle like every other
    // phase machine here. See Guidance/Boostback.cs for what each phase does.

    public PoweredGuidanceWindow.BoostbackPhase BoostbackPhase =
        PoweredGuidanceWindow.BoostbackPhase.Idle;

    /// <summary>Sim time the current phase began, and of the previous step. The step
    /// time supplies the interval the slew limit and the sensed-dV integration run
    /// over; sim time rather than wall clock, so a warp step is the interval it
    /// really is.</summary>
    public double BoostbackPhaseStart;
    public double BoostbackLastStep;

    public string BoostbackStatus = "";

    // --- tuning (per vehicle: these describe one booster's airframe and mission) ---

    /// <summary>Settling burn at minimum throttle after separation, s.</summary>
    public double BoostbackSeparationS = 2.0;

    /// <summary>
    /// How fast the commanded attitude turns during the rotation and entry slews,
    /// deg/s. Unlike the ascent's MaxSlewDegS this is not a guard against a
    /// discontinuity - it IS the manoeuvre, and it is the rate the vehicle will
    /// actually fly the flip at, so it wants to be inside what the RCS and gimbals can
    /// hold rather than merely above what guidance asks for.
    /// </summary>
    public double BoostbackSlewDegS = 30.0;

    // --- live ---

    /// <summary>Attitude held through the settling burn, latched at EXECUTE. A fixed
    /// inertial direction rather than a live reading, so the hold is a hold and the
    /// rotation has something continuous to slew from.</summary>
    public double3 BoostbackHoldDir;

    /// <summary>Correction still to fly, m/s, and the rocket-equation burn time that
    /// buys it at full throttle and the current ambient pressure. Tgo is NaN when the
    /// vehicle has no usable engine model.</summary>
    public double BoostbackDvGo;
    public double BoostbackTgo;

    /// <summary>
    /// The terminal command has frozen and the last two seconds run open loop. See
    /// Guidance/Boostback.cs (BoostbackLockTgo) for why there is a tail at all.
    ///
    /// The two dV figures beside it ARE the cutoff, not a readout: BoostbackLockDv is
    /// what was owed at the freeze, BoostbackAccumDv is the dV sensed since, integrated
    /// from the thrust the lit engines are actually producing at this altitude, and the
    /// engine stops when the second catches the first. Sensed rather than timed, so the
    /// tail self-corrects about pressure and mass with nothing being re-solved.
    /// </summary>
    public bool BoostbackLocked;
    public double3 BoostbackFrozenDir;
    public double BoostbackLockDv;
    public double BoostbackAccumDv;

    /// <summary>Sim time the burn is abandoned at regardless, sized off the burn time
    /// predicted at ignition. The closed loop has no convergence guarantee, and nothing
    /// else in it ever says stop.</summary>
    public double BoostbackBurnLimit = double.PositiveInfinity;

    /// <summary>Previous step's target direction and the low-passed rate differenced
    /// from it — the feedforward published to the flight computer.</summary>
    public double3 BoostbackPrevWant;
    public bool BoostbackPrevWantValid;
    public double3 BoostbackWantRate;

    /// <summary>What the step wants the engine at. Applied in ApplyAutopilot, which is
    /// where writes to _manualControlInputs reach the sim.</summary>
    public double BoostbackThrottle;
    public bool BoostbackEngineOn;

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
