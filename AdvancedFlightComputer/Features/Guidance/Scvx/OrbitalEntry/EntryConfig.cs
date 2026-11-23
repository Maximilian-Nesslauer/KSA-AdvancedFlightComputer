namespace AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;

/// <summary>
/// Limits and solver settings for the orbital entry planner and its tracker. SI and degrees unless named otherwise.
///
/// A limit of zero or less is off.
/// </summary>
public sealed class EntryConfig
{
    // ---- Path limits (become altitude floors) ----

    /// <summary>Stagnation heating limit, W/m^2. Off at zero.</summary>
    public double HeatingRateMax { get; set; } = 0.0;

    /// <summary>Dynamic pressure limit, Pa.</summary>
    public double DynamicPressureMax { get; set; } = 16_000.0;

    /// <summary>Aerodynamic load limit, Earth g.</summary>
    public double LoadFactorMax { get; set; } = 2.5;

    /// <summary>Lowest altitude the plan may fly, m, above the body radius.</summary>
    public double AltitudeFloor { get; set; } = 5_000.0;

    // ---- Bank limits ----

    /// <summary>Smallest bank the plan uses, deg: keeps side lift for the roll-reversal deadband to turn with.</summary>
    public double BankMinDeg { get; set; } = 15.0;

    /// <summary>Largest bank the plan uses, deg.</summary>
    public double BankMaxDeg { get; set; } = 100.0;

    /// <summary>Largest bank the tracker may command, deg: further than the plan's, so it can absorb what the plan did not foresee.</summary>
    public double OverbankDeg { get; set; } = 120.0;

    /// <summary>Roll rate of a reversal, deg/s; the plan uses PlanRollRateShare of it. 10 flew too fast a reversal (2026-11-21), and with the tracking retuned 5 is enough.</summary>
    public double RollRateDegS { get; set; } = 5.0;

    /// <summary>Share of the roll rate the plan may use, so reversals still fit.</summary>
    public double PlanRollRateShare { get; set; } = 0.5;

    // ---- Start and end ----

    /// <summary>Drag at which entry guidance takes hold, m/s^2, latched once reached (EntryTracker.InAir). Before it the vehicle coasts lift-up.</summary>
    public double StartDrag { get; set; } = 0.02;

    /// <summary>Drag the plan starts from, m/s^2: before it the planner predicts the coast down to it. Kept apart from <see cref="StartDrag"/>: in air this thin the energy hardly changes while the altitude falls tens of km, and a plan started there does not converge.</summary>
    public double PlanStartDrag { get; set; } = 0.5;

    /// <summary>The bank flown from the first touch of the air until the plan takes hold, deg, and the one the coast down to the plan's start is predicted with: the same, so the prediction is of the flight. Past 90 it digs in, a little lift down, rather than floating on.</summary>
    public double PreStartBankDeg { get; set; } = 100.0;

    /// <summary>Take hold as soon as the craft is below the top of the air, whatever the drag: a shallow entry spends a long time in air too thin to reach any start drag.</summary>
    public bool EngageInAtmosphere { get; set; } = true;

    /// <summary>
    /// The least drag the tracker's gains are taken at, m/s^2: they go as 1/D and 1/D^2, because the vertical acceleration a change of L/D buys goes as D. Below it the loop keeps its damping and slows in proportion, to w D / floor. At 1 m/s^2 (2026-11-21) a 5-10 % drag error asked for five times the 35 % clamp at 70-75 km, and the bank banged between 55 and 100 deg every 10 s for little effect; the re-plans every 2 s absorb what a softer loop leaves. Offline, 1 to 8 changed no miss by more than 1 km. The tuning (w 0.03, floor 10, 4 s measurement filter) is from --orbital-entry-gains with 3 % drag noise, the flutter measured in flight: the drag error was the same at every stiffness (6.5 % in thin air, 1 % lower down, set by the noise and the model), and this cut the bank's travel 5-25 times; a 4 s filter on the old w 0.06 left the low-L/D craft at its clamp and 21 km long.
    /// </summary>
    public double GainDragFloor { get; set; } = 10.0;

    /// <summary>The roll rate the tracker slews its bank at while tracking, deg/s; reversals and the first roll use <see cref="RollRateDegS"/>.</summary>
    public double TrackRollRateDegS { get; set; } = 5.0;

    /// <summary>Time constant of the low-pass on the vertical L/D the tracker commands, s; 0 for none.</summary>
    public double CommandSmoothS { get; set; } = 0.0;

    /// <summary>Hand-over altitude, m.</summary>
    public double HandoverAltitude { get; set; } = 25_000.0;

    /// <summary>Hand-over Mach number. With the altitude it fixes the final energy.</summary>
    public double HandoverMach { get; set; } = 2.0;

    /// <summary>Half-width of the hand-over altitude box, m.</summary>
    public double HandoverAltitudeTol { get; set; } = 3_000.0;

    /// <summary>The range to go to the site at the hand-over, m: what the terminal descent after it covers (bellyflop's predicted glide to the ground). Zero aims the hand-over itself at the site.</summary>
    public double TerminalRangeM { get; set; }

    // ---- Solver ----

    public int Nodes { get; set; } = 60;
    public int SubSteps { get; set; } = 6;
    public int ColdIterations { get; set; } = 60;
    public int WarmIterations { get; set; } = 15;

    /// <summary>Spread the linearisation over the thread pool. Off in the game, where the planner has a thread of its own.</summary>
    public bool ParallelLinearise { get; set; } = false;

    /// <summary>Smoothness weight on changes in vertical L/D between nodes.</summary>
    public double SmoothWeight { get; set; } = 0.05;

    /// <summary>Weight on the plan's curvature in vertical L/D (second differences between nodes): a reference that bends gently, so the tracker and the flaps are not asked to follow corners.</summary>
    public double CurvatureWeight { get; set; } = 0.5;

    /// <summary>Weight pulling a re-solve's vertical L/D towards the plan before it, so successive plans do not jump.</summary>
    public double ConsistencyWeight { get; set; } = 0.2;

    /// <summary>
    /// Weight keeping the plan's range to go near the middle of the corridor at every energy (the TRAJ display's two lines). The heat load alone pushes the plan to the corridor's high-energy edge: 2026-11-21, on plan to the metre but closing on the edge, it had no drag left to spare at the end and landed long.
    /// </summary>
    public double CorridorWeight { get; set; } = 1.0;

    /// <summary>Weight of the heat-load objective (normalised by the seed's).</summary>
    public double HeatWeight { get; set; } = 1.0;

    /// <summary>Exact-penalty weight on virtual control and slacks.</summary>
    public double PenaltyWeight { get; set; } = 1_000.0;

    /// <summary>Proximal weight on moving from the reference, per scaled unit squared.</summary>
    public double ProximalWeight { get; set; } = 1e-3;

    /// <summary>Largest scaled defect a published plan may have (scales in <see cref="EntryPlanner"/>: 0.25 is 1.25 km, 0.25 deg, 5 km of range). Re-solved every few seconds, a plan this close is flyable; late in the entry the roll-rate limit leaves a few tenths that 0.05 refused (2026-11-21 offline, 47 of 114).</summary>
    public double GateDefect { get; set; } = 0.25;

    /// <summary>Largest terminal range error a published plan may have, m.</summary>
    public double GateRangeM { get; set; } = 25_000.0;

    // ---- Tracker ----

    /// <summary>Natural frequency of the drag-tracking loop, rad/s.</summary>
    public double TrackOmega { get; set; } = 0.03;

    /// <summary>The most the drag feedback may add to or take from the plan's vertical L/D, as a share of the craft's L/D: the plan, re-solved every few seconds, does the work, the feedback only trims it.</summary>
    public double FeedbackShare { get; set; } = 0.35;

    /// <summary>Time constant of the low-pass on the measured drag and sink rate the feedback reads, s.</summary>
    public double MeasureSmoothS { get; set; } = 4.0;

    /// <summary>Damping ratio of the drag-tracking loop.</summary>
    public double TrackZeta { get; set; } = 0.7;

    /// <summary>
    /// The heading error to the site at which the bank reverses, deg, at speed; below <see cref="ReversalBandFullSpeed"/> (normalised) it narrows in proportion, to half at half that speed and a fifth at a fifth, so the craft still lines up on the site at the end. Offline, 5 deg kept every case within 4 km; 7.5 and 10 (the Shuttle's, at L/D 1) left the L/D 0.3 capsule 16 and 24 km to the side, its single reversal too late to come back. An angle rather than Lu's crossrange distance: a fixed 30 km was under a degree of heading 2000 km out, and the craft reversed again and again (2026-11-21).
    /// </summary>
    public double ReversalBandDeg { get; set; } = 5.0;
    public double ReversalBandFullSpeed { get; set; } = 0.25;   // about 2 km/s on Earth: half the band at 1 km/s, a fifth at 400 m/s

    /// <summary>The reversal band at a normalised speed, rad.</summary>
    public double ReversalBand(double speedNorm)
        => ReversalBandDeg * Math.PI / 180.0 * Math.Clamp(speedNorm / ReversalBandFullSpeed, 0.2, 1.0);

    public EntryConfig Clone() => (EntryConfig)MemberwiseClone();
}
