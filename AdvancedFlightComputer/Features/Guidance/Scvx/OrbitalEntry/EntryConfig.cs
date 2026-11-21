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

    /// <summary>Largest bank the tracker may command, deg, for at most <see cref="OverbankMaxS"/> at a time.</summary>
    public double OverbankDeg { get; set; } = 120.0;

    /// <summary>How long the tracker may stay past <see cref="BankMaxDeg"/>, s; 0 for as long as the sink needs it. On 2026-11-21 a 20 s limit dropped the bank back to 100 deg mid-correction and left every entry a little high.</summary>
    public double OverbankMaxS { get; set; } = 0.0;

    /// <summary>Roll rate the vehicle can sustain, deg/s. Sets the plan's rate limit and the tracker's slew.</summary>
    public double RollRateDegS { get; set; } = 5.0;

    /// <summary>Share of the roll rate the plan may use, so reversals still fit.</summary>
    public double PlanRollRateShare { get; set; } = 0.5;

    // ---- Start and end ----

    /// <summary>Drag at which entry guidance takes hold, m/s^2, latched once reached (EntryTracker.InAir). Before it the vehicle coasts lift-up.</summary>
    public double StartDrag { get; set; } = 0.02;

    /// <summary>Drag the plan starts from, m/s^2: before it the planner predicts the coast down to it. Kept apart from <see cref="StartDrag"/>: in air this thin the energy hardly changes while the altitude falls tens of km, and a plan started there does not converge.</summary>
    public double PlanStartDrag { get; set; } = 0.5;

    /// <summary>Take hold as soon as the craft is below the top of the air, whatever the drag: a shallow entry spends a long time in air too thin to reach any start drag.</summary>
    public bool EngageInAtmosphere { get; set; } = true;

    /// <summary>The least drag the tracker's gains are taken at, m/s^2: they go as 1/D and 1/D^2.</summary>
    public double GainDragFloor { get; set; } = 1.0;

    /// <summary>Time constant of the low-pass on the vertical L/D the tracker commands, s; 0 for none.</summary>
    public double CommandSmoothS { get; set; } = 0.0;

    /// <summary>Hand-over altitude, m.</summary>
    public double HandoverAltitude { get; set; } = 25_000.0;

    /// <summary>Hand-over Mach number. With the altitude it fixes the final energy.</summary>
    public double HandoverMach { get; set; } = 2.0;

    /// <summary>Half-width of the hand-over altitude box, m.</summary>
    public double HandoverAltitudeTol { get; set; } = 3_000.0;

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
    public double CorridorWeight { get; set; } = 0.3;

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
    public double TrackOmega { get; set; } = 0.06;

    /// <summary>The most the drag feedback may add to or take from the plan's vertical L/D, as a share of the craft's L/D: the plan, re-solved every few seconds, does the work, the feedback only trims it.</summary>
    public double FeedbackShare { get; set; } = 0.35;

    /// <summary>Time constant of the low-pass on the measured drag and sink rate the feedback reads, s.</summary>
    public double MeasureSmoothS { get; set; } = 1.0;

    /// <summary>Damping ratio of the drag-tracking loop.</summary>
    public double TrackZeta { get; set; } = 0.7;

    /// <summary>Crossrange deadband intercept, rad of arc (Lu 2008 Eq. 23, c0).</summary>
    public double DeadbandC0 { get; set; } = 8.71e-5;

    /// <summary>Crossrange deadband slope, rad of arc per unit of normalised speed (Lu 2008 Eq. 23, c1).</summary>
    public double DeadbandC1 { get; set; } = 5.21e-3;

    public EntryConfig Clone() => (EntryConfig)MemberwiseClone();
}
