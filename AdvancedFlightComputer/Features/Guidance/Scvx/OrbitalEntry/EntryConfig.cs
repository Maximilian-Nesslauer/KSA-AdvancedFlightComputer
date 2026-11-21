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

    /// <summary>How long the tracker may stay past <see cref="BankMaxDeg"/>, s.</summary>
    public double OverbankMaxS { get; set; } = 20.0;

    /// <summary>Roll rate the vehicle can sustain, deg/s. Sets the plan's rate limit and the tracker's slew.</summary>
    public double RollRateDegS { get; set; } = 5.0;

    /// <summary>Share of the roll rate the plan may use, so reversals still fit.</summary>
    public double PlanRollRateShare { get; set; } = 0.5;

    // ---- Start and end ----

    /// <summary>Drag at which entry guidance takes hold, m/s^2, latched once reached (EntryTracker.InAir). Before it the vehicle coasts lift-up.</summary>
    public double StartDrag { get; set; } = 0.2;

    /// <summary>Hand-over altitude, m.</summary>
    public double HandoverAltitude { get; set; } = 25_000.0;

    /// <summary>Hand-over Mach number. With the altitude it fixes the final energy.</summary>
    public double HandoverMach { get; set; } = 2.0;

    /// <summary>Half-width of the hand-over altitude box, m.</summary>
    public double HandoverAltitudeTol { get; set; } = 3_000.0;

    // ---- Solver ----

    public int Nodes { get; set; } = 60;
    public int SubSteps { get; set; } = 6;
    public int ColdIterations { get; set; } = 25;
    public int WarmIterations { get; set; } = 6;

    /// <summary>Spread the linearisation over the thread pool. Off in the game, where the planner has a thread of its own.</summary>
    public bool ParallelLinearise { get; set; } = false;

    /// <summary>Smoothness weight on changes in vertical L/D between nodes.</summary>
    public double SmoothWeight { get; set; } = 0.05;

    /// <summary>Weight of the heat-load objective (normalised by the seed's).</summary>
    public double HeatWeight { get; set; } = 1.0;

    /// <summary>Exact-penalty weight on virtual control and slacks.</summary>
    public double PenaltyWeight { get; set; } = 1_000.0;

    /// <summary>Proximal weight on moving from the reference, per scaled unit squared.</summary>
    public double ProximalWeight { get; set; } = 1e-3;

    /// <summary>Largest scaled defect a published plan may have (scales in <see cref="EntryPlanner"/>).</summary>
    public double GateDefect { get; set; } = 0.05;

    /// <summary>Largest terminal range error a published plan may have, m.</summary>
    public double GateRangeM { get; set; } = 25_000.0;

    // ---- Tracker ----

    /// <summary>Natural frequency of the drag-tracking loop, rad/s.</summary>
    public double TrackOmega { get; set; } = 0.06;

    /// <summary>Damping ratio of the drag-tracking loop.</summary>
    public double TrackZeta { get; set; } = 0.7;

    /// <summary>Crossrange deadband intercept, rad of arc (Lu 2008 Eq. 23, c0).</summary>
    public double DeadbandC0 { get; set; } = 8.71e-5;

    /// <summary>Crossrange deadband slope, rad of arc per unit of normalised speed (Lu 2008 Eq. 23, c1).</summary>
    public double DeadbandC1 { get; set; } = 5.21e-3;

    public EntryConfig Clone() => (EntryConfig)MemberwiseClone();
}
