namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>How a burn should be executed when the user engages Auto.</summary>
internal enum RcsExecutionMode
{
    /// <summary>Engine when an active engine with propellant exists, otherwise RCS.</summary>
    Default,
    Engine,
    Rcs,
}

/// <summary>Attitude handling while an RCS translation burn runs.</summary>
internal enum RcsAttitudeStrategy
{
    /// <summary>Pick Hold or Align by comparing propellant estimates.</summary>
    Auto,

    /// <summary>Keep the current attitude and fire whatever axis mix points at the target.</summary>
    Hold,

    /// <summary>Rotate the most capable translation axis onto the burn vector first.</summary>
    Align,
}

/// <summary>How the demanded impulse is turned into thruster pulses.</summary>
internal enum RcsAllocator
{
    /// <summary>Default. Stock-consistent signed-axis groups; residual torque
    /// is corrected by the attitude hold. Also the automatic fallback when the
    /// LP is infeasible for a layout.</summary>
    Groups,

    /// <summary>Fuel-optimal LP over the raw per-thruster wrenches with the
    /// zero-net-torque constraint folded in. Fuel-par and cleaner (torque-
    /// nulled) on balanced layouts, but can cost far more where a thruster
    /// axis is strongly off the CoM, since it fires opposed counter-thrust for
    /// exact zero torque. Falls back to Groups when the constraint set is
    /// infeasible for the current layout.</summary>
    Lp,
}

/// <summary>Per-burn RCS configuration, keyed by burn identity (time + dV magnitude)
/// because stock burns carry no stable id across save/load.</summary>
internal sealed class RcsBurnOptions
{
    public const double TimeMatchToleranceSec = 0.05;
    public const double DvMatchToleranceMs = 0.1;

    public required double BurnTimeSec { get; set; }
    public required double BurnDvMs { get; set; }
    public RcsExecutionMode Mode { get; set; } = RcsExecutionMode.Default;
    public RcsAttitudeStrategy Attitude { get; set; } = RcsAttitudeStrategy.Auto;

    /// <summary>Defaults to Groups: stock-consistent and robust on every
    /// layout. LP is fuel-par and cleaner on balanced layouts but can cost far
    /// more where an axis is strongly off the CoM, so it is a per-burn opt-in.
    /// See <see cref="RcsAllocator"/>.</summary>
    public RcsAllocator Allocator { get; set; } = RcsAllocator.Groups;

    public bool Matches(double timeSec, double dvMs)
        => Math.Abs(BurnTimeSec - timeSec) < TimeMatchToleranceSec
           && Math.Abs(BurnDvMs - dvMs) < DvMatchToleranceMs;
}
