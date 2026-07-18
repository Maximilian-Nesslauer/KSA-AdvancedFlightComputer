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

    public bool Matches(double timeSec, double dvMs)
        => Math.Abs(BurnTimeSec - timeSec) < TimeMatchToleranceSec
           && Math.Abs(BurnDvMs - dvMs) < DvMatchToleranceMs;
}
