#nullable disable

namespace AdvancedFlightComputer.Features.Guidance.Upfg;

// A propulsion stage as UPFG models it. Mode 1 uses constant thrust, and Mode 2 uses constant acceleration with a g-limit. The guidance loop accepts a list of stages built from the vehicle's live engine configuration.
public sealed class UpfgStage
{
    public int Mode = 1;
    public double Thrust;      // N (vacuum)
    public double Isp;         // s
    public double MassTotal;   // kg, wet (current)
    public double MassDry;     // kg, at burnout
    public double GLim;        // acceleration limit in g's (Mode 2); large = unlimited

    // These values identify the staging sequence and the engine-core count for the stage table. The guidance solver does not read them.
    public int Seq = -1;
    public int Engines;
}

public sealed class UpfgVehicle
{
    public System.Collections.Generic.List<UpfgStage> Stages { get; } = new();
}
