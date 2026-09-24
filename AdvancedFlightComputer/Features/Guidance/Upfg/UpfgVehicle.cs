#nullable disable

namespace AdvancedFlightComputer.Features.Guidance.Upfg;

// A propulsion stage as UPFG models it. Mode 1 uses constant thrust, and Mode 2 uses constant acceleration with a g-limit. The guidance loop accepts a list of stages built from the vehicle's live engine configuration.
public sealed class UpfgStage
{
    public int Mode = 1;
    public double Thrust;      // N, the burning stage at ambient pressure
    public double Isp;         // s
    public double MassTotal;   // kg, wet (current)
    public double MassDry;     // kg, at burnout
    public double GLim;        // acceleration limit in g's (Mode 2); large = unlimited

    // These values identify the staging sequence and the engine-core count for the stage table. The guidance solver does not read them.
    public int Seq = -1;
    public int Engines;

    // Full-throttle thrust (N) at each back pressure of PressureGrid (Pa), for the convex ascent planner, which flies the burning stage through the thinning air. Filled only when KsaVehicleAdapter.Build is given a grid; UPFG does not read them.
    public double[] PressureGrid;
    public double[] ThrustAtPressure;
}

public sealed class UpfgVehicle
{
    public System.Collections.Generic.List<UpfgStage> Stages { get; } = new();

    /// <summary>
    /// The factor the burning stage's thrust and Isp were scaled by for ambient pressure,
    /// 1.0 when uncorrected. A readout that compares against the stock model divides it out.
    /// </summary>
    public double BurningStageThrustRatio = 1.0;
}
