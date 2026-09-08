namespace PoweredGuidance.Upfg;

// A propulsion stage as UPFG models it. Mode 1 = constant thrust, Mode 2 = constant
// acceleration (g-limited). For the first KSA integration we build a single constant-
// thrust stage from the vehicle's live engine configuration, but the guidance loop is
// written to accept a list so multi-stage can be added later.
public sealed class UpfgStage
{
    public int Mode = 1;
    public double Thrust;      // N (vacuum)
    public double Isp;         // s
    public double MassTotal;   // kg, wet (current)
    public double MassDry;     // kg, at burnout
    public double GLim;        // acceleration limit in g's (Mode 2); large = unlimited

    // Provenance, for the stage table only — nothing in the guidance reads these.
    // Which staging sequence this arc came out of and how many engine cores the
    // game's drain simulation had burning across it: the two numbers that say
    // whether a row is a real stage or an artefact of how the burn was sliced up.
    public int Seq = -1;
    public int Engines;
}

public sealed class UpfgVehicle
{
    public System.Collections.Generic.List<UpfgStage> Stages { get; } = new();
}
