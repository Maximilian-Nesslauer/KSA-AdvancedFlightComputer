namespace AdvancedFlightComputer.Guidance.Scvx.Bellyflop;

/// <summary>Bellyflop's settings: the limits, the weights and the solve's own knobs.</summary>
public sealed class BellyflopConfig
{
    /// <summary>Nodes over the descent.</summary>
    public int Nodes { get; set; } = 30;

    /// <summary>RK4 steps per interval when shooting.</summary>
    public int SubSteps { get; set; } = 4;

    /// <summary>The largest pitch the other way, the nose down past belly-first (the airflow towards the tail), deg.</summary>
    public double AngleMaxDeg { get; set; } = 20.0;

    /// <summary>The largest pitch nose up from belly-first (the airflow towards the nose), deg: far enough to cover the entry's own attitude (40 deg from nose-first is 50 here), so the plan may go on gliding as the entry does and ease to belly-first as the range allows, rather than being made to turn belly-first at the take-over (2026-11-23).</summary>
    public double PitchUpMaxDeg { get; set; } = 55.0;

    /// <summary>The largest angle of the airflow on the belly towards the span: bank from belly-first, deg. Wider than pitch: the bank is the plan's sideways reach, which ran out at 20 deg in flight (2026-11-23). The belly-first tables reach 45 deg of sideslip (KsaAeroSweep.BellyBetaTableDeg).</summary>
    public double BankMaxDeg { get; set; } = 45.0;

    /// <summary>How fast the plan may turn pitch and bank, deg/s: what the flaps can follow.</summary>
    public double AngleRateDegS { get; set; } = 5.0;

    /// <summary>How fast the plan may turn the yaw, deg/s.</summary>
    public double YawRateDegS { get; set; } = 5.0;

    /// <summary>The plan's first node may be no further than this from the attitude flown when it was solved, s at the angle rate.</summary>
    public double StartSlackS { get; set; } = 0.5;

    /// <summary>The altitude the plan ends at, m above the site: the ground, the touchdown solver choosing the flip on the way down.</summary>
    public double TargetAltitude { get; set; } = 0.0;

    /// <summary>Price of the horizontal miss at the end, per km (L1): 1 per metre, so a degree of yaw held all the way down (YawWeight) is worth 2 m of miss. At 50 per km, 10 deg of yaw was worth 400 m, and the plan let the miss grow at the end rather than turn (2026-11-23).</summary>
    public double MissWeight { get; set; } = 1000.0;

    /// <summary>Price of pitch and bank, per deg^2 over the nodes (a mean): the plan flies as near belly-first as it can.</summary>
    public double EffortWeight { get; set; } = 0.01;

    /// <summary>Price of yaw, per deg over the nodes (a mean, L1): far above bank's, so the plan turns the yaw only once bank is pinned at its limit.</summary>
    public double YawWeight { get; set; } = 2.0;

    /// <summary>Price of the change of each angle between nodes, per deg^2.</summary>
    public double RateWeight { get; set; } = 0.01;

    /// <summary>Exact-penalty price of the dynamics defects and the terminal altitude, per scaled unit.</summary>
    public double PenaltyWeight { get; set; } = 1e3;

    /// <summary>A whisper of distance from the reference, for a unique step.</summary>
    public double ProximalWeight { get; set; } = 1e-4;

    /// <summary>The flight time's bounds, s.</summary>
    public double SigmaMin { get; set; } = 5.0;
    public double SigmaMax { get; set; } = 900.0;

    public int ColdIterations { get; set; } = 40;
    public int WarmIterations { get; set; } = 12;

    /// <summary>A plan is published as flyable within these: the largest scaled defect, and the horizontal miss, m.</summary>
    public double GateDefect { get; set; } = 0.05;
    public double GateMissM { get; set; } = 500.0;
}
