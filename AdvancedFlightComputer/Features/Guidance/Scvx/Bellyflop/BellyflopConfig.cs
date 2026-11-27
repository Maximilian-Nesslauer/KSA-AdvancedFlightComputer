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

    /// <summary>
    /// Price of the crossrange flown, per km of its mean over the nodes (L1): the distance off the vertical plane through the site along the bearing, at every node, not only at the end. With the miss priced only at the end, the plan was free to take crossrange out at any time, and the effort price made spreading it over the whole descent cheapest: bank was lazy, and the craft came down still beside the site (2026-11-24). The nodes are even in time, so this is the crossrange's time average: nulled early, it costs least. Far under the miss and the dynamics.
    /// </summary>
    public double CrossWeight { get; set; } = 300.0;

    /// <summary>Price of pitch and bank, per deg^2 over the nodes (a mean): the plan flies as near belly-first as it can.</summary>
    public double EffortWeight { get; set; } = 0.01;

    /// <summary>Price of bank alone, per deg^2 over the nodes (a mean); NaN prices it as pitch (EffortWeight). Bank is the airflow towards the span: the craft rolled about its long axis, held against the air by the flaps' roll trim. Priced above pitch, the plan steers as an aeroplane banks, turning its lift with yaw and pitch (BANK TO TURN), and keeps bank for what yaw cannot do quickly. Experiment, off by default (harness, 2026-11-27).</summary>
    public double BankWeight { get; set; } = double.NaN;

    /// <summary>Price of yaw, per deg^2 over the nodes (a mean), on top of the L1 price: with YawWeight lowered, a smooth price that keeps a free yaw from wandering. 0 off.</summary>
    public double YawQuadWeight { get; set; } = 0.0;

    /// <summary>
    /// LEVEL BY THE END: over a plan's last EndLevelS seconds bank and yaw (and pitch at EndLevelPitchWeight) are priced EndLevelWeight per deg^2 more, rising from nothing EndLevelS before the plan's end to all of it at half that: the craft comes to the flip wings level and nose along the bearing, whatever it steered with on the way. 0 s off.
    /// </summary>
    public double EndLevelS { get; set; } = 0.0;
    public double EndLevelWeight { get; set; } = 1.0;
    public double EndLevelPitchWeight { get; set; } = 0.0;

    /// <summary>Price of yaw, per deg over the nodes (a mean, L1), near belly-first: far above bank's, so the plan turns the yaw only once bank is pinned at its limit. While the plan glides it is cheaper (YawGlideScale).</summary>
    public double YawWeight { get; set; } = 2.0;

    /// <summary>The share of YawWeight yaw costs at a node gliding at YawGlideFromDeg of pitch or more, falling linearly from 1 at AngleMaxDeg: gliding, the yaw turns the lift as the entry's bank does, and is how the plan should take out crossrange, not a last resort (2026-11-23: the plan unwound the entry's 63 deg at the take-over and fought 10 km of crossrange with sideslip). Offline on the flat plate the plan still takes out crossrange with bank (sideslip) rather than yaw at any price tried (0.02 to 1); the approach cone is what makes it do so early.</summary>
    public double YawGlideScale { get; set; } = 0.1;
    public double YawGlideFromDeg { get; set; } = 25.0;

    /// <summary>The largest yaw off the bearing to the site, deg: the plan may curve towards the site, not turn round. A turn-round planned at full pitch was more than the warm re-solves could follow from the real state, and the craft flew the stale plan 132 km past the site (harness, 2026-11-23).</summary>
    public double YawMaxDeg { get; set; } = 90.0;

    /// <summary>The approach cone over the site: below ApproachConeTopM (of the plan's own altitude) the plan stays within ApproachConeDeg of the vertical through the site, plus ApproachConeFloorM, so it takes the miss out while it still has the speed to, and the last of the fall is steep. Priced as the defects (soft). 0 deg switches it off.</summary>
    public double ApproachConeDeg { get; set; } = 15.0;
    public double ApproachConeTopM { get; set; } = 12_000.0;
    public double ApproachConeFloorM { get; set; } = 300.0;
    /// <summary>Price of being outside the cone, per km: half the miss's, and far under the dynamics' (PenaltyWeight), so a craft already outside it is brought back as it can be, and the plan never bends its dynamics to keep to it.</summary>
    public double ConeWeight { get; set; } = 500.0;

    /// <summary>
    /// THE LATERAL APPROACH CONE: a cone lying on its side, its apex at the aim and opening back along the bearing, the way the craft comes from: at every node |crossrange| &lt;= tan(LateralConeDeg) x the distance still to go along the bearing, plus LateralConeFloorM. Past the aim the distance to go is negative and nothing is inside, so a plan never passes the aim and comes back: the falling leaf, overshooting and swinging back, which the miss priced only at the end and the vertical cone round the site both allowed (2026-11-24). Soft, priced on the worst node's excess (LateralConeWeight), so going past at any point costs the same as at the end. 0 deg switches it off.
    /// </summary>
    public double LateralConeDeg { get; set; } = 60.0;
    public double LateralConeFloorM { get; set; } = 25.0;
    /// <summary>Price of the worst node's excess outside the lateral cone, per km: five times the miss, half one node's position defect (PenaltyWeight), so no defect ever buys its way into the cone.</summary>
    public double LateralConeWeight { get; set; } = 5000.0;

    /// <summary>The most the cone may cost per metre outside it, summed over all its nodes, as a share of one node's position defect price (PenaltyWeight per km): under it, no defect ever buys its way into the cone (BellyflopPlanner.ConeNodePrice).</summary>
    public double ConeShareOfPenalty { get; set; } = 0.5;

    /// <summary>The share of the pitch-down and bank limits a plan may count on beyond its first MarginAfterS: the next plan, a second later, has the whole limit again, so a plan never spends the authority the craft will need to correct with (2026-11-23: plans sat at the edge of reach and missed by 0.5 km at the end).</summary>
    public double LimitMargin { get; set; } = 0.75;
    public double MarginAfterS { get; set; } = 20.0;

    /// <summary>Price of the change of each angle between nodes, per deg^2.</summary>
    public double RateWeight { get; set; } = 0.01;

    /// <summary>Exact-penalty price of the dynamics defects and the terminal altitude, per scaled unit (10 per metre of position): far above the miss (1 per metre) and the cone, so the plan never closes a miss by bending its dynamics. At 1e3 a metre of defect cost a metre of miss, and plans for a site out of reach were refused with defects matching their misses (harness, 2026-11-23).</summary>
    public double PenaltyWeight { get; set; } = 1e4;

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
