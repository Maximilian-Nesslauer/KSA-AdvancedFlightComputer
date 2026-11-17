using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Numerics.Flight;

namespace AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// A booster the ascent plans to bring home (#84): which stage it is, what it weighs and burns, where it lands and how it flies there. With one on the problem, the propellant it keeps back is not fixed beforehand but chosen by the solve, held to what the booster needs from wherever the plan separates it.
///
/// WHAT THE BOOSTER NEEDS. Two burns, by the booster's own rocket equation from its dry mass:
///   R_req = m_dry (exp((BoostbackMargin dv_boostback + dv_landing) / v_e) - 1)
/// dv_boostback is the smallest impulse at separation that puts the ballistic impact on the site, solved on the same drag-and-lift coast the in-flight return estimate and the boostback glide use (ImpactPredictor, ImpactSteering); dv_landing is LandingFactor times the booster's terminal velocity at sea level, engine first. See <see cref="BoostbackOracle"/>.
/// </summary>
public sealed class AscentRecovery
{
    /// <summary>The stage whose burn-out separates the booster: the end of the first liquid load. Its propellant and jettison are the problem's own, with <see cref="InitialReserveKg"/> already taken out of the one and added to the other.</summary>
    public required int Stage { get; init; }
    /// <summary>The reserve the stage list was built with, kg: where the solve starts. It may move either way from here (AscentSolution.RecoveryReserveKg).</summary>
    public required double InitialReserveKg { get; init; }
    /// <summary>The booster's dry mass, kg: what leaves at separation besides the reserve. For side boosters leaving together, all of theirs (<see cref="BoostersOnStage"/>).</summary>
    public required double BoosterDryKg { get; init; }
    /// <summary>
    /// How many boosters leave together at this separation and share <see cref="BoosterDryKg"/> and the reserve: two for a pair of side boosters. Each flies home alone, so its coast and its terminal velocity go with its own share of the mass against its own drag area (<see cref="ReferenceArea"/>); the reserve stays the pair's.
    /// </summary>
    public int BoostersOnStage { get; init; } = 1;
    /// <summary>
    /// The share of <see cref="Stage"/>'s propellant flow that the separating booster draws, in (0, 1]. One for a booster that is all that burns in its stage.
    ///
    /// SIDE BOOSTERS (#176). Below one, the booster leaves beside a core whose liquid burns on into the next stage, and the one throttle drives both. A reserve R kept in the booster ends the stage R / share of propellant sooner, and the core's part of that, R (1 / share - 1), is still aboard. It is burned in the next stage, not jettisoned with the booster.
    /// </summary>
    public double FlowShare { get; init; } = 1.0;
    /// <summary>
    /// OPTIMAL RETURN (a drone ship): the booster flies no boostback and lands wherever its coast from separation comes down, so it needs only the landing burn. <see cref="SiteCci"/> then only gives the surface radius the coast ends at; the landing point is where that coast lands, worked out from the plan (BoosterReturn), not set beforehand.
    /// </summary>
    public bool OptimalReturn { get; init; }
    /// <summary>The booster's engines' exhaust velocity, m/s.</summary>
    public required double ExhaustVelocity { get; init; }
    /// <summary>
    /// The booster's engines' exhaust velocity at sea level, m/s, for the landing burn; NaN takes <see cref="ExhaustVelocity"/>.
    ///
    /// THE LANDING BURN IS NEAR THE GROUND, where back pressure takes a large share of a vacuum-tuned engine's exhaust velocity: a heavy's core read 436 s in vacuum and flew its landing at 342 s, 22 % less burn per kilogram than the reserve was sized for, and came down short of propellant (2026-11-17). The boostback stays on <see cref="ExhaustVelocity"/>, high up.
    /// </summary>
    public double LandingExhaustVelocity { get; init; } = double.NaN;

    /// <summary>The exhaust velocity the landing burn is priced at: <see cref="LandingExhaustVelocity"/>, or the vacuum one where that is not known.</summary>
    public double LandingVe => LandingExhaustVelocity > 0.0 && double.IsFinite(LandingExhaustVelocity) ? LandingExhaustVelocity : ExhaustVelocity;
    /// <summary>The landing site at lift-off, in the problem's inertial frame, m. Fixed to the ground: at separation it has turned with the body.</summary>
    public required double[] SiteCci { get; init; }
    /// <summary>Drag reference area, m^2, for <see cref="Drag"/> and <see cref="Lift"/>.</summary>
    public required double ReferenceArea { get; init; }
    /// <summary>The coast's drag coefficient, retrograde-first (DragCoastSystem.Table).</summary>
    public required AeroTable Drag { get; init; }
    /// <summary>Lift coefficient for the glide, or null for none.</summary>
    public AeroTable? Lift { get; init; }
    /// <summary>Angle of attack held through the coast, radians, retrograde-first (DragCoastSystem.Alpha): the boostback glide's nominal one.</summary>
    public double Alpha { get; init; }
    /// <summary>The air the booster coasts through.</summary>
    public required AtmosphereModel Air { get; init; }
    /// <summary>The landing burn, as a multiple of the booster's terminal velocity, where <see cref="LandingAccel"/> is not known; zero for a booster that does not land (an expendable one's fall).</summary>
    public double LandingFactor { get; init; } = 1.3;
    /// <summary>
    /// The thrust acceleration the landing burn is flown at, m/s^2: the lower of the booster's sea-level thrust over its mass and the landing guidance's acceleration limit (VehicleAutopilotState.LandingAccelLimitG). NaN prices the burn with <see cref="LandingFactor"/> instead.
    ///
    /// THE BURN IS HELD TO A LIMIT, so it is not short. Falling at terminal velocity v and braking at a, it fights gravity the whole way: it takes v / (a - g) and costs v a / (a - g). At the 3 g the landing flies to that is 1.5 v, which the flat 1.3 v undersized: a heavy's core lit at 97 m/s, burned 5.4 s and needed about 150 m/s (2026-11-17).
    /// </summary>
    public double LandingAccel { get; init; } = double.NaN;
    /// <summary>The landing burn over what braking from terminal velocity at <see cref="LandingAccel"/> costs: ignition timing, the plan's last metres, steering.</summary>
    public double LandingMargin { get; init; } = 1.1;
    /// <summary>
    /// A fixed allowance for small corrections, m/s, kept on top of the boostback and the landing burn and priced at the vacuum exhaust velocity like the boostback: a small boostback after a glide-only miss, the trim the plan's impulse leaves, a separation that is not quite the plan's. Zero for none.
    /// </summary>
    public double CorrectionDv { get; init; }
    /// <summary>The boostback, as a multiple of its impulse: a real burn is finite, flipped into and flown through gravity and drag, where the impulse is none of these.</summary>
    /// <summary>
    /// What a real boostback burn costs over the impulse, as a factor on its dv. Not small, and longer the burn the more it is: from boosters_controlled's planned separation (3.3 km/s at 241 km) the booster's own planner needed 5080 m/s for a 4383 m/s impulse, 16 % more, which at 1.15 left 0.2 t of the 0.32 t the landing needs (2026-11-05).
    /// </summary>
    public double BoostbackMargin { get; init; } = 1.25;
    /// <summary>
    /// How long the booster coasts from separation before its boostback burn, s: clearing the stage it left (30 m, about 5 s), the settling burn (0.2 s) and the flip (5 to 8 s). The impulse is priced from where that coast leaves it, not from the separation itself - at staging speed every second of it is kilometres further from the site.
    /// </summary>
    public double BurnDelayS { get; init; } = 15.0;

    /// <summary>The same booster, its reserve starting at <paramref name="initialReserveKg"/>.</summary>
    public AscentRecovery WithInitialReserve(double initialReserveKg) => new()
    {
        Stage = Stage, InitialReserveKg = initialReserveKg, BoosterDryKg = BoosterDryKg, BoostersOnStage = BoostersOnStage, FlowShare = FlowShare, OptimalReturn = OptimalReturn, ExhaustVelocity = ExhaustVelocity, LandingExhaustVelocity = LandingExhaustVelocity, LandingAccel = LandingAccel, LandingMargin = LandingMargin, CorrectionDv = CorrectionDv, BurnDelayS = BurnDelayS,
        SiteCci = SiteCci, ReferenceArea = ReferenceArea, Drag = Drag, Lift = Lift, Alpha = Alpha, Air = Air,
        LandingFactor = LandingFactor, BoostbackMargin = BoostbackMargin,
    };

    public string Validate(AscentProblem p)
    {
        if (Stage < 0 || Stage >= p.Stages.Length - 1)
            return "the recovered stage is not a lower stage";
        if (!(BoosterDryKg > 0.0) || !(ExhaustVelocity > 0.0) || !(ReferenceArea > 0.0) || !(InitialReserveKg >= 0.0))
            return "the recovered booster is not valid";
        if (SiteCci is not { Length: 3 } || !SiteCci.All(double.IsFinite))
            return "the landing site is not valid";
        if (!(FlowShare > 0.0 && FlowShare <= 1.0))
            return "the booster's flow share is not in (0, 1]";
        if (FlowShare < 1.0 && !p.Stages[Stage + 1].HasLiquid)
            return "a side booster's core has no liquid to burn on in the next stage";
        return "";
    }
}

/// <summary>One booster brought home, as a plan has it (AscentSolution.Recoveries): the stage it leaves at; the reserve that stage keeps, kg, shared by every booster leaving there; what this booster needs from the plan's separation, kg; that need's boostback impulse and landing burn, m/s; and when it separates, s from lift-off.</summary>
public readonly record struct AscentRecoveryResult(int Stage, double ReserveKg, double RequiredKg, double BoostbackDv, double LandingDv, double SeparationTime);

/// <summary>
/// What a booster separated at a given state and time needs to come home, and how that moves with the state: the ascent's recovery row (AscentSubproblem) is that need linearised.
///
/// THE BOOSTBACK IS SOLVED, NOT GUESSED. Gauss-Newton on the impact point: predict the coast from (r, v + dv) and its velocity Jacobian J (ImpactPredictor.VelocityJacobian), step by ImpactSteering.Correction's damped minimum-norm correction, capped and halved until it lands nearer, until the impact is within <see cref="MissToleranceM"/>. The in-flight estimate takes only the first step from dv = 0.
///
/// THEN THE SMALLEST. The impulses that land on the site are a one-parameter family - J has a null direction - and the smallest is wanted. Increments keep whatever part of the starting guess lies along that direction, so a warm start carried between SCvx iterations drifted along the family: on the 2stage_new stack one plan's booster "needed" a 5008 m/s boostback, 37 t, where the same separation needs 903 kg from a cold start. So once it lands, the impulse's part along the null direction is taken out and it is landed again, until that part is gone: the answer is the same from any start. Solving for the whole impulse in one go does the same near the site but not far from it, where the part it takes out makes the miss worse and a line search has nowhere to go.
///
/// THE GRADIENT IS A FINITE DIFFERENCE of that solve, warm-started from the nominal answer, in each of the seven inputs: position, velocity and the separation time, which turns the site with the body. Converged to a metre, the impulse is good to about 0.01 m/s, so steps of 200 m, 2 m/s and 2 s leave about 0.005 of noise in a slope.
/// </summary>
public sealed class BoostbackOracle
{
    /// <summary>The impact is on the site when it is within this, m.</summary>
    public const double MissToleranceM = 1.0;
    private const int MaxIterations = 40;
    /// <summary>How many times a step is halved before the solve gives up on it.</summary>
    private const int BacktrackSteps = 6;
    /// <summary>Passes of taking the impulse's part along the null direction out and landing again, and the part, m/s, small enough to stop at.</summary>
    private const int NullPasses = 6;
    private const double NullToleranceMs = 0.5;
    /// <summary>The most the impulse may change in one step, m/s. A site far off the coast's impact - 1500 km, on the first linearisation - asked for 76 km/s in one step, which leaves the atmosphere and never lands; capped, it walks there.</summary>
    private const double MaxStepMs = 1500.0;
    private const double StepPositionM = 200.0, StepVelocityMs = 2.0, StepTimeS = 2.0;

    private readonly AscentRecovery _r;
    private readonly DragCoastSystem _sys;
    private readonly ImpactOptions _opt;
    private readonly Dual[] _scratch = new Dual[ImpactPredictor.ScratchLength];
    private readonly double _omega;

    /// <summary>The landing burn, m/s: LandingFactor times the terminal velocity at sea level, engine first, of the booster at its dry mass.</summary>
    public double LandingDv { get; }
    /// <summary>The booster's terminal velocity at sea level, m/s.</summary>
    public double TerminalVelocity { get; }
    /// <summary>Why the last boostback solve failed, for the log: the coast's status, or where the iteration stopped.</summary>
    public string LastFailure { get; private set; } = "";

    public BoostbackOracle(AscentProblem p, AscentRecovery r) : this(p.Mu, p.BodyRadius, p.Omega, r) { }

    /// <summary>The same, from the body alone: gravitational parameter, m^3/s^2, mean radius, m, and spin rate, rad/s.</summary>
    public BoostbackOracle(double mu, double bodyRadius, double omega, AscentRecovery r)
    {
        _r = r;
        _omega = omega;
        double g = mu / (bodyRadius * bodyRadius);
        double cd = r.Drag.Cd(0.3, 0.0);
        // One booster's share: a pair's drag area is each one's, flying home alone.
        double eachDryKg = r.BoosterDryKg / Math.Max(r.BoostersOnStage, 1);
        TerminalVelocity = cd > 0.0 && r.Air.SeaLevelDensity > 0.0
            ? Math.Sqrt(2.0 * eachDryKg * g / (r.Air.SeaLevelDensity * cd * r.ReferenceArea))
            : 0.0;
        LandingDv = LandingBurn(r, TerminalVelocity, g);
        // Through the coast the booster carries its landing propellant.
        double coastMass = eachDryKg * Math.Exp(LandingDv / r.LandingVe);
        _sys = new DragCoastSystem
        {
            Mu = mu,
            OmegaZ = omega,
            MeanRadius = bodyRadius,
            AreaOverMass = r.ReferenceArea / coastMass,
            Alpha = r.Alpha,
            Table = r.Drag,
            LiftTable = r.Lift,
            Atmosphere = r.Air,
        };
        double siteRadius = Math.Sqrt(r.SiteCci[0] * r.SiteCci[0] + r.SiteCci[1] * r.SiteCci[1] + r.SiteCci[2] * r.SiteCci[2]);
        _opt = ImpactOptions.Default(siteRadius);
        _opt.MaxTime = 1800.0;
        _opt.PathStride = 0;
    }

    /// <summary>
    /// The landing burn, m/s: braking from terminal velocity <paramref name="terminal"/> at the thrust acceleration it is flown at, against gravity <paramref name="g"/>, with its margin (AscentRecovery.LandingAccel); or the flat factor on the terminal velocity where that acceleration is not known. Zero for a booster that does not land.
    /// </summary>
    public static double LandingBurn(AscentRecovery r, double terminal, double g)
    {
        if (!(r.LandingFactor > 0.0))
            return 0.0;
        if (!(r.LandingAccel > 0.0) || !double.IsFinite(r.LandingAccel))
            return r.LandingFactor * terminal;
        // A booster that can barely outweigh gravity cannot land at all; held to 1.2 g here so the price stays finite and large rather than blowing up.
        double a = Math.Max(r.LandingAccel, 1.2 * g);
        return r.LandingMargin * terminal * a / (a - g);
    }

    /// <summary>The propellant the booster needs for a boostback of <paramref name="boostbackDv"/>, kg: the boostback high up at the vacuum exhaust velocity, then the landing burn at the sea-level one (AscentRecovery.LandingExhaustVelocity).</summary>
    public double RequiredKg(double boostbackDv)
        => _r.BoosterDryKg * (Math.Exp((_r.BoostbackMargin * boostbackDv + _r.CorrectionDv) / _r.ExhaustVelocity + LandingDv / _r.LandingVe) - 1.0);

    /// <summary>d(RequiredKg)/d(boostback dv), kg per m/s.</summary>
    public double RequiredKgSlope(double boostbackDv)
        => _r.BoosterDryKg * _r.BoostbackMargin / _r.ExhaustVelocity
           * Math.Exp((_r.BoostbackMargin * boostbackDv + _r.CorrectionDv) / _r.ExhaustVelocity + LandingDv / _r.LandingVe);

    /// <summary>
    /// The smallest impulse at separation, state (<paramref name="r"/>, <paramref name="v"/>) in SI at <paramref name="tSep"/> s after lift-off, that lands the booster on the site; <paramref name="dv"/> carries the starting guess in and the impulse out. False if the coast never lands or the correction cannot be solved.
    /// </summary>
    public bool Boostback(ReadOnlySpan<double> r, ReadOnlySpan<double> v, double tSep, double[] dv, out double dvMag, out double missM)
    {
        dvMag = double.NaN;
        double[] rr = r.ToArray(), vv = v.ToArray();
        // The coast to the burn (BurnDelayS): engine off, on the same drag the coast after it flies. The burn is then at tSep + delay, and the site has turned that much further.
        double delay = Math.Max(_r.BurnDelayS, 0.0);
        if (delay > 0.0)
        {
            Span<Dual> s0 = stackalloc Dual[ImpactPredictor.N];
            Span<Dual> s1 = stackalloc Dual[ImpactPredictor.N];
            for (int i = 0; i < 3; i++)
            {
                s0[i] = rr[i];
                s0[3 + i] = vv[i];
            }
            Rk4.Integrate(in _sys, 0.0, s0, delay, 1.0, s1, _scratch.AsSpan(0, Rk4.IntegrateScratch(ImpactPredictor.N)));
            for (int i = 0; i < 3; i++)
            {
                rr[i] = s1[i].V;
                vv[i] = s1[3 + i].V;
            }
        }
        double a = _omega * (tSep + delay), c = Math.Cos(a), s = Math.Sin(a);
        double[] site0 = _r.SiteCci;
        double sx = c * site0[0] - s * site0[1], sy = s * site0[0] + c * site0[1], sz = site0[2];
        var dG = new double[9];
        var miss = new double[3];
        var nrm = new double[3];

        // One coast from (r, v + impulse): its miss, the impact's surface normal and the velocity Jacobian.
        bool Predict(double[] imp, double[] g, double[] m, double[] n, out double mm)
        {
            Span<double> x0 = stackalloc double[ImpactPredictor.N];
            x0[0] = rr[0]; x0[1] = rr[1]; x0[2] = rr[2];
            x0[3] = vv[0] + imp[0]; x0[4] = vv[1] + imp[1]; x0[5] = vv[2] + imp[2];
            ImpactPrediction pred = ImpactPredictor.VelocityJacobian(in _sys, x0, in _opt, _scratch, g, default, default);
            mm = double.PositiveInfinity;
            if (!pred.Hit)
            {
                LastFailure = $"the coast after a {Math.Sqrt(imp[0] * imp[0] + imp[1] * imp[1] + imp[2] * imp[2]):F0} m/s impulse ends {pred.Status}";
                return false;
            }
            double fx = pred.Fx.V, fy = pred.Fy.V, fz = pred.Fz.V;
            m[0] = fx - sx; m[1] = fy - sy; m[2] = fz - sz;
            mm = Math.Sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
            double fl = Math.Sqrt(fx * fx + fy * fy + fz * fz);
            n[0] = fx / fl; n[1] = fy / fl; n[2] = fz / fl;
            return true;
        }

        // LAND IT: damped Gauss-Newton increments, each capped (MaxStepMs) and halved until it lands nearer the site (BacktrackSteps) - far off, the impact point bends too much for a full step.
        var cand = new double[3];
        var cG = new double[9];
        var cMiss = new double[3];
        var cNrm = new double[3];
        var step = new double[3];
        int iterations = 0;
        bool Land(out double mm)
        {
            if (!Predict(dv, dG, miss, nrm, out mm))
                return false;
            while (mm > MissToleranceM)
            {
                if (++iterations > MaxIterations)
                {
                    LastFailure = $"still {mm:F0} m off after {MaxIterations} iterations";
                    return false;
                }
                if (!ImpactSteering.Correction(dG, miss, nrm, ImpactSteering.DefaultLambda, step, default))
                {
                    LastFailure = "the correction could not be solved";
                    return false;
                }
                double sn = Math.Sqrt(step[0] * step[0] + step[1] * step[1] + step[2] * step[2]);
                double k = sn > MaxStepMs ? MaxStepMs / sn : 1.0;
                bool better = false;
                for (int b = 0; b <= BacktrackSteps && !better; b++, k *= 0.5)
                {
                    cand[0] = dv[0] + k * step[0]; cand[1] = dv[1] + k * step[1]; cand[2] = dv[2] + k * step[2];
                    better = Predict(cand, cG, cMiss, cNrm, out double cm) && cm < mm;
                    if (better)
                    {
                        mm = cm;
                        Array.Copy(cand, dv, 3);
                        Array.Copy(cG, dG, 9);
                        Array.Copy(cMiss, miss, 3);
                        Array.Copy(cNrm, nrm, 3);
                    }
                }
                if (!better)
                {
                    LastFailure = $"no step nearer the site from {mm:F0} m off";
                    return false;
                }
            }
            return true;
        }

        if (!Land(out missM))
            return false;
        // MAKE IT THE SMALLEST: the impulses that land are a curve through this one along J's null direction; take out the impulse's part along it and land again, until that part is gone.
        for (int pass = 0; pass < NullPasses; pass++)
        {
            if (!NullDirection(dG, out double nx, out double ny, out double nz))
                break;
            double along = dv[0] * nx + dv[1] * ny + dv[2] * nz;
            if (Math.Abs(along) < NullToleranceMs)
                break;
            double[] keep = (double[])dv.Clone();
            dv[0] -= along * nx; dv[1] -= along * ny; dv[2] -= along * nz;
            if (!Land(out double landed))
            {
                // A shorter impulse that no longer lands is not an answer; the last one that did is.
                Array.Copy(keep, dv, 3);
                Land(out landed);
                missM = landed;
                break;
            }
            missM = landed;
        }
        dvMag = Math.Sqrt(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]);
        return missM <= MissToleranceM;
    }

    /// <summary>
    /// The booster's flight from separation to the ground, with <paramref name="impulse"/> (m/s, or null for none) fired after the coast to the burn (BurnDelayS), as the boostback flies it. <paramref name="ground"/> gets the path as points fixed to the ground, x y z per point in the problem's frame at lift-off, from separation to impact; <paramref name="times"/> each point's time from lift-off. <paramref name="impact"/> is where it lands, in the same frame. False, with the reason in <see cref="LastFailure"/>, if the coast never reaches the ground.
    ///
    /// FIXED TO THE GROUND, not inertial: each point is turned back by the angle the body has turned since lift-off, so the path ends on the ground point it lands on, which is what a landing site, and a ship placed there, are.
    /// </summary>
    public bool Path(ReadOnlySpan<double> r, ReadOnlySpan<double> v, double tSep, double[]? impulse, List<double> ground, List<double> times, out double[] impact)
    {
        impact = [];
        ground.Clear();
        times.Clear();
        void Add(double x, double y, double z, double t)
        {
            double a = -_omega * t, c = Math.Cos(a), s = Math.Sin(a);
            ground.Add(c * x - s * y);
            ground.Add(s * x + c * y);
            ground.Add(z);
            times.Add(t);
        }

        double[] rr = r.ToArray(), vv = v.ToArray();
        Add(rr[0], rr[1], rr[2], tSep);
        double delay = Math.Max(_r.BurnDelayS, 0.0);
        if (delay > 0.0)
        {
            Span<Dual> s0 = stackalloc Dual[ImpactPredictor.N];
            Span<Dual> s1 = stackalloc Dual[ImpactPredictor.N];
            for (int i = 0; i < 3; i++)
            {
                s0[i] = rr[i];
                s0[3 + i] = vv[i];
            }
            Rk4.Integrate(in _sys, 0.0, s0, delay, 1.0, s1, _scratch.AsSpan(0, Rk4.IntegrateScratch(ImpactPredictor.N)));
            for (int i = 0; i < 3; i++)
            {
                rr[i] = s1[i].V;
                vv[i] = s1[3 + i].V;
            }
        }
        double tBurn = tSep + delay;

        Span<Dual> x0 = stackalloc Dual[ImpactPredictor.N];
        for (int i = 0; i < 3; i++)
        {
            x0[i] = rr[i];
            x0[3 + i] = vv[i] + (impulse != null ? impulse[i] : 0.0);
        }
        ImpactOptions opt = _opt;
        opt.PathStride = 4;
        var path = new double[ImpactPredictor.PathStrideDoubles * 2000];
        ImpactPrediction pred = ImpactPredictor.Predict(in _sys, x0, in opt, _scratch, path);
        if (!pred.Hit)
        {
            LastFailure = $"the coast from separation ends {pred.Status}";
            return false;
        }
        for (int k = 0; k < pred.PathPoints; k++)
        {
            int o = k * ImpactPredictor.PathStrideDoubles;
            Add(path[o], path[o + 1], path[o + 2], tBurn + path[o + 3]);
        }
        double tHit = tBurn + pred.TimeOfFlight.V;
        Add(pred.Rx.V, pred.Ry.V, pred.Rz.V, tHit);
        int n = ground.Count;
        impact = [ground[n - 3], ground[n - 2], ground[n - 1]];
        return true;
    }

    /// <summary>The unit direction J maps to nothing: the cross product of the two of its rows that are furthest from parallel. J's rows are the impact's three ground coordinates, held to the surface, so they span a plane.</summary>
    private static bool NullDirection(double[] g, out double nx, out double ny, out double nz)
    {
        nx = ny = nz = 0.0;
        double best = 0.0;
        for (int i = 0; i < 3; i++)
            for (int j = i + 1; j < 3; j++)
            {
                double cx = g[i * 3 + 1] * g[j * 3 + 2] - g[i * 3 + 2] * g[j * 3 + 1];
                double cy = g[i * 3 + 2] * g[j * 3] - g[i * 3] * g[j * 3 + 2];
                double cz = g[i * 3] * g[j * 3 + 1] - g[i * 3 + 1] * g[j * 3];
                double n = Math.Sqrt(cx * cx + cy * cy + cz * cz);
                if (n > best)
                {
                    best = n;
                    nx = cx / n; ny = cy / n; nz = cz / n;
                }
            }
        return best > 0.0;
    }

    /// <summary>A booster's need at one separation, and its slopes.</summary>
    public readonly record struct Need(bool Ok, double BoostbackDv, double RequiredKg, double[] Impulse, double[] GradPosition, double[] GradVelocity, double GradTime);

    /// <summary>
    /// The need at separation state (<paramref name="r"/>, <paramref name="v"/>), SI, <paramref name="tSep"/> s after lift-off, and with <paramref name="gradient"/> its slope in kg per m, per m/s and per s. <paramref name="guess"/> warm-starts the impulse: the last answer, from a nearby state.
    /// </summary>
    public Need Evaluate(ReadOnlySpan<double> r, ReadOnlySpan<double> v, double tSep, double[]? guess, bool gradient)
    {
        // An optimal return flies no boostback: its need is the landing burn wherever it separates, so it has no slope.
        if (_r.OptimalReturn)
            return new Need(true, 0.0, RequiredKg(0.0), new double[3], new double[3], new double[3], 0.0);

        double[] dv = guess != null ? (double[])guess.Clone() : new double[3];
        if (!Boostback(r, v, tSep, dv, out double mag, out _))
        {
            // A warm start that has wandered off a solution is retried cold.
            dv = new double[3];
            if (!Boostback(r, v, tSep, dv, out mag, out _))
                return new Need(false, double.NaN, double.NaN, [], [], [], double.NaN);
        }
        double need = RequiredKg(mag);
        if (!gradient)
            return new Need(true, mag, need, dv, [], [], double.NaN);

        double slope = RequiredKgSlope(mag);
        var gr = new double[3];
        var gv = new double[3];
        double[] r0 = r.ToArray(), v0 = v.ToArray();
        double Perturbed(int which, double h, double dt)
        {
            var rr = (double[])r0.Clone();
            var vv = (double[])v0.Clone();
            if (which >= 0 && which < 3) rr[which] += h;
            else if (which >= 3) vv[which - 3] += h;
            double[] w = (double[])dv.Clone();
            return Boostback(rr, vv, tSep + dt, w, out double m, out _) ? m : double.NaN;
        }
        for (int i = 0; i < 3; i++)
        {
            double up = Perturbed(i, StepPositionM, 0.0);
            gr[i] = double.IsFinite(up) ? slope * (up - mag) / StepPositionM : 0.0;
            up = Perturbed(3 + i, StepVelocityMs, 0.0);
            gv[i] = double.IsFinite(up) ? slope * (up - mag) / StepVelocityMs : 0.0;
        }
        double later = Perturbed(-1, 0.0, StepTimeS);
        double gt = double.IsFinite(later) ? slope * (later - mag) / StepTimeS : 0.0;
        return new Need(true, mag, need, dv, gr, gv, gt);
    }
}
