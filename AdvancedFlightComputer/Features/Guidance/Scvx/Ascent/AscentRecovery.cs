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
    /// <summary>The booster's dry mass, kg: what leaves at separation besides the reserve.</summary>
    public required double BoosterDryKg { get; init; }
    /// <summary>The booster's engines' exhaust velocity, m/s.</summary>
    public required double ExhaustVelocity { get; init; }
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
    public required ExponentialAtmosphere Air { get; init; }
    /// <summary>The landing burn, as a multiple of the booster's terminal velocity.</summary>
    public double LandingFactor { get; init; } = 1.3;
    /// <summary>The boostback, as a multiple of its impulse: a real burn is finite, flipped into and flown through gravity and drag, where the impulse is none of these.</summary>
    public double BoostbackMargin { get; init; } = 1.1;

    /// <summary>The same booster, its reserve starting at <paramref name="initialReserveKg"/>.</summary>
    public AscentRecovery WithInitialReserve(double initialReserveKg) => new()
    {
        Stage = Stage, InitialReserveKg = initialReserveKg, BoosterDryKg = BoosterDryKg, ExhaustVelocity = ExhaustVelocity,
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
        return "";
    }
}

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

    public BoostbackOracle(AscentProblem p, AscentRecovery r)
    {
        _r = r;
        _omega = p.Omega;
        double g = p.Mu / (p.BodyRadius * p.BodyRadius);
        double cd = r.Drag.Cd(0.3, 0.0);
        TerminalVelocity = cd > 0.0 && r.Air.SeaLevelDensity > 0.0
            ? Math.Sqrt(2.0 * r.BoosterDryKg * g / (r.Air.SeaLevelDensity * cd * r.ReferenceArea))
            : 0.0;
        LandingDv = r.LandingFactor * TerminalVelocity;
        // Through the coast the booster carries its landing propellant.
        double coastMass = r.BoosterDryKg * Math.Exp(LandingDv / r.ExhaustVelocity);
        _sys = new DragCoastSystem
        {
            Mu = p.Mu,
            OmegaZ = p.Omega,
            MeanRadius = p.BodyRadius,
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

    /// <summary>The propellant the booster needs for a boostback of <paramref name="boostbackDv"/>, kg.</summary>
    public double RequiredKg(double boostbackDv)
        => _r.BoosterDryKg * (Math.Exp((_r.BoostbackMargin * boostbackDv + LandingDv) / _r.ExhaustVelocity) - 1.0);

    /// <summary>d(RequiredKg)/d(boostback dv), kg per m/s.</summary>
    public double RequiredKgSlope(double boostbackDv)
        => _r.BoosterDryKg * _r.BoostbackMargin / _r.ExhaustVelocity
           * Math.Exp((_r.BoostbackMargin * boostbackDv + LandingDv) / _r.ExhaustVelocity);

    /// <summary>
    /// The smallest impulse at separation, state (<paramref name="r"/>, <paramref name="v"/>) in SI at <paramref name="tSep"/> s after lift-off, that lands the booster on the site; <paramref name="dv"/> carries the starting guess in and the impulse out. False if the coast never lands or the correction cannot be solved.
    /// </summary>
    public bool Boostback(ReadOnlySpan<double> r, ReadOnlySpan<double> v, double tSep, double[] dv, out double dvMag, out double missM)
    {
        dvMag = double.NaN;
        double a = _omega * tSep, c = Math.Cos(a), s = Math.Sin(a);
        double[] site0 = _r.SiteCci;
        double sx = c * site0[0] - s * site0[1], sy = s * site0[0] + c * site0[1], sz = site0[2];
        double[] rr = r.ToArray(), vv = v.ToArray();
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
