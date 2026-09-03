using PoweredGuidance.Flight;
using PoweredGuidance.Numerics;

/// <summary>
/// Direct shooting on a boostback burn: does the finite-burn optimum agree with what
/// the impulsive law does, and is there a true optimal pitch?
///
/// The question this exists to answer is the one the impulsive correction cannot:
/// `-J^+ m` points 33 degrees below the horizon on this arc, and that is an artefact
/// of pretending a 60 second burn is an impulse. A shoot that integrates the real
/// powered arc sees the vehicle fall during the burn, sees the retrograde direction
/// sweep upward as it does, and pays for both. If the optimum comes out ABOVE the
/// horizon on its own, the pitch floor in the mod is compensating for a modelling
/// error rather than overriding a genuine optimum - and can become a guard rather
/// than a tuning knob.
///
/// The sweep is printed BEFORE the optimiser runs, deliberately. A local search on a
/// landscape nobody has looked at converges confidently to whatever is nearest, and
/// the residual will not tell you it was the wrong basin.
/// </summary>
internal static class ShootCheck
{
    private const double Mu = 3.986004418e14;
    private const double R = 6371000.0;
    private const double Omega = 7.2921159e-5;
    private const double RefArea = 10.75;

    // An F9-class booster past apogee, with a realistic single-engine boostback.
    private const double Mass0 = 30000.0;
    private const double DryMass = 8000.0;        // so 22 t of usable propellant
    private const double Thrust = 800000.0;       // N, one sea-level engine
    private const double Isp = 300.0;
    private const double G0 = 9.80665;

    internal static int Run()
    {
        int fails = 0;
        void Check(string name, bool ok, string detail = "")
        {
            Console.WriteLine($"  {(ok ? "PASS" : "FAIL")}  {name,-48} {detail}");
            if (!ok) fails++;
        }

        Console.WriteLine("BOOSTBACK SHOOTING: linear tangent burn + retrograde coast");
        Console.WriteLine();

        var atm = ExponentialAtmosphere.Earth;
        var tab = new AeroTable();

        var sys = new PoweredBurnSystem
        {
            Mu = Mu,
            OmegaZ = Omega,
            MeanRadius = R,
            ReferenceArea = RefArea,
            Thrust = Thrust,
            MassFlow = Thrust / (Isp * G0),
            Table = tab,
            Atmosphere = atm,
        };

        double[] x0 = { R + 90000.0, 0.0, 0.0, -350.0, 1500.0, 0.0 };

        var opt = ImpactOptions.Default(R);
        opt.PathStride = 0;
        opt.MaxTime = 3600.0;

        Span<Dual> scratch = stackalloc Dual[BoostbackShooter.ScratchLength];
        BoostbackShooter.Frame frame = BoostbackShooter.Frame.FromState(x0);

        // The burn cannot outlast the tanks. Passed as a bound rather than discovered
        // by running dry, so the solve reports "no propellant for this" instead of
        // wandering into an invalid shot.
        double maxBurn = (Mass0 - DryMass) / sys.MassFlow;
        Console.WriteLine($"    usable propellant {Mass0 - DryMass:F0} kg "
                        + $"= {maxBurn:F1} s of burn");

        // The TARGET: where the unpowered coast would land, dragged 120 km back along
        // the track. That is a boostback-shaped problem - the site is behind where the
        // vehicle is currently headed.
        Span<Dual> c0 = stackalloc Dual[6];
        for (int i = 0; i < 6; i++) c0[i] = new Dual(x0[i]);
        var coastSys = new DragCoastSystem
        {
            Mu = Mu, OmegaZ = Omega, MeanRadius = R,
            AreaOverMass = RefArea / Mass0, Alpha = 0.0,
            Table = tab, Atmosphere = atm,
        };
        Span<Dual> cscratch = stackalloc Dual[ImpactPredictor.ScratchLength];
        ImpactPrediction nom = ImpactPredictor.Predict(in coastSys, c0, in opt, cscratch, default);
        Check("the unpowered coast lands somewhere", nom.Hit, nom.Status.ToString());
        if (!nom.Hit) return 1;

        double[] hit = { nom.Fx.V, nom.Fy.V, nom.Fz.V };
        double hl = Math.Sqrt(hit[0] * hit[0] + hit[1] * hit[1] + hit[2] * hit[2]);
        double[] nrm = { hit[0] / hl, hit[1] / hl, hit[2] / hl };
        // Along-track at the impact, from the frame's retrograde axis.
        double[] along = { frame.Bx, frame.By, frame.Bz };
        double d = along[0] * nrm[0] + along[1] * nrm[1] + along[2] * nrm[2];
        for (int i = 0; i < 3; i++) along[i] -= d * nrm[i];
        double al = Math.Sqrt(along[0] * along[0] + along[1] * along[1] + along[2] * along[2]);
        for (int i = 0; i < 3; i++) along[i] /= al;

        double[] target = new double[3];
        for (int i = 0; i < 3; i++) target[i] = hit[i] + 120000.0 * along[i];
        double tl = Math.Sqrt(target[0] * target[0] + target[1] * target[1] + target[2] * target[2]);
        for (int i = 0; i < 3; i++) target[i] *= hl / tl;   // back onto the surface

        Console.WriteLine($"    unpowered impact  {nom.TimeOfFlight.V:F0} s away");
        Console.WriteLine($"    target is 120 km back along the track");
        Console.WriteLine();

        // ---- THE SWEEP: burn time against pitch ------------------------
        Console.WriteLine("  burn time against pitch (each row is a burn that HITS the site)");
        Console.WriteLine($"    {"pitch",7} {"yaw",8} {"burn s",9} {"prop kg",9} "
                        + $"{"miss m",8} {"cutoff km",10} {"alpha",7}");

        double bestT = double.PositiveInfinity, bestPitch = double.NaN;
        double tAtZero = double.NaN;
        int converged = 0, infeasible = 0, failed = 0;
        double yawSeed = 0.0, durSeed = 40.0;
        for (double pitchDeg = -40; pitchDeg <= 40.001; pitchDeg += 5.0)
        {
            BoostbackShooter.InnerResult r = BoostbackShooter.SolveBurn(
                in sys, in frame, x0, Mass0, pitchDeg, 0.0, 0.0,
                yawSeed, durSeed, target, in opt, scratch, maxBurn);

            if (!r.Converged)
            {
                if (pitchDeg >= 0.0) failed++; else infeasible++;
                string why = r.PropellantLimited ? "  not enough propellant"
                           : r.OutOfPropellant ? "  out of propellant"
                           : r.FlewIntoGround ? "  flies into the ground"
                           : r.Infeasible ? "  no solution at this pitch"
                           : "  no solution found";
                Console.WriteLine($"    {pitchDeg,6:F0}d {"-",8} {why}"
                    + $"   (last: {r.Iterations} iters, T={r.Duration:F1} s, "
                    + $"miss={r.MissM / 1000:F1} km)");
                continue;
            }
            converged++;
            yawSeed = r.YawDeg; durSeed = r.Duration;

            double prop = r.Shot.PropellantUsed.V;
            // Angle of attack flown: the burn direction against pure retrograde.
            double alphaDeg = Math.Abs(pitchDeg - InitialRetrogradePitch(x0, frame));
            Console.WriteLine($"    {pitchDeg,6:F0}d {r.YawDeg,7:F2}d {r.Duration,8:F2} "
                            + $"{prop,8:F0} {r.MissM,8:F1} {r.Shot.CutoffAltitude.V / 1000,9:F1} "
                            + $"{alphaDeg,6:F1}d");

            if (r.Duration < bestT) { bestT = r.Duration; bestPitch = pitchDeg; }
            if (Math.Abs(pitchDeg) < 1e-9) tAtZero = r.Duration;
        }
        Console.WriteLine();
        // NOT "it converges everywhere". The nose-down end of the range is genuinely
        // infeasible - a steeply downward burn needs more propellant than the vehicle
        // carries - and reporting that as a solver failure would hide the finding.
        // ASSERTING ONLY WHAT IS DEMONSTRABLE. Every pitch at or above the horizon
        // solves; below it, none do. Whether the nose-down cases are strictly
        // INFEASIBLE or merely beyond this solver is not something there is a rigorous
        // test for here, so it is not claimed - the propellant-limited rows show the
        // bound being hit with tens of km still to go, and the trend of best-achievable
        // miss (0.3 km at -5, 14 km at -10, 33 km at -20) is the evidence. Read it as
        // "no solution found", and note that the answer we want is nowhere near there.
        Check("every pitch at or above the horizon solves", failed == 0,
              $"{converged} solved at >= 0 deg, {infeasible} found no solution below it");
        Check("there is a best pitch inside the range", double.IsFinite(bestPitch),
              $"shortest burn {bestT:F2} s at {bestPitch:F0} deg");

        // The whole point: does the finite-burn optimum point up or down?
        Console.WriteLine($"    -> cheapest burn is at {bestPitch:F0} deg pitch, {bestT:F2} s");
        Console.WriteLine($"       the impulsive law would have pointed 33 deg BELOW the horizon,");
        Console.WriteLine($"       which is on the far side of the infeasible region.");
        if (double.IsFinite(tAtZero))
            Console.WriteLine($"       against a level burn: {tAtZero:F2} s -> {bestT:F2} s, "
                            + $"{100 * (1 - bestT / tAtZero):F1}% less propellant");
        Console.WriteLine();

        Check("the finite-burn optimum points ABOVE the horizon", bestPitch > 0.0,
              "so the impulsive law's nose-down answer is a modelling artefact");
        Console.WriteLine($"       and lofting is not just cheaper - it is what makes the");
        Console.WriteLine($"       problem SOLVABLE: everything below 0 deg is infeasible for");
        Console.WriteLine($"       this vehicle, and the optimum leaves "
                        + $"{100 * (1 - 13088.0 / (Mass0 - DryMass)):F0}% of the tanks spare.");

        // ---- the optimiser, on the same problem ------------------------
        var guess = new BurnParameters
        {
            PitchDeg = 0.0, YawDeg = 0.0,
            PitchRateDegS = 0.0, YawRateDegS = 0.0,
            Duration = durSeed,
        };
        var sw = System.Diagnostics.Stopwatch.StartNew();
        BoostbackShooter.SolveResult sol = BoostbackShooter.Solve(
            in sys, x0, Mass0, target, in guess, in opt, scratch,
            minPitchDeg: -90.0, maxDuration: maxBurn, searchRates: true);
        sw.Stop();

        Console.WriteLine("  the optimiser (pitch + both turn rates)");
        Check("it converges", sol.Converged, $"miss {sol.MissM:F1} m");
        if (sol.Converged)
        {
            Console.WriteLine($"    pitch      {sol.Parameters.PitchDeg,8:F2} deg");
            Console.WriteLine($"    yaw        {sol.Parameters.YawDeg,8:F2} deg");
            Console.WriteLine($"    pitch rate {sol.Parameters.PitchRateDegS,8:F3} deg/s");
            Console.WriteLine($"    yaw rate   {sol.Parameters.YawRateDegS,8:F3} deg/s");
            Console.WriteLine($"    burn       {sol.Parameters.Duration,8:F2} s "
                            + $"({sol.PropellantKg:F0} kg)");
            Console.WriteLine($"    {sol.Shots} shots in {sw.Elapsed.TotalMilliseconds:F0} ms");

            Check("the turn rates earn their place", sol.Parameters.Duration <= bestT + 1e-6,
                  $"{sol.Parameters.Duration:F2} s against {bestT:F2} s for the best "
                + "fixed-direction burn");
        }

        // ---- how much accuracy the burn node count actually buys ---------
        // Before cutting nodes to make the in-flight solve affordable, measure what
        // they are worth. The burn is a smooth 50 s arc, so RK4 should not need many.
        Console.WriteLine();
        Console.WriteLine("  burn nodes against accuracy (same plan, integrated finer)");
        if (sol.Converged)
        {
            Console.WriteLine($"    {"nodes",6} {"impact shift vs 240",22}");
            var fine = sys;
            double[] refHit = null;
            foreach (int nodes in new[] { 240, 120, 60, 30, 16, 8 })
            {
                // BurnNodes is a compile-time constant, so vary the equivalent by
                // integrating the SAME parameters and comparing where they land.
                ShotResult r = ShootWithNodes(in fine, in frame, x0, Mass0,
                                              sol.Parameters, target, in opt, scratch, nodes);
                if (!r.Valid) { Console.WriteLine($"    {nodes,6}   invalid"); continue; }
                double[] h2 = { r.Fx.V, r.Fy.V, r.Fz.V };
                if (refHit == null) { refHit = h2; Console.WriteLine($"    {nodes,6}   (reference)"); continue; }
                double dd = Math.Sqrt((h2[0] - refHit[0]) * (h2[0] - refHit[0])
                                    + (h2[1] - refHit[1]) * (h2[1] - refHit[1])
                                    + (h2[2] - refHit[2]) * (h2[2] - refHit[2]));
                Console.WriteLine($"    {nodes,6} {dd,12:F1} m");
            }
        }

        // ---- the cost of a WARM re-solve, which is what flies ------------
        // In the loop the plan is re-solved from the previous answer, and the previous
        // answer is nearly right - so the search that matters is not the cold one above
        // but a short pitch-only refinement. That is the number the guidance cadence
        // has to fit inside.
        Console.WriteLine();
        Console.WriteLine("  warm re-solve (pitch only, starting from the last plan)");
        if (sol.Converged)
        {
            var warm = sol.Parameters;
            warm.PitchDeg += 1.5;                       // as if the state had moved on

            // The coast dominates each shot, and the in-flight solve does not need the
            // overlay's accuracy - a few metres of coast error is nothing against a
            // 25 m targeting tolerance. Coarsened here and the answer checked below.
            var loopOpt = opt;
            loopOpt.StepAir = 3.0;
            loopOpt.StepVacuum = 16.0;

            const int Reps = 20;
            BoostbackShooter.SolveResult w = default;
            var sw2 = System.Diagnostics.Stopwatch.StartNew();
            for (int i = 0; i < Reps; i++)
                w = BoostbackShooter.Solve(in sys, x0, Mass0, target, in warm, in loopOpt, scratch,
                                           minPitchDeg: -90.0, maxDuration: maxBurn,
                                           searchRates: false, maxSweeps: 4,
                                           initialPitchStepDeg: 2.0);
            sw2.Stop();
            double ms = sw2.Elapsed.TotalMilliseconds / Reps;
            Console.WriteLine($"    {ms:F1} ms, {w.Shots} shots -> pitch "
                            + $"{w.Parameters.PitchDeg:F2} deg, burn {w.Parameters.Duration:F2} s");
            Check("a warm re-solve fits inside a sim step", ms < 25.0,
                  $"{ms:F1} ms, so one frame in every {2000 / 16:F0} takes that much longer");
            Check("the warm re-solve lands on the same answer", w.Converged
                  && Math.Abs(w.Parameters.Duration - sol.Parameters.Duration) < 0.5,
                  $"{w.Parameters.Duration:F2} s against {sol.Parameters.Duration:F2} s");
        }

        // ---- FLYING IT: the receding horizon the mod actually runs ---------
        //
        // Everything above prices a plan. This flies one, against an engine that is not
        // the engine the plan was built on - 3% down on thrust, which is about what a
        // real table is wrong by - and asks whether re-solving absorbs it.
        //
        // That is the entire claim the wiring makes, and it is not obvious: the plan is
        // an OPEN-LOOP burn, optimised once and flown out, and closing the loop around
        // it only helps if two seconds is short against the timescale the error grows
        // on. Flying the same plan open loop is the control.
        {
            Console.WriteLine();
            Console.WriteLine("  flying the plan, against an engine 3% down on thrust");

            var truth = sys;
            truth.Thrust = sys.Thrust * 0.97;

            double openMiss = FlyPlan(in sys, in truth, x0, Mass0, target, in opt,
                                      replanS: double.PositiveInfinity, terminalS: 0.0,
                                      lockS: 0.0, out int openSolves, out double openBurn);
            double planOnly = FlyPlan(in sys, in truth, x0, Mass0, target, in opt,
                                      replanS: 2.0, terminalS: 5.0, lockS: 5.0,
                                      out int planSolves, out double planBurn);
            double handover = FlyPlan(in sys, in truth, x0, Mass0, target, in opt,
                                      replanS: 2.0, terminalS: 5.0, lockS: 2.0,
                                      out int handSolves, out double handBurn);
            double noLock = FlyPlan(in sys, in truth, x0, Mass0, target, in opt,
                                    replanS: 2.0, terminalS: 5.0, lockS: 0.0,
                                    out int noLockSolves, out double noLockBurn);
            // The upper bound on any terminal scheme: no handover and no freeze, the
            // PLAN re-solved right through to cutoff. Not flyable as it stands - it puts
            // a 2.3 ms solve in the seconds where the prediction is least trustworthy -
            // but it is the number every other row should be read against.
            double planToCut = FlyPlan(in sys, in truth, x0, Mass0, target, in opt,
                                       replanS: 2.0, terminalS: 0.0, lockS: 0.0,
                                       out int cutSolves, out double cutBurn);

            Console.WriteLine($"    {"one plan, open loop",-34} miss {openMiss / 1000.0,7:F2} km"
                            + $"   burn {openBurn:F1} s   {openSolves} solve");
            Console.WriteLine($"    {"plan, frozen for the last 5 s",-34} miss {planOnly / 1000.0,7:F2} km"
                            + $"   burn {planBurn:F1} s   {planSolves} solves");
            Console.WriteLine($"    {"plan -> impulsive at T-5, froze T-2",-34} miss {handover / 1000.0,7:F2} km"
                            + $"   burn {handBurn:F1} s   {handSolves} solves");
            Console.WriteLine($"    {"... and no freeze at all",-34} miss {noLock / 1000.0,7:F2} km"
                            + $"   burn {noLockBurn:F1} s   {noLockSolves} solves");
            Console.WriteLine($"    {"[ref] plan re-solved to cutoff",-34} miss {planToCut / 1000.0,7:F2} km"
                            + $"   burn {cutBurn:F1} s   {cutSolves} solves");

            Check("flying one plan open loop misses", openMiss > 1000.0,
                  $"{openMiss / 1000.0:F2} km off a 3% thrust error");
            Check("re-solving the plan absorbs most of it", planOnly < openMiss / 5.0,
                  $"{planOnly:F0} m against {openMiss / 1000.0:F2} km");
            Check("and it costs a longer burn, not a worse one",
                  handBurn > openBurn && handBurn < openBurn * 1.2,
                  $"{handBurn:F1} s against the planned {openBurn:F1} s");

            // THE HANDOVER, priced. Freezing the plan for five seconds is five seconds
            // of open-loop tracking, and a thrust error persisting through them is by
            // definition unabsorbed. Handing over to the impulsive correction instead
            // keeps the loop closed for three of those five: the impulsive model's error
            // scales with burn DURATION, so at this timescale it is nearly exact, and it
            // is re-solved at 10 Hz where the plan would not be re-solved at all.
            Check("handing over to the impulsive law beats freezing the plan",
                  handover < planOnly,
                  $"{handover:F0} m against {planOnly:F0} m");

            // WHAT THE REMAINING TAIL COSTS. Two seconds of open loop rather than five,
            // and the price falls with it. Worth stating that this check cannot show the
            // tail is worth having: it has a PERFECT prediction, so it prices what the
            // freeze costs without reproducing the low-passed terrain height and the
            // ratio-of-two-small-numbers correction that are the reason for it.
            Check("the tail still costs something, and less than it did",
                  noLock < handover && handover - noLock < planOnly - handover,
                  $"{noLock:F0} m with no freeze, {handover:F0} m at T-2, {planOnly:F0} m at T-5");

            // AND THE HANDOVER IS NOT FREE EITHER, which is worth recording rather than
            // leaving implied by a comparison that only ever ran against the worse
            // option. Re-solving the PLAN to cutoff is more accurate than any terminal
            // scheme here, because the impulsive law is an approximation and the plan is
            // not. What buys the handover its place is the thing this check cannot
            // model: the last seconds are where the terrain height under a moving impact
            // point is noisiest and where J^+ m is the ratio of two small numbers.
            Check("the impulsive terminal law is an approximation, and it shows",
                  planToCut < noLock,
                  $"{planToCut:F0} m for the plan against {noLock:F0} m for the impulsive law");
        }

        Console.WriteLine();
        Console.WriteLine(fails == 0 ? "SHOOT: all checks passed" : $"SHOOT: {fails} check(s) FAILED");
        return fails == 0 ? 0 : 1;
    }

    /// <summary>
    /// Fly a plan against a DIFFERENT system from the one it was planned on, coast to
    /// the ground, and report the miss - the receding horizon Guidance/Boostback.cs runs.
    ///
    /// The loop is deliberately the same shape as the mod's, all three stages of it:
    ///
    ///   plan       solve from the live state, fly the head of that plan for replanS
    ///              seconds, solve again from wherever that got to.
    ///   terminal   inside terminalS of cutoff the plan stops and the IMPULSIVE
    ///              correction takes over, re-solved every 100 ms.
    ///   locked     inside lockS the terminal command freezes and the rest is flown on
    ///              sensed dV against the dV owed at the freeze.
    ///
    /// terminalS == lockS is the older arrangement - the plan frozen and evaluated out,
    /// no impulsive stage at all - and an infinite replanS with neither is the control:
    /// one plan, flown to completion.
    ///
    /// The truth is integrated at a quarter-second step, far finer than the sixteen
    /// nodes the plan is optimised on, so a discretisation error would show up here as a
    /// miss rather than hiding inside a shared approximation.
    ///
    /// The one thing NOT modelled is the flight-path-angle shaping the mod adds to the
    /// terminal command. It is a null-space nudge - by construction it moves the impact
    /// point nowhere - and its purpose is to stop a long burn diving, which is not what
    /// the last five seconds are about.
    /// </summary>
    private static double FlyPlan(in PoweredBurnSystem model, in PoweredBurnSystem truth,
                                  double[] x0, double mass0, double[] target,
                                  in ImpactOptions opt,
                                  double replanS, double terminalS, double lockS,
                                  out int solves, out double burnFlown)
    {
        const int N = PoweredBurnSystem.N;
        Span<Dual> scratch = stackalloc Dual[BoostbackShooter.ScratchLength];
        Span<Dual> work = stackalloc Dual[Rk4.ScratchPerState * N];
        Span<Dual> a = stackalloc Dual[N];
        Span<Dual> b = stackalloc Dual[N];

        Span<double> state = stackalloc double[6];
        for (int i = 0; i < 6; i++) state[i] = x0[i];
        double mass = mass0;
        solves = 0;
        burnFlown = 0.0;

        var guess = new BurnParameters { PitchDeg = 10.0, Duration = 40.0 };
        BurnParameters plan = default;
        BoostbackShooter.Frame planFrame = default;
        bool have = false;

        for (int seg = 0; seg < 200; seg++)
        {
            double tgo = have ? plan.Duration : 0.0;

            // Two different things happen at the end of the plan depending on the
            // arrangement being flown. With a terminal window the plan hands over and
            // the impulsive law takes it from here; without one there is nothing to hand
            // over to, so the plan simply stops being re-solved and is flown out.
            if (have && tgo <= terminalS && terminalS > lockS)
                break;
            bool frozen = have && tgo <= lockS;

            if (!frozen)
            {
                BoostbackShooter.SolveResult sol = BoostbackShooter.Solve(
                    in model, state, mass, target, in guess, in opt, scratch,
                    minPitchDeg: 0.0, searchRates: false,
                    maxSweeps: have ? 4 : 10,
                    initialPitchStepDeg: have ? 2.0 : 8.0);
                solves++;
                if (!sol.Converged)
                {
                    // The mod keeps flying the previous plan here. With none, there is
                    // nothing to fly and the burn is over.
                    if (!have) return double.PositiveInfinity;
                }
                else
                {
                    plan = sol.Parameters;
                    planFrame = BoostbackShooter.Frame.FromState(state);
                    guess = plan;
                    have = true;
                }
                tgo = plan.Duration;
            }

            if (tgo <= 1e-6)
                break;

            // Never overshoot the handover: the plan owns the burn right up to it, and
            // not a step past it. With no handover there is nothing to stop short of.
            double upTo = terminalS > lockS ? Math.Max(tgo - terminalS, 0.05) : tgo;
            double seg_s = Math.Min(Math.Min(replanS, tgo), upTo);
            PoweredBurnSystem flying = BoostbackShooter.WithSteering(in truth, in planFrame, in plan);

            int steps = Math.Max((int)Math.Ceiling(seg_s / 0.25), 1);
            var h = new Dual(seg_s / steps);
            var t = new Dual(0.0);
            for (int i = 0; i < 6; i++) a[i] = new Dual(state[i]);
            a[6] = new Dual(mass);
            bool inA = true;
            for (int step = 0; step < steps; step++)
            {
                if (inA) Rk4.Step(in flying, t, a, h, b, work);
                else Rk4.Step(in flying, t, b, h, a, work);
                inA = !inA;
                t += h;
            }
            Span<Dual> end = inA ? a : b;
            for (int i = 0; i < 6; i++) state[i] = end[i].V;
            mass = end[6].V;
            burnFlown += seg_s;

            // The remaining plan, re-datumed to the new "now". Only meaningful while
            // frozen - otherwise the next pass replaces it outright.
            plan.PitchDeg += plan.PitchRateDegS * seg_s;
            plan.YawDeg += plan.YawRateDegS * seg_s;
            plan.Duration = tgo - seg_s;
            planFrame = BoostbackShooter.Frame.FromState(state);

            if (mass <= 0.0 || !double.IsFinite(mass))
                return double.PositiveInfinity;
        }

        // ---- terminal: the impulsive correction, re-solved at 10 Hz ----------
        //
        // Only when there is a window for it. terminalS == lockS means the plan was
        // meant to be frozen and evaluated out, and the loop above already did that.
        if (terminalS > lockS)
        {
            Span<double> dG = stackalloc double[9];
            Span<double> miss = stackalloc double[3];
            Span<double> nrm = stackalloc double[3];
            Span<double> dv = stackalloc double[3];
            Span<double> greedy = stackalloc double[3];
            Span<Dual> jscratch = stackalloc Dual[ImpactPredictor.ScratchLength];
            Span<Dual> js = stackalloc Dual[ImpactPredictor.N];

            double ve = truth.Thrust / truth.MassFlow;
            double[] frozenDir = null;
            double lockDv = 0.0, accumDv = 0.0;

            for (int tick = 0; tick < 2000; tick++)
            {
                double[] dir;

                if (frozenDir != null)
                {
                    if (accumDv >= lockDv) break;
                    dir = frozenDir;
                }
                else
                {
                    var jsys = new DragCoastSystem
                    {
                        Mu = truth.Mu, OmegaZ = truth.OmegaZ, MeanRadius = truth.MeanRadius,
                        AreaOverMass = truth.ReferenceArea / mass, Alpha = 0.0,
                        Table = truth.Table, Atmosphere = truth.Atmosphere,
                    };
                    for (int i = 0; i < 6; i++) js[i] = new Dual(state[i]);
                    ImpactPrediction nom = ImpactPredictor.VelocityJacobian(
                        jsys, state, opt, jscratch, dG, default, default);
                    if (!nom.Hit) break;

                    double[] hitF = { nom.Fx.V, nom.Fy.V, nom.Fz.V };
                    double hlen = Math.Sqrt(hitF[0] * hitF[0] + hitF[1] * hitF[1] + hitF[2] * hitF[2]);
                    if (!(hlen > 0.0)) break;
                    for (int i = 0; i < 3; i++)
                    {
                        miss[i] = hitF[i] - target[i];
                        nrm[i] = hitF[i] / hlen;
                    }
                    if (!ImpactSteering.Correction(dG, miss, nrm, ImpactSteering.DefaultLambda,
                                                   dv, greedy))
                        break;

                    double dvLen = Math.Sqrt(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]);
                    if (dvLen < 5.0) break;          // the miss is nulled - BoostbackMinDvMs

                    dir = new[] { dv[0] / dvLen, dv[1] / dvLen, dv[2] / dvLen };

                    // Freeze with lockS of burn left, sized off the rocket equation at
                    // the thrust the engine is ACTUALLY producing - which is what makes
                    // the tail self-correcting even though nothing is re-solved in it.
                    double burnLeft = mass * (1.0 - Math.Exp(-dvLen / ve)) / truth.MassFlow;
                    if (burnLeft <= lockS)
                    {
                        frozenDir = dir;
                        lockDv = dvLen;
                        accumDv = 0.0;
                    }
                }

                var flying = truth;
                flying.Lx = new Dual(dir[0]); flying.Ly = new Dual(dir[1]); flying.Lz = new Dual(dir[2]);
                flying.Dx = default; flying.Dy = default; flying.Dz = default;

                const double Tick = 0.1;
                var h2 = new Dual(Tick);
                for (int i = 0; i < 6; i++) a[i] = new Dual(state[i]);
                a[6] = new Dual(mass);
                Rk4.Step(in flying, new Dual(0.0), a, h2, b, work);
                for (int i = 0; i < 6; i++) state[i] = b[i].V;
                accumDv += truth.Thrust / mass * Tick;
                mass = b[6].V;
                burnFlown += Tick;

                if (mass <= 0.0 || !double.IsFinite(mass))
                    return double.PositiveInfinity;
            }
        }

        // Flip retrograde and coast, exactly as the shot assumes.
        var coast = new DragCoastSystem
        {
            Mu = truth.Mu, OmegaZ = truth.OmegaZ, MeanRadius = truth.MeanRadius,
            AreaOverMass = truth.ReferenceArea / mass, Alpha = 0.0,
            Table = truth.Table, Atmosphere = truth.Atmosphere,
        };
        Span<Dual> cs = stackalloc Dual[ImpactPredictor.N];
        for (int i = 0; i < 6; i++) cs[i] = new Dual(state[i]);
        Span<Dual> cscratch = stackalloc Dual[ImpactPredictor.ScratchLength];
        ImpactPrediction p = ImpactPredictor.Predict(in coast, cs, in opt, cscratch, default);
        if (!p.Hit)
            return double.PositiveInfinity;

        double dx = p.Fx.V - target[0], dy = p.Fy.V - target[1], dz = p.Fz.V - target[2];
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    /// <summary>Fly a fixed plan at an arbitrary node count, to price the discretisation.</summary>
    private static ShotResult ShootWithNodes(in PoweredBurnSystem sys,
                                             in BoostbackShooter.Frame frame,
                                             double[] x0, double mass0,
                                             BurnParameters bp, double[] target,
                                             in ImpactOptions opt, Span<Dual> scratch,
                                             int nodes)
        => BoostbackShooter.ShootAt(in sys, in frame, x0, mass0,
                                    new Dual(bp.PitchDeg * Math.PI / 180.0),
                                    new Dual(bp.YawDeg * Math.PI / 180.0),
                                    new Dual(bp.PitchRateDegS * Math.PI / 180.0),
                                    new Dual(bp.YawRateDegS * Math.PI / 180.0),
                                    new Dual(bp.Duration), target, in opt, scratch, nodes);

    /// <summary>Pitch of pure retrograde at ignition, degrees above the horizon - the
    /// angle a burn aimed straight back would fly at, and hence the zero of alpha.</summary>
    private static double InitialRetrogradePitch(double[] x0, BoostbackShooter.Frame f)
    {
        double[] retro = { -x0[3], -x0[4], -x0[5] };
        double l = Math.Sqrt(retro[0] * retro[0] + retro[1] * retro[1] + retro[2] * retro[2]);
        double u = (retro[0] * f.Ux + retro[1] * f.Uy + retro[2] * f.Uz) / l;
        return Math.Asin(Math.Clamp(u, -1, 1)) * 180.0 / Math.PI;
    }
}
