using System.Diagnostics;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using Brutal.Numerics;

internal static partial class Program
{
    private static void UpfgGuidanceRules()
    {
        UpfgSensedDeltaVChecks();
        UpfgDivergenceClearChecks();
        UpfgInsertionSearchChecks();
    }

    private static UpfgVehicle MakeTwoStageOrbitalVehicle(double stage1Thrust = 1_600_000.0, double stage2Thrust = 450_000.0)
    {
        var vehicle = new UpfgVehicle();
        vehicle.Stages.Add(new UpfgStage
        {
            Mode = 1,
            Thrust = stage1Thrust,
            Isp = 320.0,
            MassTotal = 80_000.0,
            MassDry = 32_000.0,
            GLim = 4.0,
            DrainThrottle = null,
        });
        vehicle.Stages.Add(new UpfgStage
        {
            Mode = 1,
            Thrust = stage2Thrust,
            Isp = 430.0,
            MassTotal = 26_000.0,
            MassDry = 4_000.0,
            GLim = 4.0,
        });
        return vehicle;
    }

    private static void UpfgSensedDeltaVChecks()
    {
        const double earthR = 6_371_000.0;
        const double earthMu = 3.986004418e14;
        const double dt = 1.0;

        var r0 = new double3(earthR + 80_000.0, 0.0, 0.0);
        var v0 = new double3(400.0, 2_400.0, 0.0);
        double3 g0 = -earthMu * r0 / Math.Pow(r0.Length(), 3);

        // Pure ballistic step: trapezoidal gravity accounts for the full inertial dv so sensed dv is zero.
        double3 rBallistic = r0 + v0 * dt + 0.5 * g0 * dt * dt;
        double3 g1 = -earthMu * rBallistic / Math.Pow(rBallistic.Length(), 3);
        double3 vBallistic = v0 + 0.5 * (g0 + g1) * dt;
        double3 dvBallistic = UpfgGuidance.SensedDeltaV(v0, vBallistic, g0, g1, dt);
        Check("free-fall sensed dv cancels trapezoidal gravity", dvBallistic.Length() < 1e-12,
            $"got {dvBallistic.Length():G6} m/s");
        Check("raw inertial dv on free-fall is non-zero gravity impulse", (vBallistic - v0).Length() > 9.0);

        // Powered step in gravity: sensed dv isolates thrust acceleration from gravity.
        var aThrust = new double3(12.0, 18.0, -2.5);
        double3 vPowered = v0 + (aThrust + 0.5 * (g0 + g1)) * dt;
        double3 dvPowered = UpfgGuidance.SensedDeltaV(v0, vPowered, g0, g1, dt);
        Check("powered sensed dv recovers thrust dv without gravity",
            (dvPowered - aThrust * dt).Length() < 1e-12,
            $"err={(dvPowered - aThrust * dt).Length():G6} m/s");

        // Static-state probe (dt == 0): returns raw velocity difference (zero on frozen state).
        double3 dvStatic = UpfgGuidance.SensedDeltaV(v0, v0, g0, g0, 0.0);
        Check("static-state probe with dt=0 gives zero sensed dv", dvStatic.Length() == 0.0);

        // Closed-loop Step() with dt > 0 converges cleanly on LEO ascent.
        UpfgTarget leo = UpfgTarget.FromOrbit(200.0, 200.0, 0.0, 0.0, earthR, earthMu);
        UpfgVehicle veh = MakeTwoStageOrbitalVehicle();
        var upfg = new UpfgGuidance();
        double3 r = r0;
        double3 v = v0;
        double mass = veh.Stages[0].MassTotal;
        double mdot = veh.Stages[0].Thrust / (veh.Stages[0].Isp * 9.80665);
        upfg.Step(r, v, mass, earthMu, leo, veh, 1, 0.0);
        for (int i = 0; i < 15; i++)
            upfg.Step(r, v, mass, earthMu, leo, veh, 1, 0.0);
        Check("UPFG converges before closed-loop powered steps", upfg.Converged);

        double tgoBefore = upfg.Tgo;
        double3 gPrev = -earthMu * r / Math.Pow(r.Length(), 3);
        double3 steer = upfg.Steering;
        double accel = veh.Stages[0].Thrust / mass;
        double3 rNext = r + v * dt + 0.5 * (steer * accel + gPrev) * dt * dt;
        double3 gNext = -earthMu * rNext / Math.Pow(rNext.Length(), 3);
        double3 vNext = v + (steer * accel + 0.5 * (gPrev + gNext)) * dt;
        double massNext = mass - mdot * dt;
        veh.Stages[0].MassTotal = massNext;
        upfg.Step(rNext, vNext, massNext, earthMu, leo, veh, 1, dt);
        Check("UPFG stays converged across gravity-corrected powered dt=1s step", upfg.Converged);
        Check("UPFG tgo counts down near 1 s on powered dt=1s step",
            Math.Abs((tgoBefore - upfg.Tgo) - dt) < 0.5,
            $"tgoBefore={tgoBefore:F2}, tgoAfter={upfg.Tgo:F2}");
    }

    private static void UpfgDivergenceClearChecks()
    {
        const double earthR = 6_371_000.0;
        const double earthMu = 3.986004418e14;
        UpfgTarget leo = UpfgTarget.FromOrbit(200.0, 200.0, 0.0, 0.0, earthR, earthMu);
        var r = new double3(earthR + 80_000.0, 0.0, 0.0);
        var v = new double3(400.0, 2_400.0, 0.0);
        UpfgVehicle veh = MakeTwoStageOrbitalVehicle();

        UpfgGuidance MakeConverged()
        {
            var g = new UpfgGuidance();
            for (int i = 0; i < 20; i++)
                g.Step(r, v, veh.Stages[0].MassTotal, earthMu, leo, veh, 1, 0.0);
            return g;
        }

        // 1. Non-finite rollback clears Converged.
        UpfgGuidance gNan = MakeConverged();
        Check("baseline solver starts converged before NaN test", gNan.Converged);
        gNan.Step(new double3(double.NaN, 0.0, 0.0), v, veh.Stages[0].MassTotal, earthMu, leo, veh, 1, 1.0);
        Check("non-finite step rollback clears Converged", !gNan.Converged);

        // 2. Empty vehicle resets and clears Converged.
        UpfgGuidance gEmpty = MakeConverged();
        gEmpty.Step(r, v, veh.Stages[0].MassTotal, earthMu, leo, new UpfgVehicle(), 1, 1.0);
        Check("empty stage list clears Converged", !gEmpty.Converged);

        // 3. Small transient thrust perturbation keeps Converged latched.
        UpfgGuidance gMinor = MakeConverged();
        UpfgVehicle vehMinor = MakeTwoStageOrbitalVehicle(stage1Thrust: 1_550_000.0, stage2Thrust: 440_000.0);
        gMinor.Step(r, v, vehMinor.Stages[0].MassTotal, earthMu, leo, vehMinor, 1, 1.0);
        Check("minor 3% thrust noise keeps Converged latched", gMinor.Converged);

        // 4. Runaway tgo growth clears Converged.
        UpfgGuidance gRunaway = MakeConverged();
        UpfgVehicle vehWeak = MakeTwoStageOrbitalVehicle(stage1Thrust: 350_000.0, stage2Thrust: 120_000.0);
        gRunaway.Step(r, v, vehWeak.Stages[0].MassTotal, earthMu, leo, vehWeak, 1, 1.0);
        Check("runaway tgo jump clears Converged", !gRunaway.Converged,
            $"tgo={gRunaway.Tgo:F1} s, converged={gRunaway.Converged}");

        // 5. Singular radial geometry (|dot(lambda, iz)| < 0.15) clears Converged.
        UpfgGuidance gSingular = MakeConverged();
        // Force vgo nearly radial by passing a velocity matching horizontal orbital speed with large downward radial error.
        double vOrb = Math.Sqrt(earthMu / (earthR + 200_000.0));
        var vRadialError = new double3(-3_500.0, vOrb, 0.0);
        gSingular.Step(r, vRadialError, veh.Stages[0].MassTotal, earthMu, leo, veh, 1, 0.0);
        // Step once more so the newly computed radial vgo feeds into ComputeGuidanceVectors.
        gSingular.Step(r, vRadialError, veh.Stages[0].MassTotal, earthMu, leo, veh, 1, 0.0);
        Check("singular radial lambda geometry clears Converged", !gSingular.Converged);

        // 6. Excessive turning angle (phi > MaxValidTurnAngleRad) clears Converged.
        UpfgGuidance gTurn = MakeConverged();
        UpfgTarget farAlt = UpfgTarget.FromOrbit(2_500.0, 2_500.0, 0.0, 0.0, earthR, earthMu);
        // Demand an extreme altitude jump with a very short remaining stage so lambdadot * (J/L) exceeds MaxValidTurnAngleRad.
        var highTurnVeh = new UpfgVehicle();
        highTurnVeh.Stages.Add(new UpfgStage
        {
            Mode = 1,
            Thrust = 12_000_000.0,
            Isp = 450.0,
            MassTotal = 80_000.0,
            MassDry = 10_000.0,
            GLim = 40.0,
        });
        gTurn.Step(r, new double3(-1_500.0, vOrb - 200.0, 0.0), highTurnVeh.Stages[0].MassTotal, earthMu, farAlt, highTurnVeh, 1, 0.0);
        gTurn.Step(r, new double3(-1_500.0, vOrb - 200.0, 0.0), highTurnVeh.Stages[0].MassTotal, earthMu, farAlt, highTurnVeh, 1, 0.0);
        Check("excessive turning angle or runaway step clears Converged", !gTurn.Converged);
    }

    private static void UpfgInsertionSearchChecks()
    {
        const double earthR = 6_371_000.0;
        const double earthMu = 3.986004418e14;
        var r = new double3(earthR + 200_000.0, 0.0, 0.0);
        double vCirc = Math.Sqrt(earthMu / r.Length());
        var v = new double3(0.0, vCirc, 0.0);

        // Two-stage G-limited upper vehicle climbing from 200 km circular orbit into 300 x 35,786 km GTO.
        var veh = new UpfgVehicle();
        veh.Stages.Add(new UpfgStage
        {
            Mode = 2,
            Thrust = 1_500_000.0,
            Isp = 430.0,
            MassTotal = 120_000.0,
            MassDry = 30_000.0,
            GLim = 2.0,
        });
        veh.Stages.Add(new UpfgStage
        {
            Mode = 2,
            Thrust = 400_000.0,
            Isp = 445.0,
            MassTotal = 25_000.0,
            MassDry = 5_000.0,
            GLim = 2.0,
        });

        UpfgTarget gto = UpfgTarget.FromOrbit(300.0, 35_786.0, 28.5, 0.0, earthR, earthMu);
        var live = new UpfgGuidance();
        for (int i = 0; i < 200; i++)
            live.Step(r, v, veh.Stages[0].MassTotal, earthMu, gto, veh, 1, 0.0);
        Check("GTO preview solver converges with tgo > FreezeTgoS",
            live.Converged && live.Tgo > UpfgInsertionSearch.FreezeTgoS,
            $"converged={live.Converged}, tgo={live.Tgo:F1} s");

        // 1. TryPrice respects explicit tight maxSolves cap.
        bool pricedTight = UpfgInsertionSearch.TryPrice(
            live, gto, 0.0, r, v, veh.Stages[0].MassTotal, earthMu, veh,
            out _, out _, out int tightSolves, maxSolves: 25);
        Check("TryPrice enforces explicit maxSolves cap",
            !pricedTight && tightSolves == 25,
            $"priced={pricedTight}, solves={tightSolves}");

        // 2. Full search on eccentric GTO prices insertions within bounded MaxSearchSolves.
        var search = new UpfgInsertionSearch();
        var sw = Stopwatch.StartNew();
        search.Step(0.0, UpfgInsertionSearch.SearchIntervalS, true, true, live, gto,
            r, v, veh.Stages[0].MassTotal, earthMu, veh);
        sw.Stop();
        Check("eccentric GTO search prices at least one insertion",
            search.LastSearchPriced > 0 && search.LastSearchSolves > 0,
            $"priced={search.LastSearchPriced}/{search.LastSearchPoints}, solves={search.LastSearchSolves}, wall={sw.Elapsed.TotalMilliseconds:F2} ms");
        Check("eccentric GTO search stays within hard MaxSearchSolves bound",
            search.LastSearchSolves <= UpfgInsertionSearch.MaxSearchSolves,
            $"solves={search.LastSearchSolves}, max={UpfgInsertionSearch.MaxSearchSolves}");
        double goalDeg = UpfgTarget.RadToDeg(search.GoalNu);
        Check("eccentric GTO search goal lies on climbing side within MaxNuDeg",
            goalDeg >= -1e-9 && goalDeg <= UpfgInsertionSearch.MaxNuDeg + 1e-9,
            $"goalDeg={goalDeg:F2}");

        // 3. Switching to a circular target skips search and clears stale eccentric GoalNu / SavingS / SavingMs.
        int solvesBeforeCircular = search.LastSearchSolves;
        UpfgTarget circular = UpfgTarget.FromOrbit(250.0, 250.0, 28.5, 0.0, earthR, earthMu);
        search.Step(UpfgInsertionSearch.SearchIntervalS * 2.0, UpfgInsertionSearch.SearchIntervalS,
            true, true, live, circular, r, v, veh.Stages[0].MassTotal, earthMu, veh);
        Check("circular target skips search without extra solves",
            search.LastSearchSolves == solvesBeforeCircular);
        Check("circular target resets GoalNu to DefaultNu and clears stale savings",
            Math.Abs(search.GoalNu - search.DefaultNu) < 1e-12
            && double.IsNaN(search.SavingS) && double.IsNaN(search.SavingMs));
    }
}
