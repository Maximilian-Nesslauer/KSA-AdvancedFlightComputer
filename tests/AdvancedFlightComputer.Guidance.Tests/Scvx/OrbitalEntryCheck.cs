using AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;


/// <summary>
/// --orbital-entry: the energy-domain entry planner and its tracker on an Earth-sized body with an exponential atmosphere.
///
/// Open loop: each vehicle's cold plan must pass its own gate, keep to its altitude floors and end in the hand-over box over the target.
/// Closed loop: a 3-D point-mass truth (spherical, non-rotating, the bank's side included) flies the tracker with re-solves, in air 15 % denser than the model and with L/D 8 % low, and must reach the hand-over near the target.
/// </summary>
internal static class OrbitalEntryCheck
{
    const double R0 = 6_371_000.0;
    const double Mu = 3.986004418e14;
    const double Rho0 = 1.225, Hs = 7_200.0, Top = 120_000.0, A0 = 340.0;

    sealed record Vehicle(string Name, double Mass, double Area, double Cd, double Ld, double RangeKm, double CrossKm, double GMax = 2.5);

    static readonly Vehicle[] Vehicles =
    [
        new("shuttle-like L/D 1.0", 90_000, 250, 0.85, 1.0, 7_000, 300),
        new("belly-flop L/D 0.35", 100_000, 420, 1.15, 0.35, 3_200, 60),
        new("capsule L/D 0.3", 5_800, 11.9, 1.3, 0.3, 2_600, 40, 4.0),
    ];

    static EntryModel Model(Vehicle v, double rhoScale = 1.0, double ldScale = 1.0) => new()
    {
        BodyRadius = R0, Mu = Mu,
        Density = h => Rho0 * Math.Exp(-h / Hs),
        SpeedOfSound = _ => A0,
        AtmosphereTop = Top,
        DragCoefficient = _ => v.Cd,
        LiftToDrag = _ => v.Ld,
        Mass = v.Mass, ReferenceArea = v.Area,
        DensityScale = rhoScale, LiftToDragScale = ldScale,
    };

    public static int Run(bool verbose)
    {
        int fails = TrimCheck();
        foreach (var v in Vehicles)
        {
            fails += OpenLoop(v, verbose);
            fails += ClosedLoop(v, verbose);
        }
        Console.WriteLine(fails == 0 ? "PASS orbital-entry" : $"FAIL orbital-entry ({fails})");
        return fails == 0 ? 0 : 1;
    }

    /// <summary>The trim schedule finds a restoring zero of C_m, prefers the one nearest the preferred angle, and falls back where there is none.</summary>
    static int TrimCheck()
    {
        double[] mach = [2, 10, 20];
        double[] alpha = [0, 30, 45, 60, 65, 70, 90, 120, 180];
        int nm = mach.Length, na = alpha.Length;
        var cd = new double[nm * na]; var cl = new double[nm * na]; var cm = new double[nm * na];
        for (int i = 0; i < nm; i++)
            for (int j = 0; j < na; j++)
            {
                double a = alpha[j] * Math.PI / 180;
                cd[i * na + j] = 0.2 + 1.2 * Math.Sin(a) * Math.Sin(a);
                cl[i * na + j] = 1.2 * Math.Sin(a) * Math.Cos(a);
                double trimDeg = i == 2 ? 200 : 62 + 2 * i;             // Mach 20 has no zero in the band
                cm[i * na + j] = -Math.Sin((alpha[j] - trimDeg) * Math.PI / 180);
            }
        var t = EntryTrim.Build(mach, alpha, cd, cl, cm, 30, 90, 60);
        bool ok = t.Trimmed[0] && t.Trimmed[1] && !t.Trimmed[2]
                  && Math.Abs(t.AlphaDeg[0] - 62) < 1.5 && Math.Abs(t.AlphaDeg[1] - 64) < 1.5 && Math.Abs(t.AlphaDeg[2] - 60) < 1e-9
                  && t.LiftToDrag[0] > 0.3;
        Console.WriteLine($"  {(ok ? "ok  " : "FAIL")} trim schedule: alpha {t.AlphaDeg[0]:F1}/{t.AlphaDeg[1]:F1}/{t.AlphaDeg[2]:F1}, trimmed {t.TrimmedRows}/3, L/D {t.LiftToDrag[0]:F2}");
        return ok ? 0 : 1;
    }

    static EntryConfig Config(Vehicle v) => new() { HeatingRateMax = 0, DynamicPressureMax = 16_000, LoadFactorMax = v.GMax };

    static int OpenLoop(Vehicle v, bool verbose)
    {
        var cfg = Config(v);
        var model = Model(v);
        var start = EntryPlanner.PredictStart(model, cfg, R0 + 121_000, 7_600, -1.3 * Math.PI / 180, v.RangeKm * 1000 / R0);
        if (start == null) { Console.WriteLine($"  FAIL {v.Name}: never meets the air"); return 1; }
        var planner = new EntryPlanner();
        var res = planner.Solve(model, cfg, start.Value.E, start.Value.X, null);
        var p = res.Plan;
        if (p == null) { Console.WriteLine($"  FAIL {v.Name}: no plan ({res.Why})"); return 1; }

        double floorWorst = 0;
        for (int k = 0; k < p.Count; k++) floorWorst = Math.Max(floorWorst, p.FloorAltitudeM[k] - p.AltitudeM[k]);
        double hf = p.AltitudeM[^1];
        bool ok = p.Converged && floorWorst < 1_000 && Math.Abs(hf - cfg.HandoverAltitude) < cfg.HandoverAltitudeTol + 500;
        Console.WriteLine($"  {(ok ? "ok  " : "FAIL")} {v.Name} open loop: {(p.Converged ? "converged" : "refused " + res.Why)}, "
                        + $"{p.Iterations} it, {p.SolveMs:F0} ms, defect {p.MaxDefect:F4}, range err {p.TerminalRangeErrorM / 1000:F2} km, "
                        + $"floor worst {floorWorst:F0} m, h_f {hf / 1000:F1} km, coast {start.Value.SecondsAhead:F0} s");
        if (verbose || !ok)
        {
            for (int k = 0; k < p.Count; k += Math.Max(1, p.Count / 12))
                Console.WriteLine($"      V {p.SpeedMs[k],6:F0}  h {p.AltitudeM[k] / 1000,6:F1} (floor {p.FloorAltitudeM[k] / 1000,5:F1})  "
                                + $"D {p.DragMs2[k],5:F1} [{p.DragMinMs2[k],5:F1}..{p.DragMaxMs2[k],5:F1}]  u {p.U[k],5:F2}  bank {p.At(p.E[k]).BankRefDeg,5:F0}  "
                                + $"s {p.S[k] * R0 / 1000,7:F0} km [{p.RangeShortM[k] / 1000,6:F0}..{p.RangeLongM[k] / 1000,6:F0}]");
        }
        return ok ? 0 : 1;
    }

    // ---- closed loop ----

    /// <summary>3-D point mass over a spherical, non-rotating body: r, lon, lat, V, gamma, heading (from north, clockwise).</summary>
    static void Truth(double[] y, double bankRad, EntryModel m, double[] dy)
    {
        double r = y[0], lat = y[2], v = y[3], g = y[4], psi = y[5];
        double h = r - R0;
        double d = m.DragAccel(h, v);
        double l = d * m.Ld(m.Mach(h, v));
        double grav = Mu / (r * r);
        dy[0] = v * Math.Sin(g);
        dy[1] = v * Math.Cos(g) * Math.Sin(psi) / (r * Math.Cos(lat));
        dy[2] = v * Math.Cos(g) * Math.Cos(psi) / r;
        dy[3] = -d - grav * Math.Sin(g);
        dy[4] = (l * Math.Cos(bankRad) - (grav - v * v / r) * Math.Cos(g)) / v;
        dy[5] = (l * Math.Sin(bankRad) / Math.Cos(g) + v * v / r * Math.Cos(g) * Math.Sin(psi) * Math.Tan(lat)) / v;
    }

    static (double S, double Cross) RangeAndCross(double[] y, double tLat, double tLon)
    {
        double lat = y[2], lon = y[1], psi = y[5];
        double cosS = Math.Sin(lat) * Math.Sin(tLat) + Math.Cos(lat) * Math.Cos(tLat) * Math.Cos(tLon - lon);
        double s = Math.Acos(Math.Clamp(cosS, -1, 1));
        double bearing = Math.Atan2(Math.Sin(tLon - lon) * Math.Cos(tLat),
                                    Math.Cos(lat) * Math.Sin(tLat) - Math.Sin(lat) * Math.Cos(tLat) * Math.Cos(tLon - lon));
        // + target to the left of the track
        double cross = Math.Asin(Math.Clamp(Math.Sin(s) * Math.Sin(psi - bearing), -1, 1));
        return (s, cross);
    }

    static int ClosedLoop(Vehicle v, bool verbose)
    {
        var cfg = Config(v);
        var truth = Model(v, rhoScale: 1.15, ldScale: 0.92);
        var nominal = Model(v);
        double tLat = v.CrossKm * 1000 / R0, tLon = v.RangeKm * 1000 / R0;

        double[] y = [R0 + 121_000, 0, 0, 7_600, -1.3 * Math.PI / 180, Math.PI / 2];
        var planner = new EntryPlanner();
        var tracker = new EntryTracker(cfg);
        EntryPlan? plan = null;
        double rhoK = 1.0, ldK = 1.0;
        double dt = 0.5, t = 0, nextSolve = 0;
        double ef = EntryPlanner.FinalEnergy(nominal, cfg);
        int reversals = 0, lastSide = 0, solves = 0, refused = 0;
        double gMax = 0, qMax = 0, solveMsMax = 0;
        var dy = new double[6]; var k1 = new double[6]; var k2 = new double[6]; var k3 = new double[6]; var k4 = new double[6]; var yt = new double[6];

        while (t < 4_000)
        {
            double h = y[0] - R0, vel = y[3];
            double rN = y[0] / R0, vN = vel / nominal.SpeedUnit;
            double e = EntryDynamics.Energy(rN, vN);
            if (e >= ef || h < 5_000) break;
            var (s, cross) = RangeAndCross(y, tLat, tLon);

            double dMeas = truth.DragAccel(h, vel);
            double ldMeas = truth.Ld(truth.Mach(h, vel));
            if (dMeas > 0.3)
            {
                double dModel = nominal.WithScales(1, 1).DragAccel(h, vel);
                rhoK += 0.2 * (dMeas / Math.Max(dModel, 1e-9) - rhoK);
                ldK += 0.2 * (ldMeas / nominal.Ld(nominal.Mach(h, vel)) - ldK);
            }

            if (t >= nextSolve)
            {
                var m = nominal.WithScales(rhoK, ldK);
                double e0; double[] x0;
                if (dMeas >= cfg.StartDrag) { e0 = e; x0 = [rN, y[4], s]; }
                else
                {
                    var st = EntryPlanner.PredictStart(m, cfg, y[0], vel, y[4], s);
                    if (st == null) break;
                    e0 = st.Value.E; x0 = st.Value.X;
                }
                var res = planner.Solve(m, cfg, e0, x0, plan);
                solves++;
                if (res.Plan != null && res.Plan.Converged) { plan = res.Plan; solveMsMax = Math.Max(solveMsMax, res.Plan.SolveMs); }
                else refused++;
                nextSolve = t + (dMeas >= cfg.StartDrag ? 5.0 : 30.0);
            }

            var outp = tracker.Update(new EntryTracker.Input(dt, e, dMeas, vel * Math.Sin(y[4]), ldMeas, Hs, cross, vN, plan));
            if (lastSide != 0 && outp.Side != lastSide) reversals++;
            if (outp.Side != 0) lastSide = outp.Side;
            // Positive bank lifts to the right; in Truth positive bank turns towards +heading (clockwise, right). Same sense.
            double bank = outp.BankDeg * Math.PI / 180;
            gMax = Math.Max(gMax, truth.LoadFactor(h, vel));
            qMax = Math.Max(qMax, truth.DynamicPressure(h, vel));

            Truth(y, bank, truth, k1);
            for (int i = 0; i < 6; i++) yt[i] = y[i] + 0.5 * dt * k1[i];
            Truth(yt, bank, truth, k2);
            for (int i = 0; i < 6; i++) yt[i] = y[i] + 0.5 * dt * k2[i];
            Truth(yt, bank, truth, k3);
            for (int i = 0; i < 6; i++) yt[i] = y[i] + dt * k3[i];
            Truth(yt, bank, truth, k4);
            for (int i = 0; i < 6; i++) y[i] += dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
            t += dt;
            if (verbose && ((int)(t / dt)) % 60 == 0)
                Console.WriteLine($"      t {t,5:F0}  h {h / 1000,5:F1}  V {vel,5:F0}  D {dMeas,5:F1}/{outp.DragRefMs2,5:F1}  bank {outp.BankDeg,6:F1}  s {s * R0 / 1000,6:F0}  x {cross * R0 / 1000,6:F1}  {outp.Status}");
        }

        var (sf, cf) = RangeAndCross(y, tLat, tLon);
        double rangeKm = sf * R0 / 1000, crossKm = cf * R0 / 1000;
        double hEnd = (y[0] - R0) / 1000;
        bool ok = plan != null && rangeKm < 20 && Math.Abs(crossKm) < 15 && gMax < cfg.LoadFactorMax * 1.15 && Math.Abs(hEnd - cfg.HandoverAltitude / 1000) < 6;
        Console.WriteLine($"  {(ok ? "ok  " : "FAIL")} {v.Name} closed loop (rho x1.15, L/D x0.92): miss {rangeKm:F1} km, cross {crossKm:F1} km, "
                        + $"h_end {hEnd:F1} km, V_end {y[3]:F0} m/s, g max {gMax:F2}, q max {qMax / 1000:F1} kPa, reversals {reversals}, "
                        + $"solves {solves} ({refused} refused), worst {solveMsMax:F0} ms, t {t:F0} s");
        return ok ? 0 : 1;
    }
}
