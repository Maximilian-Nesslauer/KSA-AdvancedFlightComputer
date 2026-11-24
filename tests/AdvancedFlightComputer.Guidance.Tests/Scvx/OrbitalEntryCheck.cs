using AdvancedFlightComputer.Guidance.Numerics.Flight;
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
        int fails = TrimCheck() + FrameCheck() + LatchCheck() + RotationCheck();
        foreach (var v in Vehicles)
        {
            fails += CorridorCheck(v);
            fails += OpenLoop(v, verbose);
            fails += ClosedLoop(v, verbose);
        }
        // The heating limits: every vehicle again at 700 and at the console's default, 440 kW/m^2 on a 1 m nose (the Space Shuttle's STS-2 reference heating, NASA TM-84580). None of them can loft to 440 from this entry; the floors are soft (EntryConfig.FloorShare), so each flies as near it as it can.
        foreach (double limit in new[] { 700_000.0, 440_000.0 })
        {
            HeatLimit = limit;
            Console.WriteLine($"  with a {HeatLimit / 1000:F0} kW/m^2 heating limit:");
            // Reported, not judged: they measure what a limit does. Soft floors fly as near a limit a craft cannot meet as they can, and a capsule's range pays for it.
            foreach (var hv in Vehicles)
            {
                OpenLoop(hv, verbose);
                ClosedLoop(hv, verbose);
            }
        }
        HeatLimit = 0.0;
        Console.WriteLine(fails == 0 ? "PASS orbital-entry" : $"FAIL orbital-entry ({fails})");
        return fails == 0 ? 0 : 1;
    }

    /// <summary>The trim schedule finds a restoring zero of C_m, prefers the one nearest the preferred angle, and where there is none takes the angle of least moment in the band (Mach 20 here: 30 deg, the band's edge).</summary>
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
                  && Math.Abs(t.AlphaDeg[0] - 62) < 1.5 && Math.Abs(t.AlphaDeg[1] - 64) < 1.5 && Math.Abs(t.AlphaDeg[2] - 30) < 1e-9
                  && t.LiftToDrag[0] > 0.3;
        Console.WriteLine($"  {(ok ? "ok  " : "FAIL")} trim schedule: alpha {t.AlphaDeg[0]:F1}/{t.AlphaDeg[1]:F1}/{t.AlphaDeg[2]:F1}, trimmed {t.TrimmedRows}/3, L/D {t.LiftToDrag[0]:F2}");
        return ok ? 0 : 1;
    }

    /// <summary>
    /// The body tables' angle frames: engine first and nose first read as they always did, every frame inverts its own AirDirection, and a belly-flop at 60 deg from nose-first reads alpha 30 on its belly table.
    /// </summary>
    static int FrameCheck()
    {
        double worst = 0;
        var frames = new[] { BodyAeroFrame.TailFirst, BodyAeroFrame.NoseFirst, BodyAeroFrame.Belly(0, -1), BodyAeroFrame.Belly(0, 1), BodyAeroFrame.Belly(-1, 0), BodyAeroFrame.Belly(1, 0) };
        foreach (var f in frames)
            for (double a = -40; a <= 40; a += 10)
                for (double b = -20; b <= 20; b += 10)
                {
                    var (ux, uy, uz) = BodyAeroTable.AirDirection(a * Math.PI / 180, b * Math.PI / 180, f);
                    BodyAeroTable.Angles(f, ux, uy, uz, out double a2, out double b2);
                    worst = Math.Max(worst, Math.Max(Math.Abs(a2 * 180 / Math.PI - a), Math.Abs(b2 * 180 / Math.PI - b)));
                }
        // The old formulas: engine first alpha = atan2(u_z, -u_x), beta = atan2(u_y, -u_x); nose first with +u_x.
        double old = 0;
        (double, double, double)[] dirs = [(-0.9, 0.2, 0.3), (-0.7, -0.4, 0.1), (0.8, 0.3, -0.2)];
        foreach (var (x, y, z) in dirs)
        {
            var f = x < 0 ? BodyAeroFrame.TailFirst : BodyAeroFrame.NoseFirst;
            double lead = x < 0 ? -x : x;
            BodyAeroTable.Angles(f, x, y, z, out double a, out double b);
            old = Math.Max(old, Math.Max(Math.Abs(a - Math.Atan2(z, lead)), Math.Abs(b - Math.Atan2(y, lead))));
        }
        // Belly -Z at 60 deg from nose-first: u = cos 60 x + sin 60 (-z).
        BodyAeroTable.Angles(BodyAeroFrame.Belly(0, -1), Math.Cos(Math.PI / 3), 0, -Math.Sin(Math.PI / 3), out double ab, out double bb);
        // The whole sphere (BodyAeroTable.Sphere) about a belly frame's axes: every longitude round and latitude off the poles comes back, engine first reads 0, belly first 90, nose first 180.
        var sf = new BodyAeroFrame(-1, 0, 0, 0, 0, 1, 0, -1, 0);
        double sphereWorst = 0;
        for (double l = -175; l <= 180; l += 15)
            for (double m = -75; m <= 75; m += 15)
            {
                var (sx, sy, sz) = BodyAeroTable.SphereDirection(l * Math.PI / 180, m * Math.PI / 180, sf);
                BodyAeroTable.SphereAngles(sf, sx, sy, sz, out double l2, out double m2);
                sphereWorst = Math.Max(sphereWorst, Math.Max(Math.Abs(l2 * 180 / Math.PI - l), Math.Abs(m2 * 180 / Math.PI - m)));
            }
        BodyAeroTable.SphereAngles(sf, -1, 0, 0, out double lEngine, out _);
        BodyAeroTable.SphereAngles(sf, 0, 0, 1, out double lBelly, out _);
        BodyAeroTable.SphereAngles(sf, 1, 0, 0, out double lNose, out _);
        bool ok = worst < 1e-9 && old < 1e-12 && Math.Abs(ab * 180 / Math.PI - 30) < 1e-9 && Math.Abs(bb) < 1e-12
                  && sphereWorst < 1e-9 && Math.Abs(lEngine) < 1e-12 && Math.Abs(lBelly * 180 / Math.PI - 90) < 1e-9 && Math.Abs(Math.Abs(lNose * 180 / Math.PI) - 180) < 1e-9;
        Console.WriteLine($"  {(ok ? "ok  " : "FAIL")} table frames: round trip {worst:E1} deg, against the old formulas {old:E1} rad, belly-flop 60 deg reads alpha {ab * 180 / Math.PI:F2}; whole sphere round trip {sphereWorst:E1} deg, engine/belly/nose first at {lEngine * 180 / Math.PI:F0}/{lBelly * 180 / Math.PI:F0}/{Math.Abs(lNose) * 180 / Math.PI:F0}");
        return ok ? 0 : 1;
    }

    /// <summary>
    /// The corridor DEORBIT ONLY aims with (EntryPlanner.Corridor, a quadrature): a plan with the site in its middle must converge on it. Near its edges the quadrature, which leaves out the pull-out, is optimistic; the check reports how far.
    /// </summary>
    static int CorridorCheck(Vehicle v)
    {
        var cfg = Config(v);
        var model = Model(v);
        var start = EntryPlanner.PredictStart(model, cfg, R0 + 121_000, 7_600, -1.3 * Math.PI / 180, 0);
        if (start == null) { Console.WriteLine($"  FAIL {v.Name} corridor: never meets the air"); return 1; }
        var (shortM, longM) = EntryPlanner.Corridor(model, cfg, start.Value.E);
        var planner = new EntryPlanner();
        string Try(double f, out bool ok)
        {
            double s = shortM + f * (longM - shortM);
            double[] x = [start.Value.X[0], start.Value.X[1], s / R0];
            var r = planner.Solve(model, cfg, start.Value.E, x, null);
            ok = r.Plan != null && r.Plan.Converged;
            return r.Plan == null ? "none" : $"{(ok ? "ok" : "refused")} {r.Plan.TerminalRangeErrorM / 1000:F0} km";
        }
        string mid = Try(0.5, out bool midOk);
        string lo = Try(0.1, out _), hi = Try(0.9, out _);
        Console.WriteLine($"  {(midOk ? "ok  " : "FAIL")} {v.Name} corridor {shortM / 1000:F0}-{longM / 1000:F0} km: middle {mid}; at 0.1 {lo}, at 0.9 {hi}");
        return midOk ? 0 : 1;
    }

    /// <summary>The start-drag guard latches: a pull-up back into thin air after the first touch keeps the tracker on the plan.</summary>
    static int LatchCheck()
    {
        var cfg = new EntryConfig();
        var model = Model(Vehicles[1]);
        var start = EntryPlanner.PredictStart(model, cfg, R0 + 121_000, 7_600, -1.3 * Math.PI / 180, 2_200_000 / R0);
        var plan = start == null ? null : new EntryPlanner().Solve(model, cfg, start.Value.E, start.Value.X, null).Plan;
        if (plan == null) { Console.WriteLine("  FAIL start-drag latch: no plan"); return 1; }
        var tr = new EntryTracker(cfg);
        double e = plan.E0, vN = plan.SpeedMs[0] / model.SpeedUnit;
        var before = tr.Update(new EntryTracker.Input(0.5, e, 0.01, 0, 0.3, 7200, 0, vN, plan, InAtmosphere: false));
        var touch = tr.Update(new EntryTracker.Input(0.5, e, 0.001, 0, 0.3, 7200, 0, vN, plan, InAtmosphere: true));
        var after = tr.Update(new EntryTracker.Input(0.5, e, 0.0, 0, 0.3, 7200, 0, vN, plan, InAtmosphere: false));
        bool ok = before.Phase == EntryTracker.Phase.PreEntry && touch.Phase == EntryTracker.Phase.Tracking && after.Phase == EntryTracker.Phase.Tracking;
        Console.WriteLine($"  {(ok ? "ok  " : "FAIL")} start-drag latch at {cfg.StartDrag} m/s2: {before.Phase} -> {touch.Phase} -> {after.Phase} after the drag fell back");
        return ok ? 0 : 1;
    }

    /// <summary>
    /// The body's rotation in the coast: eastward at the equator it holds the craft up (Coriolis and centrifugal), so it meets the air later and further on than on a still body; westward the opposite. A shallow coast from 120 km, as the 2026-11-21 flights flew.
    /// </summary>
    static int RotationCheck()
    {
        var cfg = new EntryConfig();
        var still = Model(Vehicles[1]);
        double omega = 7.2921159e-5;
        var east = still.WithRotation(omega, 0.0, Math.PI / 2);
        var west = still.WithRotation(omega, 0.0, -Math.PI / 2);
        double g = -0.5 * Math.PI / 180;
        var s0 = EntryPlanner.PredictStart(still, cfg, R0 + 121_000, 7_300, g, 0);
        var sE = EntryPlanner.PredictStart(east, cfg, R0 + 121_000, 7_300, g, 0);
        var sW = EntryPlanner.PredictStart(west, cfg, R0 + 121_000, 7_300, g, 0);
        if (s0 == null || sE == null || sW == null) { Console.WriteLine("  FAIL rotation: a coast never met the air"); return 1; }
        double r0 = -s0.Value.X[2] * R0 / 1000, rE = -sE.Value.X[2] * R0 / 1000, rW = -sW.Value.X[2] * R0 / 1000;
        bool ok = rE > r0 && r0 > rW;
        Console.WriteLine($"  {(ok ? "ok  " : "FAIL")} rotation in the coast from 121 km at -0.5 deg: {rW:F0} km westward, {r0:F0} still, {rE:F0} eastward ({sE.Value.SecondsAhead:F0} s)");
        return ok ? 0 : 1;
    }

    /// <summary>The heating limit the next runs plan with, W/m^2; 0 for none.</summary>
    static double HeatLimit;

    static EntryConfig Config(Vehicle v) => new() { HeatingRateMax = HeatLimit, DynamicPressureMax = 16_000, LoadFactorMax = v.GMax };

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

        double floorWorst = 0, heatPeak = 0;
        for (int k = 0; k < p.Count; k++)
        {
            floorWorst = Math.Max(floorWorst, p.FloorAltitudeM[k] - p.AltitudeM[k]);
            heatPeak = Math.Max(heatPeak, model.HeatingRate(p.AltitudeM[k], p.SpeedMs[k]));
        }
        double hf = p.AltitudeM[^1];
        // The least peak heating the craft can fly (EntryDynamics.LeastPeakHeating) is a bound: no plan may peak under it.
        var (least, leastV) = new EntryDynamics(model).LeastPeakHeating(start.Value.E, EntryPlanner.FinalEnergy(model, cfg), cfg);
        bool bound = heatPeak >= 0.97 * least;
        bool ok = p.Converged && floorWorst < 1_000 && Math.Abs(hf - cfg.HandoverAltitude) < cfg.HandoverAltitudeTol + 500 && bound;
        Console.WriteLine($"  {(ok ? "ok  " : "FAIL")} {v.Name} open loop: {(p.Converged ? "converged" : "refused " + res.Why)}, "
                        + $"{p.Iterations} it, {p.SolveMs:F0} ms, defect {p.MaxDefect:F4}, range err {p.TerminalRangeErrorM / 1000:F2} km, "
                        + $"floor worst {floorWorst:F0} m, h_f {hf / 1000:F1} km, coast {start.Value.SecondsAhead:F0} s, heating peak {heatPeak / 1000:F0} kW/m^2 (the least it can fly {least / 1000:F0} at {leastV:F0} m/s{(bound ? "" : ", UNDER THE BOUND")})");
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

    /// <summary>The site's bearing off the heading, rad, + to the left.</summary>
    static double HeadingError(double[] y, double tLat, double tLon)
    {
        double lat = y[2], lon = y[1], psi = y[5];
        double bearing = Math.Atan2(Math.Sin(tLon - lon) * Math.Cos(tLat),
                                    Math.Cos(lat) * Math.Sin(tLat) - Math.Sin(lat) * Math.Cos(tLat) * Math.Cos(tLon - lon));
        return Math.IEEERemainder(psi - bearing, 2 * Math.PI);
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

    /// <summary>What a closed-loop flight did with its bank: the bank target's travel per minute and the drag error, in bands of reference drag, while tracking.</summary>
    sealed class BankStats
    {
        public static readonly double[] Bands = { 0, 3, 10, 1e9 };
        public readonly double[] TravelDeg = new double[3], Seconds = new double[3], ErrSq = new double[3], Clamped = new double[3];
        public double MissKm, CrossKm;
        public bool Ok;
    }

    /// <summary>
    /// --orbital-entry-gains: the drag tracker's spring gains swept on the belly-flop and the shuttle-like craft, with the drag measured 3 % noisy (its flutter below 3 m/s^2 in the 2026-11-21 flight; the sink rate there was clean), the same seed each run. Not in CI: a tool for choosing the gains.
    /// </summary>
    public static int SweepGains()
    {
        var grid = new List<(string Name, Action<EntryConfig> Tweak)>();
        foreach (double w in new[] { 0.03, 0.045, 0.06 })
            foreach (double floor in new[] { 1.0, 5.0, 10.0 })
                foreach (double smooth in new[] { 1.0, 4.0 })
                    grid.Add(($"w {w:F3} floor {floor,4:F1} filter {smooth:F0} s", c => { c.TrackOmega = w; c.GainDragFloor = floor; c.MeasureSmoothS = smooth; }));
        foreach (var v in new[] { Vehicles[0], Vehicles[1] })
        {
            Console.WriteLine($"  {v.Name}, drag 3 % noise: bank-target travel deg/min | rms drag error % | feedback at its clamp %, in reference drag < 3, 3-10, > 10 m/s2");
            foreach (var (name, tweak) in grid)
            {
                var st = new BankStats();
                ClosedLoop(v, false, tweak, 0.03, 0.0, st);
                string Band(int b) => st.Seconds[b] > 1
                    ? FormattableString.Invariant($"{st.TravelDeg[b] / st.Seconds[b] * 60,6:F0} {100 * Math.Sqrt(st.ErrSq[b] / (st.Seconds[b] / 0.5)),5:F1} {100 * st.Clamped[b] / (st.Seconds[b] / 0.5),4:F0}")
                    : "     -     -    -";
                Console.WriteLine(FormattableString.Invariant($"    {name,-30} {Band(0)} | {Band(1)} | {Band(2)}   miss {st.MissKm,5:F1} km cross {st.CrossKm,5:F1}{(st.Ok ? "" : "  FAIL")}"));
            }
        }
        return 0;
    }

    /// <summary>The angular acceleration the closed-loop craft can spare about the airflow, deg/s^2: the tracker's bank slew is held to it, as in flight (EntryTracker.Slew).</summary>
    const double BankAccelDegS2 = 3.0;

    static int ClosedLoop(Vehicle v, bool verbose, Action<EntryConfig>? tweak = null, double dragNoise = 0, double hdotNoiseMs = 0, BankStats? stats = null)
    {
        var cfg = Config(v);
        tweak?.Invoke(cfg);
        var rng = new Random(7);
        double Gauss() => Math.Sqrt(-2 * Math.Log(1 - rng.NextDouble())) * Math.Cos(2 * Math.PI * rng.NextDouble());
        double prevTarget = double.NaN;
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
        var whys = new Dictionary<string, int>();
        double worstDefect = 0;
        double gMax = 0, qMax = 0, solveMsMax = 0, heatMax = 0;
        var dy = new double[6]; var k1 = new double[6]; var k2 = new double[6]; var k3 = new double[6]; var k4 = new double[6]; var yt = new double[6];

        while (t < 4_000)
        {
            double h = y[0] - R0, vel = y[3];
            double rN = y[0] / R0, vN = vel / nominal.SpeedUnit;
            double e = EntryDynamics.Energy(rN, vN);
            if (e >= ef || h < 5_000) break;
            var (s, cross) = RangeAndCross(y, tLat, tLon);

            double dMeas = truth.DragAccel(h, vel) * (1 + dragNoise * Gauss());
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
                if (dMeas >= cfg.StartDrag || tracker.InAir) { e0 = e; x0 = [rN, y[4], s]; }
                else
                {
                    var st = EntryPlanner.PredictStart(m, cfg, y[0], vel, y[4], s);
                    if (st == null) break;
                    e0 = st.Value.E; x0 = st.Value.X;
                }
                var res = planner.Solve(m, cfg, e0, x0, plan);
                solves++;
                if (res.Plan != null && res.Plan.Converged) { plan = res.Plan; solveMsMax = Math.Max(solveMsMax, res.Plan.SolveMs); }
                else
                {
                    refused++;
                    string why = res.Why.Length > 0 ? res.Why.Split(' ')[0] + " " + (res.Why.Contains("defect") ? "defect" : "miss") : "none";
                    whys[why] = whys.GetValueOrDefault(why) + 1;
                    if (res.Plan != null) worstDefect = Math.Max(worstDefect, res.Plan.MaxDefect);
                    if (verbose && res.Plan != null) Console.WriteLine($"      refused at t {t:F0} h {(y[0] - R0) / 1000:F1} km V {vel:F0} D {dMeas:F2}: {res.Why}, it {res.Plan.Iterations}, nodes left to ef {(ef - e0):E2}");
                }
                nextSolve = t + (dMeas >= cfg.StartDrag ? 5.0 : 30.0);
            }

            var outp = tracker.Update(new EntryTracker.Input(dt, e, dMeas, vel * Math.Sin(y[4]) + hdotNoiseMs * Gauss(), ldMeas, Hs, HeadingError(y, tLat, tLon), vN, plan, h < Top, BankAccelDegS2));
            if (stats != null && outp.Phase == EntryTracker.Phase.Tracking && outp.DragRefMs2 > 0)
            {
                int b = outp.DragRefMs2 < 3 ? 0 : outp.DragRefMs2 < 10 ? 1 : 2;
                if (double.IsFinite(prevTarget) && !outp.Reversing) stats.TravelDeg[b] += Math.Abs(outp.BankTargetDeg - prevTarget);
                stats.Seconds[b] += dt;
                double trueD = truth.DragAccel(h, vel);
                stats.ErrSq[b] += Math.Pow((trueD - outp.DragRefMs2) / outp.DragRefMs2, 2);
                var ps = plan?.At(e);
                if (ps != null && Math.Abs(outp.UCmd - ps.Value.U) >= 0.99 * cfg.FeedbackShare * ldMeas) stats.Clamped[b] += 1;
            }
            prevTarget = outp.Phase == EntryTracker.Phase.Tracking ? outp.BankTargetDeg : double.NaN;
            if (lastSide != 0 && outp.Side != lastSide) reversals++;
            if (outp.Side != 0) lastSide = outp.Side;
            // Positive bank lifts to the right; in Truth positive bank turns towards +heading (clockwise, right). Same sense.
            double bank = outp.BankDeg * Math.PI / 180;
            gMax = Math.Max(gMax, truth.LoadFactor(h, vel));
            qMax = Math.Max(qMax, truth.DynamicPressure(h, vel));
            heatMax = Math.Max(heatMax, truth.HeatingRate(h, vel));

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
                Console.WriteLine($"      t {t,5:F0}  h {h / 1000,5:F1}  V {vel,5:F0}  D {dMeas,5:F1}/{outp.DragRefMs2,5:F1}  bank {outp.BankDeg,6:F1}  s {s * R0 / 1000,6:F0}  x {cross * R0 / 1000,6:F1}  hdg {HeadingError(y, tLat, tLon) * 180 / Math.PI,6:F1}/{outp.DeadbandRad * 180 / Math.PI,4:F1}  {outp.Status}");
        }

        var (sf, cf) = RangeAndCross(y, tLat, tLon);
        double rangeKm = sf * R0 / 1000, crossKm = cf * R0 / 1000;
        double hEnd = (y[0] - R0) / 1000;
        bool ok = plan != null && rangeKm < 20 && Math.Abs(crossKm) < 15 && gMax < cfg.LoadFactorMax * 1.15 && Math.Abs(hEnd - cfg.HandoverAltitude / 1000) < 6;
        if (stats != null)
        {
            stats.MissKm = rangeKm; stats.CrossKm = crossKm; stats.Ok = ok;
            return ok ? 0 : 1;
        }
        Console.WriteLine($"  {(ok ? "ok  " : "FAIL")} {v.Name} closed loop (rho x1.15, L/D x0.92): miss {rangeKm:F1} km, cross {crossKm:F1} km, "
                        + $"h_end {hEnd:F1} km, V_end {y[3]:F0} m/s, g max {gMax:F2}, q max {qMax / 1000:F1} kPa, heating max {heatMax / 1000:F0} kW/m^2, reversals {reversals}, "
                        + $"solves {solves} ({refused} refused{(refused > 0 ? ": " + string.Join(", ", whys.Select(kv => kv.Key + " x" + kv.Value)) + $", worst defect {worstDefect:F3}" : "")}), worst {solveMsMax:F0} ms, t {t:F0} s");
        return ok ? 0 : 1;
    }
}
