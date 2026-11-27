using AdvancedFlightComputer.Features.Ui.Logic;

internal static partial class Program
{
    // The cut supervisor (#335): the landings it must cut, the ones it must leave alone, and its latch.
    private static void CutSupervisorRules()
    {
        var cfg = new CutSupervisorSettings(TargetSinkMs: 0.5, ShortfallCutM: 0.3, MaxCutHeightM: 2.0, WatchHeightM: 5.0);
        const double dt = 0.02;

        // Runs a craft from legs height h with vertical speed vz(h) for seconds; returns when it cut (or -1) and the height it cut at.
        static (double cutAtS, double cutHeightM) Fly(CutSupervisorSettings c, ref CutSupervisorState s, double h, double seconds, Func<double, double> vz, bool lit = true)
        {
            for (double t = 0.0; t < seconds; t += dt)
            {
                double v = vz(h);
                bool contact = h <= 0.0;
                CutSupervisorAction a = CutSupervisor.Step(ref s, in c, h, v, dt, lit, contact);
                if (a == CutSupervisorAction.Cut)
                    return (t, h);
                if (contact)
                    break;
                h += v * dt;
            }
            return (-1.0, double.NaN);
        }

        // The threshold: the full shortfall at the cut height, less in proportion lower down, never under 0.05 m.
        Check("the cut threshold is 0.3 m at 2 m, 0.15 m at 1 m, 0.05 m near the ground",
            Math.Abs(cfg.ShortfallToCutM(2.0) - 0.3) < 1e-12 && Math.Abs(cfg.ShortfallToCutM(1.0) - 0.15) < 1e-12
            && Math.Abs(cfg.ShortfallToCutM(0.1) - 0.05) < 1e-12 && Math.Abs(cfg.ShortfallToCutM(4.0) - 0.3) < 1e-12);

        // On the pad, engine lit, never descended: not armed, never cuts.
        var pad = new CutSupervisorState();
        Check("on the pad with the engine lit, the supervisor is not armed and never cuts",
            Fly(cfg, ref pad, 0.3, 10.0, _ => 0.0).cutAtS < 0.0 && !pad.Armed);

        // A normal landing: descending, slowing to 0.3 m/s at contact as the plans end. Never cut.
        var normal = new CutSupervisorState();
        var n = Fly(cfg, ref normal, 30.0, 30.0, h => -Math.Max(0.3, Math.Min(6.0, 0.3 + 1.5 * h)));
        Check("a normal landing, slowing to 0.3 m/s at contact, is never cut", n.cutAtS < 0.0, $"cut at {n.cutHeightM:F2} m");

        // 2026-11-27 10:21: the plan brought the craft to rest 0.5 m up. Threshold 0.075 m: cut in about 0.15 s.
        var hover05 = new CutSupervisorState { Armed = true };
        var h05 = Fly(cfg, ref hover05, 0.5, 5.0, _ => 0.0);
        Check("a hover 0.5 m up is cut after about 0.15 s", Math.Abs(h05.cutAtS - 0.15) < 0.05, $"cut after {h05.cutAtS:F2} s");

        // 14:28: legs 0.49 m up, sinking 0.2 m/s rising to 0.4 m/s; the old 0.1 m floor took 0.54 s. Now well under 0.4 s.
        var slide = new CutSupervisorState { Armed = true };
        var sl = Fly(cfg, ref slide, 0.49, 3.0, h => -(0.2 + 0.2 * (0.49 - h) / 0.16));
        Check("the 14:28 slow sink at 0.49 m is cut within 0.4 s", sl.cutAtS > 0.0 && sl.cutAtS < 0.4, $"cut after {sl.cutAtS:F2} s");

        // 14:27: the plan ended with the legs 0.47 m up and the craft sliding: cut at once, at the plan's end, whatever the shortfall.
        var ended = new CutSupervisorState { Armed = true };
        Check("a plan that ends inside the cut height is cut at once",
            CutSupervisor.Step(ref ended, in cfg, 0.47, -0.2, dt, true, false, planEnded: true) == CutSupervisorAction.Cut && ended.CutAtPlanEnd);
        var endedHigh = new CutSupervisorState { Armed = true };
        Check("a plan that ends above the cut height is not cut for it",
            CutSupervisor.Step(ref endedHigh, in cfg, 3.0, -0.2, dt, true, false, planEnded: true) != CutSupervisorAction.Cut);

        // 10:21 earlier: rest 1.9 m up. Threshold 0.285 m: cut after about 0.57 s.
        var hover19 = new CutSupervisorState { Armed = true };
        var h19 = Fly(cfg, ref hover19, 1.9, 5.0, _ => 0.0);
        Check("a hover 1.9 m up is cut after about 0.57 s", Math.Abs(h19.cutAtS - 0.57) < 0.05, $"cut after {h19.cutAtS:F2} s");

        // 10:52: legs 1.4 m up, sinking 0.24 m/s, then stopped: cut before the old rule's 0.7 s.
        var stop14 = new CutSupervisorState { Armed = true };
        var s14 = Fly(cfg, ref stop14, 1.46, 5.0, h => h > 1.33 ? -0.24 : 0.0);
        Check("the 10:52 stall at 1.4 m is cut within 0.7 s, before the old climb rule", s14.cutAtS > 0.0 && s14.cutAtS < 0.7, $"cut after {s14.cutAtS:F2} s");

        // 10:50: a hover with the legs 2.5 m up, above the cut height but inside the watch window: no cut there, but the shortfall is owed...
        var hover25 = new CutSupervisorState { Armed = true };
        var h25 = Fly(cfg, ref hover25, 2.5, 3.5, _ => 0.0);
        Check("a hover 2.5 m up is not cut there, but builds its shortfall", h25.cutAtS < 0.0 && hover25.ShortfallM > 0.55);
        // ...and as soon as it sinks into the cut height at 0.3 m/s, it is cut.
        var h25down = Fly(cfg, ref hover25, 2.5, 5.0, _ => -0.3);
        Check("a stall above the cut height is cut as it sinks into it", h25down.cutAtS > 0.0 && h25down.cutHeightM > 1.9, $"cut at {h25down.cutHeightM:F2} m");

        // Above the watch window: nothing builds.
        var hover6 = new CutSupervisorState { Armed = true };
        Check("a hover 6 m up, above the watch window, builds nothing", Fly(cfg, ref hover6, 6.0, 5.0, _ => 0.0).cutAtS < 0.0 && hover6.ShortfallM == 0.0);

        // A long stall up in the window is forgiven once the craft comes down properly: the shortfall is capped and paid back.
        var forgiven = new CutSupervisorState { Armed = true };
        Fly(cfg, ref forgiven, 4.0, 10.0, _ => 0.0);
        Check("a long stall's shortfall is capped at twice the cut height's threshold", Math.Abs(forgiven.ShortfallM - 0.6) < 1e-9, $"{forgiven.ShortfallM:F3}");

        // 10:50: a 3.5 s hover 2.5 m up, then guidance brought it down at 0.9 m/s to a soft landing. Owed, but no longer stalled: not cut.
        var recovered = new CutSupervisorState { Armed = true };
        Fly(cfg, ref recovered, 2.5, 3.5, _ => 0.0);
        var rec = Fly(cfg, ref recovered, 2.5, 10.0, h => -Math.Max(0.3, Math.Min(0.92, 0.3 + 0.5 * h)));
        Check("a stall the craft has come out of is not cut while it sinks", rec.cutAtS < 0.0 || rec.cutHeightM < 0.6, $"cut at {rec.cutHeightM:F2} m");
        Check("descending at 1.5 m/s after a stall at 4 m is not cut", Fly(cfg, ref forgiven, 4.0, 10.0, h => h > 2.0 ? -1.5 : -Math.Max(0.6, 1.5 * h / 2.0)).cutAtS < 0.0);

        // A climb builds fast: +0.5 m/s at 1 m is 1 m/s short against 0.15 m, cut in about 0.15 s.
        var climb = new CutSupervisorState { Armed = true };
        var c1 = Fly(cfg, ref climb, 1.0, 5.0, _ => 0.5);
        Check("a climb from 1 m at 0.5 m/s is cut in about 0.15 s", Math.Abs(c1.cutAtS - 0.15) < 0.05, $"cut after {c1.cutAtS:F2} s");

        // Sinking faster than the target pays the shortfall back, never below zero.
        var leaky = new CutSupervisorState { Armed = true };
        Fly(cfg, ref leaky, 3.0, 0.4, _ => 0.0);
        double built = leaky.ShortfallM;
        Fly(cfg, ref leaky, 3.0, 0.4, _ => -1.0);
        Check("sinking faster than the target pays the shortfall back", built > 0.15 && leaky.ShortfallM == 0.0, $"built {built:F3}, left {leaky.ShortfallM:F3}");

        // A steady sink slower than the target is a stall too: 0.3 m/s from 2 m is cut about 1.63 m up.
        var slow = new CutSupervisorState { Armed = true };
        var s1 = Fly(cfg, ref slow, 2.0, 10.0, _ => -0.3);
        Check("a steady 0.3 m/s sink from 2 m is cut about 1.63 m up", Math.Abs(s1.cutHeightM - 1.63) < 0.05, $"cut at {s1.cutHeightM:F2} m");

        // The latch: after a cut the engine stays off, whatever guidance does next.
        var latched = new CutSupervisorState { Armed = true };
        Fly(cfg, ref latched, 0.5, 5.0, _ => 0.0);
        Check("after a cut the supervisor holds the engine off",
            latched.Cut && CutSupervisor.Step(ref latched, in cfg, 0.4, -1.0, dt, true, false) == CutSupervisorAction.HoldOff);

        // Engine off: nothing builds.
        var off = new CutSupervisorState { Armed = true };
        Check("with the engine off the supervisor builds nothing", Fly(cfg, ref off, 0.5, 5.0, _ => 0.0, lit: false).cutAtS < 0.0 && off.ShortfallM == 0.0);

        // Contact clears the shortfall.
        var down = new CutSupervisorState { Armed = true, ShortfallM = 0.4 };
        Check("contact clears the shortfall", CutSupervisor.Step(ref down, in cfg, 0.0, 0.0, dt, true, true) == CutSupervisorAction.Idle && down.ShortfallM == 0.0);

        // The drop: from 2 m at rest, about 6.3 m/s; a climb rises first.
        Check("a free fall from 2 m at rest lands at about 6.3 m/s", Math.Abs(CutSupervisor.ImpactSpeedMs(2.0, 0.0, 9.81) - 6.26) < 0.01);
        Check("a climb rises before it falls", CutSupervisor.ImpactSpeedMs(1.0, 1.0, 9.81) > CutSupervisor.ImpactSpeedMs(1.0, 0.0, 9.81));

        ContactDepthRules();
    }

    // The contact height at an attitude (#339): a Starship's skirt, four splayed legs, one collider on the axis.
    private static void ContactDepthRules()
    {
        static (double s, double c) Tilt(double deg) => (Math.Sin(deg * Math.PI / 180.0), Math.Cos(deg * Math.PI / 180.0));
        // A skirt: one round collider on the axis, 4.5 m out, its bottom 22 m under the centre of mass.
        double[] sx = [0.0], sy = [0.0], shx = [4.5], shy = [4.5], sd = [22.0];
        Check("upright, the skirt's contact depth is its measured depth", Math.Abs(ContactDepth.DepthM(sx, sy, shx, shy, sd, 0, 0, 1) - 22.0) < 1e-12);
        var (s10, c10) = Tilt(10.0);
        double e10 = ContactDepth.ExtraDepthM(sx, sy, shx, shy, sd, s10, 0, c10);
        Check("tilted 10 deg the skirt's low edge is about 0.45 m deeper", Math.Abs(e10 - (22.0 * c10 + 4.5 * s10 - 22.0)) < 1e-9 && Math.Abs(e10 - 0.447) < 0.01, $"{e10:F3} m");
        var (s25, c25) = Tilt(2.5);
        double e25 = ContactDepth.ExtraDepthM(sx, sy, shx, shy, sd, s25 * Math.Sqrt(0.5), s25 * Math.Sqrt(0.5), c25);
        Check("a round skirt tilts the same whichever way it leans", Math.Abs(e25 - ContactDepth.ExtraDepthM(sx, sy, shx, shy, sd, s25, 0, c25)) < 1e-6, $"{e25:F3} m");
        // Four legs 5 m out, small feet: the low foot leads, as the skirt's edge does.
        double[] lx = [5, -5, 0, 0], ly = [0, 0, 5, -5], lh = [0.3, 0.3, 0.3, 0.3], ld = [20, 20, 20, 20];
        double el = ContactDepth.ExtraDepthM(lx, ly, lh, lh, ld, -s10, 0, c10);
        Check("legs tilted 10 deg: the low foot is deeper by about R sin - D (1 - cos)", Math.Abs(el - (20.0 * c10 + 5.3 * s10 - 20.0)) < 1e-9, $"{el:F3} m");
        // One collider on the axis with no width: tilting lifts its lowest point.
        double[] ox = [0.0], oh = [0.0], od = [10.0];
        Check("a point on the axis rises as the craft tilts", ContactDepth.ExtraDepthM(ox, ox, oh, oh, od, s10, 0, c10) < 0.0);
        Check("no colliders: nothing to add", ContactDepth.ExtraDepthM([], [], [], [], [], s10, 0, c10) == 0.0);
    }
}
