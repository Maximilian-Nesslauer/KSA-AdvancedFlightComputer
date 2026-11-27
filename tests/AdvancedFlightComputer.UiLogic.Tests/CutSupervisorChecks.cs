using AdvancedFlightComputer.Features.Ui.Logic;

internal static partial class Program
{
    // The cut supervisor (#335): the landings it must cut, the ones it must leave alone, and its latch.
    private static void CutSupervisorRules()
    {
        var cfg = new CutSupervisorSettings(TargetSinkMs: 0.5, ShortfallCutM: 0.5, MaxCutHeightM: 2.0);
        const double dt = 0.02;

        // Runs a craft from legs height h at vertical speed vz for seconds, with vz(h) giving the speed at each height, and returns the first step it cut at (or -1) and where.
        static (double cutAtS, double cutHeightM, CutSupervisorState state) Fly(CutSupervisorSettings c, ref CutSupervisorState s, double h, double seconds, Func<double, double> vz, bool lit = true)
        {
            for (double t = 0.0; t < seconds; t += dt)
            {
                double v = vz(h);
                bool contact = h <= 0.0;
                CutSupervisorAction a = CutSupervisor.Step(ref s, in c, h, v, dt, lit, contact);
                if (a == CutSupervisorAction.Cut)
                    return (t, h, s);
                if (contact)
                    break;
                h += v * dt;
            }
            return (-1.0, double.NaN, s);
        }

        // On the pad, engine lit, never descended: not armed, never cuts.
        var pad = new CutSupervisorState();
        Check("on the pad with the engine lit, the supervisor is not armed and never cuts",
            Fly(cfg, ref pad, 0.3, 10.0, _ => 0.0).cutAtS < 0.0 && !pad.Armed);

        // A normal landing: descending, slowing to 0.3 m/s at contact as the plans end. Never cut.
        var normal = new CutSupervisorState();
        var n = Fly(cfg, ref normal, 30.0, 30.0, h => -Math.Max(0.3, Math.Min(6.0, 0.3 + 1.5 * h)));
        Check("a normal landing, slowing to 0.3 m/s at contact, is never cut", n.cutAtS < 0.0, $"cut at {n.cutHeightM:F2} m");

        // 2026-11-27 10:21: the plan brought the craft to rest 0.5 m up. Cut after 1 s of hover (0.5 m of shortfall at 0.5 m/s).
        var hover05 = new CutSupervisorState { Armed = true };
        var h05 = Fly(cfg, ref hover05, 0.5, 5.0, _ => 0.0);
        Check("a hover 0.5 m up is cut after about 1 s", Math.Abs(h05.cutAtS - 1.0) < 0.05, $"cut after {h05.cutAtS:F2} s");

        // 10:21 earlier: rest 1.9 m up. Inside the 2 m window: cut.
        var hover19 = new CutSupervisorState { Armed = true };
        Check("a hover 1.9 m up is cut", Fly(cfg, ref hover19, 1.9, 5.0, _ => 0.0).cutAtS > 0.0);

        // Above the window: never cut, and nothing carries into it.
        var hover25 = new CutSupervisorState { Armed = true };
        var h25 = Fly(cfg, ref hover25, 2.5, 5.0, _ => 0.0);
        Check("a hover 2.5 m up, above the window, is not cut and builds no shortfall", h25.cutAtS < 0.0 && hover25.ShortfallM == 0.0);

        // A climb builds the shortfall fast: +0.5 m/s at 1 m is 1 m/s short, cut in 0.5 s.
        var climb = new CutSupervisorState { Armed = true };
        var c1 = Fly(cfg, ref climb, 1.0, 5.0, _ => 0.5);
        Check("a climb from 1 m at 0.5 m/s is cut in about 0.5 s", Math.Abs(c1.cutAtS - 0.5) < 0.05, $"cut after {c1.cutAtS:F2} s");

        // Sinking faster than the target takes the shortfall back down, never below zero.
        var leaky = new CutSupervisorState { Armed = true };
        Fly(cfg, ref leaky, 1.5, 0.4, _ => 0.0);
        double built = leaky.ShortfallM;
        Fly(cfg, ref leaky, 1.5, 0.4, _ => -1.0);
        Check("sinking faster than the target pays the shortfall back", built > 0.15 && leaky.ShortfallM == 0.0, $"built {built:F3}, left {leaky.ShortfallM:F3}");

        // A steady sink slower than the target is a stall too: 0.3 m/s from 2 m is cut after 2.5 s, 1.25 m up.
        var slow = new CutSupervisorState { Armed = true };
        var s1 = Fly(cfg, ref slow, 2.0, 10.0, _ => -0.3);
        Check("a steady 0.3 m/s sink from 2 m is cut about 1.25 m up", Math.Abs(s1.cutHeightM - 1.25) < 0.05, $"cut at {s1.cutHeightM:F2} m");

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
    }
}
