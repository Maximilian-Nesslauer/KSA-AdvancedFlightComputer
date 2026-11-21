namespace AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;

/// <summary>
/// Flies an entry plan: drag and sink-rate feedback on the plan's vertical L/D, bank from the measured L/D, the bank's side from a crossrange deadband, and a roll that slews through lift-up on a reversal.
///
///   u_cmd = u_ref(e) + (w^2 H / D^2)(D - D_ref(e)) - (2 zeta w / D)(hdot - hdot_ref(e))
///   |bank| = acos(u_cmd / (L/D)_measured)
///
/// The two gains put the drag error on a damped spring of natural frequency w (Shuttle entry guidance's form, derived from dD/D = -dh/H). The tracker may overbank past the plan's largest bank, to the overbank limit, for a limited time, as the Shuttle did to stop a sink after its pull-out.
///
/// Positive bank lifts to the right of the flight path. Positive crossrange means the target is to the left.
/// </summary>
public sealed class EntryTracker
{
    public enum Phase { PreEntry, Tracking, HandOver, NoPlan }

    public readonly record struct Input(
        double Dt,
        double E,                 // normalised energy now
        double DragMs2,           // measured drag acceleration
        double HdotMs,
        double LiftToDrag,        // measured
        double ScaleHeightM,      // local, from the atmosphere table
        double CrossrangeRad,     // + target to the left
        double SpeedNorm,         // normalised speed
        EntryPlan? Plan);

    public readonly record struct Output(
        Phase Phase,
        double BankDeg,           // signed, as flown (slewed)
        double BankTargetDeg,     // signed, before the slew
        double UCmd,
        double DragRefMs2,
        double HdotRefMs,
        double BankRefDeg,        // the plan's, unsigned
        double DeadbandRad,
        int Side,                 // +1 right, -1 left
        bool Reversing,
        bool Overbanked,
        bool LiftUpLimit,
        bool LiftDownLimit,
        string Status);

    readonly EntryConfig _cfg;
    double _bank;            // signed, as flown
    int _side;               // 0 until the first roll
    double _overbankS;
    bool _reversing;
    bool _inAir;

    public EntryTracker(EntryConfig cfg) => _cfg = cfg;

    public double BankDeg => _bank;
    public int Side => _side;

    /// <summary>The drag has reached the start value once: from then on the tracker flies the plan, whatever the drag does next. A lifting craft pulled up out of the thin air at its first touch (2026-11-21) and, unlatched, went back to lift-up and floated on.</summary>
    public bool InAir => _inAir;

    public void Reset()
    {
        _bank = 0;
        _side = 0;
        _overbankS = 0;
        _reversing = false;
        _inAir = false;
    }

    public Output Update(in Input inp)
    {
        var cfg = _cfg;
        double db = cfg.DeadbandC0 + cfg.DeadbandC1 * inp.SpeedNorm;

        if (inp.Plan == null)
            return Hold(inp, Phase.NoPlan, db, "NO PLAN: HOLDING BANK");

        if (inp.E >= inp.Plan.EF)
            return Hold(inp, Phase.HandOver, db, "HAND-OVER ENERGY REACHED");

        if (inp.DragMs2 >= cfg.StartDrag)
            _inAir = true;
        if (!_inAir)
        {
            Slew(0.0, inp.Dt);
            return new Output(Phase.PreEntry, _bank, 0, 0, 0, 0, 0, db, _side, false, false, false, false,
                $"PRE-ENTRY: LIFT-UP UNTIL DRAG {cfg.StartDrag:F1} m/s²");
        }

        var p = inp.Plan.At(inp.E);
        double d = Math.Max(inp.DragMs2, 1e-3);
        double w = cfg.TrackOmega, z = cfg.TrackZeta;
        double f1 = w * w * inp.ScaleHeightM / (d * d);
        double f2 = 2 * z * w / d;
        double uCmd = p.U + f1 * (inp.DragMs2 - p.DragMs2) - f2 * (inp.HdotMs - p.HdotMs);

        double ld = Math.Max(inp.LiftToDrag, 1e-3);
        double cosB = uCmd / ld;
        bool liftUp = cosB >= 1.0;
        double over = cfg.OverbankDeg * Math.PI / 180;
        bool liftDown = cosB <= Math.Cos(over);
        double bankMag = Math.Acos(Math.Clamp(cosB, Math.Cos(over), 1.0)) * 180 / Math.PI;

        // Overbank: past the plan's largest bank only for a while.
        bool overbank = bankMag > cfg.BankMaxDeg;
        if (overbank)
        {
            _overbankS += inp.Dt;
            if (_overbankS > cfg.OverbankMaxS) bankMag = cfg.BankMaxDeg;
        }
        else _overbankS = Math.Max(0, _overbankS - inp.Dt);
        bool overbankHeld = overbank && _overbankS <= cfg.OverbankMaxS;

        // Side: first roll towards the target, then reverse whenever the crossrange leaves the deadband.
        int prevSide = _side;
        if (_side == 0) _side = inp.CrossrangeRad > 0 ? -1 : 1;
        if (inp.CrossrangeRad > db) _side = -1;
        else if (inp.CrossrangeRad < -db) _side = 1;
        if (prevSide != 0 && _side != prevSide) _reversing = true;

        double target = _side * bankMag;
        Slew(target, inp.Dt);
        if (_reversing && Math.Abs(_bank - target) < 1.0) _reversing = false;

        string status;
        if (_reversing) status = $"ROLL REVERSAL TO THE {(_side > 0 ? "RIGHT" : "LEFT")}";
        else if (overbankHeld) status = $"OVERBANK {bankMag:F0}° TO STOP THE SINK ({cfg.OverbankMaxS - _overbankS:F0} s LEFT)";
        else if (overbank) status = $"OVERBANK TIMED OUT: HELD AT {cfg.BankMaxDeg:F0}°";
        else if (liftUp) status = "LIFT-UP LIMIT: NEEDS MORE LIFT THAN THE VEHICLE HAS";
        else if (liftDown) status = "LIFT-DOWN LIMIT";
        else
        {
            double err = inp.DragMs2 - p.DragMs2;
            string trend = Math.Abs(err) < 0.05 * Math.Max(p.DragMs2, 1) ? "ON PROFILE"
                         : err > 0 ? "DRAG HIGH: CLIMBING BACK" : "DRAG LOW: SINKING TO PROFILE";
            status = $"TRACKING DRAG · {trend}";
        }

        return new Output(Phase.Tracking, _bank, target, uCmd, p.DragMs2, p.HdotMs, p.BankRefDeg, db, _side,
                          _reversing, overbankHeld, liftUp, liftDown, status);
    }

    Output Hold(in Input inp, Phase phase, double db, string status)
        => new(phase, _bank, _bank, 0, 0, 0, 0, db, _side, false, false, false, false, status);

    /// <summary>Move the flown bank towards the target at the roll rate. A sign change passes through zero, so a reversal rolls through lift-up.</summary>
    void Slew(double target, double dt)
    {
        double step = _cfg.RollRateDegS * dt;
        double diff = target - _bank;
        _bank += Math.Abs(diff) <= step ? diff : Math.Sign(diff) * step;
    }
}
