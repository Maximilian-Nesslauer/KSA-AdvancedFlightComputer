#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Globalization;
using System.IO;
using System.Text;

/// <summary>
/// Per-step telemetry for entry guidance (3-DOF), written to navbox-logs as &lt;run&gt;-entry.csv beside touchdown guidance's files, so a glide that does not fly the attitude it asks for can be read back.
///  It answers one question the game log cannot: WHY the attitude flown differs from the one asked for. The error is split along the angle of attack (flown past the command, positive, is what an airframe that is unstable engine first does inside the attitude controller's deadband) and along the command's own motion (behind it, negative, is lag on a turning command), with the stock controller's error angles, deadband and RCS authority, and the aero moment the sweep predicts at the angle flown.
///  The same rules as SixDofLog: nothing here may throw, and rows are buffered and flushed on an interval so the sim step does not wait on the disk.
/// </summary>
internal static class EntryLog
{
    private const int FlushEveryRows = 50;
    private const int MaxFailedFlushes = 3;

    /// <summary>Sim seconds between rows: the re-solve cadence.</summary>
    internal const double RowIntervalS = 0.1;

    private static readonly StringBuilder _rows = new();
    private static string _path = "";
    private static int _pendingRows, _failedFlushes;
    private static double _lastRow = double.NegativeInfinity;
    private static long _lastRowTick;

    /// <summary>Wall-clock time without a row after which another craft may take the log, ms: a craft destroyed or unloaded mid-run never steps again to stop its own, and on 2026-11-18 one such booster kept every later landing that session from being recorded.</summary>
    private const long StaleOwnerMs = 5000;

    internal static bool Enabled { get; private set; }
    internal static object Owner { get; private set; }
    internal static string Path { get; private set; } = "";
    internal static string LastError { get; private set; } = "";

    /// <summary>Begin a run for this craft. One log, one craft, as SixDofLog: a second craft is not recorded while the first is, unless the first has written nothing for <see cref="StaleOwnerMs"/>.</summary>
    internal static bool Start(object owner, string vehicleName, string bodyName)
    {
        if (Enabled && Owner != null && !ReferenceEquals(Owner, owner))
        {
            if (Environment.TickCount64 - _lastRowTick < StaleOwnerMs)
                return false;
            Stop(Owner);
        }
        try
        {
            Stop(owner);
            string dir = System.IO.Path.Combine(KSA.Constants.DocumentsFolderPath, "navbox-logs");
            Directory.CreateDirectory(dir);
            string run = DateTime.Now.ToString("yyyyMMdd-HHmmss", CultureInfo.InvariantCulture);
            string name = run;
            for (int suffix = 1; File.Exists(System.IO.Path.Combine(dir, name + "-entry.csv")); suffix++)
                name = run + "-" + suffix;
            _path = System.IO.Path.Combine(dir, name + "-entry.csv");
            Path = _path;
            _rows.Clear();
            _rows.AppendLine("# navbox entry guidance run " + name + "   vehicle: " + vehicleName + "   body: " + bodyName);
            _rows.AppendLine("# accelerations in m/s^2, site frame: meas* is the measured non-gravitational acceleration, pred* the model's at the thrust axis flown, without the learned bias (predLiftBody, predSideBody: at the body axis), live* the game's own aerodynamic force on the craft as it is (KsaAeroSweep.TryLiveAero) over its mass; drag is along the airflow, lift along the thrust axis's predicted lift, side across both. meas - live is what is not aerodynamic; live - pred is the model's error");
            _rows.AppendLine("# rcsFlowKgS, rcsFiring: the RCS as fired this step, whoever commanded it (the stock flight computer or AFC's own attitude controller); ownAtt = AFC's own attitude controller flew (AttitudeController), its ac* columns in the model's axes (x, y across, z the thrust axis), N m: demand = feedback + gyroscopic - air, air its feedforward, surf what the control surfaces gave, rcsAsk/rcs what the RCS was asked for and gave, int the integral term; fins = control surfaces the controller drives, finNCmd the deflection commanded this step and finNDeg where the surface was when it was commanded, degrees (the first four); alphaRf/betaRf the air as the body meets it, retrograde-first as the aero tables read it (alpha = atan2(u_z, -u_x), beta = atan2(u_y, -u_x), u the craft's velocity through the air in body axes), degrees; rollOffset the glide's roll from the held wings-level one (UpdateGlideRoll), degrees, and rollMoment/rollMomentBest the air's rolling moment at the held roll and at the roll it heads for, N m");
            _rows.AppendLine("# angles in degrees; past = flown beyond the command along its angle of attack (+ away from the airflow); lag = flown along the command's motion (- behind it); aeroAccel = predicted aero moment / transverse inertia, rad/s^2, + turning the craft further off the airflow");
            _rows.AppendLine(string.Join(",",
                "t", "phase", "onPlan", "alt", "speed", "mach", "q",
                "alphaAsked", "alphaSent", "alphaFlown", "offAsked", "pastDeg", "sideDeg", "lagDeg", "cmdTurnDegS", "bodyRateDegS",
                "trimDeg", "trimPastDeg",
                "fcErrX", "fcErrY", "fcErrZ", "fcDeadband", "rcsAuthX", "rcsAuthY", "rcsAuthZ", "tvcAuthY", "tvcAuthZ",
                "cmFlown", "aeroMomentNm", "inertiaT", "aeroAccel", "aeroOverRcs",
                "throttle", "dragScale", "biasAccel", "refusals",
                "axisOffsetDeg", "alphaBody", "tnm", "rollDeg",
                "measDrag", "measLift", "measSide", "predDrag", "predLift", "predLiftBody", "predSideBody",
                "liveOk", "liveDrag", "liveLift", "liveSide", "liveMomentNm", "bodyAero", "rollHoldErrDeg",
                "rollRateDegS", "rollAccelDegS2", "rollIzz", "rollNetNm", "rollAirNm", "rollRcsNm", "rollResidNm", "rcsRollPosNm", "rcsRollNegNm",
                "physDtS", "pitchNetNm", "pitchIntNm", "pitchAirNm", "pitchAirDtNm", "pitchRcsNm",
                "yawNetNm", "yawIntNm", "yawAirNm", "yawAirDtNm", "yawRcsNm", "rollIntNm", "rollAirDtNm",
                "liveMxNm", "liveMyNm", "liveMzNm", "predMxNm", "predMyNm", "predMzNm", "solveWaitMs", "solveRunMs",
                "rcsFlowKgS", "rcsFiring", "ownAtt", "acErrDeg", "acRateErrDegS", "acDeadband", "acAirKnown",
                "acDemandX", "acDemandY", "acDemandZ", "acAirX", "acAirY", "acAirZ", "acSurfX", "acSurfY", "acSurfZ",
                "acRcsAskX", "acRcsAskY", "acRcsAskZ", "acRcsX", "acRcsY", "acRcsZ", "acRcsFlowKgS",
                "acIntX", "acIntY", "acIntZ", "fins", "fin1Cmd", "fin2Cmd", "fin3Cmd", "fin4Cmd", "fin1Deg", "fin2Deg", "fin3Deg", "fin4Deg",
                "alphaRfDeg", "betaRfDeg", "rollOffsetDeg", "rollMomentNm", "rollMomentBestNm", "solver"));
            _pendingRows = 0;
            _failedFlushes = 0;
            _lastRow = double.NegativeInfinity;
            _lastRowTick = Environment.TickCount64;
            LastError = "";
            Owner = owner;
            Enabled = true;
        }
        catch (Exception e)
        {
            Enabled = false;
            Owner = null;
            LastError = e.Message;
        }
        return Enabled;
    }

    internal static void Stop(object owner)
    {
        if (!Enabled || (Owner != null && !ReferenceEquals(Owner, owner)))
            return;
        Flush();
        Enabled = false;
        Owner = null;
    }

    /// <summary>Whether a row is due for this owner at sim time <paramref name="t"/>; the caller builds the row only then.</summary>
    internal static bool Due(object owner, double t)
        => Enabled && ReferenceEquals(Owner, owner) && t - _lastRow >= RowIntervalS - 1e-9;

    internal static void Row(object owner, in EntryRow r)
    {
        if (!Enabled || !ReferenceEquals(Owner, owner))
            return;
        try
        {
            _lastRow = r.T;
            _lastRowTick = Environment.TickCount64;
            var sb = _rows;
            F(sb, r.T); sb.Append(Csv(r.Phase)).Append(','); sb.Append(r.OnPlan ? 1 : 0).Append(',');
            F(sb, r.Alt); F(sb, r.Speed); F(sb, r.Mach); F(sb, r.Q);
            F(sb, r.AlphaAsked); F(sb, r.AlphaSent); F(sb, r.AlphaFlown); F(sb, r.OffAsked);
            F(sb, r.PastDeg); F(sb, r.SideDeg); F(sb, r.LagDeg); F(sb, r.CmdTurnDegS); F(sb, r.BodyRateDegS);
            F(sb, r.TrimDeg); F(sb, r.TrimPastDeg);
            F(sb, r.FcErrX); F(sb, r.FcErrY); F(sb, r.FcErrZ); F(sb, r.FcDeadband);
            F(sb, r.RcsAuthX); F(sb, r.RcsAuthY); F(sb, r.RcsAuthZ); F(sb, r.TvcAuthY); F(sb, r.TvcAuthZ);
            F(sb, r.CmFlown); F(sb, r.AeroMomentNm); F(sb, r.InertiaT); F(sb, r.AeroAccel); F(sb, r.AeroOverRcs);
            F(sb, r.Throttle); F(sb, r.DragScale); F(sb, r.BiasAccel);
            sb.Append(r.Refusals).Append(',');
            F(sb, r.AxisOffsetDeg); F(sb, r.AlphaBody); sb.Append(r.Tnm ? 1 : 0).Append(','); F(sb, r.RollDeg);
            F(sb, r.MeasDrag); F(sb, r.MeasLift); F(sb, r.MeasSide); F(sb, r.PredDrag); F(sb, r.PredLift); F(sb, r.PredLiftBody); F(sb, r.PredSideBody);
            sb.Append(r.LiveOk ? 1 : 0).Append(','); F(sb, r.LiveDrag); F(sb, r.LiveLift); F(sb, r.LiveSide); F(sb, r.LiveMomentNm);
            sb.Append(r.BodyAero ? 1 : 0).Append(','); F(sb, r.RollHoldErrDeg);
            F(sb, r.RollRateDegS); F(sb, r.RollAccelDegS2); F(sb, r.RollIzz); F(sb, r.RollNetNm); F(sb, r.RollAirNm); F(sb, r.RollRcsNm); F(sb, r.RollResidNm); F(sb, r.RcsRollPosNm); F(sb, r.RcsRollNegNm);
            F(sb, r.PhysDtS); F(sb, r.PitchNetNm); F(sb, r.PitchIntNm); F(sb, r.PitchAirNm); F(sb, r.PitchAirDtNm); F(sb, r.PitchRcsNm);
            F(sb, r.YawNetNm); F(sb, r.YawIntNm); F(sb, r.YawAirNm); F(sb, r.YawAirDtNm); F(sb, r.YawRcsNm); F(sb, r.RollIntNm); F(sb, r.RollAirDtNm);
            F(sb, r.LiveMx); F(sb, r.LiveMy); F(sb, r.LiveMz); F(sb, r.PredMx); F(sb, r.PredMy); F(sb, r.PredMz);
            F(sb, r.SolveWaitMs); F(sb, r.SolveRunMs);
            F(sb, r.RcsFlowKgS); sb.Append(r.RcsFiring).Append(',');
            sb.Append(r.OwnAtt ? 1 : 0).Append(','); F(sb, r.AcErrDeg); F(sb, r.AcRateErrDegS);
            sb.Append(r.AcDeadband ? 1 : 0).Append(','); sb.Append(r.AcAirKnown ? 1 : 0).Append(',');
            F(sb, r.AcDemand.X); F(sb, r.AcDemand.Y); F(sb, r.AcDemand.Z);
            F(sb, r.AcAir.X); F(sb, r.AcAir.Y); F(sb, r.AcAir.Z);
            F(sb, r.AcSurf.X); F(sb, r.AcSurf.Y); F(sb, r.AcSurf.Z);
            F(sb, r.AcRcsAsk.X); F(sb, r.AcRcsAsk.Y); F(sb, r.AcRcsAsk.Z);
            F(sb, r.AcRcs.X); F(sb, r.AcRcs.Y); F(sb, r.AcRcs.Z); F(sb, r.AcRcsFlowKgS);
            F(sb, r.AcInt.X); F(sb, r.AcInt.Y); F(sb, r.AcInt.Z);
            sb.Append(r.FinCommandDeg?.Length ?? 0).Append(',');
            for (int i = 0; i < 4; i++)
                F(sb, r.FinCommandDeg != null && i < r.FinCommandDeg.Length ? r.FinCommandDeg[i] : double.NaN);
            for (int i = 0; i < 4; i++)
                F(sb, r.FinNowDeg != null && i < r.FinNowDeg.Length ? r.FinNowDeg[i] : double.NaN);
            F(sb, r.AlphaRfDeg); F(sb, r.BetaRfDeg); F(sb, r.RollOffsetDeg); F(sb, r.RollMomentNm); F(sb, r.RollMomentBestNm);
            sb.Append(Csv(r.Solver)).AppendLine();
            if (++_pendingRows >= FlushEveryRows)
                Flush();
        }
        catch (Exception e)
        {
            LastError = e.Message;
        }
    }

    private static void Flush()
    {
        if (_rows.Length == 0)
            return;
        try
        {
            File.AppendAllText(_path, _rows.ToString());
            _rows.Clear();
            _pendingRows = 0;
            _failedFlushes = 0;
        }
        catch (Exception e)
        {
            _rows.Clear();
            _pendingRows = 0;
            LastError = e.Message;
            if (++_failedFlushes >= MaxFailedFlushes)
                Enabled = false;
        }
    }

    // Invariant culture and round-trip format, as SixDofLog.
    private static void F(StringBuilder sb, double v)
    {
        sb.Append(double.IsFinite(v) ? v.ToString("R", CultureInfo.InvariantCulture) : "");
        sb.Append(',');
    }

    private static string Csv(string s)
        => string.IsNullOrEmpty(s) ? "" : "\"" + s.Replace("\"", "\"\"") + "\"";

    internal struct EntryRow
    {
        public double T, Alt, Speed, Mach, Q;
        public string Phase, Solver;
        public bool OnPlan;
        public double AlphaAsked, AlphaSent, AlphaFlown, OffAsked, PastDeg, SideDeg, LagDeg, CmdTurnDegS, BodyRateDegS;
        public double TrimDeg, TrimPastDeg;
        public double FcErrX, FcErrY, FcErrZ, FcDeadband, RcsAuthX, RcsAuthY, RcsAuthZ, TvcAuthY, TvcAuthZ;
        public double CmFlown, AeroMomentNm, InertiaT, AeroAccel, AeroOverRcs;
        public double Throttle, DragScale, BiasAccel;
        public int Refusals;
        public double AxisOffsetDeg, AlphaBody, RollDeg, MeasDrag, MeasLift, MeasSide, PredDrag, PredLift, PredLiftBody, PredSideBody;
        public bool Tnm, LiveOk;
        public double LiveDrag, LiveLift, LiveSide, LiveMomentNm, RollHoldErrDeg;

        /// <summary>
        /// THE ROLL BALANCE, about the thrust axis (touchdown guidance's roll axis), each step: the measured roll rate and its change, the roll inertia touchdown guidance models, the net torque that change says the craft felt (Izz x roll accel), the game's air torque (KsaAeroSweep.TryLiveAero), the RCS's as fired (KsaRcsTorqueControl.FiredTorqueAlong), what is left over, and what the thrusters can give each way. 2026-11-20: the air's was -40 kN m against 22 kN m of RCS by both counts, yet the booster held its roll - one of these terms is wrong.
        /// </summary>
        public double RollRateDegS, RollAccelDegS2, RollIzz, RollNetNm, RollAirNm, RollRcsNm, RollResidNm, RcsRollPosNm, RcsRollNegNm;

        /// <summary>
        /// THE BALANCE ON ALL THREE AXES (model x, y = pitch, yaw; z = roll, the thrust axis), N m: the net from the measured rate change (I x d(w)/dt), the integrator's own (I x KinematicMeasurements.AngularAccelerationBody, every torque it applied), the air's as KsaAeroSweep.TryLiveAero gives it with no limiter and with the integrator's substep (physDtS), and the RCS fired. The integrator divides the air's moment by a factor of its substep (ComputeAerodynamicLoads); airDt against int - rcs says whether that is the whole difference.
        /// </summary>
        public double PhysDtS, PitchNetNm, PitchIntNm, PitchAirNm, PitchAirDtNm, PitchRcsNm,
                      YawNetNm, YawIntNm, YawAirNm, YawAirDtNm, YawRcsNm, RollIntNm, RollAirDtNm;

        /// <summary>The aerodynamic moment about the centre of mass, body axes, N m: the game's own now (KsaAeroSweep.TryLiveAero), and the body-axis tables' for the same air (KsaAeroSweep.PredictBodyMoment). NaN where there is none (set by the caller).</summary>
        public double LiveMx, LiveMy, LiveMz, PredMx, PredMy, PredMz;

        /// <summary>The last finished entry solve's wait from dispatch to starting on its thread, and its run time, ms (Ksa3DofWorker).</summary>
        public double SolveWaitMs, SolveRunMs;
        public bool BodyAero;

        /// <summary>The RCS as fired, whoever commanded it (KsaRcsTorqueControl.FiredFlow): propellant kg/s, thrusters firing.</summary>
        public double RcsFlowKgS;
        public int RcsFiring;

        /// <summary>AFC's own attitude controller (AttitudeController.Report), when it flew this step (OwnAtt); torques in the model's axes, N m.</summary>
        public bool OwnAtt, AcDeadband, AcAirKnown;
        public double AcErrDeg, AcRateErrDegS, AcRcsFlowKgS;
        public Brutal.Numerics.double3 AcDemand, AcAir, AcSurf, AcRcsAsk, AcRcs, AcInt;

        /// <summary>The control surfaces the controller drives: commanded this step, and where they were then, degrees.</summary>
        public double[] FinCommandDeg, FinNowDeg;

        /// <summary>The air as the body meets it, retrograde-first, degrees; the glide's roll offset, degrees; the air's rolling moment at the held roll and at the roll it heads for, N m.</summary>
        public double AlphaRfDeg, BetaRfDeg, RollOffsetDeg, RollMomentNm, RollMomentBestNm;
    }
}
