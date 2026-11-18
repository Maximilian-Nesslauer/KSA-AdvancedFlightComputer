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

    internal static bool Enabled { get; private set; }
    internal static object Owner { get; private set; }
    internal static string Path { get; private set; } = "";
    internal static string LastError { get; private set; } = "";

    /// <summary>Begin a run for this craft. One log, one craft, as SixDofLog: a second craft is not recorded while the first is.</summary>
    internal static bool Start(object owner, string vehicleName, string bodyName)
    {
        if (Enabled && Owner != null && !ReferenceEquals(Owner, owner))
            return false;
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
                "liveMxNm", "liveMyNm", "liveMzNm", "predMxNm", "predMyNm", "predMzNm", "solveWaitMs", "solveRunMs", "solver"));
            _pendingRows = 0;
            _failedFlushes = 0;
            _lastRow = double.NegativeInfinity;
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
            F(sb, r.LiveMx); F(sb, r.LiveMy); F(sb, r.LiveMz); F(sb, r.PredMx); F(sb, r.PredMy); F(sb, r.PredMz);
            F(sb, r.SolveWaitMs); F(sb, r.SolveRunMs);
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

        /// <summary>The aerodynamic moment about the centre of mass, body axes, N m: the game's own now (KsaAeroSweep.TryLiveAero), and the body-axis tables' for the same air (KsaAeroSweep.PredictBodyMoment). NaN where there is none (set by the caller).</summary>
        public double LiveMx, LiveMy, LiveMz, PredMx, PredMy, PredMz;

        /// <summary>The last finished entry solve's wait from dispatch to starting on its thread, and its run time, ms (Ksa3DofWorker).</summary>
        public double SolveWaitMs, SolveRunMs;
        public bool BodyAero;
    }
}
