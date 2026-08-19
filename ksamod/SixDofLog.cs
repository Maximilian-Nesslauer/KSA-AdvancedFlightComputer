using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

/// <summary>
/// Flight telemetry for the 6-DOF guidance, written to CSV so a run can be read
/// back afterwards instead of described from memory.
///
/// Three files per run, because they answer different questions:
///
///   -cycle.csv   one row per guidance step: state, solve outcome, defect,
///                thrust demand vs capability, feasibility. The time series.
///   -plan.csv    periodic snapshots of the WHOLE planned trajectory. This is the
///                one that settles "does the vehicle loop, or does the PLAN loop" —
///                a question the cycle rows genuinely cannot answer, since they
///                only ever record where the vehicle got to.
///   -events.log  engagement, node-gate steps, handover, touchdown, and every
///                refused re-solve with its reason.
///
/// DESIGN CONSTRAINTS, both learned the hard way:
///
///   NOTHING HERE MAY THROW. This is called from the sim step and the ImGui draw,
///   and an exception escaping either one unwinds past ImGui's End() and corrupts
///   the frame — the game then reports "missing End" and names the wrong function
///   entirely. Every public entry point swallows its own errors; a broken log must
///   never break a flight.
///
///   NOTHING HERE MAY STALL. Rows accumulate in memory and are flushed on an
///   interval, so the sim thread does not wait on the disk at 10 Hz.
/// </summary>
internal static class SixDofLog
{
    private const int FlushEveryRows = 60;

    private static readonly StringBuilder _cycle = new();
    private static readonly StringBuilder _plan = new();
    private static readonly StringBuilder _events = new();
    private static string _cyclePath = "", _planPath = "", _eventsPath = "";
    private static int _pendingRows;
    private static int _cycleIndex;
    private static double _lastPlanSnapshot = double.NegativeInfinity;

    internal static bool Enabled { get; private set; }
    internal static string Directory { get; private set; } = "";
    internal static string RunName { get; private set; } = "";
    internal static int RowsWritten { get; private set; }

    /// <summary>Snapshot the full plan at most this often, in seconds of sim time.</summary>
    internal static double PlanSnapshotInterval = 1.0;

    /// <summary>
    /// The vehicle this log belongs to. One log, one craft: guidance is per-vehicle
    /// now, so a second craft engaging must not silently take the file over and a
    /// first craft disengaging must not stop the log of one still flying.
    /// </summary>
    internal static object Owner { get; private set; }

    /// <summary>
    /// Begin a run for this vehicle. Refuses if another craft's run is already open -
    /// see Owner. Returns true if this call owns the log afterwards.
    ///
    /// A refusal is not an error and is not worth surfacing: it means a booster is
    /// already being recorded and the upper stage will simply not be. Interleaving two
    /// craft in one CSV would make every column ambiguous, and the log's whole value
    /// has been that a row means one thing.
    /// </summary>
    internal static bool Start(object owner, string vehicleName, string bodyName)
    {
        if (Enabled && Owner != null && !ReferenceEquals(Owner, owner))
            return false;

        try
        {
            Stop();
            Owner = owner;

            string docs = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
            Directory = Path.Combine(docs, "My Games", "Kitten Space Agency", "navbox-logs");
            System.IO.Directory.CreateDirectory(Directory);

            RunName = DateTime.Now.ToString("yyyyMMdd-HHmmss", CultureInfo.InvariantCulture);
            _cyclePath = Path.Combine(Directory, RunName + "-cycle.csv");
            _planPath = Path.Combine(Directory, RunName + "-plan.csv");
            _eventsPath = Path.Combine(Directory, RunName + "-events.log");

            _cycle.Clear();
            _plan.Clear();
            _events.Clear();
            _cycleIndex = 0;
            _pendingRows = 0;
            RowsWritten = 0;
            _lastPlanSnapshot = double.NegativeInfinity;

            // Header names match the field order in Cycle() exactly; keeping them in
            // one place would be neater but this is read by eye as often as by tool.
            _cycle.AppendLine(string.Join(",",
                "cycle", "t", "alt", "rx", "ry", "rz", "vx", "vy", "vz", "speed",
                "qw", "qx", "qy", "qz", "tiltDeg", "wx", "wy", "wz", "mass",
                "solved", "status", "scvxIters", "accepted", "solveMs", "admm",
                "defectM", "defectLimitM", "defectChan", "defectGroup", "defectRaw", "defectNode", "qFlips",
                "anchorM", "fellBack", "escalations",
                "nodes", "sigma", "planElapsed",
                "thrustDemandN", "capabilityN", "throttle", "saturated",
                "tauX", "tauY", "tauZ", "allocX", "allocY", "allocZ", "allocSat",
                "twr", "twrMin", "ambientPa", "altToGo", "descentRate", "stopDistM",
                "glideViolM", "biasX", "biasY", "biasZ", "error"));

            _plan.AppendLine("cycle,t,node,x,y,z,vx,vy,vz,thrustN,tiltDeg");

            _events.AppendLine($"# navbox 6-DOF run {RunName}");
            _events.AppendLine($"# vehicle: {vehicleName}   body: {bodyName}");
            _events.AppendLine("# t(s)  event");

            Enabled = true;
            Event(0.0, "logging started");
        }
        catch (Exception e)
        {
            Enabled = false;
            Owner = null;
            LastError = e.Message;
        }
        return Enabled;
    }

    internal static string LastError { get; private set; } = "";

    /// <summary>Close the run. Only the owner may - see Start.</summary>
    internal static void Stop(object owner)
    {
        if (Owner != null && !ReferenceEquals(Owner, owner))
            return;
        Owner = null;
        Stop();
    }

    internal static void Stop()
    {
        if (!Enabled)
            return;
        try
        {
            Flush();
        }
        catch (Exception e)
        {
            LastError = e.Message;
        }
        Enabled = false;
    }

    internal static void Event(double t, string message)
    {
        if (!Enabled)
            return;
        try
        {
            // Invariant, like every other number here. Interpolation defaults to the
            // CURRENT culture, which writes "13,00" on a comma-decimal machine — the
            // smoke test caught exactly that, and a timestamp that reads as two
            // fields is worse than useless when correlating against the CSV.
            _events.AppendLine(
                string.Format(CultureInfo.InvariantCulture, "{0,10:F2}  {1}", t, message));
        }
        catch (Exception e)
        {
            LastError = e.Message;
        }
    }

    /// <summary>
    /// One guidance cycle. Values are passed in rather than pulled from statics so
    /// this stays a pure sink — nothing here reaches back into guidance state, so it
    /// cannot perturb what it is measuring.
    /// </summary>
    internal static void Cycle(in CycleRow r)
    {
        if (!Enabled)
            return;
        try
        {
            var sb = _cycle;
            sb.Append(_cycleIndex++).Append(',');
            F(sb, r.T); F(sb, r.Alt);
            F(sb, r.Rx); F(sb, r.Ry); F(sb, r.Rz);
            F(sb, r.Vx); F(sb, r.Vy); F(sb, r.Vz);
            F(sb, Math.Sqrt(r.Vx * r.Vx + r.Vy * r.Vy + r.Vz * r.Vz));
            F(sb, r.Qw); F(sb, r.Qx); F(sb, r.Qy); F(sb, r.Qz);
            F(sb, r.TiltDeg);
            F(sb, r.Wx); F(sb, r.Wy); F(sb, r.Wz);
            F(sb, r.Mass);
            sb.Append(r.Solved ? 1 : 0).Append(',');
            sb.Append(Csv(r.Status)).Append(',');
            sb.Append(r.ScvxIters).Append(',').Append(r.Accepted).Append(',');
            F(sb, r.SolveMs); sb.Append(r.Admm).Append(',');
            F(sb, r.DefectM); F(sb, r.DefectLimitM);
            sb.Append(Csv(r.DefectChan)).Append(',').Append(Csv(r.DefectGroup)).Append(',');
            F(sb, r.DefectRaw); sb.Append(r.DefectNode).Append(',').Append(r.QFlips).Append(','); F(sb, r.AnchorM);
            sb.Append(r.FellBack ? 1 : 0).Append(',').Append(r.Escalations).Append(',');
            sb.Append(r.Nodes).Append(',');
            F(sb, r.Sigma); F(sb, r.PlanElapsed);
            F(sb, r.ThrustDemandN); F(sb, r.CapabilityN); F(sb, r.Throttle);
            sb.Append(r.Saturated ? 1 : 0).Append(',');
            F(sb, r.TauX); F(sb, r.TauY); F(sb, r.TauZ);
            F(sb, r.AllocX); F(sb, r.AllocY); F(sb, r.AllocZ); F(sb, r.AllocSat);
            F(sb, r.Twr); F(sb, r.TwrMin); F(sb, r.AmbientPa);
            F(sb, r.AltToGo); F(sb, r.DescentRate); F(sb, r.StopDistM);
            F(sb, r.GlideViolM);
            F(sb, r.BiasX); F(sb, r.BiasY); F(sb, r.BiasZ);
            sb.Append(Csv(r.Error));
            sb.AppendLine();

            if (++_pendingRows >= FlushEveryRows)
                Flush();
        }
        catch (Exception e)
        {
            LastError = e.Message;
        }
    }

    /// <summary>
    /// Snapshot the whole planned trajectory, rate-limited. THE point of this file:
    /// a cycle row can only say where the vehicle went, so on its own it can never
    /// distinguish "the plan is a loop and the vehicle followed it" from "the plan
    /// was straight and the vehicle diverged". Those have opposite causes.
    /// </summary>
    internal static void PlanSnapshot(double t, int nodes, ReadOnlySpan<double> planX,
                                      ReadOnlySpan<double> planU)
    {
        if (!Enabled || planX.Length < nodes * 14)
            return;
        if (t - _lastPlanSnapshot < PlanSnapshotInterval)
            return;
        try
        {
            _lastPlanSnapshot = t;
            for (int k = 0; k < nodes; k++)
            {
                int i = k * 14;
                double qx = planX[i + 7], qy = planX[i + 8];
                double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
                double tilt = Math.Acos(Math.Clamp(r22, -1.0, 1.0)) * 180.0 / Math.PI;

                var sb = _plan;
                sb.Append(_cycleIndex).Append(',');
                F(sb, t);
                sb.Append(k).Append(',');
                F(sb, planX[i + 0]); F(sb, planX[i + 1]); F(sb, planX[i + 2]);
                F(sb, planX[i + 3]); F(sb, planX[i + 4]); F(sb, planX[i + 5]);
                F(sb, planU.Length > k * 4 + 2 ? planU[k * 4 + 2] : 0.0);
                F(sb, tilt, last: true);
                sb.AppendLine();
            }
        }
        catch (Exception e)
        {
            LastError = e.Message;
        }
    }

    internal static void Flush()
    {
        if (!Enabled && _cycle.Length == 0)
            return;
        try
        {
            if (_cycle.Length > 0) { File.AppendAllText(_cyclePath, _cycle.ToString()); RowsWritten += _pendingRows; _cycle.Clear(); }
            if (_plan.Length > 0) { File.AppendAllText(_planPath, _plan.ToString()); _plan.Clear(); }
            if (_events.Length > 0) { File.AppendAllText(_eventsPath, _events.ToString()); _events.Clear(); }
            _pendingRows = 0;
        }
        catch (Exception e)
        {
            LastError = e.Message;
        }
    }

    // Invariant culture throughout: a machine with a comma decimal separator would
    // otherwise write "1,234" into a comma-separated file and silently shift every
    // column right of it.
    private static void F(StringBuilder sb, double v, bool last = false)
    {
        sb.Append(double.IsFinite(v) ? v.ToString("G9", CultureInfo.InvariantCulture) : "");
        if (!last) sb.Append(',');
    }

    private static string Csv(string s)
    {
        if (string.IsNullOrEmpty(s)) return "";
        // Errors are free text and routinely contain commas.
        return "\"" + s.Replace("\"", "\"\"") + "\"";
    }

    internal struct CycleRow
    {
        public double T, Alt;
        public double Rx, Ry, Rz, Vx, Vy, Vz;
        public double Qw, Qx, Qy, Qz, TiltDeg, Wx, Wy, Wz, Mass;
        public bool Solved;
        public string Status;
        public int ScvxIters, Accepted, Admm, Escalations, Nodes;
        public double SolveMs, DefectM, DefectLimitM, AnchorM, Sigma, PlanElapsed;
        // WHERE the defect is, not just how big. DefectM is a max over all fourteen
        // state channels scaled by the POSITION scale, so it is only a distance when
        // the worst channel is a position - see Ksa6DofGuidance.LastDefectChannel.
        public string DefectChan, DefectGroup;
        public double DefectRaw;
        public int DefectNode;
        // Nodes re-expressed onto the vehicle's quaternion branch this cycle. Normally
        // zero; non-zero means the double cover was about to inject a defect that has
        // no physical meaning. See Ksa6DofGuidance.AlignQuaternionBranch.
        public int QFlips;
        public bool FellBack;
        public double ThrustDemandN, CapabilityN, Throttle;
        public bool Saturated;
        public double TauX, TauY, TauZ, AllocX, AllocY, AllocZ, AllocSat;
        public double Twr, TwrMin, AmbientPa, AltToGo, DescentRate, StopDistM, GlideViolM;
        public double BiasX, BiasY, BiasZ;
        public string Error;
    }
}
