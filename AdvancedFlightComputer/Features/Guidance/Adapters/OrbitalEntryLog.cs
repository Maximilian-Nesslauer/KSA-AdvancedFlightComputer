#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;

/// <summary>
/// Per-step telemetry for orbital entry guidance, written to navbox-logs beside entry guidance's and touchdown guidance's files, under the same switch (EntryLogging):
///   &lt;run&gt;-orbital-entry.csv        one row every 0.5 s: where the craft is against the site and the corridor, what the tracker asked and why, what the attitude flew, and the plan it flew on
///   &lt;run&gt;-orbital-entry-plans.csv  every plan the solver published, node by node, so a row's plan can be drawn and set against what was flown
/// The same rules as EntryLog: nothing here may throw, one craft at a time, rows buffered and flushed on an interval.
/// </summary>
internal static class OrbitalEntryLog
{
    private const int FlushEveryRows = 40;
    internal const double RowIntervalS = 0.5;
    private const long StaleOwnerMs = 5000;

    private static readonly StringBuilder _rows = new();
    private static readonly StringBuilder _plans = new();
    private static string _path = "", _plansPath = "";
    private static int _pendingRows;
    private static double _lastRow = double.NegativeInfinity;
    private static long _lastRowTick;

    internal static bool Enabled { get; private set; }
    internal static object Owner { get; private set; }
    internal static string Path { get; private set; } = "";

    internal static readonly string[] Columns =
    {
        "t", "phase", "alt", "speed", "mach", "gammaDeg", "q",
        "energy", "energyF", "rangeKm", "downKm", "crossKm", "headingErrDeg", "reversalBandDeg",
        "rangeShortKm", "rangeLongKm", "planRangeKm", "reach",
        "drag", "dragRef", "dragMax", "dragMin", "dragModel", "hdot", "hdotRef",
        "ldMeas", "ldModel", "ldScale", "rhoScale",
        "uRef", "uCmd", "bankRef", "bankTarget", "bankCmd", "bankFlown", "side", "reversing", "overbank", "liftUp", "liftDown",
        "alphaCmd", "alphaFlown", "attErrDeg", "attRateErrDegS", "ownAtt", "airKnown",
        "demandX", "demandY", "demandZ", "airX", "airY", "airZ", "surfX", "surfY", "surfZ", "rcsX", "rcsY", "rcsZ", "rcsAskX", "rcsAskY", "rcsAskZ", "rcsFlowKgS", "spareAccXDegS2", "spareAccYDegS2", "spareAccZDegS2", "bankAccDegS2", "saturated", "rcsAxes", "rcsFiring", "bfPitchDeg", "bfBankDeg", "bfYawDeg", "bfPlanMissM", "bfPlanEndS", "bfFallKm", "bfForceScale", "bfCmdPitchDeg", "bfCmdBankDeg", "bfCmdYawDeg",
        "fins", "fin1Cmd", "fin2Cmd", "fin3Cmd", "fin4Cmd", "fin1Deg", "fin2Deg", "fin3Deg", "fin4Deg",
        "plan", "planConverged", "planAgeS", "planMissKm", "planDefect", "planIter", "planMs", "solves", "refusals", "tablesReady",
        "status",
    };

    internal static bool Start(object owner, string vehicleName, string bodyName, string setup)
    {
        // The newest start takes the log: an entry is started by the player, one at a time, and a reload leaves the old craft's state the owner with a row under StaleOwnerMs old (2026-11-23: a start 4 s after a reload was not logged).
        if (Enabled && Owner != null && !ReferenceEquals(Owner, owner))
        {
            Stop(Owner);
        }
        try
        {
            Stop(owner);
            string dir = System.IO.Path.Combine(KSA.Constants.DocumentsFolderPath, "navbox-logs");
            Directory.CreateDirectory(dir);
            string run = DateTime.Now.ToString("yyyyMMdd-HHmmss", CultureInfo.InvariantCulture);
            string name = run;
            for (int suffix = 1; File.Exists(System.IO.Path.Combine(dir, name + "-orbital-entry.csv")); suffix++)
                name = run + "-" + suffix;
            _path = System.IO.Path.Combine(dir, name + "-orbital-entry.csv");
            _plansPath = System.IO.Path.Combine(dir, name + "-orbital-entry-plans.csv");
            Path = _path;
            _rows.Clear();
            _rows.AppendLine("# navbox orbital entry run " + name + "   vehicle: " + vehicleName + "   body: " + bodyName);
            _rows.AppendLine("# " + setup);
            _rows.AppendLine("# SI unless named; angles deg. range = great circle to the site, down/cross its split along and across the ground track (cross + = site to the left), rangeShort/Long = the corridor at this energy: the range left flying the most drag the limits allow / the least the craft can hold (the TRAJ lines). reach: 0 inside, +1 site too close (high energy), -1 too far (low energy)");
            _rows.AppendLine("# drag measured from the game's own aero force; dragModel = the planner's model at this state (air x1); bank + = lift to the right; *Flown read off the craft's body axes; demand/air/surf/rcs = AFC's attitude controller, body axes, N m; fins: commanded and present deflections, the first four");
            _rows.AppendLine(string.Join(",", Columns));
            _plans.Clear();
            _plans.AppendLine("# plans of run " + name + ": one block per published plan; s = range to go at the node, short/long = the corridor");
            _plans.AppendLine("solve,t,converged,k,e,speed,alt,gammaDeg,sKm,u,bankRef,drag,dragMax,dragMin,floorAlt,shortKm,longKm,hdot,mach,ld");
            _pendingRows = 0;
            _lastRow = double.NegativeInfinity;
            _lastRowTick = Environment.TickCount64;
            Owner = owner;
            Enabled = true;
        }
        catch (Exception)
        {
            Enabled = false;
            Owner = null;
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

    internal static bool Due(object owner, double t)
        => Enabled && ReferenceEquals(Owner, owner) && t - _lastRow >= RowIntervalS - 1e-9;

    /// <summary>One row: the values in <see cref="Columns"/>' order, the last (status) as text.</summary>
    internal static void Row(object owner, double t, List<double> values, string status)
    {
        if (!Enabled || !ReferenceEquals(Owner, owner))
            return;
        try
        {
            _lastRow = t;
            _lastRowTick = Environment.TickCount64;
            foreach (double v in values)
                F(_rows, v);
            _rows.Append(Csv(status)).AppendLine();
            if (++_pendingRows >= FlushEveryRows)
                Flush();
        }
        catch (Exception)
        {
        }
    }

    /// <summary>A plan the solver published, every node.</summary>
    internal static void Plan(object owner, int solve, double t, EntryPlan p)
    {
        if (!Enabled || !ReferenceEquals(Owner, owner) || p == null)
            return;
        try
        {
            for (int k = 0; k < p.Count; k++)
            {
                var sb = _plans;
                sb.Append(solve).Append(',');
                F(sb, t); sb.Append(p.Converged ? 1 : 0).Append(',').Append(k).Append(',');
                F(sb, p.E[k]); F(sb, p.SpeedMs[k]); F(sb, p.AltitudeM[k]); F(sb, p.Gamma[k] * 180 / Math.PI);
                F(sb, p.S[k] * p.BodyRadius / 1000); F(sb, p.U[k]); F(sb, p.At(p.E[k]).BankRefDeg);
                F(sb, p.DragMs2[k]); F(sb, p.DragMaxMs2[k]); F(sb, p.DragMinMs2[k]); F(sb, p.FloorAltitudeM[k]);
                F(sb, p.RangeShortM[k] / 1000); F(sb, p.RangeLongM[k] / 1000); F(sb, p.HdotMs[k]); F(sb, p.Mach[k]);
                sb.Append(p.LiftToDrag[k].ToString("G6", CultureInfo.InvariantCulture)).AppendLine();
            }
        }
        catch (Exception)
        {
        }
    }

    private static void Flush()
    {
        try
        {
            if (_rows.Length > 0 && _path.Length > 0)
            {
                File.AppendAllText(_path, _rows.ToString());
                _rows.Clear();
            }
            if (_plans.Length > 0 && _plansPath.Length > 0)
            {
                File.AppendAllText(_plansPath, _plans.ToString());
                _plans.Clear();
            }
            _pendingRows = 0;
        }
        catch (Exception)
        {
        }
    }

    private static void F(StringBuilder sb, double v)
    {
        if (double.IsFinite(v))
            sb.Append(v.ToString("G6", CultureInfo.InvariantCulture));
        sb.Append(',');
    }

    private static readonly char[] CsvSpecial = [',', '"', '\n'];

    private static string Csv(string s)
        => s.IndexOfAny(CsvSpecial) >= 0 ? "\"" + s.Replace("\"", "\"\"") + "\"" : s;
}
