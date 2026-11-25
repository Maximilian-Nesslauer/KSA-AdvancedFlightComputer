#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Globalization;
using System.IO;
using System.Text;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

/// <summary>
/// HAND FLYING's log (HandFlying): one file per stretch of hand flying, navbox-logs\&lt;time&gt;-hand-flying.csv, a row every HandFlying.LogEveryS of game time. Model axes x, y across the craft, z along its thrust axis (roll): the keys, the authority the rates are shares of, the rate the target turns at and the rate the craft turns at, the target's lead, and what the controller asked of the surfaces and the RCS. Written to disk every <see cref="FlushRows"/> rows and when hand flying lets go. Main thread.
/// </summary>
internal sealed class HandFlyingLog
{
    private const int FlushRows = 40;
    private readonly string _path;
    private readonly StringBuilder _rows = new();
    private int _pending;

    private HandFlyingLog(string path) => _path = path;

    internal static HandFlyingLog Start(Vehicle vehicle, double share)
    {
        try
        {
            string dir = Path.Combine(KSA.Constants.DocumentsFolderPath, "navbox-logs");
            Directory.CreateDirectory(dir);
            string path = Path.Combine(dir, DateTime.Now.ToString("yyyyMMdd-HHmmss", CultureInfo.InvariantCulture) + "-hand-flying.csv");
            var log = new HandFlyingLog(path);
            log._rows.AppendLine($"# AFC manual intervention, craft '{vehicle.Id}', rate share at start {share:F2}");
            log._rows.AppendLine("# model axes: x, y across, z along the thrust axis (roll); keys -1/0/+1 roll, pitch, yaw in stock's control frame; deg, deg/s, deg/s2, N m");
            log._rows.AppendLine("t,keyRoll,keyPitch,keyYaw,share,spareX,spareY,spareZ,cmdX,cmdY,cmdZ,flownX,flownY,flownZ,leadDeg,errDeg,rateErrDegS,demandX,demandY,demandZ,surfX,surfY,surfZ,rcsX,rcsY,rcsZ,rcsFlowKgS,saturated,rcsAxes");
            File.WriteAllText(path, log._rows.ToString());
            log._rows.Clear();
            DefaultCategory.Log.Info($"[AFC] Hand flying log: {path}");
            return log;
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] Hand flying log could not be started: {ex.Message}");
            return null;
        }
    }

    internal void Row(double t, double3 keys, double share, double3 spareRadS2, double3 cmdRadS, double3 flownRadS, double leadDeg, AttitudeController.Report r)
    {
        const double D = 180.0 / Math.PI;
        var sb = _rows;
        void F(double v) => sb.Append(double.IsFinite(v) ? v.ToString("0.####", CultureInfo.InvariantCulture) : "").Append(',');
        F(t); F(keys.X); F(keys.Y); F(keys.Z); F(share);
        F(spareRadS2.X * D); F(spareRadS2.Y * D); F(spareRadS2.Z * D);
        F(cmdRadS.X * D); F(cmdRadS.Y * D); F(cmdRadS.Z * D);
        F(flownRadS.X * D); F(flownRadS.Y * D); F(flownRadS.Z * D);
        F(leadDeg); F(r.ErrorDeg); F(r.RateErrorDegS);
        F(r.DemandNm.X); F(r.DemandNm.Y); F(r.DemandNm.Z);
        F(r.SurfacesNm.X); F(r.SurfacesNm.Y); F(r.SurfacesNm.Z);
        F(r.RcsNm.X); F(r.RcsNm.Y); F(r.RcsNm.Z);
        F(r.RcsFlowKgS);
        sb.Append(r.Saturated ? 1 : 0).Append(',').Append(r.RcsAxes).AppendLine();
        if (++_pending >= FlushRows)
            Flush();
    }

    internal void Close() => Flush();

    private void Flush()
    {
        if (_rows.Length == 0)
            return;
        try
        {
            File.AppendAllText(_path, _rows.ToString());
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] Hand flying log could not be written: {ex.Message}");
        }
        _rows.Clear();
        _pending = 0;
    }
}
