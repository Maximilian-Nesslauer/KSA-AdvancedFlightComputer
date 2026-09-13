#nullable disable

using System;
using System.Text;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// The guidance feature's log lines. Debug lines are gated on <see cref="DebugConfig.Guidance"/>
/// like every other feature's, and Info lines always print. Every line names the craft, so a log
/// with several guided vehicles still reads.
/// </summary>
internal static class GuidanceLog
{
    private const double G0 = 9.80665;

    internal static bool Enabled => DebugConfig.Guidance;

    internal static void Debug(Vehicle vehicle, string message)
    {
        if (DebugConfig.Guidance)
            DefaultCategory.Log.Debug($"[AFC] Guidance '{vehicle?.Id}': {message}");
    }

    internal static void Info(Vehicle vehicle, string message)
        => DefaultCategory.Log.Info($"[AFC] Guidance '{vehicle?.Id}': {message}");

    /// <summary>
    /// The stage list the way the panel's staging bar shows it, one entry per stage with its
    /// delta-v, burn time, thrust and provenance.
    /// </summary>
    internal static string DescribeStages(UpfgVehicle model)
    {
        if (model == null || model.Stages.Count == 0)
            return "no stages";

        var text = new StringBuilder();
        for (int i = 0; i < model.Stages.Count; i++)
        {
            UpfgStage stage = model.Stages[i];
            double ve = stage.Isp * G0;
            double dv = stage.MassTotal > 0.0 && stage.MassDry > 0.0
                ? ve * Math.Log(stage.MassTotal / stage.MassDry) : 0.0;
            double burn = stage.Mode == 2
                ? dv / (stage.GLim * G0)
                : (stage.MassTotal - stage.MassDry) / (stage.Thrust / ve);
            if (i > 0)
                text.Append("; ");
            text.Append($"S{i + 1} {dv:F0} m/s {burn:F0} s {stage.Thrust / 1000.0:F0} kN Isp {stage.Isp:F0} s "
                      + $"{stage.MassTotal / 1000.0:F1}->{stage.MassDry / 1000.0:F1} t "
                      + (stage.Seq < 0 ? "live engines" : $"seq {stage.Seq} {stage.Engines} eng")
                      + (stage.Mode == 2 ? " G" : ""));
        }
        return text.ToString();
    }

    /// <summary>
    /// A coarse fingerprint of the stage list: its length and each stage's thrust and Isp in
    /// steps of five percent. Propellant draining does not change it, so a log line keyed on it
    /// fires when the staging changes shape and not on every refresh.
    /// </summary>
    internal static string StageSignature(UpfgVehicle model)
    {
        if (model == null || model.Stages.Count == 0)
            return "none";

        var text = new StringBuilder();
        for (int i = 0; i < model.Stages.Count; i++)
        {
            UpfgStage stage = model.Stages[i];
            text.Append(stage.Seq).Append(':').Append(stage.Engines).Append(':')
                .Append(Bucket(stage.Thrust)).Append(':').Append(Bucket(stage.Isp)).Append(':')
                .Append(stage.Mode).Append(' ');
        }
        return text.ToString();
    }

    private static long Bucket(double value) =>
        value > 0.0 ? (long)Math.Round(Math.Log(value) / Math.Log(1.05)) : 0;
}
