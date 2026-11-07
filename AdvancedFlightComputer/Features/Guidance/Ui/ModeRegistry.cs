#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Features.Ui.Logic;
using KSA;

/// <summary>The countdown, the numbers, the tone and the status word a registered mode gives the console head while it flies. <see cref="HeightShown"/> and <see cref="ImpactShown"/> say that the row holds the height and the end of a fall, so the place under it does not.</summary>
internal readonly record struct ModeLiveReadout(double Seconds, string SecondsLabel, IReadOnlyList<HeadValue> Numbers, bool Waiting, PlanWord Word = default, bool HeightShown = false, bool ImpactShown = false);

/// <summary>
/// The game side of a guidance mode that registers later than the built-in ones. The console reads
/// it through the mode's id: <see cref="IsLive"/> fills CraftActivity.RegisteredModeLive, so the head,
/// the one stop and the page dots see the mode; <see cref="PhaseLabel"/> names its phase in the head;
/// <see cref="Commit"/> runs from the footer verb and <see cref="Stop"/> from the stop, both on the main
/// thread with the craft's state bound. <see cref="AscentSetup"/> replaces the LAUNCH method entry with
/// the same mode id, and <see cref="Diagnostics"/> adds a TUNE > DIAG view. <see cref="Readout"/> gives
/// the head a countdown and numbers, and <see cref="StopText"/> the stop outcome when it depends on
/// what the mode does now; without them the head shows the phase alone and the stop the texts of the
/// descriptor.
/// </summary>
internal sealed record GuidanceModeHooks(
    GuidanceModeId Id,
    Func<VehicleAutopilotState, bool> IsLive,
    Func<VehicleAutopilotState, string> PhaseLabel,
    Action<Vehicle, Orbit, IParentBody> Commit,
    Action<Vehicle> Stop,
    AscentMethodUi AscentSetup = null,
    ModeDiagnostics Diagnostics = null,
    Func<Vehicle, VehicleAutopilotState, ModeLiveReadout> Readout = null,
    Func<VehicleAutopilotState, (string Outcome, string Detail)> StopText = null);

public static partial class GuidanceWindow
{
    private static readonly List<GuidanceModeHooks> _modeHooks = new();

    // AFC's own modes that use the registry. They are added on the first read after a load or a reset, so a reset can never leave a flying mode without its stop.
    private static bool _afcModesRegistered;

    /// <summary>
    /// Adds a guidance mode to the console: its descriptor (from GuidanceModes.Registered) and its
    /// game-side hooks. A later call with the same id replaces both.
    /// </summary>
    internal static void RegisterMode(GuidanceModeDescriptor descriptor, GuidanceModeHooks hooks)
    {
        if (descriptor == null || hooks == null || descriptor.Id != hooks.Id)
            throw new ArgumentException("A mode registers one descriptor and one set of hooks with the same id.");

        GuidanceModes.Register(descriptor);
        _modeHooks.RemoveAll(h => h.Id == hooks.Id);
        _modeHooks.Add(hooks);

        if (hooks.AscentSetup != null)
        {
            AscentMethodUi[] methods = AscentMethodUis;
            for (int i = 0; i < methods.Length; i++)
            {
                if (methods[i].ModeId == hooks.Id)
                    methods[i] = hooks.AscentSetup;
            }
        }
        if (hooks.Diagnostics != null)
            RegisterDiagnostics(hooks.Diagnostics);
    }

    /// <summary>Registers AFC's own registry modes once after a load or a reset. Every read of the registry calls it first.</summary>
    internal static void EnsureAfcModes()
    {
        if (_afcModesRegistered)
            return;
        _afcModesRegistered = true;
        RegisterMode(EntryGuidance.Descriptor(), EntryModeHooks());
    }

    private static GuidanceModeHooks HooksOf(GuidanceModeId id)
    {
        EnsureAfcModes();
        foreach (GuidanceModeHooks hooks in _modeHooks)
        {
            if (hooks.Id == id)
                return hooks;
        }
        return null;
    }

    // The first registered mode that flies the craft of state s. A fault in a mode's own check counts as not live, so the head and the stop keep working for the built-in modes.
    private static GuidanceModeHooks LiveRegisteredMode(VehicleAutopilotState s)
    {
        if (s == null)
            return null;
        EnsureAfcModes();
        foreach (GuidanceModeHooks hooks in _modeHooks)
        {
            try
            {
                if (hooks.IsLive(s))
                    return hooks;
            }
            catch (Exception ex)
            {
                Core.LogHelper.WarnOnce($"afc-mode-live-{hooks.Id}:{ex.GetType().Name}",
                    $"[AFC] The live check of guidance mode {hooks.Id} failed: {ex.Message}");
            }
        }
        return null;
    }

    // The stop texts of a live registered mode that has them, or nulls for the texts of its descriptor.
    private static (string Outcome, string Detail) RegisteredStopText(GuidanceModeHooks hooks, VehicleAutopilotState s)
    {
        if (hooks?.StopText == null)
            return (null, null);
        try
        {
            return hooks.StopText(s);
        }
        catch (Exception ex)
        {
            Core.LogHelper.WarnOnce($"afc-mode-stoptext-{hooks.Id}:{ex.GetType().Name}",
                $"[AFC] The stop text of guidance mode {hooks.Id} failed: {ex.Message}");
            return (null, null);
        }
    }

    private static void ResetModeHooks()
    {
        _modeHooks.Clear();
        _ascentMethodUis = null;
        _afcModesRegistered = false;
        GuidanceModes.ResetToBuiltIns();
    }
}
