#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using KSA;

/// <summary>
/// The live diagnostics a guidance mode adds under TUNE > DIAG, keyed by its mode id. A mode that
/// arrives later, such as a gravity turn ascent with its own debug view, gives its draw in
/// GuidanceModeHooks.Diagnostics and gets its own DIAG tab, labelled as it asks, without a change
/// of the TUNE page. Its tuning values go to the TUNE set of its page through the DrawTuning of its
/// AscentMethodUi entry.
/// </summary>
internal sealed record ModeDiagnostics(GuidanceModeId Id, string Label, Action<Vehicle, float> Draw);

public static partial class GuidanceWindow
{
    private static readonly List<ModeDiagnostics> _modeDiagnostics = new();
    private static GuidanceModeId? _modeDiagnosticsShown;

    /// <summary>Adds or replaces the DIAG view of a mode. The view shows while the mode's entry in GuidanceModes is available.</summary>
    internal static void RegisterDiagnostics(ModeDiagnostics diagnostics)
    {
        _modeDiagnostics.RemoveAll(d => d.Id == diagnostics.Id);
        _modeDiagnostics.Add(diagnostics);
    }

    private static readonly List<ModeDiagnostics> _availableDiagnostics = new();

    // The registered views of available modes, in the order they registered.
    private static List<ModeDiagnostics> AvailableModeDiagnostics()
    {
        _availableDiagnostics.Clear();
        foreach (ModeDiagnostics d in _modeDiagnostics)
        {
            if (GuidanceModes.Of(d.Id)?.Available ?? false)
                _availableDiagnostics.Add(d);
        }
        return _availableDiagnostics;
    }

    /// <summary>
    /// The VIEW tabs of DIAG: the built-in views in <paramref name="builtIns"/> order, then one tab per
    /// registered mode view. A pick of a built-in view sets <paramref name="diagnostics"/> and takes the
    /// selection back from a mode view. Returns the mode view to draw, or null for a built-in view.
    /// </summary>
    private static ModeDiagnostics DrawDiagnosticsTabs(string[] builtIns, ref TuneDiagnostics diagnostics)
    {
        List<ModeDiagnostics> available = AvailableModeDiagnostics();
        var labels = new string[builtIns.Length + available.Count];
        Array.Copy(builtIns, labels, builtIns.Length);
        int active = (int)diagnostics;
        ModeDiagnostics shown = null;
        for (int i = 0; i < available.Count; i++)
        {
            labels[builtIns.Length + i] = available[i].Label;
            if (available[i].Id == _modeDiagnosticsShown)
            {
                active = builtIns.Length + i;
                shown = available[i];
            }
        }
        int picked = Core.ConsoleUi.PlaceTabs("VIEW", "AfcTuneDiag", labels, active, PageText.DiagViewCards, small: true);
        if (picked >= 0 && picked < builtIns.Length)
        {
            diagnostics = (TuneDiagnostics)picked;
            _modeDiagnosticsShown = null;
            shown = null;
        }
        else if (picked >= builtIns.Length)
        {
            shown = available[picked - builtIns.Length];
            _modeDiagnosticsShown = shown.Id;
        }
        return shown;
    }

    internal static void ShowBuiltInDiagnostics() => _modeDiagnosticsShown = null;

    internal static bool ModeDiagnosticsOnScreen => _modeDiagnosticsShown != null;

    private static void ResetModeDiagnostics()
    {
        _modeDiagnostics.Clear();
        _availableDiagnostics.Clear();
        _modeDiagnosticsShown = null;
    }
}
