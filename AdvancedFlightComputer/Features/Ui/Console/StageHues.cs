using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

/// <summary>The one way a page asks for the colour of a stock sequence, so one stage has one colour on LAUNCH, STAGING, MANEUVER, TRANSFER and BURNS.</summary>
internal static class StageHues
{
    private static string _craft = "";
    private static int _frame = -1;
    private static int[] _left = Array.Empty<int>();
    private static int[] _numbers = Array.Empty<int>();
    private static int[] _engines = Array.Empty<int>();
    private static float[] _dv = Array.Empty<float>();

    private static void Read(Vehicle craft)
    {
        int frame = ImGui.GetFrameCount();
        if (_frame == frame && _craft == craft.Id)
            return;
        _frame = frame;
        _craft = craft.Id;
        if (!SharedVehicleHooks.GuidanceEnabled || !GuidanceWindow.ReadStageStack(craft, out _left, out _numbers, out _dv))
        {
            _left = _numbers = Array.Empty<int>();
            _dv = Array.Empty<float>();
        }
        IReadOnlyList<SequenceInfo> sequences = MultiPassPreviewCache.GetSequenceState(craft).Sequences;
        if (_engines.Length != sequences.Count)
            _engines = new int[sequences.Count];
        for (int i = 0; i < _engines.Length; i++)
            _engines[i] = sequences[i].Number;
    }

    /// <summary>The stage hues of the stock sequence <paramref name="sequence"/> of <paramref name="craft"/>.</summary>
    public static StageHueRange Of(Vehicle craft, int sequence)
    {
        Read(craft);
        return StageNumbers.HuesOf(_left, _numbers, _engines, sequence);
    }

    /// <summary>The rows of the stock sequence <paramref name="sequence"/>, with the share of its delta-v in <paramref name="shares"/> and the hue in <paramref name="hues"/> that the row has on the bar, for a box that shows a split stage.</summary>
    public static int Rows(Vehicle craft, int sequence, Span<float> shares, Span<int> hues)
    {
        Read(craft);
        int count = 0;
        float total = 0f;
        for (int i = 0; i < _left.Length && count < shares.Length; i++)
        {
            if (_left[i] != sequence)
                continue;
            shares[count] = _dv[i];
            hues[count++] = _numbers[i];
            total += _dv[i];
        }
        for (int i = 0; i < count; i++)
            shares[i] = total > 0f ? shares[i] / total : 1f / count;
        return count;
    }
}
