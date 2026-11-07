using System;
using System.Collections.Generic;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Console;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Pictures;

/// <summary>One burn under a picture: its label, delta-v, burn time, whether it flies now, and the time to it in seconds, NaN when the picture does not say.</summary>
internal readonly record struct BurnBar(string Label, double DvMs, double BurnTimeS, bool Lit, double InS = double.NaN, bool FuelEnds = false);

internal static class BurnBars
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public static double LastShortMs { get; private set; }

    /// <summary>What the craft holds for the burns of the last <see cref="Draw"/>, NaN with no rows or no engine data.</summary>
    public static double LastOnBoardMs { get; private set; } = double.NaN;

    /// <summary>What the burns of the last <see cref="Draw"/> ask for together, NaN with no rows.</summary>
    public static double LastTotalMs { get; private set; } = double.NaN;

    /// <summary>The stages of a craft for the estimate, from the staged performance the multi-pass splitter uses.</summary>
    public static StageFuel[] StagesOf(Vehicle vehicle)
    {
        SequenceBurnState state = MultiPassPreviewCache.GetSequenceState(vehicle);
        var stages = new StageFuel[state.Sequences.Count];
        for (int i = 0; i < stages.Length; i++)
        {
            SequenceInfo s = state.Sequences[i];
            stages[i] = new StageFuel(s.Number, s.StartMassKg, s.FuelMassKg, s.ExhaustVelocityMs);
        }
        return stages;
    }

    public static string Draw(Vehicle vehicle, IReadOnlyList<BurnBar> bars, double[]? plannedBefore = null, bool burnFits = true)
    {
        LastShortMs = 0.0;
        LastOnBoardMs = double.NaN;
        LastTotalMs = double.NaN;
        bool has = bars.Count > 1 || (bars.Count == 1 && bars[0].DvMs >= 0.05);
        ConsoleUi.NoteShape("AfcBurnRows", has ? bars.Count : 0);
        if (!has)
            return "";
        float scale = ImGuiHelper.InterfaceScale;
        bool chained = plannedBefore != null;
        StageFuel[] all = StagesOf(vehicle);
        bool known = all.Length > 0;
        StageFuel[] stages = chained ? BurnBudget.After(all, plannedBefore!) : all;
        var dvs = new double[bars.Count];
        double maxDv = 1.0, total = 0.0;
        for (int i = 0; i < bars.Count; i++)
        {
            dvs[i] = Math.Max(0.0, bars[i].DvMs);
            maxDv = Math.Max(maxDv, dvs[i]);
            total += dvs[i];
        }
        BurnPay[] pays = BurnBudget.Pay(stages, dvs);
        double onBoard = BurnBudget.OnBoard(stages);
        LastOnBoardMs = known ? onBoard : double.NaN;
        LastTotalMs = total;

        bool short_ = known && BurnBudget.IsShort(onBoard, total);
        LastShortMs = short_ ? Math.Max(0.0, total - onBoard) : 0.0;
        if (bars.Count == 1)
        {
            BurnPay only = pays.Length > 0 ? pays[0] : default;
            string line = burnFits ? OneBurnLine(known, stages, in only) : "";
            ConsoleUi.NoteShape("AfcBurnLine", line.Length > 0 ? 1 : 0);
            if (line.Length > 0)
            {
                bool marked = PayerHues(vehicle, stages, in only, out Rgb8 hue, out Rgb8? second);
                ConsoleUi.StageSentence(line.AsSpan(), in ConsoleStyle.TextMuted, marked ? hue : null, second);
                ImGui.Spacing();
            }
            return short_ ? PageText.FuelShort(total, onBoard, chained) : "";
        }
        string payer = OnePayer(stages, pays);
        bool barsShow = payer.Length == 0 && known;
        string head = !known
            ? string.Format(Inv, "{0:F0} m/s in {1} burns, no engine data", total, bars.Count)
            : barsShow ? ""
            : PageText.PassesPaidBy(payer);
        ConsoleUi.NoteShape("AfcBurnHead", head.Length > 0 ? 1 : 0);
        if (head.Length > 0)
        {
            int paying = Array.FindIndex(pays, static p => p.FirstStage >= 0);
            Rgb8 hue = default;
            Rgb8? second = null;
            bool marked = known && paying >= 0 && PayerHues(vehicle, stages, in pays[paying], out hue, out second);
            ConsoleUi.StageSentence(head.AsSpan(), in ConsoleStyle.TextMuted, marked ? hue : null, second);
        }

        float width = ConsoleStyle.ContentAvailWidth();
        float rowH = ConsoleStyle.FrameHeightPx;
        float gap = ImGui.GetStyle().ItemSpacing.X;
        float barH = 3f * scale;
        float textH = rowH - barH;
        float labelW = 64f * scale;
        for (int i = 0; i < bars.Count; i++)
            labelW = Math.Max(labelW, ImGui.CalcTextSize(bars[i].Label).X + gap);
        float barW = Math.Max(20f * scale, width - labelW);
        ImDrawListPtr dl = ImGui.GetWindowDrawList();

        for (int i = 0; i < bars.Count; i++)
        {
            float2 min = ImGui.GetCursorScreenPos();
            BurnBar bar = bars[i];
            float textY = (textH - ImGui.GetFontSize()) * 0.5f;
            dl.AddText(min + new float2(0f, textY), bar.FuelEnds ? Palette.Warn.Solid : bar.Lit ? Palette.Afc.Solid : Palette.Text.Solid, bar.Label);

            float x0 = min.X + labelW;
            float barY = min.Y + textH;
            if (barsShow)
            {
                float len = (float)(dvs[i] / maxDv) * barW;
                dl.AddRectFilled(new float2(x0, barY), new float2(x0 + barW, barY + barH), Palette.Grid.A(0.35f), 1f * scale);
                if (len > 0f)
                    dl.AddRectFilled(new float2(x0, barY), new float2(x0 + len, barY + barH), Palette.Plan.A(bar.Lit ? 0.95f : 0.7f), 1f * scale);
            }
            string value = double.IsFinite(bar.InS) && bar.InS > 0.0
                ? Fmt.Dv(dvs[i]) + "  in " + FormatHelper.FormatDuration(bar.InS)
                : Fmt.Dv(dvs[i]);
            dl.AddText(new float2(x0, min.Y + textY), Palette.Text.Solid, value);
            float valueW = ImGui.CalcTextSize(value).X;
            if (bar.BurnTimeS > 0.5 && ImGui.IsWindowHovered(ImGuiHoveredFlags.ChildWindows) && ImGui.IsMouseHoveringRect(in min, min + new float2(width, rowH)))
                ConsoleUi.Tooltip(("Burns for " + FormatHelper.FormatDuration(bar.BurnTimeS)).AsSpan());

            BurnPay pay = i < pays.Length ? pays[i] : default;
            string note;
            Rgb colour = Palette.Dim;
            if (bar.FuelEnds)
            {
                note = PageText.PassFuelEnds;
                colour = Palette.Warn;
            }
            else if (!known)
                note = "";
            else if (pay.ShortMs > 0.5)
            {
                note = string.Format(Inv, "~{0:F0} m/s short", pay.ShortMs);
                colour = Palette.Text;
            }
            else if (pay.FirstStage >= 0)
                note = barsShow ? BurnBudget.Payer(stages, in pay) + ", ~" + Fmt.Mass(pay.PropellantKg) : "~" + Fmt.Mass(pay.PropellantKg);
            else
                note = "";
            if (note.Length > 0)
            {
                Rgb8 hue = default;
                Rgb8? second = null;
                bool marked = barsShow && pay.ShortMs <= 0.5 && !bar.FuelEnds && PayerHues(vehicle, stages, in pay, out hue, out second);
                float mark = marked ? ConsoleUi.SwatchRoom * (second != null ? 2f : 1f) : 0f;
                float room = Math.Max(0f, barW - valueW - gap - mark);
                float noteX = min.X + width - Math.Min(room, ImGui.CalcTextSize(note).X);
                if (marked)
                    ConsoleUi.DrawSwatches(dl, noteX - mark, min.Y + textH * 0.5f, hue, second);
                ConsoleUi.FitText(new float2(noteX, min.Y), room, textH, note, colour.U32);
            }
            ImGui.Dummy(new float2(width, rowH));
        }
        ImGui.Spacing();
        return short_ ? PageText.FuelShort(total, onBoard, chained) : "";
    }

    /// <summary>The hues of the stock sequences that pay a burn, the same as their blocks on the staging bar.</summary>
    public static bool PayerHues(Vehicle vehicle, StageFuel[] stages, in BurnPay pay, out Rgb8 first, out Rgb8? second)
    {
        first = default;
        second = null;
        if (pay.FirstStage < 0 || pay.FirstStage >= stages.Length)
            return false;
        StageHueRange payer = StageHues.Of(vehicle, stages[pay.FirstStage].Number);
        if (!payer.Any)
            return false;
        first = ColorRules.Stage(payer.First);
        if (pay.LastStage != pay.FirstStage && pay.LastStage < stages.Length)
        {
            StageHueRange last = StageHues.Of(vehicle, stages[pay.LastStage].Number);
            if (last.Any)
                second = ColorRules.Stage(last.First);
        }
        return true;
    }

    private static string OnePayer(StageFuel[] stages, BurnPay[] pays)
    {
        string one = "";
        foreach (BurnPay pay in pays)
        {
            if (pay.FirstStage < 0)
                continue;
            string payer = BurnBudget.Payer(stages, in pay);
            if (one.Length > 0 && payer != one)
                return "";
            one = payer;
        }
        return one;
    }

    private static string OneBurnLine(bool known, StageFuel[] stages, in BurnPay pay)
    {
        if (!known)
            return "No engine data.";
        return PageText.BurnFuelLine(pay.FirstStage >= 0 ? BurnBudget.Payer(stages, in pay) : "", pay.PropellantKg, pay.ShortMs);
    }
}
