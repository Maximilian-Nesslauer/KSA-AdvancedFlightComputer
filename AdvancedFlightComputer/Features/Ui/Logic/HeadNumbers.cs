using System;
using System.Collections.Generic;
using System.Text;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal readonly record struct HeadValue(string Text, int Rank = 0, string Short = "", PlanTone Tone = PlanTone.Plain)
{
    public string ShortText => Short.Length > 0 ? Short : Text;
}

internal readonly record struct HeadCell(string Text, float X, PlanTone Tone);

internal static class HeadNumbers
{
    public const string Gap = "   ";

    private const byte Gone = 0, ShortForm = 1, LongForm = 2;

    public static string Fit(IReadOnlyList<HeadValue>? values, Func<string, float> measure, float room)
        => string.Join(Gap, Cells(values, measure, room).ConvertAll(c => c.Text));

    public static List<HeadCell> Cells(IReadOnlyList<HeadValue>? values, Func<string, float> measure, float room)
    {
        var cells = new List<HeadCell>(values?.Count ?? 0);
        int n = values?.Count ?? 0;
        if (n == 0)
            return cells;
        Span<byte> form = n <= 16 ? stackalloc byte[16] : new byte[n];
        form = form.Slice(0, n);
        Forms(values!, measure, room, form);
        float gap = measure(Gap);
        float x = 0f;
        for (int i = 0; i < n; i++)
        {
            if (form[i] == Gone)
                continue;
            string text = form[i] == LongForm ? values![i].Text : values![i].ShortText;
            cells.Add(new HeadCell(text, x, values[i].Tone));
            x += measure(HeldWidthText(text)) + gap;
        }
        return cells;
    }

    public static float WordX(IReadOnlyList<HeadCell> cells, string word, Func<string, float> measure, float room)
    {
        if (cells.Count == 0)
            return 0f;
        HeadCell last = cells[cells.Count - 1];
        float x = last.X + measure(HeldWidthText(last.Text)) + measure(Gap);
        return x + measure(word) <= room ? x : -1f;
    }

    private static void Forms(IReadOnlyList<HeadValue> values, Func<string, float> measure, float room, Span<byte> form)
    {
        int n = values.Count;
        form.Fill(ShortForm);
        float gap = measure(Gap);
        float width = Width(values, form, measure, gap);
        while (width > room)
        {
            int last = -1;
            for (int i = 0; i < n; i++)
            {
                if (form[i] != Gone && (last < 0 || values[i].Rank >= values[last].Rank))
                    last = i;
            }
            if (last < 0)
                return;
            form[last] = Gone;
            width = Width(values, form, measure, gap);
        }

        int top = 0;
        for (int i = 0; i < n; i++)
            top = Math.Max(top, values[i].Rank);
        for (int rank = 0; rank <= top; rank++)
        {
            for (int i = 0; i < n; i++)
            {
                if (values[i].Rank != rank || form[i] != ShortForm || values[i].Short.Length == 0)
                    continue;
                form[i] = LongForm;
                if (Width(values, form, measure, gap) > room)
                    form[i] = ShortForm;
            }
        }
    }

    public static string HeldWidthText(string text)
    {
        int from = 0;
        while (from < text.Length && !char.IsAsciiDigit(text[from]))
            from++;
        int to = from;
        while (to < text.Length && char.IsAsciiDigit(text[to]))
            to++;
        int pad = 3 - (to - from);
        return from < text.Length && pad > 0 ? text.Substring(0, from) + new string('0', pad) + text.Substring(from) : text;
    }

    public static string Card(IReadOnlyList<HeadValue>? values, string shown, string card)
    {
        var text = new StringBuilder();
        for (int i = 0; i < (values?.Count ?? 0); i++)
        {
            if (shown.Contains(values![i].Text, StringComparison.Ordinal))
                continue;
            if (text.Length > 0)
                text.Append('\n');
            text.Append(values[i].Text);
        }
        if (card.Length > 0)
        {
            if (text.Length > 0)
                text.Append("\n\n");
            text.Append(card);
        }
        return text.ToString();
    }

    private static float Width(IReadOnlyList<HeadValue> values, ReadOnlySpan<byte> form, Func<string, float> measure, float gap)
    {
        float width = 0f;
        bool any = false;
        for (int i = 0; i < values.Count; i++)
        {
            if (form[i] == Gone)
                continue;
            width += measure(HeldWidthText(form[i] == LongForm ? values[i].Text : values[i].ShortText)) + (any ? gap : 0f);
            any = true;
        }
        return width;
    }
}

internal readonly record struct PlanWord(string Text, bool Warns = false, bool Good = false)
{
    public bool Shows => !string.IsNullOrEmpty(Text);
}

internal static class PlanWords
{
    public const double OnTargetKm = 1.0;

    public const double OffPlanM = 100.0;

    public static readonly PlanWord Planning = new("PLANNING");
    public static readonly PlanWord OnPlan = new("ON PLAN", Good: true);
    public static readonly PlanWord LastPlan = new("LAST PLAN");
    public static readonly PlanWord NoPlan = new("NO PLAN", Warns: true);
    public static readonly PlanWord Turning = new("SLEW TO PLAN");
    public static readonly PlanWord OnTarget = new("ON TARGET", Good: true);
    public static readonly PlanWord OffTarget = new("OFF TARGET");

    public static PlanWord Of(bool hasPlan, bool noPlanLasts, bool lastPlan = false)
        => hasPlan ? (lastPlan ? LastPlan : OnPlan) : noPlanLasts ? NoPlan : Planning;

    public static PlanWord Tracking(double offM)
        => double.IsFinite(offM) && offM > OffPlanM ? new PlanWord("OFF PLAN " + Fmt.Distance(offM)) : OnPlan;

    public static PlanWord Miss(double km)
        => !double.IsFinite(km) ? default : km <= OnTargetKm ? OnTarget : OffTarget;

    public static PlanTone MissTone(double km, bool flying) => ColorRules.MissTone(km, flying, km >= FarMissKm, OnTargetKm);

    public const double FarMissKm = 400.0;

    public static PlanWord Yours(string until) => new("MANUAL UNTIL " + until);

    public static PlanWord FlyingPlan(double toKm)
        => new(double.IsFinite(toKm) ? "AFC: PLAN TO " + Fmt.Km(toKm) : "AFC: PLAN");

    public static readonly PlanWord FlyingTurn = new("AFC: GRAVITY TURN");
}

internal enum ImpactInk { Plain, Good, Bad }

internal readonly record struct ImpactMark(string Word, ImpactInk Ink);

internal static class ImpactMarks
{
    private const double ZeroMissKm = 0.05;

    public static ImpactMark Of(double missKm, bool saysOnTarget, bool burnToCome)
    {
        if (saysOnTarget && missKm <= PlanWords.OnTargetKm)
            return new ImpactMark(PageText.OnSite, ImpactInk.Good);
        if (burnToCome)
            return new ImpactMark(PageText.LandsHereNow, ImpactInk.Plain);
        if (!double.IsFinite(missKm))
            return new ImpactMark(PageText.ImpactWord, ImpactInk.Bad);
        return new ImpactMark(missKm < ZeroMissKm ? PageText.ImpactOnSite : PageText.ImpactOff(missKm), ImpactInk.Bad);
    }
}
