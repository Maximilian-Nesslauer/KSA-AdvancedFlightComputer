using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

/// <summary>The answer row says what is true now, the sentence says what the verb will do, both over the full width in a body above the stock footer, and both wrap and are never cut.</summary>
internal static class ConsoleFooter
{
    private static readonly VerbGuard _guard = new();
    private static float _lastFooterY = float.NaN;

    private static float TopGap => 4f * ImGuiHelper.InterfaceScale;

    public static void Reset()
    {
        _guard.Reset();
        _lastFooterY = float.NaN;
    }

    /// <summary>A frame with no footer has no verb, so a verb that comes back under the mouse after a hand-back counts as new (VerbGuard).</summary>
    public static void NoteNoFooter() => _guard.Update("\n", false, DateTime.UtcNow.Ticks / (double)TimeSpan.TicksPerSecond);

    public static float Width(in ConsoleLayout layout) => Math.Max(0f, layout.FooterSize.X - ConsoleStyle.FooterHorizontalPaddingPx * 2f);

    private static bool HasVerb(CommitSpec commit) => commit.Verb.Length > 0 || (commit.Secondary != null && commit.Secondary.Verb.Length > 0);

    // A footer with no verb shows a status that fits in the stock footer there, and the answer row takes one that does not.
    private static bool StatusInStockFooter(CommitSpec commit, float width)
    {
        string status = commit.AnswerStatus;
        if (HasVerb(commit) || status.Length == 0)
            return false;
        ConsoleStyle.PushValueFont();
        float w = ImGui.CalcTextSize(status).X + ImGui.GetFontSize();
        ConsoleStyle.PopFont();
        return w <= width;
    }

    private static float MarkRoom(in OwnerMark mark)
    {
        float gap = ChipGap;
        float room = ConsoleUi.SmallWidth(mark.After) + ConsoleUi.SmallWidth(mark.Before);
        if (mark.Chip.Length > 0)
        {
            room += ConsoleUi.OwnerChipWidth(Owner.Stock, mark.Chip);
            if (mark.Before.Length > 0)
                room += gap;
            if (mark.After.Length > 0)
                room += gap;
        }
        return room;
    }

    private static float ChipGap => 4f * ImGuiHelper.InterfaceScale;

    private static bool MarkOwnRow(string block, CommitSpec commit, float markRoom, float width)
        => FormRules.MarkOwnRow(block.Length > 0 ? ImGui.CalcTextSize(SentenceWrap.LastRow(ConsoleUi.BreakSentence(block, width))).X : 0f, markRoom, width, ImGui.GetStyle().ItemSpacing.X, commit.ShownOwner == Owner.StockThenAfc);

    private static float WrappedHeight(string text, float width)
        => text.Length > 0 ? ImGui.CalcTextSize(ConsoleUi.BreakSentence(text, width), false, Math.Max(1f, width)).Y : 0f;

    public static float AnswerHeight(CommitSpec commit, float width)
    {
        string status = StatusInStockFooter(commit, width) ? "" : commit.AnswerStatus;
        string outcome = commit.ShownOutcome;
        OwnerMark mark = commit.Mark;
        if (status.Length == 0 && outcome.Length == 0 && mark.IsEmpty)
            return 0f;
        ConsoleStyle.PushValueFont();
        try
        {
            float h = status.Length > 0 && outcome.Length > 0
                ? WrappedHeight(status, width) + ImGui.GetStyle().ItemSpacing.Y + MarkedBlockHeight(outcome, commit, in mark, width)
                : MarkedBlockHeight(status.Length > 0 ? status : outcome, commit, in mark, width);
            return TopGap + h;
        }
        finally
        {
            ConsoleStyle.PopFont();
        }
    }

    private static float MarkedBlockHeight(string text, CommitSpec commit, in OwnerMark mark, float width)
    {
        float room = MarkRoom(in mark);
        float h = WrappedHeight(text, width);
        if (MarkOwnRow(text, commit, room, width))
            return h + ImGui.GetStyle().ItemSpacing.Y + ImGui.GetTextLineHeight();
        return Math.Max(room > 0f ? ImGui.GetTextLineHeight() : 0f, h);
    }

    public static void DrawAnswer(CommitSpec commit)
    {
        float2 origin = ImGui.GetCursorScreenPos();
        float width = ConsoleStyle.ContentAvailWidth();
        string status = StatusInStockFooter(commit, width) ? "" : commit.AnswerStatus;
        string outcome = commit.ShownOutcome;
        OwnerMark mark = commit.Mark;
        float y = origin.Y + TopGap;
        ConsoleUi.NoteRow("AfcFooterAnswer", origin, new float2(width, ConsoleStyle.FrameHeightPx));

        ConsoleStyle.PushValueFont();
        try
        {
            float markRoom = MarkRoom(in mark);
            if (status.Length > 0 && outcome.Length > 0)
            {
                Wrapped(new float2(origin.X, y), width, status, StatusInk(commit));
                GoodOver(new float2(origin.X, y), commit.GoodWord);
                y += WrappedHeight(status, width) + ImGui.GetStyle().ItemSpacing.Y;
            }
            string block = outcome.Length > 0 ? outcome : status;
            bool ownRow = MarkOwnRow(block, commit, markRoom, width);
            float blockH = WrappedHeight(block, width);
            if (markRoom > 0f)
                DrawMark(commit, in mark, origin.X + width, ownRow ? y + blockH + ImGui.GetStyle().ItemSpacing.Y : y + Math.Max(0f, blockH - ImGui.GetTextLineHeight()), markRoom);
            if (outcome.Length > 0)
                Wrapped(new float2(origin.X, y), width, outcome, ConsoleStyle.TextLabel);
            else if (status.Length > 0)
            {
                Wrapped(new float2(origin.X, y), width, status, StatusInk(commit));
                GoodOver(new float2(origin.X, y), commit.GoodWord);
            }
        }
        finally
        {
            ConsoleStyle.PopFont();
        }
    }

    private static void DrawMark(CommitSpec commit, in OwnerMark mark, float right, float y, float room)
    {
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        float lineH = ImGui.GetTextLineHeight();
        float x = right;
        if (mark.After.Length > 0)
        {
            x -= ConsoleUi.SmallWidth(mark.After);
            PlainAt(dl, mark.After, x, y, lineH, commit.MarkDim);
            if (mark.Chip.Length > 0)
                x -= ChipGap;
        }
        if (mark.Chip.Length > 0)
        {
            x -= ConsoleUi.OwnerChipWidth(Owner.Stock, mark.Chip);
            ConsoleUi.OwnerChipAt(dl, new float2(x, y), lineH, Owner.Stock, mark.Chip, commit.MarkDim);
            if (mark.Before.Length > 0)
                x -= ChipGap;
        }
        if (mark.Before.Length > 0)
            PlainAt(dl, mark.Before, x - ConsoleUi.SmallWidth(mark.Before), y, lineH, commit.MarkDim);
        var from = new float2(right - room, y);
        ConsoleUi.NoteRow("AfcFooterMark", from, new float2(room, lineH));
        string card = PageText.FlownByCard(commit.ShownOwner);
        if (card.Length > 0 && ImGui.IsWindowHovered() && ImGui.IsMouseHoveringRect(in from, new float2(right, y + lineH)))
            ConsoleUi.Card(PageText.OnPressCard, card);
    }

    private static void PlainAt(ImDrawListPtr dl, string text, float x, float y, float lineH, bool dim)
    {
        ConsoleUi.PushSmall();
        float textY = y + (lineH - ImGui.GetFontSize()) * 0.5f;
        int afc = ColorRules.AfcWordAt(text);
        string plain = afc >= 0 ? text.Substring(0, afc) : text;
        dl.AddText(new float2(x, textY), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32), plain);
        if (afc >= 0)
            dl.AddText(new float2(x + ImGui.CalcTextSize(plain).X, textY), ConsoleStyle.ApplyAlpha(dim ? ConsoleStyle.TextMutedU32 : Palette.Afc.U32), text.Substring(afc));
        ConsoleStyle.PopFont();
    }

    private static void GoodOver(float2 at, string word)
    {
        if (word.Length > 0)
            ImGui.GetWindowDrawList().AddText(at, ConsoleStyle.ApplyAlpha(Palette.Good.U32), word);
    }

    private static float4 StatusInk(CommitSpec commit)
    {
        if (commit.AnswerInWarnInk)
            return ConsoleUi.WarnInk;
        return ActorInk(commit) is Rgb actor ? actor.F4 : ConsoleStyle.TextPrimary;
    }

    private static Rgb? ActorInk(CommitSpec commit)
    {
        if (!commit.StatusPending)
            return null;
        return commit.ShownOwner switch
        {
            Owner.Afc => Palette.Afc,
            Owner.Stock or Owner.AfcPlansStockFlies => Palette.Stock,
            _ => null,
        };
    }

    /// <summary>The pending status of ConsoleStyle.FooterStatus in the colour of what acts now, because its gold is the mark of a lit verb.</summary>
    public static void ActorStatusInStockFooter(string escaped, Rgb actor)
    {
        ConsoleLayout layout = ConsoleStyle.CurrentLayout;
        float x = ImGui.GetCursorScreenPos().X;
        ConsoleStyle.PushValueFont();
        float fontSize = ImGui.GetFontSize();
        float dot = fontSize * 0.3f;
        float midY = layout.FooterPos.Y + layout.FooterSize.Y * 0.5f;
        ImGui.GetWindowDrawList().AddCircleFilled(new float2(x + dot * 0.5f, midY), dot * 0.5f, ConsoleStyle.ApplyAlpha(actor.U32));
        ImGui.SetCursorScreenPos(new float2(x + dot + fontSize * 0.3f, layout.FooterPos.Y + (layout.FooterSize.Y - fontSize) * 0.5f));
        float4 ink = actor.F4;
        ImGui.TextColored(in ink, escaped.AsSpan());
        ConsoleStyle.PopFont();
    }

    private static void Wrapped(float2 at, float width, string text, in float4 ink)
    {
        ImGui.SetCursorScreenPos(at);
        ImGui.PushTextWrapPos(ImGui.GetCursorPosX() + Math.Max(1f, width));
        ImGui.PushStyleColor(ImGuiCol.Text, in ink);
        ImGui.Text(ConsoleUi.BreakSentence(text, width));
        ImGui.PopStyleColor();
        ImGui.PopTextWrapPos();
    }

    /// <summary>The verbs in the stock footer, or the status when there is no verb.</summary>
    public static Action? DrawCommit(CommitSpec commit)
    {
        ConsoleLayout layout = ConsoleStyle.CurrentLayout;
        float full = Width(in layout);
        CommitAction? second = commit.Secondary;
        bool hasSecond = second != null && second.Verb.Length > 0;
        bool hasPrimary = commit.Verb.Length > 0;
        double now = DateTime.UtcNow.Ticks / (double)TimeSpan.TicksPerSecond;
        bool moved = MathF.Abs(layout.FooterPos.Y - _lastFooterY) > 0.5f;
        _lastFooterY = layout.FooterPos.Y;
        _guard.Update(commit.ButtonLabel + "\n" + (second?.Verb ?? ""), ImGui.IsMouseHoveringRect(layout.FooterPos, layout.FooterPos + layout.FooterSize, clip: false), now, moved);
        bool takesClicks = _guard.TakesClicks(now);
        if (!hasPrimary && !hasSecond)
        {
            if (StatusInStockFooter(commit, full))
            {
                // ConsoleStyle.FooterStatus and FooterWarning end in ImGui.TextColored, which reads its text as a printf format, so a status such as "Calculating 40 %" must have its percent sign escaped.
                string status = ConsoleUi.EscapeFormat(commit.AnswerStatus);
                if (commit.AnswerInWarnInk)
                    ConsoleStyle.FooterWarning(status.AsSpan());
                else if (ActorInk(commit) is Rgb actor)
                    ActorStatusInStockFooter(status, actor);
                else
                    ConsoleStyle.FooterStatus(status.AsSpan(), commit.StatusPending);
            }
            return null;
        }

        float h = ConsoleWidgets.ButtonHeight;
        float spacing = ImGui.GetStyle().ItemSpacing.X;
        ConsoleStyle.FooterRightAlign(full);
        Action? result = null;
        bool lead = hasSecond && second!.Lead && second.Enabled;
        if (lead && hasPrimary)
        {
            float verbW = ConsoleWidgets.ButtonWidth(commit.ButtonLabel.AsSpan());
            if (DrawPrimary(commit, verbW, h, takesClicks) is Action act)
                result = act;
            full -= verbW + spacing;
            ImGui.SameLine(0f, spacing);
            return DrawSecond(second!, full, h, takesClicks) ?? result;
        }
        if (hasSecond)
        {
            float secondW = hasPrimary ? ConsoleWidgets.ButtonWidth(second!.Verb.AsSpan()) : full;
            result = DrawSecond(second!, secondW, h, takesClicks);
            if (!hasPrimary)
                return result;
            full -= secondW + spacing;
            ImGui.SameLine(0f, spacing);
        }
        return DrawPrimary(commit, full, h, takesClicks) ?? result;
    }

    private static Action? DrawPrimary(CommitSpec commit, float width, float h, bool takesClicks)
    {
        bool clicked;
        using (new ImGuiDisabledScope(!commit.Enabled))
            clicked = commit.Lit && _guard.MayLight
                ? ConsoleWidgets.PrimaryButton(commit.ButtonLabel.AsSpan(), "AfcFooterCommit".AsSpan(), new float2(width, h))
                : ConsoleWidgets.Button(commit.ButtonLabel.AsSpan(), "AfcFooterCommit".AsSpan(), new float2(width, h));
        ConsoleUi.NoteItem("AfcFooterCommit");
        string tooltip = commit.VerbCard;
        if (tooltip.Length > 0 && ImGui.IsItemHovered(ImGuiHoveredFlags.AllowWhenDisabled))
            ConsoleUi.Card(commit.ButtonLabel, tooltip);
        return clicked && commit.Enabled && takesClicks ? commit.Act : null;
    }

    private static Action? DrawSecond(CommitAction second, float width, float h, bool takesClicks)
    {
        var size = new float2(width, h);
        bool clicked;
        using (new ImGuiDisabledScope(!second.Enabled))
            clicked = second.Lead && !second.Quiet && second.Enabled && _guard.MayLight ? ConsoleWidgets.PrimaryButton(second.Verb.AsSpan(), "AfcFooterSecondary".AsSpan(), size)
                : second.Danger ? ConsoleWidgets.DangerButton(second.Verb.AsSpan(), "AfcFooterSecondary".AsSpan(), size)
                : ConsoleWidgets.Button(second.Verb.AsSpan(), "AfcFooterSecondary".AsSpan(), size);
        ConsoleUi.NoteItem("AfcFooterSecondary");
        string tip = second.Refusal.Length > 0 ? second.Refusal + (second.Tooltip.Length > 0 ? "\n\n" + second.Tooltip : "") : second.Tooltip;
        if (tip.Length > 0 && ImGui.IsItemHovered(ImGuiHoveredFlags.AllowWhenDisabled))
            ConsoleUi.Card(second.Verb, tip);
        return clicked && second.Enabled && takesClicks ? second.Act : null;
    }
}
