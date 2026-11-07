using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.ImGuiApi.Internal;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Label-and-control rows built on the game's ConsoleWidgets, mirroring the private row helpers
/// stock uses in TransferPlanner. The mod draws inside stock windows, so raw ImGui would render at
/// a different size, colour and column position than everything around it.
/// </summary>
internal static class ConsoleUi
{
    public static bool LastItemDisabled => ((int)ImGui.Internal.GetItemFlags() & (int)ImGuiItemFlagsPrivate.Disabled) != 0;

    public const float TitleScale = 1.25f;

    public const float AnswerScale = 1.6f;

    /// <summary>Units, ticks and legends, the stock signature font.</summary>
    public static void PushSmall() => ConsoleStyle.PushSignatureFont();

    /// <summary>Rows, tabs and sentences, the stock label font.</summary>
    public static void PushRow() => ConsoleStyle.PushLabelFont();

    /// <summary>ConsoleStyle.PushLabelFont(float) asks ImGui for the interface font at that size, and ImGui bakes the size on demand, so this needs no registered font.</summary>
    public static void PushTitle() => ConsoleStyle.PushLabelFont(TitleScale);

    public static void PushAnswer() => ConsoleStyle.PushLabelFont(AnswerScale);

    public static float LabelWidth(string text)
    {
        ConsoleStyle.PushLabelFont();
        return WidthThenPop(text);
    }

    public static float ValueWidth(string text)
    {
        ConsoleStyle.PushValueFont();
        return WidthThenPop(text);
    }

    public static float SmallWidth(string text)
    {
        PushSmall();
        return WidthThenPop(text);
    }

    private static float WidthThenPop(string text)
    {
        float w = ImGui.CalcTextSize(text).X;
        ConsoleStyle.PopFont();
        return w;
    }

    /// <summary>The window or one of its children is under the mouse and the mouse is inside the box, read at the time of the call.</summary>
    public static bool Hovering(float2 min, float2 max) => ImGui.IsWindowHovered(ImGuiHoveredFlags.ChildWindows) && ImGui.IsMouseHoveringRect(in min, in max);

    /// <summary>Stock has no such colour: its ConsoleStyle.Pending is the gold of the verb.</summary>
    public static float4 WarnInk => Palette.Warn.F4;

    /// <summary>Set when the pilot picked a tab or a choice or opened or closed a fold in this frame, because the page may now need less room.</summary>
    public static bool ShapeChanged;

    private static readonly Dictionary<string, int> _shapes = new();

    public static void NoteShape(string id, int state)
    {
        if (_shapes.TryGetValue(id, out int last) && last != state)
            ShapeChanged = true;
        _shapes[id] = state;
    }

    /// <summary>Lets a test read where a named row or control stands in this frame, in screen pixels, called as it draws.</summary>
    internal static Action<string, float2, float2>? RowProbe;

    public static void NoteRow(string name, float2 min, float2 size) => RowProbe?.Invoke(name, min, min + size);

    public static void NoteRow(string id, int index, float2 min, float2 size)
    {
        if (RowProbe != null)
            RowProbe(id + "#" + index, min, min + size);
    }

    public static void NoteItem(string name)
    {
        if (RowProbe != null)
            RowProbe(name, ImGui.GetItemRectMin(), ImGui.GetItemRectMax());
    }

    /// <summary>The vertical item spacing of a page body: the gap between two rows, so the row pitch is the frame height and this gap.</summary>
    public static float RowGapPx => ConsoleWidgets.MinRowGapPx;

    private static float SectionGapPx => 8f * ImGuiHelper.InterfaceScale;

    public static float FieldWidth => 110f * ImGuiHelper.InterfaceScale;

    public static void Gap() => ImGui.Dummy(new float2(0f, 4f * ImGuiHelper.InterfaceScale));

    // Plain ImGui.TextWrapped wraps at the edge of the body, past the stock scrollbar gutter.
    private static void PushWrapAtEdge() => ImGui.PushTextWrapPos(ImGui.GetCursorPosX() + Math.Max(1f, ConsoleStyle.ContentAvailWidth()));

    /// <summary>A sentence with a line end at each break for a row of <paramref name="width"/> in the font pushed now (SentenceWrap), so no decimal breaks and no word stands alone.</summary>
    public static string BreakSentence(string text, float width) => SentenceWrap.Break(text, Math.Max(1f, width - 1f), MeasureText);

    public static void Sentence(ReadOnlySpan<char> text, in float4 ink)
    {
        ConsoleStyle.PushValueFont();
        PushWrapAtEdge();
        ImGui.PushStyleColor(ImGuiCol.Text, in ink);
        ImGui.Text(BreakSentence(text.ToString(), ConsoleStyle.ContentAvailWidth()).AsSpan());
        ImGui.PopStyleColor();
        ImGui.PopTextWrapPos();
        ConsoleStyle.PopFont();
    }

    public static void SentenceRoom(string text)
    {
        ConsoleStyle.PushValueFont();
        float h = ImGui.CalcTextSize(BreakSentence(text, ConsoleStyle.ContentAvailWidth())).Y;
        ConsoleStyle.PopFont();
        ImGui.Dummy(new float2(0f, h));
    }

    public static void Sentence(string text, in float4 ink, string goodWord)
    {
        float2 at = ImGui.GetCursorScreenPos();
        Sentence(text.AsSpan(), in ink);
        if (goodWord.Length == 0)
            return;
        ConsoleStyle.PushValueFont();
        ImGui.GetWindowDrawList().AddText(at, ConsoleStyle.ApplyAlpha(Palette.Good.U32), goodWord);
        ConsoleStyle.PopFont();
    }

    public static void StageSentence(ReadOnlySpan<char> text, in float4 ink, Rgb8? first, Rgb8? second = null)
    {
        if (first == null)
        {
            Sentence(text, in ink);
            return;
        }
        float2 at = ImGui.GetCursorScreenPos();
        ConsoleStyle.PushValueFont();
        float mid = at.Y + ImGui.GetFontSize() * 0.5f;
        ConsoleStyle.PopFont();
        float x = at.X + DrawSwatches(ImGui.GetWindowDrawList(), at.X, mid, first, second);
        ImGui.SetCursorScreenPos(new float2(x, at.Y));
        Sentence(text, in ink);
    }

    private static float SwatchSide => 8f * ImGuiHelper.InterfaceScale;

    public static float SwatchRoom => SwatchSide + 4f * ImGuiHelper.InterfaceScale;

    public static float DrawSwatches(ImDrawListPtr dl, float x, float midY, Rgb8? first, Rgb8? second = null)
    {
        float room = 0f;
        if (first is Rgb8 one)
            room += OneSwatch(dl, x + room, midY, one);
        if (second is Rgb8 two)
            room += OneSwatch(dl, x + room, midY, two);
        return room;
    }

    private static float OneSwatch(ImDrawListPtr dl, float x, float midY, Rgb8 hue)
    {
        dl.AddRectFilled(new float2(x, midY - SwatchSide * 0.5f), new float2(x + SwatchSide, midY + SwatchSide * 0.5f), ConsoleStyle.ApplyAlpha(((Rgb)hue).U32), 1f);
        return SwatchRoom;
    }

    public static void Muted(ReadOnlySpan<char> text) => Sentence(text, in ConsoleStyle.TextMuted);

    public static void Positive(ReadOnlySpan<char> text)
    {
        ConsoleStyle.PushValueFont();
        ImGui.TextColored(in ConsoleStyle.Positive, text);
        ConsoleStyle.PopFont();
    }

    public static void Warning(ReadOnlySpan<char> text) => Sentence(text, WarnInk);

    public static void DangerWrapped(string text) => FormattedWrapped(text, in ConsoleStyle.Danger);

    public static string PageWarning { get; private set; } = "";

    public static void ClearPageWarning() => PageWarning = "";

    public static void WarningWrapped(string text)
    {
        if (PageWarning.Length == 0)
            PageWarning = text.Replace("%%", "%");
        FormattedWrapped(text, WarnInk);
    }

    public static void AdviceWrapped(string text) => FormattedWrapped(text, WarnInk);

    // ImGui.TextWrapped keeps a wrap position that is already pushed, and it reads the text as a printf format, so a caller escapes a percent sign.
    private static void FormattedWrapped(string text, in float4 ink)
    {
        PushWrapAtEdge();
        ImGui.PushStyleColor(ImGuiCol.Text, in ink);
        ImGui.TextWrapped(BreakSentence(text, ConsoleStyle.ContentAvailWidth()));
        ImGui.PopStyleColor();
        ImGui.PopTextWrapPos();
    }

    public static void TickedNote(string text, string card = "")
    {
        float2 start = ImGui.GetCursorScreenPos();
        float tick = ConsoleStyle.ModifiedTickThicknessPx;
        ImGui.SetCursorScreenPos(new float2(start.X + tick + 4f * ImGuiHelper.InterfaceScale, start.Y));
        FormattedWrapped(text, in ConsoleStyle.TextLabel);
        if (card.Length > 0 && ImGui.IsItemHovered())
            Tooltip(card.AsSpan());
        ImGui.GetWindowDrawList().AddRectFilled(start, new float2(start.X + tick, ImGui.GetItemRectMax().Y), ConsoleStyle.ApplyAlpha(Palette.Warn.U32));
    }

    public static void MutedWrapped(string text) => FormattedWrapped(text, in ConsoleStyle.TextMuted);

    public static void MutedNote(string text, string card)
    {
        FormattedWrapped(text, in ConsoleStyle.TextMuted);
        if (card.Length > 0 && ImGui.IsItemHovered())
            Tooltip(card.AsSpan());
    }

    /// <summary>ConsoleStyle.ApplyAlpha and ImGui both read this alpha.</summary>
    public static DimScope Dim(bool on) => new(on);

    private static int _dimDepth;

    public readonly struct DimScope : IDisposable
    {
        public const float Alpha = 0.55f;
        private readonly bool _on;

        public DimScope(bool on)
        {
            _on = on;
            if (!on)
                return;
            ImGui.PushStyleVar(ImGuiStyleVar.Alpha, ImGui.GetStyle().Alpha * Alpha);
            _dimDepth++;
        }

        public void Dispose()
        {
            if (!_on)
                return;
            ImGui.PopStyleVar();
            _dimDepth--;
        }
    }

    /// <summary>A section head: the layout of the stock ConsoleWidgets.RegionHeader, a line of capitals with a hairline after it, in the primary text colour.</summary>
    public static void SectionHead(string label, string detail = "") => Head(label, detail, null, out _);

    public static bool SectionHeadButton(string label, string detail, string verb, string id, string tip)
    {
        float width = Head(label, detail, verb, out float buttonW);
        float h = ConsoleStyle.FrameHeightPx;
        float2 at = ImGui.GetCursorScreenPos();
        ImGui.SetCursorScreenPos(new float2(at.X + width - buttonW, at.Y));
        bool hit = ConsoleWidgets.Button(verb.AsSpan(), id.AsSpan(), default);
        if (tip.Length > 0 && ImGui.IsItemHovered())
            Tooltip(tip.AsSpan());
        ImGui.SetCursorScreenPos(at);
        ImGui.Dummy(new float2(width, h));
        return hit;
    }

    // Without a verb the capitals are a text item; with one they are drawn on the frame height and the cursor stays at the head for the button.
    private static float Head(string label, string detail, string? verb, out float buttonW)
    {
        ConsoleStyle.EnsureInitialized();
        ImGui.Dummy(new float2(0f, SectionGapPx));
        string text = label.ToUpperInvariant();
        float width = ConsoleStyle.ContentAvailWidth();
        float h = ConsoleStyle.FrameHeightPx;
        float2 at = ImGui.GetCursorScreenPos();
        buttonW = verb != null ? ConsoleWidgets.ButtonWidth(verb.AsSpan()) : 0f;
        ConsoleStyle.PushLabelFont();
        float fontSize = ImGui.GetFontSize();
        float textW = ImGui.CalcTextSize(text).X;
        if (verb != null)
            ImGui.GetWindowDrawList().AddText(new float2(at.X, at.Y + (h - fontSize) * 0.5f), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextPrimaryU32), text);
        else
            ImGui.TextColored(in ConsoleStyle.TextPrimary, EscapeFormat(text));
        ConsoleStyle.PopFont();
        if (verb != null)
            HeadTail(at.X + textW, at.Y + h * 0.5f, at.X + width - buttonW - ConsoleWidgets.RegionHeaderRuleGapPx, detail);
        else
            HeadTail(at.X + textW, at.Y + fontSize * 0.5f, at.X + width, detail);
        return width;
    }

    private static void HeadTail(float textEnd, float midY, float to, string detail)
    {
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        float gap = ConsoleWidgets.RegionHeaderRuleGapPx;
        float from = textEnd + gap;
        if (detail.Length > 0)
        {
            PushSmall();
            string shown = FitEllipsis(detail, to - from);
            if (shown.Length > 0)
            {
                dl.AddText(new float2(from, midY - ImGui.GetFontSize() * 0.5f), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32), shown);
                from += ImGui.CalcTextSize(shown).X + gap;
            }
            ConsoleStyle.PopFont();
        }
        if (to > from)
            dl.AddLine(new float2(from, midY), new float2(to, midY), ConsoleStyle.ApplyAlpha(ConsoleStyle.HairlineU32), ConsoleStyle.WindowBorderThicknessPx);
    }

    private static readonly Dictionary<string, int> _sectionRows = new();
    private static string _sectionKey = "";
    private static string? _sectionHead;
    private static int _sectionCount;

    public static long RowsDrawn { get; private set; }

    /// <summary>The number of rows is known only after they drew, so the head follows the count of the last frame.</summary>
    public static void BeginSection(string key, string? head)
    {
        EndSection();
        _sectionKey = key;
        _sectionHead = head;
        _sectionCount = 0;
    }

    public static void EndSection()
    {
        if (_sectionKey.Length > 0)
            _sectionRows[_sectionKey] = _sectionCount;
        _sectionKey = "";
        _sectionHead = null;
    }

    /// <summary>Every row helper here calls it; content a section draws without them calls it first.</summary>
    public static void SectionRow()
    {
        RowsDrawn++;
        if (_sectionKey.Length == 0)
            return;
        _sectionCount++;
        if (_sectionHead == null)
            return;
        string head = _sectionHead;
        _sectionHead = null;
        if (_keepSectionHead || !_sectionRows.TryGetValue(_sectionKey, out int last) || last > 1)
            SectionHead(head);
    }

    private static bool _keepSectionHead;

    /// <summary>While <paramref name="on"/>, a section keeps its head with one row too, for a part of a fold that the label of the fold names, because its row would otherwise read as a row of the part before it.</summary>
    public static void KeepSectionHead(bool on) => _keepSectionHead = on;

    public static void Readout(ReadOnlySpan<char> key, ReadOnlySpan<char> value) => Readout(key, value, in ConsoleStyle.TextPrimary);

    public static void Readout(ReadOnlySpan<char> key, ReadOnlySpan<char> value, in float4 ink, bool properName = false)
    {
        SectionRow();
        if (key.Length == 0)
        {
            Sentence(value, in ink);
            return;
        }
        float width = ConsoleStyle.ContentAvailWidth();
        float h = ConsoleStyle.FrameHeightPx;
        float2 at = ImGui.GetCursorScreenPos();
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        ConsoleStyle.PushValueFont();
        float y = at.Y + (h - ImGui.GetFontSize()) * 0.5f;
        ReadOnlySpan<char> name = properName ? key : Upper(key);
        float nameW = ImGui.CalcTextSize(name).X;
        float valueW = ImGui.CalcTextSize(value).X;
        bool fits = nameW + ConsoleWidgets.ReadoutColumnGapPx + valueW <= width;
        if (nameW <= width)
            dl.AddText(new float2(at.X, y), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextLabelU32), name);
        else
            FitText(at, width, h, name.ToString(), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextLabelU32));
        if (fits && value.Length > 0)
            dl.AddText(new float2(at.X + width - valueW, y), ConsoleStyle.ApplyAlpha(ImGui.ColorConvertFloat4ToU32(in ink)), value);
        ConsoleStyle.PopFont();
        ImGui.Dummy(new float2(width, h));
        if (!fits)
            Sentence(value, in ink);
    }

    private static ReadOnlySpan<char> Upper(ReadOnlySpan<char> text)
    {
        foreach (char c in text)
        {
            if (char.IsLower(c))
                return text.ToString().ToUpperInvariant().AsSpan();
        }
        return text;
    }

    private static bool _cardsBeside;
    private static float _cardsLeft, _cardsRight;
    private static long _cardsFrame;
    private static string _heldCard = "";
    private static long _heldCardFrame;
    private static float _heldCardY;

    /// <summary>Call it once in the frame of the window, and end it with <see cref="CardsFree"/>.</summary>
    public static void CardsBeside(float left, float right)
    {
        _cardsBeside = true;
        _cardsLeft = left;
        _cardsRight = right;
        _cardsFrame++;
    }

    public static void CardsLeftEdge(float left) => _cardsLeft = left;

    /// <summary>The next cards follow the mouse again, as the card of a stock window does.</summary>
    public static void CardsFree() => _cardsBeside = false;

    /// <summary>Lets a test know that a card shows in this frame.</summary>
    internal static Action? CardProbe;

    private static void PlaceCard(ReadOnlySpan<char> text)
    {
        CardProbe?.Invoke();
        if (!_cardsBeside)
            return;
        float y = ImGui.GetMousePos().Y;
        float rowH = ConsoleStyle.FrameHeightPx;
        if (_heldCardFrame + 1 >= _cardsFrame && text.SequenceEqual(_heldCard))
            y = _heldCardY;
        else
        {
            y -= rowH * 0.5f;
            _heldCard = text.ToString();
        }
        _heldCardY = y;
        _heldCardFrame = _cardsFrame;

        ImGuiViewportPtr main = ImGui.GetMainViewport();
        float2 work = main.WorkPos;
        float2 size = main.WorkSize;
        ConsoleStyle.PushValueFont();
        float cardW = ImGui.GetFontSize() * CardLines + ConsoleStyle.ScaledWindowPaddingPx.X * 2f;
        ConsoleStyle.PopFont();
        if (DockLayout.CardBeside(_cardsLeft, _cardsRight, work.X, work.X + size.X, work.Y + size.Y * 0.5f, y, rowH, cardW, DockLayout.CardGap * ImGuiHelper.InterfaceScale) is CardPlace place)
            ImGui.SetNextWindowPos(new float2(place.X, place.Y), ImGuiCond.Always, new float2(place.PivotX, place.PivotY));
    }

    // It follows ImGuiHelper.DrawTooltip.
    private const float CardLines = 28f;

    /// <summary>The stock tooltip card with no title, its text wrapped and drawn unformatted.</summary>
    public static void Tooltip(ReadOnlySpan<char> text)
    {
        ConsoleStyle.EnsureInitialized();
        PlaceCard(text);
        ImGui.PushStyleVar(ImGuiStyleVar.Alpha, 1f);
        ImGui.PushStyleColor(ImGuiCol.PopupBg, in ConsoleStyle.PanelRaised);
        ImGui.PushStyleColor(ImGuiCol.Border, in ConsoleStyle.Frame);
        ImGui.PushStyleVar(ImGuiStyleVar.WindowRounding, ConsoleWidgets.TooltipRoundingPx);
        ImGui.PushStyleVar(ImGuiStyleVar.WindowBorderSize, ConsoleStyle.WindowBorderThicknessPx);
        ImGui.PushStyleVar(ImGuiStyleVar.WindowPadding, ConsoleStyle.ScaledWindowPaddingPx);
        if (ImGui.BeginTooltip())
        {
            CardBody(text);
            ImGui.EndTooltip();
        }
        ImGui.PopStyleVar(4);
        ImGui.PopStyleColor(2);
    }

    private static void CardBody(ReadOnlySpan<char> text)
    {
        ConsoleStyle.PushValueFont();
        ImGui.PushStyleColor(ImGuiCol.Text, in ConsoleStyle.TextLabel);
        ImGui.PushTextWrapPos(ImGui.GetFontSize() * CardLines);
        ImGui.Text(text);
        ImGui.PopTextWrapPos();
        ImGui.PopStyleColor();
        ConsoleStyle.PopFont();
    }

    /// <summary>Doubles every '%', for text that goes to an ImGui call which reads it as a printf
    /// format, such as TextColored, TextWrapped or the ConsoleWidgets readouts.</summary>
    public static string EscapeFormat(string text) => text.Replace("%", "%%");

    /// <summary>The one card of the console: the name of what it speaks about in capitals, a hairline, and sentences, on the stock tooltip card (ConsoleWidgets.BeginTooltipCard).</summary>
    public static void Card(string title, string body)
    {
        if (title.Length == 0)
        {
            Tooltip(body.AsSpan());
            return;
        }
        ImGui.PushStyleVar(ImGuiStyleVar.Alpha, 1f);
        try
        {
            if (!BeginPlacedCard(title, body.AsSpan()))
                return;
            try
            {
                CardBody(body);
            }
            finally
            {
                ConsoleWidgets.EndTooltipCard();
            }
        }
        finally
        {
            ImGui.PopStyleVar();
        }
    }

    /// <summary>While it returns true, end it with ConsoleWidgets.EndTooltipCard.</summary>
    public static bool BeginPlacedCard(string title, ReadOnlySpan<char> key)
    {
        PlaceCard(key);
        return ConsoleWidgets.BeginTooltipCard(EscapeFormat(title.ToUpperInvariant()).AsSpan(), ImGui.GetFontSize() * 14f);
    }

    /// <summary>The width of the label column in front of a place or choice row.</summary>
    public static float LabelColumnWidth => 90f * ImGuiHelper.InterfaceScale;

    private static float LabelColumn(string label)
    {
        if (label.Length == 0)
            return 0f;
        return Math.Max(LabelColumnWidth, LabelWidth(label) + ImGui.GetStyle().ItemSpacing.X);
    }

    // The label of a place or choice row, in the label font and colour of every row, centred on the frame height at the left of the row.
    private static void RowLabel(string label, float2 origin)
    {
        if (label.Length == 0)
            return;
        ConsoleStyle.PushLabelFont();
        float fontSize = ImGui.GetFontSize();
        ImGui.GetWindowDrawList().AddText(origin + new float2(0f, (ConsoleStyle.FrameHeightPx - fontSize) * 0.5f),
            ConsoleStyle.ApplyAlpha(ConsoleStyle.TextLabelU32), label);
        ConsoleStyle.PopFont();
    }

    /// <summary>Starts a console row on the stock ConsoleWidgets.BeginRow, with the cursor at the control column.</summary>
    public static void BeginRow(ReadOnlySpan<char> label, float controlWidth = 0f, uint? labelInk = null, Rgb8? leadSwatch = null)
    {
        SectionRow();
        float inset = ConsoleWidgets.RowLabelInsetPx;
        float2 at = ImGui.GetCursorScreenPos();
        ImGui.SetCursorScreenPos(new float2(at.X - inset, at.Y));
        ConsoleWidgets.BeginRow(" ".AsSpan());
        float right = ImGui.GetCursorScreenPos().X + ConsoleWidgets.RowControlWidth;
        if (RowProbe != null)
            RowProbe(label.ToString(), at, new float2(right, at.Y + ConsoleStyle.FrameHeightPx));
        float room = right - (controlWidth > 0f ? controlWidth : ConsoleWidgets.RowControlWidth) - ConsoleWidgets.MinControlGapPx - at.X;
        if (label.Length == 0)
        {
            _rowLabelEnd = at.X;
            return;
        }
        if (leadSwatch != null)
        {
            float lead = DrawSwatches(ImGui.GetWindowDrawList(), at.X, at.Y + ConsoleStyle.FrameHeightPx * 0.5f, leadSwatch);
            at = new float2(at.X + lead, at.Y);
            room -= lead;
        }
        ConsoleStyle.PushLabelFont();
        var labelAt = new float2(at.X, at.Y + (ConsoleStyle.FrameHeightPx - ImGui.GetFontSize()) * 0.5f);
        uint ink = ConsoleStyle.ApplyAlpha(labelInk ?? ConsoleStyle.TextLabelU32);
        float labelW = ImGui.CalcTextSize(label).X;
        if (labelW <= room)
        {
            _rowLabelEnd = at.X + labelW;
            ImGui.GetWindowDrawList().AddText(labelAt, ink, label);
            ConsoleStyle.PopFont();
            return;
        }
        _rowLabelEnd = at.X + room;
        string full = label.ToString();
        string shown = FitEllipsis(full, room);
        if (shown.Length > 0)
            ImGui.GetWindowDrawList().AddText(labelAt, ink, shown);
        ConsoleStyle.PopFont();
        if (ConsoleWidgets.RowHovered)
            Tooltip(full.AsSpan());
    }

    private static float _rowLabelEnd;

    public static void SwatchAfterLabel(Rgb8? first, Rgb8? second = null)
        => DrawSwatches(ImGui.GetWindowDrawList(), _rowLabelEnd + 6f * ImGuiHelper.InterfaceScale, ConsoleWidgets.RowMin.Y + ConsoleStyle.FrameHeightPx * 0.5f, first, second);

    /// <summary>It stands whole or not at all (FoldRules.WholeOrNone), because a part of a sentence says nothing.</summary>
    public static bool RowDetail(string text, float controlWidth)
    {
        float gap = ImGui.GetStyle().ItemSpacing.X;
        float from = _rowLabelEnd + gap;
        float to = ImGui.GetCursorScreenPos().X + ConsoleWidgets.RowControlWidth - controlWidth - gap;
        PushSmall();
        string shown = FoldRules.WholeOrNone(text, to - from, MeasureText);
        if (shown.Length > 0)
            ImGui.GetWindowDrawList().AddText(new float2(from, ConsoleWidgets.RowMin.Y + (ConsoleStyle.FrameHeightPx - ImGui.GetFontSize()) * 0.5f), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32), shown);
        ConsoleStyle.PopFont();
        return shown.Length > 0;
    }

    public static void EndRow()
    {
        float2 min = ConsoleWidgets.RowMin;
        ConsoleWidgets.EndRow();
        ImGui.SetCursorScreenPos(min);
        ImGui.Dummy(new float2(0f, ConsoleStyle.FrameHeightPx));
    }

    public static void RowRight(float width)
    {
        float2 control = ImGui.GetCursorScreenPos();
        ImGui.SetCursorScreenPos(new float2(control.X + ConsoleWidgets.RowControlWidth - width, control.Y));
    }

    /// <summary>The stock switch of the row just begun.</summary>
    public static bool RowCheckbox(ReadOnlySpan<char> id, ref bool value)
    {
        if (_dimDepth == 0)
            return ConsoleWidgets.Checkbox(id, ref value, pending: false);
        RowRight(SwitchWidth(""));
        return Switch("", "##" + id.ToString(), ref value, "");
    }

    public static float StepFieldWidth => 90f * ImGuiHelper.InterfaceScale + 2f * (ConsoleStyle.FrameHeightPx + ImGui.GetStyle().ItemInnerSpacing.X);

    public static float ShortFieldWidth => 80f * ImGuiHelper.InterfaceScale;

    public static void RowField()
    {
        RowRight(FieldWidth);
        ImGui.SetNextItemWidth(FieldWidth);
    }

    private static float UnitGap => 4f * ImGuiHelper.InterfaceScale;

    public static float UnitRoom(string unit) => SmallWidth(unit) + UnitGap;

    public static void RowFieldUnit(float field, string unit)
    {
        RowRight(field + UnitRoom(unit));
        float2 at = ImGui.GetCursorScreenPos();
        PushSmall();
        ImGui.GetWindowDrawList().AddText(new float2(at.X + field + UnitGap, at.Y + (ConsoleStyle.FrameHeightPx - ImGui.GetFontSize()) * 0.5f), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32), unit);
        ConsoleStyle.PopFont();
        ImGui.SetNextItemWidth(field);
    }

    public static bool LabelFits(string label, float controlWidth)
        => LabelWidth(label) <= ConsoleStyle.ContentAvailWidth() - controlWidth - ConsoleWidgets.MinControlGapPx;

    /// <summary>The stock combo control of the row just begun with its preview fitted to the control, because stock clips the text with no mark.</summary>
    public static bool BeginCombo(ReadOnlySpan<char> id, string preview)
    {
        float room = ConsoleWidgets.RowControlWidth - ConsoleWidgets.ComboTextPaddingPx * 2f - ConsoleStyle.FrameHeightPx * 0.35f;
        ConsoleStyle.PushValueFont();
        string shown = FitEllipsis(preview, room);
        ConsoleStyle.PopFont();
        // Below full alpha the stock combo draws no control at all (ConsoleWidgets.BeginComboControl).
        bool veil = _dimDepth > 0;
        float2 at = ImGui.GetCursorScreenPos();
        if (veil)
            ImGui.PushStyleVar(ImGuiStyleVar.Alpha, 1f);
        bool open = ConsoleWidgets.BeginComboControl(id, shown.AsSpan(), pending: false);
        if (veil)
        {
            ImGui.PopStyleVar();
            ImGui.GetWindowDrawList().AddRectFilled(at, at + new float2(ConsoleWidgets.RowControlWidth, ConsoleStyle.FrameHeightPx), ConsoleStyle.ApplyAlpha(ConsoleStyle.PanelU32, 1f - DimScope.Alpha), ConsoleStyle.FrameRoundingPx);
        }
        if (!open && shown != preview && ConsoleWidgets.RowHovered)
            Tooltip(preview.AsSpan());
        return open;
    }

    private static float TabDividerRoom => 13f * ImGuiHelper.InterfaceScale;

    /// <summary>
    /// A row of place tabs: where the player is, not a setting. A muted label column (none when
    /// <paramref name="label"/> is empty), then the tabs as text: idle muted with a hover wash, active in
    /// the primary text colour with a 2 px underline in the active tool colour. Tabs that do not fit
    /// go on a second line. Returns the clicked index, or -1.
    /// </summary>
    public static int PlaceTabs(string label, string id, ReadOnlySpan<string> tabs, int active, ReadOnlySpan<string> tips = default,
        int marked = -1, int dividerAt = -1, ReadOnlySpan<bool> dim = default, ReadOnlySpan<int> counts = default, bool small = false)
    {
        float2 origin = ImGui.GetCursorScreenPos();
        float avail = ConsoleStyle.ContentAvailWidth();
        float scale = ImGuiHelper.InterfaceScale;
        float pad = small ? ViewChoicePad : ImGui.GetStyle().ItemSpacing.X;
        float gap = pad;
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        PushSmall();
        float smallFont = ImGui.GetFontSize();
        float countRoom = ImGui.CalcTextSize("0").X + 2f * scale;
        float smallLabelW = label.Length > 0 ? ImGui.CalcTextSize(label).X + ImGui.GetStyle().ItemSpacing.X : 0f;
        ConsoleStyle.PopFont();
        float h = small ? smallFont + 8f * scale : ConsoleStyle.FrameHeightPx;
        float labelW = small ? smallLabelW : LabelColumn(label);
        if (!small && labelW > 0f && !PlaceTabsFit(tabs, dividerAt, avail - labelW, counts, countRoom))
        {
            label = "";
            labelW = 0f;
        }
        float right = origin.X + Math.Max(labelW + 1f, avail);
        if (!small)
            RowLabel(label, origin);
        else if (label.Length > 0)
        {
            PushSmall();
            dl.AddText(origin + new float2(0f, (h - smallFont) * 0.5f), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32), label);
            ConsoleStyle.PopFont();
        }

        float x = origin.X + labelW;
        float y = origin.Y;
        int clicked = -1;
        bool windowHovered = ImGui.IsWindowHovered(ImGuiHoveredFlags.ChildWindows);
        PushTabFont(small);
        try
        {
            float fontSize = ImGui.GetFontSize();
            float text = 0f;
            for (int i = 0; i < tabs.Length; i++)
                text += ImGui.CalcTextSize(tabs[i]).X + CountRoomOf(counts, i, countRoom);
            float dividerRoom = dividerAt > 0 ? TabDividerRoom : 0f;
            float room = right - x - dividerRoom;
            pad = DockLayout.TabPad(text, tabs.Length, room, pad, 2f * scale);
            gap = pad;
            for (int i = 0; i < tabs.Length; i++)
            {
                if (i == dividerAt && i > 0)
                {
                    float mid = x + dividerRoom * 0.5f;
                    dl.AddLine(new float2(mid, y + h * 0.2f), new float2(mid, y + h * 0.8f), ConsoleStyle.ApplyAlpha(ConsoleStyle.FrameU32), ConsoleStyle.WindowBorderThicknessPx);
                    x += dividerRoom;
                }
                float textW = ImGui.CalcTextSize(tabs[i]).X;
                float w = textW + CountRoomOf(counts, i, countRoom) + pad * 2f;
                if (i > 0 && x + w > right + DockLayout.TabRowTolerance)
                {
                    x = origin.X + labelW;
                    y += h;
                }
                var pos = new float2(x, y);
                var size = new float2(w, h);
                ImGui.SetCursorScreenPos(pos);
                bool hit = ImGui.InvisibleButton(id + "##Tab" + i, size);
                NoteRow(id, i, pos, size);
                bool hovered = windowHovered && ImGui.IsMouseHoveringRect(in pos, pos + size);
                bool isActive = i == active;
                Plate(dl, pos, size, isActive, hovered, ImDrawFlags.None, ConsoleStyle.ApplyAlpha(ConsoleStyle.ActiveToolU32),
                    small ? ConsoleStyle.WindowBorderThicknessPx : 2f * scale, raised: small);
                bool isDim = i < dim.Length && dim[i];
                uint ink = isActive ? ConsoleStyle.ApplyAlpha(ConsoleStyle.TextPrimaryU32)
                    : isDim ? ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32, 0.55f)
                    : ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32);
                dl.AddText(pos + new float2(pad, (h - fontSize) * 0.5f), ink, tabs[i]);
                if (i < counts.Length && counts[i] > 0)
                {
                    PushSmall();
                    dl.AddText(new float2(pos.X + pad + textW + 2f * scale, pos.Y + (h - smallFont) * 0.5f), ConsoleStyle.ApplyAlpha(Palette.Changed.U32), TuneRules.CountText(counts[i]));
                    ConsoleStyle.PopFont();
                }
                if (i == marked)
                {
                    float r = 2.5f * scale;
                    float inset = 3f * scale + r;
                    dl.AddCircleFilled(new float2(pos.X + w - inset, pos.Y + inset), r, ConsoleStyle.ApplyAlpha(ConsoleStyle.TextPrimaryU32));
                }
                if (hovered && i < tips.Length && tips[i].Length > 0)
                {
                    ConsoleStyle.PopFont();
                    Card(tabs[i], tips[i]);
                    PushTabFont(small);
                }
                if (hit)
                {
                    clicked = i;
                    ShapeChanged = true;
                }
                x += w + gap;
            }
        }
        finally
        {
            ConsoleStyle.PopFont();
        }

        float rows = (y - origin.Y) / h + 1f;
        ImGui.SetCursorScreenPos(origin);
        ImGui.Dummy(new float2(avail, rows * h));
        return clicked;
    }

    private static void PushTabFont(bool small)
    {
        if (small)
            PushSmall();
        else
            ConsoleStyle.PushLabelFont();
    }

    // The plate of a tab or a choice: when active a raised plate, a tint and an underline, else a hover wash.
    private static void Plate(ImDrawListPtr dl, float2 pos, float2 size, bool active, bool hovered, ImDrawFlags corners, uint underlineInk, float underlineH,
        bool raised = true, ImColor8? tint = null)
    {
        float rounding = ConsoleStyle.FrameRoundingPx;
        if (active)
        {
            if (raised)
                dl.AddRectFilled(pos, pos + size, ConsoleStyle.ApplyAlpha(ConsoleStyle.PanelRaisedU32), rounding, corners);
            if (tint is ImColor8 wash)
                dl.AddRectFilled(pos, pos + size, wash, rounding, corners);
            dl.AddRectFilled(new float2(pos.X, pos.Y + size.Y - underlineH), pos + size, underlineInk);
        }
        else if (hovered)
            dl.AddRectFilled(pos, pos + size, ConsoleStyle.ApplyAlpha(ConsoleStyle.RowHoverWashU32), rounding, corners);
    }

    private static float CountRoomOf(ReadOnlySpan<int> counts, int tab, float room) => tab < counts.Length && counts[tab] >= 0 ? room : 0f;

    private static bool PlaceTabsFit(ReadOnlySpan<string> tabs, int dividerAt, float room, ReadOnlySpan<int> counts, float countRoom)
    {
        float text = 0f;
        for (int i = 0; i < tabs.Length; i++)
            text += LabelWidth(tabs[i]) + CountRoomOf(counts, i, countRoom);
        return text + 2f * ImGuiHelper.InterfaceScale * DockLayout.TabGaps(tabs.Length) + (dividerAt > 0 ? TabDividerRoom : 0f) <= room;
    }

    public static int ChoiceRow(string label, string id, ReadOnlySpan<string> segments, int active,
        ReadOnlySpan<string> held = default, ReadOnlySpan<string> cards = default, string description = "", string labelTip = "", bool feedsPicture = false, ReadOnlySpan<Rgb8> marks = default)
    {
        SectionRow();
        float2 origin = ImGui.GetCursorScreenPos();
        float avail = ConsoleStyle.ContentAvailWidth();
        float labelW = LabelColumn(label);
        float h = ConsoleStyle.FrameHeightPx;
        RowLabel(label, origin);
        bool windowHovered = ImGui.IsWindowHovered(ImGuiHoveredFlags.ChildWindows);
        if (labelTip.Length > 0 && windowHovered && ImGui.IsMouseHoveringRect(in origin, origin + new float2(labelW, h)))
            Card(label, labelTip);

        var barMin = new float2(origin.X + labelW, origin.Y);
        ImGui.SetCursorScreenPos(barMin);
        int picked = Segmented(id, segments, active, feedsPicture, marks);

        Span<float> widths = stackalloc float[segments.Length];
        SegmentWidths(segments, avail - labelW, widths);
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        float segX = barMin.X;
        for (int i = 0; i < segments.Length; i++)
        {
            var min = new float2(segX, barMin.Y);
            var max = new float2(min.X + widths[i], min.Y + h);
            segX += widths[i];
            bool isHeld = i < held.Length && held[i].Length > 0;
            if (isHeld)
                dl.AddRectFilled(min, max, ConsoleStyle.ApplyAlpha(ConsoleStyle.PanelU32, 0.35f), ConsoleStyle.FrameRoundingPx);
            string card = i < cards.Length ? cards[i] : "";
            if ((isHeld || card.Length > 0) && windowHovered && ImGui.IsMouseHoveringRect(in min, in max))
                Card(segments[i], isHeld && card.Length > 0 ? held[i] + "\n\n" + card : isHeld ? held[i] : card);
            if (picked == i && isHeld)
                picked = -1;
        }

        ImGui.SetCursorScreenPos(origin);
        ImGui.Dummy(new float2(avail, h));
        if (description.Length > 0)
        {
            ImGui.SetCursorScreenPos(new float2(origin.X + labelW, ImGui.GetCursorScreenPos().Y));
            ImGui.PushTextWrapPos(ImGui.GetCursorPosX() + Math.Max(40f, avail - labelW));
            ImGui.PushStyleColor(ImGuiCol.Text, in ConsoleStyle.TextMuted);
            ImGui.Text(description);
            ImGui.PopStyleColor();
            ImGui.PopTextWrapPos();
        }
        return picked;
    }

    public static bool ChoiceRowFits(string label, ReadOnlySpan<string> segments)
    {
        float room = ConsoleStyle.ContentAvailWidth() - LabelColumn(label);
        float need = 0f;
        foreach (string segment in segments)
            need += LabelWidth(segment) + 2f * ConsoleWidgets.ComboTextPaddingPx;
        return need <= room;
    }

    private static void SegmentWidths(ReadOnlySpan<string> segments, float width, Span<float> widths)
    {
        Span<float> text = stackalloc float[segments.Length];
        for (int i = 0; i < segments.Length; i++)
            text[i] = LabelWidth(segments[i]);
        DockLayout.SegmentWidths(text, width, ConsoleWidgets.ComboTextPaddingPx, widths);
    }

    /// <summary>A row of choices in the frame of the stock ConsoleWidgets.Segmented.</summary>
    public static int Segmented(string id, ReadOnlySpan<string> segments, int activeIndex, bool feedsPicture = false, ReadOnlySpan<Rgb8> marks = default)
    {
        ConsoleStyle.EnsureInitialized();
        float width = ConsoleStyle.ContentAvailWidth();
        float h = ConsoleStyle.FrameHeightPx;
        float2 min = ImGui.GetCursorScreenPos();
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        dl.AddRectFilled(min, min + new float2(width, h), ConsoleStyle.ApplyAlpha(ConsoleStyle.InsetU32), ConsoleStyle.FrameRoundingPx);
        dl.AddRect(min, min + new float2(width, h), ConsoleStyle.ApplyAlpha(ConsoleStyle.FrameU32), ConsoleStyle.FrameRoundingPx, ImDrawFlags.None, ConsoleStyle.WindowBorderThicknessPx);
        if (segments.Length == 0)
        {
            ImGui.Dummy(new float2(width, h));
            return -1;
        }
        Span<float> widths = stackalloc float[segments.Length];
        SegmentWidths(segments, width, widths);
        float line = ConsoleStyle.ModifiedTickThicknessPx;
        int clicked = -1;
        float x = min.X;
        ConsoleStyle.PushLabelFont();
        try
        {
            for (int i = 0; i < segments.Length; i++)
            {
                float segW = widths[i];
                var pos = new float2(x, min.Y);
                var size = new float2(segW, h);
                x += segW;
                ImGui.SetCursorScreenPos(pos);
                NoteRow(id, i, pos, size);
                if (ImGui.InvisibleButton(id + "##Seg" + i, size))
                {
                    clicked = i;
                    ShapeChanged = true;
                }
                bool hovered = ImGui.IsItemHovered();
                bool active = i == activeIndex;
                ImDrawFlags corners = segments.Length == 1 ? ImDrawFlags.RoundCornersAll
                    : i == 0 ? ImDrawFlags.RoundCornersLeft
                    : i == segments.Length - 1 ? ImDrawFlags.RoundCornersRight
                    : ImDrawFlags.RoundCornersNone;
                Plate(dl, pos, size, active, hovered, corners, ConsoleStyle.ApplyAlpha(feedsPicture ? Palette.Afc.U32 : ConsoleStyle.ActiveToolU32), line,
                    tint: feedsPicture ? Palette.Afc.A(ChosenTint * ImGui.GetStyle().Alpha) : null);
                float markRoom = i < marks.Length ? SwatchRoom : 0f;
                string shown = FitEllipsis(segments[i], segW - 2f * ConsoleWidgets.ComboTextPaddingPx - markRoom);
                float2 textSize = ImGui.CalcTextSize(shown);
                float2 textAt = pos + (size - textSize) * 0.5f + new float2(markRoom * 0.5f, 0f);
                if (markRoom > 0f)
                    DrawSwatches(dl, textAt.X - markRoom, pos.Y + h * 0.5f, marks[i]);
                dl.AddText(textAt, ConsoleStyle.ApplyAlpha(active ? ConsoleStyle.TextPrimaryU32 : ConsoleStyle.TextLabelU32), shown);
                if (hovered && shown != segments[i])
                {
                    ConsoleStyle.PopFont();
                    Tooltip(segments[i].AsSpan());
                    ConsoleStyle.PushLabelFont();
                }
            }
        }
        finally
        {
            ConsoleStyle.PopFont();
        }
        ImGui.SetCursorScreenPos(min);
        ImGui.Dummy(new float2(0f, Math.Max(0f, h - ImGui.GetStyle().ItemSpacing.Y)));
        return clicked;
    }

    private const float ChosenTint = 0.13f;

    /// <summary>A small table whose rows are the choices of a setting: the heads in the small level, then one row per choice, the chosen one on a raised plate with a tint and a bar in the AFC colour... With <paramref name="radio"/> each row starts with a radio mark, filled on the chosen row, so a row reads as one of several choices and not as a list of facts.</summary>
    public static int ChoiceTable(string id, ReadOnlySpan<string> heads, ReadOnlySpan<string[]> rows, int active,
        ReadOnlySpan<string> held = default, ReadOnlySpan<string> cards = default, string headTip = "", ReadOnlySpan<bool> dim = default, ReadOnlySpan<TableMark> marks = default, ReadOnlySpan<CellSwatch> swatches = default,
        bool radio = false)
    {
        SectionRow();
        ConsoleStyle.EnsureInitialized();
        int columns = heads.Length;
        float scale = ImGuiHelper.InterfaceScale;
        float2 origin = ImGui.GetCursorScreenPos();
        float avail = ConsoleStyle.ContentAvailWidth();
        float pad = 6f * scale;
        float gap = 8f * scale;
        float radioR = radio ? 4.5f * scale : 0f;
        float radioRoom = radio ? 2f * radioR + pad : 0f;
        ImDrawListPtr dl = ImGui.GetWindowDrawList();

        Span<float> longest = stackalloc float[columns];
        Span<float> widths = stackalloc float[columns];
        ConsoleStyle.PushValueFont();
        for (int r = 0; r < rows.Length; r++)
            for (int c = 0; c < columns; c++)
                longest[c] = Math.Max(longest[c], ImGui.CalcTextSize(rows[r][c]).X + SwatchesRoom(FindSwatch(swatches, r, c)));
        ConsoleStyle.PopFont();
        PushSmall();
        for (int c = 0; c < columns; c++)
            longest[c] = Math.Max(longest[c], ImGui.CalcTextSize(heads[c]).X);
        float headH = ImGui.GetFontSize() + 4f * scale;
        float headX = origin.X + pad + radioRoom;
        DockLayout.TableColumns(longest, avail - 2f * pad - radioRoom - gap * Math.Max(0, columns - 1), widths);
        for (int c = 0; c < columns; c++)
        {
            dl.AddText(new float2(headX, origin.Y + 2f * scale), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32), heads[c]);
            headX += widths[c] + gap;
        }
        ConsoleStyle.PopFont();
        if (columns > 0 && headTip.Length > 0 && Hovering(origin, origin + new float2(avail, headH)))
            Card(heads[0], headTip);

        if (RowProbe != null)
            NoteRow(id + "#Head", origin, new float2(avail, headH));

        float y = origin.Y + headH;
        int picked = -1;
        ConsoleStyle.PushValueFont();
        try
        {
            float lineH = ImGui.GetFontSize();
            for (int i = 0; i < rows.Length; i++)
            {
                string[] cells = rows[i];
                float textH = lineH;
                for (int c = 0; c < columns; c++)
                {
                    CellMarks(marks, i, c, out bool method, out _);
                    textH = Math.Max(textH, CellHeight(cells[c], widths[c] - SwatchesRoom(FindSwatch(swatches, i, c)), method, lineH));
                }
                var min = new float2(origin.X, y);
                var size = new float2(avail, textH + 6f * scale);
                ImGui.SetCursorScreenPos(min);
                bool hit = ImGui.InvisibleButton(id + "##Row" + i, size);
                NoteRow(id, i, min, size);
                bool hovered = ImGui.IsItemHovered();
                bool isHeld = i < held.Length && held[i].Length > 0;
                bool isActive = i == active;
                if (isActive)
                {
                    dl.AddRectFilled(min, min + size, ConsoleStyle.ApplyAlpha(ConsoleStyle.PanelRaisedU32), ConsoleStyle.FrameRoundingPx);
                    dl.AddRectFilled(min, min + size, Palette.Afc.A(ChosenTint * ImGui.GetStyle().Alpha), ConsoleStyle.FrameRoundingPx);
                    dl.AddRectFilled(min, min + new float2(ConsoleStyle.ModifiedTickThicknessPx, size.Y), ConsoleStyle.ApplyAlpha(Palette.Afc.U32));
                }
                else if (hovered && !isHeld)
                    dl.AddRectFilled(min, min + size, ConsoleStyle.ApplyAlpha(ConsoleStyle.RowHoverWashU32), ConsoleStyle.FrameRoundingPx);

                float4 ink = ColorRules.RowToneOf(isActive, isHeld || (i < dim.Length && dim[i])) switch
                {
                    RowTone.Bright => ConsoleStyle.TextPrimary,
                    RowTone.Dim => ConsoleStyle.TextMuted,
                    _ => ConsoleStyle.TextLabel,
                };
                if (radio)
                {
                    var dot = new float2(min.X + pad + radioR, y + size.Y * 0.5f);
                    uint ring = ConsoleStyle.ApplyAlpha(isActive ? Palette.Afc.U32 : ImGui.ColorConvertFloat4ToU32(in ink));
                    dl.AddCircle(in dot, radioR, ring, 0, 1.5f * scale);
                    if (isActive)
                        dl.AddCircleFilled(dot, radioR * 0.5f, ring);
                }
                float x = min.X + pad + radioRoom;
                for (int c = 0; c < columns; c++)
                {
                    CellSwatch? swatch = FindSwatch(swatches, i, c);
                    float room = SwatchesRoom(swatch);
                    TableInk? marked = CellMarks(marks, i, c, out bool methodMark, out bool lit);
                    float cellH = CellHeight(cells[c], widths[c] - room, methodMark, lineH);
                    float cellY = y + (size.Y - cellH) * 0.5f;
                    if (swatch is CellSwatch s)
                        DrawSwatches(dl, x, cellY + lineH * 0.5f, s.Hue, s.Second);
                    float4 cellInk = marked is TableInk kind ? InkOf(kind) : ink;
                    if (methodMark && SplitMethod(cells[c], out string before, out string method))
                        DrawMethodCell(dl, new float2(x + room, cellY), widths[c] - room, before, method, in cellInk,
                            ConsoleStyle.ApplyAlpha(lit ? Palette.Afc.U32 : isActive ? Palette.DimOnBand.U32 : ConsoleStyle.TextMutedU32));
                    else
                    {
                        ImGui.SetCursorScreenPos(new float2(x + room, cellY));
                        ImGui.PushTextWrapPos(ImGui.GetCursorPosX() + widths[c] - room);
                        ImGui.PushStyleColor(ImGuiCol.Text, in cellInk);
                        ImGui.Text(cells[c]);
                        ImGui.PopStyleColor();
                        ImGui.PopTextWrapPos();
                    }
                    x += widths[c] + gap;
                }

                string card = i < cards.Length ? cards[i] : "";
                if (hovered && (isHeld || card.Length > 0))
                {
                    ConsoleStyle.PopFont();
                    Card(cells[0], isHeld && card.Length > 0 ? held[i] + "\n\n" + card : isHeld ? held[i] : card);
                    ConsoleStyle.PushValueFont();
                }
                if (hit && !isHeld)
                {
                    picked = i;
                    ShapeChanged = true;
                }
                y += size.Y;
            }
        }
        finally
        {
            ConsoleStyle.PopFont();
        }
        ImGui.SetCursorScreenPos(origin);
        ImGui.Dummy(new float2(avail, y - origin.Y));
        return picked;
    }

    // The first ink of a cell that is not a method mark, and whether a method mark and a lit one (Method) stand in the cell.
    private static TableInk? CellMarks(ReadOnlySpan<TableMark> marks, int row, int column, out bool method, out bool lit)
    {
        TableInk? ink = null;
        method = lit = false;
        foreach (TableMark mark in marks)
        {
            if (mark.Row != row || mark.Column != column)
                continue;
            if (mark.Ink is TableInk.Method or TableInk.MethodDim)
            {
                method = true;
                lit |= mark.Ink == TableInk.Method;
            }
            else
                ink ??= mark.Ink;
        }
        return ink;
    }

    private static float4 InkOf(TableInk ink) => ink switch
    {
        TableInk.Danger => ConsoleStyle.Danger,
        TableInk.DangerDim => Palette.BadDim.F4,
        TableInk.DangerOnBand => Palette.BadOnBand.F4,
        TableInk.WarnDim => Palette.WarnDim.F4,
        TableInk.WarnOnBand => Palette.WarnOnBand.F4,
        TableInk.Stock => Palette.Stock.F4,
        TableInk.Afc => Palette.Afc.F4,
        _ => WarnInk,
    };

    private static bool SplitMethod(string cell, out string before, out string method)
    {
        int open = cell.LastIndexOf('(');
        before = open > 0 ? cell.Substring(0, open) : "";
        method = open > 0 ? cell.Substring(open) : "";
        return open > 0;
    }

    private static float CellHeight(string cell, float width, bool methodMark, float lineH)
    {
        if (methodMark && ImGui.CalcTextSize(cell).X > width && SplitMethod(cell, out string before, out _))
            return ImGui.CalcTextSize(before.TrimEnd(), false, width).Y + lineH;
        return ImGui.CalcTextSize(cell, false, width).Y;
    }

    private static void DrawMethodCell(ImDrawListPtr dl, float2 at, float width, string before, string method, in float4 ink, uint methodInk)
    {
        float beforeW = ImGui.CalcTextSize(before).X;
        if (beforeW + ImGui.CalcTextSize(method).X <= width)
        {
            dl.AddText(at, ConsoleStyle.ApplyAlpha(ImGui.ColorConvertFloat4ToU32(in ink)), before);
            dl.AddText(at + new float2(beforeW, 0f), methodInk, method);
            return;
        }
        string head = before.TrimEnd();
        ImGui.SetCursorScreenPos(at);
        ImGui.PushTextWrapPos(ImGui.GetCursorPosX() + width);
        ImGui.PushStyleColor(ImGuiCol.Text, in ink);
        ImGui.Text(head);
        ImGui.PopStyleColor();
        ImGui.PopTextWrapPos();
        dl.AddText(at + new float2(0f, ImGui.CalcTextSize(head, false, width).Y), methodInk, method);
    }

    private static CellSwatch? FindSwatch(ReadOnlySpan<CellSwatch> swatches, int row, int column)
    {
        foreach (CellSwatch swatch in swatches)
        {
            if (swatch.Row == row && swatch.Column == column)
                return swatch;
        }
        return null;
    }

    private static float SwatchesRoom(CellSwatch? swatch) => swatch is CellSwatch s ? SwatchRoom * (s.Second != null ? 2f : 1f) : 0f;

    /// <summary>A read-only strip of steps with a mark between them, in the small level: a stock step as plain text in the stock colour with a STOCK chip after it, an end that is a crash in the danger colour,...</summary>
    public static void ChainStrip(ReadOnlySpan<ChainStep> steps, bool cannotFly = false)
    {
        SectionRow();
        if (steps.Length == 0)
            return;
        float scale = ImGuiHelper.InterfaceScale;
        float2 origin = ImGui.GetCursorScreenPos();
        float width = ConsoleStyle.ContentAvailWidth();
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        PushSmall();
        float lineH = ImGui.GetFontSize() + 4f * scale;
        float textY = (lineH - ImGui.GetFontSize()) * 0.5f;
        const string Mark = " > ";
        float markW = ImGui.CalcTextSize(Mark).X;
        uint markInk = ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32);
        uint afcInk = cannotFly ? markInk : ConsoleStyle.ApplyAlpha(Palette.Afc.U32);
        float chipW = OwnerChipWidth(Owner.Stock, "STOCK");
        float chipGap = 4f * scale;
        float x = 0f, y = 0f;
        for (int i = 0; i < steps.Length; i++)
        {
            ChainStep step = steps[i];
            float w = ImGui.CalcTextSize(step.Text).X + (step.Stock ? chipW + chipGap : 0f);
            if (i > 0)
            {
                if (x + markW <= width)
                    dl.AddText(origin + new float2(x, y + textY), markInk, Mark);
                x += markW;
                if (x + w > width)
                {
                    x = 0f;
                    y += lineH;
                }
            }
            var at = origin + new float2(x, y);
            if (step.Stock)
            {
                dl.AddText(at + new float2(0f, textY), cannotFly ? markInk : ConsoleStyle.ApplyAlpha(Palette.Stock.U32), step.Text);
                OwnerChipAt(dl, at + new float2(w - chipW, 0f), lineH, Owner.Stock, "STOCK", cannotFly);
                StockChipCard(at, new float2(w, lineH));
                NoteRow("AfcChainStock", at, new float2(w, lineH));
            }
            else
            {
                uint ink = step.Danger ? ConsoleStyle.ApplyAlpha(ConsoleStyle.DangerU32) : step.Warn ? ConsoleStyle.ApplyAlpha(Palette.Warn.U32) : afcInk;
                dl.AddText(at + new float2(0f, textY), ink, step.Text);
            }
            x += w;
        }
        ConsoleStyle.PopFont();
        ImGui.SetCursorScreenPos(origin);
        ImGui.Dummy(new float2(width, y + lineH));
    }

    public static void ChainRowRoom()
    {
        PushSmall();
        float lineH = ImGui.GetFontSize() + 4f * ImGuiHelper.InterfaceScale;
        ConsoleStyle.PopFont();
        ImGui.Dummy(new float2(0f, lineH));
    }

    /// <summary>ConsoleWidgets.DangerButton is a grey outline that turns red only under the mouse, so beside CLEAR or RE-CALCULATE it does not read as the way out.</summary>
    public static bool StopButton(string label, string id, float2 size) => FilledLabelButton(label, id, size, stop: true);

    /// <summary>The button of the red failure band: dark text and a dark border on the red, because the grey outline of a stock button reads as off there.</summary>
    public static bool BandButton(string label, string id, float2 size) => FilledLabelButton(label, id, size, stop: false);

    // A stop button fills in the danger colour, a band button has a dark outline instead.
    private static bool FilledLabelButton(string label, string id, float2 size, bool stop)
    {
        if (stop)
            ConsoleStyle.EnsureInitialized();
        float2 min = ImGui.GetCursorScreenPos();
        float2 max = min + size;
        bool clicked = ImGui.InvisibleButton(id, size);
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        if (stop)
            dl.AddRectFilled(min, max, ConsoleStyle.ApplyAlpha(ConsoleStyle.DangerU32), ConsoleStyle.FrameRoundingPx);
        if (ImGui.IsItemHovered())
            dl.AddRectFilled(min, max, ConsoleStyle.ApplyAlpha(ConsoleStyle.ButtonHoverLightenU32), ConsoleStyle.FrameRoundingPx);
        if (!stop)
            dl.AddRect(min, max, ConsoleStyle.ApplyAlpha(ConsoleStyle.PanelU32), ConsoleStyle.FrameRoundingPx, ImDrawFlags.None, 1.5f * ImGuiHelper.InterfaceScale);
        ConsoleStyle.PushLabelFont();
        dl.AddText(min + (size - ImGui.CalcTextSize(label)) * 0.5f, ConsoleStyle.ApplyAlpha(ConsoleStyle.PanelU32), label);
        ConsoleStyle.PopFont();
        return clicked;
    }

    public static bool Link(string label, string id, string tip = "")
    {
        string text = label + " >";
        ConsoleStyle.PushLabelFont();
        try
        {
            float2 size = ImGui.CalcTextSize(text);
            float h = ConsoleStyle.FrameHeightPx;
            float2 pos = ImGui.GetCursorScreenPos();
            bool hit = ImGui.InvisibleButton(id, new float2(size.X, h));
            NoteRow(id, pos, new float2(size.X, h));
            bool hovered = ImGui.IsItemHovered();
            ImDrawListPtr dl = ImGui.GetWindowDrawList();
            float2 at = pos + new float2(0f, (h - size.Y) * 0.5f);
            dl.AddText(at, ConsoleStyle.ApplyAlpha(hovered ? ConsoleStyle.TextPrimaryU32 : ConsoleStyle.TextLabelU32), text);
            dl.AddLine(at + new float2(0f, size.Y), at + size, ConsoleStyle.ApplyAlpha(hovered ? ConsoleStyle.TextPrimaryU32 : ConsoleStyle.FrameU32), 1f);
            if (hovered && tip.Length > 0)
            {
                ConsoleStyle.PopFont();
                Card(label, tip);
                ConsoleStyle.PushLabelFont();
            }
            return hit;
        }
        finally
        {
            ConsoleStyle.PopFont();
        }
    }

    public static float LinkWidth(string label) => LabelWidth(label + " >");

    /// <summary>A label in the label font that is a button of its own width, centred on <paramref name="h"/>.</summary>
    public static bool TextButton(string id, string label, float2 at, float h, uint ink, uint hoverInk, out bool hovered, bool noteItem = false, float width = 0f)
    {
        ImGui.SetCursorScreenPos(at);
        bool hit = ImGui.InvisibleButton(id, new float2(Math.Max(1f, width > 0f ? width : LabelWidth(label)), h));
        if (noteItem)
            NoteItem(id);
        hovered = ImGui.IsItemHovered();
        ConsoleStyle.PushLabelFont();
        ImGui.GetWindowDrawList().AddText(new float2(at.X, at.Y + (h - ImGui.GetFontSize()) * 0.5f), ConsoleStyle.ApplyAlpha(hovered ? hoverInk : ink), label);
        ConsoleStyle.PopFont();
        return hit;
    }

    /// <summary>A link at the right edge of the content, on its own line.</summary>
    public static bool LinkRight(string label, string id, string tip = "")
    {
        float avail = ConsoleStyle.ContentAvailWidth();
        float w = LinkWidth(label);
        ImGui.SetCursorPosX(ImGui.GetCursorPosX() + Math.Max(0f, avail - w));
        return Link(label, id, tip);
    }

    public static bool NextRow(string text, string link, string id, string tip = "")
    {
        SectionRow();
        float width = ConsoleStyle.ContentAvailWidth();
        float h = ConsoleStyle.FrameHeightPx;
        float2 at = ImGui.GetCursorScreenPos();
        float linkW = LinkWidth(link);
        ConsoleStyle.PushValueFont();
        FitText(at, width - linkW - ImGui.GetStyle().ItemSpacing.X, h, text, ConsoleStyle.ApplyAlpha(ConsoleStyle.TextPrimaryU32));
        ConsoleStyle.PopFont();
        ImGui.SetCursorScreenPos(new float2(at.X + width - linkW, at.Y));
        return Link(link, id, tip);
    }

    /// <summary>
    /// <paramref name="text"/> cut so it fits in <paramref name="room"/> pixels of the current font, with "..." at the end, or
    /// the whole text when it fits. The cut point is found by a binary search, so a long line costs a few text measurements a frame.
    /// </summary>
    public static string FitEllipsis(string text, float room)
    {
        if (ImGui.CalcTextSize(text).X <= room)
            return text;
        float dots = ImGui.CalcTextSize("...").X;
        int lo = 0, hi = text.Length;
        while (lo < hi)
        {
            int mid = (lo + hi + 1) / 2;
            if (ImGui.CalcTextSize(text.AsSpan(0, mid)).X + dots <= room)
                lo = mid;
            else
                hi = mid - 1;
        }
        return lo > 0 ? text.Substring(0, lo).TrimEnd() + "..." : "";
    }

    public static float FitText(float2 at, float room, float rowH, string text, uint colour)
    {
        if (text.Length == 0 || room <= 0f)
            return 0f;
        string shown = FitEllipsis(text, room);
        float2 size = ImGui.CalcTextSize(shown);
        ImGui.GetWindowDrawList().AddText(at + new float2(0f, (rowH - size.Y) * 0.5f), colour, shown);
        if (shown != text && Hovering(at, at + new float2(room, rowH)))
            Tooltip(text.AsSpan());
        return size.X;
    }

    public static void StageValues(string name, ReadOnlySpan<string> values, ReadOnlySpan<Rgb8> hues, string end)
    {
        Readout(name.AsSpan(), default);
        float width = ConsoleStyle.ContentAvailWidth();
        float2 origin = ImGui.GetCursorScreenPos();
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        ConsoleStyle.PushValueFont();
        float lineH = ImGui.GetFontSize();
        float gap = ImGui.CalcTextSize("  ").X;
        uint ink = ConsoleStyle.ApplyAlpha(ConsoleStyle.TextPrimaryU32);
        float x = 0f, y = 0f;
        for (int i = 0; i <= values.Length; i++)
        {
            bool last = i == values.Length;
            string text = last ? end : values[i];
            float room = last ? 0f : SwatchRoom;
            float w = room + ImGui.CalcTextSize(text).X;
            if (x > 0f && x + w > width)
            {
                x = 0f;
                y += lineH;
            }
            if (!last)
                DrawSwatches(dl, origin.X + x, origin.Y + y + lineH * 0.5f, hues[i]);
            dl.AddText(origin + new float2(x + room, y), ink, text);
            x += w + gap;
        }
        ConsoleStyle.PopFont();
        ImGui.Dummy(new float2(width, y + lineH));
    }

    private static readonly Dictionary<string, bool> _folds = new();

    private static float FoldDetailGap => 12f * ImGuiHelper.InterfaceScale;

    public static void ResetRows()
    {
        _folds.Clear();
        _shapes.Clear();
        _sectionRows.Clear();
        _sectionKey = "";
        _sectionHead = null;
        _timeEdits.Clear();
        _heldCard = "";
        PageWarning = "";
        _revealPlan = false;
        PlanFormBeside = false;
        _keepSectionHead = false;
    }

    public static void OpenFold(string id) => _folds[id] = true;

    public static bool FoldIsOpen(string id, bool openByDefault = false) => _folds.TryGetValue(id, out bool stored) ? stored : openByDefault;

    public static void ForgetFold(string id) => _folds.Remove(id);

    /// <summary>The capitals before the colon of a label are its name (FoldRules.Split), and what follows stands in the small level at one column, so the folds of a page read as a table.</summary>
    public static bool Fold(string label, string id, bool openByDefault = false, string tip = "", float4? ink = null, float reserve = 0f)
    {
        bool open = _folds.TryGetValue(id, out bool stored) ? stored : openByDefault;
        float width = ConsoleStyle.ContentAvailWidth() - reserve;
        float h = ConsoleStyle.FrameHeightPx;
        float2 at = ImGui.GetCursorScreenPos();
        // The form under PLAN stands below a picture that fills the page, so one frame after a press, when the form has a height, the body scrolls the row up to its middle.
        if (_revealPlan && id == FlightView.PlanFoldId)
        {
            _revealPlan = false;
            float past = PlanFormBeside ? 0f : at.Y - ImGui.GetWindowPos().Y - ImGui.GetWindowHeight() * 0.5f;
            if (past > 0f)
                ImGui.SetScrollY(ImGui.GetScrollY() + past);
        }
        bool hit = ImGui.InvisibleButton("##AfcFold" + id, new float2(Math.Max(1f, width), h));
        NoteRow(id, at, new float2(width, h));
        bool hovered = ImGui.IsItemHovered();
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        if (hovered)
            dl.AddRectFilled(at, at + new float2(width, h), ConsoleStyle.ApplyAlpha(ConsoleStyle.RowHoverWashU32));

        ConsoleStyle.PushLabelFont();
        float fontSize = ImGui.GetFontSize();
        uint chevronInk = ConsoleStyle.ApplyAlpha(hovered ? ConsoleStyle.TextPrimaryU32 : ConsoleStyle.TextLabelU32);
        uint labelInk = ink is float4 given ? ConsoleStyle.ApplyAlpha(ImGui.ColorConvertFloat4ToU32(in given)) : chevronInk;
        float reach = fontSize * 0.22f;
        var c = new float2(at.X + reach, at.Y + h * 0.5f);
        float thick = Math.Max(1f, 1.5f * ImGuiHelper.InterfaceScale);
        if (open)
        {
            dl.AddLine(c + new float2(-reach, -reach * 0.5f), c + new float2(0f, reach * 0.5f), chevronInk, thick);
            dl.AddLine(c + new float2(0f, reach * 0.5f), c + new float2(reach, -reach * 0.5f), chevronInk, thick);
        }
        else
        {
            dl.AddLine(c + new float2(-reach * 0.5f, -reach), c + new float2(reach * 0.5f, 0f), chevronInk, thick);
            dl.AddLine(c + new float2(reach * 0.5f, 0f), c + new float2(-reach * 0.5f, reach), chevronInk, thick);
        }
        float textX = at.X + reach * 2f + ConsoleWidgets.RegionHeaderRuleGapPx;
        float rule = ConsoleWidgets.RegionHeaderRuleGapPx;
        float right = at.X + width;
        (string name, string detail) = FoldRules.Split(label);
        string shownName = FitEllipsis(name, right - textX - rule);
        float end = textX + ImGui.CalcTextSize(shownName).X;
        float column = Math.Max(end, textX + ImGui.CalcTextSize(PageText.HowItFlies).X) + FoldDetailGap;
        if (shownName.Length > 0)
            dl.AddText(new float2(textX, at.Y + (h - fontSize) * 0.5f), labelInk, shownName);
        ConsoleStyle.PopFont();
        bool cut = shownName != name;
        if (detail.Length > 0 && !open)
        {
            PushSmall();
            string shownDetail = FoldRules.FitSummary(detail, right - column - rule, MeasureText);
            if (shownDetail.Length > 0)
            {
                uint detailInk = ink != null ? labelInk : ConsoleStyle.ApplyAlpha(hovered ? ConsoleStyle.TextLabelU32 : ConsoleStyle.TextMutedU32);
                dl.AddText(new float2(column, at.Y + (h - ImGui.GetFontSize()) * 0.5f), detailInk, shownDetail);
                end = column + ImGui.CalcTextSize(shownDetail).X;
            }
            cut = cut || shownDetail != detail;
            ConsoleStyle.PopFont();
        }
        if (right - end - rule >= 24f * ImGuiHelper.InterfaceScale)
            dl.AddLine(new float2(end + rule, c.Y), new float2(right, c.Y), ConsoleStyle.ApplyAlpha(ConsoleStyle.HairlineU32), ConsoleStyle.WindowBorderThicknessPx);

        string card = cut ? (tip.Length > 0 ? label + "\n\n" + tip : label) : tip;
        if (hovered && card.Length > 0)
            Card(name, card);
        if (hit)
        {
            open = !open;
            _folds[id] = open;
            ShapeChanged = true;
            _revealPlan = open && id == FlightView.PlanFoldId;
        }
        return open;
    }

    public static void FoldEmpty(string label, string tip)
    {
        float width = ConsoleStyle.ContentAvailWidth();
        float h = ConsoleStyle.FrameHeightPx;
        float2 at = ImGui.GetCursorScreenPos();
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        ConsoleStyle.PushLabelFont();
        float fontSize = ImGui.GetFontSize();
        float textX = at.X + fontSize * 0.44f + ConsoleWidgets.RegionHeaderRuleGapPx;
        (string name, string detail) = FoldRules.Split(label);
        float column = Math.Max(textX + ImGui.CalcTextSize(name).X, textX + ImGui.CalcTextSize(PageText.HowItFlies).X) + FoldDetailGap;
        uint ink = ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32, DimScope.Alpha);
        dl.AddText(new float2(textX, at.Y + (h - fontSize) * 0.5f), ink, name);
        ConsoleStyle.PopFont();
        if (detail.Length > 0)
        {
            PushSmall();
            dl.AddText(new float2(column, at.Y + (h - ImGui.GetFontSize()) * 0.5f), ink, FitEllipsis(detail, at.X + width - column));
            ConsoleStyle.PopFont();
        }
        ImGui.Dummy(new float2(width, h));
        if (tip.Length > 0 && ImGui.IsItemHovered())
            Card(name, tip);
    }

    private static readonly Func<string, float> MeasureText = static text => ImGui.CalcTextSize(text).X;

    private static bool _revealPlan;

    public static bool PlanFormBeside;

    public static void PartsFold(string name, string id, ReadOnlySpan<FoldPart> parts, int changed = 0, bool openByDefault = false, string tip = "", string saying = "", float4? ink = null)
    {
        bool shows = FoldRules.HasRows(parts);
        NoteShape("AfcFold" + id, shows ? 1 : 0);
        if (!shows)
            return;
        string card = saying.Length > 0 ? (tip.Length > 0 ? tip + "\n\n" : "") + PageText.FoldHolds(FoldRules.Contents(parts)) : tip;
        if (!Fold(saying.Length > 0 ? saying : FoldRules.Label(name, parts, changed), id, openByDefault, card, ink))
            return;
        ImGui.PushID(id);
        try
        {
            foreach (FoldPart part in parts)
                if (part.Shows)
                    part.Draw?.Invoke();
        }
        finally
        {
            ImGui.PopID();
        }
    }

    public static bool ButtonRow(string label, string verb, string id, string tip = "")
    {
        float w = ConsoleWidgets.ButtonWidth(verb.AsSpan());
        BeginRow(label.AsSpan(), w);
        RowRight(w);
        bool hit = ConsoleWidgets.Button(verb.AsSpan(), id.AsSpan(), default);
        if (tip.Length > 0 && ConsoleWidgets.RowHovered)
            Card(label, tip);
        EndRow();
        return hit;
    }

    public static bool ButtonRight(string label, string id)
    {
        ImGui.SetCursorPosX(ImGui.GetCursorPosX() + Math.Max(0f, ConsoleStyle.ContentAvailWidth() - ConsoleWidgets.ButtonWidth(label.AsSpan())));
        return ConsoleWidgets.Button(label.AsSpan(), id.AsSpan(), default);
    }

    public static void ScrollEdges(uint? panel = null)
    {
        float maxY = ImGui.GetScrollMaxY();
        if (maxY <= 0.5f)
            return;
        float y = ImGui.GetScrollY();
        float2 pos = ImGui.GetWindowPos();
        float h = 14f * ImGuiHelper.InterfaceScale;
        float right = ImGui.GetCursorScreenPos().X + ConsoleStyle.ContentAvailWidth();
        float bottom = pos.Y + ImGui.GetWindowHeight();
        uint solid = ConsoleStyle.ApplyAlpha(panel ?? ConsoleStyle.PanelU32);
        uint clear = solid & 0xFFFFFF;
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        if (y > 0.5f)
            dl.AddRectFilledMultiColor(pos, new float2(right, pos.Y + h), solid, solid, clear, clear);
        if (y < maxY - 0.5f)
            dl.AddRectFilledMultiColor(new float2(pos.X, bottom - h), new float2(right, bottom), clear, clear, solid, solid);
    }

    // The track of the stock ConsoleWidgets.Checkbox, as fractions of the row height it sits in.
    private const float TrackW = 1.1538461f, TrackH = 0.61538464f, KnobW = 11f / 26f, KnobH = 0.3846154f, KnobInset = 1f / 13f;

    /// <summary>The width <see cref="Switch"/> takes, with room for "OFF" so the row does not move when the state changes.</summary>
    public static float SwitchWidth(string label, float height = 0f)
    {
        float h = height > 0f ? height : ConsoleStyle.FrameHeightPx;
        float gap = ConsoleWidgets.CheckboxStateGapPx;
        return (label.Length > 0 ? LabelWidth(label) + gap : 0f) + h * TrackW + gap + ValueWidth("OFF");
    }

    /// <summary>Rows keep the stock Checkbox itself; this one serves picture headers, where no row exists.</summary>
    public static bool Switch(string label, string id, ref bool on, string tip, string whenTag = "", string disabledReason = "",
        float height = 0f)
    {
        float h = height > 0f ? height : ConsoleStyle.FrameHeightPx;
        bool disabled = disabledReason.Length > 0;
        float gap = ConsoleWidgets.CheckboxStateGapPx;
        float2 pos = ImGui.GetCursorScreenPos();
        float labelW = label.Length > 0 ? LabelWidth(label) + gap : 0f;
        float stateW = ValueWidth("OFF");
        var track = new float2(h * TrackW, h * TrackH);
        float clickW = labelW + track.X + gap + stateW;
        NoteRow(id, pos, new float2(clickW, h));

        bool hit;
        bool hovered;
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        using (new ImGuiDisabledScope(disabled))
        {
            hit = ImGui.InvisibleButton(id, new float2(clickW, h));
            hovered = ImGui.IsItemHovered(ImGuiHoveredFlags.AllowWhenDisabled) && !disabled;
            if (hit && !disabled)
                on = !on;

            if (label.Length > 0)
            {
                ConsoleStyle.PushLabelFont();
                dl.AddText(pos + new float2(0f, (h - ImGui.GetFontSize()) * 0.5f), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextLabelU32), label);
                ConsoleStyle.PopFont();
            }
            var tMin = new float2(pos.X + labelW, pos.Y + (h - track.Y) * 0.5f);
            float2 tMax = tMin + track;
            dl.AddRectFilled(tMin, tMax, ConsoleStyle.ApplyAlpha(ConsoleStyle.InsetU32), ConsoleStyle.FrameRoundingPx);
            uint border = !on ? (hovered ? ConsoleStyle.TextMutedU32 : ConsoleStyle.FrameU32) : (hovered ? ConsoleStyle.TextPrimaryU32 : ConsoleStyle.TextMutedU32);
            dl.AddRect(tMin, tMax, ConsoleStyle.ApplyAlpha(border), ConsoleStyle.FrameRoundingPx, ImDrawFlags.None, ConsoleStyle.WindowBorderThicknessPx);
            dl.AddLine(new float2(tMin.X + ConsoleStyle.FrameRoundingPx, tMin.Y + ConsoleStyle.BezelInsetPx),
                new float2(tMax.X - ConsoleStyle.FrameRoundingPx, tMin.Y + ConsoleStyle.BezelInsetPx),
                ConsoleStyle.ApplyAlpha(ConsoleStyle.TopHighlightU32), ConsoleStyle.WindowBorderThicknessPx);
            float inset = h * KnobInset;
            var knob = new float2(h * KnobW, h * KnobH);
            float2 kMin = tMin + new float2(on ? track.X - inset - knob.X : inset, inset);
            uint knobCol = on ? ConsoleStyle.TextPrimaryU32 : hovered ? ConsoleStyle.TextLabelU32 : ConsoleStyle.TextMutedU32;
            dl.AddRectFilled(kMin, kMin + knob, ConsoleStyle.ApplyAlpha(knobCol), ConsoleStyle.FrameRoundingPx * 0.5f);

            string state = on ? "ON" : "OFF";
            ConsoleStyle.PushValueFont();
            float w = ImGui.CalcTextSize(state).X;
            uint stateCol = !on ? (hovered ? ConsoleStyle.TextLabelU32 : ConsoleStyle.TextMutedU32) : (hovered ? ConsoleStyle.TextPrimaryU32 : ConsoleStyle.TextLabelU32);
            dl.AddText(new float2(tMax.X + gap + (stateW - w), pos.Y + (h - ImGui.GetFontSize()) * 0.5f), ConsoleStyle.ApplyAlpha(stateCol), state);
            ConsoleStyle.PopFont();
        }
        if (Hovering(pos, pos + new float2(clickW, h)))
        {
            string text = disabled ? disabledReason + (tip.Length > 0 ? "\n\n" + tip : "") : tip;
            if (whenTag.Length > 0)
                text += (text.Length > 0 ? "\n\n" : "") + whenTag;
            if (text.Length > 0)
                Card(label, text);
        }
        return hit && !disabled;
    }

    public static float ChipToButtonGap => 12f * ImGuiHelper.InterfaceScale;

    private static void PushChipFont() => ConsoleStyle.PushSignatureFont(0.9f);

    /// <summary>The width of the chip or chips <see cref="OwnerChipAt"/> draws for <paramref name="text"/>.</summary>
    public static float OwnerChipWidth(Owner owner, string text) => Chips(default, default, 0f, owner, text, false, draw: false);

    /// <summary>AFC fills in the AFC colour, STOCK in the stock plate colour, or at the level of a locked form with <paramref name="dim"/>, and YOU FLY in the bright text colour.</summary>
    public static float OwnerChipAt(ImDrawListPtr dl, float2 at, float rowH, Owner owner, string text, bool dim = false) => Chips(dl, at, rowH, owner, text, dim, draw: true);

    private static float Chips(ImDrawListPtr dl, float2 at, float rowH, Owner owner, string text, bool dim, bool draw)
    {
        if (text.Length == 0)
            return 0f;
        int split = owner == Owner.AfcPlansStockFlies ? text.IndexOf(FlownBy.PlanJoin, StringComparison.Ordinal) : -1;
        if (split > 0)
        {
            float pad = 4f * ImGuiHelper.InterfaceScale;
            float first = OneChip(dl, at, rowH, Owner.Afc, text.Substring(0, split), dim, draw);
            PushChipFont();
            float2 plus = ImGui.CalcTextSize("+");
            if (draw)
                dl.AddText(at + new float2(first + pad, (rowH - plus.Y) * 0.5f), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32), "+");
            ConsoleStyle.PopFont();
            float second = OneChip(dl, at + new float2(first + pad * 2f + plus.X, 0f), rowH, Owner.Stock, text.Substring(split + FlownBy.PlanJoin.Length), dim, draw);
            return first + pad * 2f + plus.X + second;
        }
        return OneChip(dl, at, rowH, owner, text, dim, draw);
    }

    private static float OneChip(ImDrawListPtr dl, float2 at, float rowH, Owner owner, string text, bool dim, bool draw)
    {
        float pad = 4f * ImGuiHelper.InterfaceScale;
        PushChipFont();
        try
        {
            float2 size = ImGui.CalcTextSize(text);
            float chipH = Math.Min(rowH, size.Y + 1.5f * ImGuiHelper.InterfaceScale);
            var min = new float2(at.X, at.Y + (rowH - chipH) * 0.5f);
            var max = min + new float2(size.X + pad * 2f, chipH);
            var textAt = new float2(min.X + pad, at.Y + (rowH - size.Y) * 0.5f);
            if (!draw)
                return max.X - min.X;
            switch (owner)
            {
                case Owner.Afc:
                    dl.AddRectFilled(min, max, ConsoleStyle.ApplyAlpha(Palette.Afc.U32));
                    dl.AddText(textAt, ConsoleStyle.ApplyAlpha(((Rgb)ColorRules.InkOn(ColorRules.Afc)).U32), text);
                    break;
                case Owner.Stock:
                    dl.AddRectFilled(min, max, ConsoleStyle.ApplyAlpha(dim ? Palette.StockDim.U32 : Palette.Stock.U32));
                    dl.AddText(textAt, ConsoleStyle.ApplyAlpha(((Rgb)ColorRules.InkOn(ColorRules.Stock)).U32), text);
                    break;
                case Owner.Player:
                    dl.AddRectFilled(min, max, ConsoleStyle.ApplyAlpha(ConsoleStyle.TextPrimaryU32));
                    dl.AddText(textAt, ConsoleStyle.ApplyAlpha(ConsoleStyle.PanelU32), text);
                    break;
                default:
                    dl.AddText(textAt, ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32), text);
                    break;
            }
            return max.X - min.X;
        }
        finally
        {
            ConsoleStyle.PopFont();
        }
    }

    /// <summary>An owner chip at the cursor, on a row of the frame height, as an item. Returns its width.</summary>
    public static float OwnerChip(Owner owner, string text)
    {
        float2 at = ImGui.GetCursorScreenPos();
        float h = ConsoleStyle.FrameHeightPx;
        float w = OwnerChipAt(ImGui.GetWindowDrawList(), at, h, owner, text);
        ImGui.Dummy(new float2(Math.Max(1f, w), h));
        if (owner == Owner.Stock)
            StockChipCard(at, new float2(w, h));
        return w;
    }

    public static void StockChipCard(float2 at, float2 size)
    {
        if (Hovering(at, at + size))
            Card("STOCK", PageText.StockChipTip);
    }

    /// <summary>
    /// An owner chip right after the label of the row ConsoleWidgets.BeginRow just began, where stock draws the label
    /// (ConsoleWidgets.BeginRowCore: RowLabelInsetPx from the row start, in the label font). It marks a row that also sets
    /// something of the stock game, such as the stock target of the craft.
    /// </summary>
    public static void RowChip(string label, Owner owner, string text)
    {
        var at = new float2(ConsoleWidgets.RowMin.X + ConsoleWidgets.RowLabelInsetPx + LabelWidth(label) + 6f * ImGuiHelper.InterfaceScale,
            ConsoleWidgets.RowMin.Y);
        float w = OwnerChipAt(ImGui.GetWindowDrawList(), at, ConsoleStyle.FrameHeightPx, owner, text);
        if (owner == Owner.Stock)
            StockChipCard(at, new float2(w, ConsoleStyle.FrameHeightPx));
    }

    public static void ChangedDot() => ChangedDotAfter(_rowLabelEnd);

    public static void ChangedDotAfter(float nameEnd)
    {
        float r = 2.5f * ImGuiHelper.InterfaceScale;
        var c = new float2(nameEnd + r * 2.5f, ConsoleWidgets.RowMin.Y + ConsoleStyle.FrameHeightPx * 0.5f);
        ImGui.GetWindowDrawList().AddCircleFilled(c, r, ConsoleStyle.ApplyAlpha(Palette.Changed.U32));
    }

    public static bool CheckboxRow(ReadOnlySpan<char> label, ReadOnlySpan<char> id, ref bool value,
        ReadOnlySpan<char> tooltip = default, bool stockChip = false)
    {
        BeginRow(label, SwitchWidth(""));
        if (stockChip)
            RowChip(label.ToString(), Owner.Stock, "STOCK");
        bool changed = RowCheckbox(id, ref value);
        if (tooltip.Length > 0 && ConsoleWidgets.RowHovered)
            Card(label.ToString(), tooltip.ToString());
        EndRow();
        return changed;
    }

    public static bool ComboRow<T>(ReadOnlySpan<char> label, ReadOnlySpan<char> id,
        ref T value, IReadOnlyList<T> options) where T : IComboable
    {
        BeginRow(label);
        bool changed = ComboControl(id, ref value, options);
        EndRow();
        return changed;
    }

    private static bool ComboControl<T>(ReadOnlySpan<char> id, ref T value,
        IReadOnlyList<T> options) where T : IComboable
    {
        bool changed = false;
        int current = -1;
        for (int i = 0; i < options.Count; i++)
        {
            if (options[i].GetKey().Equals(value.GetKey(), StringComparison.OrdinalIgnoreCase))
            {
                current = i;
                value = options[i];
                break;
            }
        }
        if (current < 0 && options.Count > 0)
            current = 0;

        string preview = current >= 0 ? options[current].GetName() : "N/A";
        if (!BeginCombo(id, preview))
            return false;

        for (int i = 0; i < options.Count; i++)
        {
            T option = options[i];
            bool selected = option.GetKey().Equals(value.GetKey(), StringComparison.OrdinalIgnoreCase);
            if (ImGui.Selectable(option.GetName(), selected, ImGuiSelectableFlags.None, (float2?)null))
            {
                value = option;
                changed = true;
            }
        }
        ConsoleWidgets.EndComboControl();
        return changed;
    }

    /// <summary>String-list combo. Returns the newly selected index, or -1 when nothing was picked
    /// this frame.</summary>
    public static int ComboRow(ReadOnlySpan<char> label, ReadOnlySpan<char> id,
        int activeIndex, IReadOnlyList<string> options)
    {
        BeginRow(label);
        int picked = -1;
        string preview = (activeIndex >= 0 && activeIndex < options.Count)
            ? options[activeIndex] : "N/A";
        if (BeginCombo(id, preview))
        {
            for (int i = 0; i < options.Count; i++)
            {
                if (ImGui.Selectable(options[i], i == activeIndex, ImGuiSelectableFlags.None, (float2?)null))
                    picked = i;
            }
            ConsoleWidgets.EndComboControl();
        }
        EndRow();
        return picked;
    }

    private static readonly Dictionary<string, string> _timeEdits = new();

    /// <summary>A row with a span of time in the form of Fmt.Time, in the stock text line.</summary>
    public static bool TimeRow(string label, string id, ref double seconds, string tip = "")
    {
        BeginRow(label.AsSpan());
        string text = _timeEdits.TryGetValue(id, out string? held) ? held : Fmt.Time(seconds);
        bool entered = ConsoleWidgets.TextLine(id.AsSpan(), ref text, pending: false, out bool editing);
        if (tip.Length > 0 && ConsoleWidgets.RowHovered)
            Card(label, tip);
        EndRow();
        if (editing && !entered)
            _timeEdits[id] = text;
        else
            _timeEdits.Remove(id);
        if (!entered || !Fmt.TryParseTime(text, out double parsed))
            return false;
        seconds = parsed;
        return true;
    }

    private static float ViewChoicePad => 5f * ImGuiHelper.InterfaceScale;

    public static float ViewChoiceWidth(string label, ReadOnlySpan<string> options)
    {
        float w = SmallWidth(label) + ViewChoicePad;
        foreach (string option in options)
            w += SmallWidth(option) + ViewChoicePad * 2f;
        return w;
    }

    public static int ViewChoice(string label, string id, ReadOnlySpan<string> options, int active, string tip)
    {
        float2 origin = ImGui.GetCursorScreenPos();
        float h = ConsoleStyle.FrameHeightPx;
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        int picked = -1;
        bool hoveredAny = false;
        PushSmall();
        try
        {
            float font = ImGui.GetFontSize();
            float x = origin.X;
            dl.AddText(new float2(x, origin.Y + (h - font) * 0.5f), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32), label);
            x += ImGui.CalcTextSize(label).X + ViewChoicePad;
            for (int i = 0; i < options.Length; i++)
            {
                float w = ImGui.CalcTextSize(options[i]).X + ViewChoicePad * 2f;
                var pos = new float2(x, origin.Y);
                var size = new float2(w, h);
                ImGui.SetCursorScreenPos(pos);
                if (ImGui.InvisibleButton(id + "##Opt" + i, size))
                    picked = i;
                NoteRow(id, i, pos, size);
                bool hovered = ImGui.IsItemHovered();
                hoveredAny |= hovered;
                bool isActive = i == active;
                Plate(dl, pos, size, isActive, hovered, ImDrawFlags.None, ConsoleStyle.ApplyAlpha(ConsoleStyle.ActiveToolU32), ConsoleStyle.WindowBorderThicknessPx);
                dl.AddText(new float2(x + ViewChoicePad, origin.Y + (h - font) * 0.5f), ConsoleStyle.ApplyAlpha(isActive ? ConsoleStyle.TextPrimaryU32 : ConsoleStyle.TextLabelU32), options[i]);
                x += w;
            }
        }
        finally
        {
            ConsoleStyle.PopFont();
        }
        if (hoveredAny && tip.Length > 0)
            Card(label, tip);
        return picked;
    }

    /// <summary>An InputDouble rather than a ConsoleWidgets drag control, because these fields take
    /// typed absolute values (altitudes, angles) that a drag range cannot express.</summary>
    public static bool InputDoubleRow(ReadOnlySpan<char> label, ImString id, ref double value,
        double step, double stepFast, ImString format, ReadOnlySpan<char> tooltip = default, string unit = "")
    {
        if (unit.Length > 0)
        {
            float room = UnitRoom(unit);
            BeginRow(label, StepFieldWidth + room);
            float rowEnd = ImGui.GetCursorScreenPos().X + ConsoleWidgets.RowControlWidth;
            float field = Math.Max(StepFieldWidth, Math.Min(ConsoleWidgets.RowControlWidth, rowEnd - _rowLabelEnd - ConsoleWidgets.MinControlGapPx) - room);
            RowFieldUnit(field, unit);
        }
        else
        {
            BeginRow(label);
            ImGui.SetNextItemWidth(ConsoleWidgets.RowControlWidth);
        }
        bool changed = ImGui.InputDouble(id, ref value, step, stepFast, format,
            ImGuiInputTextFlags.CharsDecimal);
        if (tooltip.Length > 0 && ConsoleWidgets.RowHovered)
            Card(label.ToString(), tooltip.ToString());
        EndRow();
        return changed;
    }
}
