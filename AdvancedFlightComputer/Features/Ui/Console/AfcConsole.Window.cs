using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Transfer;
using AdvancedFlightComputer.Features.Ui.Console.Pages;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

internal static partial class AfcConsole
{
    private const string WindowId = "afc-console";
    private const string WindowTitle = "ADVANCED FLIGHT COMPUTER";
    private const string WindowSignature = "AFC";
    private const string WindowKey = "console";
    private const string StripId = "afc-strip";

    // A shape change opens the window to shrink for a few frames, because a page can need a frame or two to settle on its new rows.
    private const int ShrinkFrames = 3;

    internal static Action<string>? BodyProbe;

    /// <summary>Lets a test press a footer verb, called with the commit of the frame.</summary>
    internal static Func<CommitSpec, Action?>? CommitProbe;

    // NaN until the first frame measured it.
    private static float _height = float.NaN;
    private static float _pageContentH;
    private static float _drawerContentH;
    private static float _answerH;
    private static float _pictureShrink;
    private static float _pictureGrowth;
    private static float _picturePaneH;
    private static float _inputsPaneH;
    private static bool _wideShown;
    private static PaneMode _paneMode;
    private static PaneSide _paneNow;
    private static PaneSplit _split;
    private static float _paneX;
    private static float _paneY;
    private static float _paneH;
    private static bool _panesFit;
    private static bool _inputsEntered;
    private static bool _inputsScroll;
    private static bool _inputsTopPending;
    private static bool _pageScrolls;
    private static bool _drawerScrolls;
    private static bool _footerGone;
    private static bool _handleHeld;
    private static bool _pageTopPending;
    private static int _shrinkFrames;
    private static int _shapeKey;
    private static int _pageTopKey;
    private static string? _shapeSubject;

    private static void ResetFollowing()
    {
        _height = float.NaN;
        _pageContentH = 0f;
        _drawerContentH = 0f;
        _answerH = 0f;
        _pictureShrink = 0f;
        _pictureGrowth = 0f;
        _picturePaneH = 0f;
        _inputsPaneH = 0f;
        _wideShown = false;
        _paneMode = PaneMode.Full;
        _paneNow = PaneSide.Picture;
        _panesFit = false;
        _inputsEntered = false;
        _inputsScroll = false;
        _inputsTopPending = false;
        _pageScrolls = false;
        _drawerScrolls = false;
        _footerGone = false;
        _handleHeld = false;
        _pageTopPending = false;
        _shrinkFrames = 0;
        _shapeKey = 0;
        _pageTopKey = 0;
        _shapeSubject = null;
        ConsoleUi.ShapeChanged = false;
        Dock.Reset();
    }

    private static void DrawWindow()
    {
        bool wide = WideOn(Page);
        if (wide != _wideShown)
        {
            _wideShown = wide;
            _applySize = true;
        }
        float2 size = Dock.Place(WindowKey, _wideShown, _applySize, _height);
        _applySize = false;

        bool open = true;
        string tag = Frame.Subject?.Id ?? "";
        // No context tag goes to BeginWindow, because ConsoleStyle.DrawTitlebarText would draw it where the WIDE icon stands.
        if (!ConsoleStyle.BeginWindow(WindowId.AsSpan(), WindowTitle.AsSpan(), WindowSignature.AsSpan(),
                ref open, size, ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoScrollWithMouse, ImGuiCond.Always,
                pinToMainViewport: false))
            return;

        Action? commit = null;
        try
        {
            if (!open)
            {
                // A close while AFC flies the craft leaves the head on screen as the strip.
                Close();
                // ConsoleStyle.BeginWindow ended with a cursor move to the body, and ImGui raises an error at End when no item follows that move.
                ImGui.Dummy(float2.Zero);
                return;
            }
            Dock.Constrain(_wideShown);
            Dock.Record(WindowKey, _wideShown);
            TrackResizeHandle(size.Y);
            ConsoleUi.CardsBeside(ImGui.GetWindowPos().X, ImGui.GetWindowPos().X + ImGui.GetWindowSize().X);
            DrawTitleTools(tag);
            commit = DrawFrame();
        }
        finally
        {
            ConsoleUi.CardsFree();
            ConsoleStyle.EndWindow();
        }

        if (commit != null)
        {
            ConsoleHead.DropCue();
            commit();
        }

        string navigation = Guidance.GuidanceWindow.PendingNavigation;
        if (navigation.Length > 0)
        {
            Guidance.GuidanceWindow.PendingNavigation = "";
            Navigate(navigation);
        }
    }

    // WIDE gives room to a picture and what stands beside it, so a page of rows keeps the narrow window (DockLayout.PageTakesWide).
    private static bool WideOn(ConsolePage page) => Wide && PageTakesWide(page);

    private static bool PageTakesWide(ConsolePage page)
    {
        Vehicle? craft = Frame.Subject;
        bool works = craft != null && (page is not (ConsolePage.Launch or ConsolePage.Land) || Frame.GuidanceOn);
        bool burns = page == ConsolePage.Rcs && craft != null
            && (BurnModeInput.NextNode(craft) != null || (SharedVehicleHooks.MultiPassEnabled && MultiPassRegistry.TryGet(craft.Id, out MultiPassExecution? _)));
        return DockLayout.PageTakesWide(page, works, burns);
    }

    private static void TrackResizeHandle(float asked)
    {
        float now = ImGui.GetWindowSize().Y;
        if (MathF.Abs(now - asked) > 0.5f)
        {
            _handleHeld = true;
            _height = now;
            Dock.RecordMaximumHeight(WindowKey, now);
        }
        else if (_handleHeld && !ImGui.IsMouseDown(ImGuiMouseButton.Left))
        {
            _handleHeld = false;
            _shrinkFrames = ShrinkFrames;
        }
    }

    private static void DrawTitleTools(string tag)
    {
        ConsoleLayout layout = ConsoleStyle.CurrentLayout;
        float box = ConsoleStyle.CloseButtonSizePx;
        float gap = ConsoleStyle.TitlebarContextTagGapPx;
        float iconX = layout.TitlebarPos.X + layout.TitlebarSize.X - ConsoleStyle.TitlebarHorizontalPaddingPx - box - gap * 0.5f - box;
        DrawWideIcon(new float2(iconX, layout.TitlebarPos.Y + (layout.TitlebarSize.Y - box) * 0.5f), box);
        if (tag.Length > 0)
            DrawTitleTag(tag, in layout, iconX - gap, gap);
    }

    // The frame is the one of ConsoleStyle.DrawCloseButton.
    private static void DrawWideIcon(float2 min, float box)
    {
        bool staysNarrow = !PageTakesWide(Page);
        var size = new float2(box, box);
        ImGui.SetCursorScreenPos(min);
        bool hit = ImGui.InvisibleButton("##AfcWide", size);
        ConsoleUi.NoteRow("AfcWide", min, size);
        bool hovered = ImGui.IsItemHovered();
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        float2 max = min + size;
        dl.AddRect(min, max, ConsoleStyle.ApplyAlpha(hovered && !staysNarrow ? ConsoleStyle.TextMutedU32 : ConsoleStyle.FrameU32), ConsoleStyle.FrameRoundingPx, ImDrawFlags.None, ConsoleStyle.WindowBorderThicknessPx);

        uint ink = staysNarrow ? ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32, 0.55f)
            : ConsoleStyle.ApplyAlpha(hovered ? ConsoleStyle.TextPrimaryU32 : ConsoleStyle.TextMutedU32);
        float2 c = min + size * 0.5f;
        float near = box * 0.1f, far = box * 0.3f, rise = box * 0.17f;
        float thick = ConsoleStyle.WindowBorderThicknessPx;
        foreach (float side in new[] { -1f, 1f })
        {
            float tip = c.X + side * (_wideShown ? near : far);
            float tail = c.X + side * (_wideShown ? far : near);
            dl.AddLine(new float2(tail, c.Y - rise), new float2(tip, c.Y), ink, thick);
            dl.AddLine(new float2(tip, c.Y), new float2(tail, c.Y + rise), ink, thick);
        }

        if (hovered)
            ConsoleUi.Card("WIDE", PageText.WideCard(Wide, DockLayout.WideHasPanes(ImGuiHelper.InterfaceScale, ImGui.GetMainViewport().Size.X), staysNarrow));
        if (hit && !staysNarrow)
            SetWide(!Wide);
    }

    // The craft tag ends at rightEnd, on the baseline of the title as ConsoleStyle.DrawTitlebarText sets it, and a long name gets an ellipsis and a card so it cannot run into the title.
    private static void DrawTitleTag(string tag, in ConsoleLayout layout, float rightEnd, float gap)
    {
        ConsoleStyle.PushSignatureFont();
        float signatureW = ImGui.CalcTextSize(WindowSignature).X;
        ConsoleStyle.PopFont();
        ConsoleStyle.PushLabelFont();
        float titleW = ImGui.CalcTextSize(WindowTitle).X;
        float fontSize = ImGui.GetFontSize();
        float baseline = MathF.Round(layout.TitlebarPos.Y + (layout.TitlebarSize.Y - fontSize) * 0.5f + FontAscent());
        ConsoleStyle.PopFont();

        float left = layout.TitlebarPos.X + ConsoleStyle.TitlebarHorizontalPaddingPx + signatureW + ConsoleStyle.TitlebarSignatureGapPx + titleW + gap;
        float room = rightEnd - left;
        if (room <= 0f)
            return;

        ConsoleStyle.PushSignatureFont();
        string shown = ConsoleUi.FitEllipsis(tag, room);
        float w = ImGui.CalcTextSize(shown).X;
        var at = new float2(rightEnd - w, baseline - MathF.Round(FontAscent()));
        ImGui.GetWindowDrawList().AddText(at, ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32), shown);
        float2 end = at + new float2(w, ImGui.GetFontSize());
        ConsoleStyle.PopFont();
        if (shown != tag && ImGui.IsWindowHovered() && ImGui.IsMouseHoveringRect(in at, in end))
            ConsoleUi.Tooltip(tag.AsSpan());
    }

    private static float FontAscent()
    {
        ImFontBakedPtr baked = ImGui.GetFontBaked();
        return baked.IsNull() ? 0f : baked.Ascent;
    }

    // The left edge is the text edge of the stock title strip and footer.
    private static (float X, float W, float Lane) ContentEdges(in ConsoleLayout layout)
    {
        float inset = ConsoleStyle.TitlebarHorizontalPaddingPx;
        float lane = ImGui.GetStyle().ScrollbarSize + ConsoleStyle.ScrollbarGutterPx;
        float w = Math.Max(0f, layout.BodySize.X - inset - lane - ConsoleStyle.WindowBorderThicknessPx);
        return (layout.BodyPos.X + inset, w, lane);
    }

    private static bool PageFlies()
    {
        if (Frame.Subject is not Vehicle subject)
            return false;
        CraftActivity a = Frame.Activity;
        if (Page is ConsolePage.Launch or ConsolePage.Land)
            return Frame.GuidanceOn && (FlightView.PageFlies(Page, LandWay, GuidanceWindow.LandSolverRoute, GuidanceWindow.CurrentAscentChoice(), in a, Frame.TransferPasses)
                || (Page == ConsolePage.Land && FlightView.PickHolds(LandWay, in a)));
        bool ownSource = Page switch
        {
            ConsolePage.Orbit => OrbitToolsPanel.Enabled && (OrbitToolsPanel.Source == null || ReferenceEquals(OrbitToolsPanel.Source, subject)),
            ConsolePage.Transfer => TransferSession.Source == null || ReferenceEquals(TransferSession.Source, subject),
            ConsolePage.Rcs => true,
            _ => false,
        };
        return ownSource && FlightView.PageFlies(Page, LandWay, LandSolver.Entry, AscentChoice.CatsUpfg, in a, Frame.TransferPasses);
    }

    // Head, chooser, page, answer and footer are sequential bodies, never nested, as in GameSettings.OnDrawUi, and the drawer is one more body that lies over the page.
    private static Action? DrawFrame()
    {
        ConsoleLayout layout = ConsoleStyle.CurrentLayout;
        float pad = ConsoleStyle.ContentPaddingPx;
        float gap = pad * 0.5f;
        float scale = ImGuiHelper.InterfaceScale;
        float windowLeft = layout.WindowPos.X, windowRight = layout.WindowPos.X + layout.WindowSize.X;
        float border = ConsoleStyle.WindowBorderThicknessPx;
        CommitSpec commit = CommitSpec.None;
        bool answerShown = false;
        bool footerGone = false;

        bool flying = PageFlies();

        ConsoleStyle.PushWidgetStyle();
        try
        {
            (float x, float w, float lane) = ContentEdges(in layout);
            float y = layout.BodyPos.Y + pad;

            float headH = ConsoleHead.Height(Frame, inStrip: false, ShowFirstOpenNotice, w);
            Body("AfcConsoleHead", x, y, w, headH, ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoScrollWithMouse,
                () => ConsoleHead.Draw(Frame, inStrip: false, ShowFirstOpenNotice));
            y += headH + gap;

            bool band = !flying;
            float chooserH = PageChooser.HeightFor(band);
            float drawerTop = y + PageChooser.TabsHeight + border;
            if (band)
                PageBand.DrawStrip(ImGui.GetWindowDrawList(), windowLeft + border, windowRight - border, y + PageChooser.TabsHeight, PageBand.Height);
            else
                PageBand.DrawRule(ImGui.GetWindowDrawList(), windowLeft + border, windowRight - border, y + PageChooser.TabsHeight);
            Body("AfcConsoleChooser", x, y, w, chooserH, ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoScrollWithMouse,
                () => PageChooser.Draw(Frame, band));
            y += chooserH + gap;

            // A tab picked in this frame can change what the page is.
            flying = PageFlies();
            float footerTop = layout.FooterPos.Y;
            float pageBottom = _footerGone ? layout.WindowPos.Y + layout.WindowSize.Y - pad : footerTop - _answerH - gap;
            float pageH = Math.Max(0f, pageBottom - y);
            _panesFit = DockLayout.WideHasPanes(scale, ImGui.GetMainViewport().Size.X);
            // A tab picked in this frame can ask for another width, and the window has it from the next frame on.
            _paneMode = DockLayout.PanesFor(_wideShown && WideOn(Page), _panesFit, flying, ConsoleUi.FoldIsOpen(FlightView.PlanFoldId));
            _split = DockLayout.SplitPanes(w + lane, lane, scale);
            _paneNow = PaneSide.Picture;
            _paneX = x;
            _paneY = y;
            _paneH = pageH;
            _inputsEntered = false;
            bool drawerBeside = _wideShown && _panesFit;
            ConsoleUi.PlanFormBeside = drawerBeside;
            float pictureBasis = flying || _paneMode == PaneMode.Full ? w : _split.LeftContent;
            Body("AfcConsolePage", x, y, _paneMode == PaneMode.Full ? w + lane : _split.LeftBody, pageH, ImGuiWindowFlags.AlwaysVerticalScrollbar, () =>
            {
                if (_pageTopPending)
                {
                    _pageTopPending = false;
                    _inputsTopPending = _paneMode == PaneMode.Two;
                    ImGui.SetScrollY(0f);
                }
                commit = TakeOverAsk.Apply(DrawPage(flying, pictureBasis), DateTime.UtcNow);
                FinishPanes();
            }, hideBar: !_pageScrolls || (DrawerOpen && !drawerBeside), rows: true);
            ConsoleUi.CardsLeftEdge(windowLeft);

            footerGone = commit.SaysNothing || FlightView.FooterGoes(flying, commit.Outcome.Length > 0 || commit.HoldsFooter);
            ConsoleHead.PageVerbLit = !footerGone && (commit.Lit || (commit.Secondary is { Lead: true, Quiet: false, Enabled: true }));
            float answerW = ConsoleFooter.Width(in layout);
            float answerH = footerGone ? 0f : ConsoleFooter.AnswerHeight(commit, answerW);
            if (answerH > 0f)
            {
                answerShown = true;
                float answerTop = footerTop - answerH;
                ImDrawListPtr dl = ImGui.GetWindowDrawList();
                dl.AddRectFilled(new float2(windowLeft, answerTop), new float2(windowRight, footerTop), ConsoleStyle.ApplyAlpha(ConsoleStyle.PanelRaisedU32));
                dl.AddLine(new float2(windowLeft, answerTop), new float2(windowRight, answerTop), ConsoleStyle.ApplyAlpha(ConsoleStyle.HairlineU32), border);
                CommitSpec shown = commit;
                Body("AfcConsoleAnswer", layout.FooterPos.X + ConsoleStyle.FooterHorizontalPaddingPx, answerTop, answerW, answerH,
                    ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoScrollWithMouse, () => ConsoleFooter.DrawAnswer(shown));
            }
            _answerH = answerH;
            _footerGone = footerGone;

            float under = footerGone ? pad : gap + answerH + ConsoleStyle.FooterHeight;
            float rows = y - layout.WindowPos.Y + under;
            float drawerWanted = 0f;
            if (DrawerOpen)
            {
                if (drawerBeside)
                    ConsoleUi.CardsLeftEdge(x + _split.LeftBody - gap);
                drawerWanted = (drawerBeside
                    ? DrawDrawer(x + _split.LeftBody, drawerTop, _split.RightBody, pageBottom, x + _split.LeftBody - gap, windowRight)
                    : DrawDrawer(x, drawerTop, w + lane, pageBottom, windowLeft, windowRight)) - layout.WindowPos.Y + under;
                ConsoleUi.CardsLeftEdge(windowLeft);
            }

            (float floor, float limit) = Dock.HeightRange(WindowKey, in layout, rows + ConsoleStyle.FrameHeightPx);
            // Read after the page drew, so a PLAN row that opened in this frame already has its room and the picture does not dip.
            if (flying && !Dock.PilotHeight(WindowKey))
                (floor, limit) = Dock.FlyingHeightRange(in layout, limit, !drawerBeside && ConsoleUi.FoldIsOpen(FlightView.PlanFoldId),
                    rows + Math.Min(_pageContentH, DockLayout.MinPictureHeight * scale + ConsoleStyle.FrameHeightPx));
            FollowContent(rows + _pageContentH, rows + _picturePaneH, drawerWanted, floor, limit, w, scale, commit.Verb, flying);
        }
        finally
        {
            ConsoleStyle.PopWidgetStyle();
        }

        Action? clicked = null;
        if (footerGone)
        {
            ConsoleFooter.NoteNoFooter();
            return CommitProbe?.Invoke(commit);
        }
        ConsoleStyle.BeginFooter();
        try
        {
            // The answer and the verb are one panel, so the hairline of the stock footer between them goes.
            if (answerShown)
                ImGui.GetWindowDrawList().AddLine(layout.FooterPos, layout.FooterPos + new float2(layout.FooterSize.X, 0f), ConsoleStyle.ApplyAlpha(ConsoleStyle.PanelRaisedU32), border);
            clicked = ConsoleFooter.DrawCommit(commit);
            if (CommitProbe != null)
                clicked = CommitProbe(commit) ?? clicked;
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"afc-console-footer:{Page}:{ex.GetType().Name}", $"[AFC] AFC console footer failed: {ex}");
        }
        finally
        {
            ConsoleStyle.EndFooter();
        }
        return clicked;
    }

    private static float DrawDrawer(float x, float top, float w, float bottom, float windowLeft, float windowRight)
    {
        float scale = ImGuiHelper.InterfaceScale;
        float border = ConsoleStyle.WindowBorderThicknessPx;
        float most = DockLayout.DrawerMaxHeight * scale;
        float wanted = Math.Min(most, _drawerContentH > 0f ? _drawerContentH : 200f * scale);
        float h = Math.Max(0f, Math.Min(wanted, bottom - top));
        var sheetMin = new float2(windowLeft + border, top);
        var sheetMax = new float2(windowRight - border, top + h);
        float shadow = Math.Clamp(bottom - sheetMax.Y, 0f, SheetShadowPx * scale);
        // The fill runs from window edge to window edge, and it has no alpha, because the page under it must not show through.
        Body("AfcConsoleSheet", sheetMin.X, top, sheetMax.X - sheetMin.X, h + shadow, ImGuiWindowFlags.NoInputs | ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoScrollWithMouse, () =>
        {
            ImDrawListPtr dl = ImGui.GetWindowDrawList();
            dl.PushClipRect(sheetMin, sheetMax + new float2(0f, shadow), false);
            dl.AddRectFilled(sheetMin, sheetMax, ConsoleStyle.PanelRaisedU32);
            if (shadow > 0f)
                dl.AddRectFilledMultiColor(new float2(sheetMin.X, sheetMax.Y), new float2(sheetMax.X, sheetMax.Y + shadow), SheetShadowInk, SheetShadowInk, 0u, 0u);
            dl.AddLine(new float2(sheetMin.X, sheetMax.Y - border), new float2(sheetMax.X, sheetMax.Y - border), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32), border);
            if (windowLeft > ConsoleStyle.CurrentLayout.WindowPos.X + 0.5f)
                dl.AddLine(sheetMin, new float2(sheetMin.X, sheetMax.Y), ConsoleStyle.ApplyAlpha(ConsoleStyle.FrameU32), border);
            dl.PopClipRect();
        });
        Body("AfcConsoleDrawer", x, top, w, h, ImGuiWindowFlags.AlwaysVerticalScrollbar, () =>
        {
            HeadDrawer.Draw(Frame);
            ConsoleUi.Gap();
            _drawerContentH = ImGui.GetCursorPosY();
            _drawerScrolls = ImGui.GetScrollMaxY() > 0.5f;
            ConsoleUi.ScrollEdges(ConsoleStyle.PanelRaisedU32);
        }, hideBar: !_drawerScrolls, rows: true);

        if (ImGui.IsMouseClicked(ImGuiMouseButton.Left) && !ConsoleHead.MoreHovered && !ImGui.IsMouseHoveringRect(sheetMin, sheetMax, clip: false))
            DrawerOpen = false;
        return top + wanted;
    }

    private const float SheetShadowPx = 14f;

    private const uint SheetShadowInk = 0xB0000000u;

    // The last visit to each pane of this frame, after the page drew: what it holds, whether it scrolls, and its fades, which lie over its rows.
    private static void FinishPanes()
    {
        ShowPane(PaneSide.Picture);
        _picturePaneH = ImGui.GetCursorPosY();
        _pageScrolls = ImGui.GetScrollMaxY() > 0.5f;
        ConsoleUi.ScrollEdges();
        _inputsPaneH = 0f;
        if (_paneMode == PaneMode.Two)
        {
            ShowPane(PaneSide.Inputs);
            _inputsPaneH = ImGui.GetCursorPosY();
            _inputsScroll = ImGui.GetScrollMaxY() > 0.5f;
            ConsoleUi.ScrollEdges();
        }
        _pageContentH = Math.Max(_picturePaneH, _inputsPaneH);
    }

    internal static void ShowPane(PaneSide side)
    {
        if (_paneMode != PaneMode.Two || side == _paneNow)
            return;
        _paneNow = side;
        LeaveBody(rows: true);
        if (side == PaneSide.Picture)
        {
            ConsoleUi.CardsLeftEdge(ConsoleStyle.CurrentLayout.WindowPos.X);
            EnterBody("AfcConsolePage", _paneX, _paneY, _split.LeftBody, _paneH, ImGuiWindowFlags.AlwaysVerticalScrollbar, hideBar: false, rows: true);
            return;
        }
        ConsoleUi.CardsLeftEdge(_paneX + _split.LeftBody);
        EnterBody("AfcConsolePane", _paneX + _split.LeftBody, _paneY, _split.RightBody, _paneH, ImGuiWindowFlags.AlwaysVerticalScrollbar, hideBar: !_inputsScroll || DrawerOpen, rows: true);
        if (_inputsEntered)
            return;
        _inputsEntered = true;
        if (_inputsTopPending)
        {
            _inputsTopPending = false;
            ImGui.SetScrollY(0f);
        }
        BodyProbe?.Invoke("AfcConsolePane");
    }

    /// <summary>Where the row of page tabs stands in a body of <paramref name="contentWidth"/> (DockLayout.TabRowPlace).</summary>
    internal static (float Room, float Inset) TabRowPlace(float contentWidth)
    {
        if (!_wideShown)
            return (contentWidth, 0f);
        ConsoleLayout layout = ConsoleStyle.CurrentLayout;
        (float narrowLeft, float narrowWidth) = Dock.NarrowPlace(WindowKey);
        return DockLayout.TabRowPlace(contentWidth, layout.WindowPos.X, layout.WindowSize.X, narrowLeft, narrowWidth);
    }

    internal static bool HasPanes => _paneMode == PaneMode.Two;

    /// <summary>In WIDE the frame of the press that opens PLAN still has one body over the whole width, so the form waits one frame for its pane.</summary>
    internal static bool ShowPlanPane()
    {
        if (_wideShown && _panesFit && _paneMode == PaneMode.Full)
            return false;
        ShowPane(PaneSide.Inputs);
        return true;
    }

    // Takes the measure of this frame into the next one: the window grows at once and shrinks only after a shape change, and a page that flies and stands taller than the limit gives up picture height before it scrolls, but not for its open PLAN, because that form scrolls under the picture.
    private static void FollowContent(float wanted, float pictureSide, float drawerWanted, float floor, float limit, float contentWidth, float scale, string verb, bool flying)
    {
        int key = HashCode.Combine(Page, LandWay, TuneSet, TuneLanding, TuneDiagnostics, DrawerOpen, _wideShown, HashCode.Combine(ShowFirstOpenNotice, TakeOverAsk.Active, verb, ConsoleHead.Rows(Frame, inStrip: false), flying));
        string? subject = Frame.Subject?.Id;
        if (ConsoleUi.ShapeChanged || key != _shapeKey || subject != _shapeSubject)
        {
            _shrinkFrames = ShrinkFrames;
            int pageTop = HashCode.Combine(Page, LandWay, flying);
            _pageTopPending = pageTop != _pageTopKey;
            _pageTopKey = pageTop;
            ConsoleUi.ShapeChanged = false;
            _shapeKey = key;
            _shapeSubject = subject;
        }
        bool mayShrink = _shrinkFrames > 0 || flying;
        if (_shrinkFrames > 0)
            _shrinkFrames--;

        if (!flying)
            _pictureShrink = 0f;
        else if (!ConsoleUi.FoldIsOpen(FlightView.PlanFoldId))
            _pictureShrink = DockLayout.NextPictureShrink(_pictureShrink, pictureSide - limit, contentWidth, scale);
        _pictureGrowth = _paneMode == PaneMode.Two && !flying ? DockLayout.NextPictureGrowth(_pictureGrowth, _inputsPaneH - _picturePaneH, _split.LeftContent, scale) : 0f;
        if (!_handleHeld)
            _height = DockLayout.FollowHeight(_height, Math.Max(wanted, drawerWanted), floor, limit, mayShrink);
    }

    private static void Body(string id, float x, float y, float w, float h, ImGuiWindowFlags flags, Action draw, bool hideBar = false, bool rows = false)
    {
        EnterBody(id, x, y, w, h, flags, hideBar, rows);
        try
        {
            BodyProbe?.Invoke(id);
            draw();
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"{id}:{Page}:{ex.GetType().Name}", $"[AFC] {id} failed on {Page}: {ex}");
        }
        finally
        {
            LeaveBody(rows);
        }
    }

    private static void EnterBody(string id, float x, float y, float w, float h, ImGuiWindowFlags flags, bool hideBar, bool rows)
    {
        if (hideBar)
        {
            ImGui.PushStyleColor(ImGuiCol.ScrollbarBg, in ConsoleStyle.Transparent);
            ImGui.PushStyleColor(ImGuiCol.ScrollbarGrab, in ConsoleStyle.Transparent);
            ImGui.PushStyleColor(ImGuiCol.ScrollbarGrabHovered, in ConsoleStyle.Transparent);
            ImGui.PushStyleColor(ImGuiCol.ScrollbarGrabActive, in ConsoleStyle.Transparent);
        }
        ConsoleStyle.BeginBody(id.AsSpan(), new float2(x, y), new float2(w, Math.Max(1f, h)), flags);
        if (hideBar)
            ImGui.PopStyleColor(4);
        if (rows)
            ImGui.PushStyleVar(ImGuiStyleVar.ItemSpacing, new float2(ImGui.GetStyle().ItemSpacing.X, ConsoleUi.RowGapPx));
    }

    private static void LeaveBody(bool rows)
    {
        Guidance.GuidanceRegion.EndOpen();
        ConsoleUi.EndSection();
        Guidance.TierScope.Reset();
        if (rows)
            ImGui.PopStyleVar();
        ConsoleStyle.EndBody();
    }

    private static CommitSpec DrawPage(bool flying, float pictureBasis)
    {
        // Only a page that flies gives up picture height, because it must stay clear of the stock burn panel.
        float pictureH = flying
            ? DockLayout.PictureHeight(pictureBasis, _pictureShrink, ImGuiHelper.InterfaceScale)
            : DockLayout.PictureHeight(pictureBasis, 0f, ImGuiHelper.InterfaceScale) + _pictureGrowth;
        var ctx = new PageContext(Frame, pictureH, UiConfig.GetBool(KeyExpertOpen, false), flying);

        string engineLeft = Frame.Idle ? SituationText.EngineLeftLine(in Frame.Situation, ConsoleFrame.EnginePercent(Frame.Subject)) : "";
        ConsoleUi.NoteShape("AfcEngineLeft", engineLeft.Length > 0 ? 1 : 0);
        if (engineLeft.Length > 0)
            ConsoleUi.Warning(engineLeft.AsSpan());

        if (Frame.Editor && !ConsolePages.WorksInEditor(Page))
        {
            ConsoleUi.MutedWrapped(PageChooser.Unavailable(Frame, Page));
            return CommitSpec.None;
        }
        if (Frame.Subject == null && Page is not (ConsolePage.Staging or ConsolePage.Tune or ConsolePage.Transfer))
        {
            ConsoleUi.MutedWrapped(Frame.Eva ? "An EVA kitten is no craft AFC flies. STAGING, TUNE and the stock planner link on TRANSFER work." : "No craft under control.");
            return CommitSpec.None;
        }

        return Page switch
        {
            ConsolePage.Launch => LaunchPage.Draw(ctx),
            ConsolePage.Orbit => OrbitPage.Draw(ctx),
            ConsolePage.Transfer => TransferPage.Draw(ctx),
            ConsolePage.Land => LandPage.Draw(ctx),
            ConsolePage.Tune => TunePage.Draw(ctx),
            _ => CommitSpec.None,
        };
    }

    /// <summary>The strip: the head without the body, at the console's place and width, or left of the stock planner while that holds the dock.</summary>
    private static void DrawStrip(bool besideStock)
    {
        Dock.ForgetOldSizeOnce(WindowKey);
        float scale = ImGuiHelper.InterfaceScale;
        ImGuiViewportPtr main = ImGui.GetMainViewport();
        DockRect dock = DockLayout.Console(main.Pos.X, main.Pos.Y, main.Size.X, main.Size.Y, scale, wide: false);
        float right = UiConfig.GetFloat(WindowKey + ".right", dock.Right);
        float top = UiConfig.GetFloat(WindowKey + ".y", dock.Y);
        float width = Math.Clamp(UiConfig.GetFloat(WindowKey + ".w", dock.W), DockLayout.MinWidth * scale, DockLayout.MaxNarrowWidth * scale);

        ConsoleStyle.PushWidgetStyle();
        float headH = ConsoleHead.Height(Frame, inStrip: true, firstOpenNotice: false, width);
        ConsoleStyle.PopWidgetStyle();
        float pad = ConsoleStyle.ContentPaddingPx;
        float height = ConsoleStyle.TitlebarHeight + headH + pad * 2f;

        DockRect place = besideStock
            ? DockLayout.StripBesideStock(main.Pos.X, main.Size.X, scale, top, width, height)
            : new DockRect(right - width, top, width, height);
        ImGui.SetNextWindowPos(new float2(place.X, place.Y), ImGuiCond.Always, (float2?)null);

        string tag = Frame.Subject?.Id ?? "";
        if (!ConsoleStyle.BeginWindow(StripId.AsSpan(), WindowTitle.AsSpan(), WindowSignature.AsSpan(), new float2(place.W, place.H),
                ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoScrollWithMouse | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoMove,
                ImGuiCond.Always, pinToMainViewport: false))
            return;
        try
        {
            ConsoleLayout layout = ConsoleStyle.CurrentLayout;
            ConsoleUi.CardsBeside(layout.WindowPos.X, layout.WindowPos.X + layout.WindowSize.X);
            if (tag.Length > 0)
                DrawTitleTag(tag, in layout, layout.TitlebarPos.X + layout.TitlebarSize.X - ConsoleStyle.TitlebarHorizontalPaddingPx, ConsoleStyle.TitlebarContextTagGapPx);
            ConsoleStyle.PushWidgetStyle();
            try
            {
                (float x, float w, _) = ContentEdges(in layout);
                ConsoleStyle.BeginBody("AfcStripHead".AsSpan(), new float2(x, layout.BodyPos.Y + pad),
                    new float2(w, headH), ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoScrollWithMouse);
                try
                {
                    ConsoleHead.Draw(Frame, inStrip: true, firstOpenNotice: false);
                }
                finally
                {
                    ConsoleStyle.EndBody();
                }
            }
            finally
            {
                ConsoleStyle.PopWidgetStyle();
            }
        }
        finally
        {
            ConsoleUi.CardsFree();
            ConsoleStyle.EndWindow();
        }
    }
}
