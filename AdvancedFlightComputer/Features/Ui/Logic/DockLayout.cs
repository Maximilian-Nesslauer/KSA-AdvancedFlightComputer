using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>A screen rectangle in pixels.</summary>
internal readonly record struct DockRect(float X, float Y, float W, float H)
{
    public float Right => X + W;
}

internal enum PaneSide { Picture, Inputs }

internal enum PaneMode { Full, Two }

internal readonly record struct PaneSplit(float LeftBody, float RightBody, float LeftContent, float RightContent);

/// <summary>Where a hover card stands: a point of the card given by a pivot, as ImGui.SetNextWindowPos takes it.</summary>
internal readonly record struct CardPlace(float X, float Y, float PivotX, float PivotY);

/// <summary>
/// Where the console docks: at the place the stock Transfer Planner opens, which is
/// TransferPlanner.WindowMarginTopPx (50 px) below the top of the viewport and 40 px from its right
/// edge (TransferPlanner.WindowMarginRightPx is 440 px for a window 400 px wide). All sizes are at
/// interface scale 1 and are multiplied by the scale here.
/// </summary>
internal static class DockLayout
{
    public const float NarrowWidth = 520f;
    public const float WideWidth = 920f;
    public const float MinWidth = 460f;
    public const float MaxNarrowWidth = 520f;
    public const float TopMargin = 50f;
    public const float RightMargin = 40f;
    public const float HeightMargin = 90f;
    public const float StockMarginRight = 440f;
    public const float StripGap = 8f;

    public const float OldNarrowWidth = 460f;

    public const float MinHeight = 200f;

    /// <summary>The height of the first frame, before the window has measured its page.</summary>
    public const float FirstHeight = 600f;
    public const float LimitHeight = 1050f;

    public const float DrawerMaxHeight = 360f;
    public const float PanelGap = 8f;
    public const float ScreenGap = 8f;
    public const float MinPictureHeight = 150f;

    /// <summary>The height of a picture with nothing to show: one sentence in a low frame.</summary>
    public const float EmptyPictureHeight = 72f;

    public const float MinTabPadding = 2f;
    public const float MaxTabPadding = 12f;

    /// <summary>The content of the right pane of WIDE: 440 px at interface scale 1.2.</summary>
    public const float RightPaneWidth = 367f;

    public const float LeastLeftPane = 380f;

    /// <summary>The room the frame of the window takes beside the panes: the left edge, the border and two lanes.</summary>
    public const float PaneFrameRoom = 80f;

    public const float CardGap = 8f;

    private const float MostPictureShare = 0.62f;

    public static DockRect Console(float vpX, float vpY, float vpW, float vpH, float scale, bool wide)
    {
        float w = (wide ? WideWidth : NarrowWidth) * scale;
        w = Math.Min(w, Math.Max(MinWidth * scale * 0.5f, vpW - 2f * RightMargin * scale));
        float h = Math.Max(MinHeight * scale, Math.Min(FirstHeight * scale, vpH - HeightMargin * scale));
        float x = vpX + vpW - RightMargin * scale - w;
        float y = vpY + TopMargin * scale;
        x = Math.Max(vpX, x);
        return new DockRect(x, y, w, h);
    }

    /// <summary>The width limits of the console window, narrow or WIDE.</summary>
    public static (float Min, float Max) WidthLimits(float scale, bool wide, float vpW)
    {
        float max = Math.Max(MinWidth * scale, vpW - 32f);
        float min = Math.Min((wide ? RightPaneWidth + LeastLeftPane + PaneFrameRoom : MinWidth) * scale, max);
        return (min, max);
    }

    public static bool WideGrowsRight(float narrowLeft, float narrowRight, float workLeft, float workRight)
        => workRight - narrowRight > narrowLeft - workLeft;

    public static float WideLeft(float narrowRight, float narrowWidth, float wideWidth, float workLeft, float workRight)
    {
        float narrowLeft = narrowRight - narrowWidth;
        float x = WideGrowsRight(narrowLeft, narrowRight, workLeft, workRight) ? narrowLeft : narrowRight - wideWidth;
        return Math.Clamp(x, workLeft, Math.Max(workLeft, workRight - wideWidth));
    }

    public static float NarrowRight(float wideLeft, float wideWidth, float narrowWidth, bool growsRight)
        => growsRight ? wideLeft + narrowWidth : wideLeft + wideWidth;

    public static (float Room, float Inset) TabRowPlace(float contentWidth, float windowLeft, float windowWidth, float narrowLeft, float narrowWidth)
    {
        float extra = Math.Max(0f, windowWidth - narrowWidth);
        return (Math.Max(0f, contentWidth - extra), Math.Clamp(narrowLeft - windowLeft, 0f, extra));
    }

    public static bool WideHasPanes(float scale, float vpW)
        => WidthLimits(scale, wide: true, vpW).Max >= (RightPaneWidth + LeastLeftPane + PaneFrameRoom) * scale;

    public static PaneSplit SplitPanes(float span, float lane, float scale)
    {
        float right = RightPaneWidth * scale;
        float leftBody = Math.Max(0f, span - right - lane);
        return new PaneSplit(leftBody, right + lane, Math.Max(0f, leftBody - lane), right);
    }

    public static bool PageTakesWide(ConsolePage page, bool pageWorks, bool burnsListed) => page switch
    {
        ConsolePage.Tune => false,
        ConsolePage.Rcs => pageWorks && burnsListed,
        _ => pageWorks,
    };

    public static PaneMode PanesFor(bool wide, bool panesFit, bool flying, bool planOpen)
        => wide && panesFit && !(flying && !planOpen) ? PaneMode.Two : PaneMode.Full;

    public static float MoveRowWidth(float labelColumn, float firstName, float secondName, float button, float gap)
        => labelColumn + firstName + secondName + button * 4f + gap * 7f;

    /// <summary>How much taller than its natural height the picture of the left pane stands in the next frame.</summary>
    public static float NextPictureGrowth(float growth, float underPicture, float contentWidth, float scale)
    {
        float most = Math.Max(0f, contentWidth * MostPictureShare - NaturalPictureHeight(contentWidth, scale));
        return Math.Clamp(growth + underPicture, 0f, most);
    }

    public static CardPlace? CardBeside(float windowLeft, float windowRight, float workLeft, float workRight, float workMiddleY, float rowY, float rowH, float cardWidth, float gap)
    {
        float roomLeft = windowLeft - gap - workLeft;
        float roomRight = workRight - windowRight - gap;
        if (Math.Max(roomLeft, roomRight) < cardWidth)
            return null;
        bool left = roomLeft >= roomRight;
        bool rises = rowY > workMiddleY;
        return new CardPlace(left ? windowLeft - gap : windowRight + gap, rises ? rowY + rowH : rowY, left ? 1f : 0f, rises ? 1f : 0f);
    }

    /// <summary>The left edge of the stock Transfer Planner when it opens at its own place.</summary>
    public static float StockLeft(float vpX, float vpW, float scale) => vpX + vpW - StockMarginRight * scale;

    /// <summary>
    /// The strip while the stock planner holds the dock: left of the stock window with a gap, at the
    /// console top and width.
    /// </summary>
    public static DockRect StripBesideStock(float vpX, float vpW, float scale, float top, float width, float height)
    {
        float x = StockLeft(vpX, vpW, scale) - StripGap * scale - width;
        return new DockRect(Math.Max(vpX, x), top, width, height);
    }

    /// <summary><paramref name="pilotMax"/> is the height the pilot set with the resize handle, or NaN.</summary>
    public static (float Floor, float Limit) HeightRange(float top, float left, float right, float vpBottom, float scale, ReadOnlySpan<DockRect> panels, float pilotMax, float rows = 0f)
    {
        float least = MinHeight * scale;
        float most = Math.Max(least, vpBottom - top - ScreenGap * scale);
        // A height the pilot dragged is the height, over the stock panels and the content, until the window place is reset.
        if (float.IsFinite(pilotMax))
        {
            float pilotHeight = Math.Clamp(pilotMax, least, most);
            return (pilotHeight, pilotHeight);
        }

        float limit = float.PositiveInfinity;
        foreach (DockRect p in panels)
        {
            if (left < p.Right && right > p.X && top < p.Y)
                limit = Math.Min(limit, p.Y - PanelGap * scale - top);
        }
        if (float.IsPositiveInfinity(limit))
            limit = LimitHeight * scale;

        float floor = Math.Min(Math.Max(least, rows), most);
        return (floor, Math.Clamp(limit, floor, most));
    }

    /// <summary>While its form is closed it also stays clear of the place of the stock BurnControl panel while that panel hides, as far as the floor allows, so a page that held this height does not change when the next burn shows the panel.</summary>
    public static (float Floor, float Limit) FlyingHeightRange(float limit, float top, float left, float right, float scale, DockRect? burnPanel, float rows = 0f)
    {
        float floor = Math.Min(Math.Max(MinHeight * scale, rows), limit);
        if (burnPanel is DockRect p && left < p.Right && right > p.X && top < p.Y)
            limit = Math.Clamp(p.Y - PanelGap * scale - top, floor, limit);
        return (floor, limit);
    }

    /// <summary>The window height for the next frame: <paramref name="content"/> inside the range. <paramref name="current"/> is NaN before the first frame.</summary>
    public static float FollowHeight(float current, float content, float floor, float limit, bool mayShrink)
    {
        float target = Math.Clamp(content, floor, Math.Max(floor, limit));
        if (float.IsNaN(current) || mayShrink || target >= current)
            return target;
        return Math.Clamp(current, floor, Math.Max(floor, limit));
    }

    public static float NaturalPictureHeight(float contentWidth, float scale, float aspect = 190f / 436f)
        => Math.Max(MinPictureHeight * scale, contentWidth * aspect);

    public static float PictureHeight(float contentWidth, float shrink, float scale)
        => Math.Max(MinPictureHeight * scale, NaturalPictureHeight(contentWidth, scale) - Math.Max(0f, shrink));

    /// <summary>How much the picture gives up in the next frame.</summary>
    public static float NextPictureShrink(float shrink, float overflow, float contentWidth, float scale)
    {
        float most = Math.Max(0f, NaturalPictureHeight(contentWidth, scale) - MinPictureHeight * scale);
        return Math.Clamp(shrink + overflow, 0f, most);
    }

    public static float TabPadding(float width, float textWidth, int tabs, float dividerWidth, float scale)
    {
        if (tabs <= 0)
            return 0f;
        float share = (width - textWidth - dividerWidth) / (tabs * 2f);
        return Math.Clamp(share, MinTabPadding * scale, MaxTabPadding * scale);
    }

    /// <summary>A closed up row fills its room exactly, and a sum of float widths can pass the end by a rounding error, which would send the last tab alone to a second row.</summary>
    public const float TabRowTolerance = 0.5f;

    public static int TabGaps(int tabs) => tabs * 2 + Math.Max(0, tabs - 1);

    public static float TabPad(float textWidth, int tabs, float room, float pad, float least)
    {
        int gaps = TabGaps(tabs);
        if (gaps == 0 || textWidth + pad * gaps <= room)
            return pad;
        float closed = (room - textWidth) / gaps;
        return closed >= least ? closed : pad;
    }

    public static void SegmentWidths(ReadOnlySpan<float> textWidths, float width, float padding, Span<float> widths)
    {
        int n = textWidths.Length;
        if (n == 0)
            return;
        float equal = width / n;
        float need = 0f;
        bool allFit = true;
        for (int i = 0; i < n; i++)
        {
            float w = textWidths[i] + padding * 2f;
            need += w;
            allFit &= w <= equal;
        }
        for (int i = 0; i < n; i++)
        {
            float w = textWidths[i] + padding * 2f;
            widths[i] = allFit ? equal
                : need <= width ? w + (width - need) / n
                : need > 0f ? width * w / need
                : equal;
        }
    }

    public static void TableColumns(ReadOnlySpan<float> longest, float room, Span<float> widths)
    {
        int n = longest.Length;
        if (n == 0)
            return;
        float need = 0f;
        for (int i = 0; i < n; i++)
            need += longest[i];
        float spare = room - need;
        for (int i = 0; i < n; i++)
            widths[i] = spare >= 0f ? longest[i] + spare / n
                : need > 0f ? room * longest[i] / need
                : room / n;
    }
}
