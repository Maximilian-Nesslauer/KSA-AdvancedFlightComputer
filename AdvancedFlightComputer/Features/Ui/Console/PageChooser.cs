using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Transfer;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Console.Pages;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

/// <summary>The page chooser, at the place where stock shows its PLAN TYPE: one row of tabs sized to their text, in the order of a mission, with a hairline before the setup pages, and the band of the page under it. A dot on a tab says that something flies on that page for the craft on screen, and a page that cannot work here is dim and does not open.</summary>
internal static class PageChooser
{
    private static float DividerRoom => 13f * ImGuiHelper.InterfaceScale;

    private static float DotRadius => 2.5f * ImGuiHelper.InterfaceScale;

    private static float DotInset => 3f * ImGuiHelper.InterfaceScale;

    public static float TabsHeight => ConsoleStyle.FrameHeightPx;

    public static float HeightFor(bool band) => TabsHeight + (band ? PageBand.Height : ConsoleStyle.WindowBorderThicknessPx);

    public static void Draw(ConsoleFrame f, bool band)
    {
        float2 origin = ImGui.GetCursorScreenPos();
        float width = ConsoleStyle.ContentAvailWidth();
        float h = TabsHeight;
        ConsolePage? picked = null;

        (float room, float inset) = AfcConsole.TabRowPlace(width);
        float pad = TabPadding(room);
        float x = origin.X + inset;
        foreach (ConsolePage page in ConsolePages.All)
        {
            if (page == ConsolePages.FirstSetupPage)
            {
                float mid = x + DividerRoom * 0.5f;
                ImGui.GetWindowDrawList().AddLine(new float2(mid, origin.Y + h * 0.2f), new float2(mid, origin.Y + h * 0.8f),
                    ConsoleStyle.ApplyAlpha(ConsoleStyle.FrameU32), ConsoleStyle.WindowBorderThicknessPx);
                x += DividerRoom;
            }
            if (DrawTab(f, page, new float2(x, origin.Y), h, pad, out float tabW))
                picked = page;
            x += tabW;
        }

        if (picked is ConsolePage chosen && chosen != AfcConsole.Page)
            AfcConsole.SetPage(chosen);

        if (band)
            PageBand.Draw(AfcConsole.Page, new float2(origin.X, origin.Y + h), width);

        ImGui.SetCursorScreenPos(origin);
        ImGui.Dummy(new float2(width, HeightFor(band)));
    }

    private static float TabPadding(float width)
    {
        ConsoleUi.PushRow();
        float text = 0f;
        foreach (ConsolePage page in ConsolePages.All)
            text += ImGui.CalcTextSize(ConsolePages.Label(page)).X;
        ConsoleStyle.PopFont();
        return DockLayout.TabPadding(width, text, ConsolePages.All.Length, DividerRoom, ImGuiHelper.InterfaceScale);
    }

    private static bool DrawTab(ConsoleFrame f, ConsolePage page, float2 pos, float h, float pad, out float width)
    {
        string label = ConsolePages.Label(page);
        string unavailable = Unavailable(f, page);
        bool dot = unavailable.Length == 0 && page != ConsolePage.Staging && Dot(f, page);
        ConsoleUi.PushRow();
        float2 textSize = ImGui.CalcTextSize(label);
        ConsoleStyle.PopFont();
        width = textSize.X + pad * 2f;
        var size = new float2(width, h);

        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        ImGui.SetCursorScreenPos(pos);
        bool hit = ImGui.InvisibleButton("AfcPage" + ConsolePages.Key(page), size);
        ConsoleUi.NoteRow("AfcPage", (int)page, pos, size);
        bool hovered = ImGui.IsItemHovered();
        bool active = page == AfcConsole.Page;
        float2 max = pos + size;

        if (active)
            DrawPlate(dl, pos, max);
        else if (hovered && unavailable.Length == 0)
            dl.AddRectFilled(pos, max, ConsoleStyle.ApplyAlpha(ConsoleStyle.RowHoverWashU32), ConsoleStyle.FrameRoundingPx, ImDrawFlags.RoundCornersTop);

        uint text = unavailable.Length > 0 && !active
            ? ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32, 0.55f)
            : ConsoleStyle.ApplyAlpha(active ? ConsoleStyle.TextPrimaryU32 : hovered ? ConsoleStyle.TextLabelU32 : ConsoleStyle.TextMutedU32);
        ConsoleUi.PushRow();
        float textX = pos.X + pad;
        dl.AddText(new float2(textX, pos.Y + (h - textSize.Y) * 0.5f), text, label);
        ConsoleStyle.PopFont();

        if (unavailable.Length > 0)
        {
            if (hovered)
                ConsoleUi.Card(label, unavailable);
            return false;
        }

        if (dot)
        {
            uint ink = page == ConsolePage.Rcs && f.Activity.StockAutoOnNode ? Palette.Stock.U32 : Palette.Afc.U32;
            dl.AddCircleFilled(new float2(max.X - DotInset - DotRadius, pos.Y + DotInset + DotRadius), DotRadius, ConsoleStyle.ApplyAlpha(ink));
        }
        if (hovered)
        {
            var facts = new TabFacts(f.Situation.Where, f.Subject != null && BurnModeInput.NextNode(f.Subject) != null, Dot(f, ConsolePage.Staging), f.Editor);
            string card = PageText.TabCard(page, in facts);
            ConsoleUi.Card(label, dot ? card + "\n\n" + PageText.PageRunsNote : card);
        }
        return hit;
    }

    private static void DrawPlate(ImDrawListPtr dl, float2 pos, float2 max)
    {
        float border = ConsoleStyle.WindowBorderThicknessPx;
        var bottom = new float2(max.X, max.Y + border);
        dl.AddRectFilled(pos, bottom, ConsoleStyle.ApplyAlpha(ConsoleStyle.PanelRaisedU32), ConsoleStyle.FrameRoundingPx, ImDrawFlags.RoundCornersTop);
        uint frame = ConsoleStyle.ApplyAlpha(ConsoleStyle.FrameU32);
        dl.AddLine(new float2(pos.X, pos.Y + ConsoleStyle.FrameRoundingPx), new float2(pos.X, max.Y), frame, border);
        dl.AddLine(new float2(max.X, pos.Y + ConsoleStyle.FrameRoundingPx), new float2(max.X, max.Y), frame, border);
        dl.AddRectFilled(pos, new float2(max.X, pos.Y + ConsoleStyle.ModifiedTickThicknessPx), ConsoleStyle.ApplyAlpha(ConsoleStyle.ActiveToolU32),
            ConsoleStyle.FrameRoundingPx, ImDrawFlags.RoundCornersTop);
    }

    /// <summary>Why <paramref name="page"/> cannot work here, or empty. The same rules the page body applies when it is open anyway.</summary>
    public static string Unavailable(ConsoleFrame f, ConsolePage page)
    {
        if (f.Editor && !ConsolePages.WorksInEditor(page))
            return PageText.FlightOnlyInEditor;
        if (f.Subject == null && page is not (ConsolePage.Staging or ConsolePage.Tune or ConsolePage.Transfer))
            return f.Eva ? "EVA: NO CRAFT. STAGING, TUNE AND STOCK PLANNER LINK WORK." : "NO CRAFT UNDER CONTROL";
        return "";
    }

    public static bool Dot(ConsoleFrame f, ConsolePage page)
    {
        Vehicle? subject = f.Subject;
        CraftActivity a = f.Activity;
        switch (page)
        {
            case ConsolePage.Launch:
                return subject != null && GuidanceModes.AnyLiveOn(ConsolePage.Launch, a);
            case ConsolePage.Land:
                return subject != null && GuidanceModes.AnyLiveOn(ConsolePage.Land, a);
            case ConsolePage.Orbit:
                return subject != null && SharedVehicleHooks.MultiPassEnabled
                    && MultiPassRegistry.TryGet(subject.Id, out MultiPassExecution? orbitExec) && OrbitToolCatalog.IsTool(orbitExec.Intent.TypeKey);
            case ConsolePage.Transfer:
                return TransferSession.Calculating || (subject != null && SharedVehicleHooks.MultiPassEnabled
                    && MultiPassRegistry.TryGet(subject.Id, out MultiPassExecution? transferExec) && !OrbitToolCatalog.IsTool(transferExec.Intent.TypeKey));
            case ConsolePage.Rcs:
                return a.RcsFlying || a.StockAutoOnNode;
            case ConsolePage.Staging:
                return subject != null && SharedVehicleHooks.AutoStageEnabled && StagingDetector.IsArmed(subject);
            default:
                return false;
        }
    }
}
