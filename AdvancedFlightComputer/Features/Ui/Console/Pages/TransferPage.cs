using System;
using System.Collections.Generic;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Transfer;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.Features.Ui.Pictures;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using CommunityToolkit.HighPerformance.Buffers;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console.Pages;

/// <summary>TRANSFER: transfers to other bodies on the AFC porkchop, with the inputs above the plot and the times and the fuel of the selection under it. RE-CALCULATE and the STOCK PLANNER link are on the line under the picture; the link opens the stock Transfer Planner, the one path to it while the planner key opens AFC, for Interstellar and stock Hohmann.</summary>
internal static class TransferPage
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    private static readonly string[] SelectLabels = { "BEST DELTA-V", "LOW ARRIVAL" };

    private static readonly Rgb8[] SelectMarks = { ColorRules.Afc, ColorRules.Good };
    private static readonly List<TransferObject> _destinations = new();
    private static readonly List<TransferObject> _sources = new();
    private static readonly List<OrbitSketch.Track> _tracks = new();
    private static readonly List<BurnBar> _bars = new();
    private static bool _setTarget;
    private static string _fuelFact = "";

    internal const string StockPlannerTip = "Opens the stock Transfer Planner: Hohmann, Circularize and Interstellar as the game ships them. No passes and no flyby there. It takes the place of AFC, and AFC comes back when it closes.";

    /// <summary>The card of the link to the stock planner.</summary>
    internal static string PlannerTipFor(CraftSituation where) => where == CraftSituation.Ground ? "Needs an orbit. It still opens.\n\n" + StockPlannerTip : StockPlannerTip;

    public static CommitSpec Draw(in PageContext ctx)
    {
        if (ctx.Frame.Editor || ctx.Frame.Subject == null)
        {
            ConsoleUi.MutedWrapped(ctx.Frame.Editor ? "AFC transfers need a craft in flight." : "AFC transfers need a craft under control.");
            if (ConsoleUi.Link("PLANNER", "AfcTransferStockOnly", StockPlannerTip))
                PlannerRoute.OpenStockPlanner();
            ImGui.SameLine();
            ConsoleUi.OwnerChip(Owner.Stock, "STOCK");
            return CommitSpec.None;
        }

        ConsoleUi.ClearPageWarning();
        Vehicle subject = ctx.Subject;
        if (TransferSession.Source == null || TransferSession.Source.IsDisposed)
            TransferSession.SetSource(subject);
        Vehicle source = TransferSession.Source!;

        // CRAFT stands under the goal, because the craft of the title is the common case, and a pick counts in this frame for the rows under it.
        if (!ctx.Flying)
        {
            ctx.ToInputs();
            DrawDestinationRow(source);
            DrawSearchFold(in ctx, source);
            DrawSourceRow(source, subject);
            source = TransferSession.Source!;
        }
        if (HohmannMultiPassUI.Enabled)
            HohmannMultiPassUI.DrawInputs();
        if (!ctx.Flying)
        {
            DrawCorrectionRow();
            DrawSelection();
        }
        ctx.ToPicture();
        DrawPicture(in ctx, source);
        DrawLegendLine(source);
        DrawSelectionResult();
        CollectBars();
        _fuelFact = BurnBars.Draw(source, _bars);
        if (ctx.Flying && MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? running))
            MultiPassUI.NoteRunningShort(running, BurnBars.LastShortMs);
        if (HohmannMultiPassUI.Enabled)
            HohmannMultiPassUI.DrawResults();

        if (ctx.Flying)
        {
            ConsoleUi.Gap();
            (string goal, int goalPasses) = OrbitPage.RunningGoal(source);
            if (ConsoleUi.Fold(FlightView.PlanLabel(PageText.PassesPlan(goal, goalPasses)), FlightView.PlanFoldId, false, PageText.PlanTip) && ctx.ToPlanForm())
            {
                ctx.PlanHead();
                ConsoleUi.MutedWrapped(FlightView.PassesPlanLine);
                using (ConsoleUi.Dim(true))
                {
                    DrawDestinationRow(source);
                    DrawSearchFold(in ctx, source);
                    DrawSourceRow(source, subject);
                    DrawCorrectionRow();
                    DrawSelection();
                    DrawHowFold(in ctx);
                }
            }
        }
        else
        {
            ctx.ToInputs();
            DrawHowFold(in ctx);
        }

        bool own = ReferenceEquals(source, subject);
        SituationFacts situation = own ? ctx.Frame.Situation : ConsoleFrame.ReadFacts(source);
        CraftActivity activity = ctx.Frame.Activity;
        bool ascentFlies = own && activity.AscentRunning;
        return BuildCommit(source, situation.Where, ascentFlies, own && !ascentFlies && activity.AnyGuidanceLive);
    }

    private static void DrawSearchFold(in PageContext ctx, Vehicle source)
    {
        string label = TransferSession.Info != null ? PageText.SearchLabel(TransferSession.MinTofSec, TransferSession.MaxTofSec) : FoldNames.Search;
        ConsoleUi.PartsFold(FoldNames.Search, "TransferSearch",
        [
            new FoldPart(FoldNames.FlightTime, TransferSession.Info != null, DrawFlightTimeRows),
            new FoldPart(FoldNames.CraftTarget, true, () => DrawSetTargetRow(source)),
        ], 0, ctx.FoldsOpen, PageText.SearchTip, label);
    }

    private static void DrawHowFold(in PageContext ctx)
    {
        ConsoleUi.Gap();
        PageKit.HowItFlies("transfer",
        [
            new FoldPart(FoldNames.Flyby, HohmannFlybyUI.FoldRowsShown, HohmannFlybyUI.DrawFoldRows),
            new FoldPart(FoldNames.Split, HohmannMultiPassUI.SplitRowShown, HohmannMultiPassUI.DrawSplitRow, HohmannMultiPassUI.SplitSummary),
        ], HohmannFlybyUI.ChangedInFold + HohmannMultiPassUI.ChangedInFold, ctx.FoldsOpen);
    }

    #region Header and destination

    private static void DrawSourceRow(Vehicle source, Vehicle subject)
        => PageKit.CraftRow(source, subject, _sources, "AfcTransferPlanFor", "AfcTransferSource", TransferSession.SetSource);

    private static void DrawDestinationRow(Vehicle source)
    {
        TransferBounds.Destinations(source, _destinations);
        if (_destinations.Count == 0)
        {
            ConsoleUi.Readout("DESTINATION".AsSpan(), "None available".AsSpan());
            return;
        }

        TransferObject selected = _destinations[0];
        bool found = false;
        if (TransferSession.Destination is Astronomical current)
        {
            selected = new TransferObject(current);
            foreach (TransferObject o in _destinations)
                if (o.GetKey() == selected.GetKey())
                    found = true;
        }
        if (!found)
        {
            // No destination yet, or one the source cannot reach any more: the craft's target first, else the first in the list.
            selected = _destinations[0];
            if (source.Target is Astronomical target)
                foreach (TransferObject o in _destinations)
                    if (o.GetKey() == target.Id)
                        selected = o;
            if (selected.Body is IOrbiter first)
                TransferSession.SetDestination(first);
        }

        string before = selected.GetKey();
        if (ConsoleUi.ComboRow("DESTINATION".AsSpan(), "AfcTransferDestination".AsSpan(), ref selected, _destinations)
            && selected.GetKey() != before && selected.Body is IOrbiter picked)
        {
            TransferSession.SetDestination(picked);
            if (_setTarget)
                InputEvents.ChangeTargetBuffer.Add(new InputEvents.ChangeTargetData { Vehicle = source, Target = picked });
        }
    }

    private static void DrawHeader()
    {
        bool lines = Patch_CelestialSystem_OnPreRender.Applied;
        int noView = 0;
        if (Picture.Header("PORKCHOP", default, ref noView, "AfcTransferPic",
                new[] { PageText.ShowInWorld }, new[] { lines && TransferSession.ShowMap },
                new[] { "Draws the selected transfer, its passes or its flyby in the world and on the map." },
                new[] { lines ? "" : Patch_CelestialSystem_OnPreRender.UnavailableTip }) == 0)
            TransferSession.ShowMap = !TransferSession.ShowMap;
    }

    // The verbs of the porkchop under it, and the link to the stock planner at the right.
    private static void DrawLegendLine(Vehicle source)
    {
        string refusal = TransferSession.CalculateRefusal();
        if (refusal.Length == 0 && TransferSession.TransferBurn != null)
            refusal = "The created transfer burn is still planned. Fly it or delete it to calculate again.";
        bool cancelHere = TransferSession.Calculating && PassesHoldFooter(source, out _);
        string[] verbs = cancelHere ? new[] { PageText.CancelVerb }
            : !TransferSession.Calculating && TransferSession.Calculated ? new[] { "RE-CALCULATE" } : Array.Empty<string>();
        string[] tips = { cancelHere ? PageText.CancelTransferTip : refusal.Length > 0 ? refusal : "Calculates every departure time and time of flight in the window." };
        bool[] disabled = { !cancelHere && refusal.Length > 0 };
        CraftSituation where = AfcConsole.Frame.Situation.Where;
        int clicked = Picture.LegendLine("", verbs, tips, "AfcTransferLegend", disabled, "PLANNER", PlannerTipFor(where), linkStock: true);
        if (clicked == verbs.Length)
            PlannerRoute.OpenStockPlanner();
        else if (clicked == 0)
        {
            if (cancelHere)
                TransferSession.CancelCalculation();
            else
                TransferSession.StartCalculation();
        }
    }

    private static bool PassesHoldFooter(Vehicle source, out MultiPassExecution? exec)
        => MultiPassRegistry.TryGet(source.Id, out exec) && exec.Intent is HohmannTransferIntent;

    #endregion

    #region Picture

    private static void DrawPicture(in PageContext ctx, Vehicle source)
    {
        PorkchopRaster? raster = TransferSession.SelectBestRel ? TransferSession.RasterRel : TransferSession.RasterDv;
        if (!Picture.Shows("AfcTransferPic", !TransferSession.Calculating && TransferSession.Calculated && raster != null))
        {
            DrawNoPorkchop(ctx.Flying, ctx.Frame.Situation.Where);
            return;
        }

        DrawHeader();
        PictureFrame probe = Picture.Probe();
        float scale = probe.Scale;
        OrbitalTransfers.TransferData? selected = TransferSession.SelectedEntry?.TransferData;
        string big = selected != null ? Fmt.Dv(selected.TransferDvVlf.Length()) : "";
        string forText = !ReferenceEquals(source, ctx.Subject) ? "For " + source.Id : "";

        double nowS = Universe.GetElapsedSeconds();
        double departLo = TransferSession.DepartStartS - nowS;
        double departSpan = Math.Max(TransferSession.DepartSpanS, 1.0);
        double departStep = AxisTicks.TimeStep(departSpan, 4);
        string departSample = AxisTicks.Time(Math.Floor((departLo + departSpan) / departStep) * departStep);

        float head = Picture.HeadHeight(in probe, big, text: true);
        float bottom = Picture.TextLaneHeight() * 2f;
        float left = Picture.TickWidth(departSample);
        float width = probe.Width;
        float side = Math.Min(148f * scale, width * 0.36f);
        // A page that flies has a limit under the stock burn panel, so its plot gives way like every other picture.
        float wanted = ctx.Flying ? ctx.PictureHeight : Math.Max(ctx.PictureHeight, 280f * scale * (AfcConsole.HasPanes ? 1.4f : 1f));
        float chop = Math.Max(100f * scale, Math.Min(width - side - left - 8f * scale, wanted - head - bottom));
        using (PictureScope pic = Picture.Open(Picture.FrameHeight(head + chop + bottom)))
        {
            PictureFrame f = pic.Frame;
            PictureLanes lanes = Picture.Lanes(in f, head);
            var plot = new PicBox(f.Min.X + left, f.Min.Y + head, chop, chop);
            var axes = new PictureLanes(lanes.Head, new PicBox(f.Min.X, plot.Y, left, chop), plot, default, new PicBox(f.Min.X, plot.Bottom, plot.Right - f.Min.X, Picture.TextLaneHeight()));
            var nameLane = new PicBox(plot.X, plot.Bottom + Picture.TextLaneHeight(), plot.W, Picture.TextLaneHeight());
            StageFuel[] stages = BurnBars.StagesOf(source);
            double budgetMs = stages.Length == 0 ? double.NaN : BurnBudget.OnBoard(stages);
            double onBoardMs = TransferSession.SelectBestRel ? double.NaN : budgetMs;
            DrawPorkchop(in f, in plot, raster!, raster!.PaidLevel(onBoardMs));

            double top = departLo;
            float DepartY(double v) => (float)(plot.Y + (v - top) / departSpan * plot.H);
            double flightMin = TransferSession.FlightMinS, flightSpan = Math.Max(TransferSession.FlightSpanS, 1.0);
            float plotX = plot.X, plotW = plot.W;
            float FlightX(double v) => (float)(plotX + (v - flightMin) / flightSpan * plotW);
            Span<double> ticks = stackalloc double[8];
            int n = AxisTicks.Fill(Math.Max(departLo, 0.0), departLo + departSpan, departStep, ticks);
            Picture.TicksY(in f, in axes, ticks.Slice(0, n), DepartY, AxisTicks.Time);
            Picture.AxisX(in f, in axes, flightMin, flightMin + flightSpan, time: true, FlightX, AxisTicks.Time);
            Picture.LaneText(in f, in nameLane, PageText.FlightTimeAxis, Palette.Dim.Solid, 0);

            Picture.HeadText(in f, lanes.Head, PageText.DepartAxis, Palette.Dim.Solid, big);
            Picture.LaneText(in f, lanes.Head, forText, Palette.Text.Solid, 0);
            bool cannotPay = selected != null && BurnBudget.IsShort(budgetMs, selected.TransferDvVlf.Length());
            Picture.BigNumber(in f, big, cannotPay ? Palette.Warn.Solid : Palette.Text.Solid);

            var column = new PicBox(plot.Right + 8f * f.Scale, plot.Y, Math.Max(10f, f.Max.X - plot.Right - 8f * f.Scale), f.Max.Y - plot.Y);
            bool noneOwnRow = raster!.HasNoValue && !NoneKeyFits(f.Scale, column.W);
            float rampH = Picture.TextLaneHeight(noneOwnRow ? 4 : 3) + 8f * f.Scale;
            var sideFrame = new PictureFrame(new float2(column.X, column.Y), new float2(column.Right, column.Bottom - rampH), f.Dl, f.Scale);
            f.Dl.AddLine(new float2(column.X - 4f * f.Scale, f.Min.Y + head), new float2(column.X - 4f * f.Scale, f.Max.Y), Palette.Grid.Solid, 1f * f.Scale);
            if (HohmannFlybyUI.FlybyRequested && TransferSession.Destination is IParentBody target)
                DrawFlybyInset(in sideFrame, target);
            else
                DrawTransferSketch(in sideFrame, source);
            DrawRampLegend(in f, new PicBox(column.X, column.Bottom - rampH, column.W, rampH), raster!, onBoardMs, noneOwnRow);
        }
    }

    private static void DrawRampLegend(in PictureFrame f, in PicBox box, PorkchopRaster raster, double onBoardMs, bool noneOwnRow)
    {
        float s = f.Scale;
        float rowH = Picture.TextLaneHeight();
        var nameRow = new PicBox(box.X, box.Y, box.W, rowH);
        Picture.LaneText(in f, in nameRow, PageText.PorkchopRampName(TransferSession.SelectBestRel), Palette.Dim.Solid);
        if (raster.HasNoValue)
            DrawNoneKey(in f, noneOwnRow ? new PicBox(box.X, box.Bottom - rowH, box.W, rowH) : nameRow);
        float barY = box.Y + rowH;
        float barH = 6f * s;
        float x0 = box.X + 4f * s, x1 = box.Right - 4f * s;
        bool below = raster.BudgetBelow(onBoardMs), above = raster.BudgetAbove(onBoardMs);
        float arrow = 4f * s;
        if (below)
        {
            DrawArrow(in f, x0, barY + barH * 0.5f, arrow, Palette.Warn.Solid);
            x0 += arrow + 3f * s;
        }
        else if (above)
        {
            DrawArrow(in f, x1, barY + barH * 0.5f, -arrow, Palette.Text.Solid);
            x1 -= arrow + 3f * s;
        }
        const int Steps = 16;
        float w = (x1 - x0) / Steps;
        for (int i = 0; i < Steps; i++)
        {
            (byte r, byte g, byte b) = PorkchopRaster.LegendColor(i, Steps, raster.PaidShare(onBoardMs));
            f.Dl.AddRectFilled(new float2(x0 + i * w, barY), new float2(x0 + (i + 1) * w + 0.5f, barY + barH), new ImColor8(r, g, b, 255), 0f);
        }
        if (!double.IsFinite(raster.MinValue))
            return;
        var values = new PicBox(box.X, barY + barH, box.W, rowH);
        Picture.LaneText(in f, in values, Fmt.DvWhole(raster.MinValue), Palette.Dim.Solid);
        Picture.LaneText(in f, in values, Fmt.DvWhole(raster.MaxValue), Palette.Dim.Solid, 1);

        string onBoard = PageText.PorkchopOnBoard(onBoardMs);
        if (onBoard.Length == 0)
            return;
        double share = raster.LegendShare(onBoardMs);
        if (double.IsFinite(share))
        {
            float tick = x0 + (x1 - x0) * (float)share;
            f.Dl.AddLine(new float2(tick, barY - 2f * s), new float2(tick, barY + barH + 2f * s), Palette.Shadow, 3f * s);
            f.Dl.AddLine(new float2(tick, barY - 2f * s), new float2(tick, barY + barH + 2f * s), Palette.Flown.Solid, 1f * s);
        }
        ConsoleUi.PushSmall();
        if (ImGui.CalcTextSize(onBoard).X <= box.W - 8f * s)
            f.Dl.AddText(new float2(box.X + 4f * s, values.Y + rowH + (rowH - ImGui.GetFontSize()) * 0.5f), below ? Palette.Warn.Solid : Palette.Text.Solid, onBoard);
        ConsoleStyle.PopFont();
    }

    private static void DrawArrow(in PictureFrame f, float x, float y, float size, ImColor8 ink)
    {
        float high = Math.Abs(size);
        f.Dl.AddLine(new float2(x + size, y - high), new float2(x, y), ink, 1.5f * f.Scale);
        f.Dl.AddLine(new float2(x, y), new float2(x + size, y + high), ink, 1.5f * f.Scale);
    }

    private static bool NoneKeyFits(float s, float columnW)
        => 20f * s + ConsoleUi.SmallWidth(PageText.PorkchopRampName(TransferSession.SelectBestRel)) + 8f * s + ConsoleUi.SmallWidth(PageText.PorkchopNoneKey) <= columnW;

    private static void DrawNoneKey(in PictureFrame f, in PicBox row)
    {
        ConsoleUi.PushSmall();
        float s = f.Scale;
        float textW = ImGui.CalcTextSize(PageText.PorkchopNoneKey).X;
        float x = row.Right - textW - 4f * s;
        (byte r, byte g, byte b) = PorkchopRaster.LevelColor(PorkchopRaster.NoLevel);
        float side = 8f * s;
        float mid = row.Y + row.H * 0.5f;
        f.Dl.AddRectFilled(new float2(x - side - 4f * s, mid - side * 0.5f), new float2(x - 4f * s, mid + side * 0.5f), new ImColor8(r, g, b, 255), 1f);
        f.Dl.AddText(new float2(x, mid - ImGui.GetFontSize() * 0.5f), Palette.Dim.Solid, PageText.PorkchopNoneKey);
        ConsoleStyle.PopFont();
    }

    // The low frame while no porkchop exists: the progress of a calculation over the full width, or the one sentence.
    private static void DrawNoPorkchop(bool flying, CraftSituation where)
    {
        if (!TransferSession.Calculating)
        {
            if (where == CraftSituation.Ground && !flying)
                return;
            string refusal = TransferSession.CalculateRefusal();
            Picture.EmptyState(flying ? PageText.PorkchopPassesFly : refusal.Length > 0 ? refusal : PageText.PorkchopEmpty);
            return;
        }
        using (PictureScope pic = Picture.Open(Picture.EmptyHeight))
        {
            PictureFrame f = pic.Frame;
            float s = f.Scale;
            float p = TransferSession.Progress;
            float mid = f.Center.Y + 10f * s;
            f.Dl.AddRectFilled(new float2(f.Min.X + 10f * s, mid - 6f * s), new float2(f.Max.X - 10f * s, mid + 6f * s), Palette.Grid.A(0.5f), 3f * s);
            f.Dl.AddRectFilled(new float2(f.Min.X + 10f * s, mid - 6f * s), new float2(f.Min.X + 10f * s + (f.Width - 20f * s) * p, mid + 6f * s), Palette.Dim.A(0.85f), 3f * s);
            f.Dl.AddText(new float2(f.Min.X + 10f * s, mid - 28f * s), Palette.Text.Solid, string.Format(Inv, "Calculating {0:F0} %", p * 100f));
        }
    }

    private static void DrawPorkchop(in PictureFrame f, in PicBox plot, PorkchopRaster raster, int paidLevel)
    {
        float s = f.Scale;
        var min = new float2(plot.X, plot.Y);
        float size = plot.W;
        float cell = size / PorkchopRaster.Cells;
        foreach (PorkchopRaster.Run run in raster.Runs)
        {
            (byte r, byte g, byte b) = PorkchopRaster.CellColor(run.Level, paidLevel);
            f.Dl.AddRectFilled(new float2(min.X + run.Col0 * cell, min.Y + run.Row * cell),
                new float2(min.X + (run.Col1 + 1) * cell, min.Y + (run.Row + 1) * cell), new ImColor8(r, g, b, 255), 0f);
        }

        float unit = size / TransferSession.Size;
        float ringR = 7f * s;
        int2 sel = TransferSession.Selected;
        double ringEntries = (ringR + 5f * s) / unit;
        if (TransferSession.BestDv.X != sel.X || TransferSession.BestDv.Y != sel.Y)
            Marker(in f, min, size, unit, PorkchopRaster.MarkPlace(TransferSession.BestDv.X, TransferSession.BestDv.Y, sel.X, sel.Y, ringEntries, -1), Palette.Plan, 3.5f * s);
        if (TransferSession.BestDvRel.X != sel.X || TransferSession.BestDvRel.Y != sel.Y)
            Marker(in f, min, size, unit, PorkchopRaster.MarkPlace(TransferSession.BestDvRel.X, TransferSession.BestDvRel.Y, sel.X, sel.Y, ringEntries, 1), Palette.Good, 3.5f * s);
        if (sel.X >= 0)
        {
            float2 c = min + new float2((sel.Y + 0.5f) * unit, (sel.X + 0.5f) * unit);
            f.Dl.AddCircle(c, ringR, Palette.Shadow, 0, 3.5f * s);
            f.Dl.AddCircle(c, ringR, Palette.Flown.Solid, 0, 1.25f * s);
        }

        // The plot takes clicks only, never the keyboard.
        ImGui.SetCursorScreenPos(min);
        ImGui.InvisibleButton("##AfcPorkchop"u8, new float2(size, size));
        if (!ImGui.IsItemHovered())
            return;
        float2 mouse = ImGui.GetMousePos() - min;
        int cr = Math.Clamp((int)(mouse.Y / cell), 0, PorkchopRaster.Cells - 1);
        int cc = Math.Clamp((int)(mouse.X / cell), 0, PorkchopRaster.Cells - 1);
        (int row, int col2) = raster.EntryOf(cr, cc);
        if (row < 0)
        {
            ConsoleUi.Tooltip(PageText.PorkchopNoTransfer.AsSpan());
            return;
        }
        if (ImGui.IsItemClicked(ImGuiMouseButton.Left) && !TransferSession.Refining)
            TransferSession.Select(new int2(row, col2));
        OrbitalTransfers.PorkChopEntry? e = TransferSession.Info?.PorkChopData?[row, col2];
        if (e?.TransferData != null && ConsoleUi.BeginPlacedCard("TRANSFER POINT", "TRANSFER POINT".AsSpan()))
        {
            ConsoleWidgets.Readout("DEPART IN".AsSpan(), FormatHelper.FormatDuration(Math.Max(0.0, (e.TransferData.Start - Universe.GetElapsedTime()).Seconds())).AsSpan());
            ConsoleWidgets.Readout("FLIGHT TIME".AsSpan(), FormatHelper.FormatDuration(e.TransferData.Transit.Seconds()).AsSpan());
            ConsoleWidgets.Readout("DEPARTURE DELTA-V".AsSpan(), Fmt.Dv(e.TransferData.TransferDvVlf.Length()).AsSpan());
            ConsoleWidgets.Readout("ARRIVAL SPEED".AsSpan(), Fmt.Speed(e.TransferData.RelativeVelocityCci.Length()).AsSpan());
            ConsoleWidgets.EndTooltipCard();
        }
    }

    private static void Marker(in PictureFrame f, float2 min, float size, float unit, (double Row, double Col)? place, Rgb colour, float half)
    {
        if (place is not (double row, double col))
            return;
        float edge = half + 1.25f * f.Scale, halo = edge + 1.25f * f.Scale;
        var c = new float2(Math.Clamp(min.X + (float)col * unit, min.X + halo, min.X + size - halo), Math.Clamp(min.Y + (float)row * unit, min.Y + halo, min.Y + size - halo));
        f.Dl.AddRectFilled(c - new float2(halo), c + new float2(halo), Palette.Shadow, 1f);
        f.Dl.AddRectFilled(c - new float2(edge), c + new float2(edge), Palette.Flown.Solid, 1f);
        f.Dl.AddRectFilled(c - new float2(half), c + new float2(half), colour.Solid, 1f);
    }

    private const int ArcSamples = 48;
    private static readonly double3[] _arc = new double3[ArcSamples + 1];

    private static OrbitalTransfers.PorkChopEntry? _estimateOf;

    private static void DrawTransferSketch(in PictureFrame f, Vehicle source)
    {
        OrbitalTransfers.TransferInfo? info = TransferSession.Info;
        if (info?.Target == null || info.Source?.Orbit == null || info.Target.Orbit == null)
        {
            Picture.Empty(in f, "No destination");
            return;
        }
        _tracks.Clear();
        _tracks.Add(new OrbitSketch.Track(info.Source.Orbit, Palette.Flown.A(0.8f), 1.5f, false));
        _tracks.Add(new OrbitSketch.Track(info.Target.Orbit, Palette.Target.Solid, 1.5f, false));
        double3? depart = null, arrive = null;
        int points = 0;
        bool estimate = false;
        OrbitalTransfers.PorkChopEntry? entry = TransferSession.SelectedEntry;
        if (entry?.TransferData != null)
        {
            depart = info.Source.Orbit.GetStateVectorsAt(entry.TransferData.Start).PositionCci;
            arrive = info.Target.Orbit.GetStateVectorsAt(entry.TransferData.Arrival).PositionCci;
            points = SampleArc(info, source, entry);
            if (points == 0)
            {
                points = SampleImpulsiveArc(info, entry);
                estimate = points > 0;
            }
        }
        OrbitSketch.DrawTransfer(in f, info.Source.Orbit, info.Target.Orbit, _tracks, depart, arrive, _arc.AsSpan(0, points), estimate);
    }

    // A plan that never leaves the body has no patch around the parent, for example with too little fuel, so the sketch draws the path the porkchop solved (OrbitalTransfers.SolveLambert keeps its start and its velocity in the frame of that parent).
    private static int SampleImpulsiveArc(OrbitalTransfers.TransferInfo info, OrbitalTransfers.PorkChopEntry entry)
    {
        if (ReferenceEquals(_estimateOf, entry))
            return ArcSamples + 1;
        OrbitalTransfers.TransferData data = entry.TransferData;
        Orbit source = info.Source.Orbit;
        double from = data.Start.Seconds(), to = data.Arrival.Seconds();
        if (source.Parent == null || !(to > from) || data.SourceTransferVelocityCci.LengthSquared() <= 0.0)
            return 0;
        Orbit path = Orbit.CreateFromStateCci(source.Parent, data.Start, data.SourcePositionCci, data.SourceTransferVelocityCci, source.OrbitLineColor);
        for (int i = 0; i <= ArcSamples; i++)
            _arc[i] = path.GetStateVectorsAt(new UniverseTime(from + (to - from) * i / ArcSamples)).PositionCci;
        _estimateOf = entry;
        return ArcSamples + 1;
    }

    private static int SampleArc(OrbitalTransfers.TransferInfo info, Vehicle source, OrbitalTransfers.PorkChopEntry entry)
    {
        IParentBody? parent = info.Source.Orbit.Parent;
        double from = entry.TransferData.Start.Seconds(), to = entry.TransferData.Arrival.Seconds();
        foreach (PatchedConic patch in TransferFlownPlan.Kept(source, entry).Patches)
        {
            if (!ReferenceEquals(patch.Orbit.Parent, parent))
                continue;
            double t0 = Math.Max(from, patch.StartTime.Seconds()), t1 = Math.Min(to, patch.EndTime.Seconds());
            if (!(t1 > t0))
                continue;
            for (int i = 0; i <= ArcSamples; i++)
                _arc[i] = patch.Orbit.GetStateVectorsAt(new UniverseTime(t0 + (t1 - t0) * i / ArcSamples)).PositionCci;
            _estimateOf = null;
            return ArcSamples + 1;
        }
        return 0;
    }

    // The flyby seen from above the target: the periapsis circle asked for, the side, and the predicted periapsis.
    private static void DrawFlybyInset(in PictureFrame f, IParentBody target)
    {
        float s = f.Scale;
        float2 c = f.Center;
        float reach = Math.Min(f.Width, f.Height) * 0.36f;
        double radius = target.MeanRadius;
        bool requested = HohmannFlybyUI.TryGetRequest(target, out double pe, out FlybySide side);
        double predicted = HohmannFlybyUI.PredictedPeriapsisRadius;
        double outer = Math.Max(radius, Math.Max(requested ? pe : radius, double.IsFinite(predicted) ? predicted : radius)) * 1.15;
        float k = (float)(reach / outer);
        f.Dl.AddCircleFilled(c, (float)(radius * k), new ImColor8(40, 46, 56, 255));
        if (requested)
        {
            f.Dl.AddCircle(c, (float)(pe * k), Palette.Plan.Solid, 0, 1f * s);
            Picture.Text(in f, new float2(f.Min.X + 2f * s, f.Min.Y + 2f * s), HohmannFlybyUI.SideLabel, Palette.Plan.Solid);
        }
        if (double.IsFinite(predicted))
        {
            float2 dir = side switch
            {
                FlybySide.Outer => new float2(1f, 0f),
                FlybySide.North => new float2(0f, -1f),
                FlybySide.South => new float2(0f, 1f),
                _ => new float2(-1f, 0f),
            };
            float2 at = c + dir * (float)(predicted * k);
            Glyphs.Diamond(f.Dl, at, 5f * s, predicted < radius ? Palette.Bad.Solid : Palette.Good.Solid);
            Picture.Text(in f, new float2(f.Min.X + 2f * s, f.Max.Y - 18f * s),
                "Pe " + Fmt.Km((predicted - radius) / 1000.0), predicted < radius ? Palette.Bad.Solid : Palette.Good.Solid);
        }
        else
            Picture.Text(in f, new float2(f.Min.X + 2f * s, f.Max.Y - 18f * s), "No prediction", Palette.Dim.Solid);
    }

    #endregion

    #region Form

    private static void DrawSelection()
    {
        if (!TransferSession.Calculated)
            return;
        int picked = ConsoleUi.ChoiceRow("SELECT", "AfcTransferSelect", SelectLabels, TransferSession.SelectBestRel ? 1 : 0,
            cards: PageText.TransferSelectCards, labelTip: PageText.TransferSelectTip, feedsPicture: true, marks: SelectMarks);
        // A click on the active segment also jumps back to its best cell, after a porkchop click or MOVE took the selection away.
        if (picked >= 0 && !TransferSession.Refining)
        {
            TransferSession.SelectBestRel = picked == 1;
            TransferSession.Select(TransferSession.SelectBestRel && TransferSession.BestDvRel.X >= 0 ? TransferSession.BestDvRel : TransferSession.BestDv);
        }

        OrbitalTransfers.PorkChopEntry? entry = TransferSession.SelectedEntry;
        if (entry?.TransferData == null)
        {
            ConsoleUi.Muted("No solution found".AsSpan());
            return;
        }
        DrawMoveRow();
        if (TransferSession.Refining)
        {
            ImGui.SameLine();
            ConsoleUi.Muted("Refining the intercept".AsSpan());
        }
    }

    private static void DrawSelectionResult()
    {
        if (!TransferSession.Calculated || TransferSession.SelectedEntry?.TransferData is not OrbitalTransfers.TransferData t)
            return;
        ConsoleUi.Readout("DEPART IN".AsSpan(), FormatHelper.FormatDuration(Math.Max(0.0, (t.Start - Universe.GetElapsedTime()).Seconds())).AsSpan());
        ConsoleUi.Readout("FLIGHT TIME".AsSpan(), FormatHelper.FormatDuration(t.Transit.Seconds()).AsSpan());
        ConsoleUi.Readout("ARRIVAL SPEED".AsSpan(), Fmt.Speed(t.RelativeVelocityCci.Length()).AsSpan());
    }

    // A porkchop row is a departure time and a column a flight time (TransferTask.WorkerTask.CalculateAutomaticTransfer), so ^ and v move the departure and < and > the flight time, and each pair stands behind its name. A pane too narrow for the two pairs has them on two rows, under one column of names.
    private static void DrawMoveRow()
    {
        float2 origin = ImGui.GetCursorScreenPos();
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        ConsoleStyle.PushLabelFont();
        float rowY = origin.Y + (ConsoleStyle.FrameHeightPx - ImGui.GetFontSize()) * 0.5f;
        dl.AddText(new float2(origin.X, rowY), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextLabelU32), PageText.MovePick);
        ConsoleStyle.PopFont();
        float x = origin.X + ConsoleUi.LabelColumnWidth;
        float w = ConsoleWidgets.ButtonWidth("<".AsSpan()) * 1.4f;
        var size = new float2(w, ConsoleWidgets.ButtonHeight);
        float gap = ImGui.GetStyle().ItemSpacing.X;
        float departW = ConsoleUi.SmallWidth(PageText.DepartAxis), flightW = ConsoleUi.SmallWidth(PageText.FlightTimeAxis);
        bool twoRows = DockLayout.MoveRowWidth(ConsoleUi.LabelColumnWidth, departW, flightW, w, gap) > ConsoleStyle.ContentAvailWidth();
        ConsoleUi.NoteShape("AfcChopMoveRows", twoRows ? 1 : 0);
        float y = origin.Y;
        float pitch = ConsoleStyle.FrameHeightPx + ConsoleUi.RowGapPx;
        bool ready = TransferSession.Calculated && TransferSession.Selected.X >= 0;
        string held = !ready ? PageText.MoveDisabled : TransferSession.Refining ? "Refining the intercept" : "";
        int2[] steps = { new(-1, 0), new(1, 0), new(0, -1), new(0, 1) };
        for (int i = 0; i < steps.Length; i++)
        {
            if (i == 2 && twoRows)
            {
                x = origin.X + ConsoleUi.LabelColumnWidth;
                y += pitch;
            }
            if (i == 0 || i == 2)
            {
                ConsoleUi.PushSmall();
                string name = i == 0 ? PageText.DepartAxis : PageText.FlightTimeAxis;
                dl.AddText(new float2(x, y + (ConsoleStyle.FrameHeightPx - ImGui.GetFontSize()) * 0.5f), Palette.Dim.Solid, name);
                x += (twoRows ? Math.Max(departW, flightW) : ImGui.CalcTextSize(name).X) + gap;
                ConsoleStyle.PopFont();
            }
            ImGui.SetCursorScreenPos(new float2(x, y));
            x += w + (i == 1 ? gap * 3f : gap);
            bool clicked;
            using (new ImGuiDisabledScope(held.Length > 0))
                clicked = ConsoleWidgets.Button(PageText.MoveLabels[i].AsSpan(), ("AfcChopMove" + i.ToString(Inv)).AsSpan(), size);
            if (ImGui.IsItemHovered(ImGuiHoveredFlags.AllowWhenDisabled))
                ConsoleUi.Tooltip((held.Length > 0 ? held + "\n\n" + PageText.MoveTips[i] : PageText.MoveTips[i]).AsSpan());
            if (clicked && held.Length == 0)
                TransferSession.Step(steps[i].X, steps[i].Y);
        }
        ImGui.SetCursorScreenPos(origin);
        ImGui.Dummy(new float2(ConsoleStyle.ContentAvailWidth(), y - origin.Y + ConsoleStyle.FrameHeightPx));
    }

    private static void DrawCorrectionRow()
    {
        if (!TransferSession.CorrectionOffered && !TransferSession.CorrectionCalculating)
            return;
        double peKm = TransferSession.CorrectionPeAltitude / 1000.0;
        if (ConsoleUi.InputDoubleRow("CORRECTION PE".AsSpan(), "##AfcCorrectionPe"u8, ref peKm, 10.0, 100.0, "%.1f"u8, unit: "km"))
            TransferSession.CorrectionPeAltitude = Math.Max(0.0, peKm) * 1000.0;
        if (TransferSession.CorrectionCalculating)
            ConsoleUi.Muted("Calculating the correction".AsSpan());
        else if (TransferSession.CorrectionFailed)
            ConsoleUi.WarningWrapped("The last correction found no solution.");
    }

    private static void CollectBars()
    {
        _bars.Clear();
        PassPreview[] passes = HohmannMultiPassUI.PreviewPasses;
        if (HohmannMultiPassUI.HasMultiPassPreview && passes.Length > 0)
        {
            int first = 1;
            if (TransferSession.Source != null && MultiPassRegistry.TryGet(TransferSession.Source.Id, out MultiPassExecution? exec))
                first = exec.PassIndex + 1;
            PageKit.AddPassBars(_bars, passes, first, first > 1, false);
        }
        else if (TransferSession.SelectedEntry?.TransferData is OrbitalTransfers.TransferData t)
            _bars.Add(new BurnBar("DEPART", t.TransferDvVlf.Length(), double.NaN, TransferSession.TransferBurn != null));
    }

    private static void DrawFlightTimeRows()
    {
        if (TransferSession.Info is not OrbitalTransfers.TransferInfo info)
            return;
        string card = PageText.FlightTimeTip(FormatHelper.FormatDuration(info.HohmannTimeOfFlight.Seconds())) + "\n" + PageText.TimeFieldTip;
        using (new ImGuiDisabledScope(TransferSession.Calculating))
        {
            double minTof = TransferSession.MinTofSec;
            double maxTof = TransferSession.MaxTofSec;
            if (ConsoleUi.TimeRow("MIN FLIGHT TIME", "AfcTofMin", ref minTof, card))
                TransferSession.MinTofSec = Math.Max(60.0, minTof);
            if (ConsoleUi.TimeRow("MAX FLIGHT TIME", "AfcTofMax", ref maxTof, card))
                TransferSession.MaxTofSec = Math.Max(TransferSession.MinTofSec + 60.0, maxTof);
        }
    }

    private static void DrawSetTargetRow(Vehicle source)
    {
        bool setTarget = _setTarget;
        if (ConsoleUi.CheckboxRow("SET AS CRAFT TARGET".AsSpan(), "AfcTransferSetTarget".AsSpan(), ref setTarget,
                "Makes the destination the stock target of the craft, as SET TARGET does in the stock planner.".AsSpan(), stockChip: true))
        {
            _setTarget = setTarget;
            InputEvents.ChangeTargetBuffer.Add(new InputEvents.ChangeTargetData
            {
                Vehicle = source,
                Target = setTarget ? TransferSession.Destination : null,
            });
        }
    }

    #endregion

    #region Commit

    private static CommitSpec BuildCommit(Vehicle source, CraftSituation where, bool ascentFlies, bool otherFlies)
    {
        int passes = HohmannMultiPassUI.Enabled ? HohmannMultiPassUI.PassCount : 1;
        bool flyby = HohmannFlybyUI.FlybyRequested;
        string verb = passes > 1 ? PlanVerbs.Create(passes) : flyby ? "CREATE FLYBY DEPARTURE" : "CREATE TRANSFER";

        if (PassesHoldFooter(source, out MultiPassExecution? exec))
            return PageKit.FlyPass(source, exec!);

        if (TransferSession.TransferBurn is Burn departure)
        {
            string created = PageText.TransferCreated(departure.DeltaVVlf.Length(),
                FormatHelper.FormatDuration(Math.Max(0.0, (departure.Time - Universe.GetElapsedTime()).Seconds())));
            if (TransferSession.CorrectionOffered && !TransferSession.CorrectionCalculating)
                return new CommitSpec("CREATE CORRECTION", FlownBy.StockAuto, "", "Transfer misses the target", true,
                    "Calculates a course correction burn that brings the periapsis at the target to the altitude above.",
                    Act: TransferSession.StartCorrection, Owner: Owner.AfcPlansStockFlies, Outcome: PageText.CorrectionOutcome);
            if (!ReferenceEquals(source, Program.ControlledVehicle))
                return CommitSpec.StatusOnly(created, false, FlownBy.StockAuto);
            if (BurnModeInput.StockAutoOnNode(source))
                return PageKit.StockAutoFlies(source);
            CommitSpec fly = PageKit.FlyNextNode(source, created);
            return fly.Verb.Length > 0 ? fly with { Caution = _fuelFact } : fly;
        }

        if (TransferSession.Calculating)
            return new CommitSpec(PageText.CancelVerb, FlownBy.Nobody, Tooltip: PageText.CancelTransferTip, Act: TransferSession.CancelCalculation, Quiet: true);

        VerbFit fit = SituationVerbs.ForTransfer(where, ascentFlies, otherFlies);
        if (!TransferSession.Calculated)
            return BuildCalculate(source, in fit);

        string outcome = passes > 1 ? PageText.PassesOutcome(passes) : PageText.CreateOutcome;
        string refusal = TransferCommitter.Refusal();
        if (refusal.Length == 0 && passes > 1 && HohmannMultiPassUI.WantedMultiPassButPreviewFailed())
            refusal = "Pass preview failed";
        string status = CreateAnswer(passes);
        string caution = fit.Text;
        if (caution.Length == 0)
            caution = _fuelFact;
        if (caution.Length == 0)
            caution = HohmannMultiPassUI.FuelShortFact();
        bool formWarns = caution.Length == 0 && ConsoleUi.PageWarning.Length > 0;
        return new CommitSpec(verb, PlanVerbs.FlownByFor(passes), refusal, status, false,
            PageText.CreateCard("the pick on the plot", passes > 1),
            Act: TransferCommitter.Create, Owner: Owner.AfcPlansStockFlies, Outcome: outcome, Secondary: PageKit.NextLink(in fit), Quiet: formWarns, Caution: caution, Calm: !fit.Fits && fit.Calm, PlannedBurns: passes);
    }

    private static CommitSpec BuildCalculate(Vehicle source, in VerbFit fit)
    {
        string refusal = TransferSession.CalculateRefusal();
        if (refusal.Length > 0 && _destinations.Count == 0)
            refusal = PageText.NoDestination(source.Parent?.Id ?? "");
        return new CommitSpec(PlanVerbs.Calculate, FlownBy.Nobody, refusal, Tooltip: PageText.CalculateTip, Act: TransferSession.StartCalculation,
            Outcome: PageText.CalculateOutcome, Secondary: PageKit.NextLink(in fit), Caution: fit.Text, Calm: fit.Calm);
    }

    private static string CreateAnswer(int passes)
    {
        if (TransferSession.SelectedEntry?.TransferData is not OrbitalTransfers.TransferData t)
            return "";
        string when = passes > 1
            ? string.Create(Inv, $"in {passes} passes")
            : "in " + FormatHelper.FormatDuration(Math.Max(0.0, (t.Start - Universe.GetElapsedTime()).Seconds()));
        return PageText.BurnAnswer(passes > 1 && double.IsFinite(BurnBars.LastTotalMs) ? BurnBars.LastTotalMs : t.TransferDvVlf.Length(), when, BurnBars.LastOnBoardMs);
    }

    #endregion

    #region World

    /// <summary>The markers of the selected transfer and its passes, after the console window has ended.</summary>
    public static void DrawAfterWindow(IGameViewport viewport)
    {
        Vehicle? source = TransferSession.Source;
        if (source == null || source.IsDisposed)
            return;
        if (HohmannMultiPassUI.ShouldRenderOverlay(out Vehicle? passSource))
            HohmannMultiPassUI.RenderMarkers(viewport, passSource!);
        if (!SelectedPreviewVisible() || HohmannFlybyUI.ReplacesCenterAimedPreview())
            return;
        OrbitalTransfers.PorkChopEntry? entry = TransferSession.SelectedEntry;
        if (entry == null)
            return;
        var context = new Astronomical.UiContext(viewport, source, Color.Green, TrueAnomaly.Zero, new TrueAnomaly(Math.PI * 2.0), TransferSession.Destination);
        TransferFlownPlan.Plan(source, entry).DrawUi(viewport, context);
    }

    /// <summary>
    /// The lines of the selected transfer, its passes and its flyby, from the CelestialSystem.OnPreRender postfix. The pass overlay draws every pass, the final one too, and then the selected transfer is not drawn, because multi-pass never flies it as one burn.
    /// </summary>
    public static void RenderLines(IViewport viewport)
    {
        if (HohmannMultiPassUI.ShouldRenderOverlay(out Vehicle? passSource))
        {
            HohmannMultiPassUI.RenderOrbits(viewport, passSource!);
            return;
        }
        if (HohmannFlybyUI.ShouldRenderPreview(out Vehicle? flybySource))
        {
            HohmannFlybyUI.RenderPreview(viewport, flybySource!);
            return;
        }
        RenderSelectedLines(viewport);
    }

    // The plan the engines fly (TransferFlownPlan), as stock TransferPlanner.DrawSelectedTransfer draws SelectedPlan.
    private static void RenderSelectedLines(IViewport viewport)
    {
        Vehicle? source = TransferSession.Source;
        OrbitalTransfers.PorkChopEntry? entry = TransferSession.SelectedEntry;
        if (source == null || entry == null || !SelectedPreviewVisible())
            return;
        FlightPlan plan = TransferFlownPlan.Plan(source, entry);
        if (plan.Patches.Count == 0)
            return;
        if (plan.Patches[0].Orbit.IsMissingPoints())
        {
            foreach (PatchedConic patch in plan.Patches)
            {
                patch.HidePatch = false;
                MemoryOwner<OrbitPointCce> points = UpdateTaskUtils.GenerateSpacedPoints(patch);
                patch.Orbit.UpdateCachedPoints(points);
            }
        }
        plan.AddLineInstances(viewport, source, isActive: true, drawVehiclePosition: false, nextBurnTime: null);
    }

    // The selected transfer draws in the world while MAP is on and TRANSFER is on screen, as stock draws its selection only while its window is open. Once the final pass of a multi-pass is queued, stock draws it as the node of the flight plan.
    private static bool SelectedPreviewVisible()
        => TransferSession.ShowMap && TransferSession.Calculated && TransferSession.PageVisible && !HohmannMultiPassUI.FinalPassQueued;

    #endregion

    public static void Reset()
    {
        _estimateOf = null;
        TransferSession.PageVisible = false;
        _setTarget = false;
        _fuelFact = "";
        _destinations.Clear();
        _sources.Clear();
        _tracks.Clear();
        _bars.Clear();
        TransferFlownPlan.Reset();
    }
}
