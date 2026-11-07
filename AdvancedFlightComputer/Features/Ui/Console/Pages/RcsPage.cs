using System;
using System.Collections.Generic;
using System.Globalization;
using System.Runtime.InteropServices;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoRemove;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.Features.Ui.Pictures;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console.Pages;

/// <summary>BURNS (key RCS): the list of every planned burn of the controlled craft and the passes still to come, and the rows that set the picked burn. The verb is FLY BURN, with DELETE BURN beside it, the same event the stock BURN gauge sends, so the RCS executor decides exactly as it does for the gauge, and it shows only for the next burn, the one stock loads.</summary>
internal static class RcsPage
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    // In the order of PageText.AttitudeLabels. FREE is RcsAttitudeStrategy.Auto; the saved key in rcs-exec.toml stays the enum name.
    private static readonly RcsAttitudeStrategy[] AttitudeOrder = { RcsAttitudeStrategy.Auto, RcsAttitudeStrategy.Hold, RcsAttitudeStrategy.Align };
    private static readonly Comparison<ListRow> ByTime = static (a, b) => a.Seconds.CompareTo(b.Seconds);
    private static readonly List<ListRow> _rows = new();
    private static readonly List<TableMark> _marks = new();
    private static readonly List<CellSwatch> _swatches = new();

    private readonly record struct ListRow(Burn? Burn, int Pass, double Seconds, double DvMs, bool Flies, bool Rcs, double RcsKg, double ChargeMs);

    private readonly record struct Picked(
        Vehicle Vehicle, Burn Node, bool IsNext, RcsExecution? Exec, RcsBurnOptions? Options, bool Active, RcsExecutionMode Mode, RcsExecutionMode Resolved,
        bool HasThrusters, bool CanTranslate, RcsEstimates Est, bool Estimates, double Available, double Need)
    {
        public bool RcsNow => BurnList.RcsTakes(Active, Mode == RcsExecutionMode.Rcs, Resolved == RcsExecutionMode.Rcs, CanTranslate);
    }

    private static Burn? _focus;

    internal static void OpenFor(Burn burn)
    {
        _focus = burn;
        AfcConsole.Open(ConsolePage.Rcs);
    }

    internal static void Reset() => _focus = null;

    public static CommitSpec Draw(in PageContext ctx)
    {
        try
        {
            return DrawPage(in ctx);
        }
        finally
        {
            _rows.Clear();
        }
    }

    private static CommitSpec DrawPage(in PageContext ctx)
    {
        Vehicle vehicle = ctx.Subject;
        RcsExecution? exec = null;
        if (SharedVehicleHooks.RcsEnabled)
            RcsExecRegistry.TryGet(vehicle.Id, out exec);
        if (exec?.Faulted == true)
        {
            ConsoleUi.DangerWrapped(exec.CleanupPending ? "RCS STOPPED. CLEANUP FAILED." : "RCS STOPPED: INTERNAL ERROR. SEE LOG.");
            if (exec.CleanupPending && ConsoleWidgets.DangerButton("RETRY CLEANUP".AsSpan(), "AfcRcsRetry".AsSpan(), default))
                RcsStop.RetryCleanup(exec);
        }

        Burn? next = BurnModeInput.NextNode(vehicle);
        Burn? node = FocusedNode(vehicle, next);
        MultiPassExecution? passes = null;
        if (SharedVehicleHooks.MultiPassEnabled)
            MultiPassRegistry.TryGet(vehicle.Id, out passes);
        CollectRows(vehicle, next, exec, passes);
        ConsoleUi.NoteShape("AfcRcsList", _rows.Count);
        if (_rows.Count == 0)
            return DrawEmpty(in ctx);

        StageFuel[] stages = BurnBars.StagesOf(vehicle);
        BurnPay[] pays = PayRows(stages);
        ReadRcsFuel(vehicle, exec);

        if (node == null)
        {
            DrawList(vehicle, next, null, stages, pays, openPlan: false);
            ctx.ToInputs();
            DrawAutoRemove();
            return passes != null ? PageKit.FlyPass(vehicle, passes) : CommitSpec.None;
        }

        Picked picked = ReadPicked(vehicle, node, ReferenceEquals(node, next), exec);
        bool rcsPicture = BurnList.RcsPictureShows(picked.Active, picked.Resolved == RcsExecutionMode.Rcs, picked.CanTranslate);
        ConsoleUi.NoteShape("AfcRcsRows", picked.RcsNow ? 1 : 0);

        if (ctx.Flying)
            DrawFlying(in ctx, in picked, next, stages, pays, rcsPicture);
        else
        {
            DrawList(vehicle, next, node, stages, pays, openPlan: false);
            if (!picked.RcsNow || _rows.Count > 1)
                DrawOnBoard(stages);
            bool pictureLeft = rcsPicture && AfcConsole.HasPanes;
            if (pictureLeft)
                DrawPicture(in picked);
            ctx.ToInputs();
            DrawSetup(in picked, rcsPicture && !pictureLeft);
            DrawAutoRemove();
            ConsoleUi.Gap();
            DrawHowItFlies(in ctx, in picked);
        }

        return BuildCommit(in picked, FuelFact(stages, pays, next), passes, ctx.Frame.Activity);
    }

    private static CommitSpec DrawEmpty(in PageContext ctx)
    {
        ConsoleUi.Muted(PageText.NoBurnPlanned.AsSpan());
        ctx.ToInputs();
        DrawAutoRemove();
        ConsolePage page = BurnList.EmptyNext(ctx.Frame.Situation.Where);
        string cue = ConsolePages.CueTarget(page);
        return CommitSpec.StatusOnly("") with
        {
            Secondary = new CommitAction(PageText.NextLabel(page, null), "", ConsolePages.HoverCard(page), () => AfcConsole.Navigate(cue), Danger: false, Lead: true),
        };
    }

    private static void DrawFlying(in PageContext ctx, in Picked picked, Burn? next, StageFuel[] stages, BurnPay[] pays, bool rcsPicture)
    {
        if (rcsPicture)
            DrawPicture(in picked);
        else
            OrbitPage.DrawBurnSketch(in ctx, picked.Vehicle, picked.Node);
        if (BurnList.ListShowsWhileFlying(_rows.Count))
        {
            DrawList(picked.Vehicle, next, picked.Node, stages, pays, openPlan: true);
            DrawOnBoard(stages);
        }
        else if (!picked.RcsNow && stages.Length > 0)
        {
            BurnPay pay = pays[0];
            string shortage = FuelFact(stages, pays, next);
            ConsoleUi.NoteShape("AfcRcsFuelShort", shortage.Length > 0 ? 1 : 0);
            if (shortage.Length > 0)
                ConsoleUi.Warning(shortage.AsSpan());
            else if (pay.ShortMs <= 0.5)
            {
                bool marked = BurnBars.PayerHues(picked.Vehicle, stages, in pay, out Rgb8 hue, out Rgb8? second);
                ConsoleUi.StageSentence(PageText.BurnFuelLine(pay.FirstStage >= 0 ? BurnBudget.Payer(stages, in pay) : "", pay.PropellantKg, pay.ShortMs).AsSpan(),
                    in ConsoleStyle.TextMuted, marked ? hue : null, second);
            }
        }

        ConsoleUi.Gap();
        if (!ConsoleUi.Fold(FlightView.PlanLabel(PlanWords(in picked)), FlightView.PlanFoldId, false, PageText.PlanTip) || !ctx.ToPlanForm())
            return;
        ctx.PlanHead();
        DrawSetup(in picked, rcsPicture: false);
        if (picked.Active && picked.Exec != null)
            ConsoleUi.Readout(PageText.RcsInUse.AsSpan(), ((picked.Exec.ResolvedStrategy == RcsAttitudeStrategy.Align
                ? "ALIGN " + RcsExecutor.AxisName(picked.Exec.ResolvedAxis)
                : "HOLD") + ", " + RcsGaugePanel.ResolvedAllocator(picked.Exec)).AsSpan());
        DrawAutoRemove();
        ConsoleUi.Gap();
        DrawHowItFlies(in ctx, in picked);
        if (!picked.Active)
        {
            Vehicle vehicle = picked.Vehicle;
            Burn node = picked.Node;
            (string verb, string card, string line) = PageKit.DeleteOf(vehicle, node, ctx.Frame.Activity);
            if (ConsoleUi.ButtonRight(verb, "AfcRcsDelete"))
                BurnModeInput.DeleteNode(vehicle, node);
            if (ImGui.IsItemHovered())
                ConsoleUi.Tooltip(card.AsSpan());
            ConsoleUi.NoteShape("AfcRcsDeleteLine", line.Length > 0 ? 1 : 0);
            if (line.Length > 0)
                ConsoleUi.Muted(line.AsSpan());
        }
    }

    private static string PlanWords(in Picked p)
    {
        (string goal, int passes) = OrbitPage.RunningGoal(p.Vehicle);
        if (goal.Length > 0)
            return PageText.PassesPlan(goal, passes);
        Orbit? now = p.Vehicle.Orbit;
        Orbit? after = OrbitToolsPanel.OrbitAfter(p.Node);
        return now == null || after == null ? ""
            : PageText.BurnGoal(now.PeriapsisAltitude / 1000.0, now.ApoapsisAltitude / 1000.0, after.PeriapsisAltitude / 1000.0, after.ApoapsisAltitude / 1000.0, after.Eccentricity < 1.0);
    }

    private static Picked ReadPicked(Vehicle vehicle, Burn node, bool isNext, RcsExecution? exec)
    {
        double timeSec = node.Time.Seconds();
        double dvMs = node.DeltaVVlf.Length();
        RcsBurnOptions? options = exec?.FindOptions(timeSec, dvMs);
        bool active = exec != null && exec.IsActive && options != null
            && options.Matches(exec.ActiveBurnTimeSec!.Value, exec.ActiveBurnDvMs!.Value);
        bool hooks = SharedVehicleHooks.RcsEnabled;
        bool hasThrusters = hooks && vehicle.Parts.Modules.Get<ThrusterController>().Length > 0;
        RcsEstimates est = default;
        bool estimates = hasThrusters && !active
            && RcsBurnPreview.TryGetEstimates(node, vehicle, vehicle.FlightComputer, exec, out est, out _);
        return new Picked(vehicle, node, isNext, exec, options, active, options?.Mode ?? RcsExecutionMode.Default, hooks ? RcsExecutor.ResolveMode(vehicle, options) : RcsExecutionMode.Engine,
            hasThrusters, BurnModeInput.CanTranslate(vehicle), est, estimates,
            hasThrusters ? RcsExecutor.AvailablePropellantCached(vehicle) : 0.0,
            estimates ? est.RequiredPropellantKg(options?.Attitude ?? RcsAttitudeStrategy.Auto) : 0.0);
    }

    // The picked burn while it is still planned on this craft, otherwise the next burn.
    private static Burn? FocusedNode(Vehicle vehicle, Burn? next)
    {
        if (_focus == null)
            return next;
        if (!ReferenceEquals(_focus.Vehicle, vehicle) || _focus.ParentDepartureBurn
            || vehicle.FlightComputer.BurnPlan.TryGetBurnIndex(_focus) < 0)
        {
            _focus = null;
            return next;
        }
        return _focus;
    }

    // The stock plan keeps its burns sorted by time, and a pass to come can lie between them.
    private static void CollectRows(Vehicle vehicle, Burn? next, RcsExecution? exec, MultiPassExecution? passes)
    {
        _rows.Clear();
        FlightComputer fc = vehicle.FlightComputer;
        BurnPlan plan = fc.BurnPlan;
        double now = Universe.GetElapsedSeconds();
        bool burnFlies = BurnModeInput.BurnFlies(vehicle);
        bool canTranslate = BurnModeInput.CanTranslate(vehicle);
        for (int i = 0; i < plan.BurnCount; i++)
        {
            if (!plan.TryGetBurn(i, out Burn? burn) || burn == null || burn.ParentDepartureBurn)
                continue;
            double timeSec = burn.Time.Seconds();
            double dvMs = burn.DeltaVVlf.Length();
            RcsBurnOptions? options = exec?.FindOptions(timeSec, dvMs);
            bool rcsFlies = exec != null && exec.IsActive && options != null && options.Matches(exec.ActiveBurnTimeSec!.Value, exec.ActiveBurnDvMs!.Value);
            bool rcs = BurnList.RcsTakes(rcsFlies, options?.Mode == RcsExecutionMode.Rcs, RcsExecutor.ResolveMode(vehicle, options) == RcsExecutionMode.Rcs, canTranslate);
            bool flies = burnFlies && ReferenceEquals(burn, next);
            double toGo = fc.Burn is BurnTarget loaded && StockBurnIdentity.IsLoaded(loaded, burn) ? loaded.DeltaVToGoCci.Length() : dvMs;
            int pass = passes != null && ReferenceEquals(burn, passes.CurrentBurn) ? passes.PassIndex + 1 : 0;
            _rows.Add(new ListRow(burn, pass, timeSec - now, dvMs, flies, rcs, double.NaN, BurnList.Charge(timeSec - now, flies, dvMs, toGo)));
        }

        if (passes != null)
        {
            PassPreview[] later = MultiPassUI.RunningPasses(vehicle, passes);
            bool rcs = BurnModeInput.ResolvesToRcs(vehicle);
            for (int i = passes.CurrentBurn != null ? 1 : 0; i < later.Length; i++)
            {
                double dvMs = later[i].DvVlf.Length();
                _rows.Add(new ListRow(null, passes.PassIndex + 1 + i, later[i].BurnTime.Seconds() - now, dvMs, false, rcs, double.NaN, dvMs));
            }
        }
        _rows.Sort(ByTime);
    }

    public static double[] PlannedEngineCharges(Vehicle vehicle)
    {
        RcsExecution? exec = null;
        if (SharedVehicleHooks.RcsEnabled)
            RcsExecRegistry.TryGet(vehicle.Id, out exec);
        MultiPassExecution? passes = null;
        if (SharedVehicleHooks.MultiPassEnabled)
            MultiPassRegistry.TryGet(vehicle.Id, out passes);
        CollectRows(vehicle, BurnModeInput.NextNode(vehicle), exec, passes);
        var charges = new double[_rows.Count];
        for (int i = 0; i < charges.Length; i++)
            charges[i] = _rows[i].Rcs ? 0.0 : _rows[i].ChargeMs;
        _rows.Clear();
        return charges;
    }

    private static BurnPay[] PayRows(StageFuel[] stages)
    {
        var dvs = new double[_rows.Count];
        for (int i = 0; i < dvs.Length; i++)
            dvs[i] = _rows[i].Rcs ? 0.0 : _rows[i].ChargeMs;
        return BurnBudget.Pay(stages, dvs);
    }

    private static void ReadRcsFuel(Vehicle vehicle, RcsExecution? exec)
    {
        for (int i = 0; i < _rows.Count; i++)
        {
            ListRow row = _rows[i];
            if (!row.Rcs || row.Burn is not Burn burn)
                continue;
            RcsAttitudeStrategy attitude = exec?.FindOptions(burn.Time.Seconds(), row.DvMs)?.Attitude ?? RcsAttitudeStrategy.Auto;
            if (RcsBurnPreview.TryGetEstimates(burn, vehicle, vehicle.FlightComputer, exec, out RcsEstimates est, out _))
                _rows[i] = row with { RcsKg = est.RequiredPropellantKg(attitude) };
        }
    }

    private static string FuelFact(StageFuel[] stages, BurnPay[] pays, Burn? next)
    {
        double needed = 0.0;
        for (int i = 0; i < _rows.Count && i < pays.Length; i++)
        {
            if (!_rows[i].Rcs)
                needed += _rows[i].ChargeMs;
            if (ReferenceEquals(_rows[i].Burn, next))
                return !_rows[i].Rcs && pays[i].ShortMs > 0.5 ? PageText.FuelShort(needed, BurnBudget.OnBoard(stages)) : "";
        }
        return "";
    }

    private static void DrawList(Vehicle vehicle, Burn? next, Burn? node, StageFuel[] stages, BurnPay[] pays, bool openPlan)
    {
        var cells = new string[_rows.Count][];
        var held = new string[_rows.Count];
        int active = -1;
        for (int i = 0; i < cells.Length; i++)
        {
            if (_rows[i].Burn != null && ReferenceEquals(_rows[i].Burn, node))
                active = i;
        }
        _marks.Clear();
        _swatches.Clear();
        for (int i = 0; i < cells.Length; i++)
        {
            ListRow row = _rows[i];
            string fuel = row.Rcs ? BurnList.RcsFuel(row.RcsKg) : BurnList.EngineFuel(stages, in pays[i]);
            cells[i] = BurnList.Row(i, row.Pass, row.Seconds, row.Flies, row.DvMs, row.Rcs, fuel);
            held[i] = row.Burn == null ? PageText.PassNotAddedYet : "";
            BurnList.MarksOf(i, i == active, row.Rcs, !row.Rcs && stages.Length > 0 && pays[i].ShortMs > 0.5, _marks);
            if (!row.Rcs && pays[i].ShortMs <= 0.5 && BurnBars.PayerHues(vehicle, stages, in pays[i], out Rgb8 hue, out Rgb8? second))
                _swatches.Add(new CellSwatch(i, BurnList.FuelSwatchColumn, hue, second));
        }
        int picked = ConsoleUi.ChoiceTable("AfcRcsBurns", BurnList.Heads, cells, active, held, default, PageText.BurnListTip, default,
            CollectionsMarshal.AsSpan(_marks), CollectionsMarshal.AsSpan(_swatches));
        if (picked < 0 || _rows[picked].Burn is not Burn chosen)
            return;
        _focus = ReferenceEquals(chosen, next) ? null : chosen;
        if (openPlan)
            ConsoleUi.OpenFold(FlightView.PlanFoldId);
    }

    private static void DrawOnBoard(StageFuel[] stages)
    {
        if (stages.Length > 0)
            ConsoleUi.Readout("ON BOARD".AsSpan(), string.Create(Inv, $"~{BurnBudget.OnBoard(stages):F0} m/s").AsSpan());
    }

    private static void DrawSetup(in Picked p, bool rcsPicture)
    {
        if (!SharedVehicleHooks.RcsEnabled)
        {
            ConsoleUi.MutedWrapped("RCS translation is unavailable. Its game hooks did not load.");
            return;
        }
        if (!p.HasThrusters)
        {
            ConsoleUi.Muted(PageText.NoRcsThrusters.AsSpan());
            return;
        }

        Vehicle vehicle = p.Vehicle;
        // RcsExecutor reads this choice when AUTO is pressed, so it counts again only after the stop of stock AUTO.
        bool stockFlies = p.IsNext && BurnModeInput.StockAutoOnNode(vehicle);
        BurnOptionRow(vehicle, p.Node, p.Active || stockFlies, "BURN WITH", "AfcRcsFlyWith", PageText.FlyWithLabels, PageText.FlyWithCards,
            stockFlies ? PageText.FlyWithHeld : "", PageText.FlyWithTip,
            static o => o?.Mode == RcsExecutionMode.Rcs ? 1 : 0, static (o, i) => o.Mode = i == 1 ? RcsExecutionMode.Rcs : RcsExecutionMode.Default);
        if (rcsPicture)
            DrawPicture(in p);
        if (p.RcsNow)
            DrawAttitudeRow(vehicle, p.Node, p.Active, "");
        if (p.RcsNow && !p.CanTranslate && !p.IsNext)
            ConsoleUi.Muted(PageText.RcsCannotTranslate.AsSpan());
    }

    private static void DrawHowItFlies(in PageContext ctx, in Picked p)
    {
        Vehicle vehicle = p.Vehicle;
        Burn node = p.Node;
        bool active = p.Active;
        bool attitudeInFold = p.HasThrusters && !p.RcsNow && p.Mode == RcsExecutionMode.Default;
        int changed = (p.Options != null && p.Options.Allocator != RcsAllocator.Groups ? 1 : 0)
            + (attitudeInFold && p.Options != null && p.Options.Attitude != RcsAttitudeStrategy.Auto ? 1 : 0);
        PageKit.HowItFlies("rcs",
        [
            new FoldPart(FoldNames.Attitude, attitudeInFold, () => DrawAttitudeRow(vehicle, node, active, PageText.AttitudeIfRcsNote), PageText.AttitudeLabels[Array.IndexOf(AttitudeOrder, p.Options?.Attitude ?? RcsAttitudeStrategy.Auto)]),
            new FoldPart(FoldNames.Allocator, p.HasThrusters, () => DrawAllocatorRow(vehicle, node, active), PageText.AllocatorLabels[p.Options?.Allocator == RcsAllocator.Lp ? 1 : 0]),
        ], changed, ctx.FoldsOpen);
    }

    private static void DrawAutoRemove()
    {
        if (SharedVehicleHooks.AutoRemoveEnabled)
            AutoRemoveFeature.DrawSwitch();
    }

    private static CommitSpec BuildCommit(in Picked p, string fuelFact, MultiPassExecution? passes, in CraftActivity activity)
    {
        Vehicle craft = p.Vehicle;
        Burn node = p.Node;
        if (p.Active)
            return CommitSpec.StatusOnly("AFC: RCS BURN", true, FlownBy.Afc);
        (string deleteVerb, string deleteCard, string deleteLine) = PageKit.DeleteOf(craft, node, in activity);
        var delete = new CommitAction(deleteVerb, "", deleteCard, () => BurnModeInput.DeleteNode(craft, node));
        if (BurnModeInput.StockAutoOnNode(craft))
            return p.IsNext ? PageKit.StockAutoFlies(craft) : CommitSpec.StatusOnly("STOCK AUTO: NEXT BURN", true, FlownBy.StockAuto) with { Secondary = delete };

        bool rcs = p.RcsNow;
        if (!p.IsNext)
            return CommitSpec.StatusOnly("SET FOR LATER BURN", false, rcs ? FlownBy.Afc : FlownBy.StockAuto) with { Secondary = delete };
        string refusal = p.Resolved == RcsExecutionMode.Rcs ? BurnModeInput.RcsRefusal(p.CanTranslate, p.Mode == RcsExecutionMode.Rcs, p.Estimates, p.Need, p.Available) : "";
        if (refusal.Length == 0)
            refusal = BurnModeInput.FlyRefusal(craft);
        bool pass = passes != null && ReferenceEquals(node, passes.CurrentBurn);
        return new CommitSpec(pass ? PlanVerbs.FlyPass(passes!.PassIndex + 1) : PlanVerbs.FlyBurn, rcs ? FlownBy.Afc : FlownBy.StockAuto, refusal, deleteLine, false,
            rcs ? PageText.FlyRcsTip : PageText.FlyStockTip,
            Act: () => BurnModeInput.FlyWithAuto(craft), Owner: rcs ? Owner.Afc : Owner.Stock,
            Secondary: delete, Quiet: pass, Caution: fuelFact);
    }

    private static void DrawPicture(in Picked p)
    {
        PictureFrame probe = Picture.Probe();
        float head = Picture.HeadHeight(in probe, "");
        float laneH = p.Active ? 0f : Picture.TextLaneHeight();
        int barRows = p.Active ? 1 : 3;
        using (PictureScope pic = Picture.Open(Picture.FrameHeight(head + barRows * 24f * probe.Scale + 4f * probe.Scale + laneH)))
        {
            PictureFrame f = pic.Frame;
            float s = f.Scale;
            PictureLanes lanes = Picture.Lanes(in f, head, 0f, 0f, laneH, Picture.StripPlot);
            RcsEstimates est = p.Est;
            double hold = p.Estimates && est.HoldFeasible ? est.HoldPropellantKg : double.NaN;
            double align = p.Estimates && est.AlignFeasible ? est.AlignTotalPropellantKg : double.NaN;
            double max = Math.Max(1.0, Math.Max(p.Available, Math.Max(double.IsFinite(hold) ? hold : 0.0, double.IsFinite(align) ? align : 0.0)));
            ConsoleUi.PushSmall();
            try
            {
                float left = f.Min.X + 4f * s + ImGui.CalcTextSize(PageText.RcsOnBoard).X + 8f * s, right = f.Max.X - 10f * s;
                float y = lanes.Plot.Y + 2f * s;
                float rowH = 24f * s;
                Bar(in f, PageText.RcsOnBoard, p.Available, max, left, right, y, Palette.Dim, Fmt.Kg(p.Available));
                if (!p.Active)
                {
                    Bar(in f, "HOLD", hold, max, left, right, y + rowH, hold > p.Available ? Palette.Warn : Palette.Plan, EstimateText(hold, est.HoldDurationSec, p.Available));
                    Bar(in f, "ALIGN", align, max, left, right, y + rowH * 2f, align > p.Available ? Palette.Warn : Palette.Plan, EstimateText(align, est.AlignDurationSec, p.Available));
                }
            }
            finally
            {
                ConsoleStyle.PopFont();
            }
            if (!p.Active)
                Picture.LaneText(in f, lanes.Bottom, p.Estimates
                    ? PageText.EstimateBasis(p.IsNext && RcsBurnUi.HasEstimatesFor(p.Node, p.Vehicle.FlightComputer.Burn, p.Exec))
                    : "NO ESTIMATE", Palette.Dim.Solid);
        }
    }

    private static string EstimateText(double kg, double seconds, double available)
    {
        if (!double.IsFinite(kg))
            return "NOT POSSIBLE";
        string text = string.Format(Inv, "~{0:F0} kg, ~{1}", kg, FormatHelper.FormatDuration(seconds));
        return kg > available ? text + ". " + PageText.RcsNotEnough : text;
    }

    private static void Bar(in PictureFrame f, string label, double kg, double max, float left, float right, float y, Rgb colour, string text)
    {
        float s = f.Scale;
        f.Dl.AddText(new float2(f.Min.X + 4f * s, y), Palette.Dim.Solid, label);
        float top = y + 17f * s, bottom = y + 20f * s;
        f.Dl.AddRectFilled(new float2(left, top), new float2(right, bottom), Palette.Grid.A(0.35f), 1f * s);
        if (double.IsFinite(kg) && kg > 0.0)
            f.Dl.AddRectFilled(new float2(left, top), new float2(left + (float)Math.Min(kg / max, 1.0) * (right - left), bottom), colour.A(0.85f), 1f * s);
        f.Dl.AddText(new float2(left, y), Palette.Text.Solid, ConsoleUi.FitEllipsis(text, right - left));
    }

    private static void DrawAttitudeRow(Vehicle vehicle, Burn node, bool active, string note)
        => BurnOptionRow(vehicle, node, active, "ATTITUDE", "AfcRcsAttitude", PageText.AttitudeLabels, PageText.AttitudeCards, note, PageText.AttitudeTip,
            static o => Array.IndexOf(AttitudeOrder, o?.Attitude ?? RcsAttitudeStrategy.Auto), static (o, i) => o.Attitude = AttitudeOrder[i]);

    private static void DrawAllocatorRow(Vehicle vehicle, Burn node, bool active)
        => BurnOptionRow(vehicle, node, active, FoldNames.Allocator, "AfcRcsAllocator", PageText.AllocatorLabels, PageText.AllocatorCards, "", PageText.AllocatorTip,
            static o => o?.Allocator == RcsAllocator.Lp ? 1 : 0, static (o, i) => o.Allocator = i == 1 ? RcsAllocator.Lp : RcsAllocator.Groups);

    // Read from the options of the burn each frame, so a change holds for this burn only.
    private static void BurnOptionRow(Vehicle vehicle, Burn node, bool disabled, string label, string id, string[] labels, string[] cards, string note, string tip,
        Func<RcsBurnOptions?, int> current, Action<RcsBurnOptions, int> write)
    {
        double timeSec = node.Time.Seconds();
        double dvMs = node.DeltaVVlf.Length();
        RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec);
        using (new ImGuiDisabledScope(disabled))
        {
            int picked = ConsoleUi.ChoiceRow(label, id, labels, current(exec?.FindOptions(timeSec, dvMs)), default, cards, note, tip);
            if (!disabled && picked >= 0)
                write(RcsExecRegistry.GetOrCreate(vehicle.Id).GetOrCreateOptions(timeSec, dvMs), picked);
        }
    }
}
