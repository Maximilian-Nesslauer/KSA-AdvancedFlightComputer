using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.Features.Transfer;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

internal static class ConsoleHead
{
    private const double NoteSeconds = 8.0;
    private const double CueSeconds = 20.0;

    private static StatusLine _cue;
    private static DateTime _cueUntil;
    private static string _note = "";
    private static DateTime _noteUntil;
    private static bool _prevAscentRunning;
    private static bool _landingAirborne;
    private static bool _subjectCuePending;
    private static string _burnEndCraft = "";
    private static Burn? _burnEndBurn;
    private static bool _burnEndPasses;
    private static bool _burnEndTransfer;
    private static readonly List<StatusLine> _newCues = new();
    private static readonly LineAges _ages = new();
    private static readonly FailureAcks _acks = new();
    private static readonly List<CraftFailure> _failures = new();
    private static readonly HashSet<string> _acknowledgedLines = new(StringComparer.Ordinal);

    /// <summary>The fixed row height of the head, with the widget style pushed.</summary>
    public static float RowHeight => ConsoleStyle.FrameHeightPx + ImGui.GetStyle().ItemSpacing.Y;

    private static bool WarpShown(ConsoleFrame f) => f.Subject != null && f.GuidanceOn && GuidanceWindow.HeadWarpPending;

    private static CommitAction? PassSlot(ConsoleFrame f)
        => f.Subject != null && f.Activity.MultiPassActive && !WarpShown(f) ? Pages.PageKit.PassToFly(f.Subject) : null;

    // The wait for a burn that stock AUTO flies.
    private static bool BurnWarpShown(ConsoleFrame f)
        => f.Subject != null && f.Activity.StockAutoOnNode && !f.Activity.LandingStockAuto && !WarpShown(f) && BurnModeInput.WarpOffered(f.Subject);

    private static bool SlotShown(ConsoleFrame f) => WarpShown(f) || BurnWarpShown(f) || PassSlot(f) != null;

    /// <summary>The pointer stands on the control of the drawer in this frame, so a click there is no click outside the drawer.</summary>
    public static bool MoreHovered { get; private set; }

    private static string MayOff(ConsoleFrame f) => f.GuidanceOn ? GuidanceWindow.MayOffLine(f.Subject) : "";

    private static int MessageIndex(ConsoleFrame f)
        => f.Subject == null ? -1 : StatusSelector.PickRow(f.Lines, !f.Idle, f.GuidanceOn ? GuidanceWindow.RefusedLine : "");

    private static HeadRowHolds IdleHolds(ConsoleFrame f)
        => StatusSelector.IdleRow(f.FailureWhat.Length > 0, SituationText.IsFalling(f.Situation.Where), MessageIndex(f) >= 0);

    private static bool LineOnRow(ConsoleFrame f)
        => f.Idle ? IdleHolds(f) != HeadRowHolds.Situation : MessageIndex(f) >= 0;

    // The node that stock AUTO flies has its burn and its time.
    private static bool NumbersShown(ConsoleFrame f)
    {
        ModeReadout mode = f.Mode;
        if (mode.ModeName.Length > 0)
            return mode.Numbers?.Count > 0 || TgoText(in mode).Length > 0;
        return f.Activity.StockAutoOnNode && BurnModeInput.NextNode(f.Subject!) != null;
    }

    private static string TgoText(in ModeReadout mode)
        => double.IsFinite(mode.TgoSeconds) && mode.TgoLabel.Length > 0
            ? mode.TgoLabel + " " + FormatHelper.FormatDuration(Math.Max(0.0, mode.TgoSeconds))
            : "";

    // A burn that stock AUTO flies says its step beside its name too, and a hover has no step to name.
    private static bool PhaseRowShown(ConsoleFrame f)
        => !f.Mode.PhaseBesideName && f.Mode.ModeName.Length > 0 && f.Mode.Phase.Length > 0;

    private static int FlyingRows(ConsoleFrame f) => 2 + (PhaseRowShown(f) ? 1 : 0) + (NumbersShown(f) ? 1 : 0);

    // In the strip only the fall row holds it, and that row draws only while nothing runs, so a burn of stock AUTO on a falling craft keeps the slim row with its link to the console.
    private static bool ControlInRow(ConsoleFrame f, bool inStrip)
        => inStrip ? f.Idle && f.FallAfterFailure : f.Idle || (f.Subject == null && !f.OtherStop.Enabled);

    private static string OtherCraftLine(ConsoleFrame f)
        => f.OtherStopCraft.Length > 0 ? "AFC flies " + f.OtherStopCraft + ". Its stop is above."
            : HeadDrawer.OtherCraftCount > 0 ? "AFC flies other craft. Their stops are under More."
            : "";

    public static int Rows(ConsoleFrame f, bool inStrip)
    {
        int rows;
        if (f.Subject == null)
            rows = (f.OtherStop.Enabled ? 2 : 1) + (OtherCraftLine(f).Length > 0 ? 1 : 0);
        else
        {
            rows = f.Idle ? 1 : FlyingRows(f);
            if (MayOff(f).Length > 0)
                rows++;
            if (f.Idle && SlotShown(f))
                rows++;
        }
        return rows;
    }

    public static float Height(ConsoleFrame f, bool inStrip, bool firstOpenNotice, float width)
        => Rows(f, inStrip) * RowHeight + (firstOpenNotice ? NoticeHeight(width) : 0f) + (ControlInRow(f, inStrip) ? 0f : FoldHeight);

    #region Status lines

    public static void OnSubjectChanged(Vehicle? subject)
    {
        _cue = default;
        _prevAscentRunning = false;
        _landingAirborne = false;
        _subjectCuePending = subject != null;
        ClearBurnEnd();
        _ages.Clear();
    }

    /// <summary>A planned burn of the controlled craft delivered its delta-v, from the stock end of a burn.</summary>
    public static void OnBurnCompleted(Vehicle vehicle, Burn? burn)
    {
        if (!ReferenceEquals(vehicle, Program.ControlledVehicle))
            return;
        _burnEndCraft = vehicle.Id;
        _burnEndBurn = burn;
        _burnEndPasses = MultiPassRegistry.TryGet(vehicle.Id, out MultiPassExecution? exec);
        _burnEndTransfer = _burnEndPasses ? !OrbitToolCatalog.IsTool(exec!.Intent.TypeKey) : ReferenceEquals(burn, TransferSession.TransferBurn);
    }

    private static void ClearBurnEnd()
    {
        _burnEndCraft = "";
        _burnEndBurn = null;
        _burnEndPasses = false;
        _burnEndTransfer = false;
    }

    private static void RaiseBurnCue(ConsoleFrame f, Vehicle subject)
    {
        if (_burnEndCraft != subject.Id)
            return;
        if (f.Activity.AnyGuidanceLive)
        {
            ClearBurnEnd();
            return;
        }
        ConsolePage page = _burnEndTransfer ? ConsolePage.Transfer : ConsolePage.Orbit;
        UniverseTime now = Universe.GetElapsedTime();
        if (MultiPassRegistry.TryGet(subject.Id, out MultiPassExecution? exec))
        {
            if (exec.CurrentBurn == null)
                return;
            RaiseCue(NextStep.AfterPass(exec.PassIndex, exec.PassIndex + 1, FormatHelper.FormatDuration(Math.Max(0.0, (exec.CurrentBurn.Time - now).Seconds())), page));
        }
        else if (_burnEndPasses)
            RaiseCue(NextStep.AfterPasses(in f.Situation, page));
        else
        {
            Burn? next = BurnModeInput.NextNode(subject);
            bool nextPlanned = next != null && !ReferenceEquals(next, _burnEndBurn) && next.Time > now;
            RaiseCue(NextStep.AfterBurn(in f.Situation, nextPlanned ? FormatHelper.FormatDuration((next!.Time - now).Seconds()) : "", page));
        }
        ClearBurnEnd();
    }

    private static void RaiseLandingCue(ConsoleFrame f)
    {
        CraftSituation where = f.Situation.Where;
        if (f.Activity.LandingFlies)
        {
            if (where != CraftSituation.Ground && where != CraftSituation.Unknown)
                _landingAirborne = true;
            return;
        }
        if (_landingAirborne && where == CraftSituation.Ground)
            RaiseCue(NextStep.AfterLanding(f.Situation.Body));
        _landingAirborne = false;
    }

    // The page draws after the head, so the row reads what the last frame said.
    private static bool _cueInFooter;
    private static bool _cueInFooterNow;

    public static StatusLine? TakeCueFor(ConsolePage page)
    {
        if (string.IsNullOrEmpty(_cue.Text) || !ConsolePages.TryParseCue(_cue.CueTarget, out ConsolePage cued, out _) || cued != page)
            return null;
        _cueInFooterNow = true;
        return _cue;
    }

    /// <summary>Every status line of the subject, the cues and the notes, in the order the selector breaks ties by.</summary>
    public static void BuildLines(ConsoleFrame f)
    {
        _cueInFooter = _cueInFooterNow;
        _cueInFooterNow = false;
        List<StatusLine> lines = f.Lines;
        Vehicle? subject = f.Subject;
        if (subject == null)
            return;

        if (f.GuidanceOn)
        {
            GuidanceWindow.BuildStatusLines(subject, lines);
            _newCues.Clear();
            GuidanceWindow.ConsumeCues(_newCues);
            if (_newCues.Count > 0)
                RaiseCue(_newCues[^1]);

            bool running = f.Activity.AscentRunning;
            if (running && !_prevAscentRunning)
                GuidanceWindow.ForgetFlownAscent();
            if (_prevAscentRunning && !running)
            {
                GuidanceWindow.NoteAscentEnded(in f.Situation);
                if (GuidanceWindow.AscentCutoffDone)
                    RaiseCue(NextStep.AfterAscent(subject.Orbit.Eccentricity, GuidanceWindow.CircularizeFitsFlown()));
            }
            _prevAscentRunning = running;
            RaiseLandingCue(f);

            if (_subjectCuePending && f.Activity.BoostbackLive)
                RaiseCue(new StatusLine(Severity.Cue, "Boostback flies this craft.",
                    ConsolePages.CueTarget(ConsolePage.Land, ConsolePages.EnumKey(LandWay.Boostback))));

            string commitNote = GuidanceWindow.PendingCommitNote;
            if (commitNote.Length > 0)
            {
                GuidanceWindow.PendingCommitNote = "";
                Note(commitNote);
            }

            if (GuidanceWindow.RetargetArmed)
                lines.Add(new StatusLine(Severity.Cue, PageText.PickSiteCue, ""));
        }
        _subjectCuePending = false;
        RaiseBurnCue(f, subject);

        if (FaultedRcs(subject) is RcsExecution faulted)
            lines.Add(new StatusLine(Severity.Error, RcsFaultLine(faulted), ConsolePages.Key(ConsolePage.Rcs)));

        if (f.Activity.MultiPassActive && MultiPassRegistry.TryGet(subject.Id, out MultiPassExecution? passes) && MultiPassUI.RunningShortMs(passes) > 0.5)
            lines.Add(new StatusLine(Severity.Warning, PageText.PassesShort(MultiPassUI.RunningShortMs(passes)), ""));

        if (PassSlot(f) is { Refusal.Length: > 0 } refusedPass)
            lines.Add(new StatusLine(Severity.Warning, refusedPass.Refusal, ""));

        foreach (string booster in HeadDrawer.NewBoosters())
            RaiseCue(new StatusLine(Severity.Cue, $"{booster} separated and flies itself home.", GuidanceWindow.DrawerTarget));

        if (!string.IsNullOrEmpty(_cue.Text))
        {
            bool shown = AfcConsole.IsOpen && CueShown(_cue.CueTarget);
            if (DateTime.UtcNow > _cueUntil || shown)
                _cue = default;
            else
                lines.Add(_cue);
        }

        if (_note.Length > 0)
        {
            if (DateTime.UtcNow > _noteUntil)
                _note = "";
            else
                lines.Add(new StatusLine(StatusSelector.CommitNote, _note, ""));
        }

        ReadFailures(f, subject);
        _ages.Filter(lines, f.Activity.AnyLive, _acknowledgedLines, DateTime.UtcNow.Ticks / (double)TimeSpan.TicksPerSecond);

        if (f.FailureWhat.Length > 0)
        {
            int top = StatusSelector.Pick(lines);
            if (top < 0 || lines[top].Severity < Severity.Error)
                lines.Add(new StatusLine(Severity.Error, f.FailureWhat + ".", ""));
        }
    }

    private static RcsExecution? FaultedRcs(Vehicle subject)
        => SharedVehicleHooks.RcsEnabled && RcsExecRegistry.TryGet(subject.Id, out RcsExecution? exec) && exec.Faulted ? exec : null;

    private static string RcsFaultLine(RcsExecution exec) => exec.CleanupPending
        ? "RCS stopped and its control cleanup failed. RETRY CLEANUP is on BURNS."
        : "RCS stopped after an internal error. See the log.";

    // The first failure the pilot has not acknowledged goes to the frame.
    private static void ReadFailures(ConsoleFrame f, Vehicle subject)
    {
        _failures.Clear();
        _acknowledgedLines.Clear();
        if (f.GuidanceOn)
            GuidanceWindow.ReadFailures(_failures);
        if (FaultedRcs(subject) is RcsExecution faulted)
            _failures.Add(new CraftFailure("The RCS burn stopped after an error",
                faulted.CleanupPending ? "Its control cleanup failed. RETRY CLEANUP is on BURNS." : "See the log.", RcsFaultLine(faulted)));

        _acks.Update(subject.Id, _failures);
        foreach (CraftFailure failure in _failures)
        {
            if (_acks.IsAcknowledged(subject.Id, failure.What, failure.Serial))
            {
                _acknowledgedLines.Add(failure.Line);
                f.FailureAnswered = true;
            }
            else if (f.FailureWhat.Length == 0)
            {
                f.FailureWhat = failure.What;
                f.FailureDetail = failure.Detail;
                f.FailureSerial = failure.Serial;
            }
        }
    }

    public static void AcknowledgeFailure(ConsoleFrame f)
    {
        if (f.Subject != null)
            _acks.Acknowledge(f.Subject.Id, f.FailureWhat, f.FailureSerial);
        f.FailureWhat = "";
        f.FailureDetail = "";
        f.FailureSerial = 0;
        f.FailureAnswered = true;
    }

    private static void RaiseCue(StatusLine cue)
    {
        _cue = cue;
        _cueUntil = DateTime.UtcNow.AddSeconds(CueSeconds);
    }

    /// <summary>The page on screen has a lit verb in its footer, as the last frame drew it.</summary>
    public static bool PageVerbLit;

    /// <summary>A commit drops the cue it answers. A cue the commit raises arrives on the next frame.</summary>
    public static void DropCue() => _cue = default;

    private static void Note(string text)
    {
        _note = text;
        _noteUntil = DateTime.UtcNow.AddSeconds(NoteSeconds);
    }

    private static bool CueShown(string cueTarget)
    {
        if (cueTarget == GuidanceWindow.DrawerTarget)
            return AfcConsole.DrawerOpen;
        if (!ConsolePages.TryParseCue(cueTarget, out ConsolePage page, out string sub) || page != AfcConsole.Page)
            return false;
        if (sub.Length == 0)
            return true;
        if (page == ConsolePage.Orbit)
            return OrbitToolCatalog.ToolIndex(OrbitToolCatalog.KeyOfCue(sub)) == OrbitToolCatalog.ToolIndex(OrbitToolsPanel.ToolKey);
        return page != ConsolePage.Land || ConsolePages.EnumKey(AfcConsole.LandWay) == sub;
    }

    public static void Reset()
    {
        OnSubjectChanged(null);
        _cueInFooter = false;
        _cueInFooterNow = false;
        PageVerbLit = false;
        _note = "";
        _newCues.Clear();
        _acks.Clear();
        _failures.Clear();
        _acknowledgedLines.Clear();
    }

    #endregion

    #region Draw

    public static void Draw(ConsoleFrame f, bool inStrip, bool firstOpenNotice)
    {
        float2 origin = ImGui.GetCursorScreenPos();
        float width = ConsoleStyle.ContentAvailWidth();
        float rowH = RowHeight;
        MoreHovered = false;

        int row;
        if (f.Subject == null)
            row = DrawNoCraft(f, origin, width, rowH);
        else
        {
            if (f.Idle)
            {
                if (inStrip && f.FallAfterFailure)
                    DrawFallRow(f, origin, width);
                else
                    DrawSituationRow(f, origin, width, inStrip);
                row = 1;
            }
            else
            {
                DrawModeRows(f, origin, width, rowH, inStrip);
                row = 2;
                if (PhaseRowShown(f))
                    DrawPhaseRow(f, origin, width, rowH, row++);
                if (NumbersShown(f))
                    DrawNumbersRow(f, origin, width, rowH, row++);
            }

            if (f.Idle && SlotShown(f))
            {
                Row(origin, rowH, row++);
                DrawSlot(f, ImGui.GetCursorScreenPos(), width, inStrip);
            }

            string mayOff = MayOff(f);
            if (mayOff.Length > 0)
                DrawMayOffRow(mayOff, origin, width, rowH, row++, inStrip);
        }
        float y = origin.Y + row * rowH;
        if (firstOpenNotice)
        {
            DrawFirstOpenNotice(new float2(origin.X, y), width);
            y += NoticeHeight(width);
        }
        if (!ControlInRow(f, inStrip))
            DrawFold(f, new float2(origin.X, y), width, inStrip);
        // A message row draws only through the draw list, and the warp prompt can expire in this frame and draw nothing.
        ImGui.Dummy(float2.Zero);
    }

    private static void Row(float2 origin, float rowH, int row)
        => ImGui.SetCursorScreenPos(new float2(origin.X, origin.Y + row * rowH));

    // Text in the value font at the left of a row, centred on the frame height.
    private static void RowText(float2 at, float room, string text, uint colour)
    {
        ConsoleStyle.PushValueFont();
        ConsoleUi.FitText(at, room, ConsoleStyle.FrameHeightPx, text, ConsoleStyle.ApplyAlpha(colour));
        ConsoleStyle.PopFont();
    }

    private static void DrawModeRows(ConsoleFrame f, float2 origin, float width, float rowH, bool inStrip)
    {
        ModeReadout mode = f.Mode;
        float gap = ImGui.GetStyle().ItemSpacing.X;
        string who = f.WhoFlies;
        float chipW = ConsoleUi.OwnerChipAt(ImGui.GetWindowDrawList(), origin, ConsoleStyle.FrameHeightPx, PageText.HeadOwner(who), who);
        var chipSize = new float2(chipW, ConsoleStyle.FrameHeightPx);
        ConsoleUi.NoteRow("AfcHeadChip", origin, chipSize);
        if (ConsoleUi.Hovering(origin, origin + chipSize))
            ConsoleUi.Card(who, PageText.HeadOwnerCard(who, f.Activity.PilotHolds));

        string name = "", method = "";
        uint colour = ConsoleStyle.TextMutedU32;
        if (mode.ModeName.Length > 0)
        {
            name = mode.PhaseBesideName ? mode.ModeName + ", " + mode.Phase : mode.ModeName;
            method = mode.Method;
            colour = mode.Tone switch
            {
                ModeTone.Flying or ModeTone.Waiting when who == FlownBy.You => ConsoleStyle.TextPrimaryU32,
                ModeTone.Flying or ModeTone.Waiting => Palette.Afc.U32,
                ModeTone.Error => ConsoleStyle.DangerU32,
                _ => ConsoleStyle.TextMutedU32,
            };
        }
        else if (f.Activity.StockAutoOnNode)
        {
            name = PageText.StockBurnName(StockBurnLit(f.Subject!));
            colour = Palette.Stock.U32;
        }

        StopSpec stop = f.Stop;
        float stopW = stop.Enabled ? StopWidth() : 0f;
        float room = width - chipW - gap - stopW - gap;
        string shownName = Ellipsis(name, room);
        float2 nameAt = origin + new float2(chipW + gap, 0f);
        RowText(nameAt, room, shownName, colour);

        CommitAction? takeOver = TakeOverOffered(f);
        bool slot = takeOver == null && SlotShown(f);
        float leftW = takeOver != null ? ConsoleWidgets.ButtonWidth(takeOver.Verb.AsSpan()) + gap : slot ? SlotWidth(f) + gap : 0f;
        float outcomeRoom = DrawOutcomeRow(stop.Outcome, origin, width, rowH, leftW);
        var left = new float2(origin.X, origin.Y + rowH);
        if (takeOver != null)
            DrawTakeOver(takeOver, left);
        else if (slot)
            DrawSlot(f, left, outcomeRoom - gap, inStrip);
        else
        {
            PlanWord word = mode.Word;
            if (word.Shows && ConsoleUi.ValueWidth(word.Text) + gap <= outcomeRoom)
                DrawWord(left, outcomeRoom - gap, in word);
        }

        if ((shownName != name || method.Length > 0) && ConsoleUi.Hovering(nameAt, nameAt + new float2(room, ConsoleStyle.FrameHeightPx)))
            ConsoleUi.Tooltip((method.Length > 0 ? name + ", " + method : name).AsSpan());

        if (stop.Enabled)
            DrawStop(stop, new float2(origin.X + width - stopW, origin.Y), stopW, "AfcHeadStop");
    }

    // Stock AUTO lights the engine half a burn before the time of the node (FlightComputer.TimeBurn) and holds it off while the craft is not aligned (FlightComputer.ComputeControl), so the engine and not the clock says whether the burn flies.
    private static bool StockBurnLit(Vehicle subject) => !ConsoleFrame.EngineKnownOff(subject);

    private static void DrawWord(float2 at, float room, in PlanWord word)
        => RowText(at, room, word.Text, word.Warns ? Palette.Warn.U32 : word.Good ? Palette.Good.U32 : ConsoleStyle.TextLabelU32);

    private static float SlotWidth(ConsoleFrame f)
    {
        float gap = ImGui.GetStyle().ItemSpacing.X;
        float chip = ConsoleUi.OwnerChipWidth(Owner.Stock, "STOCK") + ConsoleUi.ChipToButtonGap;
        if (WarpShown(f))
            return GuidanceWindow.HeadWarpWidth();
        if (BurnWarpShown(f))
            return chip + ConsoleWidgets.ButtonWidth((BurnModeInput.WarpingToBurn(f.Subject!) ? "STOP WARP" : "WARP").AsSpan());
        return PassSlot(f) is CommitAction pass ? (BurnModeInput.ResolvesToRcs(f.Subject!) ? 0f : chip) + ConsoleWidgets.ButtonWidth(pass.Verb.AsSpan()) : 0f;
    }

    private static void DrawSlot(ConsoleFrame f, float2 at, float room, bool inStrip)
    {
        ImGui.SetCursorScreenPos(at);
        bool lit = inStrip || !PageVerbLit;
        if (WarpShown(f))
            GuidanceWindow.DrawHeadWarp(f.Subject!, room, lit);
        else if (BurnWarpShown(f))
            DrawBurnWarpSlot(f.Subject!, room, lit);
        else if (PassSlot(f) is CommitAction pass)
            DrawPassSlot(pass, f.Subject!, room);
    }

    private static CommitAction? TakeOverOffered(ConsoleFrame f)
        => f.GuidanceOn && f.Subject != null && f.Stop.Target == StopTarget.Registered && f.Activity.RegisteredModeLive == GuidanceModeId.EntryHere
            ? GuidanceWindow.EntryTakeOver(f.Subject)
            : null;

    private static void DrawTakeOver(CommitAction takeOver, float2 at)
    {
        ImGui.SetCursorScreenPos(at);
        if (ConsoleWidgets.PrimaryButton(takeOver.Verb.AsSpan(), "AfcHeadTakeOver".AsSpan(), default))
            takeOver.Act!();
        ConsoleUi.NoteItem("AfcHeadTakeOver");
        if (ImGui.IsItemHovered())
            ConsoleUi.Card(takeOver.Verb, takeOver.Tooltip);
    }

    private static float DrawOutcomeRow(string outcome, float2 origin, float width, float rowH, float reserve = 0f)
    {
        float2 at = new float2(origin.X, origin.Y + rowH);
        string shown = Ellipsis(outcome, width - reserve);
        float shownW = ConsoleUi.ValueWidth(shown);
        RowText(new float2(at.X + width - shownW, at.Y), shownW, shown, ConsoleStyle.TextLabelU32);
        if (shown != outcome && ConsoleUi.Hovering(at, at + new float2(width, ConsoleStyle.FrameHeightPx)))
            ConsoleUi.Tooltip(outcome.AsSpan());
        return width - shownW;
    }

    private static float StopWidth()
    {
        float widest = 0f;
        foreach (string label in StopResolver.BuiltInLabels)
            widest = Math.Max(widest, ConsoleWidgets.ButtonWidth(label.AsSpan()));
        foreach (GuidanceModeDescriptor mode in GuidanceModes.ByStopOrder)
        {
            if (mode.StopLabel.Length > 0)
                widest = Math.Max(widest, ConsoleWidgets.ButtonWidth(mode.StopLabel.AsSpan()));
        }
        return widest;
    }

    // The one row of the head while nothing runs: the failure the pilot has not answered, or the situation, or a message for its time, by StatusSelector.IdleRow.
    private static void DrawSituationRow(ConsoleFrame f, float2 origin, float width, bool inStrip)
    {
        float controlW = inStrip ? 0f : MoreWidth(f, inStrip: false);
        float gap = ImGui.GetStyle().ItemSpacing.X;
        float room = width - controlW - gap;
        float h = ConsoleStyle.FrameHeightPx;
        ConsoleUi.NoteRow("AfcHeadSituation", origin, new float2(room, h));

        string line0 = SituationText.Line(in f.Situation);
        string card = line0;
        string shown;
        uint ink;
        bool moreCount = true;
        bool warnImpact = false;
        switch (IdleHolds(f))
        {
            case HeadRowHolds.Failure:
                BandFacts facts = f.FailureFacts();
                ConsoleStyle.PushValueFont();
                shown = FailureBand.RowLine(in facts, MeasureCurrentFont, room);
                ConsoleStyle.PopFont();
                ink = ConsoleStyle.DangerU32;
                card = FailureBand.Text(in facts) + (f.FailureDetail.Length > 0 ? "\n" + Fmt.Sentence(f.FailureDetail) : "") + "\n" + card;
                break;
            case HeadRowHolds.Message:
                StatusLine line = f.Lines[MessageIndex(f)];
                string link = LinkOf(in line);
                if (link.Length > 0)
                {
                    float goW = ConsoleWidgets.ButtonWidth(link.AsSpan());
                    room -= goW + gap;
                    ImGui.SetCursorScreenPos(new float2(origin.X + room + gap, origin.Y));
                    if (ConsoleWidgets.Button(link.AsSpan(), "AfcHeadCueGo".AsSpan(), default))
                        AfcConsole.Navigate(line.CueTarget);
                }
                ConsoleStyle.PushValueFont();
                string message = LineText(f, in line);
                shown = SituationText.MessageLine(message, MeasureCurrentFont, room);
                if (!inStrip && shown != message + " " + SituationText.YouFly)
                {
                    float wider = room + controlW - MoreWidth(f, inStrip: false, count: false);
                    TryWider(SituationText.MessageLine(message, MeasureCurrentFont, wider), wider);
                }
                ConsoleStyle.PopFont();
                ink = LineInk(line.Severity);
                card = Fmt.Sentence(line.Text) + "\n" + card;
                break;
            default:
                ConsoleStyle.PushValueFont();
                shown = SituationText.LineThatFits(in f.Situation, MeasureCurrentFont, room);
                if (!inStrip && SituationText.IsFalling(f.Situation.Where))
                {
                    float wider = width - MoreWidth(f, inStrip: false, count: false) - gap;
                    TryWider(SituationText.LineThatFits(in f.Situation, MeasureCurrentFont, wider), wider);
                }
                ConsoleStyle.PopFont();
                ink = ConsoleStyle.TextPrimaryU32;
                warnImpact = true;
                // The row says it all, so the card adds only what stock has planned.
                if (shown == line0)
                    card = "";
                break;
        }
        string row = Ellipsis(shown, room);
        (int impactAt, int impactLength) = warnImpact ? SituationText.ImpactWords(row) : (-1, 0);
        if (impactAt < 0)
            RowText(origin, room, row, ink);
        else
        {
            float x = origin.X;
            string before = row.Substring(0, impactAt), words = row.Substring(impactAt, impactLength);
            RowText(new float2(x, origin.Y), room, before, ink);
            x += ConsoleUi.ValueWidth(before);
            RowText(new float2(x, origin.Y), room, words, Palette.Warn.U32);
            x += ConsoleUi.ValueWidth(words);
            RowText(new float2(x, origin.Y), room, row.Substring(impactAt + impactLength), ink);
        }

        if (ConsoleUi.Hovering(origin, new float2(origin.X + room, origin.Y + h)))
        {
            Burn? node = f.Subject != null ? BurnModeInput.NextNode(f.Subject) : null;
            double toBurn = node != null ? Math.Max(0.0, (node.Time - Universe.GetElapsedTime()).Seconds()) : 0.0;
            string stock = PageText.StockLine(node != null, FormatHelper.FormatDuration(toBurn), node != null ? node.DeltaVVlf.Length() : 0.0);
            ConsoleUi.Tooltip((card.Length > 0 ? card + "\n" + stock : stock).AsSpan());
        }
        if (!inStrip)
        {
            float moreW = MoreWidth(f, inStrip: false, moreCount);
            DrawMore(f, new float2(origin.X + width - moreW, origin.Y), ConsoleStyle.FrameHeightPx, moreW, inStrip: false, moreCount);
        }
        ImGui.SetCursorScreenPos(origin);
        ImGui.Dummy(new float2(width, ConsoleStyle.FrameHeightPx));

        // The row takes the room of the More count when the line reads better in it.
        void TryWider(string better, float wider)
        {
            if (better == shown)
                return;
            shown = better;
            room = wider;
            moreCount = false;
        }
    }

    private static void DrawFallRow(ConsoleFrame f, float2 origin, float width)
    {
        const string openAfc = "Open AFC >";
        float openW = ConsoleUi.LabelWidth(openAfc);
        float room = width - openW - ImGui.GetStyle().ItemSpacing.X;
        string who = SituationText.YouFly + " ";
        RowText(origin, room, who, ConsoleStyle.TextPrimaryU32);
        float whoW = ConsoleUi.ValueWidth(who);
        RowText(new float2(origin.X + whoW, origin.Y), room - whoW, FailureBand.ToImpact(f.Situation.ImpactS), Palette.Warn.U32);

        bool go = ConsoleUi.TextButton("AfcStripOpen", openAfc, new float2(origin.X + width - openW, origin.Y), ConsoleStyle.FrameHeightPx,
            ConsoleStyle.TextPrimaryU32, ConsoleStyle.TextPrimaryU32, out bool hovered, noteItem: true);
        if (hovered)
            ConsoleUi.Tooltip(PageText.OpenLandNowTip.AsSpan());
        if (go)
            AfcConsole.Navigate(ConsolePages.CueTarget(ConsolePage.Land, ConsolePages.EnumKey(LandWay.FromHere)));
    }

    private static void DrawMayOffRow(string text, float2 origin, float width, float rowH, int row, bool inStrip)
    {
        Row(origin, rowH, row);
        float2 at = ImGui.GetCursorScreenPos();
        float h = ConsoleStyle.FrameHeightPx;
        string shown = Ellipsis(text, width);
        bool go = ImGui.InvisibleButton("AfcHeadMayOff", new float2(Math.Max(1f, width), h));
        bool hovered = ImGui.IsItemHovered();
        RowText(at, width, shown, Palette.Warn.U32);
        if (hovered)
        {
            ConsoleStyle.PushValueFont();
            float2 textSize = ImGui.CalcTextSize(shown);
            ConsoleStyle.PopFont();
            float baseline = at.Y + (h + textSize.Y) * 0.5f;
            ImGui.GetWindowDrawList().AddLine(new float2(at.X, baseline), new float2(at.X + textSize.X, baseline), ConsoleStyle.ApplyAlpha(Palette.Warn.U32), 1f);
            ConsoleUi.Tooltip(PageText.MayOffTip.AsSpan());
        }
        if (go)
        {
            if (inStrip)
                AfcConsole.Open();
            AfcConsole.DrawerOpen = true;
        }
    }

    private static void DrawStop(StopSpec stop, float2 at, float w, string id)
    {
        ImGui.SetCursorScreenPos(at);
        bool clicked = ConsoleUi.StopButton(stop.Label, id, new float2(w, ConsoleStyle.FrameHeightPx));
        ConsoleUi.NoteItem(id);
        if (ImGui.IsItemHovered() && stop.Detail.Length > 0)
            ConsoleUi.Card(stop.Label, stop.Detail);
        if (clicked)
            stop.Act!();
    }

    // The press that flies a pass: a stock AUTO burn after the STOCK chip, in the place of the warp offer.
    private static void DrawPassSlot(CommitAction pass, Vehicle subject, float width)
    {
        float2 start = SlotChip(!BurnModeInput.ResolvesToRcs(subject));
        bool clicked;
        using (new ImGuiDisabledScope(!pass.Enabled))
            clicked = pass.Enabled
                ? ConsoleWidgets.PrimaryButton(pass.Verb.AsSpan(), "AfcHeadFlyPass".AsSpan(), default)
                : ConsoleWidgets.Button(pass.Verb.AsSpan(), "AfcHeadFlyPass".AsSpan(), default);
        ConsoleUi.NoteItem("AfcHeadFlyPass");
        if (ImGui.IsItemHovered(ImGuiHoveredFlags.AllowWhenDisabled))
            ConsoleUi.Card(pass.Verb, pass.Refusal.Length > 0 ? pass.Refusal + "\n\n" + pass.Tooltip : pass.Tooltip);
        if (clicked && pass.Enabled)
            pass.Act!();
        SlotTail(start, width, pass.Refusal.Length == 0 ? PageText.PassWaits(BurnModeInput.ResolvesToRcs(subject)) : "");
    }

    private static void DrawBurnWarpSlot(Vehicle subject, float width, bool lit)
    {
        float2 start = SlotChip(true);
        bool warping = BurnModeInput.WarpingToBurn(subject);
        string verb = warping ? "STOP WARP" : "WARP";
        bool clicked = lit && !warping
            ? ConsoleWidgets.PrimaryButton(verb.AsSpan(), "AfcHeadBurnWarp".AsSpan(), default)
            : ConsoleWidgets.Button(verb.AsSpan(), "AfcHeadBurnWarp".AsSpan(), default);
        if (clicked)
            BurnModeInput.ToggleWarpToBurn(subject);
        ConsoleUi.NoteItem("AfcHeadBurnWarp");
        if (ImGui.IsItemHovered())
            ConsoleUi.Tooltip(PageText.BurnWarpTip.AsSpan());
        SlotTail(start, width, PageText.BurnWarpLine);
    }

    private static float2 SlotChip(bool chip)
    {
        float2 start = ImGui.GetCursorScreenPos();
        if (chip)
        {
            ConsoleUi.OwnerChip(Owner.Stock, "STOCK");
            ImGui.SameLine(0f, ConsoleUi.ChipToButtonGap);
        }
        return start;
    }

    private static void SlotTail(float2 start, float width, string line)
    {
        ImGui.SameLine();
        float2 at = ImGui.GetCursorScreenPos();
        float room = start.X + width - at.X;
        if (line.Length > 0 && ConsoleUi.ValueWidth(line) <= room)
            RowText(at, room, line, ConsoleStyle.TextLabelU32);
    }

    // The numbers of what flies with its time to go at the right, or the next node while stock AUTO flies it.
    private static void DrawNumbersRow(ConsoleFrame f, float2 origin, float width, float rowH, int row)
    {
        Row(origin, rowH, row);
        float2 at = ImGui.GetCursorScreenPos();
        ConsoleUi.NoteRow("AfcHeadNumbers", at, new float2(width, ConsoleStyle.FrameHeightPx));
        ModeReadout mode = f.Mode;
        Vehicle subject = f.Subject!;
        if (mode.ModeName.Length > 0)
        {
            string tgo = TgoText(in mode);
            float tgoW = ConsoleUi.ValueWidth(tgo);
            float heldW = tgoW > 0f ? Math.Max(tgoW, ConsoleUi.ValueWidth(mode.TgoLabel + " 00m 00s")) : 0f;
            float room = width - (heldW > 0f ? heldW + ImGui.GetStyle().ItemSpacing.X : 0f);
            ConsoleStyle.PushValueFont();
            string numbers = HeadNumbers.Fit(mode.Numbers, MeasureCurrentFont, room);
            List<HeadCell> cells = HeadNumbers.Cells(mode.Numbers, MeasureCurrentFont, room);
            PlanWord word = mode.Word;
            bool wordHere = word.Shows && SlotShown(f);
            float wordX = wordHere ? HeadNumbers.WordX(cells, word.Text, MeasureCurrentFont, room) : -1f;
            ConsoleStyle.PopFont();
            foreach (HeadCell cell in cells)
                RowText(new float2(at.X + cell.X, at.Y), room - cell.X, cell.Text, cell.Tone == PlanTone.Plain ? ConsoleStyle.TextLabelU32 : Palette.Tone(cell.Tone).U32);
            if (wordX >= 0f)
                DrawWord(new float2(at.X + wordX, at.Y), room - wordX, in word);
            RowText(new float2(at.X + width - tgoW, at.Y), tgoW, tgo, ConsoleStyle.TextPrimaryU32);
            if (ConsoleUi.Hovering(at, at + new float2(room, ConsoleStyle.FrameHeightPx)))
            {
                string card = HeadNumbers.Card(mode.Numbers, numbers, mode.NumbersCard);
                if (wordHere && wordX < 0f)
                    card = card.Length > 0 ? word.Text + "\n\n" + card : word.Text;
                if (card.Length > 0)
                    ConsoleUi.Tooltip(card.AsSpan());
            }
            return;
        }
        if (f.Activity.StockAutoOnNode && BurnModeInput.NextNode(subject) is Burn node)
        {
            string burn = "BURN " + Fmt.Dv(node.DeltaVVlf.Length());
            string when = PageText.StockBurnWhen(StockBurnLit(subject), FormatHelper.FormatDuration(Math.Max(0.0, BurnModeInput.SecondsToIgnition(subject, node))));
            float whenW = ConsoleUi.ValueWidth(when);
            RowText(at, width - whenW - 8f, burn, ConsoleStyle.TextLabelU32);
            RowText(new float2(at.X + width - whenW, at.Y), whenW, when, ConsoleStyle.TextPrimaryU32);
        }
    }

    // The stock burn of a deorbit has its STOCK chip here as on the LAND page.
    private static void DrawPhaseRow(ConsoleFrame f, float2 origin, float width, float rowH, int row)
    {
        Row(origin, rowH, row);
        float2 at = ImGui.GetCursorScreenPos();
        ConsoleUi.NoteRow("AfcHeadPhases", at, new float2(width, rowH));
        ModeReadout mode = f.Mode;
        IReadOnlyList<string> phases = mode.PhaseList ?? GuidanceModes.Live(f.Activity)?.Phases ?? Array.Empty<string>();
        string current = mode.Phase;

        float chipGap = 4f * ImGuiHelper.InterfaceScale;
        float chipW = ConsoleUi.OwnerChipWidth(Owner.Stock, "STOCK");
        ConsoleStyle.PushValueFont();
        List<LiveStep> steps = GuidanceModes.LiveChain(phases, current, MeasureCurrentFont, width, f.WhoFlies == FlownBy.You, chipW + chipGap);
        ConsoleStyle.PopFont();
        float x = 0f;
        for (int i = 0; i < steps.Count; i++)
        {
            LiveStep step = steps[i];
            if (i > 0)
            {
                string join = GuidanceModes.JoinBefore(steps, i);
                RowText(new float2(at.X + x, at.Y), width - x, join, ConsoleStyle.TextMutedU32);
                x += ConsoleUi.ValueWidth(join);
            }
            RowText(new float2(at.X + x, at.Y), width - x, step.Text, StepInk(in step));
            x += ConsoleUi.ValueWidth(step.Text);
            if (GuidanceModes.HasStockChip(in step))
            {
                ConsoleUi.OwnerChipAt(ImGui.GetWindowDrawList(), new float2(at.X + x + chipGap, at.Y), ConsoleStyle.FrameHeightPx, Owner.Stock, "STOCK");
                x += chipW + chipGap;
            }
        }

        int index = GuidanceModes.PhaseIndex(phases, current);
        if (phases.Count > 0 && ConsoleUi.Hovering(at, at + new float2(width, rowH))
            && ConsoleUi.BeginPlacedCard("PHASES", "PHASES".AsSpan()))
        {
            if (index >= 0)
                ImGui.TextColored(ConsoleStyle.TextMuted, ConsoleUi.EscapeFormat(PageText.StepOf(index + 1, phases.Count)));
            for (int i = 0; i < phases.Count; i++)
            {
                bool isNow = i == index;
                ImGui.TextColored(isNow ? Palette.Afc.F4 : ConsoleStyle.TextLabel, ConsoleUi.EscapeFormat(isNow ? "> " + current : "  " + phases[i]));
            }
            ConsoleWidgets.EndTooltipCard();
        }
    }

    private static uint StepInk(in LiveStep step)
    {
        if (step.Place == StepPlace.Dropped)
            return ConsoleStyle.TextMutedU32;
        bool now = step.Place == StepPlace.Now;
        float back = step.Place == StepPlace.Next ? NextStepAlpha : PastStepAlpha;
        return step.Who switch
        {
            StepWho.Stock => now ? Palette.Stock.U32 : Palette.Stock.U32At(back),
            StepWho.Pilot => now ? ConsoleStyle.TextPrimaryU32 : ConsoleStyle.TextMutedU32,
            _ => now ? Palette.Afc.U32 : Palette.Afc.U32At(back),
        };
    }

    private const float NextStepAlpha = 0.8f;
    private const float PastStepAlpha = 0.55f;

    private static readonly Func<string, float> MeasureCurrentFont = text => ImGui.CalcTextSize(text).X;

    private static string LinkOf(in StatusLine line)
        => line.CueTarget.Length > 0 && !(_cueInFooter && line.Severity == Severity.Cue) ? NextStep.LinkLabel(in line) : "";

    private static string LineText(ConsoleFrame f, in StatusLine line)
        => line.Severity == Severity.Error && f.FailureWhat.Length > 0 ? f.FailureWhat + "." : Fmt.Sentence(line.Text);

    private static uint LineInk(Severity severity) => severity switch
    {
        Severity.Error => ConsoleStyle.DangerU32,
        Severity.Warning => Palette.Warn.U32,
        Severity.Cue => ConsoleStyle.TextPrimaryU32,
        _ => ConsoleStyle.TextLabelU32,
    };

    // The text cut so it fits in room pixels of the value font, with "..." at the end.
    private static string Ellipsis(string text, float room)
    {
        ConsoleStyle.PushValueFont();
        try { return ConsoleUi.FitEllipsis(text, room); }
        finally { ConsoleStyle.PopFont(); }
    }

    private static int DrawNoCraft(ConsoleFrame f, float2 origin, float width, float rowH)
    {
        float gap = ImGui.GetStyle().ItemSpacing.X;
        string what = f.Editor ? "In the editor."
            : f.Eva ? "On EVA. STAGING, TUNE and TRANSFER work."
            : "No craft under control.";
        StopSpec stop = f.OtherStop;
        float rightW = stop.Enabled ? StopWidth() : MoreWidth(f, inStrip: false);
        float room = width - rightW - gap;
        RowText(origin, room, Ellipsis(what, room), ConsoleStyle.TextMutedU32);
        int rows = 1;
        if (stop.Enabled)
        {
            DrawOutcomeRow(stop.Outcome, origin, width, rowH);
            DrawStop(stop, new float2(origin.X + width - rightW, origin.Y), rightW, "AfcHeadStop");
            rows = 2;
        }
        else
            DrawMore(f, new float2(origin.X + width - rightW, origin.Y), ConsoleStyle.FrameHeightPx, rightW, inStrip: false);

        string others = OtherCraftLine(f);
        if (others.Length == 0)
            return rows;
        RowText(new float2(origin.X, origin.Y + rows * rowH), width, Ellipsis(others, width), ConsoleStyle.TextMutedU32);
        return rows + 1;
    }

    public static float FoldHeight
    {
        get
        {
            ConsoleStyle.PushLabelFont();
            float h = ImGui.GetFontSize();
            ConsoleStyle.PopFont();
            return h + 6f * ImGuiHelper.InterfaceScale;
        }
    }

    private static string MoreLabel(ConsoleFrame f, bool inStrip, bool count = true)
        => PageText.MoreLabel(count ? StatusSelector.MoreCount(f.Lines, LineOnRow(f)) + HeadDrawer.OtherCraftCount : 0, AfcConsole.DrawerOpen && !inStrip);

    private static string MoreCard(ConsoleFrame f)
        => PageText.MoreCard(StatusSelector.MoreCount(f.Lines, LineOnRow(f)), HeadDrawer.OtherCraftCount);

    private static float MoreWidth(ConsoleFrame f, bool inStrip, bool count = true) => ConsoleUi.LabelWidth(MoreLabel(f, inStrip, count));

    // The width w is measured before a click on the row can flip the drawer, so the label of this frame may be a little wider or narrower than w.
    private static void DrawMore(ConsoleFrame f, float2 at, float h, float w, bool inStrip, bool count = true)
    {
        bool toggle = ConsoleUi.TextButton("AfcHeadFold", MoreLabel(f, inStrip, count), at, h, ConsoleStyle.TextLabelU32, ConsoleStyle.TextPrimaryU32, out bool hovered, noteItem: true, width: w);
        MoreHovered = hovered;
        if (hovered)
            ConsoleUi.Card("MORE", MoreCard(f));
        if (toggle)
        {
            if (inStrip)
            {
                AfcConsole.Open();
                AfcConsole.DrawerOpen = true;
            }
            else
                AfcConsole.DrawerOpen = !AfcConsole.DrawerOpen;
        }
    }

    private static void DrawFold(ConsoleFrame f, float2 at, float width, bool inStrip)
    {
        float h = FoldHeight;
        float labelW = MoreWidth(f, inStrip);
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        float gap = ImGui.GetStyle().ItemSpacing.X;
        const string openAfc = "Open AFC >";
        float openW = inStrip ? ConsoleUi.LabelWidth(openAfc) : 0f;

        float labelX = at.X + width - labelW;
        float midY = at.Y + h * 0.5f;
        float textX = inStrip ? at.X + openW + gap : at.X;

        int pick = f.Idle ? -1 : MessageIndex(f);
        StatusLine line = pick >= 0 ? f.Lines[pick] : default;
        string link = pick >= 0 ? LinkOf(in line) : "";
        float linkW = ConsoleUi.LabelWidth(link);
        float rightEnd = (link.Length > 0 ? labelX - gap - linkW : labelX) - gap;

        string place = inStrip ? "" : SituationText.FlyingPlace(in f.Situation, f.Mode.NumbersHoldOrbit, f.Mode.HeightShown, f.Mode.ImpactShown);
        string text = pick >= 0 ? LineText(f, in line) : place;
        string card = pick >= 0 ? Fmt.Sentence(line.Text) + (place.Length > 0 ? "\n" + place : "") : place;
        float lineStart = textX;
        if (text.Length > 0)
        {
            ConsoleStyle.PushLabelFont();
            float fontSize = ImGui.GetFontSize();
            string shown = ConsoleUi.FitEllipsis(text, rightEnd - textX);
            dl.AddText(new float2(textX, at.Y + (h - fontSize) * 0.5f), ConsoleStyle.ApplyAlpha(pick >= 0 ? LineInk(line.Severity) : ConsoleStyle.TextMutedU32), shown);
            lineStart = textX + ImGui.CalcTextSize(shown).X + gap;
            ConsoleStyle.PopFont();
            ConsoleUi.NoteRow("AfcHeadMessage", new float2(textX, at.Y), new float2(Math.Max(1f, lineStart - textX), h));
            if ((shown != text || pick >= 0) && ConsoleUi.Hovering(new float2(textX, at.Y), new float2(lineStart, at.Y + h)))
                ConsoleUi.Tooltip(card.AsSpan());
        }
        if (rightEnd > lineStart)
            dl.AddLine(new float2(lineStart, midY), new float2(rightEnd, midY), ConsoleStyle.ApplyAlpha(ConsoleStyle.HairlineU32), ConsoleStyle.WindowBorderThicknessPx);

        if (link.Length > 0 && ConsoleUi.TextButton("AfcHeadCueGo", link, new float2(labelX - gap - linkW, at.Y), h,
                ConsoleStyle.TextLabelU32, ConsoleStyle.TextPrimaryU32, out _, noteItem: true))
            AfcConsole.Navigate(line.CueTarget);

        DrawMore(f, new float2(labelX, at.Y), h, labelW, inStrip);

        if (inStrip)
        {
            bool go = ConsoleUi.TextButton("AfcStripOpen", openAfc, at, h,
                f.FailureWhat.Length > 0 ? ConsoleStyle.TextPrimaryU32 : ConsoleStyle.TextMutedU32, ConsoleStyle.TextPrimaryU32, out bool openHovered);
            if (openHovered)
                ConsoleUi.Tooltip("Opens the console on the page of what runs.".AsSpan());
            if (go)
                AfcConsole.OpenRunning(ConsolePages.PageOf(f.Stop.Target, f.TransferPasses, f.Activity.RegisteredModeLive));
        }
        ImGui.SetCursorScreenPos(at);
        ImGui.Dummy(new float2(width, h));
    }

    private static string NoticeText()
        => Input.GetAssignmentString(InputAction.ToggleTransferPlan) + " opens AFC, not the stock planner. Change this in TUNE, PREFERENCES.";

    private static float NoticeRoom(float width) => Math.Max(1f, width - ConsoleWidgets.ButtonWidth("OK".AsSpan()) - ImGui.GetStyle().ItemSpacing.X);

    private static float NoticeHeight(float width)
    {
        ConsoleStyle.PushValueFont();
        float text = ImGui.CalcTextSize(NoticeText(), false, NoticeRoom(width)).Y;
        ConsoleStyle.PopFont();
        return Math.Max(ConsoleStyle.FrameHeightPx, text) + ImGui.GetStyle().ItemSpacing.Y;
    }

    private static void DrawFirstOpenNotice(float2 at, float width)
    {
        float room = NoticeRoom(width);
        ImGui.SetCursorScreenPos(at);
        ConsoleStyle.PushValueFont();
        ImGui.PushTextWrapPos(ImGui.GetCursorPosX() + room);
        ImGui.PushStyleColor(ImGuiCol.Text, in ConsoleStyle.TextPrimary);
        ImGui.Text(NoticeText());
        ImGui.PopStyleColor();
        ImGui.PopTextWrapPos();
        ConsoleStyle.PopFont();
        ImGui.SetCursorScreenPos(new float2(at.X + width - ConsoleWidgets.ButtonWidth("OK".AsSpan()), at.Y));
        if (ConsoleWidgets.Button("OK".AsSpan(), "AfcFirstOpenOk".AsSpan(), default))
            AfcConsole.DismissFirstOpenNotice();
    }

    #endregion
}
