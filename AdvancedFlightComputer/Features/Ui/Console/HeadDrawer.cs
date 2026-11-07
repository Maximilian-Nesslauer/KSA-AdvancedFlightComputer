using System;
using System.Collections.Generic;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

/// <summary>The drawer, a sheet over the page that the More control of the head opens (AfcConsole.DrawDrawer).</summary>
internal static class HeadDrawer
{
    internal readonly record struct CraftRow(Vehicle? Vehicle, string Id, string Mode, string Detail, CraftActivity Activity);

    private const double ScanSeconds = 0.5;

    private const int NewestShown = 3;
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    private static readonly List<CraftRow> _others = new();
    private static readonly HashSet<string> _seen = new(StringComparer.Ordinal);
    private static readonly HashSet<string> _boostersSeen = new(StringComparer.Ordinal);
    private static readonly List<string> _newBoosters = new();
    private static bool _boostersPrimed;
    private static DateTime _nextScan;

    /// <summary>How many other craft AFC flies or holds, for the drawer button and the AFC menu.</summary>
    public static int OtherCraftCount => _others.Count;

    /// <summary>Refreshes the other craft twice a second. The first scan after a load takes in the boosters that already fly home without a cue.</summary>
    public static void Refresh(Vehicle? subject, bool force)
    {
        if (!force && DateTime.UtcNow < _nextScan)
            return;
        _nextScan = DateTime.UtcNow.AddSeconds(ScanSeconds);
        _others.Clear();
        _seen.Clear();
        if (subject != null)
            _seen.Add(subject.Id);

        if (SharedVehicleHooks.GuidanceEnabled)
        {
            foreach (var entry in VehicleAutopilotState.Snapshot())
            {
                Vehicle vehicle = entry.Key;
                if (vehicle == null || vehicle.IsDisposed || _seen.Contains(vehicle.Id))
                    continue;
                ModeReadout mode = GuidanceWindow.ReadMode(vehicle, entry.Value);
                if (mode.Activity.AnyLive)
                    Add(vehicle, mode);
            }
        }

        foreach (Vehicle vehicle in Program.VehiclesInFrame)
        {
            if (vehicle == null || _seen.Contains(vehicle.Id))
                continue;
            ModeReadout mode = GuidanceWindow.ReadMode(vehicle, null);
            if (mode.Activity.AnyLive)
                Add(vehicle, mode);
        }

        // A multi-pass can run on a craft outside the physics range, which is in the registry but not in frame.
        if (SharedVehicleHooks.MultiPassEnabled)
        {
            string saveId = SaveLoadObserver.CurrentSaveId;
            foreach (var entry in MultiPassRegistry.Snapshot)
            {
                if (entry.Key.Item1 != saveId || _seen.Contains(entry.Key.Item2))
                    continue;
                MultiPassExecution exec = entry.Value;
                _seen.Add(entry.Key.Item2);
                _others.Add(new CraftRow(null, entry.Key.Item2, "PASSES",
                    string.Format(Inv, "PASS {0} OF {1}", exec.PassIndex + 1, exec.PassCountTotal),
                    new CraftActivity { MultiPassActive = true }));
            }
        }

        foreach (CraftRow row in _others)
        {
            if (row.Mode == "BOOSTBACK" && _boostersSeen.Add(row.Id) && _boostersPrimed)
                _newBoosters.Add(row.Id);
        }
        _boostersPrimed = true;
    }

    private static void Add(Vehicle vehicle, in ModeReadout mode)
    {
        _seen.Add(vehicle.Id);
        string detail = mode.Phase != mode.ModeName ? mode.Phase : "";
        if (double.IsFinite(mode.TgoSeconds) && mode.TgoLabel.Length > 0)
            detail += (detail.Length > 0 ? "   " : "") + PageText.Tgo(mode.TgoLabel, FormatHelper.FormatDuration(Math.Max(0.0, mode.TgoSeconds)));
        _others.Add(new CraftRow(vehicle, vehicle.Id, mode.ModeName, detail, mode.Activity));
    }

    /// <summary>The stop of the other craft when exactly one of them has a stop, for the AFC menu while no craft is the subject.</summary>
    public static bool TryGetSingleOtherStop(out Vehicle? vehicle, out StopSpec stop)
    {
        vehicle = null;
        stop = StopSpec.None;
        foreach (CraftRow row in _others)
        {
            if (row.Vehicle == null || row.Vehicle.IsDisposed)
                continue;
            StopSpec candidate = ConsoleFrame.StopFor(row.Vehicle, row.Activity);
            if (!candidate.Enabled)
                continue;
            if (vehicle != null)
                return false;
            vehicle = row.Vehicle;
            stop = candidate;
        }
        return vehicle != null;
    }

    /// <summary>The boosters that started to fly home since the last call, for the head cue.</summary>
    public static IEnumerable<string> NewBoosters()
    {
        if (_newBoosters.Count == 0)
            yield break;
        string[] list = _newBoosters.ToArray();
        _newBoosters.Clear();
        foreach (string id in list)
            yield return id;
    }

    public static void Draw(ConsoleFrame f)
    {
        Vehicle? subject = f.Subject;
        string subjectId = subject?.Id ?? "";
        IReadOnlyList<ConsoleMessages.Entry> messages = ConsoleMessages.Entries;
        if (messages.Count > 0)
        {
            ConsoleUi.SectionHead("MESSAGES");
            for (int i = 0; i < Math.Min(NewestShown, messages.Count); i++)
                DrawMessage(messages[i], subjectId);
        }

        bool runs = false;
        if (subject != null)
        {
            ConsoleUi.SectionHead("GNC SWITCHES");
            bool armsAtLiftOff = f.GuidanceOn && f.Situation.Where == CraftSituation.Ground && GuidanceWindow.LaunchFlownByNote(subject).Length > 0;
            GuidanceWindow.DrawCraftSwitches(subject, f.Activity.AnyGuidanceLive, armsAtLiftOff ? PageText.ArmsAtLiftOffDrawer : "");
            if (SharedVehicleHooks.AutoStageEnabled && GuidanceWindow.AutoStageGuidanceNote(subject, f.Activity.AnyGuidanceLive).Length > 0)
                ConsoleUi.MutedWrapped(PageText.ArmedByGuidance);

            ConsoleUi.BeginSection("DrawerRunning", "ACTIVE");
            runs = DrawRunning(f, subject);
            ConsoleUi.EndSection();
        }

        if (!runs && !f.SubjectBusy && messages.Count == 0 && _others.Count == 0)
            ConsoleUi.Muted(PageText.DrawerEmpty.AsSpan());

        ConsoleUi.BeginSection("DrawerOthers", "OTHER VEHICLES");
        for (int i = 0; i < _others.Count; i++)
        {
            CraftRow row = _others[i];
            string value = row.Mode + (row.Detail.Length > 0 ? ", " + row.Detail : "");
            ConsoleUi.Readout(row.Id.AsSpan(), value.AsSpan(), in ConsoleStyle.TextPrimary, properName: true);
            if (row.Vehicle != null)
            {
                StopSpec stop = ConsoleFrame.StopFor(row.Vehicle, row.Activity);
                if (stop.Enabled && row.Id != f.OtherStopCraft)
                    StopButton(stop, "AfcDrawerOtherStop" + i);
            }
        }
        ConsoleUi.EndSection();

        if (messages.Count > NewestShown && ConsoleUi.Fold("LOG", "DrawerEarlier", tip: PageText.EarlierMessagesTip))
        {
            for (int i = NewestShown; i < messages.Count; i++)
                DrawMessage(messages[i], subjectId);
        }

        if (subject != null && f.GuidanceOn && !StopResolver.NothingToRelease(f.Activity))
        {
            ConsoleUi.Gap();
            ConsoleWidgets.Rule();
            if (ConsoleWidgets.Button("RELEASE ALL".AsSpan(), "AfcDrawerRelease".AsSpan(), default))
                GuidanceWindow.ReleaseGuidance(subject);
            if (ImGui.IsItemHovered())
                ConsoleUi.Card("RELEASE ALL", StopResolver.ReleaseOutcomeFor(f.Activity));
        }
    }

    private static void DrawMessage(in ConsoleMessages.Entry entry, string subjectId)
        => Coloured(entry.Severity, entry.CraftId.Length > 0 && entry.CraftId != subjectId
            ? entry.Time + "  " + entry.CraftId + ": " + Fmt.Sentence(entry.Text)
            : entry.Time + "  " + Fmt.Sentence(entry.Text));

    // The head holds what the one stop ends and the numbers of the burn that stock AUTO flies or that a pass waits for, so those have no row of their own.
    private static bool DrawRunning(ConsoleFrame f, Vehicle subject)
    {
        CraftActivity a = f.Activity;
        // The guidance stop alone: the multi-pass and the RCS burn have their own rows below.
        StopTarget guidance = StopResolver.Resolve(a with { MultiPassActive = false, RcsFlying = false, StockAutoOnNode = false });
        bool any = false;
        if (StopResolver.IsGuidance(guidance) && guidance != f.Stop.Target)
        {
            any = true;
            ModeReadout mode = f.Mode;
            ConsoleUi.Readout("GUIDANCE".AsSpan(), (mode.ModeName
                + (mode.Phase.Length > 0 && mode.Phase != mode.ModeName ? ", " + mode.Phase : "")).AsSpan());
            StopSpec stop = new(guidance, StopResolver.Label(guidance, in a), StopResolver.OutcomeLine(guidance, in a),
                StopResolver.EngineOutcome(guidance, in a), ConsoleFrame.StopAction(subject, guidance));
            StopButton(stop, "AfcDrawerStopGuidance");
        }

        if (a.MultiPassActive && f.Stop.Target != StopTarget.MultiPass && MultiPassRegistry.TryGet(subject.Id, out MultiPassExecution? exec))
        {
            any = true;
            ConsoleUi.Readout("PASSES".AsSpan(), string.Format(Inv, "{0}, PASS {1} OF {2}",
                OrbitToolCatalog.DisplayName(exec.Intent.TypeKey), exec.PassIndex + 1, exec.PassCountTotal).AsSpan());
            StopSpec stop = new(StopTarget.MultiPass, StopResolver.Label(StopTarget.MultiPass), StopResolver.OutcomeLine(StopTarget.MultiPass, in a),
                StopResolver.EngineOutcome(StopTarget.MultiPass, in a), ConsoleFrame.StopAction(subject, StopTarget.MultiPass));
            StopButton(stop, "AfcDrawerStopPasses");
        }

        if (SharedVehicleHooks.RcsEnabled && RcsExecRegistry.TryGet(subject.Id, out RcsExecution? rcs) && rcs.Faulted)
        {
            any = true;
            ConsoleUi.Readout("RCS BURN".AsSpan(), (rcs.CleanupPending ? "STOPPED, CLEANUP FAILED" : "STOPPED: ERROR").AsSpan());
            if (rcs.CleanupPending && ConsoleWidgets.Button("RETRY CLEANUP".AsSpan(), "AfcDrawerRcsRetry".AsSpan(), default))
                RcsStop.RetryCleanup(rcs);
        }
        else if (a.RcsFlying && f.Stop.Target != StopTarget.Rcs)
        {
            any = true;
            ConsoleUi.Readout("RCS BURN".AsSpan(), "ACTIVE".AsSpan());
            StopSpec stop = new(StopTarget.Rcs, StopResolver.Label(StopTarget.Rcs), StopResolver.OutcomeLine(StopTarget.Rcs, in a),
                StopResolver.EngineOutcome(StopTarget.Rcs, in a), ConsoleFrame.StopAction(subject, StopTarget.Rcs));
            StopButton(stop, "AfcDrawerStopRcs");
        }

        Burn? node = BurnModeInput.NextNode(subject);
        if (node != null)
        {
            any = true;
            if (a.StockAutoOnNode)
            {
                StopSpec stop = new(StopTarget.StockAuto, StopResolver.Label(StopTarget.StockAuto), StopResolver.OutcomeLine(StopTarget.StockAuto, in a),
                    StopResolver.EngineOutcome(StopTarget.StockAuto, in a), () => BurnModeInput.StopAuto(subject));
                if (f.Stop.Target != StopTarget.StockAuto)
                    DrawStockAutoRow(in stop);
            }
            DrawNextBurnRow(subject, node, in a);
        }

        return any;
    }

    // The stop of stock AUTO in the form of the row of the next burn: its name at the left, the press at the right and what it does under them, so it does not read as a part of the row above it.
    private static void DrawStockAutoRow(in StopSpec stop)
    {
        float w = ConsoleWidgets.ButtonWidth(PageText.StopWord.AsSpan());
        ConsoleUi.BeginRow(FlownBy.StockAutoName.AsSpan(), w);
        ConsoleUi.RowRight(w);
        bool clicked = ConsoleWidgets.DangerButton(PageText.StopWord.AsSpan(), "AfcDrawerStopAuto".AsSpan(), default);
        ConsoleUi.NoteItem("AfcDrawerStopAuto");
        if (ImGui.IsItemHovered())
            ConsoleUi.Card(stop.Label, stop.Detail);
        ConsoleUi.EndRow();
        if (stop.Outcome.Length > 0)
            ConsoleUi.Muted(stop.Outcome.AsSpan());
        if (clicked)
            stop.Act!();
    }

    // The head holds the numbers while stock AUTO flies the burn or a pass waits, and it holds FLY PASS, so this row has neither.
    private static void DrawNextBurnRow(Vehicle subject, Burn node, in CraftActivity a)
    {
        (string deleteVerb, string deleteCard, string deleteLine) = Pages.PageKit.DeleteOf(subject, node, in a);
        bool headHolds = a.StockAutoOnNode || a.MultiPassActive;
        bool fly = !headHolds && !a.RcsFlying && BurnModeInput.FlyNextRefusal(subject).Length == 0;
        bool delete = !a.RcsFlying;
        string numbers = headHolds ? "" : string.Format(Inv, "{0} IN {1}", Fmt.Dv(node.DeltaVVlf.Length()), FormatHelper.FormatDuration(Math.Max(0.0, (node.Time - Universe.GetElapsedTime()).Seconds())));
        if (!fly && !delete && numbers.Length == 0)
            return;

        float gap = ImGui.GetStyle().ItemSpacing.X;
        float flyW = ConsoleWidgets.ButtonWidth(PlanVerbs.FlyBurn.AsSpan());
        float deleteW = ConsoleWidgets.ButtonWidth(deleteVerb.AsSpan());
        float buttons = (fly ? flyW : 0f) + (fly && delete ? gap : 0f) + (delete ? deleteW : 0f);
        ConsoleUi.BeginRow("NEXT BURN".AsSpan(), Math.Max(1f, buttons));
        bool numbersUnder = numbers.Length > 0 && !ConsoleUi.RowDetail(numbers, buttons);
        ConsoleUi.RowRight(buttons);
        if (fly)
        {
            if (ConsoleWidgets.PositiveButton(PlanVerbs.FlyBurn.AsSpan(), "AfcDrawerFly".AsSpan(), default))
                BurnModeInput.FlyWithAuto(subject);
            if (delete)
                ImGui.SameLine();
        }
        if (delete)
        {
            if (ConsoleWidgets.DangerButton(deleteVerb.AsSpan(), "AfcDrawerDeleteNode".AsSpan(), default))
                BurnModeInput.DeleteNode(subject, node);
            if (ImGui.IsItemHovered())
                ConsoleUi.Card(deleteVerb, deleteCard);
        }
        ConsoleUi.EndRow();
        if (numbersUnder)
            ConsoleUi.Muted(numbers.AsSpan());
        ConsoleUi.NoteShape("AfcDrawerDeleteLine", delete && deleteLine.Length > 0 ? 1 : 0);
        if (delete && deleteLine.Length > 0)
            ConsoleUi.Muted(deleteLine.AsSpan());
    }

    private static void StopButton(StopSpec stop, string id)
    {
        bool clicked = ConsoleWidgets.DangerButton(stop.Label.AsSpan(), id.AsSpan(), default);
        if (ImGui.IsItemHovered())
            ConsoleUi.Card(stop.Label, stop.Detail);
        if (stop.Outcome.Length > 0)
            ConsoleUi.Muted(stop.Outcome.AsSpan());
        if (clicked)
            stop.Act?.Invoke();
    }

    private static void Coloured(Severity severity, string text)
    {
        ConsoleUi.Sentence(text.AsSpan(), severity switch
        {
            Severity.Error => ConsoleStyle.Danger,
            Severity.Warning => ConsoleUi.WarnInk,
            Severity.Cue => ConsoleStyle.TextPrimary,
            _ => ConsoleStyle.TextLabel,
        });
    }

    public static void Reset()
    {
        _others.Clear();
        _seen.Clear();
        _boostersSeen.Clear();
        _newBoosters.Clear();
        _boostersPrimed = false;
        _nextScan = default;
    }
}
