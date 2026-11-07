using System;
using System.Collections.Generic;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console.Pages;

/// <summary>The picture is the staging bar of the craft, one block per stock sequence that burns now or is still to fire, with the delta-v stock gives it and its ignition and decoupler delays under it.</summary>
internal static class StagingPage
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private static bool _openExpertNext;

    internal static void OpenWithDelays()
    {
        _openExpertNext = true;
        AfcConsole.Open(ConsolePage.Staging);
    }

    public static void Reset()
    {
        _openExpertNext = false;
        _boxes.Clear();
    }

    public static CommitSpec Draw(in PageContext ctx)
    {
        if (!SharedVehicleHooks.AutoStageEnabled)
        {
            ConsoleUi.MutedWrapped("AutoStage is unavailable because its game hooks did not load, or the standalone mod is installed.");
            return CommitSpec.None;
        }

        Vehicle? vehicle = ctx.Frame.Subject;
        if (vehicle != null)
            DrawPicture(vehicle);

        ctx.ToInputs();
        bool flight = vehicle != null && !ctx.Frame.Editor;
        string refusal = flight && !StagingHelpers.HasNextEngineSequence(vehicle!) ? PageText.NoStageLeft : "";
        // The switch makes the StagingDetector.Arm call of the AUTOSTAGE gauge button.
        if (flight)
            GuidanceWindow.DrawAutoStageRow(vehicle!, ctx.Frame.Activity.AnyGuidanceLive, PageText.AutoStageRowLabel, refusal, withLink: false,
                "It stages when the active engines run out of propellant, and drops spent boosters while the rest keeps firing. "
                + PageText.AutoStageHudNote);
        bool off = flight && !StagingDetector.IsArmed(vehicle!);
        using (ConsoleUi.Dim(off))
            AutoStageDefaults.DrawDropSpentRow();
        bool note = off && refusal.Length == 0;
        ConsoleUi.NoteShape("AfcStagingDropNote", note ? 1 : 0);
        if (note)
            ConsoleUi.MutedWrapped(PageText.DropSpentNeedsAutoStage);

        if (_openExpertNext)
        {
            ConsoleUi.OpenFold(PageKit.FoldId("staging"));
            _openExpertNext = false;
        }
        ConsoleUi.Gap();
        ConsoleUi.PartsFold(PageText.Delays, PageKit.FoldId("staging"),
        [
            new FoldPart("", true, static () => ConsoleUi.MutedWrapped(PageText.DelaysRule)),
            new FoldPart(FoldNames.ThisCraft, vehicle != null, () => DrawThisCraftDelays(vehicle!)),
            new FoldPart(FoldNames.NewCraft, true, () => DrawDefaultDelays(vehicle)),
        ], vehicle != null ? CraftOverrideCount(vehicle) : 0, ctx.FoldsOpen || ctx.Frame.Editor, PageText.DelaysTip);

        if (AutoStageDefaults.Unsaved)
            return new CommitSpec(PageText.SaveDefaultsVerb, FlownBy.Nobody, "", "Defaults not saved", true, SaveDefaultsTip,
                Act: AutoStageDefaults.SaveDefaults, Owner: Owner.Settings, Outcome: PageText.SaveDefaultsOutcome, Quiet: true);
        if (off && refusal.Length == 0 && ctx.Frame.GuidanceOn && ctx.Frame.Situation.Where == CraftSituation.Ground && GuidanceWindow.LaunchFlownByNote(vehicle!).Length > 0)
            return CommitSpec.StatusOnly(PageText.AutoStageOffUntilLaunch);
        return CommitSpec.None;
    }

    private static void DrawThisCraftDelays(Vehicle vehicle)
    {
        ConsoleUi.SectionHead(FoldNames.ThisCraft, PageText.CraftDelaysDetail);
        DrawCraftDelays(vehicle);
    }

    private static void DrawDefaultDelays(Vehicle? vehicle)
    {
        if (ConsoleUi.SectionHeadButton(FoldNames.NewCraft, PageText.DefaultDelaysDetail, PageText.SaveDefaultsVerb, "AfcStagingSave", SaveDefaultsTip))
            AutoStageDefaults.SaveDefaults();
        AutoStageDefaults.DrawDefaultTables(vehicle);
    }

    private const string SaveDefaultsTip = "Writes DROP SPENT STAGES EARLY and the default delays to autostage.toml.\n" + PageText.TagSaveDefaults;

    // It holds no reference to the game, so it may outlive the frame.
    private readonly record struct Box(int Number, double Dv, bool Active, bool Next, StageHueRange Hue, double Ignition, double Decoupler, bool Burning);

    private const int MostSplits = 4;

    private static readonly List<Box> _boxes = new();

    private static void ReadBoxes(Vehicle vehicle, SequenceList list)
    {
        _boxes.Clear();
        SequenceBurnState state = MultiPassPreviewCache.GetSequenceState(vehicle);
        int activeSequence = list.ActiveSequence;
        bool nextFound = false;
        bool thrust = !ConsoleFrame.EngineKnownOff(vehicle);
        ReadOnlySpan<Sequence> sequences = list.Sequences;
        for (int i = 0; i < sequences.Length; i++)
        {
            Sequence seq = sequences[i];
            double dv = state.DeltaVOf(seq.Number);
            if (!StagingRules.HasBox(seq.Activated, seq.Number, activeSequence, dv))
                continue;
            bool next = !seq.Activated && !nextFound;
            nextFound |= next;
            StageHueRange hue = StagingRules.HoldsDv(dv) ? StageHues.Of(vehicle, seq.Number) : StageHueRange.None;
            _boxes.Add(new Box(seq.Number, dv, seq.Activated, next, hue,
                StagingConfig.GetSequenceEngineDelay(vehicle, seq.Number), StagingConfig.GetSequenceDecouplerDelay(vehicle, seq.Number), seq.Activated && thrust));
        }
    }

    // The total is the figure of the stock panel, because the blocks read the numbers of that panel.
    private static void DrawPicture(Vehicle vehicle)
    {
        SequenceList? list = vehicle.Parts?.SequenceList;
        if (list != null)
            ReadBoxes(vehicle, list);
        else
            _boxes.Clear();
        double totalDv = 0.0;
        bool ignitions = false, decouplers = false;
        foreach (Box box in _boxes)
        {
            totalDv += box.Dv;
            ignitions |= StagingRules.DelayShows(box.Active, box.Ignition);
            decouplers |= StagingRules.DelayShows(box.Active, box.Decoupler);
        }
        if (!Picture.Shows("AfcStagingPic", _boxes.Count > 0))
        {
            Picture.EmptyState(list == null ? PageText.NoStages : PageText.AllStagesFired);
            return;
        }

        int laneRows = (ignitions ? 1 : 0) + (decouplers ? 1 : 0);
        ConsoleUi.NoteShape("AfcStagingDelays", laneRows);
        string big = StagingRules.Total(totalDv, _boxes.Count);
        Picture.StockCaption(PageText.SequenceWord);
        PictureFrame probe = Picture.Probe();
        float head = Picture.HeadHeight(in probe, big);
        float laneH = laneRows > 0 ? Picture.TextLaneHeight(laneRows) : 2f * probe.Scale;
        using (PictureScope pic = Picture.Open(Picture.FrameHeight(head + 46f * probe.Scale + laneH)))
        {
            PictureFrame f = pic.Frame;
            float s = f.Scale;
            PictureLanes lanes = Picture.Lanes(in f, head, 0f, 0f, laneH, Picture.StripPlot);
            PicBox plot = lanes.Plot;
            float left = plot.X + 4f * s, right = plot.Right - 4f * s;
            float top = plot.Y + 2f * s, bottom = plot.Bottom - 2f * s;
            float minW = 26f * s;
            float spare = Math.Max(0f, (right - left) - minW * _boxes.Count);
            Span<float> widths = stackalloc float[Math.Min(_boxes.Count, 64)];
            ConsoleUi.PushSmall();
            try
            {
                float font = ImGui.GetFontSize();
                bool withWord = true;
                for (int i = 0; i < widths.Length; i++)
                {
                    Box box = _boxes[i];
                    widths[i] = minW + (totalDv > 0.0 ? (float)(box.Dv / totalDv) * spare : spare / _boxes.Count);
                    if (ImGui.CalcTextSize(StagingRules.BoxLabel(box.Number, box.Next, true)).X + 8f * s > widths[i])
                        withWord = false;
                }
                float x = left;
                Span<float> shares = stackalloc float[MostSplits];
                Span<int> rowHues = stackalloc int[MostSplits];
                for (int i = 0; i < widths.Length; i++)
                {
                    Box box = _boxes[i];
                    float w = widths[i];
                    // The next one has the mark stock gives it.
                    Rgb8 fill = box.Hue.Any ? ColorRules.Stage(box.Hue.First) : default;
                    if (box.Hue.Any)
                        DrawBoxFill(in f, vehicle, in box, shares, rowHues, x + 1f * s, x + w - 1f * s, top, bottom);
                    else
                        f.Dl.AddRectFilled(new float2(x + 1f * s, top), new float2(x + w - 1f * s, bottom), Palette.Dim.A(0.3f), 0f);
                    ImColor8 ink = box.Hue.Any ? ((Rgb)ColorRules.InkOn(fill)).Solid : Palette.Text.Solid;
                    if (box.Next)
                        f.Dl.AddRectFilled(new float2(x + 1f * s, top), new float2(x + w - 1f * s, top + 2f * s), Palette.Flown.Solid, 0f);
                    string name = StagingRules.BoxLabel(box.Number, box.Next, withWord, burns: box.Burning);
                    if (ImGui.CalcTextSize(name).X + 8f * s > w)
                        name = StagingRules.BoxLabel(box.Number, box.Next, withWord);
                    f.Dl.AddText(new float2(x + 4f * s, top + 4f * s), ink, name);
                    string dvText = StagingRules.BoxDv(box.Dv);
                    if (ImGui.CalcTextSize(dvText).X + 8f * s <= w)
                        f.Dl.AddText(new float2(x + 4f * s, top + 6f * s + font), StagingRules.HoldsDv(box.Dv) ? ink : Palette.Dim.Solid, dvText);
                    x += w;
                }

                float rowH = font + 3f * s;
                float rowY = lanes.Bottom.Y + 2f * s;
                if (ignitions)
                {
                    DrawDelayLane(in f, widths, left, rowY, StagingRules.IgnitionWord, static box => StagingRules.DelayShows(box.Active, box.Ignition) ? box.Ignition : 0.0);
                    rowY += rowH;
                }
                if (decouplers)
                    DrawDelayLane(in f, widths, left, rowY, StagingRules.DecouplerWord, static box => StagingRules.DelayShows(box.Active, box.Decoupler) ? box.Decoupler : 0.0);
            }
            finally
            {
                ConsoleStyle.PopFont();
            }
            Picture.BigNumber(in f, big, Palette.Text.Solid);
            Picture.HeadText(in f, lanes.Head, PageText.SequenceCaption, Palette.Dim.Solid, big);
            DrawBoxCard(widths, left, top, bottom);
        }
    }

    private static void DrawBoxFill(in PictureFrame f, Vehicle vehicle, in Box box, Span<float> shares, Span<int> hues, float left, float right, float top, float bottom)
    {
        int rows = box.Hue.TwoHues ? StageHues.Rows(vehicle, box.Number, shares, hues) : 0;
        if (rows < 2)
        {
            f.Dl.AddRectFilled(new float2(left, top), new float2(right, bottom), Palette.Stage(box.Hue.First), 0f);
            return;
        }
        float from = left;
        for (int k = 0; k < rows; k++)
        {
            float to = k == rows - 1 ? right : from + (right - left) * shares[k];
            f.Dl.AddRectFilled(new float2(from, top), new float2(to, bottom), Palette.Stage(hues[k]), 0f);
            from = to;
        }
    }

    private static void DrawDelayLane(in PictureFrame f, ReadOnlySpan<float> widths, float left, float y, string word, Func<Box, double> delay)
    {
        float s = f.Scale;
        float x = left;
        for (int i = 0; i < widths.Length; i++)
        {
            double seconds = delay(_boxes[i]);
            if (seconds > 0.0)
            {
                float room = f.Max.X - x - 2f * s;
                float nextX = x + widths[i];
                for (int j = i + 1; j < widths.Length; j++)
                {
                    if (delay(_boxes[j]) > 0.0)
                    {
                        room = nextX - x - 6f * s;
                        break;
                    }
                    nextX += widths[j];
                }
                string text = StagingRules.Delay(word, seconds);
                if (ImGui.CalcTextSize(text).X > room)
                    text = StagingRules.Delay("", seconds);
                f.Dl.AddText(new float2(x + 2f * s, y), Palette.Dim.Solid, ConsoleUi.FitEllipsis(text, room));
            }
            x += widths[i];
        }
    }

    private static void DrawBoxCard(ReadOnlySpan<float> widths, float left, float top, float bottom)
    {
        float x = left;
        for (int i = 0; i < widths.Length; i++)
        {
            if (ConsoleUi.Hovering(new float2(x, top), new float2(x + widths[i], bottom)))
            {
                Box box = _boxes[i];
                ConsoleUi.Tooltip(StagingRules.Card(box.Number, box.Dv, box.Burning, box.Next, box.Active ? 0.0 : box.Ignition, box.Active ? 0.0 : box.Decoupler).AsSpan());
                return;
            }
            x += widths[i];
        }
    }

    // This craft's delays per sequence: the same overrides the part window block edits, saved per craft design.
    private static void DrawCraftDelays(Vehicle vehicle)
    {
        SequenceList? list = vehicle.Parts?.SequenceList;
        if (list == null)
            return;
        StagingConfig.LoadVehicleOverrides(vehicle.Id);
        ReadOnlySpan<Sequence> sequences = list.Sequences;
        bool any = false;
        for (int i = 0; i < sequences.Length; i++)
        {
            Sequence seq = sequences[i];
            if (seq.Activated)
                continue;
            any = true;
            if (Fires(seq, DelayKind.Engine))
                DelayRow(vehicle, seq.Number, DelayKind.Engine);
            if (Fires(seq, DelayKind.Decoupler))
                DelayRow(vehicle, seq.Number, DelayKind.Decoupler);
        }
        if (!any)
            ConsoleUi.Muted("Every stage has fired".AsSpan());
    }

    // The delay overrides this craft holds for its stages still to fire.
    private static int CraftOverrideCount(Vehicle vehicle)
    {
        SequenceList? list = vehicle.Parts?.SequenceList;
        if (list == null)
            return 0;
        StagingConfig.LoadVehicleOverrides(vehicle.Id);
        int count = 0;
        foreach (Sequence seq in list.Sequences)
        {
            if (seq.Activated)
                continue;
            count += (StagingConfig.HasSequenceEngineOverride(vehicle, seq.Number) ? 1 : 0)
                + (StagingConfig.HasSequenceDecouplerOverride(vehicle, seq.Number) ? 1 : 0);
        }
        return count;
    }

    // A delay row only for a module kind the sequence fires, the same test the part window and StagingConfig.ComputeSequenceMaxDelay use.
    private static bool Fires(Sequence seq, DelayKind kind)
    {
        ReadOnlySpan<Part> parts = seq.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            foreach (ISequenced module in parts[i].InSequence(seq.Number))
            {
                if (SequencedModules.Matches(module, kind))
                    return true;
            }
        }
        return false;
    }

    private static void DelayRow(Vehicle vehicle, int sequence, DelayKind kind)
    {
        bool engine = kind == DelayKind.Engine;
        double value = engine ? StagingConfig.GetSequenceEngineDelay(vehicle, sequence) : StagingConfig.GetSequenceDecouplerDelay(vehicle, sequence);
        double partDefault = engine ? StagingConfig.ComputeSequenceEngineDelay(vehicle, sequence) : StagingConfig.ComputeSequenceDecouplerDelay(vehicle, sequence);
        bool overridden = engine ? StagingConfig.HasSequenceEngineOverride(vehicle, sequence) : StagingConfig.HasSequenceDecouplerOverride(vehicle, sequence);
        string label = string.Format(Inv, "SEQ {0} {1}", sequence, engine ? "IGNITION" : "DECOUPLER");

        float field = ConsoleUi.FieldWidth;
        float gap = ImGui.GetStyle().ItemSpacing.X;
        float resetW = ConsoleWidgets.ButtonWidth(PageText.ResetVerb.AsSpan());
        ConsoleUi.BeginRow(label.AsSpan(), field + (overridden ? resetW + gap : 0f));
        StageHueRange hue = StageHues.Of(vehicle, sequence);
        if (hue.Any)
            ConsoleUi.SwatchAfterLabel(ColorRules.Stage(hue.First), hue.TwoHues ? ColorRules.Stage(hue.Second) : null);
        float2 control = ImGui.GetCursorScreenPos();
        if (overridden)
        {
            ConsoleUi.RowRight(field + resetW + gap);
            if (ConsoleWidgets.Button(PageText.ResetVerb.AsSpan(), $"AfcDelayReset{kind}{sequence}".AsSpan(), default))
            {
                if (engine)
                    StagingConfig.ClearSequenceEngineOverride(vehicle, sequence);
                else
                    StagingConfig.ClearSequenceDecouplerOverride(vehicle, sequence);
                StagingConfig.FlushPendingSaves();
            }
            if (ImGui.IsItemHovered())
                ConsoleUi.Tooltip(string.Format(Inv, "Back to the part default, {0:F1} s.", partDefault).AsSpan());
            ImGui.SetCursorScreenPos(control);
        }
        ConsoleUi.RowRight(field);
        float delay = (float)value;
        ImGui.SetNextItemWidth(field);
        if (ImGui.InputFloat($"##AfcDelay{kind}{sequence}", ref delay, 0.1f, 1.0f, "%.1f"))
        {
            if (engine)
                StagingConfig.SetSequenceEngineOverride(vehicle, sequence, delay);
            else
                StagingConfig.SetSequenceDecouplerOverride(vehicle, sequence, delay);
        }
        if (ImGui.IsItemHovered())
            ConsoleUi.Tooltip(("The delay of this sequence on this craft, from the staging trigger. Saved at once.\n" + PageText.TagCraftDesign).AsSpan());
        if (ImGui.IsItemDeactivatedAfterEdit())
            StagingConfig.FlushPendingSaves();
        ConsoleUi.EndRow();
    }
}
