#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;

//  The spec rows carry a tier and draw only in the scope of that tier (see TierScope). The plain rows without a spec mean the Player tier, so a row nobody tagged stays on the page and never disappears.
//  Ids are explicit rather than ImGuiHelper's _widgetId counter, because that counter is global and reset by ImGuiHelper.StartFrame, which the game's own windows call, so a mod incrementing it would collide with them.
public static partial class GuidanceWindow
{
    // The key of the last spec row drawn and whether its edit ended in that frame. ConsoleWidgets.EndRow submits an item after the input, so a caller cannot ask ImGui about the input itself any more.
    private static string _lastRowKey = "";
    private static bool _lastRowEditEnded;

    /// <summary>Starts a row. <paramref name="changed"/> marks a value that differs from its default, with a dot after its label; stock amber stays for a pending edit, and an AFC value applies at once. <paramref name="controlWidth"/> names a control narrower than the stock column, so the label gets the room.</summary>
    private static void BeginKitRow(string label, bool changed = false, float controlWidth = 0f)
    {
        ConsoleUi.BeginRow(label, controlWidth);
        if (changed)
            ConsoleUi.ChangedDot();
    }

    /// <summary>A switch that is not a guidance setting, such as an overlay toggle, drawn in the scope of <paramref name="tier"/>.</summary>
    private static bool GaugeRowCheck(UiTier tier, string label, string id, ref bool value)
    {
        if (!TierScope.Show(tier))
            return false;
        BeginKitRow(label, controlWidth: ConsoleUi.SwitchWidth(""));
        bool changed = ConsoleUi.RowCheckbox(id.AsSpan().TrimStart('#'), ref value);
        ConsoleUi.EndRow();
        return changed;
    }

    /// <summary>
    /// A typed number row for a spec. The label carries the unit and is marked while the value
    /// differs from its default, the tooltip on the label gives the help text and the default, and
    /// the value is clamped to the spec's range once the edit ends. RowEditEnded says whether the
    /// edit of the row ended in this frame. On TUNE a changed value also shows its default and a RESET (ResetTail).
    /// </summary>
    private static bool GaugeRow(in SettingSpec spec, ref double value)
    {
        if (!TierScope.Show(spec.Tier))
            return false;
        string lockReason = SettingLockReason(spec.Key);
        using var lockScope = new ImGuiDisabledScope(lockReason.Length > 0);
        string label = SettingMath.DisplayLabel(spec);
        ResetTail tail = ResetTail.For(spec.Tier, spec.Key, SettingMath.IsChanged(spec, value), SettingMath.DefaultValue(spec), label, ConsoleUi.FieldWidth);
        // A pane too narrow for the name with its unit has the unit behind a shorter field, so no name loses its end.
        string unit = SettingMath.ShownUnit(spec);
        bool unitBehind = unit.Length > 0 && tail.Width <= 0f && !ConsoleUi.LabelFits(label, ConsoleUi.FieldWidth);
        float control = unitBehind ? ConsoleUi.ShortFieldWidth + ConsoleUi.UnitRoom(unit) : ConsoleUi.FieldWidth + tail.Width;
        if (unitBehind)
            label = spec.Label;
        BeginSpecRow(spec.Key, label, SettingMath.IsChanged(spec, value), control);
        bool hovered = ConsoleWidgets.RowHovered;
        bool reset = tail.Draw(spec.Key, ConsoleUi.FieldWidth);
        if (unitBehind)
            ConsoleUi.RowFieldUnit(ConsoleUi.ShortFieldWidth, unit);
        else
            ConsoleUi.RowField();
        double shown = SettingMath.ToShown(spec, value);
        bool changed;
        using (new ChangedField(ShowsChange(in spec, value)))
            changed = ImGui.InputDouble("##" + spec.Key, ref shown, 0.0, 0.0, SettingMath.ShownFormat(spec), ImGuiInputTextFlags.CharsDecimal);
        if (changed)
            value = SettingMath.FromShown(spec, shown);
        bool ended = ClampOnEnd(in spec, ref value, ref changed);
        if (reset)
        {
            value = spec.Default;
            changed = true;
            ended = true;
        }
        SpecTooltip(label, hovered, spec.Key, LockedHelp(lockReason, spec.Help), SettingMath.GhostText(spec), SettingMath.IsChanged(spec, value));
        ConsoleUi.EndRow();
        _lastRowKey = spec.Key;
        _lastRowEditEnded = ended;
        return changed;
    }

    private static bool ClampOnEnd(in SettingSpec spec, ref double value, ref bool changed, bool round = false)
    {
        if (!ImGui.IsItemDeactivatedAfterEdit())
            return false;
        double clamped = SettingMath.Clamp(spec, value);
        if (round)
            clamped = Math.Round(clamped);
        if (clamped != value)
        {
            value = clamped;
            changed = true;
        }
        return true;
    }

    private static bool ShowsChange(in SettingSpec spec, double value) => SettingMath.IsChanged(spec, value) && SettingAccess.SetBy(_s, spec.Key).Length == 0;

    private readonly struct ChangedField : IDisposable
    {
        private readonly bool _on;

        public ChangedField(bool on)
        {
            _on = on;
            if (!on)
                return;
            float4 ink = Ui.Canvas.Palette.Changed.F4;
            ImGui.PushStyleColor(ImGuiCol.Text, in ink);
        }

        public void Dispose()
        {
            if (_on)
                ImGui.PopStyleColor();
        }
    }

    // The room of one short field of a row that holds several: the gap after a name, with space for the mark of a changed value, the gap before a unit, and the gap to the next field.
    private static float CellNameGap => 10f * ImGuiHelper.InterfaceScale;
    private static float CellUnitGap => 4f * ImGuiHelper.InterfaceScale;
    private static float CellGap => 10f * ImGuiHelper.InterfaceScale;

    /// <summary>The width of each number field on a row that holds the given names and units in <paramref name="room"/> pixels, one width for all so the row reads as one, and never wider than a field on a whole row. <paramref name="rowWidth"/> is what the fields then take, so the row can end at the right edge.</summary>
    private static float CellFieldWidth(float room, out float rowWidth, params ReadOnlySpan<(string Name, string Unit)> cells)
    {
        float used = CellsWithoutFields(cells);
        float field = Math.Clamp((room - used) / Math.Max(1, cells.Length), 30f * ImGuiHelper.InterfaceScale, ConsoleUi.FieldWidth);
        rowWidth = used + field * cells.Length;
        return field;
    }

    private static float CellsWithoutFields(ReadOnlySpan<(string Name, string Unit)> cells)
    {
        float used = CellGap * Math.Max(0, cells.Length - 1);
        foreach ((string name, _) in cells)
            used += ConsoleUi.LabelWidth(name) + CellNameGap;
        foreach ((_, string unit) in cells)
            if (unit.Length > 0)
                used += ConsoleUi.SmallWidth(unit) + CellUnitGap;
        return used;
    }

    /// <summary>
    /// One number field of a row that holds several, in the row just begun: a short name, the field and its unit, from <paramref name="x"/> at the row top <paramref name="y"/>. It behaves as <see cref="GaugeRow"/> does: the same id, the same clamp when the edit ends, the same card on a hover, and RowEditEnded answers for it. Moves <paramref name="x"/> to the next field. A row with little room can leave out the unit and show the value at a shorter <paramref name="format"/>, which changes only what the field shows, because ImGui parses a value only from an edit. <paramref name="marksChange"/> is false for a goal that a pilot always sets, such as a landing site, which no default makes normal. A name that ties to a colour of a picture takes <paramref name="nameInk"/>.
    /// </summary>
    private static bool GaugeCell(in SettingSpec spec, string name, ref float x, float y, float fieldW, ref double value, string format = null, bool showUnit = true, bool marksChange = true, uint? nameInk = null)
    {
        if (!TierScope.Show(spec.Tier))
            return false;
        string lockReason = SettingLockReason(spec.Key);
        using var lockScope = new ImGuiDisabledScope(lockReason.Length > 0);
        float h = ConsoleStyle.FrameHeightPx;
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        float from = x;

        ConsoleStyle.PushLabelFont();
        float nameW = ImGui.CalcTextSize(name).X;
        dl.AddText(new float2(x, y + (h - ImGui.GetFontSize()) * 0.5f), ConsoleStyle.ApplyAlpha(nameInk ?? ConsoleStyle.TextLabelU32), name);
        ConsoleStyle.PopFont();
        if (marksChange && ShowsChange(in spec, value))
            ConsoleUi.ChangedDotAfter(x + nameW);
        x += nameW + CellNameGap;

        ImGui.SetCursorScreenPos(new float2(x, y));
        ImGui.SetNextItemWidth(fieldW);
        bool changed = ImGui.InputDouble("##" + spec.Key, ref value, 0.0, 0.0, format ?? spec.Format, ImGuiInputTextFlags.CharsDecimal);
        bool ended = ClampOnEnd(in spec, ref value, ref changed);
        x += fieldW;

        if (showUnit)
        {
            x += CellUnitGap;
            ConsoleUi.PushSmall();
            float unitW = ImGui.CalcTextSize(spec.Unit).X;
            dl.AddText(new float2(x, y + (h - ImGui.GetFontSize()) * 0.5f), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32), spec.Unit);
            ConsoleStyle.PopFont();
            x += unitW;
        }

        bool hovered = ConsoleWidgets.RowHovered && ImGui.IsMouseHoveringRect(new float2(from, y), new float2(x, y + h));
        SpecTooltip(spec.Label, hovered, spec.Key, LockedHelp(lockReason, spec.Help), SettingMath.GhostText(spec), SettingMath.IsChanged(spec, value));
        ConsoleUi.NoteRow(spec.Key, new float2(from, y), new float2(x - from, h));
        x += CellGap;
        _lastRowKey = spec.Key;
        _lastRowEditEnded = ended;
        return changed;
    }

    /// <summary>True when the row of <paramref name="spec"/> was the last spec row drawn in this scope and its edit ended this frame.</summary>
    private static bool RowEditEnded(in SettingSpec spec)
        => TierScope.Show(spec.Tier) && _lastRowEditEnded && _lastRowKey == spec.Key;

    private static bool GaugeRowInt(in SettingSpec spec, ref int value)
    {
        if (!TierScope.Show(spec.Tier))
            return false;
        string lockReason = SettingLockReason(spec.Key);
        using var lockScope = new ImGuiDisabledScope(lockReason.Length > 0);
        string label = SettingMath.DisplayLabel(spec);
        float field = ConsoleUi.FieldWidth;
        ResetTail tail = ResetTail.For(spec.Tier, spec.Key, SettingMath.IsChanged(spec, value), SettingMath.DefaultValue(spec), label, field);
        BeginSpecRow(spec.Key, label, SettingMath.IsChanged(spec, value), field + tail.Width);
        bool hovered = ConsoleWidgets.RowHovered;
        if (tail.Draw(spec.Key, field))
            value = (int)Math.Round(spec.Default);
        ConsoleUi.RowRight(field);
        ImGui.SetNextItemWidth(field);
        bool changed;
        using (new ChangedField(ShowsChange(in spec, value)))
            changed = ImGui.InputInt("##" + spec.Key, ref value);
        double whole = value;
        ClampOnEnd(in spec, ref whole, ref changed, round: true);
        value = (int)whole;
        SpecTooltip(label, hovered, spec.Key, LockedHelp(lockReason, spec.Help), SettingMath.GhostText(spec), SettingMath.IsChanged(spec, value));
        ConsoleUi.EndRow();
        return changed;
    }

    private static bool GaugeRowCheck(in BoolSpec spec, ref bool value)
    {
        if (!TierScope.Show(spec.Tier))
            return false;
        string lockReason = SettingLockReason(spec.Key);
        using var lockScope = new ImGuiDisabledScope(lockReason.Length > 0);
        float control = ConsoleUi.SwitchWidth("");
        ResetTail tail = ResetTail.For(spec.Tier, spec.Key, SettingMath.IsChanged(spec, value), SettingMath.DefaultValue(spec), spec.Label, control);
        BeginSpecRow(spec.Key, spec.Label, SettingMath.IsChanged(spec, value), control + tail.Width);
        bool hovered = ConsoleWidgets.RowHovered;
        bool reset = tail.Draw(spec.Key, control);
        bool changed = ConsoleUi.RowCheckbox(spec.Key.AsSpan(), ref value);
        if (reset)
        {
            value = spec.Default;
            changed = true;
        }
        SpecTooltip(spec.Label, hovered, spec.Key, LockedHelp(lockReason, spec.Help), SettingMath.GhostText(spec), SettingMath.IsChanged(spec, value));
        ConsoleUi.EndRow();
        return changed;
    }

    private static void GaugeRowText(string label, string value, float4? colour = null)
        => GaugeRowText(UiTier.Player, label, value, colour);

    private static void GaugeRowText(UiTier tier, string label, string value, float4? colour = null)
    {
        if (!TierScope.Show(tier))
            return;
        if (label.Length == 0 && value.Length == 0)
        {
            ConsoleUi.Gap();
            return;
        }
        float4 ink = colour ?? ConsoleStyle.TextPrimary;
        ConsoleUi.Readout(label.AsSpan(), Fmt.Sentence(value.Trim()).AsSpan(), in ink);
    }

    private readonly struct ResetTail
    {
        private readonly string _text;

        public readonly float Width;

        private ResetTail(string text, float width)
        {
            _text = text;
            Width = width;
        }

        public static ResetTail For(UiTier tier, string key, bool changed, string defaultValue, string label, float controlW)
        {
            if (tier != UiTier.Tuning || !changed || SettingAccess.SetBy(_s, key).Length > 0)
                return default;
            float gap = ImGui.GetStyle().ItemSpacing.X;
            float fixedW = ConsoleWidgets.ButtonWidth(PageText.ResetVerb.AsSpan()) + gap * 2f;
            float labelW = ConsoleUi.LabelWidth(label);
            string full = "Default " + defaultValue;
            float fullW = ConsoleUi.SmallWidth(full);
            float shortW = ConsoleUi.SmallWidth(defaultValue);
            float room = ConsoleStyle.ContentAvailWidth() - controlW - ConsoleWidgets.MinControlGapPx - labelW;
            return room >= fullW + fixedW ? new ResetTail(full, fullW + fixedW) : new ResetTail(defaultValue, shortW + fixedW);
        }

        /// <summary>Draws the tail in the row just begun, with the cursor at the control column, and leaves the cursor there, because a stock switch takes the whole column. Returns true on a press of RESET.</summary>
        public bool Draw(string key, float controlW)
        {
            if (Width <= 0f)
                return false;
            float2 control = ImGui.GetCursorScreenPos();
            float gap = ImGui.GetStyle().ItemSpacing.X;
            float right = control.X + ConsoleWidgets.RowControlWidth - controlW - gap;
            float buttonW = ConsoleWidgets.ButtonWidth(PageText.ResetVerb.AsSpan());
            float h = ConsoleStyle.FrameHeightPx;
            ImGui.SetCursorScreenPos(new float2(right - buttonW, control.Y + (h - ConsoleWidgets.ButtonHeight) * 0.5f));
            bool hit = ConsoleWidgets.Button(PageText.ResetVerb.AsSpan(), ("AfcRowReset" + key).AsSpan(), default);
            if (!ConsoleUi.LastItemDisabled)
                TuneRowResets++;
            ConsoleUi.PushSmall();
            float textW = ImGui.CalcTextSize(_text).X;
            ImGui.GetWindowDrawList().AddText(new float2(right - buttonW - gap - textW, control.Y + (h - ImGui.GetFontSize()) * 0.5f), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32), _text);
            ConsoleStyle.PopFont();
            ImGui.SetCursorScreenPos(control);
            return hit;
        }
    }

    // A value guidance writes by itself is not marked, and its tooltip says who sets it instead of a default the player does not get.
    private static void BeginSpecRow(string key, string label, bool changed, float controlWidth = 0f)
    {
        if (SettingAccess.SetBy(_s, key).Length > 0)
            changed = false;
        BeginKitRow(label, changed, controlWidth);
    }

    // A locked row is drawn disabled, so its card says first why it takes no edit.
    private static string LockedHelp(string lockReason, string help) => lockReason.Length > 0 ? lockReason + "\n\n" + help : help;

    private static void SpecTooltip(string title, bool hovered, string key, string help, string ghost, bool changed)
    {
        if (!hovered || ImGui.IsItemActive())
            return;
        ConsoleUi.Card(title, PageText.SettingTooltip(help, changed, ghost, SettingAccess.SetBy(_s, key)));
    }

    private static void VehicleHeightRows()
    {
        GaugeRowCheck(GuidanceSettingSpecs.VehicleHeightAuto, ref _s.VehicleHeightAuto);
        SettingSpec height = GuidanceSettingSpecs.VehicleHeightM;
        if (!_s.VehicleHeightAuto)
        {
            GaugeRow(height, ref _s.VehicleHeightM);
            return;
        }
        GaugeRowText(height.Tier, height.Label, Fmt.Meters(_s.VehicleHeightM));
        if (TierScope.Show(height.Tier))
            SpecTooltip(height.Label, ImGui.IsItemHovered(), height.Key, height.Help, SettingMath.GhostText(height), false);
        if (_s.VehicleHeightStatus.Length == 0 || _s.VehicleHeightStowed)
            return;
        bool warn = _s.VehicleHeightStatus.StartsWith("no colliders", StringComparison.Ordinal);
        GaugeRowText(GuidanceSettingSpecs.VehicleHeightM.Tier, "", Fmt.Sentence(_s.VehicleHeightStatus),
            warn ? ConsoleUi.WarnInk : ConsoleStyle.TextMuted);
    }

    private static void DrawLegsNote(Vehicle vehicle, LandWay way)
    {
        Orbit orbit = vehicle?.Orbit;
        IParentBody parent = orbit?.Parent;
        bool shows = false;
        bool roomOnly = false;
        double impact = double.NaN;
        if (parent != null && _s.VehicleHeightAuto && _s.VehicleHeightStowed && _s.VehicleHeightStatus.Length > 0)
        {
            impact = ImpactSeconds(vehicle, guidanceBound: true);
            SituationFacts facts = Ui.Console.ConsoleFrame.ReadFacts(vehicle);
            ChainStep[] chain = LandChain.For(way, LandSolverOf(way), facts.Where, facts.HasAir, _s.ThreeDofFromGlide, _s.ThreeDofSixDofHandover, BelowBrakingStart(vehicle), _landHoverFlies);
            shows = FormRules.LegsNoteShows(facts.Where, TerminalHeight(orbit, parent, parent.MeanRadius), impact, LandFlies);
            roomOnly = shows && LandChain.EndsInImpact(chain);
        }
        ConsoleUi.NoteShape("AfcLandLegs", shows ? 1 : 0);
        if (!shows)
            return;
        if (roomOnly)
        {
            ConsoleUi.SentenceRoom(PageText.LegsStowed);
            return;
        }
        if (FormRules.LegsUrgent(impact))
        {
            ConsoleUi.Warning(PageText.LegsStowed.AsSpan());
            if (_s.VehicleHeightCard.Length > 0 && ImGui.IsItemHovered())
                ConsoleUi.Tooltip(_s.VehicleHeightCard.AsSpan());
        }
        else
            ConsoleUi.TickedNote(ConsoleUi.EscapeFormat(PageText.LegsStowed), _s.VehicleHeightCard);
    }

    private static void DrawLiftNote(Vehicle vehicle, LandWay way, bool roomOnly = false)
    {
        string note = AeroLiftNote(vehicle?.Orbit?.Parent);
        bool shows = note.Length > 0 && (LandFlies || FormRules.FlightNoticeShows(Ui.Console.ConsoleFrame.ReadFacts(vehicle).Where, way));
        ConsoleUi.NoteShape("AfcLandLift", shows ? 1 : 0);
        if (shows && roomOnly)
            ConsoleUi.SentenceRoom(note);
        else if (shows)
            ConsoleUi.MutedNote(ConsoleUi.EscapeFormat(note), Fmt.Sentence(AeroLiftWarning()));
    }
}
