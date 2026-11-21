using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Canvas;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

namespace AdvancedFlightComputer.Features.Guidance;

public static partial class GuidanceWindow
{
    private const float EngineMapSizePx = 168f;

    /// <summary>A returnable stage's LANDING ENGINES section on its LAUNCH booster row. The row draws it only once the stage comes home (a site or an optimal return), so it opens by default: picking the target is when the landing engines matter. A fold the player closed stays closed.</summary>
    private static void DrawBoosterEngineGroups(Vehicle vehicle, ReturnableStage stage)
    {
        Part? root = vehicle.Parts.Find(stage.RootId);
        if (root == null || stage.EngineGroupScope.Length == 0
            || _s.ReturnableStages.Count(other => other.EngineGroupScope == stage.EngineGroupScope) != 1
            || vehicle.Orbit?.Parent is not { } parent)
            return;
        HashSet<Part> leaving = PartsLeavingWith(vehicle, root, stage.SequenceIndex);
        // The booster lands nearly empty, and it starts the landing burn with at most the reserve the climb keeps for it.
        double dry = BoosterShare(vehicle, leaving, stage.SequenceNumber).DryKg;
        double reserve = PlannedBoosterReserveKg(vehicle, stage.RootId);
        double loaded = double.IsFinite(reserve) && reserve > 0 ? dry + reserve : double.NaN;
        DrawLandingEngineGroups(vehicle, root, stage.EngineGroupScope, leaving, parent,
            KsaEnginePerf.AmbientPressureAt(parent, 0.0), dry, loaded, "WITH RESERVE", openByDefault: true);
    }

    /// <summary>
    /// The craft's LANDING ENGINES section, on every way of LAND, in setup and in flight: the craft being flown only. A returnable stage's choice is made on its booster row under LAUNCH, and it goes with the stage when it separates (RememberStageEngineChoices).
    /// </summary>
    internal static void DrawLandEngineRows(Vehicle vehicle)
    {
        if (vehicle.Parts?.Root is not { } root || vehicle.Orbit?.Parent is not { } parent)
            return;
        double dry = vehicle.TotalMass - vehicle.PropellantMass;
        DrawLandingEngineGroups(vehicle, root, LandingEngineGroups.CraftScope, null, parent,
            KsaEnginePerf.AmbientPressureAt(parent, SiteTerrainHeight(parent)), dry, vehicle.TotalMass, "NOW");
    }

    internal static string LandingEngineFoldId(string scope) => "EngineGroups:" + scope;

    private static void DrawLandingEngineGroups(Vehicle vehicle, Part root, string scope, HashSet<Part>? allowed, IParentBody parent,
                                                double pressurePa, double dryKg, double loadedKg, string loadedLabel,
                                                bool openByDefault = false)
    {
        if (vehicle.Parts == null)
            return;
        LandingEngineGroups settings = _s.GetEngineGroups(scope);
        ImGui.PushID(scope.Length > 0 ? scope : "CraftEngineGroups");
        try
        {
            var candidates = LandingEngineGroups.Controllers(vehicle, root, allowed);
            double radius = parent.MeanRadius;
            var layout = LandingEngineLayout.Build(vehicle, candidates, pressurePa, parent.Mu / (radius * radius), dryKg, loadedKg);
            LandingEngineOption[] options = layout.Suggestions();
            HashSet<string> chosen = settings.Chosen is { } group ? group.Engines.ToHashSet(StringComparer.Ordinal) : [];
            int onPicture = layout.Dots.Count(d => d.AftPlane);
            string title = chosen.Count == 0 ? "AS STAGED"
                : $"{settings.Chosen!.Name} ({chosen.Count} OF {layout.Dots.Length})";
            if (!ConsoleUi.Fold("LANDING ENGINES  " + title, LandingEngineFoldId(scope), openByDefault))
                return;
            if (settings.Chosen is { } assigned && !LandingEngineGroups.Resolve(assigned, candidates, out _))
                ConsoleUi.WarningWrapped("An engine of the landing choice is missing from this stage.");
            if (layout.Dots.Length == 0)
            {
                ConsoleUi.MutedWrapped("This stage has no engine controller to choose.");
                return;
            }

            bool craft = scope == LandingEngineGroups.CraftScope;
            bool editable = !LandFlies && !_s.Running && !_s.LaunchArmed && !_s.LandingEngineCommitPending.HasValue;
            bool manual = craft && editable && CanSwitchEnginesManually(vehicle);

            if (onPicture > 0 && DrawEngineMap(vehicle, layout, chosen, craft, editable) is { } clicked)
                Choose(scope, Toggle(chosen, clicked), layout, options);
            ConsoleUi.MutedWrapped(editable
                ? "Click an engine to add it to the landing or take it out, or pick a choice below."
                : "The choice is fixed while AFC flies.");
            DrawOtherEngines(layout, chosen, scope, options, editable);
            DrawEngineChoiceNumbers(layout, chosen, loadedLabel);
            DrawEngineSuggestions(layout, options, chosen, scope, editable, loadedLabel);
            if (craft)
                GaugeRowCheck(Specs.ParkIdleEngines, ref _s.ParkIdleEngines);

            if (manual && chosen.Count > 0)
            {
                if (ConsoleWidgets.Button("APPLY NOW".AsSpan(), "ApplyEngineGroup".AsSpan(), default))
                    _s.ManualEngineChange = new ManualEngineChange(chosen.ToArray(), null);
                if (ImGui.IsItemHovered())
                    ConsoleUi.Tooltip("Switch on exactly these engines now, for flying by hand. The stock throttle still fires them.");
            }
        }
        finally
        {
            ImGui.PopID();
        }
    }

    /// <summary>Draws the aft view of the engines and returns the key of an engine the player clicked, or null.</summary>
    private static string? DrawEngineMap(Vehicle vehicle, LandingEngineLayout layout, HashSet<string> chosen, bool live, bool editable)
    {
        float scale = ImGuiHelper.InterfaceScale;
        float size = Math.Min(ConsoleStyle.ContentAvailWidth(), EngineMapSizePx * scale);
        float2 at = ImGui.GetCursorScreenPos();
        bool clicked = ImGui.InvisibleButton("##EngineMap", new float2(size, size));
        bool hovered = ImGui.IsItemHovered();
        float2 mouse = ImGui.GetMousePos();
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        var middle = new float2(at.X + size * 0.5f, at.Y + size * 0.5f);
        float outer = size * 0.5f - 2f * scale;
        float perMetre = (outer - 4f * scale) / Math.Max(layout.Extent, 0.1f);
        dl.AddCircleFilled(middle, outer, ConsoleStyle.ApplyAlpha(new Rgb(8, 10, 14).U32At(0.75f)));
        dl.AddCircle(middle, outer, ConsoleStyle.ApplyAlpha(Palette.Grid.U32), 0, 1.5f * scale);

        ReadOnlySpan<RocketState> states = live ? RocketStates.Of(vehicle.Parts) : default;
        // A choice row under the mouse in the last frame shows its engines here, because the rows are drawn after the picture.
        string[]? preview = _enginePreview;
        _enginePreview = null;
        LandingEngineDot? under = null;
        float r = Math.Max(layout.DotRadius * perMetre, 3f * scale);
        foreach (LandingEngineDot dot in layout.Dots)
        {
            if (!dot.AftPlane)
                continue;
            // Seen from behind the stage, so the picture keeps the stage's own left and right.
            var p = new float2(middle.X + dot.X * perMetre, middle.Y - dot.Y * perMetre);
            bool isChosen = chosen.Contains(dot.Key);
            bool previewed = preview != null && Array.IndexOf(preview, dot.Key) >= 0;
            dl.AddCircleFilled(p, r, ConsoleStyle.ApplyAlpha(isChosen ? Palette.Afc.U32 : Palette.Grid.U32At(0.7f)));
            dl.AddCircle(p, r, ConsoleStyle.ApplyAlpha(isChosen ? Palette.Text.U32 : Palette.Dim.U32), 0, 1.2f * scale);
            if (previewed)
                dl.AddCircle(p, r - 2.5f * scale, ConsoleStyle.ApplyAlpha(Palette.Warn.U32), 0, 2.5f * scale);
            if (!dot.Gimballed)
                dl.AddCircleFilled(p, Math.Max(1.5f * scale, 0.18f * r), ConsoleStyle.ApplyAlpha(Palette.Dim.U32));
            if (live)
            {
                (bool active, bool fed, bool firing) = EngineState(dot.Engine, states);
                if (firing)
                    dl.AddCircle(p, r + 2.5f * scale, ConsoleStyle.ApplyAlpha(Palette.Good.U32), 0, 2f * scale);
                else if (active && !fed)
                    dl.AddCircle(p, r + 2.5f * scale, ConsoleStyle.ApplyAlpha(Palette.Bad.U32), 0, 2f * scale);
                else if (active)
                    dl.AddCircle(p, r + 2.5f * scale, ConsoleStyle.ApplyAlpha(Palette.Text.U32At(0.8f)), 0, 1f * scale);
            }
            float dx = mouse.X - p.X, dy = mouse.Y - p.Y;
            if (hovered && dx * dx + dy * dy <= r * r)
            {
                under = dot;
                dl.AddCircle(p, r + 1f * scale, ConsoleStyle.ApplyAlpha(Palette.Text.U32), 0, 2f * scale);
            }
        }
        DrawEngineLegend(dl, new float2(at.X + size + 10f * scale, at.Y + 6f * scale), live, preview != null);
        if (under != null)
            ConsoleUi.Tooltip(DescribeEngineDot(vehicle, under, live, states));
        return clicked && editable && under != null ? under.Key : null;
    }

    private static string[]? _enginePreview;

    /// <summary>The key to the picture, in the room to its right.</summary>
    private static void DrawEngineLegend(ImDrawListPtr dl, float2 at, bool live, bool previewing)
    {
        float scale = ImGuiHelper.InterfaceScale;
        float line = ImGui.GetTextLineHeightWithSpacing();
        float dot = 5f * scale;
        float room = ConsoleStyle.ContentAvailWidth() - (at.X - ImGui.GetCursorScreenPos().X);
        if (room < 90f * scale)
            return;
        void Row(int i, uint fill, uint ring, float ringWidth, string text, bool innerDot = false)
        {
            var c = new float2(at.X + dot, at.Y + i * line + line * 0.5f);
            dl.AddCircleFilled(c, dot, ConsoleStyle.ApplyAlpha(fill));
            if (innerDot)
                dl.AddCircleFilled(c, 1.5f * scale, ConsoleStyle.ApplyAlpha(Palette.Dim.U32));
            if (ringWidth > 0f)
                dl.AddCircle(c, dot + 2f * scale, ConsoleStyle.ApplyAlpha(ring), 0, ringWidth);
            dl.AddText(new float2(at.X + 3f * dot, at.Y + i * line + (line - ImGui.GetTextLineHeight()) * 0.5f),
                ConsoleStyle.ApplyAlpha(Palette.Dim.U32), text);
        }
        int row = 0;
        Row(row++, Palette.Afc.U32, 0, 0f, "LANDS");
        Row(row++, Palette.Grid.U32, 0, 0f, "STAYS OFF");
        Row(row++, Palette.Grid.U32, 0, 0f, "NO GIMBAL", innerDot: true);
        if (previewing)
            Row(row++, Palette.Grid.U32, Palette.Warn.U32, 2f * scale, "IN THE CHOICE");
        if (!live)
            return;
        Row(row++, Palette.Grid.U32, Palette.Text.U32, 1f * scale, "SWITCHED ON");
        Row(row++, Palette.Grid.U32, Palette.Good.U32, 2f * scale, "FIRING");
        Row(row, Palette.Grid.U32, Palette.Bad.U32, 2f * scale, "NO PROPELLANT");
    }

    private static void DrawOtherEngines(LandingEngineLayout layout, HashSet<string> chosen, string scope, LandingEngineOption[] options, bool editable)
    {
        LandingEngineDot[] others = layout.Dots.Where(d => !d.AftPlane).ToArray();
        if (others.Length == 0)
            return;
        ConsoleUi.MutedWrapped("Engines further up the stack:");
        ImGui.BeginDisabled(!editable);
        try
        {
            foreach (LandingEngineDot dot in others)
            {
                ImGui.PushID(dot.Key);
                string label = (chosen.Contains(dot.Key) ? "LANDS: " : "") + dot.Engine.TemplateId;
                if (ConsoleWidgets.Button(label.AsSpan(), "OtherEngine".AsSpan(), default))
                    Choose(scope, Toggle(chosen, dot.Key), layout, options);
                ImGui.PopID();
            }
        }
        finally
        {
            ImGui.EndDisabled();
        }
    }

    private static void DrawEngineChoiceNumbers(LandingEngineLayout layout, HashSet<string> chosen, string loadedLabel)
    {
        IReadOnlyCollection<string> keys = chosen.Count > 0 ? chosen : layout.Dots.Where(d => d.Engine.Sequence > 0).Select(d => d.Key).ToArray();
        if (keys.Count == 0)
            return;
        LandingEngineStats s = layout.Stats(keys);
        ConsoleUi.Gap();
        if (chosen.Count == 0)
            ConsoleUi.MutedWrapped("As staged, the numbers are for every staged engine of the stage.");
        ConsoleUi.Readout("ENGINES", EngineText($"{s.Engines} OF {layout.Dots.Length}, {s.Gimbals} ON GIMBALS"));
        ConsoleUi.Readout("THRUST", EngineText($"{s.ThrustMinN / 1e3:F0} - {s.ThrustMaxN / 1e3:F0} kN"));
        ConsoleUi.Readout("TWR EMPTY", EngineText($"{s.TwrMinDry:F2} - {s.TwrMaxDry:F2} AT {layout.DryKg / 1e3:F1} t"));
        if (double.IsFinite(layout.LoadedKg))
            ConsoleUi.Readout("TWR " + loadedLabel, EngineText($"UP TO {s.TwrMaxLoaded:F2} AT {layout.LoadedKg / 1e3:F1} t"));
        (string shortText, string text, float4 ink) = VerdictText(s);
        ConsoleUi.Readout("LANDING", shortText, in ink);
        if (ImGui.IsItemHovered())
            ConsoleUi.Tooltip(text);
    }

    private static void DrawEngineSuggestions(LandingEngineLayout layout, LandingEngineOption[] options, HashSet<string> chosen,
                                              string scope, bool editable, string loadedLabel)
    {
        string[] staged = layout.Dots.Where(d => d.Engine.Sequence > 0).Select(d => d.Key).ToArray();
        if (options.Length == 0 && staged.Length == 0)
            return;
        bool loaded = double.IsFinite(layout.LoadedKg);
        ConsoleUi.Gap();
        ConsoleUi.MutedWrapped(loaded
            ? $"HOVER: TWR at minimum throttle, empty (under 1 hovers). BRAKE: TWR at full throttle, {loadedLabel.ToLowerInvariant()}."
            : "HOVER: TWR at minimum throttle, empty (under 1 hovers). BRAKE: TWR at full throttle, empty until the climb is solved.");
        ImGui.TextColored(in ConsoleStyle.TextMuted, EngineText($"  {"CHOICE",-17}{"N",2}  {"HOVER",5}  {"BRAKE",5}").AsSpan());
        ImGui.BeginDisabled(!editable);
        try
        {
            if (staged.Length > 0)
            {
                LandingEngineStats s = layout.Stats(staged);
                if (SuggestionRow("AS STAGED", s, "", chosen.Count == 0, staged))
                    Choose(scope, [], layout, options);
            }
            LandingEngineOption? current = layout.Matching(options, chosen);
            foreach (LandingEngineOption option in options)
            {
                bool isChosen = ReferenceEquals(option, current);
                if (SuggestionRow(option.Name, option.Stats, option.Recommended ? "BEST" : "", isChosen, option.Keys) && !isChosen)
                    Choose(scope, option.Keys, layout, [option]);
            }
        }
        finally
        {
            ImGui.EndDisabled();
        }
    }

    /// <summary>One clickable row of the choice table. Under the mouse, the picture marks its engines on the next frame.</summary>
    private static bool SuggestionRow(string name, LandingEngineStats s, string tag, bool isChosen, string[] keys)
    {
        string row = EngineText($"{(isChosen ? ">" : " ")} {name,-17}{s.Engines,2}  {s.TwrMinDry,5:F2}  {s.TwrMaxLoaded,5:F2}  {ShortVerdict(s.Verdict)}{(tag.Length > 0 ? "  " + tag : "")}");
        ImGui.PushID(name);
        if (isChosen)
            ImGui.PushStyleColor(ImGuiCol.Text, Palette.Afc.F4);
        bool picked = ImGui.Selectable(row.AsSpan(), isChosen, ImGuiSelectableFlags.None, (float2?)null);
        if (isChosen)
            ImGui.PopStyleColor();
        if (ImGui.IsItemHovered())
        {
            _enginePreview = keys;
            ConsoleUi.Tooltip(VerdictText(s).Text);
        }
        ImGui.PopID();
        return picked;
    }

    private static string ShortVerdict(LandingEngineVerdict verdict) => verdict switch
    {
        LandingEngineVerdict.Good => "OK",
        LandingEngineVerdict.NoHover => "NO HOVER",
        LandingEngineVerdict.Weak => "WEAK",
        LandingEngineVerdict.TooWeak => "TOO WEAK",
        _ => "OFF AXIS",
    };

    private static (string Short, string Text, float4 Ink) VerdictText(LandingEngineStats s) => s.Verdict switch
    {
        LandingEngineVerdict.Good => ("CAN HOVER AND BRAKE", "Can hover near the ground and has braking margin.", Palette.Good.F4),
        LandingEngineVerdict.NoHover => ("CANNOT HOVER",
            "Even minimum thrust lifts the empty stage, so the burn has to end exactly at the ground. Fewer engines can hover.", Palette.Warn.F4),
        LandingEngineVerdict.Weak => ("LITTLE BRAKING MARGIN", "Under 1.5 TWR at full throttle at the start of the burn: the burn starts low and long.", Palette.Warn.F4),
        LandingEngineVerdict.TooWeak => ("TOO WEAK TO BRAKE", "Full thrust does not lift the stage. AFC refuses this choice.", Palette.Bad.F4),
        _ => (EngineText($"{s.OffsetM:F2} m OFF AXIS"),
            "The thrust center is off the stage axis, so the engines turn the stage and their gimbals must hold them. Touchdown guidance plans for that moment. When entry guidance engages, AFC measures it from the centre of mass and lands on all engines instead if it takes over 80 % of what the gimbals can hold. A balanced set keeps the gimbals free.",
            Palette.Warn.F4),
    };

    private static string DescribeEngineDot(Vehicle vehicle, LandingEngineDot dot, bool live, ReadOnlySpan<RocketState> states)
    {
        string text = EngineText($"{dot.Engine.TemplateId} on {dot.Engine.Parent.FullPart.DisplayName}\n")
            + EngineText($"Thrust {dot.ThrustMinN / 1e3:F0} - {dot.ThrustMaxN / 1e3:F0} kN, minimum throttle {dot.Engine.MinimumThrottle:P0}\n")
            + (dot.Gimballed ? "On a gimbal" : "No gimbal");
        if (!live)
            return text;
        (bool active, bool fed, bool firing) = EngineState(dot.Engine, states);
        return text + "\n" + (firing ? "Firing" : active ? (fed ? "Switched on" : "Switched on, no propellant") : "Switched off");
    }

    private static (bool Active, bool Fed, bool Firing) EngineState(EngineController engine, ReadOnlySpan<RocketState> states)
    {
        bool fed = false, firing = false;
        foreach (Rocket rocket in engine.Rockets)
        {
            if ((uint)rocket.StatesIdx >= (uint)states.Length)
                continue;
            fed |= states[rocket.StatesIdx].IsPropellantAvailable;
            firing |= states[rocket.StatesIdx].Throttle > 0;
        }
        return (engine.IsActive, fed, firing);
    }

    private static string[] Toggle(HashSet<string> chosen, string key)
    {
        var next = new HashSet<string>(chosen, StringComparer.Ordinal);
        if (!next.Add(key))
            next.Remove(key);
        return next.ToArray();
    }

    /// <summary>Makes <paramref name="keys"/> the landing choice of the scope, named after the matching suggestion, or as staged when empty.</summary>
    private static void Choose(string scope, string[] keys, LandingEngineLayout layout, LandingEngineOption[] options)
    {
        if (keys.Length == 0)
        {
            SetLandingEngineGroups(scope, LandingEngineGroups.Empty);
            return;
        }
        string name = layout.Matching(options, keys)?.Name ?? $"CUSTOM {keys.Length}";
        var group = new LandingEngineGroup(name, keys.Order(StringComparer.Ordinal).ToArray());
        SetLandingEngineGroups(scope, new LandingEngineGroups([group], group.Name));
    }

    private static void SetLandingEngineGroups(string scope, LandingEngineGroups settings)
    {
        _s.EngineGroups[scope] = settings;
        _s.StageModelDirty = true;
        // One line per change, so a flight's log says which choice each stage carried.
        string stage = scope == LandingEngineGroups.CraftScope ? "this craft"
            : _s.ReturnableStages.Find(s => s.EngineGroupScope == scope) is { } owner ? $"stage '{owner.Name}'" : "a stage no longer listed";
        if (Program.ControlledVehicle is { } subject)
            GuidanceLog.Info(subject, settings.Chosen is { } chosen
                ? $"landing engines for {stage}: '{chosen.Name}', {chosen.Engines.Length} controller(s)."
                : $"landing engines for {stage}: as staged.");
        if (scope == LandingEngineGroups.CraftScope)
            ResetLandingEngineSelection();
    }

    private static string EngineText(FormattableString text) => text.ToString(CultureInfo.InvariantCulture);
}
