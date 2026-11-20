using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Canvas;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

public static partial class GuidanceWindow
{
    private const float EngineMapSizePx = 168f;

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
            KsaEnginePerf.AmbientPressureAt(parent, 0.0), dry, loaded, "WITH RESERVE");
    }

    /// <summary>The craft's LANDING ENGINES section, on every way of LAND, in setup and in flight.</summary>
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
                                                double pressurePa, double dryKg, double loadedKg, string loadedLabel)
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
            if (!ConsoleUi.Fold("LANDING ENGINES  " + title, LandingEngineFoldId(scope), false))
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
                Choose(scope, Toggle(chosen, clicked), options);
            ConsoleUi.MutedWrapped(editable
                ? "Click an engine to add it to the landing or take it out. Blue engines land."
                : "The choice is fixed while AFC flies.");
            DrawOtherEngines(layout, chosen, scope, options, editable);
            DrawEngineChoiceNumbers(layout, chosen, loadedLabel);
            DrawEngineSuggestions(options, chosen, scope, editable);

            ImGui.BeginDisabled(!editable);
            try
            {
                if (chosen.Count > 0 && ConsoleWidgets.Button("AS STAGED".AsSpan(), "EngineAsStaged".AsSpan(), default))
                    Choose(scope, [], options);
                if (manual && chosen.Count > 0)
                {
                    if (chosen.Count > 0)
                        ImGui.SameLine();
                    if (ConsoleWidgets.Button("APPLY NOW".AsSpan(), "ApplyEngineGroup".AsSpan(), default))
                        _s.ManualEngineChange = new ManualEngineChange(chosen.ToArray(), null);
                    if (ImGui.IsItemHovered())
                        ConsoleUi.Tooltip("Switch on exactly these engines now, for flying by hand. The stock throttle still fires them.");
                }
            }
            finally
            {
                ImGui.EndDisabled();
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
        LandingEngineDot? under = null;
        float r = Math.Max(layout.DotRadius * perMetre, 3f * scale);
        foreach (LandingEngineDot dot in layout.Dots)
        {
            if (!dot.AftPlane)
                continue;
            // Seen from behind the stage, so the picture keeps the stage's own left and right.
            var p = new float2(middle.X + dot.X * perMetre, middle.Y - dot.Y * perMetre);
            bool isChosen = chosen.Contains(dot.Key);
            dl.AddCircleFilled(p, r, ConsoleStyle.ApplyAlpha(isChosen ? Palette.Afc.U32 : Palette.Grid.U32At(0.7f)));
            dl.AddCircle(p, r, ConsoleStyle.ApplyAlpha(isChosen ? Palette.Text.U32 : Palette.Dim.U32), 0, 1.2f * scale);
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
        if (under != null)
            ConsoleUi.Tooltip(DescribeEngineDot(vehicle, under, live, states));
        return clicked && editable && under != null ? under.Key : null;
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
                    Choose(scope, Toggle(chosen, dot.Key), options);
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
        (string text, float4 ink) = VerdictText(s);
        ConsoleUi.Readout("LANDING", text, in ink);
    }

    private static void DrawEngineSuggestions(LandingEngineOption[] options, HashSet<string> chosen, string scope, bool editable)
    {
        if (options.Length == 0)
            return;
        ConsoleUi.Gap();
        ConsoleUi.MutedWrapped("Balanced choices (TWR from empty at minimum throttle to the start of the burn at full throttle):");
        ImGui.BeginDisabled(!editable);
        try
        {
            foreach (LandingEngineOption option in options)
            {
                bool isChosen = option.Keys.Length == chosen.Count && option.Keys.All(chosen.Contains);
                string row = EngineText($"{option.Name,-16} {option.Stats.Engines,2}  {option.Stats.TwrMinDry,4:F1}-{option.Stats.TwrMaxLoaded,-4:F1}  {ShortVerdict(option)}");
                ImGui.PushID(option.Name);
                if (ImGui.Selectable(row.AsSpan(), isChosen, ImGuiSelectableFlags.None, (float2?)null))
                    Choose(scope, option.Keys, [option]);
                if (ImGui.IsItemHovered())
                    ConsoleUi.Tooltip(VerdictText(option.Stats).Text);
                ImGui.PopID();
            }
        }
        finally
        {
            ImGui.EndDisabled();
        }
    }

    private static string ShortVerdict(LandingEngineOption option) => (option.Recommended ? "BEST " : "") + option.Stats.Verdict switch
    {
        LandingEngineVerdict.Good => "HOVERS",
        LandingEngineVerdict.NoHover => "NO HOVER",
        LandingEngineVerdict.Weak => "WEAK",
        LandingEngineVerdict.TooWeak => "TOO WEAK",
        _ => "OFF AXIS",
    };

    private static (string Text, float4 Ink) VerdictText(LandingEngineStats s) => s.Verdict switch
    {
        LandingEngineVerdict.Good => ("CAN HOVER AND BRAKE", Palette.Good.F4),
        LandingEngineVerdict.NoHover => ("CANNOT HOVER: MINIMUM THRUST OVER WEIGHT", Palette.Warn.F4),
        LandingEngineVerdict.Weak => ("LITTLE BRAKING MARGIN", Palette.Warn.F4),
        LandingEngineVerdict.TooWeak => ("TOO WEAK TO BRAKE", Palette.Bad.F4),
        _ => (EngineText($"THRUST {s.OffsetM:F2} m OFF THE AXIS: TURNS THE STAGE"), Palette.Bad.F4),
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
    private static void Choose(string scope, string[] keys, LandingEngineOption[] options)
    {
        if (keys.Length == 0)
        {
            SetLandingEngineGroups(scope, LandingEngineGroups.Empty);
            return;
        }
        string name = LandingEngineLayout.Matching(options, keys)?.Name ?? $"CUSTOM {keys.Length}";
        var group = new LandingEngineGroup(name, keys.Order(StringComparer.Ordinal).ToArray());
        SetLandingEngineGroups(scope, new LandingEngineGroups([group], group.Name));
    }

    private static void SetLandingEngineGroups(string scope, LandingEngineGroups settings)
    {
        _s.EngineGroups[scope] = settings;
        _s.StageModelDirty = true;
        if (scope == LandingEngineGroups.CraftScope)
            ResetLandingEngineSelection();
    }

    private static string EngineText(FormattableString text) => text.ToString(CultureInfo.InvariantCulture);
}
