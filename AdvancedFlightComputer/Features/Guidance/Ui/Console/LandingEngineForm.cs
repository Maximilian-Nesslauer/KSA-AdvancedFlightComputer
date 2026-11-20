using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

public static partial class GuidanceWindow
{
    private static void DrawBoosterEngineGroups(Vehicle vehicle, ReturnableStage stage)
    {
        Part? root = vehicle.Parts.Find(stage.RootId);
        if (root == null || stage.EngineGroupScope.Length == 0
            || _s.ReturnableStages.Count(other => other.EngineGroupScope == stage.EngineGroupScope) != 1)
            return;
        DrawLandingEngineGroups(vehicle, root, stage.EngineGroupScope, PartsLeavingWith(vehicle, root, stage.SequenceIndex));
    }

    private static void DrawLandingEngineGroups(Vehicle vehicle, Part root, string scope, HashSet<Part>? allowed = null)
    {
        PartTree? tree = vehicle.Parts;
        if (tree == null)
            return;
        LandingEngineGroups settings = _s.GetEngineGroups(scope);
        ImGui.PushID(scope.Length > 0 ? scope : "CraftEngineGroups");
        try
        {
            // One fold state per scope, so each booster row and the LAND page open on their own.
            if (!ConsoleUi.Fold("LANDING ENGINES  " + (settings.Chosen?.Name ?? "AS STAGED"), LandingEngineFoldId(scope), false))
                return;
            var candidates = LandingEngineGroups.Controllers(vehicle, root, allowed);
            if (settings.Chosen is { } selected && !LandingEngineGroups.Resolve(selected, candidates, out _))
                ConsoleUi.MutedWrapped("The assigned group has a controller that is missing or ambiguous.");
            bool craft = scope == LandingEngineGroups.CraftScope;
            bool editable = !LandFlies && !_s.Running && !_s.LaunchArmed && !_s.LandingEngineCommitPending.HasValue;
            bool manual = craft && editable && CanSwitchEnginesManually(vehicle);
            ImGui.BeginDisabled(!editable);
            try
            {
                if (ConsoleWidgets.Button("AS STAGED".AsSpan(), "EngineAsStaged".AsSpan(), default))
                    SetLandingEngineGroups(scope, settings with { Selected = "" });
                if (!_s.EngineGroupDrafts.TryGetValue(scope, out LandingEngineGroupDraft? draft))
                    _s.EngineGroupDrafts.Add(scope, draft = new LandingEngineGroupDraft());
                foreach (LandingEngineGroup group in settings.Groups)
                {
                    ImGui.PushID(group.Name);
                    ConsoleUi.MutedWrapped($"{group.Name} ({group.Engines.Length})");
                    if (ConsoleWidgets.Button("USE FOR LANDING".AsSpan(), "SelectGroup".AsSpan(), default))
                        SetLandingEngineGroups(scope, settings with { Selected = group.Name });
                    ImGui.SameLine();
                    if (ConsoleWidgets.Button("EDIT".AsSpan(), "EditGroup".AsSpan(), default))
                    {
                        draft.Engines.Clear();
                        draft.Engines.UnionWith(group.Engines);
                        draft.Name.Value16 = group.Name;
                    }
                    if (manual)
                    {
                        ImGui.SameLine();
                        if (ConsoleWidgets.Button("ACTIVATE".AsSpan(), "ActivateGroup".AsSpan(), default))
                            _s.ManualEngineChange = new ManualEngineChange(group.Engines, null);
                    }
                    ImGui.PopID();
                }

                int index = 0;
                foreach ((string key, EngineController? item) in candidates)
                {
                    if (item == null)
                        continue;
                    EngineController engine = item;
                    ImGui.PushID(key);
                    bool member = draft.Engines.Contains(key);
                    string label = $"{++index}. {engine.Parent.DisplayName} - {engine.TemplateId} ({engine.Rockets.Length})";
                    if (ImGui.Checkbox(label.AsSpan(), ref member))
                    {
                        if (member)
                            draft.Engines.Add(key);
                        else
                            draft.Engines.Remove(key);
                    }
                    if (craft)
                    {
                        ImGui.SameLine();
                        ConsoleUi.MutedWrapped(DescribeLandingEngine(vehicle, engine));
                    }
                    if (manual && ConsoleWidgets.Button((engine.IsActive ? "DISABLE" : "ENABLE").AsSpan(), "ToggleEngine".AsSpan(), default))
                        _s.ManualEngineChange = new ManualEngineChange([key], !engine.IsActive);
                    ImGui.PopID();
                }

                ImGui.InputText("Group name"u8, draft.Name);
                string name = draft.Name.ToString().Trim();
                bool valid = name.Length > 0 && draft.Engines.Count > 0
                    && !name.Any(char.IsControl)
                    && draft.Engines.All(key => candidates.TryGetValue(key, out EngineController? engine) && engine != null);
                using (ConsoleUi.Dim(!valid))
                {
                    if (ConsoleWidgets.Button("SAVE GROUP".AsSpan(), "SaveEngineGroup".AsSpan(), default) && valid)
                    {
                        var group = new LandingEngineGroup(name, draft.Engines.Order(StringComparer.Ordinal).ToArray());
                        SetLandingEngineGroups(scope, new LandingEngineGroups(
                            settings.Groups.Where(old => old.Name != name).Append(group).ToArray(), name));
                    }
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

    internal static string LandingEngineFoldId(string scope) => "EngineGroups:" + scope;

    private static void SetLandingEngineGroups(string scope, LandingEngineGroups settings)
    {
        _s.EngineGroups[scope] = settings;
        _s.StageModelDirty = true;
        if (scope == LandingEngineGroups.CraftScope)
            ResetLandingEngineSelection();
    }

    private static string DescribeLandingEngine(Vehicle vehicle, EngineController engine)
    {
        ReadOnlySpan<RocketState> states = RocketStates.Of(vehicle.Parts);
        int fed = 0, firing = 0;
        foreach (Rocket rocket in engine.Rockets)
        {
            if ((uint)rocket.StatesIdx >= (uint)states.Length)
                continue;
            ref readonly RocketState state = ref states[rocket.StatesIdx];
            if (state.IsPropellantAvailable) fed++;
            if (state.Throttle > 0) firing++;
        }
        return $"{(engine.IsActive ? "Active" : "Inactive")}, {fed} fed, {firing} firing";
    }
}
