using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

// The AUTOSTAGE section of the Mods settings page, drawn through ModSettingsPage.
internal static class AutoStageSettingsPage
{
    public static void DrawSection()
    {
        ConsoleWidgets.Rule();
        ConsoleWidgets.RegionHeader("AUTOSTAGE".AsSpan());

        bool active = StagingDetector.Active;
        if (ConsoleUi.CheckboxRow("AUTOMATIC STAGING".AsSpan(), "AutoStageActive".AsSpan(), ref active,
                "The same switch as the AUTOSTAGE gauge button. Stages when the active engines run out of propellant, and drops spent boosters while the rest keeps firing.".AsSpan()))
            StagingDetector.Active = active;

        // Takes effect immediately; SAVE below writes it to disk, like the delay tables.
        bool dropSpentStages = StagingConfig.DropSpentStages;
        if (ConsoleUi.CheckboxRow("DROP SPENT STAGES EARLY".AsSpan(), "AutoStageDropSpent".AsSpan(), ref dropSpentStages,
                "Stage as soon as the next sequence would shed nothing but burnt-out engines, so spent boosters drop while the core stage keeps firing. Off: staging waits until every active engine is dry.".AsSpan()))
            StagingConfig.DropSpentStages = dropSpentStages;

        if (GameReflection.ModLibrary_AllParts == null)
        {
            ImGui.TextDisabled("(delay settings unavailable on this game build)"u8);
            return;
        }

        ImGui.TextWrapped(
            "Per-part-variant delays in seconds. Both delays are measured from the staging trigger, " +
            "so set the decoupler delay shorter than the engine delay if the decoupler should fire first.");
        ImGui.Spacing();

        List<PartInfo> engines = GetKnownParts(ref _knownEngines, DeclaresEngine);
        List<PartInfo> decouplers = GetKnownParts(ref _knownDecouplers, DeclaresDecoupler);

        if (ImGui.CollapsingHeader("Engine Ignition Delays"u8, ImGuiTreeNodeFlags.DefaultOpen))
            DrawDelayTable(engines, "eng", StagingConfig.GetEngineDelay, (id, v) => StagingConfig.EngineDelays[id] = v);

        if (ImGui.CollapsingHeader("Decoupler Delays"u8, ImGuiTreeNodeFlags.DefaultOpen))
            DrawDelayTable(decouplers, "dec", StagingConfig.GetDecouplerDelay, (id, v) => StagingConfig.DecouplerDelays[id] = v);

        ImGui.Spacing();
        if (ConsoleWidgets.Button("SAVE".AsSpan()))
        {
            StagingConfig.SaveGlobalConfig();
            TimedAlert.Create("AutoStage config saved", Color.Green, 2.0);
        }
    }

    // An InputFloat rather than a drag control, so typing an exact delay and the step buttons keep working.
    private static void DrawDelayTable(List<PartInfo> parts, string idPrefix,
        Func<string, double> get, Action<string, double> set)
    {
        if (parts.Count == 0)
        {
            ImGui.TextDisabled("(no matching parts loaded)"u8);
            return;
        }

        foreach (PartInfo p in parts)
        {
            float delay = (float)get(p.TemplateId);
            ConsoleWidgets.BeginRow(p.DisplayName.AsSpan());
            ImGui.SetNextItemWidth(ConsoleWidgets.RowControlWidth);
            if (ImGui.InputFloat($"###{idPrefix}_{p.TemplateId}", ref delay, 0.1f, 1.0f, "%.1f"))
                set(p.TemplateId, Math.Max(0.0, (double)delay));
            ConsoleWidgets.EndRow();
        }
    }

    // Same scope the game sequences: the template plus its direct sub-parts.
    private static bool DeclaresEngine(PartTemplate template)
        => DeclaresModule(template, static t => t.RocketEngineControllers.Count > 0);

    private static bool DeclaresDecoupler(PartTemplate template)
        => DeclaresModule(template, HasDecouplerComponent);

    private static bool DeclaresModule(PartTemplate template, Func<PartTemplate, bool> onOwnTemplate)
    {
        if (onOwnTemplate(template))
            return true;

        foreach (PartInstance subPart in template.SubPartInstances)
        {
            try
            {
                if (onOwnTemplate(subPart.GetTemplate()))
                    return true;
            }
            catch (Exception ex)
            {
                // Contained, so one malformed part cannot empty the whole table.
                DefaultCategory.Log.Warning(
                    $"[AFC] Part '{template.Id}' references the sub-part '{subPart.InstanceOf}', which does not resolve: {ex.Message}");
            }
        }
        return false;
    }

    private static bool HasDecouplerComponent(PartTemplate template)
    {
        foreach (ModuleBase.TemplateDataBase component in template.Components)
        {
            if (component is Decoupler.TemplateData)
                return true;
        }
        return false;
    }

    private struct PartInfo
    {
        public string TemplateId;
        public string DisplayName;
    }

    private static List<PartInfo>? _knownEngines;
    private static List<PartInfo>? _knownDecouplers;

    private static List<PartInfo> GetKnownParts(ref List<PartInfo>? cache, Func<PartTemplate, bool> filter)
    {
        if (cache != null)
            return cache;

        cache = new List<PartInfo>();
        try
        {
            if (GameReflection.ModLibrary_AllParts?.GetValue(null) is not SerializedCollection<PartTemplate> collection)
                return cache;

            var raw = new List<(string id, string name)>();
            foreach (PartTemplate template in collection.GetList())
            {
                // A delay is keyed on the tree part, so a sub-part row would be inert.
                if (!template.IsSubPart && filter(template))
                    raw.Add((template.Id, template.DisplayName));
            }

            var nameCounts = new Dictionary<string, int>();
            foreach (var (_, name) in raw)
                nameCounts[name] = nameCounts.GetValueOrDefault(name) + 1;

            foreach (var (id, name) in raw)
            {
                string displayName = name;
                if (nameCounts[name] > 1)
                {
                    int lastUnderscore = id.LastIndexOf('_');
                    displayName = $"{name} ({(lastUnderscore >= 0 ? id.Substring(lastUnderscore + 1) : id)})";
                }
                cache.Add(new PartInfo { TemplateId = id, DisplayName = displayName });
            }

            cache.Sort((a, b) => string.Compare(a.DisplayName, b.DisplayName, StringComparison.Ordinal));
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] Failed to enumerate the part templates: {ex.Message}");
        }
        return cache;
    }

    internal static void Reset()
    {
        _knownEngines = null;
        _knownDecouplers = null;
    }
}
