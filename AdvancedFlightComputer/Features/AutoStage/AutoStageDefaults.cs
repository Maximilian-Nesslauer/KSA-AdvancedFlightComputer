using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

internal static class AutoStageDefaults
{
    /// <summary>
    /// The DROP SPENT STAGES EARLY switch. It sits in the player form while SAVE DEFAULTS sits in the
    /// DELAYS fold, so a click saves it at once, as AUTO REMOVE FINISHED BURNS does. The file also holds
    /// the default delays, so the save writes them too.
    /// </summary>
    public static void DrawDropSpentRow()
    {
        bool dropSpentStages = StagingConfig.DropSpentStages;
        if (ConsoleUi.CheckboxRow("DROP SPENT STAGES EARLY".AsSpan(), "AutoStageDropSpent".AsSpan(), ref dropSpentStages,
                ("Stage as soon as the next sequence would shed nothing but burnt-out engines, so spent boosters drop while the core stage keeps firing. Off: staging waits until every active engine is dry. Saved for every craft on a click, together with the default delays.\n" + PageText.TagEveryCraft).AsSpan()))
        {
            StagingConfig.DropSpentStages = dropSpentStages;
            SaveDefaults();
        }
    }

    public static void DrawDefaultTables(Vehicle? craft)
    {
        if (GameReflection.ModLibrary_AllParts == null)
        {
            ConsoleUi.Muted("Delay settings are unavailable on this game build.".AsSpan());
            return;
        }

        List<PartInfo> engines = GetKnownParts(ref _knownEngines, DeclaresEngine);
        List<PartInfo> decouplers = GetKnownParts(ref _knownDecouplers, DeclaresDecoupler);

        if (craft != null)
            DrawCraftParts(craft, engines, decouplers);

        if (!ConsoleUi.Fold(PageText.AllPartsLabel(engines.Count + decouplers.Count), "AutoStageAllParts", openByDefault: craft == null, PageText.DelaysTip))
            return;

        ConsoleUi.BeginRow("FILTER".AsSpan());
        string typed = _filter;
        if (ConsoleWidgets.TextLine("AutoStageFilter".AsSpan(), ref typed, pending: false, out _))
            _filter = typed.Trim();
        if (ConsoleWidgets.RowHovered)
            ConsoleUi.Tooltip("Part name or template id. Enter applies it, and an empty field shows the two lists again.".AsSpan());
        ConsoleUi.EndRow();
        string filter = _filter;

        if (filter.Length > 0)
        {
            DrawFiltered("IGNITION", engines, filter, "eng", StagingConfig.GetEngineDelay, SetEngineDelay);
            DrawFiltered("DECOUPLER", decouplers, filter, "dec", StagingConfig.GetDecouplerDelay, SetDecouplerDelay);
            return;
        }
        if (ConsoleUi.Fold("ENGINE IGNITION DELAYS", "AutoStageEngineDelays", tip: PageText.EngineDelaysTip))
            DrawDelayTable(engines, "eng", StagingConfig.GetEngineDelay, SetEngineDelay);
        if (ConsoleUi.Fold("DECOUPLER DELAYS", "AutoStageDecouplerDelays", tip: PageText.DecouplerDelaysTip))
            DrawDelayTable(decouplers, "dec", StagingConfig.GetDecouplerDelay, SetDecouplerDelay);
    }

    private static readonly Action<string, double> SetEngineDelay = static (id, v) => StagingConfig.EngineDelays[id] = v;
    private static readonly Action<string, double> SetDecouplerDelay = static (id, v) => StagingConfig.DecouplerDelays[id] = v;

    private static void DrawCraftParts(Vehicle craft, List<PartInfo> engines, List<PartInfo> decouplers)
    {
        _craftEngines.Clear();
        _craftDecouplers.Clear();
        SequenceList? list = craft.Parts?.SequenceList;
        if (list == null)
            return;
        foreach (Sequence seq in list.Sequences)
        {
            ReadOnlySpan<Part> parts = seq.Parts;
            for (int i = 0; i < parts.Length; i++)
            {
                foreach (ISequenced module in parts[i].InSequence(seq.Number))
                {
                    if (SequencedModules.Matches(module, DelayKind.Engine))
                        _craftEngines.Add(SequencedModules.DelayKey(module));
                    else if (SequencedModules.Matches(module, DelayKind.Decoupler))
                        _craftDecouplers.Add(SequencedModules.DelayKey(module));
                }
            }
        }
        DrawCraftList("IGNITION", engines, _craftEngines, "ceng", StagingConfig.GetEngineDelay, SetEngineDelay);
        DrawCraftList("DECOUPLER", decouplers, _craftDecouplers, "cdec", StagingConfig.GetDecouplerDelay, SetDecouplerDelay);
    }

    private static void DrawCraftList(string head, List<PartInfo> known, HashSet<string> keys, string idPrefix,
        Func<string, double> get, Action<string, double> set)
    {
        if (keys.Count == 0)
            return;
        ConsoleUi.SectionHead(head);
        foreach (PartInfo p in known)
        {
            if (keys.Contains(p.TemplateId))
                DrawDelayRow(in p, idPrefix, get, set);
        }
    }

    private static void DrawFiltered(string head, List<PartInfo> known, string filter, string idPrefix,
        Func<string, double> get, Action<string, double> set)
    {
        bool any = false;
        foreach (PartInfo p in known)
        {
            if (!p.DisplayName.Contains(filter, StringComparison.OrdinalIgnoreCase) && !p.TemplateId.Contains(filter, StringComparison.OrdinalIgnoreCase))
                continue;
            if (!any)
                ConsoleUi.SectionHead(head);
            any = true;
            DrawDelayRow(in p, idPrefix, get, set);
        }
    }

    /// <summary>Whether a default delay was edited since the last save, so STAGING offers SAVE DEFAULTS as its footer verb.</summary>
    public static bool Unsaved { get; private set; }

    /// <summary>Writes the defaults to autostage.toml.</summary>
    public static void SaveDefaults()
    {
        Unsaved = false;
        StagingConfig.SaveGlobalConfig();
        AfcAlerts.Info("AutoStage defaults saved", Color.Green, 2.0);
    }

    // An InputFloat rather than a drag control, so typing an exact delay and the step buttons keep working.
    private static void DrawDelayTable(List<PartInfo> parts, string idPrefix,
        Func<string, double> get, Action<string, double> set)
    {
        if (parts.Count == 0)
        {
            ConsoleUi.Muted("No matching parts loaded".AsSpan());
            return;
        }

        foreach (PartInfo p in parts)
            DrawDelayRow(in p, idPrefix, get, set);
    }

    private static void DrawDelayRow(in PartInfo p, string idPrefix, Func<string, double> get, Action<string, double> set)
    {
        float delay = (float)get(p.TemplateId);
        float field = ConsoleUi.FieldWidth;
        ConsoleUi.BeginRow(p.DisplayName.AsSpan(), field);
        bool rowHovered = ConsoleWidgets.RowHovered;
        ConsoleUi.RowRight(field);
        ImGui.SetNextItemWidth(field);
        if (ImGui.InputFloat($"###{idPrefix}_{p.TemplateId}", ref delay, 0.1f, 1.0f, "%.1f"))
        {
            set(p.TemplateId, Math.Max(0.0, (double)delay));
            Unsaved = true;
        }
        if (rowHovered)
            ConsoleUi.Tooltip((p.TemplateId + "\nThe delay of this part in every craft that has no delay of its own.\n" + PageText.TagSaveDefaults).AsSpan());
        ConsoleUi.EndRow();
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

        foreach (TransformDataReference<SubPartTemplate> subPart in template.SubPartInstances)
        {
            // The content loader resolves each sub-part reference and leaves Value null when the Ref names no sub-part template.
            SubPartTemplate? subTemplate = subPart.Value;
            if (subTemplate == null)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] Part '{template.Id}' references the sub-part '{subPart.Ref}', which does not resolve.");
                continue;
            }

            try
            {
                if (onOwnTemplate(subTemplate))
                    return true;
            }
            catch (Exception ex)
            {
                // Contained, so one malformed part cannot empty the whole table.
                DefaultCategory.Log.Warning(
                    $"[AFC] Part '{template.Id}' has the sub-part '{subPart.Ref}', which could not be read: {ex.Message}");
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
    private static readonly HashSet<string> _craftEngines = new();
    private static readonly HashSet<string> _craftDecouplers = new();
    private static string _filter = "";

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
                    raw.Add((template.Id, PartNames.Display(template.DisplayName)));
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
                    displayName = $"{name} ({PartNames.Display(lastUnderscore >= 0 ? id.Substring(lastUnderscore + 1) : id)})";
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
        Unsaved = false;
        _knownEngines = null;
        _knownDecouplers = null;
        _craftEngines.Clear();
        _craftDecouplers.Clear();
        _filter = "";
    }
}
