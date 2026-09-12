using System.Reflection;
using System.Reflection.Emit;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Core;

// The one owner of the Mods settings page hook. Every settings page renders into one body child
// closed by a single ConsoleStyle.PopWidgetStyle, so a call inserted before it lands inside the
// body with the widget style still pushed. Features register a section; nothing stock is replaced,
// so other mods can do the same.
[HarmonyPatch(typeof(GameSettings), nameof(GameSettings.OnDrawUi), new[] { typeof(Camera) })]
internal static class ModSettingsPage
{
    private static readonly List<Action> _sections = new();

    private static readonly MethodInfo? Anchor =
        AccessTools.Method(typeof(ConsoleStyle), nameof(ConsoleStyle.PopWidgetStyle), Type.EmptyTypes);

    internal static bool IsAnchorPresent
    {
        get
        {
            MethodBase? target = AccessTools.Method(typeof(GameSettings), nameof(GameSettings.OnDrawUi), new[] { typeof(Camera) });
            return target != null && FindAnchor(PatchProcessor.GetOriginalInstructions(target)) >= 0;
        }
    }

    internal static void ApplyPatches(Harmony harmony)
        => harmony.CreateClassProcessor(typeof(ModSettingsPage)).Patch();

    internal static void Register(Action section)
    {
        if (!_sections.Contains(section))
            _sections.Add(section);
    }

    internal static void Unregister(Action section) => _sections.Remove(section);

    internal static void Reset() => _sections.Clear();

    static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
    {
        var codes = new List<CodeInstruction>(instructions);
        int anchorIdx = FindAnchor(codes);
        if (anchorIdx < 0)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] Settings page: no ConsoleStyle.PopWidgetStyle call in GameSettings.OnDrawUi ({codes.Count} instructions), mod sections not drawn.");
            return codes;
        }

        // Labels stay on the anchor, so a jump to it skips the sections instead of landing mid-call.
        codes.Insert(anchorIdx, new CodeInstruction(OpCodes.Call,
            AccessTools.Method(typeof(ModSettingsPage), nameof(DrawSections))));
        return codes;
    }

    private static int FindAnchor(List<CodeInstruction> codes)
    {
        if (Anchor == null)
            return -1;
        for (int i = 0; i < codes.Count; i++)
        {
            if (codes[i].Calls(Anchor))
                return i;
        }
        return -1;
    }

    private static bool IsModsPageOpen()
        => GameReflection.GameSettings_openTab_Mods is { } mods
           && mods.Equals(GameReflection.GameSettings_openTab!.GetValue(null));

    public static void DrawSections()
    {
        if (!IsModsPageOpen())
            return;
        foreach (Action section in _sections)
        {
            try
            {
                section();
            }
            catch (Exception ex)
            {
                string owner = section.Method.DeclaringType?.Name ?? section.Method.Name;
                LogHelper.WarnOnce($"settings-section:{owner}:{ex.GetType().Name}", $"[AFC] Settings section {owner} failed to draw: {ex}");
            }
        }
    }
}
