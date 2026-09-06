using Brutal.Logging;
using HarmonyLib;

namespace AdvancedFlightComputer.Core;

internal sealed class FeaturePatchSet(string idPrefix)
{
    private readonly List<Harmony> _owners = new();

    internal bool TryApply(string feature, Action<Harmony> apply)
    {
        Harmony harmony = new($"{idPrefix}.{feature}");
        _owners.Add(harmony);
        try
        {
            apply(harmony);
            return true;
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] {feature} patching failed. Removing its patches: {ex}");
            if (TryUnpatch(harmony))
                _owners.Remove(harmony);
            return false;
        }
    }

    internal void UnpatchAll()
    {
        for (int i = _owners.Count - 1; i >= 0; i--)
        {
            if (TryUnpatch(_owners[i]))
                _owners.RemoveAt(i);
        }
    }

    private static bool TryUnpatch(Harmony harmony)
    {
        try
        {
            harmony.UnpatchAll(harmony.Id);
            return true;
        }
        catch (Exception ex)
        {
            // Keep the owner so unload can retry without leaving other feature owners unprocessed.
            DefaultCategory.Log.Warning($"[AFC] Could not remove patches for '{harmony.Id}': {ex}");
            return false;
        }
    }
}
