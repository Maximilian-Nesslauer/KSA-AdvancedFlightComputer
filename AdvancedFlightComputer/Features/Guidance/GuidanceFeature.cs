using System.Reflection;
using AdvancedFlightComputer.Core;
using KSA;
using Brutal.ImGuiApi;
using Brutal.Logging;
using HarmonyLib;

namespace AdvancedFlightComputer.Features.Guidance;

internal static class GuidanceFeature
{
    internal const string UnavailableReason = "Guidance is unavailable until control ownership is integrated.";

    private static bool _resolverRegistered;
    private static bool _menuFailed;

    internal static void ApplyPatches(Harmony harmony)
    {
        if (!_resolverRegistered)
        {
            AppDomain.CurrentDomain.AssemblyResolve += ResolveManagedLibrary;
            _resolverRegistered = true;
        }

        // The control code stays unreachable until all execution paths share vehicle ownership.
        harmony.Patch(GameReflection.Program_DrawProgramMenusHook!,
            postfix: new HarmonyMethod(typeof(GuidanceFeature), nameof(DrawMenu)));

        // Clear guidance state after a save replaces the vehicles.
        SaveLoadObserver.SaveLoaded += GuidanceWindow.ReleaseAllVehicles;
        DefaultCategory.Log.Warning($"[AFC] {UnavailableReason}");
    }

    private static void DrawMenu()
    {
        if (_menuFailed)
            return;

        try
        {
            if (!ImGui.BeginMenu("AFC Guidance"u8))
                return;
            try
            {
                ImGui.Text(UnavailableReason);

                // Keep release errors visible while the guidance panel is hidden.
                string failure = GuidanceWindow.ReleaseFailure(Program.ControlledVehicle);
                if (failure.Length > 0)
                    ImGui.Text(failure);
            }
            finally
            {
                ImGui.EndMenu();
            }
        }
        catch (Exception ex)
        {
            _menuFailed = true;
            DefaultCategory.Log.Warning($"[AFC] Guidance diagnostic menu failed: {ex}");
        }
    }

    private static Assembly? ResolveManagedLibrary(object? sender, ResolveEventArgs args)
    {
        string? name = new AssemblyName(args.Name).Name;
        if (name is not ("AdvancedFlightComputer.Guidance.Numerics"
            or "AdvancedFlightComputer.Guidance.Gfold"
            or "AdvancedFlightComputer.Guidance.Scvx"))
            return null;

        string? directory = Path.GetDirectoryName(typeof(GuidanceFeature).Assembly.Location);
        if (string.IsNullOrEmpty(directory))
            return null;

        string path = Path.Combine(directory, name + ".dll");
        return File.Exists(path) ? Assembly.LoadFrom(path) : null;
    }

    internal static void Reset()
    {
        SaveLoadObserver.SaveLoaded -= GuidanceWindow.ReleaseAllVehicles;

        // On unload, the patches are already removed and no later step can retry cleanup.
        GuidanceWindow.ReleaseAllVehicles();
        if (_resolverRegistered)
            AppDomain.CurrentDomain.AssemblyResolve -= ResolveManagedLibrary;
        _resolverRegistered = false;
        _menuFailed = false;
    }
}
