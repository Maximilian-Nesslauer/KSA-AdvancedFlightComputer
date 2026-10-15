using System.Reflection;
using System.Runtime.Loader;

namespace AdvancedFlightComputer.HarnessTests.Fixtures;

// KSAero, the player's aerodynamics mod, replaces stock KSA's atmospheric forces - and it patches the game in its [StarMapAllModsLoaded] hook, which a headless run never reaches: the harness runs the suite from [StarMapBeforeMain] and exits. Without it a booster flies on stock aero, which on 2stage_controlled's booster is three times the drag and no lift (Cd(a0) 3.15 against 1.04, CL(a15) 0 against 0.45 at Mach 0.39, 2026-10-08), so a glide or landing test measures a different vehicle from the one the player flies.
//
// Apply loads KSAero.dll from the game's mods folder, if it is deployed, and runs the mod's own hook; disposing runs its unload. Once per process: the hook patches under one Harmony id, and patching twice would stack its transpiler. Absent - CI, another machine - stock aero stays, and Status says so.
internal sealed class KsaeroScope : IDisposable
{
    private static object? _mod;
    private static int _users;

    public bool Active { get; }
    public string Status { get; }

    private KsaeroScope(bool active, string status)
    {
        Active = active;
        Status = status;
    }

    public static KsaeroScope Apply()
    {
        if (_mod != null)
        {
            _users++;
            return new KsaeroScope(true, "KSAero aerodynamics active (already loaded)");
        }

        string path = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
                                   "My Games", "Kitten Space Agency", "mods", "KSAero", "KSAero.dll");
        if (!File.Exists(path))
            return new KsaeroScope(false, $"KSAero not deployed ({path}) - stock aerodynamics");

        try
        {
            AssemblyLoadContext alc = AssemblyLoadContext.GetLoadContext(typeof(KsaeroScope).Assembly) ?? AssemblyLoadContext.Default;
            Assembly assembly = alc.Assemblies.FirstOrDefault(a => a.GetName().Name == "KSAero") ?? alc.LoadFromAssemblyPath(path);
            Type modType = assembly.GetType("KSAero.Mod", throwOnError: true)!;
            object mod = Activator.CreateInstance(modType)!;
            modType.GetMethod("OnFullyLoaded")!.Invoke(mod, null);
            // The hook catches its own patch failure and leaves stock aero in place; its Harmony handle is only kept on success.
            object? harmony = modType.GetField("_harmony", BindingFlags.NonPublic | BindingFlags.Static)?.GetValue(null);
            if (harmony == null)
                return new KsaeroScope(false, "KSAero's patch failed - stock aerodynamics (see the game log)");
            _mod = mod;
            _users = 1;
            return new KsaeroScope(true, $"KSAero aerodynamics active ({assembly.GetName().Version})");
        }
        catch (Exception e)
        {
            return new KsaeroScope(false, $"KSAero could not be loaded ({e.GetBaseException().Message}) - stock aerodynamics");
        }
    }

    public void Dispose()
    {
        if (!Active || _mod == null || --_users > 0)
            return;
        try
        {
            _mod.GetType().GetMethod("Unload")?.Invoke(_mod, null);
        }
        finally
        {
            _mod = null;
        }
    }
}
