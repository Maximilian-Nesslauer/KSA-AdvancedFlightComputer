using System;
using System.IO;
using System.Reflection;
using HarmonyLib;
using KSA;
using StarMap.API;

// StarMap entry point for the Powered Guidance mod. StarMap discovers this assembly (declared
// as the EntryAssembly in mod.toml), constructs this [StarMapMod] class, and invokes
// the lifecycle methods below.
//
// StarMap (a Harmony-based loader) provides Harmony at runtime, so we don't ship it;
// our only private deps are Gfold.Core.dll + native ecos.dll, copied into the mod
// folder beside this DLL and resolved by ResolveFromModDir / Gfold.Core's own
// DllImport resolver.
[StarMapMod]
public sealed class Mod
{
    private static Harmony _harmony;
    private static readonly string ModDir = ResolveModDir();

    private static string _lastError = "";
    private static DateTime _lastErrorTime = DateTime.MinValue;

    // Runs once on the main thread after all mods have loaded — the game and its
    // assemblies are fully available here (StarMap guarantees KSA is loaded).
    [StarMapAllModsLoaded]
    public void OnLoaded()
    {
        // Resolve our private managed dependency (Gfold.Core) from the mod folder;
        // ecos.dll is found by Gfold.Core's own DllImport resolver next to it.
        AppDomain.CurrentDomain.AssemblyResolve += ResolveFromModDir;

        try
        {
            _harmony = new Harmony("poweredguidance.ksa.integration");

            // Apply the autopilot right before the sim snapshots the flight computer:
            // a prefix on Vehicle.PrepareWorker, where FC writes reliably reach the
            // control loop (writes from the UI draw are erased by the sim copy-back).
            MethodInfo prepTarget = typeof(Vehicle).GetMethod(
                nameof(Vehicle.PrepareWorker), BindingFlags.Public | BindingFlags.Instance);
            MethodInfo prepPrefix = typeof(Mod).GetMethod(
                nameof(OnPrepareWorker), BindingFlags.NonPublic | BindingFlags.Static);
            _harmony.Patch(prepTarget, prefix: new HarmonyMethod(prepPrefix));

            // Direct gimbal control. A POSTFIX on ComputeControl, because that runs
            // after ComputeTvcControl has already allocated (or zeroed) every
            // gimbal — so our command is the last write and holds regardless of the
            // vehicle's attitude mode. See KsaGimbalControl for the details.
            MethodInfo tvcTarget = typeof(FlightComputer).GetMethod(
                nameof(FlightComputer.ComputeControl), BindingFlags.Public | BindingFlags.Instance);
            MethodInfo tvcPostfix = typeof(Mod).GetMethod(
                nameof(OnComputeControl), BindingFlags.NonPublic | BindingFlags.Static);
            _harmony.Patch(tvcTarget, postfix: new HarmonyMethod(tvcPostfix));

            Console.WriteLine("[PG] loaded via StarMap; mod dir = " + ModDir);
        }
        catch (Exception e)
        {
            Console.Error.WriteLine("[PG] load failed: " + e);
        }
    }

    // Drawn every frame inside the active ImGui frame, after the game's own GUI. The
    // viewport comes from the game (StarMap's GUI hook passes only a delta time).
    [StarMapAfterGui]
    public void DrawGui(double dt)
    {
        try
        {
            PoweredGuidanceWindow.Draw(Program.MainViewport);
        }
        catch (Exception e)
        {
            LogErrorThrottled("draw failed: ", e);
        }
    }

    [StarMapUnload]
    public void Unload()
    {
        _harmony?.UnpatchAll(_harmony.Id);
        _harmony = null;
        AppDomain.CurrentDomain.AssemblyResolve -= ResolveFromModDir;
        Console.WriteLine("[PG] unloaded.");
    }

    private static void OnPrepareWorker(Vehicle __instance)
    {
        try
        {
            PoweredGuidanceWindow.ApplyAutopilot(__instance);
        }
        catch (Exception e)
        {
            LogErrorThrottled("autopilot apply failed: ", e);
        }
    }

    // Runs on a VehicleSolvers job thread, not the main thread — keep it allocation-free
    // and non-throwing. __instance is the worker's FlightComputer COPY, which is why
    // KsaGimbalControl identifies the vehicle by VehicleConfig rather than by this.
    private static void OnComputeControl(FlightComputer __instance, ref FlightComputerOutput outputs)
    {
        try
        {
            KsaGimbalControl.OnComputeControl(__instance, ref outputs);
        }
        catch (Exception e)
        {
            LogErrorThrottled("gimbal override failed: ", e);
        }
    }

    private static Assembly ResolveFromModDir(object sender, ResolveEventArgs args)
    {
        string name = new AssemblyName(args.Name).Name + ".dll";
        string candidate = Path.Combine(ModDir, name);
        return File.Exists(candidate) ? Assembly.LoadFrom(candidate) : null;
    }

    // Where this DLL (and its siblings) live. Prefer the loaded assembly's location;
    // fall back to the known install path if StarMap loaded us from bytes (Location
    // empty), so Gfold.Core/ecos still resolve.
    private static string ResolveModDir()
    {
        string dir = Path.GetDirectoryName(typeof(Mod).Assembly.Location);
        if (!string.IsNullOrEmpty(dir) && File.Exists(Path.Combine(dir, "Gfold.Core.dll")))
            return dir;
        return Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
            "My Games", "Kitten Space Agency", "mods", "PoweredGuidance");
    }

    // Rate-limit recurring exceptions (an error every frame would flood the console
    // and drag the frame rate down).
    private static void LogErrorThrottled(string prefix, Exception e)
    {
        string msg = prefix + e.Message;
        DateTime now = DateTime.UtcNow;
        if (msg == _lastError && (now - _lastErrorTime).TotalSeconds < 5)
            return;
        _lastError = msg;
        _lastErrorTime = now;
        Console.Error.WriteLine("[PG] " + prefix + e);
    }
}
