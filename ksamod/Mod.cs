using System;
using System.IO;
using System.Reflection;
using HarmonyLib;
using Brutal.ImGuiApi;
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

            // The target's TURNING RATE, into the one window where it can be set:
            // after UpdateAttitudeTarget has built AttitudeTarget from our
            // CustomAttitudeTarget, and before UpdateAttitudeError reads it. Private
            // and instance, hence AccessTools rather than GetMethod. See
            // KsaAttitudeRate for what goes in and why the FC needs it.
            MethodInfo rateTarget = AccessTools.Method(
                typeof(FlightComputer), "UpdateAttitudeTarget");
            MethodInfo ratePostfix = typeof(Mod).GetMethod(
                nameof(OnUpdateAttitudeTarget), BindingFlags.NonPublic | BindingFlags.Static);
            if (rateTarget != null)
                _harmony.Patch(rateTarget, postfix: new HarmonyMethod(ratePostfix));
            else
                Console.Error.WriteLine("[PG] FlightComputer.UpdateAttitudeTarget not found "
                                      + "- steering rate feedforward is OFF.");

            // OUR ENTRY IN THE GAME'S MENU BAR. Program.DrawProgramMenusHook is an
            // empty two-byte stub - literally just a ret - called by Program.DrawMenuBar
            // as its second-to-last call, immediately before EndMenuBar. That is the
            // game's own extension point for exactly this, so a postfix on it runs
            // INSIDE the menu bar and can open a menu without any of the fragility of
            // patching DrawMenuBar itself (6 KB of IL, and a transpiler would have to
            // find a spot in it).
            MethodInfo menuTarget = typeof(Program).GetMethod(
                nameof(Program.DrawProgramMenusHook), BindingFlags.Public | BindingFlags.Instance);
            MethodInfo menuPostfix = typeof(Mod).GetMethod(
                nameof(OnDrawProgramMenus), BindingFlags.NonPublic | BindingFlags.Static);
            _harmony.Patch(menuTarget, postfix: new HarmonyMethod(menuPostfix));

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

    /// <summary>
    /// Draws the mod's own menu into the game's menu bar. Runs inside
    /// BeginMenuBar/EndMenuBar via the postfix on Program.DrawProgramMenusHook, so
    /// BeginMenu is legal here and nowhere else.
    ///
    /// The switch is the whole point of the feature: someone who is not flying a guided
    /// descent should not have a panel over their view, and "off" has to mean the mod
    /// is not touching their vehicle either. Toggling it back on is non-destructive -
    /// the panel returns and nothing was thrown away - but the craft has been handed
    /// back in the meantime, so anything that was engaged has to be engaged again.
    /// </summary>
    private static void OnDrawProgramMenus()
    {
        try
        {
            if (!ImGui.BeginMenu("PoweredGuidance", true))
                return;

            bool active = PoweredGuidanceWindow.ModActive;
            if (ImGui.MenuItem("Enabled", "", ref active, true))
                PoweredGuidanceWindow.SetModActive(active);

            ImGui.EndMenu();
        }
        catch (Exception e)
        {
            // A throw here would unbalance the game's OWN menu bar, not ours - the
            // BeginMenuBar above us is theirs - so this must never propagate. If
            // BeginMenu succeeded and the throw came after, EndMenu is skipped and
            // ImGui complains for one frame; that is still better than taking the
            // whole menu bar down every frame.
            LogErrorThrottled("menu draw failed: ", e);
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

    // Runs on a VehicleSolvers job thread against the worker's FlightComputer COPY,
    // like OnComputeControl below. Deliberately does NOT bail when the mod is switched
    // off: KsaAttitudeRate publishes nothing unless a vehicle is engaged, and the
    // hand-back clears it, so there is no state here for an off switch to strand.
    private static void OnUpdateAttitudeTarget(FlightComputer __instance)
    {
        try
        {
            KsaAttitudeRate.OnUpdateAttitudeTarget(__instance);
        }
        catch (Exception e)
        {
            LogErrorThrottled("attitude rate feedforward failed: ", e);
        }
    }

    // Runs on a VehicleSolvers job thread, not the main thread — keep it allocation-free
    // and non-throwing. __instance is the worker's FlightComputer COPY, which is why
    // KsaGimbalControl identifies the vehicle by VehicleConfig rather than by this.
    private static void OnComputeControl(FlightComputer __instance, ref FlightComputerOutput outputs)
    {
        // Switched off: stop driving the nozzles on the very next control step rather
        // than waiting for the per-vehicle hand-back. HandBackVehicle disengages the
        // override properly a step later, but that step runs on a different thread, and
        // "off" should not leave the mod steering anything for even one of them.
        if (!PoweredGuidanceWindow.ModActive)
            return;

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
