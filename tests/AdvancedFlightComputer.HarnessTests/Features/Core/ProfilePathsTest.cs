using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoRemove;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Framework;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Every file that AFC reads or writes in the player profile must be under the profile folder of the game, Constants.DocumentsFolderPath.
// A launcher can move that folder (StarMap reads STARMAP_INSTANCE_PATH), so a path built from the Documents folder of the user would read and write another profile.
// The test reads the paths that the mod computed, so it also shows that the redirected folder reaches the code of the mod.
// The harness does not run Mod.OnFullyLoaded, so a path is set only when an earlier test initialised its feature; an empty path is a skip.
public sealed class ProfilePathsTest : AfcTest
{
    public override string Name => "afc-profile-paths";

    protected override void Execute(TestContext t)
    {
        string profile = Full(Constants.DocumentsFolderPath);
        string userDocuments = Full(Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "My Games", "Kitten Space Agency"));
        t.Info($"profile folder: {profile}");

        string? instance = Environment.GetEnvironmentVariable("STARMAP_INSTANCE_PATH");
        if (string.IsNullOrEmpty(instance))
        {
            t.Info("no STARMAP_INSTANCE_PATH, so the profile is the default one and every path below is also under Documents");
        }
        else
        {
            t.Check("the profile folder is the instance folder", SamePath(profile, Full(instance)),
                $"profile '{profile}', instance '{instance}'");
            t.Check("the profile folder is not the Documents profile", !SamePath(profile, userDocuments));
        }

        string modFolder = Path.Combine(ModLibrary.LocalModsFolderPath, "AdvancedFlightComputer");
        CheckField(t, profile, typeof(StagingConfig), "_configPath", "autostage.toml");
        CheckField(t, profile, typeof(StagingConfig), "_vehiclesDir", "autostage-vehicles");
        CheckField(t, profile, typeof(MultiPassRegistry), "_configPath", "multipass.toml");
        CheckField(t, profile, typeof(RcsExecRegistry), "_configPath", "rcs-exec.toml");
        CheckField(t, profile, typeof(AutoRemoveConfig), "_configPath", "autoremove.toml");
        CheckField(t, profile, typeof(UiConfig), "_path", "ui.toml");
        t.Check("the mods folder of the game is under the profile folder", IsUnder(Full(modFolder), profile), modFolder);

        CheckSixDofLog(t, profile);
    }

    private static void CheckField(TestContext t, string profile, Type type, string field, string leaf)
    {
        FieldInfo? info = type.GetField(field, BindingFlags.NonPublic | BindingFlags.Static);
        if (!t.Check($"{type.Name}.{field} exists", info != null))
            return;
        string? value = info!.GetValue(null) as string;
        if (string.IsNullOrEmpty(value))
        {
            t.Skip($"{type.Name}.{field} is empty: the harness does not run Mod.OnFullyLoaded, and no earlier test initialised it.");
            return;
        }
        t.Info($"{type.Name}.{field} = {value}");
        t.Check($"{leaf} is under the profile folder", IsUnder(Full(value), profile), value);
        t.Check($"{leaf} is in the AFC mod folder", SamePath(
            Full(Path.GetDirectoryName(value) ?? ""),
            Full(Path.Combine(ModLibrary.LocalModsFolderPath, "AdvancedFlightComputer"))), value);
    }

    // The 6-DOF telemetry folder is set when a run starts, so the test starts a run with no rows and removes the files that the run wrote.
    private static void CheckSixDofLog(TestContext t, string profile)
    {
        if (SixDofLog.Enabled)
        {
            t.Skip("a 6-DOF log run is open, so the test does not start one.");
            return;
        }
        object owner = new();
        if (!t.Check("a 6-DOF log run starts", SixDofLog.Start(owner, "afc-profile-paths", "none"), SixDofLog.LastError))
            return;
        string directory = SixDofLog.Directory;
        string run = SixDofLog.RunName;
        SixDofLog.Stop(owner);
        t.Info($"6-DOF log folder: {directory}");
        t.Check("the 6-DOF log folder is the navbox-logs folder of the profile",
            SamePath(Full(directory), Full(Path.Combine(profile, "navbox-logs"))), directory);
        foreach (string suffix in new[] { "-cycle.csv", "-plan.csv", "-events.log" })
        {
            string file = Path.Combine(directory, run + suffix);
            try
            {
                if (File.Exists(file)) File.Delete(file);
            }
            catch (IOException e)
            {
                t.Info($"could not remove {file}: {e.Message}");
            }
        }
    }

    private static string Full(string path)
        => Path.TrimEndingDirectorySeparator(Path.GetFullPath(path));

    private static bool SamePath(string a, string b)
        => string.Equals(a, b, StringComparison.OrdinalIgnoreCase);

    private static bool IsUnder(string path, string folder)
        => path.StartsWith(folder + Path.DirectorySeparatorChar, StringComparison.OrdinalIgnoreCase);
}
