using System.Reflection;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Drives the guidance staging cue against the real AutoStage switch on a spawned craft. Guidance arms the switch once per flight and then leaves it to the player. The detector never ticks here, so a staging request is counted instead of flown.
public sealed class GuidanceStagingArmTest : AfcTest
{
    public override string Name => "afc-guidance-staging-arm";

    private const BindingFlags PrivateStatic = BindingFlags.NonPublic | BindingFlags.Static;

    private static double _now;
    private static int _requests;

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        VehicleSave? save = DefaultVehicleSaves.FindSave("Rocket");
        if (save?.VehicleSaveData.RootPartInstance == null)
        {
            t.Fail("default vehicle", "Rocket is not available");
            return;
        }

        FieldInfo ambient = typeof(GuidanceWindow).GetField("_s", PrivateStatic)!;
        object? previousAmbient = ambient.GetValue(null);
        double cooldown = (double)typeof(GuidanceWindow).GetField("SequenceCooldown", PrivateStatic)!.GetRawConstantValue()!;
        var harmony = new Harmony("com.maxi.afc.harnesstests.guidance.stagingarm");
        Vehicle? craft = null;
        using AutoStageTestPatches.Scope patches = AutoStageTestPatches.Apply();
        try
        {
            harmony.Patch(Method("SimNow"), prefix: Prefix(nameof(Clock)));
            harmony.Patch(Method("ShouldStageForReserve"), prefix: Prefix(nameof(NoReserve)));
            harmony.Patch(AccessTools.Method(typeof(StagingDetector), nameof(StagingDetector.RequestStaging)),
                prefix: Prefix(nameof(CountRequest)));

            craft = VehicleFixtures.SpawnDesign(t.System, home, save.VehicleSaveData.RootPartInstance,
                "AfcStagingArm_" + Guid.NewGuid().ToString("N"),
                OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime()));

            TheFirstStepArmsAndADisarmHolds(t, ambient, craft, cooldown);
            TheReleaseDisarmsWhatGuidanceArmed(t, ambient, craft, cooldown);
            ACraftThePlayerArmedStaysArmed(t, ambient, craft, cooldown);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            ambient.SetValue(null, previousAmbient);
            if (craft != null)
                VehicleSpawner.Despawn(craft);
        }
    }

    private static void TheFirstStepArmsAndADisarmHolds(TestContext t, FieldInfo ambient, Vehicle craft, double cooldown)
    {
        StagingDetector.Arm(craft, false);
        var state = new VehicleAutopilotState();
        _requests = 0;
        double time = 100;

        Cue(ambient, state, craft, time);
        t.Check("the first staging step arms a disarmed craft", StagingDetector.IsArmed(craft) && state.ArmedStaging);
        t.Check("the armed craft takes the staging cue", _requests == 1, $"requests={_requests}");

        // The player switches it off, and every later step past the cooldown has to leave it off.
        StagingDetector.Arm(craft, false);
        for (int step = 0; step < 5; step++)
        {
            time += 2 * cooldown;
            Cue(ambient, state, craft, time);
        }
        t.Check("a player disarm holds on every later step", !StagingDetector.IsArmed(craft));
        t.Check("no staging is requested while the player holds it off", _requests == 1, $"requests={_requests}");
        t.Check("the status says the player switched it off", state.Status.Contains("switched off"), state.Status);

        StagingDetector.Arm(craft, true);
        time += 2 * cooldown;
        Cue(ambient, state, craft, time);
        t.Check("a player re-arm is used as it is", StagingDetector.IsArmed(craft) && _requests == 2, $"requests={_requests}");

        Release(ambient, state, craft);
        t.Check("the release leaves the player's own re-arm on", StagingDetector.IsArmed(craft));
    }

    private static void TheReleaseDisarmsWhatGuidanceArmed(TestContext t, FieldInfo ambient, Vehicle craft, double cooldown)
    {
        StagingDetector.Arm(craft, false);
        var state = new VehicleAutopilotState();
        Cue(ambient, state, craft, 200);
        Release(ambient, state, craft);
        t.Check("the release disarms a craft guidance armed", !StagingDetector.IsArmed(craft));

        Cue(ambient, state, craft, 200 + 2 * cooldown);
        t.Check("the next flight arms the craft again", StagingDetector.IsArmed(craft) && state.ArmedStaging);
        Release(ambient, state, craft);
        t.Check("that flight's release disarms it again", !StagingDetector.IsArmed(craft));
    }

    private static void ACraftThePlayerArmedStaysArmed(TestContext t, FieldInfo ambient, Vehicle craft, double cooldown)
    {
        StagingDetector.Arm(craft, true);
        var state = new VehicleAutopilotState();
        Cue(ambient, state, craft, 300);
        t.Check("a craft the player armed is left as it is", StagingDetector.IsArmed(craft) && !state.ArmedStaging);

        StagingDetector.Arm(craft, false);
        Cue(ambient, state, craft, 300 + 2 * cooldown);
        Cue(ambient, state, craft, 300 + 4 * cooldown);
        t.Check("a disarm holds on a craft the player armed", !StagingDetector.IsArmed(craft));

        StagingDetector.Arm(craft, true);
        Cue(ambient, state, craft, 300 + 6 * cooldown);
        Release(ambient, state, craft);
        t.Check("the release leaves a craft the player armed on", StagingDetector.IsArmed(craft));
        StagingDetector.Arm(craft, false);
    }

    private static void Cue(FieldInfo ambient, VehicleAutopilotState state, Vehicle craft, double time)
    {
        _now = time;
        ambient.SetValue(null, state);
        Method("AutoSequence").Invoke(null, [craft]);
    }

    private static void Release(FieldInfo ambient, VehicleAutopilotState state, Vehicle craft)
    {
        ambient.SetValue(null, state);
        Method("HandBackVehicle").Invoke(null, [craft]);
    }

    private static MethodInfo Method(string name) =>
        typeof(GuidanceWindow).GetMethod(name, PrivateStatic)
        ?? throw new MissingMethodException(nameof(GuidanceWindow), name);

    private static HarmonyMethod Prefix(string name) => new(typeof(GuidanceStagingArmTest), name);

    private static bool Clock(ref double __result)
    {
        __result = _now;
        return false;
    }

    private static bool NoReserve(ref bool __result)
    {
        __result = false;
        return false;
    }

    private static bool CountRequest(ref StagingDetector.StagingRequest __result)
    {
        _requests++;
        __result = StagingDetector.StagingRequest.Queued;
        return false;
    }
}
