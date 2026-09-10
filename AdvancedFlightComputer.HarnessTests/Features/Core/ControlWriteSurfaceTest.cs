using System.Linq;
using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Compares public FlightComputer field values around one RCS activation and cancellation.
// This shallow snapshot does not detect writes inside referenced objects or unchanged-value writes.
// Keep the field list aligned with docs/control-ownership.md by hand.
public sealed class ControlWriteSurfaceTest : AfcTest
{
    public override string Name => "afc-control-write-surface";

    private static readonly string[] Documented =
    {
        nameof(FlightComputer.RCSMode),
        nameof(FlightComputer.BurnMode),
        nameof(FlightComputer.AttitudeMode),
        nameof(FlightComputer.AttitudeFrame),
        nameof(FlightComputer.AttitudeTrackTarget),
        nameof(FlightComputer.CustomAttitudeTarget),
        nameof(FlightComputer.RollMode),
        nameof(FlightComputer.AttitudeTarget),
        nameof(FlightComputer.LastThrustTime),
    };

    // Allowed differences after release, including the known attitude-mode gap.
    private static readonly string[] NulledByRelease =
    {
        nameof(FlightComputer.AttitudeMode),
        nameof(FlightComputer.AttitudeFrame),
        nameof(FlightComputer.AttitudeTrackTarget),
        nameof(FlightComputer.AttitudeTarget),
        nameof(FlightComputer.LastThrustTime),
    };

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(RcsTestVehicles.Candidates);
        if (saves.Count == 0)
        {
            t.Skip("no RCS test vehicle save present.");
            return;
        }

        EveryDocumentedFieldExists(t);

        using RcsTestPatches.Scope patches = RcsTestPatches.Apply();
        AnRcsBurnStaysInsideTheDocumentedSet(t, home, saves[0]);
        TakingTheAttitudeLeavesNoStandingRotation(t, home, saves[0]);
    }

    private static void EveryDocumentedFieldExists(TestContext t)
    {
        string[] missing = Documented
            .Where(name => typeof(FlightComputer).GetField(name, BindingFlags.Public | BindingFlags.Instance) == null)
            .ToArray();
        t.Check("every documented field exists on this game build", missing.Length == 0,
            missing.Length == 0 ? "all resolved" : "missing " + string.Join(", ", missing));
    }

    private static void AnRcsBurnStaysInsideTheDocumentedSet(
        TestContext t, IParentBody home, string save)
    {
        RcsFlightSupport.RunOnSave(t, home, save, 500_000.0, "HarnessWriteSurface",
            (vehicle, driver) =>
            {
                FlightComputer fc = vehicle.FlightComputer;
                try
                {
                    RcsFlightSupport.CleanupBurns(fc);
                    fc.BurnMode = FlightComputerBurnMode.Manual;
                    TestSupport.SetManualControlInputs(vehicle, 0f, engineOn: false);
                    driver.Step(0.05, 40);
                    if (!RcsCapability.Probe(vehicle).HasAnyTranslation)
                    {
                        t.Skip("save has no active RCS translation capability.");
                        return;
                    }
                    // Stay inside the lead window to observe acquisition without stock simulation updates.
                    if (RcsFlightSupport.AddBurn(vehicle, driver, double3.UnitX, 1.0, 5.0) == null)
                    {
                        t.Fail("write surface setup", "no patch or loaded burn target");
                        return;
                    }

                    // Start from the setting the executor has to force, so the mode change is real.
                    fc.RCSMode = FlightComputerRCSMode.Disabled;
                    Dictionary<string, object?> before = Snapshot(fc);

                    RcsExecutor.Activate(vehicle);
                    if (!RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec) || !exec.IsActive)
                    {
                        t.Fail("write surface setup", "the RCS burn did not engage");
                        return;
                    }

                    string[] changed = Changed(before, Snapshot(fc));
                    t.Info("engage changed " + (changed.Length == 0 ? "nothing" : string.Join(", ", changed)));
                    string[] undocumented = changed.Except(Documented).ToArray();
                    t.Check("an engaged burn changes only documented fields", undocumented.Length == 0,
                        undocumented.Length == 0 ? "inside the table" : "also " + string.Join(", ", undocumented));
                    t.Check("the engage really took a documented field", changed.Length > 0);

                    FlightComputerRCSMode rcsBefore = (FlightComputerRCSMode)before[nameof(FlightComputer.RCSMode)]!;
                    FlightComputerBurnMode burnBefore = (FlightComputerBurnMode)before[nameof(FlightComputer.BurnMode)]!;
                    RcsExecutor.Cancel(vehicle, exec, "no usable translation");

                    t.Check("the release gives the modes it forced back",
                        fc.RCSMode == rcsBefore && fc.BurnMode == burnBefore);

                    // These fields do not establish the rotation rate computed on the next step.
                    t.Check("the release selects None in the burn frame",
                        fc.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.None
                        && fc.AttitudeFrame == VehicleReferenceFrame.BurnBody);

                    string[] outstanding = Changed(before, Snapshot(fc)).Except(NulledByRelease).ToArray();
                    t.Check("nothing else stays changed", outstanding.Length == 0,
                        outstanding.Length == 0 ? "clean" : "still changed " + string.Join(", ", outstanding));
                    t.Info("left by the release, as documented: "
                        + string.Join(", ", Changed(before, Snapshot(fc)).Intersect(NulledByRelease)));
                }
                finally
                {
                    VehicleControlOwnership.ReleaseAll(vehicle);
                }
            });
    }

    // Taking the attitude runs through FlightComputer.RateHold, which selects Auto and tracking None.
    // In that mode UpdateAttitudeTarget reads CustomAttitudeTarget as a rotation rate, so coordinates
    // that were there before acquisition would become a standing turn once AFC lets go.
    private static void TakingTheAttitudeLeavesNoStandingRotation(
        TestContext t, IParentBody home, string save)
    {
        RcsFlightSupport.RunOnSave(t, home, save, 500_000.0, "HarnessAttitudeRelease",
            (vehicle, driver) =>
            {
                FlightComputer fc = vehicle.FlightComputer;
                try
                {
                    RcsFlightSupport.CleanupBurns(fc);
                    fc.BurnMode = FlightComputerBurnMode.Manual;
                    TestSupport.SetManualControlInputs(vehicle, 0f, engineOn: false);
                    driver.Step(0.05, 40);
                    if (!RcsCapability.Probe(vehicle).HasAnyTranslation)
                    {
                        t.Skip("save has no active RCS translation capability.");
                        return;
                    }
                    if (RcsFlightSupport.AddBurn(vehicle, driver, double3.UnitX, 1.0, 5.0) == null)
                    {
                        t.Fail("attitude release setup", "no patch or loaded burn target");
                        return;
                    }

                    // A pointing target the player set before the burn, on the mode AFC has to give back.
                    fc.AttitudeMode = FlightComputerAttitudeMode.Manual;
                    fc.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Custom;
                    fc.CustomAttitudeTarget = new double3(0.0, 0.0, Math.PI / 2.0);

                    RcsExecutor.Activate(vehicle);
                    if (!RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec) || !exec.IsActive)
                    {
                        t.Fail("attitude release setup", "the RCS burn did not engage");
                        return;
                    }
                    t.Check("taking the attitude records the mode it replaced",
                        exec.ForcedAttitudeAuto && fc.AttitudeMode == FlightComputerAttitudeMode.Auto);

                    RcsExecutor.Cancel(vehicle, exec, "no usable translation");
                    t.Check("the release hands the attitude mode back",
                        fc.AttitudeMode == FlightComputerAttitudeMode.Manual);
                    t.Check("the release takes the custom coordinates with it",
                        fc.CustomAttitudeTarget.Equals(default(double3)));

                    // The game recomputes the target every control step, so this is what the craft
                    // actually flies with after the release.
                    driver.Step(0.05, 4);
                    double rate = fc.AttitudeTarget.RatesCci.Length();
                    t.Info($"commanded rate after the release: {rate:F6} rad/s");
                    t.Check("no standing rotation is commanded after the release", rate < 0.05,
                        $"{rate:F6} rad/s");
                }
                finally
                {
                    VehicleControlOwnership.ReleaseAll(vehicle);
                }
            });
    }

    private static Dictionary<string, object?> Snapshot(FlightComputer fc)
        => typeof(FlightComputer)
            .GetFields(BindingFlags.Public | BindingFlags.Instance)
            .ToDictionary(field => field.Name, field => field.GetValue(fc));

    private static string[] Changed(
        Dictionary<string, object?> before, Dictionary<string, object?> after)
        => after.Where(entry => !Equals(entry.Value, before[entry.Key]))
            .Select(entry => entry.Key).OrderBy(name => name, StringComparer.Ordinal).ToArray();
}
