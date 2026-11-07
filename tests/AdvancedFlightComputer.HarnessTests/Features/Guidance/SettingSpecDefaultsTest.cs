using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.HarnessTests.Framework;

namespace AdvancedFlightComputer.HarnessTests;

// Every guidance setting spec names a real field of VehicleAutopilotState, and its default is that field's initializer, so the CHANGED counts and the resets of the console agree with a fresh craft.
public sealed class SettingSpecDefaultsTest : AfcTest
{
    public override string Name => "afc-ui-setting-defaults";

    protected override void Execute(TestContext t)
    {
        var fresh = new VehicleAutopilotState();

        foreach (SettingSpec spec in GuidanceSettingSpecs.AllDoubles)
        {
            if (!SettingAccess.TryGetDouble(fresh, spec.Key, out double value))
            {
                t.Fail("resolves " + spec.Key, "no double or int field of that name");
                continue;
            }
            t.Check("default of " + spec.Key, !SettingMath.IsChanged(spec, value), $"field {value}, spec {spec.Default}");
        }

        foreach (BoolSpec spec in GuidanceSettingSpecs.AllBools)
        {
            if (!SettingAccess.TryGetBool(fresh, spec.Key, out bool value))
            {
                t.Fail("resolves " + spec.Key, "no bool field of that name");
                continue;
            }
            t.Check("default of " + spec.Key, value == spec.Default, $"field {value}, spec {spec.Default}");
        }

        t.Check("a fresh craft has no changed tuning value",
            SettingAccess.CountChanged(fresh, GuidanceSettingSpecs.AllTuning, GuidanceSettingSpecs.AllTuningBools) == 0);

        // A measured vehicle height is the craft's, not the player's change, until the measurement is turned off.
        var measured = new VehicleAutopilotState { VehicleHeightM = 23.0 };
        t.Check("a measured vehicle height is not counted",
            SettingAccess.CountChanged(measured, new[] { GuidanceSettingSpecs.VehicleHeightM }) == 0);
        measured.VehicleHeightAuto = false;
        t.Check("a typed vehicle height is counted",
            SettingAccess.CountChanged(measured, new[] { GuidanceSettingSpecs.VehicleHeightM }) == 1);

        var edited = new VehicleAutopilotState();
        SettingAccess.SetDouble(edited, GuidanceSettingSpecs.GlideKp.Key, GuidanceSettingSpecs.GlideKp.Default + 1.0);
        t.Check("an edit counts as changed",
            SettingAccess.CountChanged(edited, GuidanceSettingSpecs.TuningBoostback) == 1);
        SettingAccess.ResetToDefaults(edited, GuidanceSettingSpecs.TuningBoostback);
        t.Check("a reset clears the change",
            SettingAccess.CountChanged(edited, GuidanceSettingSpecs.TuningBoostback) == 0);
    }
}
