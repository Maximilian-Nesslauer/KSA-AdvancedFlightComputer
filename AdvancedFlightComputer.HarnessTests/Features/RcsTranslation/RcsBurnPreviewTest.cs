using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class RcsBurnPreviewTest : AfcTest
{
    public override string Name => "afc-rcs-burn-preview";

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
        using RcsTestPatches.Scope patches = RcsTestPatches.Apply();
        foreach (string save in saves)
            RcsFlightSupport.RunOnSave(t, home, save, 500_000.0, "HarnessRcsPreview",
                (vehicle, driver) =>
                {
                    RcsExecutor.ResetUiCache();
                    FlightComputer fc = vehicle.FlightComputer;
                    RcsFlightSupport.CleanupBurns(fc);
                    fc.BurnMode = FlightComputerBurnMode.Manual;
                    TestSupport.SetManualControlInputs(vehicle, 0f, engineOn: false);
                    driver.Step(0.05, 40);
                    if (!RcsCapability.Probe(vehicle).HasAnyTranslation)
                    {
                        t.Skip("save has no active RCS translation capability.");
                        return;
                    }
                    var first = RcsFlightSupport.AddBurn(vehicle, driver, double3.UnitX, 0.5, 20.0);
                    var later = RcsFlightSupport.AddBurn(vehicle, driver, double3.UnitY, 1.0, 120.0);
                    if (first == null || later == null)
                    {
                        t.Fail("preview setup", "no patch or loaded burn target");
                        return;
                    }
                    BurnTarget? loaded = fc.Burn;
                    float3 accumulated = loaded!.DeltaVAccumCci;
                    float3 target = loaded.DeltaVTargetCci;
                    RcsExecution exec = new() { SaveId = string.Empty, VehicleId = vehicle.Id };
                    exec.Estimates = new RcsEstimates { Valid = true, HoldDurationSec = -123.0 };
                    bool valid = RcsBurnPreview.TryGetEstimates(later.Burn, vehicle, fc, exec,
                        out RcsEstimates estimate, out bool currentVehicle);
                    t.Check("later burn uses current vehicle preview", valid && currentVehicle);
                    RcsCapabilitySnapshot cap = RcsExecutor.ProbeCached(vehicle);
                    RcsEstimates expected = RcsExecutor.ComputeEstimates(vehicle,
                        BurnTarget.CreateFromBurn(later.Burn), in cap);
                    t.CheckAbs("planned direction uses stock target", estimate.HoldDurationSec,
                        expected.HoldDurationSec, 1e-6);
                    t.Check("preview retains loaded target", ReferenceEquals(loaded, fc.Burn));
                    t.Check("preview retains accumulated impulse", loaded.DeltaVAccumCci.Equals(accumulated));
                    t.Check("preview retains loaded direction", loaded.DeltaVTargetCci.Equals(target));
                    later.Burn.DeltaVVlf *= 2.0;
                    RcsBurnPreview.TryGetEstimates(later.Burn, vehicle, fc, exec,
                        out RcsEstimates edited, out _);
                    t.CheckAbs("edited delta V refreshes duration", edited.AlignDurationSec,
                        2.0 * estimate.AlignDurationSec, 1e-5);
                    later.Burn.Time = first.Burn.Time + 0.25;
                    bool closeValid = RcsBurnPreview.TryGetEstimates(later.Burn, vehicle, fc, exec,
                        out _, out bool closeCurrentVehicle);
                    t.Check("close later burn keeps preview identity", closeValid && closeCurrentVehicle);
                    t.Check("preview retains executor estimates", exec.Estimates.HoldDurationSec == -123.0);
                    RcsFlightSupport.CleanupBurns(fc);
                });
    }
}
