using System.Globalization;
using System.Reflection;
using System.Text;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Transfer;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The trajectory that the TRANSFER page shows before CREATE must be the trajectory that stock
// propagates for the burn that CREATE queues, for the flyby departure and for the plain transfer.
// A craft in a low orbit of the home body plans to the first moon with the AFC porkchop, reads the
// preview plan, creates the burn and steps the simulation so that stock computes the flight plan of
// the burn in BurnPlan.CalculateNewFlightPlansFromFlightComputerOnly.
//
// The two plans can only agree when the preview is built with the phases that stock gives the burn.
// PhysicsBubble.UpdateStagedBurn fills FlightComputer.StagedBurn from the first sequence that is not
// activated while FlightComputer.ActiveEnginePerformanceMax has no thrust, and from the active
// sequence once it has thrust, but then only on a replan, which a craft without burns does not get
// before a burn is added. So the test reads the phases that
// AFC plans with before CREATE and compares them with the stock phases after CREATE, and it
// requires for the staged craft with engines on that the stock list before CREATE is another one,
// which is the state where a preview built on the stock list shows a trajectory that is not flown.
//
// The cases are the three-stage Rocket with its engines on (a low and a high flyby, and the plain
// transfer), the same craft with every engine switched off, where stock plans with the next stage
// before and after CREATE, the same craft with engines on and no active sequence, and the other
// default craft that are present. The test also checks that an
// unchanged craft and selection cost no rebuild of the preview, and that CREATE refuses a flyby that
// cannot arm and makes no burn aimed at the centre of the moon.
public sealed class FlybyCreateMatchesPreviewTest : AfcTest
{
    private static readonly TimeSpan CalculationTimeout = TimeSpan.FromMinutes(3);
    private static readonly TimeSpan RefineTimeout = TimeSpan.FromMinutes(1);
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private const double ParkingAltitudeM = 200_000.0;
    private const double StepDt = 1.0;
    private const int SettleSteps = 3;
    private const int PlanSteps = 4;
    private const int IdleFrames = 20;
    private const double PeriapsisTolM = 25_000.0;
    private const double EncounterTimeTolS = 120.0;
    // The flown periapsis must meet the request this closely, so a low flyby stays clear of the surface.
    private const double RequestRelTol = 0.02;
    private const double PhaseRelTol = 1e-6;
    // Below the terrain of the moon, so the flyby cannot arm.
    private const double RefusedAltitudeKm = 0.5;

    // Destination null is the first moon of the home body. A planet is a transfer across parents, from the low orbit to another body of the star.
    // Staged false spawns the craft without the active sequence of its save, the state of a craft whose engines the player switched on in their part windows without staging. SequencePerformanceList.FillStagedBurn(fromActive: true) then finds no sequence, and stock plans the burn with one phase of the running engines.
    private readonly record struct Case(string Craft, bool EnginesOn, double? FlybyKm, string? Destination = null, bool Staged = true);

    public override string Name => "afc-flyby-create-matches-preview";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        Celestial? moon = TestWorld.FindMoon(home);
        if (moon == null || DefaultVehicleSaves.FindSave("Rocket") == null)
        {
            t.Skip("a home moon and the Rocket default save are required.");
            return;
        }

        var cases = new List<Case>
        {
            new("Rocket", true, 100.0),
            new("Rocket", true, 10_000.0),
            new("Rocket", true, null),
            new("Rocket", false, 100.0),
            new("Rocket", true, 100.0, Staged: false),
            new("Rocket", true, null, Staged: false),
        };
        if (t.System.Get("Mars") is Celestial)
        {
            cases.Add(new Case("Rocket", true, 1_000.0, "Mars"));
            cases.Add(new Case("Rocket", true, null, "Mars"));
        }
        foreach (string other in new[] { "Gemini7", "Polaris" })
        {
            if (DefaultVehicleSaves.FindSave(other) != null)
                cases.Add(new Case(other, true, 100.0));
        }

        bool wasFlyby = HohmannFlybyUI.Enabled;
        Vehicle? originalControlled = Program.ControlledVehicle;
        Vehicle? vehicle = null;
        TransferSession.CancelAll();
        HohmannFlybyUI.Reset();
        HohmannMultiPassUI.Reset();
        try
        {
            foreach (Case c in cases)
            {
                vehicle = Respawn(t, home, moon, vehicle, c);
                Celestial destination = c.Destination == null ? moon : (Celestial)t.System.Get(c.Destination)!;
                RunCase(t, t.Session.CreateDriver(), vehicle, destination, c);
            }
            vehicle = Respawn(t, home, moon, vehicle, new Case("Rocket", true, RefusedAltitudeKm));
            RunRefusal(t, t.Session.CreateDriver(), vehicle, moon);
        }
        finally
        {
            Program.ControlledVehicle = originalControlled;
            TransferSession.CancelAll();
            HohmannFlybyUI.Reset();
            HohmannFlybyUI.Enabled = wasFlyby;
            HohmannMultiPassUI.Reset();
            if (vehicle != null)
                VehicleSpawner.Despawn(vehicle);
        }
    }

    private static Vehicle Respawn(TestContext t, IParentBody home, Celestial moon, Vehicle? old, Case c)
    {
        TransferSession.CancelAll();
        HohmannFlybyUI.Reset();
        if (old != null)
            VehicleSpawner.Despawn(old);
        VehicleSave save = DefaultVehicleSaves.FindSave(c.Craft)!;
        string id = "FlybyCreate_" + Guid.NewGuid().ToString("N");
        Orbit parking = ParkingInMoonPlane(home, moon, Universe.GetElapsedTime());
        Vehicle vehicle = c.Staged
            ? VehicleFixtures.SpawnFromSaveData(t.System, home, save.VehicleSaveData, id, parking)
            : VehicleFixtures.SpawnDesign(t.System, home, save.VehicleSaveData.RootPartInstance!, id, parking);
        if (!c.EnginesOn)
        {
            MethodInfo setActive = AccessTools.PropertySetter(typeof(EngineController), nameof(EngineController.IsActive));
            foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
                setActive.Invoke(engine, new object[] { false });
        }
        // PhysicsBubble.UpdateStagedBurn keeps the staged phases only for the controlled craft or a craft with burns.
        Program.ControlledVehicle = vehicle;
        t.Session.CreateDriver().Step(StepDt, SettleSteps);
        return vehicle;
    }

    private static void RunCase(TestContext t, SimDriver driver, Vehicle vehicle, Celestial moon, Case c)
    {
        string label = string.Format(Inv, "{0}{1}{2} to {3}, {4}", c.Craft, c.EnginesOn ? "" : " engines off", c.Staged ? "" : " never staged", moon.Id,
            c.FlybyKm is double km ? string.Format(Inv, "flyby {0:F0} km", km) : "plain transfer");
        if (!Calculate(t, vehicle, moon, label))
            return;
        OrbitalTransfers.PorkChopEntry entry = TransferSession.SelectedEntry!;
        OrbitalTransfers.TransferInfo info = TransferSession.Info!;
        var body = (IParentBody)moon;

        // Frames pass in the game between the selection and the first preview.
        driver.Step(StepDt, SettleSteps);
        HohmannFlybyUI.Enabled = true;
        if (c.FlybyKm is double altitudeKm)
        {
            AccessTools.Field(typeof(HohmannFlybyUI), "_flybyOn").SetValue(null, true);
            AccessTools.Field(typeof(HohmannFlybyUI), "_inputValueKm").SetValue(null, altitudeKm);
        }
        FlightPlan? preview = null;
        // The TRANSFER page asks for the preview on every frame.
        for (int frame = 0; frame <= SettleSteps; frame++)
        {
            preview = ShowPreview(vehicle, entry, info, c.FlybyKm.HasValue);
            driver.Step(StepDt);
        }
        int buildsBefore = PreviewBuilds(c.FlybyKm.HasValue);
        int recomputesBefore = Counter(typeof(FiniteBurns).Assembly.GetType("AdvancedFlightComputer.Core.PlannedBurnPhases"), "Recomputes");
        for (int frame = 0; frame < IdleFrames; frame++)
        {
            preview = ShowPreview(vehicle, entry, info, c.FlybyKm.HasValue);
            driver.Step(StepDt);
        }
        preview = ShowPreview(vehicle, entry, info, c.FlybyKm.HasValue);
        if (buildsBefore < 0)
            t.Info($"{label}: this build has no preview build counter, so the idle cost is not checked.");
        else
        {
            t.Check($"{label}: {IdleFrames} idle frames build the preview no more than once",
                PreviewBuilds(c.FlybyKm.HasValue) - buildsBefore <= 1,
                $"{PreviewBuilds(c.FlybyKm.HasValue) - buildsBefore} builds");
            int recomputes = Counter(typeof(FiniteBurns).Assembly.GetType("AdvancedFlightComputer.Core.PlannedBurnPhases"), "Recomputes") - recomputesBefore;
            t.Check($"{label}: {IdleFrames} idle frames run no drain simulation", recomputes == 0, $"{recomputes} runs");
        }

        if (c.FlybyKm.HasValue)
        {
            if (!HohmannFlybyUI.TryGetArmed(vehicle, out FlybyTargeting.FlybyResult armed) || preview == null)
            {
                object? prediction = AccessTools.Field(typeof(HohmannFlybyUI), "_prediction").GetValue(null);
                t.Fail($"{label}: flyby armed with a preview", $"preview={(preview != null)} prediction={prediction}"
                    + (preview != null ? "\n" + Describe(preview) : ""));
                return;
            }
            t.Info($"{label}: departure t={armed.BurnTime.Seconds():F1}s impulsive dv={armed.DvVlf.Length():F1} m/s b={armed.ImpactParameterMeters:F0} m");
        }
        else if (!t.Check($"{label}: the selected transfer has a preview", preview != null))
            return;
        t.Info($"{label}: preview plan\n{Describe(preview!)}");

        FlightComputer fc = vehicle.FlightComputer;
        FiniteBurnPhase[] stockBefore = fc.GetFullThrottlePhases().ToArray();
        FiniteBurnPhase[] planned = FiniteBurns.FullThrottlePhases(vehicle).ToArray();
        t.Info(string.Format(Inv, "{0}: before CREATE ActiveEnginePerformanceMax.Thrust={1:F0} N, active sequence {2}",
            label, fc.ActiveEnginePerformanceMax.Thrust, vehicle.Parts.SequenceList.ActiveSequence));
        t.Info($"{label}: stock phases before CREATE {DescribePhases(stockBefore)}");
        t.Info($"{label}: planned phases before CREATE {DescribePhases(planned)}");

        int before = fc.BurnPlan.BurnCount;
        TransferCommitter.Create();
        driver.Step(StepDt, PlanSteps);
        Burn? burn = TransferSession.TransferBurn;
        if (!t.Check($"{label}: CREATE queued one burn", burn != null
                && fc.BurnPlan.BurnCount == before + 1 && fc.BurnPlan.TryGetBurn(burn)))
            return;
        FlightPlan created = burn!.FlightPlan;
        FiniteBurnPhase[] stockAfter = fc.GetFullThrottlePhases().ToArray();
        t.Info($"{label}: stock phases after CREATE {DescribePhases(stockAfter)}");
        t.Info($"{label}: created burn t={burn.Time.Seconds():F1}s dv={burn.DeltaVVlf.Length():F1} m/s plan\n{Describe(created)}");

        if (c.Craft == "Rocket" && c.Staged)
            t.Check($"{label}: the stock phases before CREATE are {(c.EnginesOn ? "not " : "")}the phases of the created burn",
                SamePhases(stockBefore, stockAfter) != c.EnginesOn);
        t.Check($"{label}: the phases the preview was planned with are the phases stock propagates the burn with",
            SamePhases(planned, stockAfter));

        PatchedConic? previewArrival = FindArrival(preview!, body);
        PatchedConic? createdArrival = FindArrival(created, body);
        t.Check($"{label}: the preview reaches the destination", previewArrival != null);
        if (!t.Check($"{label}: the created burn reaches the destination", createdArrival != null) || previewArrival == null)
        {
            RemoveBurn(driver, vehicle, burn);
            return;
        }
        double rpPreview = previewArrival.Orbit.Periapsis;
        double rpCreated = createdArrival!.Orbit.Periapsis;
        t.Check($"{label}: periapsis of preview and created burn agree within {PeriapsisTolM / 1000.0:F0} km",
            Math.Abs(rpCreated - rpPreview) <= PeriapsisTolM,
            string.Format(Inv, "preview {0:F0} m, created {1:F0} m", rpPreview, rpCreated));
        double dt = Math.Abs((createdArrival.StartTime - previewArrival.StartTime).Seconds());
        t.Check($"{label}: encounter time of preview and created burn agree within {EncounterTimeTolS:F0} s", dt <= EncounterTimeTolS,
            string.Format(Inv, "preview {0:F1} s, created {1:F1} s", previewArrival.StartTime.Seconds(), createdArrival.StartTime.Seconds()));
        // The aim across parents starts the transfer conic at the centre of the parking body, so its periapsis is checked against the preview only.
        if (c.FlybyKm is double wantedKm && c.Destination == null)
        {
            double rpWanted = body.MeanRadius + wantedKm * 1000.0;
            t.Check($"{label}: the created periapsis is near the requested one",
                Math.Abs(rpCreated - rpWanted) <= RequestRelTol * rpWanted,
                string.Format(Inv, "created {0:F0} m, wanted {1:F0} m", rpCreated, rpWanted));
        }
        RemoveBurn(driver, vehicle, burn);
    }

    // A flyby that is requested and cannot arm must not become the transfer aimed at the centre of the moon.
    private static void RunRefusal(TestContext t, SimDriver driver, Vehicle vehicle, Celestial moon)
    {
        const string label = "Rocket, flyby below the floor";
        if (!Calculate(t, vehicle, moon, label))
            return;
        OrbitalTransfers.PorkChopEntry entry = TransferSession.SelectedEntry!;
        OrbitalTransfers.TransferInfo info = TransferSession.Info!;
        driver.Step(StepDt, SettleSteps);
        HohmannFlybyUI.Enabled = true;
        AccessTools.Field(typeof(HohmannFlybyUI), "_flybyOn").SetValue(null, true);
        AccessTools.Field(typeof(HohmannFlybyUI), "_inputValueKm").SetValue(null, RefusedAltitudeKm);
        for (int frame = 0; frame <= SettleSteps; frame++)
        {
            HohmannFlybyUI.ShowSingleDeparture(vehicle, entry, info);
            driver.Step(StepDt);
        }
        if (HohmannFlybyUI.TryGetArmed(vehicle, out _))
        {
            t.Skip($"{label}: the flyby armed, so there is nothing to refuse.");
            return;
        }
        string refusal = TransferCommitter.Refusal();
        t.Check($"{label}: CREATE names why it refuses", refusal.Length > 0, $"refusal '{refusal}'");
        int before = vehicle.FlightComputer.BurnPlan.BurnCount;
        TransferCommitter.Create();
        driver.Step(StepDt, PlanSteps);
        t.Check($"{label}: CREATE makes no burn", TransferSession.TransferBurn == null
            && vehicle.FlightComputer.BurnPlan.BurnCount == before);
        if (TransferSession.TransferBurn is Burn made)
            RemoveBurn(driver, vehicle, made);
    }

    private static FlightPlan? ShowPreview(
        Vehicle vehicle, OrbitalTransfers.PorkChopEntry entry, OrbitalTransfers.TransferInfo info, bool flyby)
    {
        if (!flyby)
        {
            FlightPlan plan = TransferFlownPlan.Plan(vehicle, entry);
            return ReferenceEquals(plan, entry.FlightPlan) ? null : plan;
        }
        HohmannFlybyUI.ShowSingleDeparture(vehicle, entry, info);
        return (FlightPlan?)AccessTools.Field(typeof(HohmannFlybyUI), "_previewPlan").GetValue(null);
    }

    private static int PreviewBuilds(bool flyby)
        => flyby ? Counter(typeof(HohmannFlybyUI), "PreviewBuilds") : Counter(typeof(TransferFlownPlan), "Builds");

    // -1 when the build has no such counter.
    private static int Counter(Type? type, string name)
        => type == null ? -1 : AccessTools.Property(type, name)?.GetValue(null) is int value ? value : -1;

    private static bool SamePhases(FiniteBurnPhase[] a, FiniteBurnPhase[] b)
    {
        if (a.Length != b.Length)
            return false;
        for (int i = 0; i < a.Length; i++)
        {
            if (a[i].StartsStage != b[i].StartsStage
                || !Near(a[i].Thrust, b[i].Thrust) || !Near(a[i].MassFlowRate, b[i].MassFlowRate)
                || !Near(a[i].StartMass, b[i].StartMass) || !Near(a[i].MaxSeconds, b[i].MaxSeconds))
                return false;
        }
        return true;
    }

    private static bool Near(double a, double b)
        => a == b || Math.Abs(a - b) <= PhaseRelTol * Math.Max(Math.Abs(a), Math.Abs(b));

    private static void RemoveBurn(SimDriver driver, Vehicle vehicle, Burn burn)
    {
        vehicle.FlightComputer.RemoveBurn(burn);
        driver.Step(StepDt);
    }

    // A circular parking orbit in the plane of the moon, so the porkchop finds a transfer of about 3.1 km/s from low orbit as in a normal lunar mission, not one that also changes the plane.
    private static Orbit ParkingInMoonPlane(IParentBody home, Celestial moon, UniverseTime now)
    {
        StateVectors m = moon.Orbit.GetStateVectorsAt(now);
        double3 normal = double3.Cross(m.PositionCci, m.VelocityCci).NormalizeOrZero();
        double3 radial = m.PositionCci.NormalizeOrZero();
        double r = home.MeanRadius + ParkingAltitudeM;
        double v = Math.Sqrt(home.Mu / r);
        return Orbit.CreateFromStateCci(home, now, radial * r, double3.Cross(normal, radial) * v, VehicleSpawner.OrbitLineColor);
    }

    private static bool Calculate(TestContext t, Vehicle vehicle, Celestial moon, string label)
    {
        TransferSession.CancelAll();
        TransferSession.SetSource(vehicle);
        TransferSession.SetDestination(moon);
        TransferSession.StartCalculation();
        DateTime start = DateTime.UtcNow;
        while (TransferSession.Calculating && DateTime.UtcNow - start < CalculationTimeout)
        {
            Thread.Sleep(50);
            TransferSession.Tick();
        }
        DateTime refineStart = DateTime.UtcNow;
        while (TransferSession.Refining && DateTime.UtcNow - refineStart < RefineTimeout)
            Thread.Sleep(50);
        return t.Check($"{label}: the porkchop has a selected entry",
            TransferSession.Calculated && !TransferSession.Refining && TransferSession.SelectedEntry != null);
    }

    // The patch inside the destination's SOI that the encounter starts.
    private static PatchedConic? FindArrival(FlightPlan plan, IParentBody body)
    {
        foreach (PatchedConic patch in plan.Patches)
        {
            if (patch.PrimaryBody?.Id == body.Id && patch.StartTransition == PatchTransition.Encounter)
                return patch;
        }
        return null;
    }

    private static string DescribePhases(ReadOnlySpan<FiniteBurnPhase> phases)
    {
        var sb = new StringBuilder();
        foreach (FiniteBurnPhase p in phases)
            sb.AppendFormat(Inv, "[F={0:F0}N mdot={1:F2} m0={2:F0} max={3:F1}s stage={4}] ", p.Thrust, p.MassFlowRate, p.StartMass, p.MaxSeconds, p.StartsStage);
        return sb.ToString();
    }

    private static string Describe(FlightPlan plan)
    {
        var sb = new StringBuilder();
        for (int i = 0; i < plan.Patches.Count; i++)
        {
            PatchedConic p = plan.Patches[i];
            sb.AppendFormat(Inv, "  [{0}] {1} {2}->{3} t={4:F1}..{5} rp={6:F0} e={7:F4} burnArc={8}\n",
                i, p.PrimaryBody?.Id ?? "?", p.StartTransition, p.EndTransition, p.StartTime.Seconds(),
                p.EndTime.IsEndOfTime() ? "end" : p.EndTime.Seconds().ToString("F1", Inv),
                p.Orbit.Periapsis, p.Orbit.Eccentricity, p.BurnTrajectory.HasValue);
        }
        return sb.ToString();
    }
}
