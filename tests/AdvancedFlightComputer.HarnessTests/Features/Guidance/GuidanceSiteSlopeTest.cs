using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The ground slope at a landing site on Luna and the search for flatter ground, with the landing craft in orbit of Luna and their legs deployed.
//   stance:  the polygon the legs make and the limits from it are finite and ordered: the tip-over limit is under the limit of a
//            static stand on the inner radius, which is under the limit of a stand with the best heading.
//   slope:   the readout of the LAND page at three sites equals the slope of a plane through rings of terrain samples taken
//            here. The terrain height of the harness is not the one of the windowed game at these sites (5.9 against 10.0 deg
//            at the same point), so the slope a flight met there is only printed beside it.
//            The tip-over limit is no higher than the one of the craft with its propellant of now, and a moved site has no readout until it is
//            measured, which happens at once.
//   drain:   emptying the stores from the lowest up never brings the limit of the craft under the one worked out in orbit.
//   search:  from the steepest site the search ends within its budget of frames, its answer is flatter than the site and under
//            the tip-over limit, is within the radius, and moving the site there makes the readout show the answer's slope. An answer that still tips the craft over offers a search
//            three times as far, up to the widest one.
public sealed class GuidanceSiteSlopeTest : AfcTest
{
    private const double StepDt = 0.05;

    // Sites on the equator of Luna whose terrain was sampled on rings in logged landings: level, 0.18 m and 0.24 m of rise per metre.
    private static readonly (double LonDeg, double ExpectDeg)[] Sites = [(-144.303, 0.0), (-140.258, 10.2), (-141.377, 13.5)];

    public override string Name => "afc-guidance-site-slope";

    protected override void Execute(TestContext t)
    {
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves("Luna 1", "Luna 2");
        if (saves.Count == 0 || t.System.All.Get("Luna") is not Celestial lunaBody || lunaBody is not IParentBody luna)
        {
            t.Skip("a landing craft save (Luna 1 or Luna 2) and the Luna body are required.");
            return;
        }
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        try
        {
            for (int i = 0; i < saves.Count; i++)
                Run(t, saves[i], luna, lunaBody, search: i == 0);
        }
        finally
        {
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static void Run(TestContext t, string save, IParentBody luna, Celestial body, bool search)
    {
        Vehicle? craft = null;
        var ambient = AccessTools.Field(typeof(GuidanceWindow), "_s");
        object? previous = ambient.GetValue(null);
        try
        {
            craft = VehicleSpawner.SpawnFromSave(save, t.System, luna, "AfcSiteSlope_" + Guid.NewGuid().ToString("N"),
                OrbitFixtures.CircularAt(luna, 50_000, Universe.GetElapsedTime()));
            SimDriver driver = t.Session.CreateDriver();
            driver.Step(StepDt, 2);
            foreach (KeyframeAnimationModule anim in craft.Parts.Modules.Get<KeyframeAnimationModule>())
            {
                if (KsaLandingLegs.IsLeg(anim))
                    anim.TimeGoal = anim.Shared.Duration;
            }
            for (int i = 0; i < 400 && KeyframeAnimationModule.AnyAnimating(craft.Parts.States); i++)
                driver.Step(StepDt);
            driver.Step(StepDt, 2);
            if (!t.Check($"{save}: measures the craft", KsaVehicleHeight.TryMeasure(craft, out KsaVehicleHeight.Result h)))
                return;

            SiteSlope.Stance stance = GuidanceWindow.StanceOf(h.Footprints, corners: false);
            SiteSlope.Stance outer = GuidanceWindow.StanceOf(h.Footprints, corners: true);
            Ksa6DofSetup.Inertia(craft, out double ixx, out double iyy, out double izz);
            double k2 = Math.Max(ixx, iyy) / craft.TotalMass;
            double g = luna.Mu / (luna.MeanRadius * luna.MeanRadius);
            var limitsWatch = System.Diagnostics.Stopwatch.StartNew();
            GuidanceWindow.Limits(craft, h.Footprints, g, out double tipRad, out double standRad, out _);
            t.Info($"{save}: the limits took {limitsWatch.Elapsed.TotalMilliseconds:F2} ms over {craft.Parts.SubstanceStores.Length} stores and {h.Footprints.Length} colliders");
            double tip = tipRad * 180.0 / Math.PI;
            double stand = standRad * 180.0 / Math.PI;
            double staticInner = Math.Atan(stance.InnerM / stance.HeightM) * 180.0 / Math.PI;
            t.Info($"{save}: height {stance.HeightM:F3} m, feet centres: inner {stance.InnerM:F3} m, outer {stance.OuterM:F3} m, {stance.Corners} corners; bounds corners: inner {outer.InnerM:F3} m, outer {outer.OuterM:F3} m; k^2 {k2:F2} m^2 (Ixx {ixx:F0}, Iyy {iyy:F0}, Izz {izz:F0}), mass {craft.TotalMass:F0} kg, g {g:F3}");
            t.Info($"{save}: tips over above {tip:F2} deg (contact at {GuidanceWindow.TipContactMs:F1} m/s), static on the inner radius {staticInner:F2} deg, no stand above {stand:F2} deg");
            foreach (double v in new[] { 0.0, 0.5, 2.0 })
                t.Info($"{save}: tip-over limit at {v:F1} m/s contact {SiteSlope.TipLimitRad(in stance, k2, v, g) * 180.0 / Math.PI:F2} deg");
            double liveTip = SiteSlope.TipLimitRad(in stance, k2, GuidanceWindow.TipContactMs, g) * 180.0 / Math.PI;
            KsaFrameBridge.BodyAxes(craft, out _, out _, out double3 mz);
            OffsetMassProperties dry = craft.Props.InertMassPropsAsmb;
            double dryRise = double3.Dot(double3.Unpack(in dry.Offset) - craft.CenterOfMassAsmb, mz);
            t.Info($"{save}: with the propellant of now it tips over above {liveTip:F2} deg; without propellant {dry.Props.Mass:F0} kg of {craft.TotalMass:F0} kg, centre of mass {dryRise:F3} m higher");
            t.Check($"{save}: the tip-over limit holds over the propellant states down to empty tanks", tip <= liveTip + 1e-9, $"{tip:F2} against {liveTip:F2} deg now");
            LogFeet(t, save, h);
            if (!t.Check($"{save}: the legs make a polygon around the axis", stance.Known && stance.InnerM > 0.0,
                    $"{stance.Corners} corners, inner {stance.InnerM:F3} m"))
                return;
            t.Check($"{save}: the limits are ordered", tip > 0.0 && tip <= staticInner + 1e-9 && staticInner <= stand + 1e-9,
                $"tip {tip:F2}, static {staticInner:F2}, stand {stand:F2} deg");

            VehicleAutopilotState state = VehicleAutopilotState.For(craft);
            ambient.SetValue(null, state);
            state.VehicleHeightAuto = true;
            foreach ((double lon, double expect) in Sites)
            {
                Refresh(state, craft, 0.0, lon);
                double sampled = SampledSlopeDeg(body, 0.0, lon, state.SiteFootprintM);
                t.Info($"{save}: lon {lon:F3}: readout {state.SiteSlopeDeg:F2} deg over {state.SiteFootprintM:F1} m, sampled here {sampled:F2} deg, rise logged in flights {expect:F1} deg, verdict {SiteSlopeText.Verdict(state.SiteSlopeDeg, state.SiteTipDeg, state.SiteStandDeg)}");
                t.CheckAbs($"{save}: lon {lon:F3}: the readout is the slope of the samples", state.SiteSlopeDeg, sampled, 0.01);
                t.CheckAbs($"{save}: lon {lon:F3}: the readout limit is this craft's", state.SiteTipDeg, tip, 0.5);
            }
            state.VehicleHeightNextMs = long.MaxValue;
            state.SiteLonDeg = Sites[0].LonDeg + 0.01;
            t.Check($"{save}: a moved site has no readout before it is measured", double.IsNaN(GuidanceWindow.SiteSlopeNowDeg(luna)));
            AccessTools.Method(typeof(GuidanceWindow), "RefreshVehicleHeight").Invoke(null, new object[] { craft });
            t.Check($"{save}: a moved site is measured at once, not on the refresh clock", double.IsFinite(GuidanceWindow.SiteSlopeNowDeg(luna)));
            if (search)
                Search(t, save, state, craft, luna);
            Drain(t, save, craft, g, tip);
        }
        finally
        {
            ambient.SetValue(null, previous);
            if (craft != null)
                VehicleAutopilotState.Remove(craft);
        }
    }

    private static void Search(TestContext t, string save, VehicleAutopilotState state, Vehicle craft, IParentBody luna)
    {
        (double lon, _) = Sites[^1];
        Refresh(state, craft, 0.0, lon);
        double siteSlope = state.SiteSlopeDeg;
        if (!t.Check($"{save}: the search starts", GuidanceWindow.StartFlatSearch(craft)))
            return;
        var watch = System.Diagnostics.Stopwatch.StartNew();
        int frames = 0;
        while (!GuidanceWindow.StepFlatSearch(luna, GuidanceWindow.FlatSearchBudgetMs) && frames < 2000)
            frames++;
        watch.Stop();
        SiteSlope.FlatSearch run = state.FlatSearch;
        t.Info($"{save}: search of {run.Count} centres {run.StepM:F1} m apart within {run.RadiusM:F0} m: {frames + 1} frames, {watch.Elapsed.TotalMilliseconds:F0} ms, {watch.Elapsed.TotalMilliseconds * 1000.0 / Math.Max(run.Count, 1):F0} us per centre");
        t.Info($"{save}: answer {state.FlatSiteSlopeDeg:F2} deg at lat {state.FlatSiteLat:F5} lon {state.FlatSiteLon:F5}, {state.FlatSiteDistanceM:F0} m bearing {state.FlatSiteBearingDeg:F0} deg; the site {siteSlope:F2} deg; '{SiteSlopeText.Found(state.FlatSiteSlopeDeg, state.FlatSiteDistanceM, state.FlatSiteBearingDeg, siteSlope, run.RadiusM)}'");
        if (!t.Check($"{save}: the search ends", run.Done && double.IsFinite(state.FlatSiteSlopeDeg), $"{run.Evaluated} of {run.Count}"))
            return;
        t.Check($"{save}: the search fits in a few seconds of frames", frames < 600, $"{frames + 1} frames of {GuidanceWindow.FlatSearchBudgetMs:F0} ms");
        t.Check($"{save}: the answer is flatter than the site", SiteSlopeText.IsBetter(state.FlatSiteSlopeDeg, siteSlope), $"{state.FlatSiteSlopeDeg:F2} against {siteSlope:F2} deg");
        t.Check($"{save}: the answer is under the tip-over limit, or a search three times as far is offered",
            state.FlatSiteSlopeDeg <= state.SiteTipDeg || GuidanceWindow.NextFlatSearchRadiusM(luna) == 3.0 * run.RadiusM,
            $"{state.FlatSiteSlopeDeg:F2} against {state.SiteTipDeg:F2} deg, next radius {GuidanceWindow.NextFlatSearchRadiusM(luna):F0} m");
        t.Check($"{save}: the answer is within the radius", state.FlatSiteDistanceM <= run.RadiusM + 1e-6);
        // The search lays its grid on the ground at the site, so its distances are at the surface radius there, not at the mean radius;
        // on Luna the two differ by several kilometres, which makes about a metre over a few hundred.
        double groundRadius = luna.MeanRadius + KsaSurfaceHeight.At(luna, GuidanceWindow.SiteDirCcf(0.0, lon));
        double km = GroundKm(luna, 0.0, lon, state.FlatSiteLat, state.FlatSiteLon) * groundRadius / luna.MeanRadius;
        t.CheckAbs($"{save}: the distance shown is the distance on the ground", km * 1000.0, state.FlatSiteDistanceM, 1.0);

        double answer = state.FlatSiteSlopeDeg;
        state.FlatSearch = new SiteSlope.FlatSearch(GuidanceWindow.FlatSearchRadiusM, run.StepM);
        state.FlatSiteSlopeDeg = state.SiteTipDeg + 1.0;
        t.Check($"{save}: an answer that tips the craft over offers a search three times as far", GuidanceWindow.NextFlatSearchRadiusM(luna) == 3.0 * GuidanceWindow.FlatSearchRadiusM);
        state.FlatSearch = new SiteSlope.FlatSearch(GuidanceWindow.FlatSearchWidestM, GuidanceWindow.FlatSearchWidestM / 30.0);
        t.Check($"{save}: after the widest search no search is offered", GuidanceWindow.NextFlatSearchRadiusM(luna) == 0.0 && !GuidanceWindow.StartFlatSearch(craft));
        (state.FlatSearch, state.FlatSiteSlopeDeg) = (run, answer);
        t.Check($"{save}: the site moves to the answer", GuidanceWindow.MoveToFlatSite(luna));
        t.Check($"{save}: the site is the answer", state.SiteLatDeg == state.FlatSiteLat && state.SiteLonDeg == state.FlatSiteLon);
        Refresh(state, craft, state.SiteLatDeg, state.SiteLonDeg);
        t.CheckAbs($"{save}: the readout at the new site is the slope the answer showed", state.SiteSlopeDeg, answer, 1e-9);
        t.Check($"{save}: the answer of the old site is gone", !GuidanceWindow.FlatSearchFits(luna));
    }

    // Empties the stores one by one from the lowest up, the order that keeps the centre of mass highest, and takes the tip-over limit of the craft as it is after each.
    private static void Drain(TestContext t, string save, Vehicle craft, double g, double tipDeg)
    {
        KsaFrameBridge.BodyAxes(craft, out _, out _, out double3 mz);
        var stores = new List<(ISubstanceStore Store, double Up)>();
        foreach (ISubstanceStore store in craft.Parts.SubstanceStores)
        {
            OffsetMassProperties p = store.ComputeMassPropertiesVehicleAsmb(craft.Parts.Moles.States);
            if (p.Props.Mass > 0.0f)
                stores.Add((store, double3.Dot(double3.Unpack(in p.Offset), mz)));
        }
        stores.Sort((a, b) => a.Up.CompareTo(b.Up));
        double least = double.PositiveInfinity;
        var seen = new List<string>();
        foreach ((ISubstanceStore store, _) in stores)
        {
            store.DepleteAll(craft.Parts.Moles);
            craft.RecomputeMassAfterSupplyChange();
            if (!KsaVehicleHeight.TryMeasure(craft, out KsaVehicleHeight.Result h))
                continue;
            SiteSlope.Stance stance = GuidanceWindow.StanceOf(h.Footprints, corners: false);
            Ksa6DofSetup.Inertia(craft, out double ixx, out double iyy, out _);
            double live = SiteSlope.TipLimitRad(in stance, Math.Max(ixx, iyy) / craft.TotalMass, GuidanceWindow.TipContactMs, g) * 180.0 / Math.PI;
            least = Math.Min(least, live);
            seen.Add($"{craft.TotalMass / 1000.0:F1} t {live:F2}");
        }
        t.Info($"{save}: tip-over limits as the stores run empty from the lowest up: {string.Join(", ", seen)} deg");
        t.Check($"{save}: no state on the way to empty tanks goes over below the limit worked out in orbit", seen.Count > 0 && least >= tipDeg - 0.01,
            $"lowest {least:F2} against {tipDeg:F2} deg");
    }

    private static void Refresh(VehicleAutopilotState state, Vehicle craft, double latDeg, double lonDeg)
    {
        state.SiteLatDeg = latDeg;
        state.SiteLonDeg = lonDeg;
        state.VehicleHeightNextMs = 0;
        AccessTools.Method(typeof(GuidanceWindow), "RefreshVehicleHeight").Invoke(null, new object[] { craft });
    }

    // A plane through 16 points on every half metre out to the footprint and the centre, fitted here without the mod.
    private static double SampledSlopeDeg(Celestial body, double latDeg, double lonDeg, double reachM)
    {
        double3 up = GuidanceWindow.SiteDirCcf(latDeg, lonDeg);
        double3 east = double3.Normalize(double3.Cross(new double3(0, 0, 1), up));
        double3 north = double3.Cross(up, east);
        double radius = body.MeanRadius + body.GetTerrainHeightFromDirCcf(up);
        var samples = new List<SiteSlope.GroundSample> { new(0.0, 0.0, radius - body.MeanRadius) };
        for (double r = 0.5; r <= reachM + 1e-9; r += 0.5)
        {
            for (int i = 0; i < 16; i++)
            {
                double a = i * Math.PI / 8.0;
                double e = r * Math.Cos(a), n = r * Math.Sin(a);
                samples.Add(new SiteSlope.GroundSample(e, n, body.GetTerrainHeightFromDirCcf(double3.Normalize(up * radius + east * e + north * n))));
            }
        }
        return SiteSlope.SlopeRad(samples.ToArray(), out _, out _) * 180.0 / Math.PI;
    }

    private static double GroundKm(IParentBody body, double lat1, double lon1, double lat2, double lon2)
    {
        double3 a = GuidanceWindow.SiteDirCcf(lat1, lon1), b = GuidanceWindow.SiteDirCcf(lat2, lon2);
        return Math.Acos(Math.Clamp(double3.Dot(a, b), -1.0, 1.0)) * body.MeanRadius / 1000.0;
    }

    // The colliders that carry the craft on level ground, for the record of the stance.
    private static void LogFeet(TestContext t, string save, in KsaVehicleHeight.Result h)
    {
        double deepest = 0.0;
        foreach (KsaVehicleHeight.Footprint p in h.Footprints)
            deepest = Math.Max(deepest, p.DepthM);
        int shown = 0;
        foreach (KsaVehicleHeight.Footprint p in h.Footprints)
        {
            if (p.DepthM < deepest - Math.Max(0.05, 0.05 * deepest) || shown++ >= 16)
                continue;
            t.Info($"{save}: foot at x {p.X:F3} y {p.Y:F3}, half {p.HalfX:F3} x {p.HalfY:F3}, depth {p.DepthM:F3} m, {Math.Atan2(p.Y, p.X) * 180.0 / Math.PI:F0} deg around the axis");
        }
    }
}
