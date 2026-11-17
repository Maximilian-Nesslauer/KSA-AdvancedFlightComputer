#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Diagnostics;
using BepuUtilities;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;

// The ground slope at the landing site against what the craft stands on, and the search for flatter ground near the site (SiteSlope). The terrain is sampled from the height function that the game builds its terrain collision mesh from (BubbleTerrainGrid.SampleRadius calls Celestial.GetTerrainHeightFromDirCcf).
public static partial class GuidanceWindow
{
    private const double SlopeRingStepM = 0.5;
    private const int SlopeRingPoints = 16;
    private const int SearchRingPoints = 8;

    /// <summary>The vertical speed at the first contact that the tip-over limit is computed for, m/s. The landing modes aim well under it, and the limit moves little with it, because the fall onto the downhill feet gives most of the turn.</summary>
    internal const double TipContactMs = 1.0;

    /// <summary>How far from the site the first flat-ground search looks, m. A search whose answer still tips the craft over offers the next one three times as far, up to FlatSearchWidestM.</summary>
    internal const double FlatSearchRadiusM = 300.0;

    internal const double FlatSearchWidestM = 2700.0;

    // About 2800 candidate centres for any radius, so every search takes the same few frames.
    private const double FlatSearchCells = 30.0;

    // A candidate this much steeper than the flattest one still wins when it is nearer to the site.
    private const double FlatSearchToleranceDeg = 0.5;

    /// <summary>The time the search may take per frame, ms.</summary>
    internal const double FlatSearchBudgetMs = 3.0;

    private const double RadToDeg = 180.0 / Math.PI;

    // The colliders whose lowest point is this close to the deepest one carry the craft on level ground, m.
    private const double SupportToleranceM = 0.1;

    /// <summary>
    /// The stance of the craft from its colliders in the deployed pose, with the colliders that carry it on level ground. With <paramref name="corners"/> each of them gives the corners of its bounds across the thrust axis, which reach past a round foot, so the polygon is the largest one the feet can make. Without, each gives its centre, which is where a round foot meets the ground, so the polygon is the smallest one.
    /// </summary>
    internal static SiteSlope.Stance StanceOf(KsaVehicleHeight.Footprint[] prints, bool corners)
        => prints == null || prints.Length == 0 ? default : FootingOf(prints, corners).At(0.0, 0.0, 0.0);

    private static SiteSlope.Footing FootingOf(KsaVehicleHeight.Footprint[] prints, bool corners)
    {
        var points = new SiteSlope.SupportPoint[prints.Length * (corners ? 4 : 1)];
        int n = 0;
        foreach (KsaVehicleHeight.Footprint p in prints)
        {
            if (!corners)
            {
                points[n++] = new SiteSlope.SupportPoint(p.X, p.Y, p.DepthM);
                continue;
            }
            points[n++] = new SiteSlope.SupportPoint(p.X - p.HalfX, p.Y - p.HalfY, p.DepthM);
            points[n++] = new SiteSlope.SupportPoint(p.X + p.HalfX, p.Y - p.HalfY, p.DepthM);
            points[n++] = new SiteSlope.SupportPoint(p.X + p.HalfX, p.Y + p.HalfY, p.DepthM);
            points[n++] = new SiteSlope.SupportPoint(p.X - p.HalfX, p.Y + p.HalfY, p.DepthM);
        }
        return SiteSlope.Footing.Of(points, SupportToleranceM);
    }

    // The propellant states between the propellant of now and empty tanks that the limits are worked out for, per drain order, besides the state as each store runs empty.
    private const int PropellantSteps = 10;

    // A mass with its centre of mass in the assembly frame and its moments of inertia about the model X and Y axes through that centre.
    private readonly record struct Lump(double Mass, double3 Com, double Ix, double Iy);

    /// <summary>
    /// The tip-over limit and the stand limit of the craft, in radians, over the propellant states from the propellant of now down to empty tanks, because the craft lands somewhere between them and its centre of mass and inertia change as the tanks drain. Which tank drains first depends on how the craft is plumbed, so the states are taken for the two extreme orders: the lowest tank first, which keeps the centre of mass highest, and the highest tank first, which keeps it lowest. The tip-over limit is the lowest over the states, so the warning holds for the craft at the ground. The stand limit is the highest, so the refusal comes only where no state and no heading can stand. The tip-over limit takes the centres of the feet, the stand limit their corners, so the warning comes early and the refusal only where no foot of any shape could hold the craft. A craft that stands on one wide collider, such as an engine bell, has no polygon of centres and takes the corners for both. <paramref name="outer"/> is the stance with the propellant of now.
    /// </summary>
    internal static bool Limits(Vehicle vehicle, KsaVehicleHeight.Footprint[] prints, double gravity, out double tipRad, out double standRad, out SiteSlope.Stance outer)
    {
        tipRad = standRad = 0.0;
        outer = default;
        ref readonly OffsetMassProperties total = ref vehicle.TotalMassPropsAsmb;
        if (prints == null || !(total.Props.Mass > 0.0f))
            return false;
        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);
        Lump now = LumpOf(in total, mx, my);
        var footing = (Centres: FootingOf(prints, corners: false), Corners: FootingOf(prints, corners: true));
        if (!LimitsAt(footing, default, Math.Max(now.Ix, now.Iy) / now.Mass, gravity, out tipRad, out standRad, out outer))
            return false;

        // VehicleProperties.InertMassPropsAsmb is the craft without propellant (Vehicle.UpdateAfterPartTreeModification), and the propellant is the sum of the substance stores (PartTree.ComputePropellantMassPropertiesAsmb). A tank holds its propellant at a fixed centre with an inertia in proportion to the mass (Tank.ComputeMassPropertiesPartAsmb), so a part-full store is its full lump scaled.
        ref readonly OffsetMassProperties dry = ref vehicle.Props.InertMassPropsAsmb;
        if (!(dry.Props.Mass > 0.0f))
            return true;
        ReadOnlySpan<MoleState> moles = vehicle.Parts.Moles.States;
        var stores = new List<Lump>();
        foreach (ISubstanceStore store in vehicle.Parts.SubstanceStores)
        {
            OffsetMassProperties p = store.ComputeMassPropertiesVehicleAsmb(moles);
            if (p.Props.Mass > 0.0f)
                stores.Add(LumpOf(in p, mx, my));
        }
        if (stores.Count == 0)
            return true;
        stores.Sort((a, b) => double3.Dot(a.Com, mz).CompareTo(double3.Dot(b.Com, mz)));
        double propellant = 0.0;
        foreach (Lump s in stores)
            propellant += s.Mass;
        Lump dryLump = LumpOf(in dry, mx, my);
        var drains = new List<double>(PropellantSteps + stores.Count);
        for (int order = 0; order < 2; order++)
        {
            drains.Clear();
            double emptied = 0.0;
            for (int k = 0; k < stores.Count; k++)
            {
                emptied += stores[order == 0 ? k : stores.Count - 1 - k].Mass;
                drains.Add(emptied);
            }
            for (int i = 1; i < PropellantSteps; i++)
                drains.Add(propellant * i / PropellantSteps);
            foreach (double amount in drains)
            {
                double drained = amount;
                var left = new double[stores.Count];
                for (int k = 0; k < stores.Count; k++)
                {
                    int j = order == 0 ? k : stores.Count - 1 - k;
                    double take = Math.Min(drained, stores[j].Mass);
                    drained -= take;
                    left[j] = Math.Max(stores[j].Mass - take, 0.0) / stores[j].Mass;
                }
                Lump state = Combine(dryLump, stores, left, mx, my);
                double3 shift = state.Com - now.Com;
                if (LimitsAt(footing, new double3(double3.Dot(shift, mx), double3.Dot(shift, my), double3.Dot(shift, mz)), Math.Max(state.Ix, state.Iy) / state.Mass, gravity, out double tip, out double stand, out _))
                {
                    tipRad = Math.Min(tipRad, tip);
                    standRad = Math.Max(standRad, stand);
                }
            }
        }
        return true;
    }

    private static Lump LumpOf(in OffsetMassProperties p, double3 mx, double3 my)
        => new(p.Props.Mass, double3.Unpack(in p.Offset), Across(in p.Props.Inertia, mx), Across(in p.Props.Inertia, my));

    // The dry craft with each store at the share of its mass that is left, with the parallel-axis terms about the joint centre of mass.
    private static Lump Combine(Lump dry, List<Lump> stores, double[] left, double3 mx, double3 my)
    {
        double m = dry.Mass;
        double3 sum = dry.Com * dry.Mass;
        for (int j = 0; j < stores.Count; j++)
        {
            m += stores[j].Mass * left[j];
            sum += stores[j].Com * (stores[j].Mass * left[j]);
        }
        double3 com = sum / m;
        double ix = dry.Ix + dry.Mass * Offset(dry.Com - com, mx), iy = dry.Iy + dry.Mass * Offset(dry.Com - com, my);
        for (int j = 0; j < stores.Count; j++)
        {
            double share = left[j];
            ix += share * (stores[j].Ix + stores[j].Mass * Offset(stores[j].Com - com, mx));
            iy += share * (stores[j].Iy + stores[j].Mass * Offset(stores[j].Com - com, my));
        }
        return new Lump(m, com, ix, iy);
    }

    // The limits with the centre of mass moved by shift along the model axes from where the footprints were measured.
    private static bool LimitsAt((SiteSlope.Footing Centres, SiteSlope.Footing Corners) footing, double3 shift, double gyrationSq, double gravity, out double tipRad, out double standRad, out SiteSlope.Stance outer)
    {
        tipRad = standRad = 0.0;
        outer = footing.Corners.At(shift.X, shift.Y, shift.Z);
        if (!outer.Known || !double.IsFinite(gyrationSq))
            return false;
        SiteSlope.Stance inner = footing.Centres.At(shift.X, shift.Y, shift.Z);
        if (!inner.Known || !(inner.InnerM > 0.0))
            inner = outer;
        tipRad = SiteSlope.TipLimitRad(in inner, gyrationSq, TipContactMs, gravity);
        standRad = SiteSlope.StandLimitRad(in outer);
        return true;
    }

    // The moment of inertia about an axis through the centre of mass, kg m^2 (as Ksa6DofSetup.Inertia projects it).
    private static double Across(in Symmetric3x3 t, double3 a)
        => t.XX * a.X * a.X + t.YY * a.Y * a.Y + t.ZZ * a.Z * a.Z + 2.0 * (t.YX * a.X * a.Y + t.ZX * a.X * a.Z + t.ZY * a.Y * a.Z);

    // The parallel-axis term per unit mass of a point at offset d for an axis along a, m^2.
    private static double Offset(double3 d, double3 a)
    {
        double along = double3.Dot(d, a);
        return double3.Dot(d, d) - along * along;
    }

    // The tangent frame of the ground at a body-fixed direction. East is undefined at a pole, so a pole takes the X axis for it.
    private static void GroundFrame(double3 up, out double3 east, out double3 north)
    {
        east = double3.Cross(new double3(0, 0, 1), up);
        east = east.LengthSquared() > 1e-12 ? double3.Normalize(east) : new double3(1, 0, 0);
        north = double3.Cross(up, east);
    }

    /// <summary>The slope of the ground in radians on rings around a body-fixed direction out to <paramref name="reachM"/>, from <paramref name="rings"/> rings of <paramref name="perRing"/> points and the centre.</summary>
    internal static double SlopeAround(Celestial body, double3 up, double reachM, int rings, int perRing)
    {
        GroundFrame(up, out double3 east, out double3 north);
        double radius = body.MeanRadius + KsaSurfaceHeight.At(body, up);
        var samples = new SiteSlope.GroundSample[rings * perRing + 1];
        int n = 0;
        samples[n++] = new SiteSlope.GroundSample(0.0, 0.0, radius - body.MeanRadius);
        for (int ring = 1; ring <= rings; ring++)
        {
            double r = reachM * ring / rings;
            for (int i = 0; i < perRing; i++)
            {
                double a = i * 2.0 * Math.PI / perRing;
                double e = r * Math.Cos(a), no = r * Math.Sin(a);
                samples[n++] = new SiteSlope.GroundSample(e, no, KsaSurfaceHeight.At(body, double3.Normalize(up * radius + east * e + north * no)));
            }
        }
        return SiteSlope.SlopeRad(samples, out _, out _);
    }

    // The footprint the slope is measured over: out to the corners of the stance, on the ring step, and no wider than the rings of the early contact.
    private static double SlopeReach(in SiteSlope.Stance stance)
        => Math.Min(Math.Max(Math.Ceiling(stance.OuterM / SlopeRingStepM), 1.0) * SlopeRingStepM, SiteGroundMaxReachM);

    private static double SiteSlopeRad(Celestial body, double latDeg, double lonDeg, double reachM)
        => SlopeAround(body, SiteDirCcf(latDeg, lonDeg), reachM, (int)Math.Round(reachM / SlopeRingStepM), SlopeRingPoints);

    // The readout of the LAND page, kept with the vehicle height. The terrain is sampled once per site, body and footprint; the limits follow the mass and the centre of mass as the propellant drains.
    private static void RefreshSiteSlope(Vehicle vehicle, in KsaVehicleHeight.Result h)
    {
        IParentBody parent = vehicle.Orbit?.Parent;
        double gravity = double.NaN;
        if (parent != null)
        {
            double siteRadius = parent.MeanRadius + SiteTerrainHeight(parent);
            gravity = parent.Mu / (siteRadius * siteRadius);
        }
        if (parent is not Celestial body || !Limits(vehicle, h.Footprints, gravity, out double tip, out double stand, out SiteSlope.Stance stance))
        {
            ForgetSiteSlope(parent);
            return;
        }
        double reach = SlopeReach(in stance);
        if (_s.SiteSlopeDeg is double.NaN || !SiteSlopeFits(parent) || reach != _s.SiteSlopeCacheReachM)
        {
            _s.SiteSlopeDeg = SiteSlopeRad(body, _s.SiteLatDeg, _s.SiteLonDeg, reach) * RadToDeg;
            _s.SiteSlopeCacheLat = _s.SiteLatDeg;
            _s.SiteSlopeCacheLon = _s.SiteLonDeg;
            _s.SiteSlopeCacheReachM = reach;
            _s.SiteSlopeCacheBody = parent;
        }
        _s.SiteFootprintM = reach;
        _s.SiteTipDeg = tip * RadToDeg;
        _s.SiteStandDeg = stand * RadToDeg;
    }

    /// <summary>The readout was last refreshed for the site and body that are picked now. A site that moved has no readout until the next refresh, which RefreshVehicleHeight then runs at once.</summary>
    internal static bool SiteSlopeFits(IParentBody parent)
        => ReferenceEquals(parent, _s.SiteSlopeCacheBody) && _s.SiteSlopeCacheLat.Equals(_s.SiteLatDeg) && _s.SiteSlopeCacheLon.Equals(_s.SiteLonDeg);

    /// <summary>The slope at the site that is picked now in degrees, or NaN while the readout belongs to another site.</summary>
    internal static double SiteSlopeNowDeg(IParentBody parent) => SiteSlopeFits(parent) ? _s.SiteSlopeDeg : double.NaN;

    // No readout for the site. The key still names the site, so the next try comes on the refresh clock and not on every frame.
    private static void ForgetSiteSlope(IParentBody parent)
    {
        _s.SiteSlopeDeg = _s.SiteTipDeg = _s.SiteStandDeg = double.NaN;
        _s.SiteSlopeCacheLat = _s.SiteLatDeg;
        _s.SiteSlopeCacheLon = _s.SiteLonDeg;
        _s.SiteSlopeCacheBody = parent;
    }

    /// <summary>The search runs for this site, body and footprint, or has an answer for them.</summary>
    internal static bool FlatSearchFits(IParentBody parent)
        => _s.FlatSearch != null && _s.FlatSearchLat == _s.SiteLatDeg && _s.FlatSearchLon == _s.SiteLonDeg
            && ReferenceEquals(parent, _s.FlatSearchBody) && _s.FlatSearchReachM == _s.SiteFootprintM;

    /// <summary>The radius the next search looks in: three times the last one while its answer for this site still tips the craft over, else the first radius. Zero when the widest search has its answer and that still tips the craft over, so no search can find more.</summary>
    internal static double NextFlatSearchRadiusM(IParentBody parent)
    {
        if (!FlatSearchFits(parent) || !double.IsFinite(_s.FlatSiteSlopeDeg) || _s.FlatSiteSlopeDeg <= _s.SiteTipDeg)
            return FlatSearchRadiusM;
        double wider = _s.FlatSearch.RadiusM * 3.0;
        return wider <= FlatSearchWidestM + 1e-6 ? wider : 0.0;
    }

    /// <summary>Starts a search for flatter ground around the site with the footprint of the craft, in the radius NextFlatSearchRadiusM gives. False while the craft or the body gives no footprint or terrain.</summary>
    internal static bool StartFlatSearch(Vehicle vehicle)
    {
        IParentBody parent = vehicle?.Orbit?.Parent;
        if (parent is not Celestial || !(_s.SiteFootprintM > 0.0) || !double.IsFinite(SiteSlopeNowDeg(parent)))
            return false;
        double radius = NextFlatSearchRadiusM(parent);
        if (!(radius > 0.0))
            return false;
        _s.FlatSearch = new SiteSlope.FlatSearch(radius, Math.Max(radius / FlatSearchCells, _s.SiteFootprintM));
        _s.FlatSearchLat = _s.SiteLatDeg;
        _s.FlatSearchLon = _s.SiteLonDeg;
        _s.FlatSearchBody = parent;
        _s.FlatSearchReachM = _s.SiteFootprintM;
        _s.FlatSiteLat = _s.FlatSiteLon = _s.FlatSiteSlopeDeg = _s.FlatSiteDistanceM = _s.FlatSiteBearingDeg = double.NaN;
        return true;
    }

    /// <summary>Measures candidates for up to <paramref name="budgetMs"/> of wall time, and measures the answer as the site readout does once the last candidate is in. True when the search has its answer.</summary>
    internal static bool StepFlatSearch(IParentBody parent, double budgetMs)
    {
        if (!FlatSearchFits(parent) || parent is not Celestial body)
            return false;
        if (double.IsFinite(_s.FlatSiteSlopeDeg))
            return true;
        SiteSlope.FlatSearch search = _s.FlatSearch;
        double3 up = SiteDirCcf(_s.FlatSearchLat, _s.FlatSearchLon);
        GroundFrame(up, out double3 east, out double3 north);
        double radius = body.MeanRadius + KsaSurfaceHeight.At(body, up);
        double reach = _s.FlatSearchReachM;
        double SlopeAt(double e, double n)
            => SlopeAround(body, double3.Normalize(up * radius + east * e + north * n), reach, 2, SearchRingPoints);
        long start = Stopwatch.GetTimestamp();
        while (!search.Done && Stopwatch.GetElapsedTime(start).TotalMilliseconds < budgetMs)
            search.Step(SlopeAt, 16);
        if (!search.Done)
            return false;
        // Terrain with no finite slope anywhere gives no answer, and the search ends so the page offers it again.
        if (!search.TryBest(FlatSearchToleranceDeg / RadToDeg, out double bestE, out double bestN, out _))
        {
            _s.FlatSearch = null;
            return false;
        }
        double3 dir = double3.Normalize(up * radius + east * bestE + north * bestN);
        _s.FlatSiteLat = Math.Asin(Math.Clamp(dir.Z, -1.0, 1.0)) * RadToDeg;
        _s.FlatSiteLon = Math.Atan2(dir.Y, dir.X) * RadToDeg;
        _s.FlatSiteSlopeDeg = SiteSlopeRad(body, _s.FlatSiteLat, _s.FlatSiteLon, reach) * RadToDeg;
        _s.FlatSiteDistanceM = Math.Sqrt(bestE * bestE + bestN * bestN);
        _s.FlatSiteBearingDeg = Math.Atan2(bestE, bestN) * RadToDeg;
        return true;
    }

    /// <summary>Moves the site to the answer of the search, through the retarget that a click in the world uses, so a landing that flies plans again. False while there is no answer that is flatter than the site, or the site is locked.</summary>
    internal static bool MoveToFlatSite(IParentBody parent)
    {
        if (!FlatSearchFits(parent) || !double.IsFinite(_s.FlatSiteSlopeDeg) || DeorbitTargetLocked || _retargetStageId != 0
            || !Ui.Logic.SiteSlopeText.IsBetter(_s.FlatSiteSlopeDeg, SiteSlopeNowDeg(parent)))
            return false;
        RetargetLandingSite(_s.FlatSiteLat, _s.FlatSiteLon);
        _s.FlatSearch = null;
        return true;
    }
}
