namespace AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// The slope of the ground at a landing site, the slopes on which an upright craft can stand, and the search for flatter ground nearby.
/// A craft that lands upright on a slope meets the ground first with its uphill feet, turns about them until its downhill feet meet the ground, and goes on over the downhill feet when the turn still has the energy to lift the centre of mass over them. That is the rocking block of Housner (1963, "The behavior of inverted pendulum structures during earthquakes", BSSA 53(2)), with one plastic impact at the first contact and one at the second.
/// </summary>
public static class SiteSlope
{
    /// <summary>A terrain height in metres at a point east and north of the site centre, in metres on the tangent plane.</summary>
    public readonly record struct GroundSample(double EastM, double NorthM, double HeightM);

    /// <summary>The slope of the plane that fits the samples best in least squares, in radians, and its downhill direction as a unit vector east and north. NaN with fewer than three samples that span an area.</summary>
    public static double SlopeRad(ReadOnlySpan<GroundSample> samples, out double downhillEast, out double downhillNorth)
    {
        downhillEast = 0.0;
        downhillNorth = 0.0;
        int n = 0;
        double me = 0.0, mn = 0.0, mh = 0.0;
        foreach (GroundSample s in samples)
        {
            if (!double.IsFinite(s.EastM) || !double.IsFinite(s.NorthM) || !double.IsFinite(s.HeightM))
                continue;
            me += s.EastM;
            mn += s.NorthM;
            mh += s.HeightM;
            n++;
        }
        if (n < 3)
            return double.NaN;
        me /= n;
        mn /= n;
        mh /= n;
        double see = 0.0, snn = 0.0, sen = 0.0, seh = 0.0, snh = 0.0;
        foreach (GroundSample s in samples)
        {
            if (!double.IsFinite(s.EastM) || !double.IsFinite(s.NorthM) || !double.IsFinite(s.HeightM))
                continue;
            double e = s.EastM - me, no = s.NorthM - mn, h = s.HeightM - mh;
            see += e * e;
            snn += no * no;
            sen += e * no;
            seh += e * h;
            snh += no * h;
        }
        double det = see * snn - sen * sen;
        if (!(Math.Abs(det) > 1e-12 * Math.Max(see * snn, 1e-300)))
            return double.NaN;
        double gradEast = (seh * snn - snh * sen) / det;
        double gradNorth = (snh * see - seh * sen) / det;
        double grad = Math.Sqrt(gradEast * gradEast + gradNorth * gradNorth);
        if (grad > 0.0)
        {
            downhillEast = -gradEast / grad;
            downhillNorth = -gradNorth / grad;
        }
        return Math.Atan(grad);
    }

    /// <summary>
    /// Where a craft stands on level ground: the height of the centre of mass over the ground, and the polygon of its support points around the thrust axis, given by the radius of the circle inside it (the least distance from the axis to an edge) and of the circle around it (the largest distance from the axis to a corner), in metres.
    /// An inner radius of zero means the axis is not inside the polygon, so the craft cannot stand upright on its own feet.
    /// </summary>
    public readonly record struct Stance(double HeightM, double InnerM, double OuterM, int Corners)
    {
        public bool Known => HeightM > 0.0 && double.IsFinite(HeightM) && Corners >= 3;
    }

    /// <summary>A point of the craft in the plane across the thrust axis, in metres from the axis, at its depth under the centre of mass along the axis.</summary>
    public readonly record struct SupportPoint(double X, double Y, double DepthM);

    /// <summary>
    /// The stance from the points that carry the craft on level ground: every point within <paramref name="toleranceM"/> of the deepest one. The polygon is their convex hull.
    /// </summary>
    public static Stance StanceOf(ReadOnlySpan<SupportPoint> points, double toleranceM)
        => Footing.Of(points, toleranceM).At(0.0, 0.0, 0.0);

    /// <summary>
    /// The points that carry the craft on level ground and their convex hull, which do not change when the centre of mass moves, so the stance for many centres of mass costs one hull.
    /// </summary>
    public sealed class Footing
    {
        private readonly List<(double X, double Y)> _hull;
        private readonly double _deepest;

        private Footing(List<(double X, double Y)> hull, double deepest)
        {
            _hull = hull;
            _deepest = deepest;
        }

        public static Footing Of(ReadOnlySpan<SupportPoint> points, double toleranceM)
        {
            double deepest = double.NegativeInfinity;
            foreach (SupportPoint p in points)
            {
                if (double.IsFinite(p.DepthM) && double.IsFinite(p.X) && double.IsFinite(p.Y))
                    deepest = Math.Max(deepest, p.DepthM);
            }
            var carry = new List<(double X, double Y)>();
            if (double.IsFinite(deepest))
            {
                foreach (SupportPoint p in points)
                {
                    if (double.IsFinite(p.DepthM) && double.IsFinite(p.X) && double.IsFinite(p.Y) && p.DepthM >= deepest - toleranceM)
                        carry.Add((p.X, p.Y));
                }
            }
            return new Footing(Hull(carry), deepest);
        }

        /// <summary>The stance with the centre of mass moved by <paramref name="dx"/> and <paramref name="dy"/> across the axis and <paramref name="dz"/> up it from where the points were measured, m.</summary>
        public Stance At(double dx, double dy, double dz)
        {
            double height = _deepest + dz;
            if (!double.IsFinite(height) || !(height > 0.0))
                return default;
            double outer = 0.0;
            foreach ((double x, double y) in _hull)
                outer = Math.Max(outer, Math.Sqrt((x - dx) * (x - dx) + (y - dy) * (y - dy)));
            if (_hull.Count < 3)
                return new Stance(height, 0.0, outer, _hull.Count);
            double inner = double.PositiveInfinity;
            for (int i = 0; i < _hull.Count; i++)
            {
                (double ax, double ay) = _hull[i];
                (double bx, double by) = _hull[(i + 1) % _hull.Count];
                double ex = bx - ax, ey = by - ay;
                double len = Math.Sqrt(ex * ex + ey * ey);
                if (!(len > 0.0))
                    continue;
                // The hull runs counter-clockwise, so the axis is inside when it stands left of every edge, and this is its distance from the edge.
                inner = Math.Min(inner, (ex * (dy - ay) - ey * (dx - ax)) / len);
            }
            return new Stance(height, double.IsFinite(inner) ? Math.Max(inner, 0.0) : 0.0, outer, _hull.Count);
        }
    }

    // Andrew's monotone chain, counter-clockwise, without collinear points.
    private static List<(double X, double Y)> Hull(List<(double X, double Y)> points)
    {
        points.Sort((a, b) => a.X != b.X ? a.X.CompareTo(b.X) : a.Y.CompareTo(b.Y));
        var hull = new List<(double X, double Y)>(points.Count + 1);
        static double Cross((double X, double Y) o, (double X, double Y) a, (double X, double Y) b)
            => (a.X - o.X) * (b.Y - o.Y) - (a.Y - o.Y) * (b.X - o.X);
        for (int pass = 0; pass < 2; pass++)
        {
            int start = hull.Count;
            for (int k = 0; k < points.Count; k++)
            {
                var p = points[pass == 0 ? k : points.Count - 1 - k];
                while (hull.Count >= start + 2 && Cross(hull[^2], hull[^1], p) <= 1e-12)
                    hull.RemoveAt(hull.Count - 1);
                hull.Add(p);
            }
            hull.RemoveAt(hull.Count - 1);
        }
        return hull;
    }

    /// <summary>
    /// The steepest slope on which the craft can rest upright with some heading, in radians: above it the centre of mass is outside the polygon of the feet whichever way the craft faces, so no landing there can end with the craft standing.
    /// </summary>
    public static double StandLimitRad(in Stance stance)
        => stance.Known ? Math.Atan(stance.OuterM / stance.HeightM) : 0.0;

    /// <summary>
    /// The steepest slope on which the craft does not go over after an upright touchdown with the worst heading, in radians.
    /// The worst heading puts an edge of the polygon across the slope, so the craft turns about the edge at the inner radius uphill and lands on the edge at the inner radius downhill. Between the first and the second contact the centre of mass falls by h(1 - cos a) + r sin a for a slope a, inner radius r and height h; the second contact keeps the share (k^2 + h^2 - r^2) / (k^2 + h^2 + r^2) of the turn rate (Housner's coefficient for a block whose pivots lie at the height h under the centre of mass, with k^2 the transverse inertia over the mass); the craft goes over when that turn lifts the centre of mass by sqrt(r^2 + h^2) - (r sin a + h cos a). The vertical speed at the first contact adds a turn rate m r v / I about the uphill edge.
    /// </summary>
    /// <param name="gyrationSqM2">The largest transverse moment of inertia about the centre of mass over the mass, m^2.</param>
    /// <param name="contactMs">The vertical speed at the first contact, m/s.</param>
    /// <param name="gravity">The local gravity, m/s^2.</param>
    public static double TipLimitRad(in Stance stance, double gyrationSqM2, double contactMs, double gravity)
    {
        if (!stance.Known || !(stance.InnerM > 0.0) || !(gravity > 0.0) || !(gyrationSqM2 >= 0.0))
            return 0.0;
        double r = stance.InnerM, h = stance.HeightM;
        double hi = Math.Atan(r / h), lo = 0.0;
        if (!Tips(hi, r, h, gyrationSqM2, contactMs, gravity))
            return hi;
        if (Tips(lo, r, h, gyrationSqM2, contactMs, gravity))
            return 0.0;
        for (int i = 0; i < 50; i++)
        {
            double mid = 0.5 * (lo + hi);
            if (Tips(mid, r, h, gyrationSqM2, contactMs, gravity))
                hi = mid;
            else
                lo = mid;
        }
        return lo;
    }

    private static bool Tips(double slope, double r, double h, double k2, double v, double g)
    {
        double pivotSq = k2 + r * r + h * h;
        double keep = (k2 + h * h - r * r) / pivotSq;
        double fall = h * (1.0 - Math.Cos(slope)) + r * Math.Sin(slope);
        // Kinetic energy over mass just before the second contact: what the first contact leaves of the vertical speed as a turn about the uphill edge, and the fall.
        double before = 0.5 * v * v * r * r / pivotSq + g * fall;
        double rise = Math.Sqrt(r * r + h * h) - (r * Math.Sin(slope) + h * Math.Cos(slope));
        return keep > 0.0 && keep * keep * before > g * rise;
    }

    /// <summary>
    /// The search for the flattest ground within a radius around a site, over a square grid of candidate centres taken from the site outward, a few at a time so it can run over several frames.
    /// The answer is the nearest candidate whose slope is within a tolerance of the flattest one, so a point that is as good as the flattest but closer wins.
    /// </summary>
    public sealed class FlatSearch
    {
        private readonly (double E, double N)[] _points;
        private readonly double[] _slopes;
        private int _next;

        public FlatSearch(double radiusM, double stepM)
        {
            if (!(radiusM > 0.0) || !(stepM > 0.0))
                throw new ArgumentException("The radius and the step must be positive.");
            RadiusM = radiusM;
            StepM = stepM;
            int n = (int)Math.Floor(radiusM / stepM);
            var points = new List<(double E, double N)>();
            for (int i = -n; i <= n; i++)
            {
                for (int j = -n; j <= n; j++)
                {
                    double e = i * stepM, no = j * stepM;
                    if (e * e + no * no <= radiusM * radiusM + 1e-9)
                        points.Add((e, no));
                }
            }
            points.Sort((a, b) =>
            {
                int c = (a.E * a.E + a.N * a.N).CompareTo(b.E * b.E + b.N * b.N);
                return c != 0 ? c : Math.Atan2(a.N, a.E).CompareTo(Math.Atan2(b.N, b.E));
            });
            _points = points.ToArray();
            _slopes = new double[_points.Length];
        }

        public double RadiusM { get; }
        public double StepM { get; }
        public int Count => _points.Length;
        public int Evaluated => _next;
        public bool Done => _next >= _points.Length;

        /// <summary>Measures up to <paramref name="count"/> more candidates with the slope at a point east and north of the site, in radians.</summary>
        public void Step(Func<double, double, double> slopeAt, int count)
        {
            for (int k = 0; k < count && _next < _points.Length; k++, _next++)
                _slopes[_next] = slopeAt(_points[_next].E, _points[_next].N);
        }

        /// <summary>The nearest measured candidate whose slope is at most <paramref name="toleranceRad"/> above the flattest one measured. False while no candidate has a finite slope.</summary>
        public bool TryBest(double toleranceRad, out double eastM, out double northM, out double slopeRad)
        {
            eastM = northM = 0.0;
            slopeRad = double.NaN;
            double least = double.PositiveInfinity;
            for (int i = 0; i < _next; i++)
            {
                if (double.IsFinite(_slopes[i]))
                    least = Math.Min(least, _slopes[i]);
            }
            if (!double.IsFinite(least))
                return false;
            for (int i = 0; i < _next; i++)
            {
                if (double.IsFinite(_slopes[i]) && _slopes[i] <= least + toleranceRad)
                {
                    (eastM, northM) = _points[i];
                    slopeRad = _slopes[i];
                    return true;
                }
            }
            return false;
        }
    }
}
