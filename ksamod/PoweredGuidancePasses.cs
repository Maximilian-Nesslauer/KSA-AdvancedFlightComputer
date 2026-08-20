using System;
using Brutal.Numerics;
using KSA;

// Upcoming site passes, in closed form.
//
// This replaced a sampled scan: 240 conic propagations per revolution over five
// revolutions, time-sliced across ~25 frames, suspended under time warp and rerun on
// a 5 s timer. That is a lot of machinery for a question with an analytic answer, and
// it made the readout lag the orbit badly enough to be misleading.
//
// The geometry: the closest a revolution brings the ground track to the site is the
// perpendicular distance from the site to the orbit plane, asin(s·n)·R — and the
// moment it happens is when the vehicle's true anomaly matches that of the site
// PROJECTED into the plane. Both are one dot product and one Kepler conversion, so a
// revolution costs a handful of trig calls instead of 240 propagations and the whole
// set can be rebuilt every frame.
//
// WHAT THIS ASSUMES: that the closest approach sits at the foot of the perpendicular
// from the site to the orbit plane. That is exact for a non-rotating body; the body
// does turn during a pass, which the iteration below absorbs by re-evaluating the
// site where the vehicle will actually meet it. What remains is the track's own
// curvature away from the great circle within a single pass — small over the few
// minutes a pass lasts, and well inside what this display is for, which is choosing
// which revolution to commit to rather than aiming with it.
public static partial class PoweredGuidanceWindow
{
    private static void RefreshPasses(Orbit orbit, IParentBody parent, double mu,
                                      double bodyRadius)
    {
        _s.Passes.Clear();

        double3 r0 = orbit.StateVectors.PositionCci;
        double3 v0 = orbit.StateVectors.VelocityCci;
        double rMag = r0.Length();
        double sma = 1.0 / (2.0 / rMag - double3.Dot(v0, v0) / mu);
        if (!(sma > 0.0) || double.IsNaN(sma))
            return;   // hyperbolic or parabolic: there is no next revolution

        double period = 2.0 * Math.PI * Math.Sqrt(sma * sma * sma / mu);
        double meanMotion = 2.0 * Math.PI / period;

        double3 h = double3.Cross(r0, v0);
        if (h.Length() < 1e-6)
            return;
        double3 n = double3.Normalize(h);

        // In-plane reference direction. The eccentricity vector is the natural one,
        // but it vanishes on a circular orbit — and any fixed in-plane direction will
        // do, because only DIFFERENCES of anomalies are used below.
        double3 eVec = double3.Cross(v0, h) * (1.0 / mu) - r0 * (1.0 / rMag);
        double ecc = eVec.Length();
        double3 u = ecc > 1e-8 ? double3.Normalize(eVec) : double3.Normalize(r0);
        double3 w = double3.Cross(n, u);

        double m0 = MeanFromTrue(Math.Atan2(double3.Dot(r0, w), double3.Dot(r0, u)), ecc);

        for (int k = 0; k < PassesToShow; k++)
        {
            // The site moves while the vehicle coasts round to meet it, so the time of
            // closest approach and the site's position at that time are solved
            // together. The body turns slowly against an orbital period, so this is a
            // contraction and settles in a couple of passes; four is comfortable.
            double t = k * period;
            double crossM = double.NaN;
            double dM = 0.0;

            for (int iter = 0; iter < 4; iter++)
            {
                double3 s = double3.Normalize(SiteDirCciAt(parent, t));
                double sn = Math.Clamp(double3.Dot(s, n), -1.0, 1.0);
                crossM = Math.Asin(sn) * bodyRadius;

                double3 proj = s - sn * n;
                if (proj.Length() < 1e-9)
                    break;   // site on the orbit axis: no perpendicular foot to solve for
                proj = double3.Normalize(proj);

                double nuSite = Math.Atan2(double3.Dot(proj, w), double3.Dot(proj, u));
                double raw = MeanFromTrue(nuSite, ecc) - m0;

                // The first iteration PICKS the crossing - the next one ahead of us -
                // and every one after it stays on that same crossing by unwrapping
                // toward the running estimate instead of re-wrapping into [0, 2pi).
                //
                // Re-wrapping every time is what made a pass we had just flown briefly
                // reappear at the front of the list: within a few seconds of closest
                // approach the answer sits right on the wrap boundary, so successive
                // iterations flipped between "just happened" and "one revolution away".
                dM = iter == 0 ? Wrap2Pi(raw) : raw + TwoPi * Math.Round((dM - raw) / TwoPi);
                t = dM / meanMotion + k * period;
            }

            if (double.IsNaN(crossM))
                continue;
            // Never behind us: unwrapping can land a hair before now, and a pass in the
            // past is not a pass. The crossing after it is a full revolution away.
            if (t < 0.0)
                t += period;
            _s.Passes.Add((t, Math.Abs(crossM) / 1000.0, crossM / 1000.0));
        }
    }

    private const double TwoPi = 2.0 * Math.PI;

    /// <summary>Mean anomaly from true anomaly, through the eccentric anomaly.</summary>
    private static double MeanFromTrue(double trueAnomaly, double ecc)
    {
        double e = Math.Clamp(ecc, 0.0, 0.999999);
        double eAnom = 2.0 * Math.Atan2(
            Math.Sqrt(1.0 - e) * Math.Sin(trueAnomaly * 0.5),
            Math.Sqrt(1.0 + e) * Math.Cos(trueAnomaly * 0.5));
        return eAnom - e * Math.Sin(eAnom);
    }

    /// <summary>
    /// Index of the pass that comes CLOSEST to the site — the one worth acting on, and
    /// so the only one the strip colours. Not the soonest: the soonest is wherever the
    /// vehicle happens to be in its cycle, which says nothing about whether it is a
    /// good opportunity.
    /// </summary>
    private static int NextPassIndex()
    {
        int best = -1;
        double bestT = double.MaxValue;
        for (int i = 0; i < _s.Passes.Count; i++)
        {
            if (_s.Passes[i].tSec >= bestT)
                continue;
            bestT = _s.Passes[i].tSec;
            best = i;
        }
        return best;
    }

    private static int ClosestPassIndex()
    {
        int best = -1;
        double bestKm = double.MaxValue;
        for (int i = 0; i < _s.Passes.Count; i++)
        {
            if (_s.Passes[i].minKm >= bestKm)
                continue;
            bestKm = _s.Passes[i].minKm;
            best = i;
        }
        return best;
    }
}
