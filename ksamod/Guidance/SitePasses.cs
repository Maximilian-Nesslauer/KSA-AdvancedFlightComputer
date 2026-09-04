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
// The passes are then indexed by PHASE — how many times the vehicle has lapped that
// projection — rather than by revolution number. See RefreshPasses: that is what makes
// flying over the site advance the list by exactly one, seamlessly, instead of briefly
// inventing a close approach that is not there.
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

        // PHASE, not position, is what has to be bracketed.
        //
        // Let phi(t) = (vehicle mean anomaly) - (mean anomaly of the site PROJECTED
        // into the orbit plane). The vehicle laps that projection once per revolution,
        // so phi rises steadily and every crossing is phi = 2*pi*j for an integer j.
        // Choosing j picks the pass, and j only ever increases.
        //
        // Bracketing in vehicle anomaly instead — "the crossing in [0, 2pi)" — is what
        // produced a false close approach the moment one was flown. The site rotates
        // roughly 22 degrees of a low orbit's period, so re-evaluating it at the newly
        // estimated time moved the target enough to land back at the START of that
        // window, and the solve settled on a crossing a few minutes away that does not
        // exist. In phi the same event simply increments j.
        double ms0 = SiteMeanAnomaly(parent, 0.0, n, u, w, ecc, bodyRadius, m0, out _);
        if (double.IsNaN(ms0))
            return;   // site on the orbit axis: no perpendicular foot to solve for

        // ms0 was unwrapped toward m0, so phi0 lands in [-pi, pi): a crossing is
        // imminent when it is just below zero and just flown when it is just above.
        double phi0 = m0 - ms0;

        // Roughly how fast phi rises: the vehicle's mean motion less the rate the
        // site's projection drifts. Only ever used to pick a step size — the drift is
        // strongly non-uniform over a lap, so this is nowhere near good enough to
        // extrapolate a root from.
        double probe = period * 0.01;
        double msProbe = SiteMeanAnomaly(parent, probe, n, u, w, ecc, bodyRadius, ms0, out _);
        double phiDot = double.IsNaN(msProbe)
            ? meanMotion
            : meanMotion - (msProbe - ms0) / probe;
        if (!(phiDot > 1e-9))
            phiDot = meanMotion;   // degenerate geometry: fall back to the orbit alone

        double lap = TwoPi / phiDot;
        double step = Math.Max(lap / 12.0, period / 24.0);

        // BRACKET, then refine. phi is monotonic but distinctly non-linear across a
        // lap, so walking it in steps and refining inside whichever step contains the
        // crossing finds every root, in order, and cannot invent or skip one.
        //
        // Extrapolating instead — Newton against a single measured slope — was wrong
        // by about a fifth over several laps, which is comparable to a whole lap by
        // the fourth pass: it walked into the NEXT root and dropped one entirely.
        // Still only a few dozen trig evaluations, against 1200 propagations before.
        long j = (long)Math.Floor(phi0 / TwoPi) + 1;
        double tA = 0.0, phiA = phi0, msPrev = ms0;

        for (int guard = 0; guard < PassStepBudget && _s.Passes.Count < PassesToShow; guard++)
        {
            double tB = tA + step;
            double msB = SiteMeanAnomaly(parent, tB, n, u, w, ecc, bodyRadius, msPrev, out _);
            if (double.IsNaN(msB))
                break;
            double phiB = m0 + meanMotion * tB - msB;

            // A single step can straddle more than one target if the orbit is short
            // against the step, so drain them all before moving on.
            while (_s.Passes.Count < PassesToShow && phiB >= TwoPi * j)
            {
                double target = TwoPi * j;
                double lo = tA, phiLo = phiA, hi = tB, phiHi = phiB;
                double t = lo + (target - phiLo) * (hi - lo) / (phiHi - phiLo);

                // False position: the bracket is kept valid at every step, so this
                // cannot leave the interval however non-linear phi is inside it.
                for (int i = 0; i < 4; i++)
                {
                    double msT = SiteMeanAnomaly(parent, t, n, u, w, ecc, bodyRadius, msPrev, out _);
                    if (double.IsNaN(msT))
                        break;
                    double phiT = m0 + meanMotion * t - msT;
                    if (phiT < target) { lo = t; phiLo = phiT; }
                    else { hi = t; phiHi = phiT; }
                    if (phiHi - phiLo < 1e-12)
                        break;
                    t = lo + (target - phiLo) * (hi - lo) / (phiHi - phiLo);
                }

                SiteMeanAnomaly(parent, t, n, u, w, ecc, bodyRadius, msPrev, out double crossM);
                if (!double.IsNaN(crossM) && t >= 0.0)
                    _s.Passes.Add((t, Math.Abs(crossM) / 1000.0, crossM / 1000.0));
                j++;
            }

            tA = tB;
            phiA = phiB;
            msPrev = msB;
        }
    }

    /// <summary>
    /// The mean anomaly the vehicle would be at if it sat where the site projects into
    /// the orbit plane, plus that projection's perpendicular offset from the plane —
    /// the pass distance. Unwrapped toward <paramref name="near"/> so the caller can
    /// keep a continuous phase rather than a value that jumps every revolution.
    /// NaN when the site lies on the orbit's axis and there is no projection to take.
    /// </summary>
    private static double SiteMeanAnomaly(IParentBody parent, double t, double3 n, double3 u,
                                          double3 w, double ecc, double bodyRadius,
                                          double near, out double crossM)
    {
        double3 s = double3.Normalize(SiteDirCciAt(parent, t));
        double sn = Math.Clamp(double3.Dot(s, n), -1.0, 1.0);
        crossM = Math.Asin(sn) * bodyRadius;

        double3 proj = s - sn * n;
        if (proj.Length() < 1e-9)
            return double.NaN;
        proj = double3.Normalize(proj);

        double m = MeanFromTrue(Math.Atan2(double3.Dot(proj, w), double3.Dot(proj, u)), ecc);
        return m + TwoPi * Math.Round((near - m) / TwoPi);
    }

    private const double TwoPi = 2.0 * Math.PI;

    /// <summary>Hard cap on the bracketing walk, so a degenerate orbit cannot spin.</summary>
    private const int PassStepBudget = 256;

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
