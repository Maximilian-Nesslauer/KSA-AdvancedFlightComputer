using System;
using Brutal.Numerics;

namespace PoweredGuidance.Upfg;

// Conic State Extrapolation (Shepperd's method). Given a state (r0, v0) and a time
// delta, returns the Keplerian-propagated state. UPFG uses it to estimate the
// gravitational contribution over the remaining burn.
//
// Ported explicitly from navbox's OrbitalMechanics.CSEroutine, converted to double
// precision (Brutal double3) and a small mutable state struct instead of a
// string-keyed dictionary. Mu is passed in rather than read from a global.
public struct CseState
{
    public double Dtcp;
    public double Xcp;
    public double A;
    public double D;
    public double E;

    public static CseState Zero => new CseState();
}

public static class CseRoutine
{
    public static (double3 r, double3 v, CseState last) Run(
        double3 r0, double3 v0, double dt, double mu, CseState last)
    {
        double dtcp = last.Dtcp == 0 ? dt : last.Dtcp;
        double xcp = last.Xcp;
        double x = xcp;
        double A = last.A;
        double D = last.D;
        double E = last.E;

        int kmax = 10;
        int imax = 10;

        double f0 = dt >= 0 ? 1 : -1;

        double r0m = r0.Length();

        double f1 = f0 * Math.Sqrt(r0m / mu);
        double f2 = 1 / f1;
        double f3 = f2 / r0m;
        double f4 = f1 * r0m;
        double f5 = f0 / Math.Sqrt(r0m);
        double f6 = f0 * Math.Sqrt(r0m);

        double3 ir0 = r0 * (1.0 / r0m);
        double3 v0s = f1 * v0;

        double sigma0s = double3.Dot(ir0, v0s);
        double b0 = double3.Dot(v0s, v0s) - 1;
        double alphas = 1 - b0;

        double xguess = f5 * x;
        double xlast = f5 * xcp;
        double xmin = 0;
        double dts = f3 * dt;
        double dtlast = f3 * dtcp;
        double dtmin = 0;

        double xmax = 2 * Math.PI / Math.Sqrt(Math.Abs(alphas));
        double xP = 0;
        double Ps = 0;
        double dtmax = 0;

        if (alphas > 0)
        {
            dtmax = xmax / alphas;
            xP = xmax;
            Ps = dtmax;

            while (dts >= Ps)
            {
                dts -= Ps;
                dtlast -= Ps;
                xguess -= xP;
                xlast -= xP;
            }
        }
        else
        {
            (dtmax, _, _, _) = Ktti(xmax, sigma0s, alphas, kmax);
            while (dtmax < dts)
            {
                dtmin = dtmax;
                xmin = xmax;
                xmax = 2 * xmax;
                (dtmax, _, _, _) = Ktti(xmax, sigma0s, alphas, kmax);
            }
        }

        if (xmin >= xguess || xguess >= xmax)
            xguess = 0.5 * (xmin + xmax);

        (double dtguess, _, _, _) = Ktti(xguess, sigma0s, alphas, kmax);

        if (dts < dtguess)
        {
            if (xguess < xlast && xlast < xmax && dtguess < dtlast && dtlast < dtmax)
            {
                xmax = xlast;
                dtmax = dtlast;
            }
        }
        else
        {
            if (xmin < xlast && xlast < xguess && dtmin < dtlast && dtlast < dtguess)
            {
                xmin = xlast;
                dtmin = dtlast;
            }
        }

        (xguess, dtguess, A, D, E) = Kil(imax, dts, xguess, dtguess,
                                         xmin, dtmin, xmax, dtmax,
                                         sigma0s, alphas, kmax, A, D, E);

        double rs = 1 + 2 * (b0 * A + sigma0s * D * E);
        double b4 = 1 / rs;

        // navbox tracks an integer revolution count here; for ascent dt is small enough
        // that it stays zero, so the multi-revolution terms drop out.
        double xc = f6 * xguess;
        double dtc = f4 * dtguess;

        last.Dtcp = dtc;
        last.Xcp = xc;
        last.A = A;
        last.D = D;
        last.E = E;

        double F = 1 - 2 * A;
        double Gs = 2 * (D * E + sigma0s * A);
        double Fts = -2 * b4 * D * E;
        double Gt = 1 - 2 * b4 * A;

        double3 r = r0m * (F * ir0 + Gs * v0s);
        double3 v = f2 * (Fts * ir0 + Gt * v0s);

        return (r, v, last);
    }

    private static (double t, double A, double D, double E) Ktti(double xarg, double s0s, double a, int kmax)
    {
        double u1 = Uss(xarg, a, kmax);
        double zs = 2 * u1;
        double E = 1 - 0.5 * a * zs * zs;
        double w = Math.Sqrt(Math.Max(0.5 + E / 2, 0));
        double D = w * zs;
        double A = D * D;
        double B = 2 * (E + s0s * D);
        double Q = Qcf(w);
        double t = D * (B + A * Q);
        return (t, A, D, E);
    }

    private static double Uss(double xarg, double a, int kmax)
    {
        double du1 = xarg / 4;
        double u1 = du1;
        double f7 = -a * du1 * du1;
        int k = 3;
        while (k < kmax)
        {
            du1 = f7 * du1 / (k * (k - 1));
            double u1old = u1;
            u1 += du1;
            if (u1 == u1old) break;
            k += 2;
        }
        return u1;
    }

    private static double Qcf(double w)
    {
        double xq;
        if (w < 1)
            xq = 21.04 - 13.04 * w;
        else if (w < 4.625)
            xq = (5.0 / 3.0) * (2 * w + 5);
        else if (w < 13.846)
            xq = (10.0 / 7.0) * (w + 12);
        else if (w < 44)
            xq = 0.5 * (w + 60);
        else if (w < 100)
            xq = 0.25 * (w + 164);
        else
            xq = 70;

        double y = (w - 1) / (w + 1);
        int j = (int)Math.Floor(xq);
        double b = y / (1 + (j - 1) / (j + 2.0) * (1 - 0));
        while (j > 2)
        {
            j--;
            b = y / (1 + (j - 1) / (j + 2.0) * (1 - b));
        }

        double Q = 1 / (w * w) * (1 + (2 - b / 2) / (3 * w * (w + 1)));
        return Q;
    }

    private static (double xguess, double dtguess, double A, double D, double E) Kil(
        int imax, double dts, double xguess, double dtguess,
        double xmin, double dtmin, double xmax, double dtmax,
        double s0s, double a, int kmax, double A, double D, double E)
    {
        int i = 1;
        while (i < imax)
        {
            double dterror = dts - dtguess;
            if (Math.Abs(dterror) < 1e-6) break;

            var si = Si(dterror, xguess, dtguess, xmin, dtmin, xmax, dtmax);
            double dxs = si.dxs;
            xmin = si.xmin;
            dtmin = si.dtmin;
            xmax = si.xmax;
            dtmax = si.dtmax;

            double xold = xguess;
            xguess += dxs;
            if (xguess == xold) break;

            double dtold = dtguess;
            var ktti = Ktti(xguess, s0s, a, kmax);
            dtguess = ktti.t;
            A = ktti.A;
            D = ktti.D;
            E = ktti.E;

            if (dtguess == dtold) break;

            i++;
        }
        return (xguess, dtguess, A, D, E);
    }

    private static (double dxs, double xmin, double dtmin, double xmax, double dtmax) Si(
        double dterror, double xguess, double dtguess, double xmin, double dtmin, double xmax, double dtmax)
    {
        double etp = 1e-6;
        double dtminp = dtguess - dtmin;
        double dtmaxp = dtguess - dtmax;
        double dxs;

        if (Math.Abs(dtminp) < etp || Math.Abs(dtmaxp) < etp)
        {
            dxs = 0;
        }
        else
        {
            if (dterror < 0)
            {
                dxs = (xguess - xmax) * (dterror / dtmaxp);
                if ((xguess + dxs) <= xmin)
                    dxs = (xguess - xmin) * (dterror / dtminp);
                xmax = xguess;
                dtmax = dtguess;
            }
            else
            {
                dxs = (xguess - xmin) * (dterror / dtminp);
                if ((xguess + dxs) >= xmax)
                    dxs = (xguess - xmax) * (dterror / dtmaxp);
                xmin = xguess;
                dtmin = dtguess;
            }
        }

        return (dxs, xmin, dtmin, xmax, dtmax);
    }
}
