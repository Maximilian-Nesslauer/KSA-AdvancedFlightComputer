namespace AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;

/// <summary>
/// The longitudinal entry equations with energy as the independent variable (Lu 2008, Eqs. 7-9, Earth rotation included).
///
/// Normalised units: length by the body radius R0, speed by sqrt(mu/R0), acceleration by g0 = mu/R0^2.
///
///   e = 1/r - V^2/2                     the negative of specific energy; rises through entry, de/dtau = D V
///   state x = (r, gamma, s)             radius, flight-path angle, range to go (rad of arc)
///   control u = (L/D) cos(sigma)        vertical lift over drag
///
///   r'     = sin(gamma) / D
///   gamma' = u / V^2 + (V^2 - 1/r) cos(gamma) / (r D V^2)
///   s'     = -cos(gamma) / (r D)
///
/// V is not a state: it follows from e and r. Bank angle appears nowhere; the tracker turns u into roll.
/// </summary>
public sealed class EntryDynamics
{
    public const int NX = 3;
    public const int IR = 0, IG = 1, IS = 2;

    /// <summary>Drag floor in the propagation, normalised, so a skip still advances in energy.</summary>
    public const double DragFloor = 1e-5;

    public EntryModel Model { get; }

    public EntryDynamics(EntryModel model) => Model = model;

    public double R0 => Model.BodyRadius;
    public double G0 => Model.G0;
    public double Vs => Model.SpeedUnit;

    public double Altitude(double r) => (r - 1.0) * R0;

    /// <summary>Normalised speed at an energy and radius. Zero if the pair is unreachable.</summary>
    public static double Speed(double e, double r)
    {
        double v2 = 2.0 * (1.0 / r - e);
        return v2 > 0 ? Math.Sqrt(v2) : 0.0;
    }

    public static double Energy(double r, double v) => 1.0 / r - 0.5 * v * v;

    /// <summary>Normalised drag at an energy and radius.</summary>
    public double Drag(double e, double r)
    {
        double v = Speed(e, r);
        return Model.DragAccel(Altitude(r), v * Vs) / G0;
    }

    /// <summary>Lift over drag at an energy and radius (at the Mach the pair implies).</summary>
    public double LiftToDrag(double e, double r)
    {
        double v = Speed(e, r);
        return Model.Ld(Model.Mach(Altitude(r), v * Vs));
    }

    /// <summary>dx/de.</summary>
    public void F(double e, ReadOnlySpan<double> x, double u, Span<double> dx)
    {
        double r = x[IR], g = x[IG];
        double v = Math.Max(Speed(e, r), 1e-4);
        double v2 = v * v;
        double d = Math.Max(Drag(e, r), DragFloor);
        double cg = Math.Cos(g);
        Model.RotationTerms(r, v, g, out double vRot, out double vGammaRot);
        // In the rotating frame de/dtau = V * (D - vRot), matching PredictStart's time-domain dynamics.
        double dEff = Math.Max(d - vRot, DragFloor);
        dx[IR] = Math.Sin(g) / dEff;
        dx[IG] = (u * d + (v2 - 1.0 / r) * cg / r + vGammaRot) / (dEff * v2);
        dx[IS] = -cg / (r * dEff);
    }

    /// <summary>
    /// Integrate from (e0, x0) to e1 with RK4, the control varying linearly from u0 to u1 (first-order hold).
    /// </summary>
    public void Propagate(double e0, double e1, ReadOnlySpan<double> x0, double u0, double u1, int subSteps, Span<double> x1)
    {
        Span<double> x = stackalloc double[NX];
        Span<double> k1 = stackalloc double[NX];
        Span<double> k2 = stackalloc double[NX];
        Span<double> k3 = stackalloc double[NX];
        Span<double> k4 = stackalloc double[NX];
        Span<double> t = stackalloc double[NX];
        x0.CopyTo(x);
        double h = (e1 - e0) / subSteps;
        for (int i = 0; i < subSteps; i++)
        {
            double ea = e0 + i * h;
            double fa = (double)i / subSteps, fm = (i + 0.5) / subSteps, fb = (i + 1.0) / subSteps;
            double ua = u0 + (u1 - u0) * fa, um = u0 + (u1 - u0) * fm, ub = u0 + (u1 - u0) * fb;
            F(ea, x, ua, k1);
            for (int j = 0; j < NX; j++) t[j] = x[j] + 0.5 * h * k1[j];
            F(ea + 0.5 * h, t, um, k2);
            for (int j = 0; j < NX; j++) t[j] = x[j] + 0.5 * h * k2[j];
            F(ea + 0.5 * h, t, um, k3);
            for (int j = 0; j < NX; j++) t[j] = x[j] + h * k3[j];
            F(ea + h, t, ub, k4);
            for (int j = 0; j < NX; j++) x[j] += h / 6.0 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
            // Keep the radius where the speed is real; a state past it means the plan left the atmosphere at escape energy.
            double rMax = 1.0 / Math.Max(ea + h, 1e-9) * 0.999999;
            if (x[IR] > rMax) x[IR] = rMax;
        }
        x.CopyTo(x1);
    }

    /// <summary>
    /// The altitude floor at an energy: the highest radius at which a path limit is reached. Heating, dynamic pressure and load all grow as the vehicle flies lower at a fixed energy, because density and speed both rise.
    /// </summary>
    public double FloorRadius(double e, EntryConfig cfg)
    {
        double rGround = 1.0 + cfg.AltitudeFloor / R0;
        double rAtmTop = 1.0 + Model.AtmosphereTop / R0;
        double rTop = e > 1e-9 ? Math.Min(rAtmTop, 1.0 / e * 0.999999) : rAtmTop;
        if (rTop <= rGround) return rGround;
        if (Violation(e, rGround, cfg) < 1.0) return rGround;
        if (Violation(e, rTop, cfg) >= 1.0) return rTop;

        // Scan down from the top so a bumpy drag curve gives the highest crossing, then bisect.
        const int scan = 64;
        double hi = rTop, lo = rTop;
        for (int i = 1; i <= scan; i++)
        {
            double r = rTop - (rTop - rGround) * i / scan;
            if (Violation(e, r, cfg) >= 1.0) { lo = r; break; }
            hi = r;
            lo = r;
        }
        for (int i = 0; i < 40; i++)
        {
            double mid = 0.5 * (lo + hi);
            if (Violation(e, mid, cfg) >= 1.0) lo = mid; else hi = mid;
        }
        return hi;
    }

    /// <summary>The worst path limit as a fraction of its maximum.</summary>
    public double Violation(double e, double r, EntryConfig cfg)
    {
        double h = Altitude(r);
        double v = Speed(e, r) * Vs;
        double worst = 0.0;
        if (cfg.HeatingRateMax > 0) worst = Math.Max(worst, Model.HeatingRate(h, v) / cfg.HeatingRateMax);
        if (cfg.DynamicPressureMax > 0) worst = Math.Max(worst, Model.DynamicPressure(h, v) / cfg.DynamicPressureMax);
        if (cfg.LoadFactorMax > 0) worst = Math.Max(worst, Model.LoadFactor(h, v) / cfg.LoadFactorMax);
        return worst;
    }

    /// <summary>
    /// The equilibrium-glide radius at an energy with vertical lift L/D cos(sigma): where that lift exactly balances gravity less the centrifugal term, u D = (1/r)(1/r - V^2). Above it the vehicle sinks even fully lifted.
    /// </summary>
    public double EquilibriumGlideRadius(double e, double cosBank, EntryConfig cfg)
    {
        double rGround = 1.0 + cfg.AltitudeFloor / R0;
        double rAtmTop = 1.0 + Model.AtmosphereTop / R0;
        double rTop = e > 1e-9 ? Math.Min(rAtmTop, 1.0 / e * 0.999999) : rAtmTop;
        double Balance(double r)
        {
            double v = Speed(e, r);
            Model.RotationTerms(r, v, 0.0, out _, out double vGammaRot);
            double need = (1.0 / r) * (1.0 / r - v * v) - vGammaRot;
            return LiftToDrag(e, r) * cosBank * Drag(e, r) - need;
        }
        if (Balance(rTop) >= 0) return rTop;
        if (Balance(rGround) <= 0) return rGround;
        double lo = rGround, hi = rTop;
        for (int i = 0; i < 50; i++)
        {
            double mid = 0.5 * (lo + hi);
            if (Balance(mid) > 0) lo = mid; else hi = mid;
        }
        return 0.5 * (lo + hi);
    }

    /// <summary>
    /// THE LEAST PEAK HEATING this craft can fly, W/m^2, and the speed it peaks at, m/s: the equilibrium glide with the plan's most lift up (L/D cos BankMinDeg), from energy e0 to ef. Nothing the plan flies sinks slower at any energy, and heating rises as the craft flies lower at an energy, so no plan's peak lies below this one. A heating limit under it cannot be held. Zero where the glide never meets the air.
    /// </summary>
    public (double PeakWm2, double SpeedMs) LeastPeakHeating(double e0, double ef, EntryConfig cfg)
    {
        double cosMin = Math.Cos(cfg.BankMinDeg * Math.PI / 180.0);
        double rTopAir = 1.0 + Model.AtmosphereTop / R0;
        // Above circular speed at the top of the air nothing holds the craft down to an equilibrium: start where the glide begins.
        e0 = Math.Max(e0, 0.5 / rTopAir);
        double peak = 0.0, peakV = 0.0;
        if (!(ef > e0))
            return (peak, peakV);
        const int n = 400;
        for (int i = 0; i <= n; i++)
        {
            double e = e0 + (ef - e0) * i / n;
            double r = EquilibriumGlideRadius(e, cosMin, cfg);
            double v = Speed(e, r) * Vs;
            double q = Model.HeatingRate(Altitude(r), v);
            if (q > peak) (peak, peakV) = (q, v);
        }
        return (peak, peakV);
    }

    /// <summary>
    /// Heat load per unit of energy, up to a constant: 1 / (C_D sqrt(rho)). From Q = integral 2 m k / (S C_D sqrt(rho)) dE, in which speed cancels.
    /// </summary>
    public double HeatPerEnergy(double e, double r)
    {
        double h = Altitude(r);
        double rho = Math.Max(Model.Rho(h), 1e-12);
        double cd = Math.Max(Model.DragCoefficient(Model.Mach(h, Speed(e, r) * Vs)), 1e-6);
        return 1.0 / (cd * Math.Sqrt(rho));
    }
}
