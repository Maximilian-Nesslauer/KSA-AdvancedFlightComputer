using AdvancedFlightComputer.Guidance.Numerics;

namespace AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// The landing engines as the game runs them: thrust over throttle and ambient pressure, and mass flow over throttle, sampled from the game once and fitted to cubic splines.
///
/// THE NOZZLE LOSS IS NOT A CONSTANT. throttle * F_vac - Pa A_e is exact at full throttle and wrong below it, because KSA's nozzle loses less to the air as the chamber pressure falls. On the 2026-10-08 booster (3822 kN vacuum, 2895 kN at sea level) the 10 % floor gives 201 kN at sea level where the constant loss says -545 kN: the planner thought the booster could throttle below its weight, asked ~30 % throttle for the floor's thrust, and expected three times the propellant the engines burn, so every burn re-solve was refused on the mass channel. The loss is not proportional to the pressure either - at half the sea-level pressure it is still ~0.82 of the sea-level loss - so no exit area fits.
///
/// SAMPLED RATHER THAN MODELLED, because the game's nozzle passes through several regimes and none is a closed form worth fitting. The splines are C2 with analytic slopes, as the aero tables are, so the linearisation sees a smooth law, and outside the sampled box they extend linearly.
///
/// THE FLOW FOLLOWS THE THROTTLE, not the thrust, and does not depend on the air: the chamber is what the throttle sets.
///
/// Immutable once built, and reads nothing the game owns, so a solve on a worker thread can share it.
/// </summary>
public sealed class EngineThrustTable
{
    private readonly CubicBSplineNd _thrust;
    private readonly CubicBSplineNd _flow;

    /// <summary>The lowest throttle sampled - the engines' minimum.</summary>
    public double ThrottleMin { get; }

    /// <param name="pressureGridPa">Strictly increasing ambient pressures, Pa.</param>
    /// <param name="throttleGrid">Strictly increasing throttles, the engines' minimum to 1.</param>
    /// <param name="thrustN">Row-major, pressure slowest: thrustN[i * throttleGrid.Length + j] at pressureGridPa[i] and throttleGrid[j].</param>
    /// <param name="massFlowKgS">The flow at each throttle breakpoint, kg/s.</param>
    public EngineThrustTable(double[] pressureGridPa, double[] throttleGrid, double[] thrustN, double[] massFlowKgS)
    {
        ArgumentNullException.ThrowIfNull(pressureGridPa);
        ArgumentNullException.ThrowIfNull(throttleGrid);
        ArgumentNullException.ThrowIfNull(thrustN);
        ArgumentNullException.ThrowIfNull(massFlowKgS);
        if (thrustN.Length != pressureGridPa.Length * throttleGrid.Length)
            throw new ArgumentException($"Expected {pressureGridPa.Length} x {throttleGrid.Length} thrusts, got {thrustN.Length}.", nameof(thrustN));
        if (massFlowKgS.Length != throttleGrid.Length)
            throw new ArgumentException($"Expected {throttleGrid.Length} flows, got {massFlowKgS.Length}.", nameof(massFlowKgS));
        // A NaN here would reach SCS, which is native and does not check its input; caught where the message can still say which sample.
        for (int i = 0; i < thrustN.Length; i++)
            if (!double.IsFinite(thrustN[i]))
                throw new ArgumentException($"thrust[{i}] is not finite ({thrustN[i]}).", nameof(thrustN));
        for (int i = 0; i < massFlowKgS.Length; i++)
            if (!double.IsFinite(massFlowKgS[i]))
                throw new ArgumentException($"flow[{i}] is not finite ({massFlowKgS[i]}).", nameof(massFlowKgS));

        ThrottleMin = throttleGrid[0];
        _thrust = CubicBSplineNd.Fit([(double[])pressureGridPa.Clone(), (double[])throttleGrid.Clone()],
                                     (double[])thrustN.Clone(), 1, EdgeMode.Linear);
        _flow = CubicBSplineNd.Fit((double[])throttleGrid.Clone(), (double[])massFlowKgS.Clone(), EdgeMode.Linear);
    }

    /// <summary>Thrust at a throttle and ambient pressure, N.</summary>
    public double Thrust(double throttle, double pressure)
    {
        Span<double> point = stackalloc double[2] { pressure, throttle };
        Span<double> value = stackalloc double[1];
        _thrust.Evaluate(point, value);
        return value[0];
    }

    /// <summary>Thrust with its slopes in throttle (N) and in pressure (N/Pa).</summary>
    public double Thrust(double throttle, double pressure, out double dThrottle, out double dPressure)
    {
        Span<double> point = stackalloc double[2] { pressure, throttle };
        Span<double> value = stackalloc double[1];
        Span<double> gradient = stackalloc double[2];
        _thrust.EvaluateWithGradient(point, value, gradient);
        dPressure = gradient[0];
        dThrottle = gradient[1];
        return value[0];
    }

    public Dual Thrust(Dual throttle, Dual pressure)
    {
        if (throttle.D == 0.0 && pressure.D == 0.0)
            return new Dual(Thrust(throttle.V, pressure.V), 0.0);
        double value = Thrust(throttle.V, pressure.V, out double dThrottle, out double dPressure);
        return new Dual(value, dThrottle * throttle.D + dPressure * pressure.D);
    }

    /// <summary>Mass flow at a throttle, kg/s.</summary>
    public double MassFlow(double throttle)
    {
        Span<double> point = stackalloc double[1] { throttle };
        Span<double> value = stackalloc double[1];
        _flow.Evaluate(point, value);
        return value[0];
    }

    public Dual MassFlow(Dual throttle)
    {
        if (throttle.D == 0.0)
            return new Dual(MassFlow(throttle.V), 0.0);
        Span<double> point = stackalloc double[1] { throttle.V };
        Span<double> value = stackalloc double[1];
        Span<double> gradient = stackalloc double[1];
        _flow.EvaluateWithGradient(point, value, gradient);
        return new Dual(value[0], gradient[0] * throttle.D);
    }

    /// <summary>
    /// The throttle that gives a thrust at a pressure, unclamped: <see cref="Thrust(double, double)"/>'s inverse, by Newton from the straight line through zero and full throttle. The thrust rises with the throttle across the table and along its linear extension, so the iteration has one root to find.
    /// </summary>
    public double ThrottleFor(double thrust, double pressure)
    {
        double full = Thrust(1.0, pressure);
        double u = full > 0.0 ? thrust / full : 1.0;
        for (int i = 0; i < 30; i++)
        {
            double t = Thrust(u, pressure, out double slope, out _);
            if (!(slope > 0.0))
                break;
            double step = (thrust - t) / slope;
            u += step;
            if (Math.Abs(step) < 1e-10)
                break;
        }
        return u;
    }
}
