namespace PoweredGuidance.Numerics;

/// <summary>
/// An ODE right-hand side, written in <see cref="Dual"/> so that integrating it
/// propagates derivatives alongside the state.
///
/// IMPLEMENT THIS ON A STRUCT, and mark the method <c>readonly</c>. Both matter:
/// the struct constraint on <see cref="Rk4"/> lets the JIT devirtualise and inline
/// the call, so a stage costs no more than writing the derivative inline would; and
/// <c>readonly</c> stops the <c>in</c> parameter from being defensively copied on
/// every one of the four stages. A class here would work and would quietly cost an
/// interface dispatch per stage per step.
/// </summary>
public interface IOdeSystem
{
    /// <summary>
    /// dx/dt at (t, x). Must write exactly x.Length entries.
    ///
    /// Everything is <see cref="Dual"/>, including t, so a caller can seed any input
    /// - a state component, a parameter folded into the system, or the time itself -
    /// and read that column of the Jacobian out of the integrated result.
    /// </summary>
    void Derivative(Dual t, ReadOnlySpan<Dual> x, Span<Dual> dx);
}

/// <summary>
/// Classical fourth-order Runge-Kutta, generic over <see cref="IOdeSystem"/> and
/// carrying <see cref="Dual"/>s throughout.
///
/// WHY THE WHOLE INTEGRATOR IS DUAL rather than a double integrator with a separate
/// sensitivity pass: propagating the derivative THROUGH the integrator gives the
/// exact sensitivity of the discrete solution the caller actually gets, not of the
/// continuous solution it approximates. Those differ by the truncation error, and
/// the difference is precisely what makes a finite-difference Jacobian noisy at
/// small steps. It also means there is one implementation to be right rather than
/// two that have to agree - the variational equations do not have to be derived,
/// written down, or kept in step with the dynamics when the dynamics change.
///
/// The cost is that the value-only path also carries a zero derivative through every
/// operation, roughly doubling its flops. For the problems this exists for - six
/// states, a few hundred steps - that is microseconds, and it buys the property that
/// a Jacobian is one seed away rather than a rewrite away.
///
/// SEEDING. To differentiate the trajectory with respect to an initial condition,
/// seed that component of x0 with <see cref="Dual.Seed"/> and leave the rest as
/// plain values; every integrated state then carries d(state)/d(that input). One
/// sweep per input column, exactly as Dynamics6Dof.Jacobian sweeps the dynamics.
/// </summary>
public static class Rk4
{
    /// <summary>Scratch entries needed per state component: k1..k4 and one stage temporary.</summary>
    public const int ScratchPerState = 5;

    /// <summary>
    /// One RK4 step of size h from (t, x) into xNext.
    ///
    /// The caller owns the scratch so that a stepping loop allocates nothing at all -
    /// stackalloc inside would be a stack allocation per step, which is the CA2014
    /// trap. It must be at least <see cref="ScratchPerState"/> * x.Length long.
    /// xNext may alias neither x nor scratch.
    /// </summary>
    public static void Step<TSys>(in TSys sys, Dual t, ReadOnlySpan<Dual> x, Dual h,
                                  Span<Dual> xNext, Span<Dual> scratch)
        where TSys : struct, IOdeSystem
    {
        int n = x.Length;
        if (xNext.Length < n)
            throw new ArgumentException("xNext is shorter than the state.", nameof(xNext));
        if (scratch.Length < ScratchPerState * n)
            throw new ArgumentException(
                $"scratch must be at least {ScratchPerState} * {n} = {ScratchPerState * n} long.",
                nameof(scratch));

        Span<Dual> k1 = scratch.Slice(0, n);
        Span<Dual> k2 = scratch.Slice(n, n);
        Span<Dual> k3 = scratch.Slice(2 * n, n);
        Span<Dual> k4 = scratch.Slice(3 * n, n);
        Span<Dual> tmp = scratch.Slice(4 * n, n);

        Dual half = h * 0.5;

        sys.Derivative(t, x, k1);
        for (int i = 0; i < n; i++) tmp[i] = x[i] + half * k1[i];

        sys.Derivative(t + half, tmp, k2);
        for (int i = 0; i < n; i++) tmp[i] = x[i] + half * k2[i];

        sys.Derivative(t + half, tmp, k3);
        for (int i = 0; i < n; i++) tmp[i] = x[i] + h * k3[i];

        sys.Derivative(t + h, tmp, k4);

        Dual sixth = h * (1.0 / 6.0);
        for (int i = 0; i < n; i++)
            xNext[i] = x[i] + sixth * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    /// <summary>
    /// Fixed-step integration from t0 to t0 + duration, in whole steps of at most
    /// maxStep, with the last step shortened to land exactly on the end time.
    ///
    /// For anything that has to stop on a CONDITION rather than a time - hitting the
    /// ground, crossing an altitude - step manually and refine the crossing, which is
    /// what ImpactPredictor does. Rolling that into here would mean guessing at the
    /// event semantics for every future caller.
    /// </summary>
    public static void Integrate<TSys>(in TSys sys, Dual t0, ReadOnlySpan<Dual> x0,
                                       double duration, double maxStep,
                                       Span<Dual> x, Span<Dual> scratch)
        where TSys : struct, IOdeSystem
    {
        if (!(maxStep > 0.0))
            throw new ArgumentOutOfRangeException(nameof(maxStep), maxStep, "Step must be positive.");

        int n = x0.Length;
        x0.CopyTo(x);
        if (duration == 0.0)
            return;

        int steps = (int)System.Math.Ceiling(System.Math.Abs(duration) / maxStep);
        double h = duration / steps;

        // One buffer to ping-pong into, so a step never writes its own input.
        Span<Dual> other = scratch.Slice(ScratchPerState * n, n);
        Span<Dual> work = scratch.Slice(0, ScratchPerState * n);

        Dual t = t0;
        bool inX = true;
        for (int s = 0; s < steps; s++)
        {
            if (inX) Step(in sys, t, x, h, other, work);
            else Step(in sys, t, other, h, x, work);
            inX = !inX;
            t += h;
        }
        if (!inX)
            other.CopyTo(x);
    }

    /// <summary>Scratch length <see cref="Integrate"/> needs for an n-state system.</summary>
    public static int IntegrateScratch(int n) => (ScratchPerState + 1) * n;
}
