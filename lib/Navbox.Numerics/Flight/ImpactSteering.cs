namespace Navbox.Flight;

/// <summary>
/// Turning d(impact)/d(v) into a velocity correction that moves the landing point
/// toward a target.
///
/// TWO DIRECTIONS, AND THEY ARE NOT THE SAME ONE. Given the Jacobian J and a miss
/// vector m = impact - target:
///
///   steepest descent   dv = -J^T m        the GREEDY direction: the fastest
///                                         reduction in |m| per unit of |dv|. It is
///                                         a descent direction and nothing more.
///   Gauss-Newton       dv = -J^+ m        the direction that NULLS the miss to
///                                         first order. This is the one that points
///                                         at the target.
///
/// They coincide when the miss lies along a singular direction of J - which for this
/// geometry means a PURELY along-track or PURELY cross-track miss, both measuring
/// 0.0 degrees - and part company for anything in between: 13.6 degrees at 45, and
/// 9.5 degrees in the case --impact exercises. The cost is not the angle but the
/// residual. Judged FAIRLY - steepest descent given its own exact line search
/// alpha = |J^T m|^2 / |J J^T m|^2, both laws iterated to convergence - the greedy
/// law costs 204.44 m/s against 186.16, and four iterations against three. About 10%
/// more dv, not the factor of two an equal-magnitude comparison suggests. The bigger
/// practical difference is that the pseudo-inverse needs no step length at all, while
/// the greedy law needs a line search or a gain in units of 1/s^2 that has to be
/// retuned as the time of flight shrinks. See docs/impact-steering.md.
///
/// J IS RANK DEFICIENT, and for exactly ONE reason - which is not a constraint. The
/// predictor reports the point where the trajectory CROSSES the surface, and a
/// crossing point of a sphere is on that sphere, so |p| = R identically for every
/// initial state. A function whose values all lie on a sphere has a derivative landing
/// in that sphere's tangent plane: three velocity inputs onto two output directions.
///
/// Planarity is NOT a second reason, though an earlier version of this comment said so.
/// Two independent deficiencies would leave rank 1 and two zero singular values; there
/// is one. On an inclined, non-planar arc the singular values are still
/// (130.89, 101.66, 0). Planarity fixes WHERE the null direction points, not that
/// there is one.
///
/// So the third singular value is zero to machine precision and a plain inverse is
/// meaningless. What handles that is DAMPING, and nothing else:
///
///     dv = -J^T (J J^T + lambda^2 I)^-1 m
///
/// the Levenberg-Marquardt form, and an ORDINARY one - nothing in it is aware of the
/// sphere, the tangent plane or the rotating frame; it is applied to whatever J
/// arrives. As lambda -> 0 it becomes the plain Moore-Penrose pseudo-inverse. The only
/// reason lambda is not zero is that the exact right-inverse J^T (J J^T)^-1 needs full
/// row rank, which J does not have. No inverse is formed: the 3x3 system is solved and
/// the result multiplied by J^T. Each singular direction is scaled by
/// sigma^2/(sigma^2 + lambda^2), so at lambda = 1 against singular values of 129 and
/// 102 the useful directions pass through at 0.9999 and the null one is annihilated.
/// It degrades smoothly to steepest descent as lambda grows and to the exact
/// Gauss-Newton step as lambda falls.
///
/// THE TANGENT PROJECTION IS DEFENSIVE, NOT LOAD-BEARING, and an earlier version of
/// this comment claimed otherwise. The impact point is driven to |g| = TargetRadius
/// for every perturbed trajectory, so d|g|/dv is zero and the surface normal is a
/// LEFT NULL VECTOR of J - measured at |n^T J|/|J| = 1e-17. A radial miss component
/// is therefore annihilated by the solve on its own, and removing it first changes
/// the answer by 1e-14 m/s (both measured by --impact). It is kept because it costs
/// ten flops and stops being redundant the moment the target radius becomes
/// state-dependent - a terrain-following target would do it - but it is not what
/// makes the solve well behaved. Lambda is.
///
/// WHAT THIS IS NOT: guidance. It answers one narrow question - "what is the
/// smallest instantaneous velocity change that puts the impact on the target, to
/// first order" - and that is greedy in the sense that matters for propellant:
///
///   It is a LINEARISATION. One step removes 93.7% of a 21.5 km miss; iterating
///   re-linearises and converges 21.5 km -> 1.35 km -> 5 m -> 0. Converged, it costs
///   7.2% more dv than the first step, which is the useful number: a guidance cycle
///   can fly the greedy step and re-solve next cycle rather than iterating first.
///
///   It assumes an IMPULSE, applied NOW. A real boostback burn has duration, and the
///   position moves during it.
///
///   It has no cost over the trajectory - no gravity or drag loss during the burn, no
///   throttle or attitude limit - and, worst of all, no opinion about WHEN to burn.
///   The sensitivity varies along the coast, so the same miss costs different dv at
///   different times, and a law that only ever nulls the miss now cannot see that.
///
///   It DISCARDS THE NULL DIRECTION. Minimum norm puts zero component along J's null
///   space, but that direction is free with respect to the impact point - it is a
///   spare degree of freedom that a real law could spend on entry conditions or
///   attitude. Here it is treated as a nuisance rather than a resource.
///
/// So this is the arrow on an overlay and a reference implementation, not the
/// boostback law. Jo and Ahn's closed form is what encodes the finite-burn and fuel
/// structure; the sensitivities from ImpactPredictor.VelocityJacobian are an INPUT to
/// that machinery, and are the part this file exists to demonstrate.
/// </summary>
public static class ImpactSteering
{
    /// <summary>
    /// Default damping, in metres of impact movement per m/s of velocity change.
    ///
    /// The two useful singular values of J are of order 100 m per m/s and the third
    /// is zero, so a lambda of 1 leaves the well-conditioned directions essentially
    /// untouched while capping what the near-null direction can demand. Raise it to
    /// trade precision for a smaller, safer correction.
    /// </summary>
    public const double DefaultLambda = 1.0;

    /// <summary>
    /// Velocity corrections that move the predicted impact toward a target.
    /// </summary>
    /// <param name="dGroundDv">Row-major 3x3 from
    /// <see cref="ImpactPredictor.VelocityJacobian"/>: [i*3+j] = d(ground_i)/d(v_j).</param>
    /// <param name="miss">impact - target, in the same ground frame, metres.</param>
    /// <param name="surfaceNormal">Unit radial at the impact point. The miss is
    /// projected perpendicular to this before solving. Redundant as long as the
    /// target radius is fixed - see the type summary - so passing an empty span to
    /// skip it changes nothing measurable today.</param>
    /// <param name="lambda">Damping; see <see cref="DefaultLambda"/>.</param>
    /// <param name="deltaV">3 out: the damped Gauss-Newton correction, m/s. This is
    /// the one to fly.</param>
    /// <param name="steepest">3 out: -J^T m, the greedy direction, UNNORMALISED and
    /// not in m/s - it is a gradient, and its magnitude is metres squared per m/s.
    /// Supplied for comparison and diagnosis, not for steering. Pass an empty span
    /// to skip.</param>
    /// <returns>False if the system could not be solved, in which case the outputs
    /// are zeroed.</returns>
    public static bool Correction(ReadOnlySpan<double> dGroundDv, ReadOnlySpan<double> miss,
                                  ReadOnlySpan<double> surfaceNormal, double lambda,
                                  Span<double> deltaV, Span<double> steepest)
    {
        if (dGroundDv.Length < 9) throw new ArgumentException("Need a 3x3.", nameof(dGroundDv));
        if (miss.Length < 3) throw new ArgumentException("Need a 3-vector.", nameof(miss));
        if (deltaV.Length < 3) throw new ArgumentException("Need room for 3.", nameof(deltaV));

        deltaV[0] = deltaV[1] = deltaV[2] = 0.0;
        if (steepest.Length >= 3) steepest[0] = steepest[1] = steepest[2] = 0.0;

        for (int i = 0; i < 9; i++)
            if (!double.IsFinite(dGroundDv[i]))
                return false;

        // Drop the part of the miss that points out of the surface. There usually IS
        // one - the site sits at its terrain height and the impact at its own, so the
        // two radii differ - and no velocity change can fix it.
        //
        // The solve would discard it anyway: n is a left null vector of J, so both
        // J^T m and the damped solve annihilate the radial part on their own. This is
        // insurance against a future where the target radius varies with position,
        // not a correction to the present answer.
        Span<double> m = stackalloc double[3] { miss[0], miss[1], miss[2] };
        if (surfaceNormal.Length >= 3)
        {
            double nn = surfaceNormal[0] * surfaceNormal[0]
                      + surfaceNormal[1] * surfaceNormal[1]
                      + surfaceNormal[2] * surfaceNormal[2];
            if (nn > 1e-12)
            {
                double d = (m[0] * surfaceNormal[0] + m[1] * surfaceNormal[1]
                          + m[2] * surfaceNormal[2]) / nn;
                m[0] -= d * surfaceNormal[0];
                m[1] -= d * surfaceNormal[1];
                m[2] -= d * surfaceNormal[2];
            }
        }

        // steepest = -J^T m
        if (steepest.Length >= 3)
            for (int j = 0; j < 3; j++)
                steepest[j] = -(dGroundDv[0 * 3 + j] * m[0]
                              + dGroundDv[1 * 3 + j] * m[1]
                              + dGroundDv[2 * 3 + j] * m[2]);

        // A = J J^T + lambda^2 I, symmetric positive definite for lambda > 0.
        Span<double> a = stackalloc double[9];
        for (int i = 0; i < 3; i++)
            for (int k = 0; k < 3; k++)
            {
                double s = 0.0;
                for (int j = 0; j < 3; j++)
                    s += dGroundDv[i * 3 + j] * dGroundDv[k * 3 + j];
                a[i * 3 + k] = s + (i == k ? lambda * lambda : 0.0);
            }

        Span<double> y = stackalloc double[3];
        if (!Solve3(a, m, y))
            return false;

        // dv = -J^T y
        for (int j = 0; j < 3; j++)
            deltaV[j] = -(dGroundDv[0 * 3 + j] * y[0]
                        + dGroundDv[1 * 3 + j] * y[1]
                        + dGroundDv[2 * 3 + j] * y[2]);

        return double.IsFinite(deltaV[0]) && double.IsFinite(deltaV[1]) && double.IsFinite(deltaV[2]);
    }

    /// <summary>
    /// The FREE direction: the velocity change that moves the impact point nowhere.
    ///
    /// J maps three velocity components onto two impact directions, so exactly one
    /// combination of velocity produces no impact movement at all. Spending dv along
    /// it costs propellant and buys no targeting - which is why
    /// <see cref="Correction"/> puts zero there - but it is also a degree of freedom
    /// that is FREE with respect to the target. Anything you would like the burn to do
    /// besides hit the site (hold a flight path angle, set up an entry attitude) can be
    /// bought along this direction without disturbing where the vehicle lands.
    ///
    /// Found as the cross product of two rows of J, taking whichever pair is least
    /// nearly parallel. For a planar trajectory two of the three rows ARE nearly
    /// parallel - the in-plane pair - so picking blindly would produce noise.
    /// </summary>
    /// <param name="dGroundDv">Row-major 3x3 from
    /// <see cref="ImpactPredictor.VelocityJacobian"/>.</param>
    /// <param name="freeDir">3 out: unit vector, or zeroed if none could be found.</param>
    /// <returns>False if J is too close to full rank or to zero for the direction to
    /// mean anything, in which case there is no free steering to be had.</returns>
    public static bool FreeDirection(ReadOnlySpan<double> dGroundDv, Span<double> freeDir)
    {
        if (dGroundDv.Length < 9) throw new ArgumentException("Need a 3x3.", nameof(dGroundDv));
        if (freeDir.Length < 3) throw new ArgumentException("Need room for 3.", nameof(freeDir));

        freeDir[0] = freeDir[1] = freeDir[2] = 0.0;

        double scale = 0.0;
        for (int i = 0; i < 9; i++)
        {
            if (!double.IsFinite(dGroundDv[i])) return false;
            scale += dGroundDv[i] * dGroundDv[i];
        }
        scale = System.Math.Sqrt(scale);
        if (scale <= 0.0) return false;

        double best = 0.0;
        for (int a = 0; a < 3; a++)
            for (int b = a + 1; b < 3; b++)
            {
                double u0 = dGroundDv[a * 3 + 0], u1 = dGroundDv[a * 3 + 1], u2 = dGroundDv[a * 3 + 2];
                double w0 = dGroundDv[b * 3 + 0], w1 = dGroundDv[b * 3 + 1], w2 = dGroundDv[b * 3 + 2];
                double c0 = u1 * w2 - u2 * w1;
                double c1 = u2 * w0 - u0 * w2;
                double c2 = u0 * w1 - u1 * w0;
                double len = System.Math.Sqrt(c0 * c0 + c1 * c1 + c2 * c2);
                if (len > best)
                {
                    best = len;
                    freeDir[0] = c0 / len; freeDir[1] = c1 / len; freeDir[2] = c2 / len;
                }
            }

        // A cross product of two rows of a matrix whose entries are O(scale) is
        // O(scale^2) when the rows are independent. Well below that and the two rows
        // were parallel to within noise, so the "direction" is numerical dust.
        if (best < 1e-6 * scale * scale)
        {
            freeDir[0] = freeDir[1] = freeDir[2] = 0.0;
            return false;
        }
        return true;
    }

    /// <summary>3x3 solve by Gaussian elimination with partial pivoting. Small enough
    /// that a factorisation would be more code than it saves.</summary>
    private static bool Solve3(Span<double> a, ReadOnlySpan<double> b, Span<double> x)
    {
        Span<double> m = stackalloc double[12];
        for (int i = 0; i < 3; i++)
        {
            m[i * 4 + 0] = a[i * 3 + 0];
            m[i * 4 + 1] = a[i * 3 + 1];
            m[i * 4 + 2] = a[i * 3 + 2];
            m[i * 4 + 3] = b[i];
        }

        for (int c = 0; c < 3; c++)
        {
            int piv = c;
            for (int r = c + 1; r < 3; r++)
                if (System.Math.Abs(m[r * 4 + c]) > System.Math.Abs(m[piv * 4 + c]))
                    piv = r;
            if (System.Math.Abs(m[piv * 4 + c]) < 1e-300)
                return false;
            if (piv != c)
                for (int k = 0; k < 4; k++)
                    (m[c * 4 + k], m[piv * 4 + k]) = (m[piv * 4 + k], m[c * 4 + k]);

            for (int r = 0; r < 3; r++)
            {
                if (r == c) continue;
                double f = m[r * 4 + c] / m[c * 4 + c];
                for (int k = c; k < 4; k++)
                    m[r * 4 + k] -= f * m[c * 4 + k];
            }
        }

        for (int i = 0; i < 3; i++)
            x[i] = m[i * 4 + 3] / m[i * 4 + i];
        return true;
    }
}
