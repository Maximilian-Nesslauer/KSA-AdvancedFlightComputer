namespace AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// The height of the centre of mass over the centre of the landing site at which an upright craft first meets ground that is not level.
/// The plan ends at rest with the lowest point of the craft on the height of the site centre. On a slope a foot that stands away from the thrust axis meets the ground before that, while the plan still brakes, and the speed left at that moment is sqrt(2 a h) for a braking acceleration a and an early contact by h. A craft whose minimum thrust is above its weight brakes at several times the local gravity to the end, so half a metre of early contact is a hard landing.
/// </summary>
public static class TouchdownContact
{
    /// <summary>A point of the craft that can meet the ground: its distance from the thrust axis and its depth under the centre of mass along that axis, both in metres.</summary>
    public readonly record struct FootPoint(double RadiusM, double DepthM);

    /// <summary>
    /// The rise of the ground over the height of the site centre, as the highest value on and inside each ring. Ring i has the radius (i + 1) * StepM.
    /// </summary>
    public sealed class GroundRise
    {
        public double StepM { get; }
        private readonly double[] _highestWithin;

        /// <param name="ringHighest">The highest terrain height on each ring, relative to the site centre. A value below zero counts as zero, because the centre is inside every ring.</param>
        public GroundRise(double stepM, ReadOnlySpan<double> ringHighest)
        {
            if (!(stepM > 0.0))
                throw new ArgumentException("The ring step must be positive.");
            StepM = stepM;
            _highestWithin = new double[ringHighest.Length];
            double highest = 0.0;
            for (int i = 0; i < ringHighest.Length; i++)
            {
                if (double.IsFinite(ringHighest[i]))
                    highest = Math.Max(highest, ringHighest[i]);
                _highestWithin[i] = highest;
            }
        }

        public double ReachM => _highestWithin.Length * StepM;

        /// <summary>The highest ground within the radius, from the ring at or outside it, so a point between two rings takes the outer one.</summary>
        public double Within(double radiusM)
        {
            if (_highestWithin.Length == 0 || !(radiusM > 0.0))
                return 0.0;
            int ring = Math.Min((int)Math.Ceiling(radiusM / StepM) - 1, _highestWithin.Length - 1);
            return _highestWithin[Math.Max(ring, 0)];
        }
    }

    /// <summary>
    /// How far above the height that the level-ground plan ends at the first contact comes, in metres, zero or more.
    /// Each point meets the ground when the centre of mass is its depth plus the ground under it above the site centre. The heading of the craft at the touchdown is not known, so each point takes the highest ground within its radius. That is the true value when a foot points up the slope and an upper bound otherwise, and the bound errs to a stop slightly above the ground.
    /// </summary>
    public static double EarlyContactM(ReadOnlySpan<FootPoint> points, GroundRise ground)
    {
        double lowest = double.NegativeInfinity, first = double.NegativeInfinity;
        foreach (FootPoint p in points)
        {
            if (!double.IsFinite(p.DepthM) || !double.IsFinite(p.RadiusM))
                continue;
            lowest = Math.Max(lowest, p.DepthM);
            first = Math.Max(first, p.DepthM + ground.Within(p.RadiusM));
        }
        return double.IsFinite(lowest) ? Math.Max(first - lowest, 0.0) : 0.0;
    }

    /// <summary>The vertical speed left when the ground comes earlyM before the end of a plan that brakes to rest at brakingMs2, in m/s.</summary>
    public static double SpeedAtEarlyContact(double earlyM, double brakingMs2)
        => earlyM > 0.0 && brakingMs2 > 0.0 ? Math.Sqrt(2.0 * brakingMs2 * earlyM) : 0.0;
}
