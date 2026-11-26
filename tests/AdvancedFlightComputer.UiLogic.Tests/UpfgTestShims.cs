namespace Brutal.Numerics
{
    // Minimal double3 shim so pure UPFG guidance files compile and run in offline CI tests without game DLLs.
    public readonly struct double3
    {
        public readonly double X;
        public readonly double Y;
        public readonly double Z;

        public double3(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public double LengthSquared() => X * X + Y * Y + Z * Z;
        public double Length() => Math.Sqrt(LengthSquared());

        public static double Dot(double3 a, double3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

        public static double3 Cross(double3 a, double3 b) => new(
            a.Y * b.Z - a.Z * b.Y,
            a.Z * b.X - a.X * b.Z,
            a.X * b.Y - a.Y * b.X);

        public static double3 Normalize(double3 v)
        {
            double inv = 1.0 / v.Length();
            return new(v.X * inv, v.Y * inv, v.Z * inv);
        }

        public static double3 operator +(double3 a, double3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static double3 operator -(double3 a, double3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static double3 operator -(double3 v) => new(-v.X, -v.Y, -v.Z);
        public static double3 operator *(double3 v, double s) => new(v.X * s, v.Y * s, v.Z * s);
        public static double3 operator *(double s, double3 v) => new(v.X * s, v.Y * s, v.Z * s);
        public static double3 operator /(double3 v, double s) => new(v.X / s, v.Y / s, v.Z / s);
    }
}

namespace AdvancedFlightComputer.Features.Guidance
{
    internal static class StageThrottleLimits
    {
        internal readonly record struct Phase(double Minimum, bool FollowsThrottle, bool HasSolids);
    }
}
