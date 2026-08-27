using System;
using Brutal.Numerics;
using KSA;

// Deterministic check of the steering-direction -> CustomAttitudeTarget conversion,
// calling KSA's own quaternion functions (no game boot). For several sample geometries
// it reconstructs the orientation exactly as the flight computer will and confirms the
// resulting thrust axis points along the desired steering vector.
class Program
{
    const string KsaDir = @"C:\Program Files\Kitten Space Agency";

    static void Main()
    {
        AppDomain.CurrentDomain.AssemblyResolve += (_, a) =>
        {
            string p = System.IO.Path.Combine(KsaDir, new System.Reflection.AssemblyName(a.Name).Name + ".dll");
            return System.IO.File.Exists(p) ? System.Reflection.Assembly.LoadFrom(p) : null;
        };
        Run();
        RollReference();
    }

    static void Run()
    {
        var rng = new Random(1);
        double worstQuat = 1, worstAlign = 1;
        for (int i = 0; i < 8; i++)
        {
            double3 pos = Rand(rng);
            double3 steer = Rand(rng);
            // A representative frame2Cci (any orientation); use ECL builder with a random cce2Cci.
            doubleQuat cce2Cci = doubleQuat.Normalize(new doubleQuat(rng.NextDouble(), rng.NextDouble(), rng.NextDouble(), rng.NextDouble()));
            doubleQuat frame2Cci = VehicleReferenceFrameEx.GetEclBody2Cci(cce2Cci);

            float3 posDir = float3.Pack(double3.Normalize(pos));
            float3 steerDir = float3.Pack(double3.Normalize(steer));
            doubleQuat desired = BurnTarget.ComputeBurnBody2Cci(posDir, steerDir);

            double3 euler = doubleQuat.Concatenate(desired, doubleQuat.Inverse(frame2Cci)).ToRollYawPitchRadians();
            doubleQuat recon = doubleQuat.Concatenate(QuaternionEx.CreateFromRollYawPitchRadians(euler), frame2Cci);

            double qdot = Math.Abs(desired.X*recon.X + desired.Y*recon.Y + desired.Z*recon.Z + desired.W*recon.W);
            double3 thrustAxis = double3.Transform(double3.UnitX, recon);
            double align = double3.Dot(thrustAxis, double3.Normalize(steer));

            Console.WriteLine($"#{i}: quatMatch={qdot:F6}  thrustAxis.steer={align:F6}");
            worstQuat = Math.Min(worstQuat, qdot);
            worstAlign = Math.Min(worstAlign, align);
        }
        Console.WriteLine($"WORST quatMatch={worstQuat:F6} (want 1)  WORST align={worstAlign:F6} (want 1)");
        Console.WriteLine(worstQuat > 0.9999 && worstAlign > 0.9999 ? "PASS" : "FAIL");
    }

    // Mirrors PoweredGuidanceWindow.SteerBody2Cci — the ascent's replacement for
    // BurnTarget.ComputeBurnBody2Cci, which takes its roll reference from
    // cross(steer, position) and so has nothing to work with while the two are the
    // same vector. Kept in step by the first check below: where the stock one IS well
    // conditioned, the two must agree exactly.
    static doubleQuat SteerBody2Cci(double3 steerDir, double3 rollRef)
    {
        double3 x = double3.Normalize(steerDir);
        double3 y = rollRef - double3.Dot(rollRef, x) * x;
        y = y.Length() > 1e-9
            ? double3.Normalize(y)
            : double3.Normalize(double3.Cross(x, Math.Abs(x.Z) < 0.9 ? new double3(0, 0, 1) : new double3(1, 0, 0)));
        double3 z = double3.Cross(x, y);
        return doubleQuat.CreateFromRotationMatrix(new double4x4(
            x.X, x.Y, x.Z, 0.0,
            y.X, y.Y, y.Z, 0.0,
            z.X, z.Y, z.Z, 0.0,
            0.0, 0.0, 0.0, 1.0));
    }

    // The ascent's roll reference. Two things to prove:
    //
    //   1. Where the stock construction is well conditioned, the plane-referenced one
    //      is the SAME orientation — so nothing about the commanded attitude changes
    //      over the part of the ascent that was never the problem.
    //   2. Through the pitch-over it is CONTINUOUS, where the stock one is not: with
    //      the steering within a float epsilon of the position vector the cross
    //      product normalises to zero, KSA substitutes an arbitrary orthogonal
    //      direction, and the commanded roll jumps the moment the pitch program makes
    //      the cross product representable again.
    static void RollReference()
    {
        Console.WriteLine();
        var rng = new Random(7);

        // 1 - agreement away from the singularity
        double worst = 1;
        for (int i = 0; i < 8; i++)
        {
            double3 pos = double3.Normalize(Rand(rng));
            double3 steer = double3.Normalize(Rand(rng));
            if (Math.Abs(double3.Dot(pos, steer)) > 0.9) { i--; continue; }   // want them well apart

            doubleQuat stock = BurnTarget.ComputeBurnBody2Cci(float3.Pack(pos), float3.Pack(steer));
            doubleQuat ours = SteerBody2Cci(steer, double3.Cross(steer, pos));
            double qdot = Math.Abs(stock.X*ours.X + stock.Y*ours.Y + stock.Z*ours.Z + stock.W*ours.W);
            worst = Math.Min(worst, qdot);
        }
        Console.WriteLine($"roll ref: agrees with KSA away from the singularity, worst quatMatch={worst:F6} (want 1)");

        // 2 - continuity through the pitch-over. Straight up, then pitching over at
        // 1 deg/s toward the downrange direction, sampled every 10 ms of it.
        double3 up = double3.Normalize(new double3(0.62, 0.31, 0.72));       // some launch site
        double3 east = double3.Normalize(double3.Cross(new double3(0, 0, 1), up));
        double3 north = double3.Cross(up, east);
        double3 downrange = double3.Normalize(0.8 * east + 0.6 * north);
        double3 planeNormal = double3.Normalize(double3.Cross(downrange, up));  // -h, the ascent roll ref

        double worstStock = 0, worstOurs = 0;
        doubleQuat prevStock = default, prevOurs = default;
        for (int k = 0; k <= 300; k++)
        {
            double pitch = (90.0 - k * 0.01) * Math.PI / 180.0;               // 1 deg/s, 10 ms steps
            double3 steer = double3.Normalize(Math.Sin(pitch) * up + Math.Cos(pitch) * downrange);

            doubleQuat stock = BurnTarget.ComputeBurnBody2Cci(float3.Pack(up), float3.Pack(steer));
            doubleQuat ours = SteerBody2Cci(steer, planeNormal);
            if (k > 0)
            {
                worstStock = Math.Max(worstStock, Step(prevStock, stock));
                worstOurs = Math.Max(worstOurs, Step(prevOurs, ours));
            }
            prevStock = stock;
            prevOurs = ours;
        }
        Console.WriteLine($"roll ref: largest one-step attitude jump over the pitch-over — " +
                          $"KSA {worstStock:F1} deg, plane-referenced {worstOurs:F3} deg");

        // 3 - the LATCHED roll (mirrors PoweredGuidanceWindow.AscentRollRef): the
        // commanded attitude at engagement must have the roll the vehicle already has,
        // so nothing is asked to spin about its long axis at lift-off — and it must
        // still be continuous through the pitch-over afterwards.
        double worstRollErr = 0, worstLatchedStep = 0;
        for (int trial = 0; trial < 6; trial++)
        {
            // Some pad roll: the vehicle's body Y, perpendicular to its thrust axis.
            double padRoll = trial * 60.0 * Math.PI / 180.0;
            double3 y0 = double3.Normalize(planeNormal * -1.0);
            double3 z0 = double3.Cross(up, y0);
            double3 bodyY = Math.Cos(padRoll) * y0 + Math.Sin(padRoll) * z0;

            double phi0 = LatchRoll(up, planeNormal, bodyY);

            // At engagement: commanded body Y must BE the vehicle's body Y.
            double3 cmdY = RollRef(up, planeNormal, phi0);
            worstRollErr = Math.Max(worstRollErr,
                Math.Acos(Math.Clamp(double3.Dot(cmdY, bodyY), -1, 1)) * 180.0 / Math.PI);

            // ...and the pitch-over is still smooth with that offset applied.
            doubleQuat prev = default;
            for (int k = 0; k <= 300; k++)
            {
                double pitch = (90.0 - k * 0.01) * Math.PI / 180.0;
                double3 steer = double3.Normalize(Math.Sin(pitch) * up + Math.Cos(pitch) * downrange);
                doubleQuat q = SteerBody2Cci(steer, RollRef(steer, planeNormal, phi0));
                if (k > 0) worstLatchedStep = Math.Max(worstLatchedStep, Step(prev, q));
                prev = q;
            }
        }
        Console.WriteLine($"roll ref: latched to the pad roll — worst roll error at engage " +
                          $"{worstRollErr:F6} deg, worst step over the pitch-over {worstLatchedStep:F3} deg");

        Console.WriteLine(worst > 0.9999 && worstOurs < 0.05
                          && worstRollErr < 1e-4 && worstLatchedStep < 0.05 ? "PASS" : "FAIL");
    }

    // The plane-referenced frame perpendicular to the thrust axis, turned by a latched
    // roll offset — the vector AscentRollRef hands to SteerBody2Cci.
    static double3 RollRef(double3 steer, double3 planeNormal, double phi)
    {
        double3 x = double3.Normalize(steer);
        double3 baseRef = -planeNormal;
        double3 yRef = double3.Normalize(baseRef - double3.Dot(baseRef, x) * x);
        double3 zRef = double3.Cross(x, yRef);
        return Math.Cos(phi) * yRef + Math.Sin(phi) * zRef;
    }

    // The vehicle's own roll about the thrust axis, measured off that same frame.
    static double LatchRoll(double3 steer, double3 planeNormal, double3 bodyY)
    {
        double3 x = double3.Normalize(steer);
        double3 baseRef = -planeNormal;
        double3 yRef = double3.Normalize(baseRef - double3.Dot(baseRef, x) * x);
        double3 zRef = double3.Cross(x, yRef);
        return Math.Atan2(double3.Dot(bodyY, zRef), double3.Dot(bodyY, yRef));
    }

    // Angle between two orientations, in degrees.
    static double Step(doubleQuat a, doubleQuat b)
    {
        double d = Math.Abs(a.X*b.X + a.Y*b.Y + a.Z*b.Z + a.W*b.W);
        return 2.0 * Math.Acos(Math.Clamp(d, -1.0, 1.0)) * 180.0 / Math.PI;
    }

    static double3 Rand(Random r) => new double3(r.NextDouble()*2-1, r.NextDouble()*2-1, r.NextDouble()*2-1);
}
