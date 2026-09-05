namespace Gfold;

// Problem parameters for the G-FOLD powered-descent solve.
//
// Coordinate convention (matches the reference Python): x is UP (altitude),
// y/z span the horizontal plane, gravity acts along -x, and the landing
// target Rf is the origin of the frame.
//
// Defaults are "Numerical Example 1" from the reference implementation
// (G-FOLD-Python Static_Solution/GFOLD_Static_Parms.py — the original
// paper's Mars case) so results can be compared against it directly.
public sealed record GfoldParams
{
    public const double G0 = 9.80665;

    public double GravityMag { get; init; } = 3.71;   // m/s^2, along -x
    public double DryMass { get; init; } = 2000;      // kg
    public double FuelMass { get; init; } = 300;      // kg
    public double Isp { get; init; } = 203.94;        // s
    public double ThrustMax { get; init; } = 24000;   // N, all engines
    public double ThrottleMin { get; init; } = 0.2;   // r1 = min*Tmax
    public double ThrottleMax { get; init; } = 0.8;   // r2 = max*Tmax
    public double VMax { get; init; } = 90;           // m/s
    public double GlideSlopeDeg { get; init; } = 30;  // min approach elevation
    public double PointingMaxDeg { get; init; } = 45; // max thrust tilt from up

    public double[] R0 { get; init; } = [2400, 2000, 0];
    public double[] V0 { get; init; } = [-40, 30, 0];
    public double[] Rf { get; init; } = [0, 0, 0];
    public double[] Vf { get; init; } = [0, 0, 0];

    public double WetMass => DryMass + FuelMass;
    public double Alpha => 1.0 / (Isp * G0);          // mass flow per thrust
    public double R1 => ThrottleMin * ThrustMax;
    public double R2 => ThrottleMax * ThrustMax;

    // Bounds on the time of flight from the reference: below tf_min the
    // vehicle cannot brake in time even at full thrust; above tf_max the
    // minimum throttle burns all fuel before touchdown.
    public double TfMin => DryMass * Math.Sqrt(V0.Sum(v => v * v)) / R2;
    public double TfMax => FuelMass / (Alpha * R1);
}
