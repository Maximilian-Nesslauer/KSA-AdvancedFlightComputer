namespace Scvx;

/// <summary>
/// Vehicle limits and SCvx weights for the 6-DOF subproblem.
///
/// Defaults mirror 6dof.py so the C# result can be diffed against the Python
/// oracle. They are NOT flight values — for flight these must come from the
/// vehicle and the target body (thrust and Isp from the staging model, inertia
/// and mass live, gravity from the body being landed on, gimbal limit from the
/// actual parts). See the constants-drift guard in python_ref/loop_ref.py: the
/// scenario travels through loop_ref.csv, but everything here is hand-mirrored
/// and will silently desynchronise if only one side is edited.
/// </summary>
public sealed class Scvx6DofConfig
{
    public int Nodes { get; init; } = 30;

    public double Tmax { get; init; } = 6.0e6;
    public double ThrottleFloor { get; init; } = 0.40;          // Tmin = floor * Tmax
    public double GimbalMaxDeg { get; init; } = 10.0;
    public double TauRollMax { get; init; } = 1.0e5;
    public double TiltMaxDeg { get; init; } = 30.0;
    public double GroundFloor { get; init; } = -1.0;            // X[n,2] >= this

    public double RhoVc { get; init; } = 1e5;                   // virtual-control penalty
    public double WDu { get; init; } = 0.2;                     // control-rate smoothing
    public double WW { get; init; } = 1.0;                      // angular-rate damping
    public double SigmaMin { get; init; } = 5.0;
    public double SigmaMax { get; init; } = 25.0;
    public double SigmaScale { get; init; } = 12.0;

    public double[] XScale { get; init; } =
        [100, 100, 300, 50, 50, 50, 1, 1, 1, 1, 1, 1, 1, 250000.0];
    public double[]? UScale { get; init; }                      // defaults from Tmax/gimbal

    public double Tmin => ThrottleFloor * Tmax;
    public double TanGimbal => Math.Tan(GimbalMaxDeg * Math.PI / 180.0);
    public double CosTilt => Math.Cos(TiltMaxDeg * Math.PI / 180.0);

    public double[] ResolvedUScale => UScale ??
        [Tmax * TanGimbal, Tmax * TanGimbal, Tmax, TauRollMax];
}
