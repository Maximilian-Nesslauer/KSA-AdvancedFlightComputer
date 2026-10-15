using AdvancedFlightComputer.Guidance.Numerics;

namespace AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// 3dof.py's physics, line for line, so the solver can be diffed against the script: flat earth, constant gravity, an exponential atmosphere, lift from a single slope, an induced-drag polar and a Gaussian wave-drag bump.
///
/// Not a flight model. The guidance flies <c>KsaPointMassModel</c>; this exists so the port has an oracle, and so headless checks have a cheap model with the right shape.
///
///   vdot = g + (T b + D + L) / m,      T = throttle * Tmax
///   L    = q S Cl_a s p,               s = smooth sign(b . v_hat): lift opposes the tilt when flying retrograde
///   D    = -q S Cd v_hat,              Cd = (Cd0 + k Cl_a^2 |p|^2) (1 + A exp(-((M - Mp) / W)^2))
///   mdot = -T / (Isp g0)
/// </summary>
public sealed class Toy3DofModel : IPointMassModel
{
    public double Gz { get; init; } = -9.81;
    public double Rho0 { get; init; } = 1.225;
    public double ScaleHeight { get; init; } = 8.5e3;
    public double Area { get; init; } = 10.8;
    public double Cd0 { get; init; } = 0.8;
    public double InducedK { get; init; } = 0.5;
    public double ClAlpha { get; init; } = 2.0;
    public double Isp { get; init; } = 282.0;
    public double G0 { get; init; } = 9.81;
    public double Tmax { get; init; } = 845000.0;

    // The placeholder wave drag.
    public double Gamma { get; init; } = 1.4;
    public double GasConstant { get; init; } = 287.0;
    public double MachPeak { get; init; } = 1.1;
    public double MachWidth { get; init; } = 0.3;
    public double MachAmplitude { get; init; } = 1.25;

    public void F(ReadOnlySpan<Dual> x, ReadOnlySpan<Dual> u, Span<Dual> xdot)
    {
        Dual vx = x[PointMass3Dof.IV + 0], vy = x[PointMass3Dof.IV + 1], vz = x[PointMass3Dof.IV + 2];
        Dual m = x[PointMass3Dof.IM];
        Dual h = x[PointMass3Dof.IR + 2];
        Dual bx = u[PointMass3Dof.IB + 0], by = u[PointMass3Dof.IB + 1], bz = u[PointMass3Dof.IB + 2];
        Dual thrust = u[PointMass3Dof.ITH] * Tmax;

        Dual rho = Rho0 * Dual.Exp(-h / ScaleHeight);
        Dual vmag2 = vx * vx + vy * vy + vz * vz;
        Dual vmag = Dual.Sqrt(vmag2 + PointMass3Dof.VEps2);
        Dual hx = vx / vmag, hy = vy / vmag, hz = vz / vmag;
        Dual q = 0.5 * rho * vmag2;

        // ISA temperature with the tropopause floor, as the script's jnp.maximum.
        Dual temperature = 288.15 - 0.0065 * h;
        if (temperature.V < 216.65)
            temperature = new Dual(216.65);
        Dual sound = Math.Sqrt(Gamma * GasConstant) * Dual.Sqrt(temperature);
        Dual mach = vmag / sound;

        Dual bv = bx * hx + by * hy + bz * hz;
        Dual px = bx - bv * hx, py = by - bv * hy, pz = bz - bv * hz;
        Dual s = bv / Dual.Sqrt(bv * bv + 1e-6);
        Dual pp = px * px + py * py + pz * pz;

        Dual lift = q * Area * ClAlpha * s;
        Dual dm = (mach - MachPeak) / MachWidth;
        Dual cd = (Cd0 + InducedK * ClAlpha * ClAlpha * pp) * (1.0 + MachAmplitude * Dual.Exp(-(dm * dm)));
        Dual drag = q * Area * cd;

        xdot[PointMass3Dof.IR + 0] = vx;
        xdot[PointMass3Dof.IR + 1] = vy;
        xdot[PointMass3Dof.IR + 2] = vz;
        xdot[PointMass3Dof.IV + 0] = (thrust * bx - drag * hx + lift * px) / m;
        xdot[PointMass3Dof.IV + 1] = (thrust * by - drag * hy + lift * py) / m;
        xdot[PointMass3Dof.IV + 2] = Gz + (thrust * bz - drag * hz + lift * pz) / m;
        xdot[PointMass3Dof.IM] = -(thrust / (Isp * G0));
    }

    public double DynamicPressure(ReadOnlySpan<double> x)
    {
        double v2 = x[PointMass3Dof.IV] * x[PointMass3Dof.IV] + x[PointMass3Dof.IV + 1] * x[PointMass3Dof.IV + 1]
                  + x[PointMass3Dof.IV + 2] * x[PointMass3Dof.IV + 2];
        return 0.5 * Rho0 * Math.Exp(-x[PointMass3Dof.IR + 2] / ScaleHeight) * v2;
    }
}
