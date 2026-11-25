namespace AdvancedFlightComputer.Guidance.Numerics.Flight;

/// <summary>
/// Aero moments moved to another centre of mass, so one sweep serves a craft whose centre of mass has moved
/// (KsaAeroSweep's cache). With d = c0 - c1, the old centre of mass less the new, a force F at any point has the moment
/// M1 = M0 + d x F about the new one. In coefficients - moment per q S L, force per q S, the same S and L -
/// C_M1 = C_M0 + d x C_F / L, node by node.
/// </summary>
public static class AeroMomentMove
{
    /// <summary>
    /// A body-axis moment table's values (BodyAeroTable.Values: three per node, C_l C_m C_n) moved by d = c0 - c1,
    /// with the force table's values on the same grid beside it (C_x C_y C_z per node). Null where the two do not match.
    /// </summary>
    public static double[]? Moved(double[] force, double[] moment, double dx, double dy, double dz, double length)
    {
        if (force == null || moment == null || force.Length != moment.Length || moment.Length % 3 != 0 || !(length > 0.0))
            return null;
        var moved = new double[moment.Length];
        for (int i = 0; i < moment.Length; i += 3)
        {
            double fx = force[i], fy = force[i + 1], fz = force[i + 2];
            moved[i] = moment[i] + (dy * fz - dz * fy) / length;
            moved[i + 1] = moment[i + 1] + (dz * fx - dx * fz) / length;
            moved[i + 2] = moment[i + 2] + (dx * fy - dy * fx) / length;
        }
        return moved;
    }

    /// <summary>A body-axis moment table moved by d = c0 - c1 (see <see cref="Moved(double[], double[], double, double, double, double)"/>); the table as it was where either is missing or they do not match.</summary>
    public static BodyAeroTable? Moved(BodyAeroTable? force, BodyAeroTable? moment, double dx, double dy, double dz, double length)
    {
        if (force == null || moment == null)
            return moment;
        double[]? moved = Moved(force.Values, moment.Values, dx, dy, dz, length);
        return moved == null ? moment : new BodyAeroTable(moment.MachGrid, moment.AlphaGridDeg, moment.BetaGridDeg, moved, moment.Frame, moment.Sphere);
    }

    /// <summary>
    /// Belly-first trim rows' pitch moment (KsaAeroSweep.BellySweep: C_D, C_L, C_m per Mach row and angle of attack,
    /// Mach-major) moved by d = c0 - c1. On belly axis b (body axes, square to x), at angle of attack a the air comes
    /// from u = cos(a) x + sin(a) b and lift is along l = sin(a) x - cos(a) b, so the force in that plane is
    /// C_F = -C_D u + C_L l; the pitch moment is about b x x, which a side force does not change:
    /// C_m1 = C_m0 + ((d x C_F) . (b x x)) / L.
    /// </summary>
    public static double[]? MovedTrimCm(double[] cd, double[] cl, double[] cm, double[] alphaDeg,
                                        double bx, double by, double bz, double dx, double dy, double dz, double length)
    {
        if (cd == null || cl == null || cm == null || alphaDeg == null || alphaDeg.Length == 0
            || cd.Length != cm.Length || cl.Length != cm.Length || cm.Length % alphaDeg.Length != 0 || !(length > 0.0))
            return null;
        // b x x, with x = (1, 0, 0).
        double px = 0.0, py = bz, pz = -by;
        int na = alphaDeg.Length;
        var moved = new double[cm.Length];
        for (int cell = 0; cell < cm.Length; cell++)
        {
            double a = alphaDeg[cell % na] * Math.PI / 180.0;
            double ca = Math.Cos(a), sa = Math.Sin(a);
            double ux = ca + sa * bx, uy = sa * by, uz = sa * bz;
            double lx = sa - ca * bx, ly = -ca * by, lz = -ca * bz;
            double fx = -cd[cell] * ux + cl[cell] * lx;
            double fy = -cd[cell] * uy + cl[cell] * ly;
            double fz = -cd[cell] * uz + cl[cell] * lz;
            double mx = dy * fz - dz * fy, my = dz * fx - dx * fz, mz = dx * fy - dy * fx;
            moved[cell] = cm[cell] + (mx * px + my * py + mz * pz) / length;
        }
        return moved;
    }
}
