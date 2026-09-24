namespace AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// One ascent problem turned into what the solver works with: canonical units, the per-stage node mesh, and every constant the subproblem, the merit and the seed share. Built once per solve and read-only afterwards.
///
/// THE MESH IS THE SCRIPT'S. Each stage gets its own block of nodes evenly spaced in its own tau in [0, 1], stretched by that stage's free burn time sigma_i. The last node of one stage and the first of the next are the same instant, joined by the separation jump rather than by an integrated interval.
/// </summary>
internal sealed class AscentCase
{
    public const int NX = AscentDynamics.NX;
    public const int NU = AscentDynamics.NU;

    public readonly AscentProblem Problem;
    public readonly AscentSettings Settings;
    public readonly AscentDynamics Dyn;

    // Mesh.
    public readonly int S, N;
    public readonly int[] NodeStage, Starts, Ends;
    public readonly double[] Dtau;       // per interval; zero across a staging link
    public readonly int[] Coll;          // collocation intervals, in order
    public readonly int[] CollRow;       // interval -> virtual-control row, -1 for a staging link

    // Canonical constants.
    public readonly double[] X0 = new double[NX];
    public readonly double[] LiftoffDir = new double[3];
    public readonly double[] Nhat = new double[3];
    public readonly double[] E2 = new double[3];     // in-plane downrange at lift-off
    public readonly double RTarget, VTarget, RvTarget;
    public readonly double RFloor, MFloor;
    public readonly double[] PropC, DryC;
    public readonly double[] SigMin, SigMax;
    public readonly double[] ThrottleMin;
    public readonly double SigScale;
    public readonly double[] XScale = new double[NX];
    public readonly double[] DScale = new double[NX];
    public readonly double QMax, QAlphaMax;

    public AscentCase(AscentProblem p)
    {
        Problem = p;
        Settings = p.Settings;
        Dyn = new AscentDynamics(p);
        AscentSettings st = Settings;

        S = p.Stages.Length;
        int per = Math.Max(3, st.NodesPerStage);
        N = S * per;
        NodeStage = new int[N];
        Starts = new int[S];
        Ends = new int[S];
        for (int i = 0; i < S; i++)
        {
            Starts[i] = i * per;
            Ends[i] = i * per + per - 1;
            for (int k = Starts[i]; k <= Ends[i]; k++) NodeStage[k] = i;
        }

        Dtau = new double[N - 1];
        CollRow = new int[N - 1];
        var coll = new List<int>();
        for (int k = 0; k < N - 1; k++)
        {
            if (NodeStage[k] == NodeStage[k + 1])
            {
                // linspace(0, 1, per) differenced, exactly as the script's uniform mesh.
                int j = k - Starts[NodeStage[k]];
                Dtau[k] = (double)(j + 1) / (per - 1) - (double)j / (per - 1);
                CollRow[k] = coll.Count;
                coll.Add(k);
            }
            else
            {
                Dtau[k] = 0.0;
                CollRow[k] = -1;
            }
        }
        Coll = coll.ToArray();

        double lu = Dyn.LU, vu = Dyn.VU, tu = Dyn.TU, mu = Dyn.MU;
        for (int i = 0; i < 3; i++)
        {
            X0[i] = p.R0[i] / lu;
            X0[3 + i] = p.V0[i] / vu;
        }
        X0[6] = 1.0;

        double nn = Norm3(p.PlaneNormal);
        for (int i = 0; i < 3; i++) Nhat[i] = p.PlaneNormal[i] / nn;

        // Straight up: the script pins the first thrust along the lift-off AIR-RELATIVE velocity, not v0 itself, which on the pad is almost all co-rotation.
        double[] up = p.LiftoffDirection ?? AirVelocity(X0);
        double un = Norm3(up);
        for (int i = 0; i < 3; i++) LiftoffDir[i] = up[i] / un;

        // e2 = normalise(n x e1), e1 the lift-off radial: the in-plane prograde direction the seed kicks toward.
        double r0n = Norm3(X0);
        double e1x = X0[0] / r0n, e1y = X0[1] / r0n, e1z = X0[2] / r0n;
        double cx = Nhat[1] * e1z - Nhat[2] * e1y;
        double cy = Nhat[2] * e1x - Nhat[0] * e1z;
        double cz = Nhat[0] * e1y - Nhat[1] * e1x;
        double cn = Math.Sqrt(cx * cx + cy * cy + cz * cz);
        E2[0] = cx / cn; E2[1] = cy / cn; E2[2] = cz / cn;

        RTarget = p.TargetRadius / lu;
        VTarget = p.TargetSpeed / vu;
        RvTarget = p.TargetRadialRate / (lu * vu);

        RFloor = p.GroundRadius / lu;
        MFloor = p.FinalMassFloor / mu;

        PropC = new double[S];
        DryC = new double[S];
        SigMin = new double[S];
        SigMax = new double[S];
        ThrottleMin = new double[S];
        for (int i = 0; i < S; i++)
        {
            AscentStage stg = p.Stages[i];
            PropC[i] = stg.PropellantMass / mu;
            DryC[i] = i < S - 1 ? stg.JettisonMass / mu : 0.0;
            double full = stg.FullBurnTime;
ThrottleMin[i] = double.IsNaN(stg.ThrottleMin) ? st.ThrottleMin : stg.ThrottleMin;
            // The script's 20-700 s, widened only for a stage that could not otherwise burn its load: a long upper-stage burn, or one shorter than the floor.
            SigMin[i] = Math.Min(st.SigmaMinSeconds, 0.5 * full) / tu;
            SigMax[i] = Math.Max(st.SigmaMaxSeconds, 1.1 * full / ThrottleMin[i]) / tu;
        }
        SigScale = st.SigmaTrustSeconds / tu;

        for (int i = 0; i < 3; i++)
        {
            XScale[i] = st.PositionTrust;
            XScale[3 + i] = st.VelocityTrust;
            DScale[i] = st.PositionDefectScale;
            DScale[3 + i] = VTarget;
        }
        XScale[6] = 1.0;
        DScale[6] = 1.0;

        QMax = p.QMax;
        QAlphaMax = p.QAlphaMax;
    }

    /// <summary>Canonical air-relative velocity of a state.</summary>
    public double[] AirVelocity(ReadOnlySpan<double> x)
    {
        double w = Dyn.OmegaC;
        return [x[3] + w * x[1], x[4] - w * x[0], x[5]];
    }

    /// <summary>The five TRUE insertion residuals, canonical: |r| - R, |v| - V, r.v - (r.v)*, n.r, n.v.</summary>
    public void TerminalResidual(double[] x, Span<double> res)
    {
        int o = (N - 1) * NX;
        double rx = x[o], ry = x[o + 1], rz = x[o + 2];
        double vx = x[o + 3], vy = x[o + 4], vz = x[o + 5];
        res[0] = Math.Sqrt(rx * rx + ry * ry + rz * rz) - RTarget;
        res[1] = Math.Sqrt(vx * vx + vy * vy + vz * vz) - VTarget;
        res[2] = rx * vx + ry * vy + rz * vz - RvTarget;
        res[3] = Nhat[0] * rx + Nhat[1] * ry + Nhat[2] * rz;
        res[4] = Nhat[0] * vx + Nhat[1] * vy + Nhat[2] * vz;
    }

    public static double Norm3(ReadOnlySpan<double> v) => Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}
