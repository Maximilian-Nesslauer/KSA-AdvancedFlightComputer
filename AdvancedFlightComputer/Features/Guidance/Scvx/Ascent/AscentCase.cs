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
    public readonly bool[] IsCoast;
    public readonly int[] NodeStage, Starts, Ends;
    public readonly double[] Dtau;       // per interval; zero across a staging link
    public readonly int[] Coll;          // collocation intervals, in order
    public readonly int[] CollRow;       // interval -> virtual-control row, -1 for a staging link

    // Canonical constants.
    public readonly double[] X0 = new double[NX];
    public readonly double[] LiftoffDir = new double[3];
    public readonly double[][] VerticalDirections;
    public readonly double PadVerticalTime;
    public readonly bool PadRollFits = true;
    public readonly double PadTurnStep;
    /// <summary>The target plane's normal at <see cref="PlaneTurn0"/>, the turn the solve starts from: what the seed flies into and kicks along.</summary>
    public readonly double[] Nhat = new double[3];
    /// <summary>The problem's own plane normal, unturned. <see cref="PlaneNormal"/> turns it.</summary>
    public readonly double[] N0 = new double[3];
    /// <summary>The plane turn the solve starts from: none, or the nearest the bounds allow. See AscentProblem.PlaneTurnMin.</summary>
    public readonly double PlaneTurn0, PlaneTurnMin, PlaneTurnMax;
    public readonly double[] E2 = new double[3];     // in-plane downrange at lift-off
    public readonly double RTarget, VTarget, RvTarget;
    public readonly double RFloor, MFloor;
    public readonly double[] PropC, DryC;
    public readonly double[] SigMin, SigMax;
    /// <summary>Canonical burn time of a stage whose duration is pinned, NaN where it is free.</summary>
    public readonly double[] SigFixed;
    public readonly double[] ThrottleMin;
    public readonly double[] PhysicalThrottleMin;
    /// <summary>Each stage's thrust acceleration limit, canonical, positive infinity where it has none or where its full thrust cannot reach it.</summary>
    public readonly double[] AccC;
    /// <summary>The nodes of the stages with a finite <see cref="AccC"/>, in order.</summary>
    public readonly int[] LimitedNodes;
    /// <summary>The liquid throttle the seed flies each stage at.</summary>
    public readonly double[] SeedThrottle;
    /// <summary>
    /// The propellant constraints, one per run of stages that share a liquid load: the first and last stage of the run, the canonical propellant the run burns, and whether that is exact or a ceiling. A run is exact when a free burn time ends it - its liquid load runs dry - and it is not the last; a ceiling when it is the last, or when a pinned burn ends it and its liquid can come in under the load. A run with no liquid at all needs no row: its solids' drop is fixed by the dynamics.
    /// </summary>
    public readonly (int First, int Last, double PropC, bool Exact)[] Loads;

    /// <summary>
    /// One booster reserve the solve chooses: the boosters leaving at the end of <see cref="Stage"/> keep it (AscentProblem.Recoveries).
    /// R0C is the reserve the stage list already holds back, canonical; the solve's variable is the change from it, between RMinC and RMaxC: none at all, to the same 98 % of the boosters' liquid load UPFG's reserve stops at. Share is the boosters' share of the stage's propellant flow (AscentRecovery.FlowShare): one, or below one for side boosters beside a core that burns on.
    /// </summary>
    public readonly record struct Reserve(int Stage, double R0C, double RMinC, double RMaxC, double Share);

    /// <summary>One booster brought home: the reserve it keeps (an index into <see cref="Reserves"/>), and its need from where that reserve's stage ends, as the oracle prices it for its own site.</summary>
    public readonly record struct Need(int Reserve, BoostbackOracle Oracle);

    /// <summary>The reserves, one per stage that boosters leave from, in stage order; empty when no booster is brought home.</summary>
    public readonly Reserve[] Reserves = [];
    /// <summary>The boosters brought home, one per entry of AscentProblem.Recoveries and in its order.</summary>
    public readonly Need[] Needs = [];

    /// <summary>The first reserve's stage, -1 for none: the single-booster view the warm start uses.</summary>
    public int RecStage => Reserves.Length > 0 ? Reserves[0].Stage : -1;
    /// <summary>Each node's solids, at its fixed second of its stage.</summary>
    public readonly SolidPoint[] NodeSolid;
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
        IsCoast = new bool[S];
        int perBurn = Math.Max(3, st.NodesPerStage);
        int perCoast = Math.Max(5, st.CoastNodes);
        Starts = new int[S];
        Ends = new int[S];
        int totalNodes = 0;
        for (int i = 0; i < S; i++)
        {
            IsCoast[i] = p.Stages[i].IsCoast;
            int cnt = IsCoast[i] ? perCoast : perBurn;
            Starts[i] = totalNodes;
            Ends[i] = totalNodes + cnt - 1;
            totalNodes += cnt;
        }
        N = totalNodes;
        NodeStage = new int[N];
        for (int i = 0; i < S; i++)
            for (int k = Starts[i]; k <= Ends[i]; k++) NodeStage[k] = i;

        Dtau = new double[N - 1];
        CollRow = new int[N - 1];
        var coll = new List<int>();
        for (int k = 0; k < N - 1; k++)
        {
            if (NodeStage[k] == NodeStage[k + 1])
            {
                // linspace(0, 1, cnt) differenced, matching the uniform per-stage mesh.
                int stIdx = NodeStage[k];
                int cnt = Ends[stIdx] - Starts[stIdx] + 1;
                int j = k - Starts[stIdx];
                Dtau[k] = (double)(j + 1) / (cnt - 1) - (double)j / (cnt - 1);
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
        for (int i = 0; i < 3; i++) N0[i] = p.PlaneNormal[i] / nn;
        PlaneTurnMin = p.PlaneTurnMin;
        PlaneTurnMax = p.PlaneTurnMax;
        PlaneTurn0 = Math.Clamp(0.0, PlaneTurnMin, PlaneTurnMax);
        PlaneNormal(PlaneTurn0).CopyTo(Nhat, 0);

        // The pad roll follows the local vertical, which is also the usual direction of air-relative lift-off velocity.
        double[] up = p.PadRoll != null ? X0[..3] : p.LiftoffDirection ?? AirVelocity(X0);
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
        SigFixed = new double[S];
        ThrottleMin = new double[S];
        PhysicalThrottleMin = new double[S];
        SeedThrottle = new double[S];
        AccC = new double[S];
        for (int i = 0; i < S; i++)
        {
            AscentStage stg = p.Stages[i];
            PropC[i] = stg.PropellantMass / mu;
            DryC[i] = i < S - 1 ? stg.JettisonMass / mu : 0.0;
            if (stg.IsCoast)
            {
                ThrottleMin[i] = 0.0;
                PhysicalThrottleMin[i] = 0.0;
                SeedThrottle[i] = 0.0;
                AccC[i] = double.PositiveInfinity;
                SigFixed[i] = double.NaN;
                SigMin[i] = stg.CoastMinSeconds / tu;
                SigMax[i] = stg.CoastMaxSeconds / tu;
                continue;
            }
            double full = stg.FullBurnTime;
            ThrottleMin[i] = double.IsNaN(stg.ThrottleMin) ? st.ThrottleMin : stg.ThrottleMin;
            PhysicalThrottleMin[i] = stg.PhysicalThrottleMin;
            SeedThrottle[i] = Math.Max(stg.SeedThrottle, Math.Max(ThrottleMin[i], PhysicalThrottleMin[i]));
            AccC[i] = stg.AccelerationLimit / Dyn.AU;
            SigFixed[i] = stg.IsPinned ? stg.FixedBurnTime / tu : double.NaN;
            if (stg.IsPinned)
            {
                SigMin[i] = SigMax[i] = SigFixed[i];
            }
            else
            {
                // The script's 20-700 s, widened only for a stage that could not otherwise burn its load: a long upper-stage burn, or one shorter than the floor.
                SigMin[i] = Math.Min(st.SigmaMinSeconds, 0.5 * full) / tu;
                SigMax[i] = Math.Max(st.SigmaMaxSeconds, 1.1 * full / ThrottleMin[i]) / tu;
            }
        }
        // Relax SigMin on burn stages adjacent to a coast so short pre-coast or circularisation burns are not blocked by SigmaMinSeconds.
        for (int i = 0; i < S; i++)
            if (!IsCoast[i] && !p.Stages[i].IsPinned && ((i > 0 && IsCoast[i - 1]) || (i < S - 1 && IsCoast[i + 1])))
                SigMin[i] = Math.Min(SigMin[i], 1.0 / tu);
        SigScale = st.SigmaTrustSeconds / tu;

        var loads = new List<(int, int, double, bool)>();
        var stageRunFirst = new int[S];
        var stageRunProp = new double[S];
        for (int first = 0; first < S;)
        {
            int last = first;
            while (last < S - 1 && p.Stages[last].LiquidCarriesOver) last++;
            double prop = 0.0;
            bool liquid = false;
            for (int i = first; i <= last; i++)
            {
                prop += PropC[i];
                liquid |= p.Stages[i].HasLiquid;
            }
            for (int i = first; i <= last; i++)
            {
                stageRunFirst[i] = first;
                stageRunProp[i] = prop;
            }
            if (liquid)
                loads.Add((first, last, prop, last < S - 1 && !p.Stages[last].IsPinned));
            first = last + 1;
        }
        Loads = loads.ToArray();

        // A limit binds only on a stage whose highest liquid thrust over its lightest mass passes the backed-off limit, so every other stage drops it and adds nothing to the subproblem.
        // For carry-over runs, accumulate the combined propellant across the chain to widen both minMass and SigMax for every burn half in the run.
        var stageStartMass = new double[S];
        double mStart = X0[AscentDynamics.IM];
        for (int i = 0; i < S; i++)
        {
            stageStartMass[i] = mStart;
            mStart -= PropC[i] + DryC[i];
        }
        for (int i = 0; i < S; i++)
        {
            if (IsCoast[i])
                continue;
            int first = stageRunFirst[i];
            double runProp = stageRunProp[i];
            double dryBefore = 0.0;
            for (int j = first; j < i; j++) dryBefore += DryC[j];
            double mHigh = Math.Max(stageStartMass[first] - dryBefore, stageStartMass[i]);
            double minMass = Math.Max(stageStartMass[first] - runProp - dryBefore, MFloor);
            if (!p.Stages[i].IsPinned && runProp > PropC[i] && p.Stages[i].HasLiquid)
            {
                double fullCombined = runProp / Dyn.MassFlowC(i);
                SigMax[i] = Math.Max(SigMax[i], 1.1 * fullCombined / ThrottleMin[i]);
            }
            if (double.IsFinite(AccC[i]))
            {
                double peak = PeakLiquidThrustC(p.Stages[i]);
                if (peak <= AccC[i] * st.PathBackoff * minMass)
                    AccC[i] = double.PositiveInfinity;
                else if (!p.Stages[i].IsPinned)
                    SigMax[i] = Math.Max(SigMax[i], 1.1 * LimitedBurnTime(i, mHigh, runProp, 1.0, peak) / ThrottleMin[i]);
            }
        }

        var limited = new List<int>();
        for (int k = 0; k < N; k++)
            if (double.IsFinite(AccC[NodeStage[k]]))
                limited.Add(k);
        LimitedNodes = limited.ToArray();

        if (p.Recoveries.Count > 0)
        {
            // One reserve per stage the boosters leave from, in stage order; the problem has checked that boosters sharing a stage agree on it.
            var reserves = new List<Reserve>();
            foreach (AscentRecovery rec in p.Recoveries.OrderBy(r => r.Stage))
            {
                if (reserves.Count > 0 && reserves[^1].Stage == rec.Stage)
                    continue;
                int s = rec.Stage;
                double r0 = rec.InitialReserveKg / mu;
                // The boosters' own load is their share of the stage's, with what they already keep added back.
                double rMax = Math.Max(0.0, 0.98 * (rec.FlowShare * PropC[s] + r0) - r0);
                reserves.Add(new Reserve(s, r0, -r0, rMax, rec.FlowShare));
                // A side booster's core burns its share of the reserve in the next stage, so that stage may run longer than its load as built.
                if (rec.FlowShare < 1.0 && !p.Stages[s + 1].IsPinned && PropC[s + 1] > 0.0)
                {
                    double extra = (rMax + r0) * (1.0 / rec.FlowShare - 1.0);
                    SigMax[s + 1] *= (PropC[s + 1] + extra) / PropC[s + 1];
                }
            }
            Reserves = reserves.ToArray();
            Needs = p.Recoveries.Select(rec => new Need(reserves.FindIndex(r => r.Stage == rec.Stage), new BoostbackOracle(p, rec))).ToArray();
        }

        NodeSolid = new SolidPoint[N];
        for (int i = 0; i < S; i++)
        {
            AscentSolidBurn? table = p.Stages[i].Solid;
            if (table == null)
            {
                for (int k = Starts[i]; k <= Ends[i]; k++) NodeSolid[k] = SolidPoint.None;
                continue;
            }
            // Each node at its fixed second of the stage. The collocation integrates the mass flow by trapezoid over the nodes, so it is scaled to make that trapezoid burn exactly the table's grain: otherwise the stage's drop and its propellant disagree by the trapezoid's error, which only the virtual control could absorb.
            double seconds = p.Stages[i].FixedBurnTime;
            int cnt = Ends[i] - Starts[i] + 1;
            int np = table.Pressures;
            var flow = new double[cnt];
            var rows = new double[cnt][];
            double trap = 0.0;
            for (int j = 0; j < cnt; j++)
            {
                double t = seconds * j / (cnt - 1);
                flow[j] = table.MassFlowAt(t);
                rows[j] = new double[np];
                table.ThrustRow(t, rows[j]);
                if (j > 0) trap += 0.5 * (flow[j - 1] + flow[j]) * seconds / (cnt - 1);
            }
            double burned = table.Propellant(0.0, seconds);
            double scale = trap > 0.0 ? burned / trap : 1.0;
            for (int j = 0; j < cnt; j++)
                NodeSolid[Starts[i] + j] = SolidPoint.AtNode(flow[j] * scale * tu / mu, table.PressureGrid, rows[j]);
        }
        Dyn.BindNodes(NodeStage, NodeSolid);

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

        int lastVertical = 0;
        if (p.PadRoll is { } roll)
        {
            double dt = p.Stages[0].FixedBurnTime / (Ends[0] - Starts[0]);
            double clear = AscentSeed.TimeToClearPad(this, roll.ClearanceM);
            double deadline = clear + roll.MaxSeconds;
            PadRollFits = deadline <= p.Stages[0].FixedBurnTime;
            if (PadRollFits)
            {
                // Include the first node at or after the deadline so interpolation stays vertical through the roll.
                lastVertical = (int)Math.Min(Ends[0], Math.Ceiling(deadline / dt));
                PadVerticalTime = lastVertical * dt / tu;
                // The squared thrust difference includes four times the product of both throttles times the squared half-angle sine.
                double turn = Math.Min(Math.PI, roll.MaxTurnRateDegS * (Math.PI / 180.0) * dt);
                PadTurnStep = 2.0 * ThrottleMin[0] * Math.Sin(0.5 * turn);
            }
        }
        VerticalDirections = new double[lastVertical + 1][];
        VerticalDirections[0] = LiftoffDir;
        for (int k = 1; k <= lastVertical; k++)
        {
            VerticalDirections[k] = new double[3];
            PadVertical(k * PadVerticalTime / lastVertical, VerticalDirections[k]);
        }
    }

    // The pad's zenith rotates in the inertial frame during the hold.
    public void PadVertical(double t, Span<double> direction)
    {
        (double sin, double cos) = Math.SinCos(Dyn.OmegaC * t);
        direction[0] = cos * LiftoffDir[0] - sin * LiftoffDir[1];
        direction[1] = sin * LiftoffDir[0] + cos * LiftoffDir[1];
        direction[2] = LiftoffDir[2];
    }

    public bool PadRollControlsMet(ReadOnlySpan<double> u)
    {
        if (Problem.PadRoll == null)
            return true;
        if (!PadRollFits || u.Length < N * NU)
            return false;
        const double tolerance = 1e-6;
        for (int k = 0; k < VerticalDirections.Length; k++)
        {
            ReadOnlySpan<double> thrust = u.Slice(k * NU, NU);
            double[] up = VerticalDirections[k];
            double parallel = thrust[0] * up[0] + thrust[1] * up[1] + thrust[2] * up[2];
            if (!(parallel > 0.0))
                return false;
            double x = thrust[0] - parallel * up[0];
            double y = thrust[1] - parallel * up[1];
            double z = thrust[2] - parallel * up[2];
            if (!(x * x + y * y + z * z <= tolerance * tolerance * parallel * parallel))
                return false;
        }
        double stepLimit = PadTurnStep + tolerance;
        for (int k = 0; k < Ends[0]; k++)
        {
            int j = k * NU;
            double x = u[j + NU] - u[j], y = u[j + NU + 1] - u[j + 1], z = u[j + NU + 2] - u[j + 2];
            if (!(x * x + y * y + z * z <= stepLimit * stepLimit))
                return false;
        }
        return true;
    }

    /// <summary>Per-node control trust-box scale, normalized to the local throttle cap on high-TWR stages.</summary>
    public double ControlTrustScale(int node, ReadOnlySpan<double> x)
    {
        int s = NodeStage[node];
        if (!Settings.ScaleControlTrust || IsCoast[s])
            return 1.0;
        double uCap = LimitedThrottle(s, x);
        if (Problem.Stages[s].HasLiquid)
        {
            double full = Dyn.LiquidThrustC(s, x);
            double m = Math.Max(x[AscentDynamics.IM], 1e-6);
            double aFull = full / m;
            if (aFull > 6.0)
                uCap = Math.Min(uCap, 6.0 / aFull);
        }
        return Math.Clamp(uCap, 0.02, 1.0);
    }

    /// <summary>Per-stage duration trust-box scale, widened on long ballistic coasts.</summary>
    public double StageSigScale(int stage)
        => IsCoast[stage] ? Math.Max(SigScale, 0.5 * (SigMax[stage] - SigMin[stage])) : SigScale;

    /// <summary>Combined canonical propellant in the carry-over chain starting at <paramref name="stage"/>.</summary>
    public double RunPropellantFrom(int stage)
    {
        double prop = PropC[stage];
        int s = stage;
        while (s < S - 1 && Problem.Stages[s].LiquidCarriesOver)
        {
            s++;
            prop += PropC[s];
        }
        return prop;
    }

    /// <summary>The most throttle a stage's acceleration limit leaves it at a state, held at the subproblem's back-off, and 1 for a stage with no limit or no liquid thrust.</summary>
    public double LimitedThrottle(int stage, ReadOnlySpan<double> x)
    {
        if (!double.IsFinite(AccC[stage]))
            return 1.0;
        double full = Dyn.LiquidThrustC(stage, x);
        return full > 0.0 ? Math.Min(1.0, AccC[stage] * Settings.PathBackoff * x[AscentDynamics.IM] / full) : 1.0;
    }

    /// <summary>
    /// The liquid throttle the seed flies a stage at from a state. That is its seed throttle, or less where the seed throttle would push past the stage's acceleration limit, held at the same back-off the subproblem holds the limit to.
    /// </summary>
    public double SeedThrottleAt(int stage, ReadOnlySpan<double> x)
        => IsCoast[stage] ? 0.0 : Math.Max(PhysicalThrottleMin[stage], Math.Min(SeedThrottle[stage], LimitedThrottle(stage, x)));

    /// <summary>
    /// The seed's burn time for a stage's whole load from mass <paramref name="m0"/>, canonical.
    /// It integrates the full-throttle, acceleration-limited and hardware-floor portions using reference thrust.
    /// </summary>
    public double SeedBurnTime(int stage, double m0)
        => IsCoast[stage] ? Problem.Stages[stage].FullBurnTime / Dyn.TU
            : LimitedBurnTime(stage, m0, PropC[stage], SeedThrottle[stage], Dyn.ThrustRefC(stage));

    /// <summary>The seed's burn time for an arbitrary canonical propellant amount <paramref name="propC"/> from mass <paramref name="m0"/>.</summary>
    public double SeedBurnTimeForProp(int stage, double m0, double propC)
        => LimitedBurnTime(stage, m0, propC, SeedThrottle[stage], Dyn.ThrustRefC(stage));

    // The same burn time at throttle theta with the canonical liquid thrust T for canonical propellant propC.
    private double LimitedBurnTime(int stage, double m0, double propC, double theta, double thrust)
    {
        double flow = Dyn.MassFlowC(stage);
        double atFull = propC / (flow * theta);
        if (!double.IsFinite(AccC[stage]))
            return atFull;
        double a = AccC[stage] * Settings.PathBackoff;
        double m1 = m0 - propC;
        double mStar = theta * thrust / a;
        if (!(m1 > 0.0) || mStar <= m1)
            return atFull;
        double physical = PhysicalThrottleMin[stage];
        double mFloorStart = physical * thrust / a;
        if (mFloorStart >= m0)
            return propC / (flow * physical);
        double mLimitedStart = Math.Min(mStar, m0);
        double mLimitedEnd = Math.Max(mFloorStart, m1);
        double tau = thrust / (flow * a);
        double tail = mLimitedEnd > m1 ? (mLimitedEnd - m1) / (flow * physical) : 0.0;
        return (m0 - mLimitedStart) / (flow * theta) + tau * Math.Log(mLimitedStart / mLimitedEnd) + tail;
    }

    // The liquid engines' highest full thrust, canonical: the reference thrust or the most the pressure table gives.
    private double PeakLiquidThrustC(AscentStage stage)
    {
        double peak = stage.Thrust;
        if (stage.ThrustAtPressure != null)
            foreach (double t in stage.ThrustAtPressure)
                peak = Math.Max(peak, t);
        return peak / Dyn.FU;
    }

    /// <summary>Canonical air-relative velocity of a state.</summary>
    public double[] AirVelocity(ReadOnlySpan<double> x)
    {
        double w = Dyn.OmegaC;
        return [x[3] + w * x[1], x[4] - w * x[0], x[5]];
    }

    /// <summary>The target plane's normal turned about the spin axis (z) by <paramref name="turn"/>, radians, east positive.</summary>
    public double[] PlaneNormal(double turn)
    {
        double c = Math.Cos(turn), s = Math.Sin(turn);
        return [c * N0[0] - s * N0[1], s * N0[0] + c * N0[1], N0[2]];
    }

    /// <summary>The five TRUE insertion residuals, canonical: |r| - R, |v| - V, r.v - (r.v)*, n.r, n.v, with the plane turned by <paramref name="turn"/>.</summary>
    public void TerminalResidual(double[] x, double turn, Span<double> res)
    {
        double[] nh = PlaneNormal(turn);
        int o = (N - 1) * NX;
        double rx = x[o], ry = x[o + 1], rz = x[o + 2];
        double vx = x[o + 3], vy = x[o + 4], vz = x[o + 5];
        res[0] = Math.Sqrt(rx * rx + ry * ry + rz * rz) - RTarget;
        res[1] = Math.Sqrt(vx * vx + vy * vy + vz * vz) - VTarget;
        res[2] = rx * vx + ry * vy + rz * vz - RvTarget;
        res[3] = nh[0] * rx + nh[1] * ry + nh[2] * rz;
        res[4] = nh[0] * vx + nh[1] * vy + nh[2] * vz;
    }

    /// <summary>A booster's separation, SI, from a trajectory: position and velocity at the last node of <paramref name="stage"/>, and the time since lift-off, the burns up to it.</summary>
    public void Separation(int stage, double[] x, double[] sig, double[] r, double[] v, out double t)
    {
        int o = Ends[stage] * NX;
        for (int i = 0; i < 3; i++)
        {
            r[i] = x[o + i] * Dyn.LU;
            v[i] = x[o + 3 + i] * Dyn.VU;
        }
        t = 0.0;
        for (int s = 0; s <= stage; s++)
            t += sig[s] * Dyn.TU;
    }

    /// <summary>Each booster's need at a trajectory's separations (<see cref="Needs"/>), with its slopes when <paramref name="gradient"/>; <paramref name="impulses"/> warm-starts each from an earlier answer, or null.</summary>
    public BoostbackOracle.Need[] EvaluateNeeds(double[] x, double[] sig, BoostbackOracle.Need[]? impulses, bool gradient)
    {
        var needs = new BoostbackOracle.Need[Needs.Length];
        double[] r = new double[3], v = new double[3];
        for (int j = 0; j < Needs.Length; j++)
        {
            Separation(Reserves[Needs[j].Reserve].Stage, x, sig, r, v, out double t);
            double[]? warm = impulses != null && j < impulses.Length && impulses[j].Ok ? impulses[j].Impulse : null;
            needs[j] = Needs[j].Oracle.Evaluate(r, v, t, warm, gradient);
        }
        return needs;
    }

    public static double Norm3(ReadOnlySpan<double> v) => Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}
