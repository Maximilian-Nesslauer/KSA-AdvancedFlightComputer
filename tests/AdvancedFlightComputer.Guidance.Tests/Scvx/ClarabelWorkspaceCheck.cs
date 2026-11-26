using AdvancedFlightComputer.Guidance.Conic;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// Unit checks for ClarabelWorkspace recovery on in-place updates, backend-bound
/// EffectiveIterationCap reporting, and deterministic IDisposable cleanup.
/// </summary>
internal static class ClarabelWorkspaceCheck
{
    public static int Run()
    {
        bool prevIr = ClarabelWorkspace.IterativeRefinementDefault;
        bool prevPersistent = ClarabelWorkspace.Persistent;
        ScvxConicBackend prevBackend = ScvxConic.Backend;

        try
        {
            CheckRebuildAfterUpdateFailureWithIrEnabled();
            CheckFreshRebuildDoesNotDoubleRebuildOnMaxIterations();
            CheckEffectiveIterationCapStaysBoundToLastSolve();
            CheckDeterministicDispose();
            CheckToleranceCapAndPresolveDisabled();
            CheckSolveMsSamplesRingBuffer();
            Console.WriteLine("PASS clarabel-workspace");
            return 0;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine("FAIL clarabel-workspace: " + ex);
            return 1;
        }
        finally
        {
            ClarabelWorkspace.IterativeRefinementDefault = prevIr;
            ClarabelWorkspace.Persistent = prevPersistent;
            ScvxConic.Backend = prevBackend;
        }
    }

    private static (CcsAssembler A, CcsAssembler P, double[] b, double[] c) BuildTinyQp(double a00, double a11)
    {
        // Minimize 0.5 * (x0^2 + x1^2) - (x0 + x1) subject to:
        //   a00 * x0 + s0 = a00 * 2,  s0 >= 0  (x0 <= 2)
        //   a11 * x1 + s1 = a11 * 2,  s1 >= 0  (x1 <= 2)
        var A = new CcsAssembler(rows: 2, cols: 2);
        A.Add(0, 0, a00);
        A.Add(1, 1, a11);
        A.Freeze();

        var P = new CcsAssembler(rows: 2, cols: 2);
        P.Add(0, 0, 1.0);
        P.Add(1, 1, 1.0);
        P.Freeze();

        return (A, P, [a00 * 2.0, a11 * 2.0], [-1.0, -1.0]);
    }

    private static void CheckRebuildAfterUpdateFailureWithIrEnabled()
    {
        ClarabelWorkspace.IterativeRefinementDefault = true;
        ClarabelWorkspace.Persistent = true;
        ClarabelWorkspace.ResetTimers();

        using var ws = new ClarabelWorkspace();
        var (A, P, b, c) = BuildTinyQp(1.0, 1.0);

        // First solve builds the workspace with a tight cap of 1 iteration so MaxIterations triggers.
        ScsStatus st1 = ws.Solve(A, b, c, P, zeroDim: 0, posDim: 2, socDims: [], warmStart: false,
                                 maxIterations: 1, epsAbs: 1e-9, epsRel: 1e-9);
        Require(ws.LastRebuilt, "first solve must build the workspace");
        Require(ws.HitIterationLimit && st1 == ScsStatus.SolvedInaccurate, "cap=1 must report HitIterationLimit");
        Require(ClarabelWorkspace.TotalRebuilds == 1, $"fresh rebuild hitting MaxIterations must not rebuild twice (got {ClarabelWorkspace.TotalRebuilds})");
        Require(ClarabelWorkspace.TotalRefinementFallbacks == 0, "IR was already enabled, fallback count must stay 0");

        // Second solve has the exact same structure and cap: it enters Update(), hits MaxIterations,
        // and must trigger one self-healing Rebuild() even though iterative refinement is already true.
        ScsStatus st2 = ws.Solve(A, b, c, P, zeroDim: 0, posDim: 2, socDims: [], warmStart: false,
                                 maxIterations: 1, epsAbs: 1e-9, epsRel: 1e-9);
        Require(ws.LastRebuilt, "failed Update() must trigger a self-healing Rebuild() even when IR is true");
        Require(ws.HitIterationLimit && st2 == ScsStatus.SolvedInaccurate, "cap=1 recovery solve stays bounded");
        Require(ClarabelWorkspace.TotalRebuilds == 2, $"expected 2 total rebuilds after update recovery (got {ClarabelWorkspace.TotalRebuilds})");
        Require(ClarabelWorkspace.TotalRefinementFallbacks == 0, "IR fallback count must stay 0 when IR is already true");

        // Third solve with normal budget rebuilds on cap change and solves cleanly, then updates in place.
        ScsStatus st3 = ws.Solve(A, b, c, P, zeroDim: 0, posDim: 2, socDims: [], warmStart: false,
                                 maxIterations: 100, epsAbs: 1e-9, epsRel: 1e-9);
        Require(st3 == ScsStatus.Solved && ws.LastRebuilt, "cap change must rebuild and solve");

        ScsStatus st4 = ws.Solve(A, b, c, P, zeroDim: 0, posDim: 2, socDims: [], warmStart: false,
                                 maxIterations: 100, epsAbs: 1e-9, epsRel: 1e-9);
        Require(st4 == ScsStatus.Solved && !ws.LastRebuilt, "converged Update() must not rebuild");

        // Extreme scaling shift between solves while IR is already enabled: Update() reuses frozen equilibration,
        // so recovery must rebuild with fresh equilibration and keep the solution accurate.
        A.BeginRefill();
        A.Add(0, 0, 1e-8);
        A.Add(1, 1, 1e8);
        A.EndRefill();
        double[] bShift = [2e-8, 2e8];
        ScsStatus st5 = ws.Solve(A, bShift, c, P, zeroDim: 0, posDim: 2, socDims: [], warmStart: false,
                                 maxIterations: 100, epsAbs: 1e-9, epsRel: 1e-9);
        Require(st5.IsUsable() && !ws.HitIterationLimit, $"scaling shift must remain usable (got {st5} / {ws.NativeStatus})");
        Require(Math.Abs(ws.X[0] - 1.0) < 1e-4 && Math.Abs(ws.X[1] - 1.0) < 1e-4,
                $"unexpected shifted solution ({ws.X[0]:G6}, {ws.X[1]:G6})");
    }

    private static void CheckFreshRebuildDoesNotDoubleRebuildOnMaxIterations()
    {
        ClarabelWorkspace.IterativeRefinementDefault = false;
        ClarabelWorkspace.Persistent = true;
        ClarabelWorkspace.ResetTimers();

        using var ws = new ClarabelWorkspace();
        var (A, P, b, c) = BuildTinyQp(1.0, 1.0);

        ScsStatus st = ws.Solve(A, b, c, P, zeroDim: 0, posDim: 2, socDims: [], warmStart: false,
                                maxIterations: 1, epsAbs: 1e-9, epsRel: 1e-9);
        Require(ws.LastRebuilt && ws.HitIterationLimit && st == ScsStatus.SolvedInaccurate, "cap=1 fresh solve must stop at cap");
        Require(ClarabelWorkspace.TotalRebuilds == 1, $"fresh solve hitting MaxIterations with IR=false must rebuild once (got {ClarabelWorkspace.TotalRebuilds})");
        Require(ClarabelWorkspace.TotalRefinementFallbacks == 0, "MaxIterations must not flip iterative refinement");
    }

    private static void CheckEffectiveIterationCapStaysBoundToLastSolve()
    {
        var cfg6 = new Scvx6DofConfig { Nodes = 5 };
        var cfg3 = new Scvx3DofConfig { Nodes = 5, GlideIntervals = 2 };

        ScvxConic.Backend = ScvxConicBackend.Clarabel;
        using var sub6Clarabel = new Scvx6DofSubproblemScs(cfg6);
        using var sub3Clarabel = new Scvx3DofSubproblemScs(cfg3);

        Require(sub6Clarabel.EffectiveIterationCap(10_000) == ClarabelWorkspace.IpmIterationCeiling,
                "6-DOF Clarabel cap must clamp to IPM ceiling");
        Require(sub3Clarabel.EffectiveIterationCap(10_000) == ClarabelWorkspace.IpmIterationCeiling,
                "3-DOF Clarabel cap must clamp to IPM ceiling");

        // Flip global toggle mid-iteration before checking escalation cap: both subproblems must still report
        // the backend that belongs to _ws (_wss.Last), not the newly toggled ScvxConic.Backend.
        ScvxConic.Backend = ScvxConicBackend.Scs;
        Require(sub6Clarabel.EffectiveIterationCap(10_000) == ClarabelWorkspace.IpmIterationCeiling,
                "6-DOF cap must remain bound to last workspace after backend toggle");
        Require(sub3Clarabel.EffectiveIterationCap(10_000) == ClarabelWorkspace.IpmIterationCeiling,
                "3-DOF cap must remain bound to last workspace after backend toggle");

        // Reverse direction: created on SCS, toggled to Clarabel.
        using var sub6Scs = new Scvx6DofSubproblemScs(cfg6);
        using var sub3Scs = new Scvx3DofSubproblemScs(cfg3);
        ScvxConic.Backend = ScvxConicBackend.Clarabel;
        Require(sub6Scs.EffectiveIterationCap(10_000) == 10_000,
                "6-DOF SCS cap must remain bound to last workspace after backend toggle");
        Require(sub3Scs.EffectiveIterationCap(10_000) == 10_000,
                "3-DOF SCS cap must remain bound to last workspace after backend toggle");
    }

    private static void CheckDeterministicDispose()
    {
        ClarabelWorkspace.Persistent = true;
        ScvxConic.Backend = ScvxConicBackend.Clarabel;

        var wss = new ScvxConicWorkspaces();
        var (A, P, b, c) = BuildTinyQp(1.0, 1.0);
        ScsStatus st = wss.ForSolve().Solve(A, b, c, P, zeroDim: 0, posDim: 2, socDims: [], warmStart: false,
                                            maxIterations: 100, epsAbs: 1e-8, epsRel: 1e-8);
        Require(st == ScsStatus.Solved, "workspace solve before dispose must succeed");

        // Idempotent disposal of workspace wrapper and subproblem wrappers.
        wss.Dispose();
        wss.Dispose();

        var sub6 = new Scvx6DofSubproblemScs(new Scvx6DofConfig { Nodes = 5 });
        sub6.Dispose();
        sub6.Dispose();

        var sub3 = new Scvx3DofSubproblemScs(new Scvx3DofConfig { Nodes = 5, GlideIntervals = 2 });
        sub3.Dispose();
        sub3.Dispose();
    }

    private static void CheckToleranceCapAndPresolveDisabled()
    {
        ClarabelWorkspace.IterativeRefinementDefault = false;
        ClarabelWorkspace.Persistent = true;
        ClarabelWorkspace.ResetTimers();

        Require(ClarabelWorkspace.EffectiveTolerance(1e-3, 1e-3) == ClarabelWorkspace.MaxPrimaryTolerance,
                "loose 1e-3 request must clamp to MaxPrimaryTolerance");
        Require(ClarabelWorkspace.EffectiveTolerance(1e-4, 1e-4) == ClarabelWorkspace.MaxPrimaryTolerance,
                "loose 1e-4 request must clamp to MaxPrimaryTolerance");
        Require(ClarabelWorkspace.EffectiveTolerance(1e-8, 1e-8) == 1e-8,
                "tighter request must not be loosened");

        using var ws = new ClarabelWorkspace();
        var (A, P, b, c) = BuildTinyQp(1.0, 1.0);

        ScsStatus st1 = ws.Solve(A, b, c, P, zeroDim: 0, posDim: 2, socDims: [], warmStart: false,
                                 maxIterations: 100, epsAbs: 1e-3, epsRel: 1e-3);
        Require(st1 == ScsStatus.Solved && ws.LastRebuilt, "initial loose solve must build and solve");
        Require(Math.Abs(ws.X[0] - 1.0) < 1e-5 && Math.Abs(ws.X[1] - 1.0) < 1e-5,
                $"capped tolerance must solve accurately (got {ws.X[0]:G8}, {ws.X[1]:G8})");

        // Changing eps from 1e-3 to 1e-4 maps to the same capped tolerance, so the persistent solver
        // must update in place (which also verifies PresolveEnable=false and InputSparseDropzeros=false).
        A.BeginRefill();
        A.Add(0, 0, 2.0);
        A.Add(1, 1, 2.0);
        A.EndRefill();
        double[] b2 = [4.0, 4.0];
        ScsStatus st2 = ws.Solve(A, b2, c, P, zeroDim: 0, posDim: 2, socDims: [], warmStart: false,
                                 maxIterations: 100, epsAbs: 1e-4, epsRel: 1e-4);
        Require(st2 == ScsStatus.Solved && !ws.LastRebuilt,
                "switching between capped tolerances must reuse the persistent solver in place");
        Require(Math.Abs(ws.X[0] - 1.0) < 1e-5 && Math.Abs(ws.X[1] - 1.0) < 1e-5,
                $"in-place update must remain accurate (got {ws.X[0]:G8}, {ws.X[1]:G8})");

        // Tightening below MaxPrimaryTolerance must trigger a rebuild with the tighter setting.
        ScsStatus st3 = ws.Solve(A, b2, c, P, zeroDim: 0, posDim: 2, socDims: [], warmStart: false,
                                 maxIterations: 100, epsAbs: 1e-8, epsRel: 1e-8);
        Require(st3 == ScsStatus.Solved && ws.LastRebuilt,
                "tightening tolerance below MaxPrimaryTolerance must rebuild the native solver");
    }

    private static void CheckSolveMsSamplesRingBuffer()
    {
        const int solves = 2500;
        var (A, P, b, c) = BuildTinyQp(1.0, 1.0);

        ScsWorkspace.ResetTimers();
        var scs = new ScsWorkspace();
        for (int i = 0; i < solves; i++)
            scs.Solve(A, b, c, P, zeroDim: 0, posDim: 2, socDims: [], warmStart: false, maxIterations: 200);
        Require(ScsWorkspace.TotalCalls == solves, $"SCS TotalCalls expected {solves}, got {ScsWorkspace.TotalCalls}");
        Require(ScsWorkspace.SolveMsSamples.Count == 1024, $"SCS SolveMsSamples expected 1024, got {ScsWorkspace.SolveMsSamples.Count}");
        Require(ScsWorkspace.SolveMsPercentile(0.5) >= 0.0, "SCS median percentile must be non-negative");
        Require(ScsWorkspace.SolveMsPercentile(0.95) >= ScsWorkspace.SolveMsPercentile(0.5),
                "SCS p95 must be at least p50");

        ClarabelWorkspace.Persistent = true;
        ClarabelWorkspace.ResetTimers();
        using var clarabel = new ClarabelWorkspace();
        for (int i = 0; i < solves; i++)
            clarabel.Solve(A, b, c, P, zeroDim: 0, posDim: 2, socDims: [], warmStart: false, maxIterations: 100);
        Require(ClarabelWorkspace.TotalCalls == solves, $"Clarabel TotalCalls expected {solves}, got {ClarabelWorkspace.TotalCalls}");
        Require(ClarabelWorkspace.SolveMsSamples.Count == 1024, $"Clarabel SolveMsSamples expected 1024, got {ClarabelWorkspace.SolveMsSamples.Count}");
        Require(ClarabelWorkspace.SolveMsPercentile(0.5) >= 0.0, "Clarabel median percentile must be non-negative");
        Require(ClarabelWorkspace.SolveMsPercentile(0.95) >= ClarabelWorkspace.SolveMsPercentile(0.5),
                "Clarabel p95 must be at least p50");
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
            throw new InvalidOperationException(message);
    }
}
