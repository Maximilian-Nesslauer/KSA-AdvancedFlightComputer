using AdvancedFlightComputer.Features.Guidance;
internal static class Program
{
    private static int Main()
    {
        Action[] checks =
        [
            InlineTransitionWaitsForCollection,
            ColdFailureKeepsItsCause,
            ColdRefusalIsNotAnException,
            RebuildKeepsSourceIdentity,
            InitColdProducesGuidanceOffCallerThread,
            InitColdFaultAndNullHandling,
            DisposedWorkerDisposesOrphanProducedGuidance,
            DisposedWorkerCannotReturnLateResults,
            ConcurrentDispatchHasOneWinner,
            DisposeBeforePickupIsSafe,
        ];
        try
        {
            foreach (Action check in checks)
            {
                check();
                Console.WriteLine("PASS " + check.Method.Name);
            }
            return 0;
        }
        catch (Exception error)
        {
            Console.Error.WriteLine(error);
            return 1;
        }
    }

    private static void InlineTransitionWaitsForCollection()
    {
        using var entered = new ManualResetEventSlim();
        using var release = new ManualResetEventSlim();
        using var worker = new Ksa6DofSolveWorker();
        double observed = 0;
        var guidance = new Ksa6DofGuidance
        {
            Solve = (x, _, _) =>
            {
                entered.Set();
                Wait(release, "blocked update release");
                observed = x[0];
                return true;
            },
        };
        double[] input = [42];
        Require(worker.TryDispatchUpdate(guidance, input, 5, 2), "dispatch update");
        try
        {
            Wait(entered, "update entry");
            input[0] = -1;
            Require(worker.IsBusy && !worker.IsIdle, "running work owns solver");
            Require(!worker.TryStopWhenIdle(), "inline switch waits while running");
            Require(!worker.TryCollect(out _), "running work has no result");
        }
        finally { release.Set(); }

        Finished(worker);
        Require(observed == 42, "request retains its input snapshot");
        Require(!worker.IsIdle, "completed work owns slot until collected");
        Require(!worker.TryStopWhenIdle(), "inline switch preserves unread result");
        Require(!worker.TryDispatchStepCold(guidance, [0], 6), "cold restart cannot erase warm result");
        Require(worker.TryCollect(out var result) && result.Ok, "collect update result");
        Require(result.Matches(Ksa6DofJob.Update, guidance), "source and job match");
        Require(!result.Matches(Ksa6DofJob.StepCold, guidance), "cold phase rejects warm result");
        Require(!result.Matches(Ksa6DofJob.Update, new Ksa6DofGuidance()), "new owner rejects old result");
        Require(!worker.TryCollect(out _), "result is collected once");
        Require(worker.IsIdle && worker.TryStopWhenIdle(), "inline switch after collection");
        Require(guidance.Update([7], 6, 1) && observed == 7, "retained solver can now run inline");
        Require(!worker.TryDispatchUpdate(guidance, [0], 0, 1), "stopped worker rejects dispatch");
    }

    private static void ColdFailureKeepsItsCause()
    {
        using var worker = new Ksa6DofSolveWorker();
        var guidance = new Ksa6DofGuidance
        {
            Solve = (_, _, _) => throw new InvalidOperationException("native solver unavailable"),
        };
        Require(worker.TryDispatchStepCold(guidance, [1], 0), "dispatch cold failure");
        Finished(worker);
        Require(worker.TryCollect(out var result), "collect cold failure");
        Require(result.Matches(Ksa6DofJob.StepCold, guidance), "cold error belongs to its source");
        Require(!result.Ok && result.Faulted, "exception is distinct from more iterations needed");
        Require(result.Error.StartsWith("solve threw: native solver unavailable (InvalidOperationException at ", StringComparison.Ordinal), "exception cause retained");
    }

    private static void ColdRefusalIsNotAnException()
    {
        using var worker = new Ksa6DofSolveWorker();
        var guidance = new Ksa6DofGuidance { Error = "position defect too large" };
        Require(worker.TryDispatchStepCold(guidance, [1], 0), "dispatch cold refusal");
        Finished(worker);
        Require(worker.TryCollect(out var result), "collect cold refusal");
        Require(!result.Ok && !result.Faulted && result.Error == guidance.Error, "refusal reason retained");
    }

    private static void RebuildKeepsSourceIdentity()
    {
        using var worker = new Ksa6DofSolveWorker();
        var source = new Ksa6DofGuidance();
        var next = new Ksa6DofGuidance();
        Require(worker.TryDispatchRebuild((from, x, now) =>
        {
            Require(ReferenceEquals(from, source) && x[0] == 3 && now == 4, "rebuild request");
            return next;
        }, source, [3], 4), "dispatch rebuild");
        Finished(worker);
        Require(worker.TryCollect(out var result), "collect rebuild");
        Require(result.Matches(Ksa6DofJob.Rebuild, source) && result.Ok, "rebuild source retained");
        Require(ReferenceEquals(result.Produced, next), "rebuilt guidance returned");
    }

    private static void InitColdProducesGuidanceOffCallerThread()
    {
        using var worker = new Ksa6DofSolveWorker();
        int callerThread = Environment.CurrentManagedThreadId;
        int workerThread = 0;
        var created = new Ksa6DofGuidance();
        double[] input = [11.5];
        Require(worker.TryDispatchInitCold((x, now) =>
        {
            workerThread = Environment.CurrentManagedThreadId;
            Require(x[0] == 11.5 && now == 9.0, "init cold request snapshot");
            return created;
        }, input, 9.0), "dispatch init cold");
        input[0] = -99.0;
        Finished(worker);
        Require(workerThread != 0 && workerThread != callerThread, "init cold runs on worker thread");
        Require(worker.TryCollect(out var result), "collect init cold");
        Require(result.Job == Ksa6DofJob.InitCold && result.Source == null, "init cold job and source");
        Require(result.Ok && !result.Faulted && ReferenceEquals(result.Produced, created), "init cold produced guidance");
    }

    private static void InitColdFaultAndNullHandling()
    {
        using var worker = new Ksa6DofSolveWorker();
        Require(worker.TryDispatchInitCold((_, _) => null, [1], 2), "dispatch null init cold");
        Finished(worker);
        Require(worker.TryCollect(out var nullResult), "collect null init cold");
        Require(nullResult.Job == Ksa6DofJob.InitCold && !nullResult.Ok && !nullResult.Faulted
            && nullResult.Error == "cold init failed", "null init cold error");

        Require(worker.TryDispatchInitCold((_, _) => throw new InvalidOperationException("bad seed"), [1], 3),
            "dispatch faulting init cold");
        Finished(worker);
        Require(worker.TryCollect(out var faultResult), "collect faulting init cold");
        Require(faultResult.Job == Ksa6DofJob.InitCold && !faultResult.Ok && faultResult.Faulted
            && faultResult.Error.StartsWith("solve threw: bad seed (InvalidOperationException at ", StringComparison.Ordinal),
            "faulting init cold retains cause");
    }

    private static void DisposedWorkerDisposesOrphanProducedGuidance()
    {
        using var entered = new ManualResetEventSlim();
        using var release = new ManualResetEventSlim();
        using var worker = new Ksa6DofSolveWorker();
        var late = new Ksa6DofGuidance();
        Require(worker.TryDispatchInitCold((_, _) =>
        {
            entered.Set();
            Wait(release, "orphan init cold release");
            return late;
        }, [0], 0), "dispatch orphan init cold");
        try
        {
            Wait(entered, "orphan init cold entry");
            worker.Dispose();
        }
        finally { release.Set(); }
        Finished(worker);
        Require(!worker.TryCollect(out _), "disposed init cold result discarded");
        Require(late.Disposed, "late init cold guidance disposed");

        using var worker2 = new Ksa6DofSolveWorker();
        var uncollected = new Ksa6DofGuidance();
        Require(worker2.TryDispatchInitCold((_, _) => uncollected, [0], 0), "dispatch uncollected init cold");
        Finished(worker2);
        worker2.Dispose();
        Require(uncollected.Disposed, "uncollected init cold guidance disposed on worker dispose");
    }

    private static void DisposedWorkerCannotReturnLateResults()
    {
        using var entered = new ManualResetEventSlim();
        using var release = new ManualResetEventSlim();
        using var oldWorker = new Ksa6DofSolveWorker();
        var oldGuidance = new Ksa6DofGuidance();
        Require(oldWorker.TryDispatchRebuild((_, _, _) =>
        {
            entered.Set();
            Wait(release, "orphan rebuild release");
            return new Ksa6DofGuidance();
        }, oldGuidance, [0], 0), "dispatch orphan rebuild");
        try
        {
            Wait(entered, "orphan rebuild entry");
            oldWorker.Dispose();
            oldWorker.Dispose();
            Require(oldWorker.IsBusy, "dispose returns before native work ends");
            Require(!oldWorker.TryCollect(out _), "disposed worker has no collectable result");

            using var newWorker = new Ksa6DofSolveWorker();
            var newGuidance = new Ksa6DofGuidance { Solve = (_, _, _) => true };
            Require(newWorker.TryDispatchStepCold(newGuidance, [1], 1), "new owner dispatch");
            Finished(newWorker);
            Require(newWorker.TryCollect(out var current) && current.Ok, "new owner completes independently");
            Require(current.Matches(Ksa6DofJob.StepCold, newGuidance), "new owner result identity");
        }
        finally { release.Set(); }
        Finished(oldWorker);
        Require(!oldWorker.TryCollect(out _), "late orphan result discarded");
        oldWorker.Dispose();
    }

    private static void ConcurrentDispatchHasOneWinner()
    {
        using var release = new ManualResetEventSlim();
        using var worker = new Ksa6DofSolveWorker();
        var guidance = new Ksa6DofGuidance
        {
            Solve = (_, _, _) => { Wait(release, "parallel dispatch release"); return true; },
        };
        int winners = 0;
        try
        {
            Parallel.For(0, 16, i =>
            {
                if (worker.TryDispatchUpdate(guidance, [i], i, 1))
                    Interlocked.Increment(ref winners);
            });
            Require(winners == 1 && worker.Dispatched == 1 && worker.Skipped == 15,
                "dispatch reserves the slot atomically");
        }
        finally { release.Set(); }
        Finished(worker);
        Require(worker.TryCollect(out var result) && result.Ok, "winning dispatch completes");
    }

    private static void DisposeBeforePickupIsSafe()
    {
        for (int i = 0; i < 100; i++)
        {
            using var worker = new Ksa6DofSolveWorker();
            var guidance = new Ksa6DofGuidance { Solve = (_, _, _) => true };
            Require(worker.TryDispatchUpdate(guidance, [0], 0, 1), "dispatch before immediate disposal");
            worker.Dispose();
            worker.Dispose();
            Finished(worker);
            Require(!worker.TryCollect(out _), "immediately disposed result discarded");
        }
    }

    private static void Wait(ManualResetEventSlim signal, string label)
        => Require(signal.Wait(TimeSpan.FromSeconds(5)), "timeout at " + label);

    private static void Finished(Ksa6DofSolveWorker worker)
        => Require(SpinWait.SpinUntil(() => !worker.IsBusy, TimeSpan.FromSeconds(5)), "worker completion timeout");

    private static void Require(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }
}

// Event-controlled solver work makes ownership races reproducible without a game
// or native solver. The production worker is compiled directly into this runner.
public sealed class Ksa6DofGuidance : IDisposable
{
    public Func<double[], double, int, bool> Solve = (_, _, _) => false;
    public string Error = "";
    public volatile bool Disposed;
    public bool Update(double[] x, double now, int iterations) => Solve(x, now, iterations);
    public bool StepCold(double[] x, double now) => Solve(x, now, 1);
    public bool Track(double[] x, double now, int iterations) => Solve(x, now, iterations);
    public void Dispose() => Disposed = true;
}
