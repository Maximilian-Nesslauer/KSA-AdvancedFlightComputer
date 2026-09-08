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
        Require(result.Error == "solve threw: native solver unavailable", "exception cause retained");
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
public sealed class Ksa6DofGuidance
{
    public Func<double[], double, int, bool> Solve = (_, _, _) => false;
    public string Error = "";
    public bool Update(double[] x, double now, int iterations) => Solve(x, now, iterations);
    public bool StepCold(double[] x, double now) => Solve(x, now, 1);
}
