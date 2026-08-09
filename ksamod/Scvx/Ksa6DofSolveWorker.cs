using System;
using System.Threading;

/// <summary>What kind of solve a worker job is. All three block a frame if run inline.</summary>
public enum Ksa6DofJob
{
    /// <summary>The ordinary warm MPC re-solve.</summary>
    Update,
    /// <summary>One iteration of a cold solve, re-anchored at the state passed in.</summary>
    StepCold,
    /// <summary>Construct a guidance at a new node count and reseed it from the current plan.</summary>
    Rebuild,
}

/// <summary>
/// Runs the guidance solve on a background thread so the sim thread never waits for it.
///
/// WHY, and what it does not fix. The warm cycle is already bounded — CycleBudgetMs
/// stops it between SCvx iterations — but the floor is ONE iteration, and one iteration
/// is indivisible: measured at 43 ms typically and 300 ms at its worst. That floor is
/// the stutter, and no budgeting gets underneath it. The ways down were fewer nodes,
/// which --coldn showed this vehicle cannot afford, or splitting an iteration, which
/// discards the ADMM iterate mid-solve. So the floor is removed rather than lowered.
///
/// ALL THREE JOB KINDS, not just the warm one. A first version threaded Update alone
/// and flight log 20260809-113132 still hitched throughout, because the cold solve and
/// the node-step reseed are solves too — that run did three cold solves and six
/// reseeds, all inline. Anything that calls into the solver has to come through here or
/// the frame pays for it.
///
/// THE OWNERSHIP RULE, which is the whole design:
///
///   While a job is in flight the worker owns the guidance outright. The sim thread
///   must not solve on it, rebuild it, or replace it. It may read Published, an
///   immutable snapshot swapped in by one reference assignment, and it may publish
///   Inputs, immutable and folded into the model only at the next solve's entry.
///
///   Everything else waits for idle. Those things are rare and already tolerate a
///   cycle of delay; a solve is not.
///
/// One thread, one slot, no queue. If a job is still running when the next tick
/// arrives it is skipped: queueing work whose input state will be stale by the time it
/// runs buys nothing.
/// </summary>
public sealed class Ksa6DofSolveWorker : IDisposable
{
    private readonly Thread _thread;
    private readonly SemaphoreSlim _work = new(0, 1);
    private readonly object _gate = new();

    private volatile bool _running = true;
    private volatile bool _busy;

    // Request: written by the sim thread before signalling, read by the worker after.
    // The semaphore is the barrier, so nothing here is touched by both at once.
    private Ksa6DofJob _job;
    private Ksa6DofGuidance _guidance;
    private Ksa6DofGuidance _rebuildFrom;
    private Func<Ksa6DofGuidance, double[], double, Ksa6DofGuidance> _rebuild;
    private double[] _x0;
    private double _now;
    private int _maxIterations;

    // Result: written by the worker before it clears _busy, read by the sim thread
    // only after it has observed _busy false.
    private bool _ok;
    private string _error = "";
    private Ksa6DofGuidance _produced;
    private double _solveMs;

    public Ksa6DofSolveWorker()
    {
        _thread = new Thread(Loop)
        {
            IsBackground = true,        // never keeps the game alive on shutdown
            Name = "navbox-6dof-solve",
            // Below normal: guidance is allowed to be late, the frame is not. If the
            // machine is saturated, the right thing to lose is plan freshness.
            Priority = ThreadPriority.BelowNormal,
        };
        _thread.Start();
    }

    /// <summary>True while a job is in flight. The guidance is off limits until it clears.</summary>
    public bool IsBusy => _busy;

    public int Dispatched { get; private set; }
    public int Completed { get; private set; }

    /// <summary>Ticks skipped because a job was still running, for the readout.</summary>
    public int Skipped { get; private set; }

    /// <summary>Wall-clock of the last completed job, ms.</summary>
    public double LastSolveMs => _solveMs;

    /// <summary>The warm MPC re-solve.</summary>
    public bool TryDispatchUpdate(Ksa6DofGuidance guidance, double[] x0, double now, int maxIterations)
        => Dispatch(Ksa6DofJob.Update, guidance, x0, now, maxIterations, null, null);

    /// <summary>
    /// One cold iteration, re-anchored at x0. Dispatched per frame rather than looping
    /// on the worker, so the anchor stays as fresh as it was when the sim thread did
    /// this inline — the vehicle falls fast enough during a cold solve that freezing
    /// x0 for its whole duration would seed the next warm cycle from where the vehicle
    /// used to be.
    /// </summary>
    public bool TryDispatchStepCold(Ksa6DofGuidance guidance, double[] x0, double now)
        => Dispatch(Ksa6DofJob.StepCold, guidance, x0, now, 1, null, null);

    /// <summary>
    /// Construct a guidance at a new node count and reseed it. The caller supplies the
    /// pure half of the rebuild as a delegate, because the other half reads the vehicle
    /// and can only run on the sim thread. Returns the new guidance through TryCollect.
    /// </summary>
    public bool TryDispatchRebuild(Func<Ksa6DofGuidance, double[], double, Ksa6DofGuidance> rebuild,
                                   Ksa6DofGuidance from, double[] x0, double now)
        => Dispatch(Ksa6DofJob.Rebuild, null, x0, now, 1, rebuild, from);

    private bool Dispatch(Ksa6DofJob job, Ksa6DofGuidance guidance, double[] x0, double now,
                          int maxIterations,
                          Func<Ksa6DofGuidance, double[], double, Ksa6DofGuidance> rebuild,
                          Ksa6DofGuidance from)
    {
        if (_busy || !_running)
        {
            Skipped++;
            return false;
        }

        lock (_gate)
        {
            _job = job;
            _guidance = guidance;
            _rebuild = rebuild;
            _rebuildFrom = from;
            // COPIED: the caller's array is rebuilt from the vehicle every frame.
            _x0 = (double[])x0.Clone();
            _now = now;
            _maxIterations = maxIterations;
            _ok = false;
            _error = "";
            _produced = null;
        }

        Dispatched++;
        _busy = true;
        _work.Release();
        return true;
    }

    /// <summary>
    /// Collect a finished job. False if one is still running, or if nothing has
    /// completed since the last collection.
    /// </summary>
    public bool TryCollect(out Ksa6DofJob job, out bool ok, out string error,
                           out Ksa6DofGuidance produced)
    {
        job = default;
        ok = false;
        error = "";
        produced = null;
        if (_busy || Completed == _collected)
            return false;

        lock (_gate)
        {
            job = _job;
            ok = _ok;
            error = _error;
            produced = _produced;
        }
        _collected = Completed;
        return true;
    }

    private int _collected;

    private void Loop()
    {
        while (_running)
        {
            try { _work.Wait(); }
            catch (ObjectDisposedException) { return; }
            if (!_running) return;

            Ksa6DofJob job;
            Ksa6DofGuidance g, from;
            Func<Ksa6DofGuidance, double[], double, Ksa6DofGuidance> rebuild;
            double[] x0;
            double now;
            int iters;
            lock (_gate)
            {
                job = _job; g = _guidance; from = _rebuildFrom; rebuild = _rebuild;
                x0 = _x0; now = _now; iters = _maxIterations;
            }

            bool ok = false;
            string error = "";
            Ksa6DofGuidance produced = null;
            var sw = System.Diagnostics.Stopwatch.StartNew();
            try
            {
                switch (job)
                {
                    case Ksa6DofJob.Update:
                        ok = g != null && g.Update(x0, now, iters);
                        if (!ok && g != null) error = g.Error;
                        break;
                    case Ksa6DofJob.StepCold:
                        ok = g != null && g.StepCold(x0, now);
                        if (!ok && g != null) error = g.Error;
                        break;
                    case Ksa6DofJob.Rebuild:
                        produced = rebuild?.Invoke(from, x0, now);
                        ok = produced != null;
                        if (!ok) error = "reseed failed";
                        break;
                }
            }
            catch (Exception e)
            {
                // A solve throwing must not take the game down. The sim thread sees a
                // failure, keeps flying the plan it has, and the circuit breaker
                // escalates from there exactly as it would for any refusal.
                ok = false;
                produced = null;
                error = "solve threw: " + e.Message;
            }
            _solveMs = sw.Elapsed.TotalMilliseconds;

            lock (_gate)
            {
                _ok = ok;
                _error = error;
                _produced = produced;
            }
            Completed++;
            _busy = false;      // LAST: the sim thread may touch the guidance again
        }
    }

    public void Dispose()
    {
        _running = false;
        try { _work.Release(); } catch (ObjectDisposedException) { }
        // Give an in-flight job a moment rather than abandoning the worker mid-solve on
        // an object about to be collected. Background thread, so the wait is bounded
        // and survivable either way.
        _thread.Join(TimeSpan.FromSeconds(2));
        _work.Dispose();
    }
}
