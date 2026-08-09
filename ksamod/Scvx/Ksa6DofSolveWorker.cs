using System;
using System.Threading;

/// <summary>
/// Runs the guidance solve on a background thread so the sim thread never waits for it.
///
/// WHY, and what it does not fix. The warm cycle is already bounded — CycleBudgetMs
/// stops it between SCvx iterations — but the floor is ONE iteration, and one iteration
/// is indivisible and measured at 43 ms typically and 300 ms at its worst. That floor
/// is the remaining stutter, and no amount of budgeting gets underneath it: the only
/// ways down are fewer nodes, which --coldn showed this vehicle cannot afford, or
/// splitting an iteration, which discards the ADMM iterate mid-solve.
///
/// Moving the solve off the thread removes the floor rather than lowering it. A 300 ms
/// solve stops being a 300 ms hitch and becomes a plan that refreshes at 3 Hz instead
/// of 10 Hz — which the loop already tolerates, since it flew planElapsed of 0.7 s
/// through the refusal runs without complaint. It does NOT make guidance faster, and
/// the plan is one solve older than it used to be.
///
/// THE OWNERSHIP RULE, which is the whole design:
///
///   While a solve is in flight the worker owns the guidance object outright. The sim
///   thread must not solve on it, rebuild it, or replace it. It may read Published,
///   because that is an immutable snapshot swapped in by a single reference
///   assignment, and it may publish Inputs, because those are immutable too and are
///   only folded into the model at the next solve's entry.
///
///   Everything else — node-count rebuilds, cold restarts, disengage — has to wait for
///   idle. Those are rare and already tolerate a cycle of delay; a solve is not.
///
/// One thread, one request slot, no queue. If a solve is still running when the next
/// cadence tick arrives, the tick is simply skipped: there is nothing to be gained by
/// queueing work whose input state will be stale by the time it runs.
/// </summary>
public sealed class Ksa6DofSolveWorker : IDisposable
{
    private readonly Thread _thread;
    private readonly SemaphoreSlim _work = new(0, 1);
    private readonly object _gate = new();

    private volatile bool _running = true;
    private volatile bool _busy;

    // Request, written by the sim thread before signalling and read by the worker
    // after. The semaphore is the barrier: nothing here is touched by both at once.
    private Ksa6DofGuidance _guidance;
    private double[] _x0;
    private double _now;
    private int _maxIterations;

    // Result, written by the worker before clearing _busy and read by the sim thread
    // after it observes _busy false.
    private bool _ok;
    private string _error = "";
    private double _solveMs;

    public Ksa6DofSolveWorker()
    {
        _thread = new Thread(Loop)
        {
            IsBackground = true,        // never keeps the game alive on shutdown
            Name = "navbox-6dof-solve",
            // Below normal: guidance is allowed to be late, the frame is not. If the
            // machine is saturated the right thing to lose is plan freshness.
            Priority = ThreadPriority.BelowNormal,
        };
        _thread.Start();
    }

    /// <summary>True while a solve is in flight. The guidance is off limits until it clears.</summary>
    public bool IsBusy => _busy;

    /// <summary>Solves dispatched and completed, for the readout.</summary>
    public int Dispatched { get; private set; }
    public int Completed { get; private set; }

    /// <summary>Cadence ticks skipped because a solve was still running.</summary>
    public int Skipped { get; private set; }

    /// <summary>Wall-clock of the last completed solve, ms.</summary>
    public double LastSolveMs => _solveMs;

    /// <summary>
    /// Hand a solve to the worker. Returns false — and does nothing — if one is
    /// already running, which the caller should treat as "not this cycle" rather than
    /// as a failure.
    ///
    /// x0 is COPIED, because the caller's array is rebuilt from the vehicle every frame.
    /// </summary>
    public bool TryDispatch(Ksa6DofGuidance guidance, double[] x0, double now, int maxIterations)
    {
        if (_busy || !_running)
        {
            Skipped++;
            return false;
        }

        lock (_gate)
        {
            _guidance = guidance;
            _x0 = (double[])x0.Clone();
            _now = now;
            _maxIterations = maxIterations;
            _ok = false;
            _error = "";
        }

        Dispatched++;
        _busy = true;
        _work.Release();
        return true;
    }

    /// <summary>
    /// Collect a finished solve. Returns false if one is still running or none has
    /// completed since the last collection.
    /// </summary>
    public bool TryCollect(out bool solved, out string error)
    {
        solved = false;
        error = "";
        if (_busy || Completed == _collected)
            return false;

        lock (_gate)
        {
            solved = _ok;
            error = _error;
        }
        _collected = Completed;
        return true;
    }

    private int _collected;

    private void Loop()
    {
        while (_running)
        {
            try
            {
                _work.Wait();
            }
            catch (ObjectDisposedException)
            {
                return;
            }
            if (!_running)
                return;

            Ksa6DofGuidance g;
            double[] x0;
            double now;
            int iters;
            lock (_gate)
            {
                g = _guidance;
                x0 = _x0;
                now = _now;
                iters = _maxIterations;
            }

            bool ok = false;
            string error = "";
            var sw = System.Diagnostics.Stopwatch.StartNew();
            try
            {
                ok = g != null && g.Update(x0, now, iters);
                if (!ok && g != null)
                    error = g.Error;
            }
            catch (Exception e)
            {
                // A solve throwing must not take the game down with it. The sim thread
                // sees a failed solve, keeps flying the plan it has, and the circuit
                // breaker escalates from there exactly as it would for any refusal.
                ok = false;
                error = "solve threw: " + e.Message;
            }
            _solveMs = sw.Elapsed.TotalMilliseconds;

            lock (_gate)
            {
                _ok = ok;
                _error = error;
            }
            Completed++;
            _busy = false;      // LAST: the sim thread may touch the guidance again
        }
    }

    public void Dispose()
    {
        _running = false;
        try { _work.Release(); } catch (ObjectDisposedException) { }
        // Give an in-flight solve a moment to finish so it is not abandoned holding
        // the guidance. It is a background thread, so a timeout here is survivable.
        _thread.Join(TimeSpan.FromSeconds(2));
        _work.Dispose();
    }
}
