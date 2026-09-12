#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Threading;

public enum Ksa6DofJob
{
    Update,
    StepCold,
    Rebuild,
}

public sealed record Ksa6DofSolveResult(
    Ksa6DofJob Job, Ksa6DofGuidance Source, bool Ok, string Error,
    bool Faulted, Ksa6DofGuidance Produced)
{
    public bool Matches(Ksa6DofJob job, Ksa6DofGuidance source)
        => Job == job && ReferenceEquals(Source, source);
}

/// <summary>
/// Owns one mutable guidance during a solve. Callers can read its immutable plan and publish immutable inputs, but must collect the result before changing the solver.
/// A completed result holds the slot until collection so another request cannot erase it.
/// </summary>
public sealed class Ksa6DofSolveWorker : IDisposable
{
    private readonly Thread _thread;
    private readonly SemaphoreSlim _work = new(0, 1);
    private readonly object _gate = new();
    private bool _running = true;
    private bool _busy;
    private Request _request;
    private Ksa6DofSolveResult _result;
    private int _dispatched, _completed, _skipped;
    private double _solveMs;

    private sealed record Request(
        Ksa6DofJob Job, Ksa6DofGuidance Guidance, double[] X0, double Now,
        int MaxIterations, Func<Ksa6DofGuidance, double[], double, Ksa6DofGuidance> Rebuild);

    public Ksa6DofSolveWorker()
    {
        _thread = new Thread(Loop)
        {
            IsBackground = true,
            Name = "afc-6dof-solve",
            Priority = ThreadPriority.BelowNormal,
        };
        _thread.Start();
    }

    public bool IsBusy { get { lock (_gate) return _busy; } }
    public bool IsIdle { get { lock (_gate) return !_busy && _result == null; } }
    public int Dispatched { get { lock (_gate) return _dispatched; } }
    public int Completed { get { lock (_gate) return _completed; } }
    public int Skipped { get { lock (_gate) return _skipped; } }
    public double LastSolveMs { get { lock (_gate) return _solveMs; } }

    public bool TryDispatchUpdate(Ksa6DofGuidance guidance, double[] x0, double now, int maxIterations)
        => Dispatch(Ksa6DofJob.Update, guidance, x0, now, maxIterations, null);

    public bool TryDispatchStepCold(Ksa6DofGuidance guidance, double[] x0, double now)
        => Dispatch(Ksa6DofJob.StepCold, guidance, x0, now, 1, null);

    public bool TryDispatchRebuild(Func<Ksa6DofGuidance, double[], double, Ksa6DofGuidance> rebuild,
                                   Ksa6DofGuidance from, double[] x0, double now)
        => Dispatch(Ksa6DofJob.Rebuild, from, x0, now, 1, rebuild);

    private bool Dispatch(Ksa6DofJob job, Ksa6DofGuidance guidance, double[] x0, double now,
                          int maxIterations,
                          Func<Ksa6DofGuidance, double[], double, Ksa6DofGuidance> rebuild)
    {
        lock (_gate)
        {
            if (_busy || !_running || _result != null)
            {
                _skipped++;
                return false;
            }

            _request = new Request(job, guidance, (double[])x0.Clone(), now, maxIterations, rebuild);
            _dispatched++;
            _busy = true;
            _work.Release();
            return true;
        }
    }

    public bool TryCollect(out Ksa6DofSolveResult result)
    {
        lock (_gate)
        {
            result = null;
            if (!_running || _busy || _result == null)
                return false;

            result = _result;
            _result = null;
            return true;
        }
    }

    /// <summary>
    /// Releases solver ownership for inline use only after the last result is collected.
    /// A refused transition leaves the worker intact for the next simulation step.
    /// </summary>
    public bool TryStopWhenIdle()
    {
        lock (_gate)
        {
            if (_busy || _result != null)
                return false;
            Stop();
            return true;
        }
    }

    private void Loop()
    {
        try
        {
            while (true)
            {
                _work.Wait();
                Request request;
                lock (_gate)
                {
                    if (!_running)
                    {
                        _request = null;
                        _busy = false;
                        return;
                    }
                    request = _request;
                }

                bool ok = false;
                bool faulted = false;
                string error = "";
                Ksa6DofGuidance produced = null;
                var sw = System.Diagnostics.Stopwatch.StartNew();
                try
                {
                    switch (request.Job)
                    {
                        case Ksa6DofJob.Update:
                            ok = request.Guidance.Update(request.X0, request.Now, request.MaxIterations);
                            if (!ok) error = request.Guidance.Error;
                            break;
                        case Ksa6DofJob.StepCold:
                            ok = request.Guidance.StepCold(request.X0, request.Now);
                            if (!ok) error = request.Guidance.Error;
                            break;
                        case Ksa6DofJob.Rebuild:
                            produced = request.Rebuild(request.Guidance, request.X0, request.Now);
                            ok = produced != null;
                            if (!ok) error = "reseed failed";
                            break;
                    }
                }
                catch (Exception e)
                {
                    faulted = true;
                    error = "solve threw: " + e.Message;
                }

                lock (_gate)
                {
                    _solveMs = sw.Elapsed.TotalMilliseconds;
                    _completed++;
                    // Dispose abandons both the worker and its guidance. No result from that owner can be collected after a later engagement.
                    if (_running)
                        _result = new Ksa6DofSolveResult(request.Job, request.Guidance,
                            ok, error, faulted, produced);
                    _request = null;
                    _busy = false;
                    if (!_running) return;
                }
            }
        }
        finally
        {
            _work.Dispose();
        }
    }

    // Called under the gate. Only the worker disposes the semaphore after it exits.
    private void Stop()
    {
        if (!_running) return;
        _running = false;
        _result = null;
        if (_work.CurrentCount == 0)
            _work.Release();
    }

    /// <summary>
    /// Abandons the worker without waiting for native work. The caller must also abandon the guidance, because an active solve can still change that instance.
    /// Use TryStopWhenIdle to retain the guidance for inline execution.
    /// </summary>
    public void Dispose()
    {
        lock (_gate) Stop();
    }
}
