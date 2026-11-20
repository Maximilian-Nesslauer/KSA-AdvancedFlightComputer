#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Diagnostics;
using System.Threading;

/// <summary>
/// One 3-DOF solve at a time, off the sim thread. The guidance belongs to the job while it runs; the sim thread reads only the immutable plan it publishes, and calls nothing on the guidance until the job is done.
///
/// ITS OWN THREAD, as touchdown guidance's solver has (Ksa6DofSolveWorker), not the shared thread pool. On the pool a solve waited its turn behind the game's own work: on 2026-11-18 re-solves of 40 ms sat queued for 0.7 to 3.6 s, the last of them 2.35 s at 1 km, and the ignition locked on a plan that stale. <see cref="LastWaitMs"/> and <see cref="LastRunMs"/> keep the queue and the solve apart, for the entry log.
/// </summary>
public sealed class Ksa3DofWorker : IDisposable
{
    private sealed record Job(object Guidance, Action<CancellationToken> Run, CancellationToken Stop, long Queued);
    private sealed record Failure(object Guidance, string Text);

    private readonly Thread _thread;
    private readonly SemaphoreSlim _work = new(0, 1);
    private readonly object _gate = new();
    private bool _running = true;
    private bool _busy;
    private Job _pending;
    private CancellationTokenSource _cancel;
    private Failure _failure;
    private double _waitMs = double.NaN, _runMs = double.NaN;

    public Ksa3DofWorker()
    {
        _thread = new Thread(Loop)
        {
            IsBackground = true,
            Name = "afc-entry-solve",
        };
        _thread.Start();
    }

    /// <summary>True from a job's dispatch until it has finished.</summary>
    public bool Busy { get { lock (_gate) return _busy; } }

    /// <summary>The last finished job's time from dispatch to starting, and its own run time, ms. NaN before the first.</summary>
    public double LastWaitMs { get { lock (_gate) return _waitMs; } }
    public double LastRunMs { get { lock (_gate) return _runMs; } }

    public bool TryRun(object guidance, Action<CancellationToken> job)
    {
        lock (_gate)
        {
            if (_busy || !_running)
                return false;
            _cancel?.Dispose();
            _cancel = new CancellationTokenSource();
            _failure = null;
            _pending = new Job(guidance, job, _cancel.Token, Stopwatch.GetTimestamp());
            _busy = true;
        }
        _work.Release();
        return true;
    }

    private void Loop()
    {
        while (true)
        {
            _work.Wait();
            Job job;
            lock (_gate)
            {
                if (!_running)
                    return;
                job = _pending;
                _pending = null;
            }
            if (job == null)
                continue;
            long start = Stopwatch.GetTimestamp();
            try
            {
                if (!job.Stop.IsCancellationRequested)
                    job.Run(job.Stop);
            }
            catch (Exception e)
            {
                if (!job.Stop.IsCancellationRequested)
                    Interlocked.Exchange(ref _failure, new Failure(job.Guidance, e.ToString()));
            }
            long end = Stopwatch.GetTimestamp();
            lock (_gate)
            {
                _waitMs = (start - job.Queued) * 1000.0 / Stopwatch.Frequency;
                _runMs = (end - start) * 1000.0 / Stopwatch.Frequency;
                _busy = false;
            }
        }
    }

    internal string TakeError(object guidance)
    {
        Failure failure = Interlocked.Exchange(ref _failure, null);
        return failure != null && ReferenceEquals(failure.Guidance, guidance) ? failure.Text : "";
    }

    /// <summary>Cancel any job and end the thread once it is done; a running solve finishes its iteration and stops.</summary>
    public void Dispose()
    {
        lock (_gate)
        {
            if (!_running)
                return;
            _running = false;
            _cancel?.Cancel();
            _cancel?.Dispose();
            _cancel = null;
        }
        Interlocked.Exchange(ref _failure, null);
        try
        {
            _work.Release();
        }
        catch (SemaphoreFullException)
        {
            // A job was dispatched and not yet taken: the thread wakes on it and sees the stop.
        }
    }
}
