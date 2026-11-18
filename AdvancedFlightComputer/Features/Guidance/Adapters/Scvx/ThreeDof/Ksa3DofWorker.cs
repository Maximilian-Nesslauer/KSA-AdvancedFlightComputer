#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Threading.Tasks;

/// <summary>
/// One 3-DOF solve at a time, off the sim thread. The guidance belongs to the job while it runs; the sim thread reads only the immutable plan it publishes, and calls nothing on the guidance until the job is done.
/// </summary>
public sealed class Ksa3DofWorker : IDisposable
{
    private Task _task;
    private CancellationTokenSource _cancel;
    private sealed record Failure(object Guidance, string Text);
    private Failure _failure;

    public bool Busy => _task is { IsCompleted: false };

    public bool TryRun(object guidance, Action<CancellationToken> job)
    {
        if (Busy)
            return false;
        _cancel?.Dispose();
        _cancel = new CancellationTokenSource();
        _failure = null;
        CancellationToken stop = _cancel.Token;
        _task = Task.Run(() =>
        {
            try
            {
                if (!stop.IsCancellationRequested)
                    job(stop);
            }
            catch (Exception e)
            {
                if (!stop.IsCancellationRequested)
                    Interlocked.Exchange(ref _failure, new Failure(guidance, e.ToString()));
            }
        });
        return true;
    }

    internal string TakeError(object guidance)
    {
        Failure failure = Interlocked.Exchange(ref _failure, null);
        return failure != null && ReferenceEquals(failure.Guidance, guidance) ? failure.Text : "";
    }

    public void Dispose()
    {
        _cancel?.Cancel();
        _cancel?.Dispose();
        _cancel = null;
        Interlocked.Exchange(ref _failure, null);
    }
}
