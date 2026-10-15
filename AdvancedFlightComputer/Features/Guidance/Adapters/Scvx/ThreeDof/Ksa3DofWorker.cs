#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Threading.Tasks;

/// <summary>
/// One 3-DOF solve at a time, off the sim thread. The guidance belongs to the job while it runs; the sim thread reads only the immutable plan it publishes, and calls nothing on the guidance until the job is done.
/// </summary>
public sealed class Ksa3DofWorker
{
    private Task _task;

    public bool Busy => _task is { IsCompleted: false };

    /// <summary>The last job's exception, for the log; cleared by the reader.</summary>
    public string LastError = "";

    public bool TryRun(Action job)
    {
        if (Busy)
            return false;
        _task = Task.Run(() =>
        {
            try { job(); }
            catch (Exception e) { LastError = e.ToString(); }
        });
        return true;
    }
}
