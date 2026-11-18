using System.Diagnostics;
using AdvancedFlightComputer.Guidance.Gfold;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>One powered-descent solve from a snapshot containing only numeric flight data.</summary>
internal sealed class GfoldSolveJob : IDisposable
{
    private readonly CancellationTokenSource _stop = new();
    private readonly Task<Result> _task;
    private Result? _preview;

    internal sealed record Request(GfoldParams Parameters, int Nodes, double SimTime, double RetainedArrival,
                                   double MinTime, double MaxTime, bool RestSearch, GfoldOptions Options);

    internal enum Outcome { Ready, Resting, Unreachable, SearchTimedOut, FuelShort, Fault }

    internal sealed record Result(Outcome Status, double SolveMs, double Arrival,
                                  GfoldTrajectory? Trajectory = null, string? Error = null);

    internal double SimTime { get; }
    internal GfoldParams Parameters { get; }

    public GfoldSolveJob(Request request)
    {
        SimTime = request.SimTime;
        Parameters = request.Parameters;
        CancellationToken stop = _stop.Token;
        _task = Task.Run(() => Solve(request, stop, preview => Interlocked.Exchange(ref _preview, preview)));
    }

    public bool TryTakeResult(out Result? result, out bool final)
    {
        final = _task.IsCompletedSuccessfully;
        result = final ? _task.Result : Interlocked.Exchange(ref _preview, null);
        return result != null;
    }

    public void Dispose()
    {
        _stop.Cancel();
        _stop.Dispose();
    }

    private static Result Solve(Request request, CancellationToken stop, Action<Result> offerPreview)
    {
        var clock = Stopwatch.StartNew();
        GfoldParams p = request.Parameters;
        double remaining = request.RetainedArrival - request.SimTime;
        double perSolve = request.Options.SolveTimeLimitS > 0.0 ? Math.Min(request.Options.SolveTimeLimitS, 0.12) : 0.12;
        GfoldOptions options = request.Options with { SolveTimeLimitS = perSolve };
        double limit = Math.Clamp(perSolve * 8.0, 0.4, 0.6);
        try
        {
            if (!stop.IsCancellationRequested && remaining > request.MinTime)
            {
                GfoldTrajectory t = GfoldPlanner.SolveMinFuel(p, remaining, request.Nodes, p.Rf, options: options);
                if (t.IsUsable && t.FuelUsed <= p.FuelMass)
                    return new(Outcome.Ready, clock.Elapsed.TotalMilliseconds, request.RetainedArrival, t);
            }
            if (request.RestSearch)
                return new(Outcome.Resting, clock.Elapsed.TotalMilliseconds, request.RetainedArrival);

            double searchBudget = limit - clock.Elapsed.TotalSeconds;
            GfoldPlanner.SearchResult? best = !stop.IsCancellationRequested && searchBudget > 0
                ? GfoldPlanner.SearchMinFuel(p, request.Nodes, tfLo: request.MinTime, tfHi: request.MaxTime,
                    options: options, tfGuess: remaining > request.MinTime ? remaining : EstimateTime(p),
                    searchTimeLimitS: searchBudget, stop: stop, onCandidate: candidate =>
                    {
                        if (candidate.FuelUsed <= p.FuelMass)
                            offerPreview(new Result(Outcome.Ready, clock.Elapsed.TotalMilliseconds,
                                request.SimTime + candidate.TimeOfFlight, candidate.Trajectory));
                    })
                : null;
            if (best == null)
                return new(clock.Elapsed.TotalSeconds >= limit ? Outcome.SearchTimedOut : Outcome.Unreachable,
                    clock.Elapsed.TotalMilliseconds, double.NaN);
            if (!best.Complete && best.FuelUsed > p.FuelMass)
                return new(Outcome.SearchTimedOut, clock.Elapsed.TotalMilliseconds, double.NaN);

            Outcome status = best.FuelUsed <= p.FuelMass ? Outcome.Ready : Outcome.FuelShort;
            return new(status, clock.Elapsed.TotalMilliseconds, request.SimTime + best.TimeOfFlight, best.Trajectory);
        }
        catch (Exception error)
        {
            return new(Outcome.Fault, clock.Elapsed.TotalMilliseconds, double.NaN, Error: error.Message);
        }
    }

    private static double EstimateTime(GfoldParams p)
    {
        double accel = Math.Max(0.05, Math.Min(p.R2 / p.WetMass, p.AccelMax) - p.GravityMag);
        double height = Math.Max(0.0, p.R0[0] - p.Rf[0]);
        double vertical = Math.Abs(p.V0[0] - p.Vf[0]) / accel + Math.Sqrt(2.0 * height / accel);
        double across = Math.Sqrt(Math.Pow(p.R0[1] - p.Rf[1], 2.0) + Math.Pow(p.R0[2] - p.Rf[2], 2.0));
        return Math.Max(vertical, 2.0 * Math.Sqrt(across / accel));
    }
}
