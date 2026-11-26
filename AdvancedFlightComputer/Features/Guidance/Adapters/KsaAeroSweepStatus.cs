#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;

public static partial class KsaAeroSweep
{
    /// <summary>
    /// What one sweep on its own thread (<see cref="Job"/>) is doing, for TUNE &gt; DIAG &gt; AERO. Its thread writes it, the UI reads it.
    /// A sweep hands its body table over first and sweeps its control surfaces after (two passes, see Sweep): a mode that starts on the body table flies without surfaces until the surface stage is DONE.
    /// </summary>
    public sealed class SweepProgress
    {
        public enum Stage { Mesh, Body, Surfaces, Done, Failed }

        /// <summary>The craft's vehicle id, as the sweep's log lines name it.</summary>
        public string Craft = "";

        /// <summary>The tables swept: TAIL, NOSE, BELLY TRIM, BELLY +Z and the like, or BOOSTER and its name.</summary>
        public string Tables = "";

        public int SurfacesTotal;

        /// <summary>A cache hit or a saved sweep handed over (KsaAeroSweepCache), not a sweep: seconds, not minutes.</summary>
        public bool Cached;

        private int _surfacesDone;
        private long _surfaceStarted;
        private double _surfaceSecondsSum;
        private int _surfacesTimed;
        private int _surfacesMissing;
        private int _stage;
        private string _surface = "";
        private string _error = "";
        private readonly long _started = Stopwatch.GetTimestamp();
        private long _bodyReady, _finished;

        public Stage Now => (Stage)Volatile.Read(ref _stage);
        public int SurfacesDone => Volatile.Read(ref _surfacesDone);
        /// <summary>Surfaces the private mesh did not have (their pose binder found no panels): skipped, so the craft flies without them.</summary>
        public int SurfacesMissing => Volatile.Read(ref _surfacesMissing);
        /// <summary>The surface being swept now.</summary>
        public string Surface => Volatile.Read(ref _surface);
        public string Error => Volatile.Read(ref _error);
        /// <summary>Seconds since the sweep started, to its end once it has ended.</summary>
        public double ElapsedS => Seconds(_started, Volatile.Read(ref _finished) is long f && f != 0 ? f : Stopwatch.GetTimestamp());
        /// <summary>Seconds from the start to the body table, NaN before it.</summary>
        public double BodyS => Volatile.Read(ref _bodyReady) is long b && b != 0 ? Seconds(_started, b) : double.NaN;
        /// <summary>Seconds since the sweep ended, NaN while it runs.</summary>
        public double AgoS => Volatile.Read(ref _finished) is long f && f != 0 ? Seconds(f, Stopwatch.GetTimestamp()) : double.NaN;
        public bool Running => Now is not (Stage.Done or Stage.Failed);

        private static double Seconds(long from, long to) => (to - from) / (double)Stopwatch.Frequency;

        // THE ESTIMATE'S PRIORS: the body pass and one surface as the last sweep took them, this game session. Own guesses before any: 10 s and 4 s (2026-11-25: 9-10 s body on a Starship; a flap 10 s on 27 Mach rows, about a third of that on the surfaces' ten).
        private static double _bodyPriorS = 10.0, _surfacePriorS = 4.0;

        /// <summary>
        /// Seconds until this sweep has every table, an estimate: the body pass (the last sweep's, less the time already gone) and each surface left (this sweep's own mean once one is done, else the last sweep's), less the time the current one has run; a few seconds for a cache hit. At least one second while it runs, zero once it has ended.
        /// </summary>
        public double RemainingS
        {
            get
            {
                if (!Running)
                    return 0.0;
                if (Cached)
                    return Math.Max(2.0 - ElapsedS, 1.0);
                int timed = Volatile.Read(ref _surfacesTimed);
                double perSurface = timed > 0 ? Volatile.Read(ref _surfaceSecondsSum) / timed : Volatile.Read(ref _surfacePriorS);
                int left = Math.Max(SurfacesTotal - SurfacesDone, 0);
                // A stage that runs past its estimate is not nearly done: it is taken to need a tenth of the estimate again, then another, so the time left grows with it rather than sitting at a second (2026-11-25: flaps took 24 s in the air against a 4 s prior, and the band read ~1 S for 20 s each).
                static double Left(double estimate, double gone) => Math.Max(estimate - gone, 0.1 * Math.Max(estimate, gone));
                double remaining;
                if (Now is Stage.Mesh or Stage.Body)
                    remaining = Left(Volatile.Read(ref _bodyPriorS), ElapsedS) + left * perSurface;
                else
                {
                    long started = Volatile.Read(ref _surfaceStarted);
                    double inSurface = started != 0 ? Seconds(started, Stopwatch.GetTimestamp()) : 0.0;
                    remaining = left > 0 ? Left(perSurface, inSurface) + (left - 1) * perSurface : 0.0;
                }
                return Math.Max(remaining, 1.0);
            }
        }

        internal void Enter(Stage stage)
        {
            if (!Cached && stage == Stage.Surfaces && Volatile.Read(ref _bodyReady) == 0)
                Volatile.Write(ref _bodyPriorS, ElapsedS);
            if (stage == Stage.Surfaces || (stage == Stage.Done && Volatile.Read(ref _bodyReady) == 0))
                Interlocked.CompareExchange(ref _bodyReady, Stopwatch.GetTimestamp(), 0);
            if (stage is Stage.Done or Stage.Failed)
                Interlocked.CompareExchange(ref _finished, Stopwatch.GetTimestamp(), 0);
            Volatile.Write(ref _stage, (int)stage);
        }

        internal void SurfaceStarted(string name)
        {
            Volatile.Write(ref _surface, name ?? "");
            Volatile.Write(ref _surfaceStarted, Stopwatch.GetTimestamp());
        }

        internal void SurfaceEnded(bool missing)
        {
            if (missing)
                Interlocked.Increment(ref _surfacesMissing);
            else if (Volatile.Read(ref _surfaceStarted) is long started && started != 0)
            {
                // Written by the sweep thread only; write sum before count so the UI thread never divides a stale sum.
                double took = Seconds(started, Stopwatch.GetTimestamp());
                Volatile.Write(ref _surfaceSecondsSum, _surfaceSecondsSum + took);
                Volatile.Write(ref _surfacesTimed, _surfacesTimed + 1);
                Volatile.Write(ref _surfacePriorS, took);
            }
            Interlocked.Increment(ref _surfacesDone);
        }

        /// <summary>A cache hit's row: the surface tables it handed over, of those the sweep had; the rest found no part on the craft now.</summary>
        internal void Handed(int swept, int kept)
        {
            SurfacesTotal = swept;
            Volatile.Write(ref _surfacesMissing, Math.Max(swept - kept, 0));
            Volatile.Write(ref _surfacesDone, swept);
        }

        /// <summary>Why part of the sweep is missing, the sweep itself going on.</summary>
        internal void Note(string error) => Volatile.Write(ref _error, error ?? "");

        internal void Fail(string error)
        {
            Volatile.Write(ref _error, error ?? "");
            Enter(Stage.Failed);
        }
    }

    /// <summary>How many ended sweeps the status keeps, newest first, beside every one still running.</summary>
    private const int EndedSweepsKept = 6;

    private static readonly object SweepsGate = new();
    private static readonly List<SweepProgress> Sweeps = new();

    private static SweepProgress Track(Prepared p) => TrackCached(p.Name ?? "", TablesText(p), p.Surfaces.Count, cached: false);

    /// <summary>A row of its own: a sweep, or a cache hit handing one over (KsaAeroSweepCache).</summary>
    private static SweepProgress TrackCached(string craft, string tables, int surfaces, bool cached = true)
    {
        var progress = new SweepProgress
        {
            Craft = craft,
            Tables = tables,
            SurfacesTotal = surfaces,
            Cached = cached,
        };
        lock (SweepsGate)
        {
            Sweeps.Add(progress);
            int ended = 0;
            for (int i = Sweeps.Count - 1; i >= 0; i--)
            {
                if (Sweeps[i].Running)
                    continue;
                if (++ended > EndedSweepsKept)
                    Sweeps.RemoveAt(i);
            }
        }
        return progress;
    }

    /// <summary>The longest any sweep of this craft still has to run, s (SweepProgress.RemainingS), with whether all of them are cache hand-overs; 0 when none runs. Main thread.</summary>
    public static (double Seconds, bool Loading) AeroDataRemaining(string craft)
    {
        double seconds = 0.0;
        bool loading = true, any = false;
        lock (SweepsGate)
        {
            foreach (SweepProgress p in Sweeps)
            {
                if (p.Craft != craft || !p.Running)
                    continue;
                any = true;
                seconds = Math.Max(seconds, p.RemainingS);
                loading &= p.Cached;
            }
        }
        return any ? (seconds, loading) : (0.0, false);
    }

    /// <summary>The sweeps of a craft (its whole-craft and booster sweeps), newest first: every one still running and the last few ended. Main thread.</summary>
    public static List<SweepProgress> SweepsOf(string craft, List<SweepProgress> into)
    {
        into.Clear();
        lock (SweepsGate)
        {
            for (int i = Sweeps.Count - 1; i >= 0; i--)
                if (Sweeps[i].Craft == craft)
                    into.Add(Sweeps[i]);
        }
        return into;
    }

    private static string TablesText(Prepared p)
    {
        if (p.BoosterName.Length > 0)
            return "BOOSTER " + p.BoosterName;
        string text = "TAIL";
        if (p.NoseFirst)
            text += " + NOSE";
        if (p.BellyFirst)
            text += " + BELLY TRIM";
        if (p.BellySide >= 0 && p.BellySide < BellyAxes.Length)
        {
            Brutal.Numerics.double3 a = BellyAxes[p.BellySide];
            text += " + BELLY " + (a.Y + a.Z > 0 ? "+" : "-") + (a.Y != 0 ? "Y" : "Z");
        }
        return text;
    }
}
