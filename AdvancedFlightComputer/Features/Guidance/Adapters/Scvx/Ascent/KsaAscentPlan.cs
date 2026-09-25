#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using Brutal.Numerics;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// The optimal ascent as the vehicle flies it: PITCH AND AZIMUTH AGAINST AIR-RELATIVE SPEED, read off the SCvx solution node by node.
///
/// INDEXED BY SPEED, NOT TIME. An open-loop program indexed by time flies the plan's attitude at the plan's clock whatever the vehicle is actually doing, so a vehicle a few percent heavier or weaker than modelled pitches over too early for the speed it has. Indexed by speed, the vehicle pitches when it has the speed the plan pitched at, which absorbs most of that mismatch - the same reason a gravity turn is programmed on velocity. Air-relative rather than inertial speed because it starts from zero on the pad (inertial speed starts at the pad's co-rotation, and barely moves through the vertical rise) and because it is what the aerodynamic loading the plan was shaped by depends on.
///
/// EXCEPT WHERE SPEED CANNOT BE AN INDEX. A KSA stack drags hard, and an upper stage lit in the thick air behind a short first stage can lose speed for a minute before it gains it back - while the plan pitches it over by thirty degrees. No table in speed can say that. So the profile is really a table in the PLAN'S TIME, and speed only says where on it the vehicle is: while the vehicle is accelerating to speeds it has never had, the plan time is the time the plan reached that speed; while it is not, the plan time runs on with the clock. Where speed rises monotonically that is exactly pitch against speed, and where it does not the plan is flown on its own schedule until speed catches up with it again.
///
/// AND STAGE BY STAGE. The plan's stages each own a stretch of plan time, and speed is only ever looked up within the stage the vehicle is actually burning, which its mass says (see <see cref="StageOfMass"/>). A vehicle that stages a little slower than planned would otherwise be sent, by the speeds its upper stage then reaches, to the moments the plan had those speeds - at the end of the plan's FIRST stage, accelerating four times as hard - and fly its upper stage on first-stage attitudes. And after the first stage the index is the speed GAINED since the stage lit, not the speed itself: a vehicle that stages slower than the plan never reaches the plan's upper-stage speeds until long after the plan has pitched on, while the speed gained starts from zero for both. From the pad the two are the same thing, so the first stage is pitch against speed exactly. Staging is where the plan and the vehicle are put back in step. See <see cref="Advance"/>.
/// </summary>
public sealed class ConvexAscentProfile
{
    // The plan by time: every node, one per instant (the second of each staging pair is kept, since it carries the new stage's thrust).
    private readonly double[] _time, _pitch, _azimuth, _altitude, _speed, _throttle;

    // Per planned stage: the stretch of plan time it burns over; the mass it starts at and burns out at; the air speed it lights at; and speed gained since then to plan time over the nodes within it at which the plan had gained more than at any earlier node of the stage - monotone by construction.
    private readonly double[] _stageStart, _stageEnd, _stageStartMass, _stageEndMass, _stageStartSpeed;
    private readonly double[][] _recordSpeed, _recordTime;

    public int Count => _time.Length;
    public int Stages => _stageStart.Length;
    public double EndTime => _time[^1];
    public ReadOnlySpan<double> Time => _time;
    public ReadOnlySpan<double> Speed => _speed;
    public ReadOnlySpan<double> PitchRad => _pitch;
    public ReadOnlySpan<double> Altitude => _altitude;

    private ConvexAscentProfile(double[] time, double[] pitch, double[] azimuth, double[] altitude, double[] speed,
                                double[] throttle, double[] stageStart, double[] stageEnd,
                                double[] stageStartMass, double[] stageEndMass, double[] stageStartSpeed,
                                double[][] recordSpeed, double[][] recordTime)
    {
        _stageStartSpeed = stageStartSpeed;
        _time = time;
        _pitch = pitch;
        _azimuth = azimuth;
        _altitude = altitude;
        _speed = speed;
        _throttle = throttle;
        _stageStart = stageStart;
        _stageEnd = stageEnd;
        _stageStartMass = stageStartMass;
        _stageEndMass = stageEndMass;
        _recordSpeed = recordSpeed;
        _recordTime = recordTime;
    }

    /// <param name="omega">The body's spin rate about CCI +z, rad/s: the air co-rotates.</param>
    /// <param name="bodyRadius">Mean radius, for the altitude column.</param>
    public static ConvexAscentProfile FromSolution(AscentSolution s, double omega, double bodyRadius)
    {
        int n = s.Nodes;
        var time = new List<double>(n);
        var pitch = new List<double>(n);
        var az = new List<double>(n);
        var alt = new List<double>(n);
        var speed = new List<double>(n);
        var throttle = new List<double>(n);
        var stage = new List<int>(n);
        var hasAz = new List<bool>(n);

        for (int k = 0; k < n; k++)
        {
            var r = new double3(s.Position[k * 3], s.Position[k * 3 + 1], s.Position[k * 3 + 2]);
            var v = new double3(s.Velocity[k * 3], s.Velocity[k * 3 + 1], s.Velocity[k * 3 + 2]);
            var u = new double3(s.Throttle[k * 3], s.Throttle[k * 3 + 1], s.Throttle[k * 3 + 2]);
            if (!(u.Length() > 1e-9))
                continue;

            double3 up = double3.Normalize(r);
            double3 dir = double3.Normalize(u);
            double3 east = double3.Cross(new double3(0, 0, 1), up);
            east = east.Length() > 1e-9 ? double3.Normalize(east) : new double3(1, 0, 0);
            double3 north = double3.Cross(up, east);
            double sinP = Math.Clamp(double3.Dot(dir, up), -1.0, 1.0);
            double3 horiz = dir - sinP * up;
            // Straight up has no azimuth; it is filled in from the first node that has one.
            bool defined = horiz.Length() > 1e-4;
            double a = defined ? Math.Atan2(double3.Dot(horiz, east), double3.Dot(horiz, north)) : 0.0;
            double sp = (v - double3.Cross(new double3(0, 0, omega), r)).Length();

            // Two nodes share each staging instant: the later one - the new stage's first - replaces the earlier.
            if (time.Count > 0 && !(s.Time[k] > time[^1] + 1e-9))
            {
                int last = time.Count - 1;
                pitch[last] = Math.Asin(sinP);
                az[last] = a;
                hasAz[last] = defined;
                alt[last] = r.Length() - bodyRadius;
                speed[last] = sp;
                throttle[last] = Math.Min(u.Length(), 1.0);
                stage[last] = s.NodeStage[k];
                continue;
            }
            time.Add(s.Time[k]);
            pitch.Add(Math.Asin(sinP));
            az.Add(a);
            hasAz.Add(defined);
            alt.Add(r.Length() - bodyRadius);
            speed.Add(sp);
            throttle.Add(Math.Min(u.Length(), 1.0));
            stage.Add(s.NodeStage[k]);
        }
        if (time.Count < 2)
            return null;

        // Back-fill the undefined azimuths from the next defined one, then unwrap so interpolation never goes the long way round.
        for (int k = 0; k < az.Count; k++)
            if (!hasAz[k])
            {
                int next = k;
                while (next < az.Count && !hasAz[next]) next++;
                az[k] = next < az.Count ? az[next] : (k > 0 ? az[k - 1] : 0.0);
            }
        for (int k = 1; k < az.Count; k++)
        {
            double d = az[k] - az[k - 1];
            while (d > Math.PI) { az[k] -= 2.0 * Math.PI; d -= 2.0 * Math.PI; }
            while (d < -Math.PI) { az[k] += 2.0 * Math.PI; d += 2.0 * Math.PI; }
        }

        // The stages: where each starts and ends in plan time, and in mass. Masses come from the solution's own nodes, both of each staging pair, so the burnout mass before a jettison is kept.
        int stages = s.BurnTime.Length;
        var stageStart = new double[stages];
        var stageEnd = new double[stages];
        var startMass = new double[stages];
        var endMass = new double[stages];
        for (int i = 0; i < stages; i++)
        {
            int first = Array.IndexOf(s.NodeStage, i), lastNode = Array.LastIndexOf(s.NodeStage, i);
            stageStart[i] = s.Time[first];
            stageEnd[i] = s.Time[lastNode];
            startMass[i] = s.Mass[first];
            endMass[i] = s.Mass[lastNode];
        }
        stageEnd[stages - 1] = time[^1];

        var recordSpeed = new double[stages][];
        var recordTime = new double[stages][];
        var startSpeed = new double[stages];
        for (int i = 0; i < stages; i++)
        {
            int first = stage.IndexOf(i);
            // The first stage's records are in speed itself (gained from nothing); the others' in speed gained since the stage lit.
            startSpeed[i] = i == 0 || first < 0 ? 0.0 : speed[first];
            var rs = new List<double>();
            var rt = new List<double>();
            for (int k = 0; k < speed.Count; k++)
                if (stage[k] == i && (rs.Count == 0 || speed[k] - startSpeed[i] > rs[^1] + 1e-6))
                {
                    rs.Add(speed[k] - startSpeed[i]);
                    rt.Add(time[k]);
                }
            if (rs.Count == 0)
            {
                rs.Add(0.0);
                rt.Add(stageStart[i]);
            }
            recordSpeed[i] = rs.ToArray();
            recordTime[i] = rt.ToArray();
        }

        return new ConvexAscentProfile(time.ToArray(), pitch.ToArray(), az.ToArray(), alt.ToArray(), speed.ToArray(),
                                       throttle.ToArray(), stageStart, stageEnd, startMass, endMass, startSpeed,
                                       recordSpeed, recordTime);
    }

    /// <summary>
    /// The planned stage a vehicle of this mass is burning. Mass falls monotonically through the ascent and each stage burns through its own band of it, so a separation drops the vehicle into the next band: it is in stage i once it weighs no more than stage i starts at, give or take a quarter of the structure the previous stage drops. A stage boundary with nothing dropped - an engine set changing - is crossed as the previous stage's propellant runs out, which is when the vehicle's own burn changes there too.
    /// </summary>
    public int StageOfMass(double mass)
    {
        int stage = 0;
        for (int i = 1; i < _stageStart.Length; i++)
        {
            double dropped = Math.Max(_stageEndMass[i - 1] - _stageStartMass[i], 0.0);
            if (mass <= _stageStartMass[i] + 0.25 * dropped)
                stage = i;
        }
        return stage;
    }

    /// <summary>
    /// Where on the plan the vehicle is, as plan time, from where it was a step ago. Carried between steps: <paramref name="stage"/>, the planned stage it is burning; <paramref name="stageSpeed"/>, the air speed it lit that stage at (zero for the first); and <paramref name="fastest"/>, the most speed it has gained in that stage.
    ///
    /// Gained more than ever before in this stage: the plan time at which the plan's same stage had first gained as much. Otherwise: the plan time runs on by the step. The first is pitch against speed; the second carries the vehicle through a stretch where speed dips - a burnout in thick air - on the plan's own schedule.
    ///
    /// ON STAGING the plan time moves up to the plan's own separation if it is short of it, and the speed gained starts again from the speed the vehicle staged at. Within a stage the plan time is held to that stage's stretch of the plan: a vehicle that burns longer than planned holds the plan's burnout attitude until it actually separates, instead of running on into the next stage's.
    ///
    /// NEVER BACKWARD. A vehicle that has gained more than ever in this stage but less than the plan at the same point waits for the plan time rather than pulling it back.
    /// </summary>
    public double Advance(double planTime, double speed, double mass, double dt, ref int stage, ref double stageSpeed,
                          ref double fastest)
    {
        int now = Math.Max(stage, StageOfMass(mass));
        if (now != stage)
        {
            stage = now;
            stageSpeed = speed;
            fastest = 0.0;
            planTime = Math.Max(planTime, _stageStart[stage]);
        }

        double gained = speed - stageSpeed;
        if (gained > fastest)
        {
            fastest = gained;
            planTime = Math.Max(planTime, TimeAtGain(stage, gained));
        }
        else
        {
            planTime += Math.Max(dt, 0.0);
        }
        return Math.Clamp(planTime, _stageStart[stage], _stageEnd[stage]);
    }

    /// <summary>The time the plan's given stage had first gained this much air speed since it lit - for the first stage, first reached this air speed. Held at the ends of the stage's records.</summary>
    public double TimeAtGain(int stage, double speed)
    {
        double[] rs = _recordSpeed[stage], rt = _recordTime[stage];
        int n = rs.Length;
        if (speed <= rs[0]) return rt[0];
        if (speed >= rs[n - 1]) return rt[n - 1];
        int hi = Array.BinarySearch(rs, speed);
        if (hi < 0) hi = ~hi;
        int lo = hi - 1;
        double f = (speed - rs[lo]) / (rs[hi] - rs[lo]);
        return rt[lo] + f * (rt[hi] - rt[lo]);
    }

    /// <summary>Pitch (above the horizon) and azimuth (from north, toward east) at a plan time, radians. Held at the ends.</summary>
    public void Attitude(double planTime, out double pitch, out double azimuth)
    {
        Interpolate(planTime, out int lo, out int hi, out double f);
        pitch = _pitch[lo] + f * (_pitch[hi] - _pitch[lo]);
        azimuth = _azimuth[lo] + f * (_azimuth[hi] - _azimuth[lo]);
    }

    /// <summary>The plan's altitude at a plan time, m.</summary>
    public double AltitudeAt(double planTime)
    {
        Interpolate(planTime, out int lo, out int hi, out double f);
        return _altitude[lo] + f * (_altitude[hi] - _altitude[lo]);
    }

    /// <summary>The plan's throttle at a plan time, a fraction of full thrust at the altitude flown.</summary>
    public double Throttle(double planTime)
    {
        Interpolate(planTime, out int lo, out int hi, out double f);
        return _throttle[lo] + f * (_throttle[hi] - _throttle[lo]);
    }

    /// <summary>The lowest throttle anywhere in the plan.</summary>
    public double MinThrottle => _throttle.Min();

    /// <summary>The plan's air speed at a plan time, m/s.</summary>
    public double SpeedAt(double planTime)
    {
        Interpolate(planTime, out int lo, out int hi, out double f);
        return _speed[lo] + f * (_speed[hi] - _speed[lo]);
    }

    private void Interpolate(double t, out int lo, out int hi, out double f)
    {
        int n = _time.Length;
        if (t <= _time[0]) { lo = hi = 0; f = 0.0; return; }
        if (t >= _time[n - 1]) { lo = hi = n - 1; f = 0.0; return; }
        hi = Array.BinarySearch(_time, t);
        if (hi < 0) hi = ~hi;
        lo = hi - 1;
        f = (t - _time[lo]) / (_time[hi] - _time[lo]);
    }

    /// <summary>The commanded thrust direction, CCI, at a position and a plan time.</summary>
    public double3 Direction(double3 position, double planTime)
    {
        Attitude(planTime, out double pitch, out double azimuth);
        double3 up = double3.Normalize(position);
        double3 east = double3.Cross(new double3(0, 0, 1), up);
        east = east.Length() > 1e-9 ? double3.Normalize(east) : new double3(1, 0, 0);
        double3 north = double3.Cross(up, east);
        return Math.Sin(pitch) * up + Math.Cos(pitch) * (Math.Cos(azimuth) * north + Math.Sin(azimuth) * east);
    }
}

/// <summary>The plan's trajectory as the panel plots it, one entry per node.</summary>
public sealed class AscentPlanSeries
{
    public double[] Time, AltitudeKm, DownrangeKm, AirSpeed, PitchDeg, QFraction, QAlphaFraction, ThrottlePct;
    /// <summary>The first node of every stage after the first: where each separation is.</summary>
    public int[] StagingNodes;

    /// <param name="omega">Body spin rate, rad/s: downrange is measured over the rotating ground.</param>
    public static AscentPlanSeries Build(AscentSolution s, double omega, double bodyRadius, double qMax, double qAlphaMax)
    {
        int n = s.Nodes;
        var ser = new AscentPlanSeries
        {
            Time = (double[])s.Time.Clone(),
            AltitudeKm = new double[n],
            DownrangeKm = new double[n],
            AirSpeed = new double[n],
            PitchDeg = new double[n],
            QFraction = new double[n],
            QAlphaFraction = new double[n],
            ThrottlePct = new double[n],
        };
        var staging = new List<int>();
        if (n == 0)
        {
            ser.StagingNodes = [];
            return ser;
        }
        var r0 = double3.Normalize(new double3(s.Position[0], s.Position[1], s.Position[2]));
        for (int k = 0; k < n; k++)
        {
            var r = new double3(s.Position[k * 3], s.Position[k * 3 + 1], s.Position[k * 3 + 2]);
            var v = new double3(s.Velocity[k * 3], s.Velocity[k * 3 + 1], s.Velocity[k * 3 + 2]);
            var u = new double3(s.Throttle[k * 3], s.Throttle[k * 3 + 1], s.Throttle[k * 3 + 2]);
            double rl = r.Length();
            ser.AltitudeKm[k] = (rl - bodyRadius) / 1000.0;
            // Back into the ground frame of lift-off: turn the point back by the angle the body has turned since.
            double ang = -omega * s.Time[k];
            var rg = new double3(r.X * Math.Cos(ang) - r.Y * Math.Sin(ang), r.X * Math.Sin(ang) + r.Y * Math.Cos(ang), r.Z);
            ser.DownrangeKm[k] = bodyRadius * Math.Acos(Math.Clamp(double3.Dot(double3.Normalize(rg), r0), -1.0, 1.0)) / 1000.0;
            ser.AirSpeed[k] = (v - double3.Cross(new double3(0, 0, omega), r)).Length();
            double ul = u.Length();
            ser.PitchDeg[k] = ul > 1e-9 ? Math.Asin(Math.Clamp(double3.Dot(u, r) / (ul * rl), -1.0, 1.0)) * 180.0 / Math.PI : 90.0;
            ser.QFraction[k] = s.DynamicPressure[k] / qMax;
            ser.QAlphaFraction[k] = s.QAlpha[k] / qAlphaMax;
            ser.ThrottlePct[k] = 100.0 * Math.Min(ul, 1.0);
            if (k > 0 && s.NodeStage[k] != s.NodeStage[k - 1])
                staging.Add(k);
        }
        ser.StagingNodes = staging.ToArray();
        return ser;
    }
}

/// <summary>
/// A finished ascent plan and what it was planned for. Immutable once built: the worker publishes it, the flight reads it, and a recalculation replaces it whole rather than editing it under a vehicle that may be flying it.
/// </summary>
public sealed class AscentPlan
{
    public AscentSolution Solution { get; init; }
    public ConvexAscentProfile Profile { get; init; }

    /// <summary>Per-node series for the panel's plots.</summary>
    public AscentPlanSeries Series { get; init; }

    public string VehicleId { get; init; }
    public string BodyName { get; init; }
    /// <summary>The body it was planned over, for identity only - never read from the worker.</summary>
    public object Body { get; init; }
    public double Omega { get; init; }
    public double BodyRadius { get; init; }
    /// <summary>Sim time of the state the plan starts from - what the drawn plan is rotated forward from. The lift-off it was solved for: when it was requested, or the launch window's instant when EXECUTE armed for one.</summary>
    public double SolvedAt { get; init; }
    public double Mass0 { get; init; }
    /// <summary>The lift-off position in the body-fixed frame, to tell whether the vehicle has moved since.</summary>
    public double3 StartCcf { get; init; }

    // What the plan was for.
    public double PeKm { get; init; }
    public double ApKm { get; init; }
    public double IncDeg { get; init; }
    public double LanDeg { get; init; }
    /// <summary>The max-q the plan was solved to, kPa; above <see cref="QMaxRequestedKpa"/> when that was out of the vehicle's reach.</summary>
    public double QMaxKpa { get; init; }
    public double QMaxRequestedKpa { get; init; }
    public bool QRelaxed => QMaxKpa > QMaxRequestedKpa + 1e-6;
    public double QAlphaMax { get; init; }
    public double InsertionAltKm { get; init; }

    /// <summary>The throttle floor the plan was solved to, percent, and how many planned stages have solid motors burning in them (#73).</summary>
    public double ThrottleMinPct { get; init; }
    /// <summary>The floor as the panel asked for it, before the engines' own minimum and the 99 % cap.</summary>
    public double ThrottleMinRequestedPct { get; init; }
    public int SolidStages { get; init; }
    /// <summary>A core that would have run dry before its boosters is planned throttled down to outlast them (#32).</summary>
    public bool CoreThrottledDown { get; init; }
    /// <summary>One line per planned stage, and what was decided about the solids: for the log and the tests.</summary>
    public IReadOnlyList<string> StageLines { get; init; } = [];
    public IReadOnlyList<string> StageNotes { get; init; } = [];

    /// <summary>What is left when the last planned stage is empty, kg: the stages above it and the payload.</summary>
    public double FinalMassFloor { get; init; }

    public int StagesPlanned { get; init; }
    public int StagesAvailable { get; init; }
    public IReadOnlyList<string> Notes { get; init; }
    public double WallSeconds { get; init; }

    public bool Usable => Solution != null && Solution.Converged && Profile != null;

    /// <summary>True if the target orbit on the panel is not the one this plan was solved for.</summary>
    public bool TargetDiffers(double peKm, double apKm, double incDeg, double lanDeg)
        => Math.Abs(peKm - PeKm) > 0.5 || Math.Abs(apKm - ApKm) > 0.5
        || Math.Abs(incDeg - IncDeg) > 0.05 || Math.Abs(Math.IEEERemainder(lanDeg - LanDeg, 360.0)) > 0.5;

    /// <summary>True if the panel's limits are not the ones this plan was solved to.</summary>
    public bool SettingsDiffer(double qMaxKpa, double qAlphaMax, double throttleMinPct)
        => Math.Abs(qMaxKpa - QMaxRequestedKpa) > 1e-6 || Math.Abs(qAlphaMax - QAlphaMax) > 1e-6
        || Math.Abs(throttleMinPct - ThrottleMinRequestedPct) > 1e-6;
}

/// <summary>
/// The ascent solve, on its own thread. Seconds of work, so it never runs on the sim or the draw: the sim step builds the problem from the game (main thread only) and hands it here as plain data, and polls for the plan.
///
/// A MAX-Q THE VEHICLE CANNOT HOLD is relaxed here, once. The problem keeps the throttle within 1 % of full, as the script's does, and a KSA stack lifting off at three or five g climbs through the thick air well above the script's 35 kPa however steeply it goes - which is why the panel's default is four times that - and one that cannot hold the limit it is given: the problem then has no feasible trajectory, and SCvx can only hide the violation in its virtual control and stall. The seed search measures the least peak q the steepest climb reaches, so when every seed exceeded the limit the same stages are planned again to QRelaxFactor of that. A vehicle that can hold the limit is planned to it exactly.
///
/// EVERY RETRY SAYS SO. An attempt that did not converge is followed by one with looser constraints - that max-q, or one more stage to burn - and <see cref="Retry"/> says which and why while it runs, so the panel never shows a second solve as if it were the first.
///
/// HOW MANY STAGES THE PLAN BURNS is decided here, by trying. The script's formulation burns every stage but the last to depletion, which is right for a vehicle whose last stage is the one that reaches orbit and wrong for one that reaches orbit early: a stage that has to burn its whole load when orbit needed half of it can only waste the rest, and the plan it produces lofts and weaves to do so. So the first attempt plans only as many stages as the ideal dV says orbit needs, anything above them riding along as payload; a plan that cannot reach orbit adds the next stage, and one whose last stage is pinned at its shortest allowed burn drops it.
/// </summary>
public sealed class AscentPlanJob : IDisposable
{
    private readonly CancellationTokenSource _cts = new();
    private readonly Thread _thread;
    /// <summary>A relaxed max-q sits this far above the least peak q the vehicle's steepest seed reached.</summary>
    public const double QRelaxFactor = 1.15;

    private readonly Func<int, double, AscentProblem> _build;
    private readonly int _available, _initial;
    private readonly double _qMax;
    private readonly Func<AscentSolution, int, double, string[], double, AscentPlan> _publish;
    private readonly object _gate = new();
    private string _progress = "starting";
    private string _retry = "";
    private AscentPlan _result;
    private bool _done;
    private readonly System.Diagnostics.Stopwatch _clock = System.Diagnostics.Stopwatch.StartNew();

    /// <param name="build">The problem for a number of stages and a max-q, Pa. Must touch nothing of the game.</param>
    /// <param name="available">Stages the vehicle has.</param>
    /// <param name="initial">Stages to try first.</param>
    /// <param name="qMax">The max-q asked for, Pa.</param>
    /// <param name="publish">Wraps the chosen solution, its stage count, the max-q it was planned to, the attempt notes and the wall time in a plan; run on the worker, so it too must touch nothing of the game.</param>
    public AscentPlanJob(Func<int, double, AscentProblem> build, int available, int initial, double qMax,
                         Func<AscentSolution, int, double, string[], double, AscentPlan> publish)
    {
        _build = build;
        _qMax = qMax;
        _available = available;
        _initial = Math.Clamp(initial, 1, available);
        _publish = publish;
        _thread = new Thread(Run)
        {
            IsBackground = true,
            Name = "afc-ascent-scvx",
            Priority = ThreadPriority.BelowNormal,
        };
        _thread.Start();
    }

    private readonly List<string> _notes = [];

    /// <summary>One line per attempt so far.</summary>
    public string[] Notes { get { lock (_gate) return _notes.ToArray(); } }

    public double ElapsedSeconds => _clock.Elapsed.TotalSeconds;

    public string Progress { get { lock (_gate) return _progress; } }

    /// <summary>Why the attempt running now is a retry: the one before it did not converge, and what was loosened. Empty on the first attempt.</summary>
    public string Retry { get { lock (_gate) return _retry; } }

    public bool IsDone { get { lock (_gate) return _done; } }

    /// <summary>The plan once done, else null. A cancelled job finishes with no plan.</summary>
    public AscentPlan Result { get { lock (_gate) return _done ? _result : null; } }

    public void Cancel() => _cts.Cancel();

    public void Dispose() => _cts.Cancel();

    private void SetProgress(string text)
    {
        lock (_gate) _progress = text;
    }

    private void Run()
    {
        AscentPlan plan = null;
        try
        {
            AscentSolution best = null;
            int bestK = 0;
            double bestQ = _qMax, qMax = _qMax;
            bool relaxed = false;
            var tried = new HashSet<int>();
            int k = _initial;
            while (tried.Add(k) && !_cts.IsCancellationRequested)
            {
                int stages = k;
                string what = $"{stages} stage{(stages == 1 ? "" : "s")}{(relaxed ? $", {qMax / 1000.0:F0} kPa" : "")}";
                AscentProblem problem = _build(stages, qMax);
                AscentSolution sol = AscentScvx.Solve(problem,
                    it => SetProgress($"{what}: iteration {it.Index + 1}, {it.Status}, miss {it.TerminalViolation:E1}, dV {it.DeltaV:F0} m/s"),
                    phase => SetProgress($"{what}: {phase}"),
                    _cts.Token);
                lock (_gate)
                    _notes.Add($"{stages} of {_available} stages at {qMax / 1000.0:F0} kPa: {sol.Message}"
                            + (sol.Nodes > 0 ? $", {sol.FinalMass / 1000.0:F2} t to orbit" : ""));
                if (sol.Status == AscentStatus.Cancelled)
                    break;

                if (sol.Converged)
                {
                    best = sol;
                    bestK = stages;
                    bestQ = qMax;
                    // The last stage pinned at its shortest allowed burn: the stages below it already reach orbit, and forcing it to burn at all distorts the plan.
                    if (stages > 1 && FinalStageIdle(problem, sol) && !tried.Contains(stages - 1))
                    {
                        lock (_gate)
                            _retry = $"Converged on {stages} stages, but the last one barely burns: planning {stages - 1} instead.";
                        k = stages - 1;
                        continue;
                    }
                    break;
                }

                // Every seed climbed through more than the limit: re-plan the same stages, once, to a limit the vehicle can hold.
                if (!relaxed && double.IsFinite(sol.SeedLeastMaxQ) && sol.SeedLeastMaxQ > 0.98 * qMax)
                {
                    double held = Math.Ceiling(QRelaxFactor * sol.SeedLeastMaxQ / 5000.0) * 5000.0;
                    lock (_gate)
                    {
                        _notes.Add($"did not converge at max q {qMax / 1000.0:F0} kPa, which is out of reach at full throttle - the steepest climb reaches "
                                 + $"{sol.SeedLeastMaxQ / 1000.0:F0} kPa - so retrying with looser constraints, max q {held / 1000.0:F0} kPa");
                        _retry = $"Did not converge: max q {qMax / 1000.0:F0} kPa is out of this vehicle's reach at full throttle (its steepest climb reaches "
                               + $"{sol.SeedLeastMaxQ / 1000.0:F0} kPa). Retrying with looser constraints: max q {held / 1000.0:F0} kPa.";
                    }
                    qMax = held;
                    relaxed = true;
                    tried.Remove(stages);
                    continue;
                }

                // Not converged. Keep the closest miss for the readout, but a plan that DID converge always wins.
                if (best == null || (!best.Converged && sol.Nodes > 0
                                     && (best.Nodes == 0 || TerminalMiss(sol) < TerminalMiss(best))))
                {
                    best = sol;
                    bestK = stages;
                    bestQ = qMax;
                }
                if (best.Converged)
                    break;
                // Short of orbit, or no flyable seed: plan one more stage while there is one.
                if (stages < _available)
                {
                    lock (_gate)
                        _retry = $"Did not converge on {stages} stage{(stages == 1 ? "" : "s")} ({sol.Message}). Retrying with looser constraints: {stages + 1} stages to burn.";
                    k = stages + 1;
                    continue;
                }
                break;
            }

            if (best != null && !_cts.IsCancellationRequested)
                plan = _publish(best, bestK, bestQ, Notes, ElapsedSeconds);
        }
        catch (Exception e)
        {
            lock (_gate) _notes.Add("solve threw: " + e.Message);
        }
        finally
        {
            lock (_gate)
            {
                _result = plan;
                _done = true;
                _progress = plan == null ? "no plan" : plan.Solution.Message;
            }
        }
    }

    private static double TerminalMiss(AscentSolution s)
        => s.TerminalResidual.Length == 0 ? double.PositiveInfinity
         : Math.Abs(s.TerminalResidual[0]) / 1000.0 + Math.Abs(s.TerminalResidual[1]);

    private static bool FinalStageIdle(AscentProblem p, AscentSolution s)
    {
        int last = p.Stages.Length - 1;
        // A solid burns its whole grain whatever the plan wants: its stage is never idle, only pinned.
        if (p.Stages[last].IsPinned)
            return false;
        double floor = Math.Min(p.Settings.SigmaMinSeconds, 0.5 * p.Stages[last].FullBurnTime);
        return s.BurnTime[last] <= floor * 1.05;
    }
}
