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
    // The commands on both sides of a staging boundary share an instant.
    private readonly double[] _time, _pitch, _altitude, _speed, _throttle;
    private readonly double3[] _steering;
    private readonly int[] _firstNode, _lastNode;

    // Per planned stage: the stretch of plan time it burns over; the mass it starts at and burns out at; the air speed it lights at; and speed gained since then to plan time over the nodes within it at which the plan had gained more than at any earlier node of the stage - monotone by construction.
    private readonly double[] _stageStart, _stageEnd, _stageStartMass, _stageEndMass;
    private readonly double[][] _recordSpeed, _recordTime;

    public int Count => _time.Length;
    public int Stages => _stageStart.Length;
    public double EndTime => _time[^1];
    public double StageEndTime(int stage) => _stageEnd[stage];
    public ReadOnlySpan<double> Time => _time;
    public ReadOnlySpan<double> Speed => _speed;
    public ReadOnlySpan<double> PitchRad => _pitch;
    public ReadOnlySpan<double> Altitude => _altitude;

    private ConvexAscentProfile(double[] time, double[] pitch, double3[] steering, double[] altitude, double[] speed,
                                double[] throttle, double[] stageStart, double[] stageEnd,
                                double[] stageStartMass, double[] stageEndMass,
                                double[][] recordSpeed, double[][] recordTime, int[] firstNode, int[] lastNode)
    {
        _time = time;
        _pitch = pitch;
        _steering = steering;
        _altitude = altitude;
        _speed = speed;
        _throttle = throttle;
        _stageStart = stageStart;
        _stageEnd = stageEnd;
        _stageStartMass = stageStartMass;
        _stageEndMass = stageEndMass;
        _recordSpeed = recordSpeed;
        _recordTime = recordTime;
        _firstNode = firstNode;
        _lastNode = lastNode;
    }

    /// <param name="omega">The body's spin rate about CCI +z, rad/s: the air co-rotates.</param>
    /// <param name="bodyRadius">Mean radius, for the altitude column.</param>
    public static ConvexAscentProfile FromSolution(AscentSolution s, double omega, double bodyRadius)
    {
        int n = s.Nodes;
        var time = new List<double>(n);
        var pitch = new List<double>(n);
        var steering = new List<double3>(n);
        var alt = new List<double>(n);
        var speed = new List<double>(n);
        var throttle = new List<double>(n);

        for (int k = 0; k < n; k++)
        {
            var r = new double3(s.Position[k * 3], s.Position[k * 3 + 1], s.Position[k * 3 + 2]);
            var v = new double3(s.Velocity[k * 3], s.Velocity[k * 3 + 1], s.Velocity[k * 3 + 2]);
            var u = new double3(s.Throttle[k * 3], s.Throttle[k * 3 + 1], s.Throttle[k * 3 + 2]);
            double3 up = double3.Normalize(r);
            double thrustFraction = u.Length();
            double3 dir = thrustFraction > 1e-9 ? u / thrustFraction : up;
            double3 east = double3.Cross(new double3(0, 0, 1), up);
            east = east.Length() > 1e-9 ? double3.Normalize(east) : new double3(1, 0, 0);
            double3 north = double3.Cross(up, east);
            var local = new double3(double3.Dot(dir, north), double3.Dot(dir, east), double3.Dot(dir, up));
            double sp = (v - double3.Cross(new double3(0, 0, omega), r)).Length();

            time.Add(s.Time[k]);
            pitch.Add(Math.Asin(Math.Clamp(local.Z, -1.0, 1.0)));
            steering.Add(double3.Normalize(local));
            alt.Add(r.Length() - bodyRadius);
            speed.Add(sp);
            throttle.Add(Math.Min(thrustFraction, 1.0));
        }
        if (time.Count < 2)
            return null;

        // The stages: where each starts and ends in plan time, and in mass. Masses come from the solution's own nodes, both of each staging pair, so the burnout mass before a jettison is kept.
        int stages = s.BurnTime.Length;
        var stageStart = new double[stages];
        var stageEnd = new double[stages];
        var startMass = new double[stages];
        var endMass = new double[stages];
        var firstNode = new int[stages];
        var lastNode = new int[stages];
        for (int i = 0; i < stages; i++)
        {
            int first = Array.IndexOf(s.NodeStage, i), last = Array.LastIndexOf(s.NodeStage, i);
            firstNode[i] = first;
            lastNode[i] = last;
            stageStart[i] = s.Time[first];
            stageEnd[i] = s.Time[last];
            startMass[i] = s.Mass[first];
            endMass[i] = s.Mass[last];
        }

        var recordSpeed = new double[stages][];
        var recordTime = new double[stages][];
        var startSpeed = new double[stages];
        for (int i = 0; i < stages; i++)
        {
            // The first stage's records are in speed itself (gained from nothing); the others' in speed gained since the stage lit.
            startSpeed[i] = i == 0 ? 0.0 : speed[firstNode[i]];
            var rs = new List<double>();
            var rt = new List<double>();
            for (int k = firstNode[i]; k <= lastNode[i]; k++)
                if (rs.Count == 0 || speed[k] - startSpeed[i] > rs[^1] + 1e-6)
                {
                    rs.Add(speed[k] - startSpeed[i]);
                    rt.Add(time[k]);
                }
            recordSpeed[i] = rs.ToArray();
            recordTime[i] = rt.ToArray();
        }

        return new ConvexAscentProfile(time.ToArray(), pitch.ToArray(), steering.ToArray(), alt.ToArray(), speed.ToArray(),
                                       throttle.ToArray(), stageStart, stageEnd, startMass, endMass,
                                       recordSpeed, recordTime, firstNode, lastNode);
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
            return _stageStart[stage];
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
        => Attitude(planTime, StageAtTime(planTime), out pitch, out azimuth);

    public void Attitude(double planTime, int stage, out double pitch, out double azimuth)
    {
        double3 dir = SteerLocal(planTime, stage);
        pitch = Math.Asin(Math.Clamp(dir.Z, -1.0, 1.0));
        azimuth = Math.Atan2(dir.Y, dir.X);
    }

    /// <summary>The plan's altitude at a plan time, m.</summary>
    public double AltitudeAt(double planTime)
    {
        Interpolate(planTime, StageAtTime(planTime), out int lo, out int hi, out double f);
        return _altitude[lo] + f * (_altitude[hi] - _altitude[lo]);
    }

    /// <summary>The plan's throttle at a plan time, a fraction of full thrust at the altitude flown.</summary>
    public double Throttle(double planTime)
        => Throttle(planTime, StageAtTime(planTime));

    public double Throttle(double planTime, int stage)
    {
        Interpolate(planTime, stage, out int lo, out int hi, out double f);
        return _throttle[lo] + f * (_throttle[hi] - _throttle[lo]);
    }

    /// <summary>The lowest throttle anywhere in the plan.</summary>
    public double MinThrottle => _throttle.Min();

    /// <summary>The plan's air speed at a plan time, m/s.</summary>
    public double SpeedAt(double planTime)
    {
        Interpolate(planTime, StageAtTime(planTime), out int lo, out int hi, out double f);
        return _speed[lo] + f * (_speed[hi] - _speed[lo]);
    }

    private int StageAtTime(double t)
    {
        int stage = Array.BinarySearch(_stageStart, t);
        return stage >= 0 ? stage : Math.Max(0, ~stage - 1);
    }

    private void Interpolate(double t, int stage, out int lo, out int hi, out double f)
    {
        int first = _firstNode[stage], last = _lastNode[stage];
        if (t <= _time[first]) { lo = hi = first; f = 0.0; return; }
        if (t >= _time[last]) { lo = hi = last; f = 0.0; return; }
        hi = Array.BinarySearch(_time, first, last - first + 1, t);
        if (hi < 0) hi = ~hi;
        lo = hi - 1;
        f = (t - _time[lo]) / (_time[hi] - _time[lo]);
    }

    /// <summary>The commanded thrust direction, CCI, at a position and a plan time.</summary>
    public double3 Direction(double3 position, double planTime)
        => Direction(position, planTime, StageAtTime(planTime));

    public double3 Direction(double3 position, double planTime, int stage)
    {
        double3 dir = SteerLocal(planTime, stage);
        double3 up = double3.Normalize(position);
        double3 east = double3.Cross(new double3(0, 0, 1), up);
        east = east.Length() > 1e-9 ? double3.Normalize(east) : new double3(1, 0, 0);
        double3 north = double3.Cross(up, east);
        return dir.X * north + dir.Y * east + dir.Z * up;
    }

    private double3 SteerLocal(double planTime, int stage)
    {
        Interpolate(planTime, stage, out int lo, out int hi, out double f);
        double3 from = _steering[lo], to = _steering[hi];
        if (f <= 0.0)
            return from;
        if (f >= 1.0)
            return to;

        // An azimuth sweep near vertical can take a longer arc than the angle between the commands.
        double cos = Math.Clamp(double3.Dot(from, to), -1.0, 1.0);
        double3 tangent = to - cos * from;
        double sin = tangent.Length();
        if (sin <= 1e-12)
        {
            if (cos >= 0.0)
                return from;
            tangent = double3.Cross(from, Math.Abs(from.Z) < 0.9 ? double3.UnitZ : double3.UnitX);
        }
        double angle = f * Math.Atan2(sin, cos);
        return Math.Cos(angle) * from + Math.Sin(angle) * double3.Normalize(tangent);
    }
}

/// <summary>The plan's trajectory as the LAUNCH page plots it, one entry per node.</summary>
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
/// One returnable stage's way down from a plan (BoosterReturn), with what the pictures need: which booster, how it comes home, and its path's altitude and downrange from the pad, km, on the same footing as AscentPlanSeries.
/// </summary>
public sealed class BoosterTrackView
{
    public uint RootId { get; init; }
    public string Name { get; init; } = "";
    public BoosterReturnTrack Track { get; init; }
    public double[] AltitudeKm { get; init; } = [];
    public double[] DownrangeKm { get; init; } = [];
    public BoosterReturnMode Mode => Track.Mode;

    /// <summary>The track, its points turned about the spin axis by <paramref name="turn"/> (radians) into the frame the plan is published in, with altitude and downrange from <paramref name="pad"/>, the unit direction of the pad at lift-off.</summary>
    public static BoosterTrackView From(uint rootId, string name, BoosterReturnTrack track, double turn, double3 pad, double bodyRadius)
    {
        if (!track.Ok)
            return new BoosterTrackView { RootId = rootId, Name = name, Track = track };
        double c = Math.Cos(turn), s = Math.Sin(turn);
        double[] g = (double[])track.Ground.Clone();
        for (int i = 0; i + 2 < g.Length; i += 3)
        {
            double x = g[i], y = g[i + 1];
            g[i] = c * x - s * y;
            g[i + 1] = s * x + c * y;
        }
        double[] impact = [c * track.Impact[0] - s * track.Impact[1], s * track.Impact[0] + c * track.Impact[1], track.Impact[2]];
        int n = track.Points;
        var alt = new double[n];
        var down = new double[n];
        for (int k = 0; k < n; k++)
        {
            var p = new double3(g[k * 3], g[k * 3 + 1], g[k * 3 + 2]);
            double rl = p.Length();
            alt[k] = (rl - bodyRadius) / 1000.0;
            down[k] = bodyRadius * Math.Acos(Math.Clamp(double3.Dot(p / rl, pad), -1.0, 1.0)) / 1000.0;
        }
        return new BoosterTrackView
        {
            RootId = rootId,
            Name = name,
            Track = track with { Ground = g, Impact = impact },
            AltitudeKm = alt,
            DownrangeKm = down,
        };
    }
}

/// <summary>
/// A finished ascent plan and what it was planned for. Immutable once built: the worker publishes it, the flight reads it, and a recalculation replaces it whole rather than editing it under a vehicle that may be flying it.
/// </summary>
public sealed class AscentPlan
{
    public AscentSolution Solution { get; init; }
    public ConvexAscentProfile Profile { get; init; }

    /// <summary>Per-node series for the plots of the LAUNCH page.</summary>
    public AscentPlanSeries Series { get; init; }

    public string VehicleId { get; init; }
    public string BodyName { get; init; }
    /// <summary>The body it was planned over, for identity only - never read from the worker.</summary>
    public object Body { get; init; }
    public double Omega { get; init; }
    public double BodyRadius { get; init; }
    /// <summary>Sim time of the state the plan starts from - what the drawn plan is rotated forward from. The lift-off it flies: when it was requested, or the launch window's instant when the launch commit armed for one, plus <see cref="LaunchDelayS"/>.</summary>
    public double SolvedAt { get; init; }
    /// <summary>The lift-off the problem was posed for: the launch window's instant, or the moment it was requested. A plan with a free launch time lifts off <see cref="LaunchDelayS"/> after it.</summary>
    public double WindowAt { get; init; }
    /// <summary>How long after <see cref="WindowAt"/> the plan lifts off, s, negative for before: the launch time it chose (VehicleAutopilotState.ConvexOptimiseTiming). Zero for a plan whose launch time was not free.</summary>
    public double LaunchDelayS { get; init; }
    /// <summary>The plan chose its LAN (no target, ConvexOptimiseTiming): <see cref="LanDeg"/> is that LAN, and the LAN setting is set to it.</summary>
    public bool LanChosen { get; init; }
    /// <summary>ConvexOptimiseTiming when it was planned, so a change to it asks for a new plan.</summary>
    public bool OptimisedTiming { get; init; }
    /// <summary>The boosters and landing targets it sized reserves for, GuidanceWindow.RecoverySitesKey; empty with no target. A target set, moved or cleared since asks for a new plan.</summary>
    public string RecoverySite { get; init; } = "";
    /// <summary>Each returnable stage's way down from this plan's separation, in staging order: to its site, to its drone ship's point, or its fall when it is expendable. For the LAUNCH profile and the map, and the optimal returns' landing points.</summary>
    public BoosterTrackView[] BoosterTracks { get; init; } = [];

    /// <summary>The root part of each booster the plan brings home, in the order of the problem's recoveries: entry j of <see cref="AscentSolution.Recoveries"/> is the booster rooted at entry j here. A stack planned on fewer stages solves only a head of them.</summary>
    public uint[] RecoveryRoots { get; init; } = [];

    /// <summary>The plan's entry for the booster rooted at <paramref name="rootId"/>, or false when it brings no such booster home.</summary>
    public bool TryGetRecovery(uint rootId, out AdvancedFlightComputer.Guidance.Scvx.Ascent.AscentRecoveryResult result)
    {
        result = default;
        int j = Array.IndexOf(RecoveryRoots, rootId);
        if (Solution == null || j < 0 || j >= Solution.Recoveries.Length)
            return false;
        result = Solution.Recoveries[j];
        return true;
    }
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
    /// <summary>The floor as the LAUNCH page asked for it, before the engines' own minimum and the 99 % cap.</summary>
    public double ThrottleMinRequestedPct { get; init; }
    /// <summary>The player's g-limit the plan was solved to, g, positive infinity when it was off.</summary>
    public double GLimitG { get; init; } = double.PositiveInfinity;

    /// <summary>A plan whose dynamic pressure reaches this is not flown, kPa. Positive infinity for none.</summary>
    public double QCeilingKpa { get; init; } = double.PositiveInfinity;
    public double PeakQKpa => Solution is { Nodes: > 0 } s ? s.DynamicPressure.Max() / 1000.0 : 0.0;
    public bool OverQCeiling => PeakQKpa >= QCeilingKpa;
    public int SolidStages { get; init; }
    public bool StartsWithSolid { get; init; }
    public bool PlansPadRoll { get; init; }
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

    /// <summary>The stages, numbered from 1, that this plan throttles although the flight burns them at full thrust after the hand-over, because the solve with them held at full thrust did not converge. Empty when the plan burns every stage as the flight does.</summary>
    public string ThrottledStages { get; init; } = "";
    /// <summary>The mass to orbit of the last try of that held solve, kg, NaN when it accepted no step or did not end under this plan's <see cref="AscentSolution.FinalMass"/>. It comes from a solve that did not converge, so it is only a rough guide to what the flight delivers.</summary>
    public double HeldTryMassKg { get; init; } = double.NaN;

    /// <summary>Fit to fly (<see cref="AscentSolution.Usable"/>) and under the dynamic-pressure ceiling.</summary>
    public bool Usable => Solution != null && Solution.Usable && Profile != null && !OverQCeiling;

    /// <summary>True if the target orbit on the LAUNCH page is not the one this plan was solved for.</summary>
    public bool TargetDiffers(double peKm, double apKm, double incDeg, double lanDeg)
        => Math.Abs(peKm - PeKm) > 0.5 || Math.Abs(apKm - ApKm) > 0.5
        || Math.Abs(incDeg - IncDeg) > 0.05 || Math.Abs(Math.IEEERemainder(lanDeg - LanDeg, 360.0)) > 0.5;

    /// <summary>True if the limits on the LAUNCH page are not the ones this plan was solved to. The structural limit is not compared, because it follows the parts, and a part change moves the mass the plan is checked against.</summary>
    public bool SettingsDiffer(double qMaxKpa, double qAlphaMax, double throttleMinPct, double gLimitG)
        => Math.Abs(qMaxKpa - QMaxRequestedKpa) > 1e-6 || Math.Abs(qAlphaMax - QAlphaMax) > 1e-6
        || Math.Abs(throttleMinPct - ThrottleMinRequestedPct) > 1e-6
        || !(gLimitG == GLimitG || Math.Abs(gLimitG - GLimitG) <= 1e-6);
}

/// <summary>
/// The ascent solve, on its own thread. Seconds of work, so it never runs on the sim or the draw: the sim step builds the problem from the game (main thread only) and hands it here as plain data, and polls for the plan.
///
/// A MAX-Q THE VEHICLE CANNOT HOLD is relaxed here, once. The problem keeps the throttle within 1 % of full, as the script's does, and a KSA stack lifting off at three or five g climbs through the thick air well above the script's 35 kPa however steeply it goes - which is why the default MAX Q is four times that - and one that cannot hold the limit it is given: the problem then has no feasible trajectory, and SCvx can only hide the violation in its virtual control and stall. The seed search measures the least peak q the steepest climb reaches, so when every seed exceeded the limit the same stages are planned again to QRelaxFactor of that. A vehicle that can hold the limit is planned to it exactly.
///
/// NO RETRY ABOVE THE CEILING. A relaxed retry is planned at <see cref="AscentMaxQ.Highest"/> of the ceiling at most, and a solve already at or above that is not retried.
///
/// EVERY RETRY SAYS SO. An attempt that did not converge is followed by one with looser constraints - that max-q, or one more stage to burn - and <see cref="Retry"/> says which and why while it runs, so the console never shows a second solve as if it were the first.
///
/// HOW MANY STAGES THE PLAN BURNS is decided here, by trying. The script's formulation burns every stage but the last to depletion, which is right for a vehicle whose last stage is the one that reaches orbit and wrong for one that reaches orbit early: a stage that has to burn its whole load when orbit needed half of it can only waste the rest, and the plan it produces lofts and weaves to do so. So the first attempt plans only as many stages as the ideal dV says orbit needs, anything above them riding along as payload; a plan that cannot reach orbit adds the next stage, and one whose last stage is pinned at its shortest allowed burn drops it.
///
/// A STAGE THE PLAN MUST NOT THROTTLE is found from the converged plan, because which stages those are depends on where the plan flies them. The caller names them, and the same stages are solved once more held at full thrust, starting from the converged plan: from a gravity-turn seed the held solve takes several times the iterations and, with the trust region shrunk on the way, stops further from the optimum, where its steering at the hand-over can be some 20 deg from UPFG's. A held solve that does not converge leaves the first plan in place, and the publisher gets the stages that plan throttles and the held solve's last try (<see cref="ThrottledKept"/>).
/// </summary>
public sealed class AscentPlanJob : IDisposable
{
    private readonly CancellationTokenSource _cts = new();
    private readonly Thread _thread;

    private readonly Func<int, double, bool[], AscentProblem> _build;
    private readonly Func<AscentProblem, AscentSolution, bool[]> _toHold;
    private readonly int _available, _initial;
    private readonly double _qMax, _qCeiling;
    private readonly Func<AscentSolution, int, double, string[], double, ThrottledKept, AscentPlan> _publish;
    private readonly object _gate = new();
    private string _progress = "starting";
    private string _retry = "";
    private AscentPlan _result;
    private bool _done;
    private readonly System.Diagnostics.Stopwatch _clock = System.Diagnostics.Stopwatch.StartNew();

    /// <param name="build">The problem for a number of stages, a max-q, Pa, and the stages held at full thrust (null for none). Must touch nothing of the game.</param>
    /// <param name="available">Stages the vehicle has.</param>
    /// <param name="initial">Stages to try first.</param>
    /// <param name="qMax">The max-q asked for, Pa.</param>
    /// <param name="publish">Wraps the chosen solution, its stage count, the max-q it was planned to, the attempt notes, the wall time and the stages it throttles against a held solve that did not converge (null for none) in a plan; run on the worker, so it too must touch nothing of the game.</param>
    /// <param name="toHold">From the problem and its trajectory, the stages to solve again held at full thrust, or null for none. Run on the worker.</param>
    /// <param name="qCeiling">The ceiling on a retry's max-q, Pa. Positive infinity for none.</param>
    public AscentPlanJob(Func<int, double, bool[], AscentProblem> build, int available, int initial, double qMax,
                         Func<AscentSolution, int, double, string[], double, ThrottledKept, AscentPlan> publish,
                         Func<AscentProblem, AscentSolution, bool[]> toHold = null,
                         double qCeiling = double.PositiveInfinity)
    {
        _build = build;
        _toHold = toHold;
        _qMax = qMax;
        _qCeiling = qCeiling;
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
    private int _attempts;

    /// <summary>The stages, numbered from 1, that a plan throttles although the flight burns them at full thrust, because their held solve did not converge, and the mass to orbit of that solve's last try, kg, NaN without one.</summary>
    public sealed record ThrottledKept(string Stages, double TryMassKg);

    /// <summary>One line per attempt so far, and a line for each retry that was or could not be made.</summary>
    public string[] Notes { get { lock (_gate) return _notes.ToArray(); } }

    /// <summary>The solves of the search so far, not counting the final solve with stages held at full thrust.</summary>
    public int Attempts { get { lock (_gate) return _attempts; } }

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
            AscentProblem bestProblem = null;
            ThrottledKept kept = null;
            int bestK = 0;
            double bestQ = _qMax, qMax = _qMax;
            bool relaxed = false;
            var tried = new HashSet<int>();
            int k = _initial;
            while (tried.Add(k) && !_cts.IsCancellationRequested)
            {
                int stages = k;
                string what = $"{stages} stage{(stages == 1 ? "" : "s")}{(relaxed ? $", {qMax / 1000.0:F0} kPa" : "")}";
                AscentProblem problem = _build(stages, qMax, null);
                AscentSolution sol = SolveOne(problem, what);
                lock (_gate)
                {
                    _attempts++;
                    _notes.Add($"{stages} of {_available} stages at {qMax / 1000.0:F0} kPa: {sol.Message}"
                            + (sol.Nodes > 0 ? $", {sol.FinalMass / 1000.0:F2} t to orbit" : ""));
                }
                if (sol.Status == AscentStatus.Cancelled)
                    break;

                if (sol.Usable)
                {
                    best = sol;
                    bestProblem = problem;
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

                // Every seed climbed through more than the limit: re-plan the same stages, once, to a limit the vehicle can hold, but never above the ceiling.
                if (!relaxed && AscentMaxQ.OutOfReach(sol, qMax))
                {
                    double held = AscentMaxQ.Relaxed(sol.SeedLeastMaxQ, _qCeiling);
                    relaxed = true;
                    string reach = $"the steepest climb, at full thrust or at the acceleration limit, reaches {sol.SeedLeastMaxQ / 1000.0:F0} kPa";
                    if (held > qMax)
                    {
                        string capped = held < AscentMaxQ.Relaxed(sol.SeedLeastMaxQ, double.PositiveInfinity)
                            ? ", the highest a retry may use" : "";
                        lock (_gate)
                        {
                            _notes.Add($"did not converge at max q {qMax / 1000.0:F0} kPa, which is out of reach ({reach}), "
                                     + $"so retrying with looser constraints, max q {held / 1000.0:F0} kPa{capped}");
                            _retry = $"Did not converge: max q {qMax / 1000.0:F0} kPa is out of this vehicle's reach ({reach}). "
                                   + $"Retrying with looser constraints: max q {held / 1000.0:F0} kPa{capped}.";
                        }
                        qMax = held;
                        tried.Remove(stages);
                        continue;
                    }
                    lock (_gate)
                        _notes.Add($"did not converge at max q {qMax / 1000.0:F0} kPa, which is out of reach ({reach}), "
                                 + $"and no retry may use more than {AscentMaxQ.Highest(_qCeiling) / 1000.0:F0} kPa");
                }

                // Not converged. Keep the closest miss for the readout, but a plan that DID converge always wins.
                if (best == null || (!best.Usable && sol.Nodes > 0
                                     && (best.Nodes == 0 || TerminalMiss(sol) < TerminalMiss(best))))
                {
                    best = sol;
                    bestProblem = problem;
                    bestK = stages;
                    bestQ = qMax;
                }
                if (best.Usable)
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

            // The held throttle can also give a rate-constrained solid launch a feasible trajectory.
            if (best != null && (best.Usable || (bestProblem?.PadRoll != null && best.Nodes > 0))
                && _toHold != null && !_cts.IsCancellationRequested)
            {
                bool[] hold = _toHold(bestProblem, best);
                if (hold != null && Array.IndexOf(hold, true) >= 0)
                {
                    string which = string.Join(", ", Enumerable.Range(0, hold.Length).Where(i => hold[i]).Select(i => (i + 1).ToString()));
                    lock (_gate)
                        _retry = $"Stage {which} is flown at full thrust after the hand-over: planning it held at full thrust.";
                    AscentSolution held = SolveOne(_build(bestK, bestQ, hold), $"stage {which} held at full thrust", best);
                    lock (_gate)
                        _notes.Add($"{bestK} of {_available} stages at {bestQ / 1000.0:F0} kPa, stage {which} held at full thrust: {held.Message}"
                                + (held.Nodes > 0 ? $", {held.FinalMass / 1000.0:F2} t to orbit" : "")
                                + (!held.Usable && best.Usable ? "; the throttled plan is kept" : ""));
                    if (held.Usable)
                        best = held;
                    else if (best.Usable && held.Status != AscentStatus.Cancelled)
                        // The held solve starts from this plan, so until it accepts a step its mass is this plan's own.
                        kept = new ThrottledKept(which, held.Nodes > 0 && held.Accepted > 0 && held.FinalMass < best.FinalMass ? held.FinalMass : double.NaN);
                }
            }

            if (best != null && !_cts.IsCancellationRequested)
                plan = _publish(best, bestK, bestQ, Notes, ElapsedSeconds, kept);
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

    private AscentSolution SolveOne(AscentProblem problem, string what, AscentSolution start = null)
        => AscentScvx.Solve(problem,
            it => SetProgress($"{what}: iteration {it.Index + 1}, {it.Status}, miss {it.TerminalViolation:E1}, dV {it.DeltaV:F0} m/s"),
            phase => SetProgress($"{what}: {phase}"),
            _cts.Token, start);

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
