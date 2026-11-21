using System;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// Where the gimbals of the engines a landing burn leaves off are parked, as Starship parks its idle engines: each idle nozzle tilts away from the lit ones, as far as it gains by it, without coming near another nozzle. It is for looks. An idle engine makes no thrust, so its deflection changes nothing in the flight.
/// Game-independent, so the console checks run it without KSA: the adapter (KsaIdleEngineParking) gives every engine on the aft plane as the exits its nozzle can reach, in the vehicle frame, and takes back one choice per engine.
///  THE RULE. An idle nozzle takes the reachable exit that maximises its mean distance to the live exits less PushThreshold times how far it moved. The gradient of the mean distance is the mean of the unit vectors from the live exits to it: about 1 for an engine with all the lit engines on one side, such as a ring around a lit centre, and near 0 for one surrounded by them or at the centre of a lit ring. Under the threshold the engine stays at rest, over it it tilts to its limit, so a centre or surrounded engine stays put and a middle ring tilts out.
///  THE CLEARANCE. Two rims keep the largest of MinGapM, RadiusGapShare of the two exit radii together and RestGapShare of the gap they have at rest, so the gap scales with the engines and with how the cluster is packed: a parked nozzle closes at most half the room it had. A live engine is taken as its rest exit plus the full swing of its gimbal, so its own steering can never reach a parked nozzle. A pair already closer than that at rest may not come closer than it is at rest. Every engine at rest therefore always passes, and the search starts there and keeps to choices that pass.
///  JOINTLY. The idle engines move together, so the search takes them in turns, each one's best given the others' current choices, until a sweep changes nothing. An engine blocked by an idle outer ring at rest can move once the outer ring has tilted out of its way.
/// </summary>
internal static class IdleEngineParking
{
    /// <summary>The share of each metre moved that the mean distance to the live exits must gain for an idle nozzle to move at all.</summary>
    internal const double PushThreshold = 0.35;

    /// <summary>The least space kept between two nozzle rims, m.</summary>
    internal const double MinGapM = 0.05;

    /// <summary>The space kept between two rims as a share of their two exit radii together, so larger engines keep more.</summary>
    internal const double RadiusGapShare = 0.1;

    /// <summary>The share of the rim gap a pair has at rest that it keeps when parked.</summary>
    internal const double RestGapShare = 0.5;

    /// <summary>The search's sweeps at most. Each sweep moves every idle engine once.</summary>
    internal const int MaxSweeps = 8;

    /// <summary>Command magnitudes searched, as shares of the gimbal's range, with <see cref="Directions"/> directions each.</summary>
    internal const int Magnitudes = 10;
    internal const int Directions = 24;

    internal readonly record struct V3(double X, double Y, double Z)
    {
        public static V3 operator +(V3 a, V3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static V3 operator -(V3 a, V3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static V3 operator *(double s, V3 a) => new(s * a.X, s * a.Y, s * a.Z);
        public double Length() => Math.Sqrt(X * X + Y * Y + Z * Z);
        public static double Distance(V3 a, V3 b) => (a - b).Length();
    }

    /// <summary>
    /// One engine on the aft plane: its exits for each searched command, the first being its rest exit (command 0, 0), and its exit radius, m. A live engine or one without a gimbal holds only its rest exit, plus the reach of its gimbal for a live engine (how far its exit swings, m).
    /// </summary>
    internal sealed record Engine(bool Live, double RadiusM, V3[] Exits, double[] CommandY, double[] CommandZ, double ReachM = 0.0)
    {
        internal V3 Rest => Exits[0];
    }

    /// <summary>
    /// The commands searched, as (y, z) shares of the range: rest first, then <see cref="Magnitudes"/> steps out to the limit in <see cref="Directions"/> directions. With <paramref name="circle"/> the limit is the unit circle (Gimbal.ConstrainToCircle), otherwise the square of the two axes' own ranges.
    /// </summary>
    internal static (double Y, double Z)[] Commands(bool circle)
    {
        var commands = new (double Y, double Z)[1 + Magnitudes * Directions];
        commands[0] = (0.0, 0.0);
        int next = 1;
        for (int m = 1; m <= Magnitudes; m++)
        {
            double magnitude = (double)m / Magnitudes;
            for (int d = 0; d < Directions; d++)
            {
                double angle = 2.0 * Math.PI * d / Directions;
                double y = Math.Cos(angle), z = Math.Sin(angle);
                if (!circle)
                {
                    double edge = Math.Max(Math.Abs(y), Math.Abs(z));
                    y /= edge;
                    z /= edge;
                }
                commands[next++] = (magnitude * y, magnitude * z);
            }
        }
        return commands;
    }

    /// <summary>The choice for every engine, an index into its exits: 0 for a live or fixed engine and for an idle engine that stays at rest.</summary>
    internal static int[] Solve(Engine[] engines)
    {
        int n = engines.Length;
        var choice = new int[n];
        int live = 0;
        foreach (Engine e in engines)
            if (e.Live)
                live++;
        if (live == 0 || live == n)
            return choice;

        for (int sweep = 0; sweep < MaxSweeps; sweep++)
        {
            bool moved = false;
            for (int i = 0; i < n; i++)
            {
                Engine engine = engines[i];
                if (engine.Live || engine.Exits.Length <= 1)
                    continue;
                int best = choice[i];
                double bestScore = Score(engines, i, engine.Exits[best]);
                for (int c = 0; c < engine.Exits.Length; c++)
                {
                    if (c == choice[i])
                        continue;
                    V3 exit = engine.Exits[c];
                    double score = Score(engines, i, exit);
                    // Only a real gain moves an engine, so a tie keeps what it has and the search ends.
                    if (score > bestScore + 1e-9 && Clear(engines, choice, i, exit))
                    {
                        best = c;
                        bestScore = score;
                    }
                }
                if (best != choice[i])
                {
                    choice[i] = best;
                    moved = true;
                }
            }
            if (!moved)
                break;
        }
        return choice;
    }

    /// <summary>The mean distance from this exit to the live exits, less <see cref="PushThreshold"/> times how far it is from engine <paramref name="i"/>'s rest exit.</summary>
    internal static double Score(Engine[] engines, int i, V3 exit)
    {
        double sum = 0.0;
        int count = 0;
        foreach (Engine other in engines)
        {
            if (!other.Live)
                continue;
            sum += V3.Distance(exit, other.Rest);
            count++;
        }
        return (count > 0 ? sum / count : 0.0) - PushThreshold * V3.Distance(exit, engines[i].Rest);
    }

    /// <summary>The space two rims keep, m (see <see cref="RimGapM(double, double, double)"/>).</summary>
    internal static double RimGapM(Engine a, Engine b)
        => RimGapM(a.RadiusM, b.RadiusM, V3.Distance(a.Rest, b.Rest) - a.RadiusM - b.RadiusM);

    /// <summary>The space two rims keep, m: the largest of <see cref="MinGapM"/>, <see cref="RadiusGapShare"/> of their radii together and <see cref="RestGapShare"/> of their gap at rest.</summary>
    internal static double RimGapM(double radiusA, double radiusB, double restGapM)
        => Math.Max(MinGapM, Math.Max(RadiusGapShare * (radiusA + radiusB), RestGapShare * restGapM));

    /// <summary>The distance two exits keep, m: their radii and <see cref="RimGapM(Engine, Engine)"/>, plus a live engine's swing; at most what they have at rest.</summary>
    internal static double Required(Engine a, Engine b)
    {
        double need = a.RadiusM + b.RadiusM + RimGapM(a, b) + (a.Live ? a.ReachM : 0.0) + (b.Live ? b.ReachM : 0.0);
        return Math.Min(need, V3.Distance(a.Rest, b.Rest));
    }

    /// <summary>Whether engine <paramref name="i"/> at this exit keeps its distance to every other engine where they are now.</summary>
    internal static bool Clear(Engine[] engines, int[] choice, int i, V3 exit)
    {
        for (int j = 0; j < engines.Length; j++)
        {
            if (j == i)
                continue;
            Engine other = engines[j];
            V3 at = other.Live ? other.Rest : other.Exits[choice[j]];
            if (V3.Distance(exit, at) < Required(engines[i], other) - 1e-9)
                return false;
        }
        return true;
    }
}
