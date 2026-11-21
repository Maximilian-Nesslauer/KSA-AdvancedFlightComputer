using AdvancedFlightComputer.Features.Guidance;
using Engine = AdvancedFlightComputer.Features.Guidance.IdleEngineParking.Engine;
using V3 = AdvancedFlightComputer.Features.Guidance.IdleEngineParking.V3;

// The parking of idle engines on made-up clusters: the thrust axis is +Z, every pivot sits at z = 0 with its nozzle exit NozzleM below it, and a gimbal tilts about the X and Y axes by up to MaxDeg.
internal static partial class Program
{
    private const double NozzleM = 2.0, ExitRadiusM = 0.45, MaxDeg = 10.0;

    // How far an exit swings at the limit of its gimbal, m.
    private static readonly double ReachM = NozzleM * Math.Sin(MaxDeg * Math.PI / 180.0);

    private static V3 Exit(double x, double y, double cy, double cz)
    {
        double a = cy * MaxDeg * Math.PI / 180.0, b = cz * MaxDeg * Math.PI / 180.0;
        return new V3(x + NozzleM * Math.Sin(a), y + NozzleM * Math.Sin(b) * Math.Cos(a), -NozzleM * Math.Cos(a) * Math.Cos(b));
    }

    private static Engine Gimballed(double x, double y, bool live)
    {
        (double Y, double Z)[] commands = IdleEngineParking.Commands(circle: true);
        V3[] exits = commands.Select(c => Exit(x, y, c.Y, c.Z)).ToArray();
        if (live)
            return new Engine(true, ExitRadiusM, [exits[0]], [0.0], [0.0], exits.Max(e => V3.Distance(e, exits[0])));
        return new Engine(false, ExitRadiusM, exits, commands.Select(c => c.Y).ToArray(), commands.Select(c => c.Z).ToArray());
    }

    private static Engine Fixed(double x, double y, bool live) => new(live, ExitRadiusM, [Exit(x, y, 0, 0)], [0.0], [0.0]);

    private static IEnumerable<(double X, double Y)> Ring(int count, double radius, double phaseDeg = 0.0)
    {
        for (int i = 0; i < count; i++)
        {
            double angle = (phaseDeg + 360.0 * i / count) * Math.PI / 180.0;
            yield return (radius * Math.Cos(angle), radius * Math.Sin(angle));
        }
    }

    // The exit's move from rest, split into its part away from the axis and across it.
    private static (double Out, double Across, double Total) Move(Engine e, int choice)
    {
        V3 rest = e.Rest, at = e.Exits[choice];
        double r = Math.Sqrt(rest.X * rest.X + rest.Y * rest.Y);
        double ux = r > 1e-9 ? rest.X / r : 0.0, uy = r > 1e-9 ? rest.Y / r : 0.0;
        double dx = at.X - rest.X, dy = at.Y - rest.Y;
        double outward = dx * ux + dy * uy;
        return (outward, Math.Abs(-dx * uy + dy * ux), V3.Distance(at, rest));
    }

    // Every pair keeps its rims apart by the clearance, a live engine's full swing included, or no closer than at rest.
    private static bool AllClear(Engine[] engines, int[] choice, out string worst)
    {
        double least = double.PositiveInfinity;
        worst = "";
        for (int i = 0; i < engines.Length; i++)
            for (int j = i + 1; j < engines.Length; j++)
            {
                V3 a = engines[i].Live ? engines[i].Rest : engines[i].Exits[choice[i]];
                V3 b = engines[j].Live ? engines[j].Rest : engines[j].Exits[choice[j]];
                double spare = V3.Distance(a, b) - IdleEngineParking.Required(engines[i], engines[j]);
                if (spare < least)
                {
                    least = spare;
                    worst = $"{i}-{j} spare {spare:F3} m";
                }
            }
        return least >= -1e-9;
    }

    private static void IdleEngineParkingRules()
    {
        // A ring of 8 around 3 at the centre.
        (double X, double Y)[] centre = Ring(3, 0.6, 90.0).ToArray();
        (double X, double Y)[] ring = Ring(8, 2.2).ToArray();

        // The centre lit: every ring engine tilts straight out to its limit.
        Engine[] centreLit = [.. centre.Select(p => Gimballed(p.X, p.Y, live: true)), .. ring.Select(p => Gimballed(p.X, p.Y, live: false))];
        int[] choice = IdleEngineParking.Solve(centreLit);
        for (int i = centre.Length; i < centreLit.Length; i++)
        {
            (double outward, double across, _) = Move(centreLit[i], choice[i]);
            Check("with the centre lit, ring engine " + i + " tilts out to its limit", outward > 0.95 * ReachM && across < 0.05 * ReachM,
                $"out {outward:F3} m, across {across:F3} m, reach {ReachM:F3} m");
        }
        Check("the centre stays live and unparked", choice.Take(centre.Length).All(c => c == 0));
        Check("the parked ring keeps its clearance", AllClear(centreLit, choice, out string worst), worst);

        // The ring lit: the centre engines are surrounded and stay put.
        Engine[] ringLit = [.. centre.Select(p => Gimballed(p.X, p.Y, live: false)), .. ring.Select(p => Gimballed(p.X, p.Y, live: true))];
        choice = IdleEngineParking.Solve(ringLit);
        for (int i = 0; i < centre.Length; i++)
        {
            double moved = Move(ringLit[i], choice[i]).Total;
            Check("with the ring lit, centre engine " + i + " barely moves", moved < 0.1 * ReachM, $"moved {moved:F3} m");
        }

        // One lit centre, an idle middle ring of 6 and an idle fixed outer ring of 12, as on a booster whose outer engines do not gimbal. The middle ring tilts away from the centre only as far as the outer ring leaves it room.
        (double X, double Y)[] middle = Ring(6, 1.15).ToArray();
        (double X, double Y)[] outer = Ring(12, 2.3).ToArray();
        Engine[] fixedOuter = [Gimballed(0, 0, live: true), .. middle.Select(p => Gimballed(p.X, p.Y, live: false)), .. outer.Select(p => Fixed(p.X, p.Y, live: false))];
        choice = IdleEngineParking.Solve(fixedOuter);
        double fixedOut = double.PositiveInfinity;
        for (int i = 1; i <= middle.Length; i++)
        {
            (double outward, _, double total) = Move(fixedOuter[i], choice[i]);
            fixedOut = Math.Min(fixedOut, outward);
            Check("a middle engine " + i + " moves out from the lit centre", outward > 0.2 * ReachM, $"out {outward:F3} m");
            Check("a middle engine " + i + " stops short of the fixed outer ring", total < ReachM - 1e-6, $"moved {total:F3} m of {ReachM:F3} m");
        }
        Check("the middle ring keeps its clearance from the fixed outer ring", AllClear(fixedOuter, choice, out worst), worst);

        // The same with the outer ring on gimbals too: it tilts out first, and the middle ring follows it further than before.
        Engine[] gimballedOuter = [Gimballed(0, 0, live: true), .. middle.Select(p => Gimballed(p.X, p.Y, live: false)), .. outer.Select(p => Gimballed(p.X, p.Y, live: false))];
        choice = IdleEngineParking.Solve(gimballedOuter);
        for (int i = 1; i < gimballedOuter.Length; i++)
        {
            (double outward, _, _) = Move(gimballedOuter[i], choice[i]);
            bool isMiddle = i <= middle.Length;
            Check((isMiddle ? "a middle" : "an outer") + " engine " + i + " tilts out with the outer ring on gimbals",
                outward > (isMiddle ? fixedOut + 1e-6 : 0.95 * ReachM), $"out {outward:F3} m, the fixed outer ring let {fixedOut:F3} m");
        }
        Check("both rings keep their clearance", AllClear(gimballedOuter, choice, out worst), worst);

        // A live engine's own steering never reaches a parked nozzle: the idle one keeps clear of the live one's whole swing.
        Engine[] pair = [Gimballed(-1.4, 0, live: true), Gimballed(0, 0, live: false), Gimballed(1.4, 0, live: true)];
        choice = IdleEngineParking.Solve(pair);
        Check("an idle engine between two live ones stays put", Move(pair[1], choice[1]).Total < 0.1 * ReachM);
        Check("an idle engine between two live ones stays clear of both swings", AllClear(pair, choice, out worst), worst);
        Engine[] beside = [Gimballed(0, 0, live: true), Gimballed(1.4, 0, live: false)];
        choice = IdleEngineParking.Solve(beside);
        Check("an idle engine beside a live one tilts away from it", beside[1].Exits[choice[1]].X > beside[1].Rest.X + 0.95 * ReachM);

        Check("with nothing lit nothing moves", IdleEngineParking.Solve([.. ring.Select(p => Gimballed(p.X, p.Y, live: false))]).All(c => c == 0));
        Check("with everything lit nothing moves", IdleEngineParking.Solve([.. ring.Select(p => Gimballed(p.X, p.Y, live: true))]).All(c => c == 0));
        Check("the commands start at rest and stay in the range",
            IdleEngineParking.Commands(true)[0] == (0.0, 0.0)
            && IdleEngineParking.Commands(true).All(c => c.Y * c.Y + c.Z * c.Z <= 1.0 + 1e-9)
            && IdleEngineParking.Commands(false).All(c => Math.Abs(c.Y) <= 1.0 + 1e-9 && Math.Abs(c.Z) <= 1.0 + 1e-9)
            && IdleEngineParking.Commands(false).Any(c => Math.Abs(c.Y) > 0.99 && Math.Abs(c.Z) > 0.99));
    }
}
