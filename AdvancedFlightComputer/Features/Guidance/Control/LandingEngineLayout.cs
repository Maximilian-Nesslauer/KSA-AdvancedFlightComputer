using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>One engine controller as the landing engine picker draws it: its place across the thrust axis and its sea-level performance.</summary>
internal sealed record LandingEngineDot(string Key, EngineController Engine, float X, float Y, double ThrustMaxN, double ThrustMinN,
                                        double FlowMaxKgS, bool Gimballed, bool AftPlane);

internal enum LandingEngineVerdict { Good, NoHover, Weak, TooWeak, OffAxis }

/// <summary>What a set of engines gives the landing. TWR is thrust over weight at the landing site's gravity and air pressure.</summary>
internal readonly record struct LandingEngineStats(int Engines, double ThrustMaxN, double ThrustMinN, double VeMs,
                                                   double TwrMinDry, double TwrMaxDry, double TwrMaxLoaded, double OffsetM,
                                                   int Gimbals, LandingEngineVerdict Verdict);

internal sealed record LandingEngineOption(string Name, string[] Keys, LandingEngineStats Stats, bool Recommended);

/// <summary>
/// The engines of a stage laid out for the landing engine picker, with the numbers of any choice and a list of balanced choices.
/// The picture looks along the thrust axis from behind the stage. Engines more than <see cref="AftPlaneDepthM"/> forward of the
/// aft-most nozzles are not on the picture, because a stack's upper stage engine would sit on top of the center engine.
/// </summary>
internal sealed class LandingEngineLayout
{
    internal const double AftPlaneDepthM = 2.5;
    // A choice whose thrust center is further than this from the cluster's axis turns the stage: the picker marks it OFF AXIS and suggests none. The flight measures the moment from the centre of mass at entry guidance's engage and lands on all engines when it is over LandingMomentShare of the gimbals (LandingEngineSelection).
    internal const double BalancedOffsetM = 0.05;
    // The landing burn needs braking room over gravity at the start of the burn, and the flight refuses a choice under 1.0.
    internal const double WeakTwr = 1.5;

    internal LandingEngineDot[] Dots = [];
    internal float Extent = 1f;
    internal float DotRadius = 0.5f;
    internal double DryKg;
    internal double LoadedKg = double.NaN;
    internal double Gravity;

    internal static LandingEngineLayout Build(Vehicle vehicle, Dictionary<string, EngineController?> candidates,
                                              double pressurePa, double gravity, double dryKg, double loadedKg)
    {
        var layout = new LandingEngineLayout { DryKg = dryKg, LoadedKg = loadedKg, Gravity = gravity };
        var gimballed = new HashSet<RocketNozzle>();
        foreach (GimbalController gimbal in vehicle.Parts.Modules.Get<GimbalController>())
            foreach (RocketNozzle nozzle in gimbal.Data.Nozzles)
                gimballed.Add(nozzle);

        var raw = new List<(string Key, EngineController Engine, double3 Position, double3 Direction, bool Gimballed)>();
        foreach ((string key, EngineController? engine) in candidates)
        {
            if (engine == null || engine.Rockets.Length == 0)
                continue;
            double3 position = double3.Zero, direction = double3.Zero;
            int count = 0;
            bool onGimbal = false;
            foreach (Rocket rocket in engine.Rockets)
                foreach (RocketNozzle nozzle in rocket.Nozzles)
                {
                    if (nozzle.Parent is not { } part)
                        continue;
                    float3 at = nozzle.LocationAsmb.Transform(float4x4.Pack(part.MatrixAsmb2VehicleAsmb));
                    float3 thrust = (-nozzle.ExhaustDirectionAsmb).Transform(floatQuat.Pack(part.Asmb2VehicleAsmb));
                    position += new double3(at.X, at.Y, at.Z);
                    direction += new double3(thrust.X, thrust.Y, thrust.Z);
                    onGimbal |= gimballed.Contains(nozzle);
                    count++;
                }
            if (count > 0)
                raw.Add((key, engine, position / count, direction, onGimbal));
        }
        if (raw.Count == 0)
            return layout;

        double3 axis = double3.Zero;
        foreach (var r in raw)
            axis += r.Direction;
        axis = axis.LengthSquared() > 1e-12 ? axis.Normalized() : new double3(1, 0, 0);
        double3 helper = Math.Abs(axis.X) < 0.9 ? new double3(1, 0, 0) : new double3(0, 1, 0);
        double3 u = double3.Cross(axis, helper).Normalized();
        double3 v = double3.Cross(axis, u);

        double aft = raw.Min(r => double3.Dot(r.Position, axis));
        double3 center = double3.Zero;
        int onPlane = 0;
        foreach (var r in raw)
            if (double3.Dot(r.Position, axis) - aft <= AftPlaneDepthM)
            {
                center += r.Position;
                onPlane++;
            }
        center /= Math.Max(onPlane, 1);

        float pressure = (float)Math.Max(pressurePa, 0.0);
        layout.Dots = raw.Select(r =>
        {
            RocketControllerData full = RocketControllerData.ComputeFromRockets(r.Engine.Rockets, pressure, 1f);
            // A command of zero is held to each rocket's own minimum throttle inside ComputeFromRockets.
            RocketControllerData floor = RocketControllerData.ComputeFromRockets(r.Engine.Rockets, pressure, 0f);
            double3 rel = r.Position - center;
            return new LandingEngineDot(r.Key, r.Engine, (float)double3.Dot(rel, u), (float)double3.Dot(rel, v),
                full.ThrustMax.Length(), floor.ThrustMax.Length(), full.MassFlowRateMax, r.Gimballed,
                double3.Dot(r.Position, axis) - aft <= AftPlaneDepthM);
        }).ToArray();

        LandingEngineDot[] plane = layout.Dots.Where(d => d.AftPlane).ToArray();
        float nearest = float.PositiveInfinity;
        for (int i = 0; i < plane.Length; i++)
            for (int j = i + 1; j < plane.Length; j++)
                nearest = Math.Min(nearest, MathF.Sqrt((plane[i].X - plane[j].X) * (plane[i].X - plane[j].X)
                                                       + (plane[i].Y - plane[j].Y) * (plane[i].Y - plane[j].Y)));
        float reach = plane.Length == 0 ? 0f : plane.Max(d => MathF.Sqrt(d.X * d.X + d.Y * d.Y));
        layout.DotRadius = float.IsFinite(nearest) && nearest > 1e-3f ? 0.42f * nearest : Math.Max(0.35f * reach, 0.5f);
        layout.Extent = reach + layout.DotRadius;
        return layout;
    }

    internal LandingEngineStats Stats(IReadOnlyCollection<string> keys)
    {
        double thrust = 0, floor = 0, flow = 0, momentX = 0, momentY = 0;
        int engines = 0, gimbals = 0;
        foreach (LandingEngineDot dot in Dots)
        {
            if (!keys.Contains(dot.Key))
                continue;
            engines++;
            thrust += dot.ThrustMaxN;
            floor += dot.ThrustMinN;
            flow += dot.FlowMaxKgS;
            momentX += dot.ThrustMaxN * dot.X;
            momentY += dot.ThrustMaxN * dot.Y;
            if (dot.Gimballed)
                gimbals++;
        }
        double weightDry = DryKg * Gravity;
        double twrMinDry = weightDry > 0 ? floor / weightDry : double.NaN;
        double twrMaxDry = weightDry > 0 ? thrust / weightDry : double.NaN;
        double twrMaxLoaded = LoadedKg > 0 && Gravity > 0 ? thrust / (LoadedKg * Gravity) : twrMaxDry;
        double offset = thrust > 0 ? Math.Sqrt(momentX * momentX + momentY * momentY) / thrust : 0;
        LandingEngineVerdict verdict = engines == 0 || !(twrMaxLoaded >= 1.0) ? LandingEngineVerdict.TooWeak
            : offset > BalancedOffsetM ? LandingEngineVerdict.OffAxis
            : twrMaxLoaded < WeakTwr ? LandingEngineVerdict.Weak
            : twrMinDry >= 1.0 ? LandingEngineVerdict.NoHover
            : LandingEngineVerdict.Good;
        return new LandingEngineStats(engines, thrust, floor, flow > 0 ? thrust / flow : double.NaN,
            twrMinDry, twrMaxDry, twrMaxLoaded, offset, gimbals, verdict);
    }

    /// <summary>
    /// Balanced choices built from the symmetry of the aft plane: the center engine, each ring whole, one opposite pair, every third
    /// and every fourth engine of a ring, alone or with the center, and all engines. The fewest engines that can hover at touchdown and
    /// still brake well at the start of the burn is marked as recommended.
    /// </summary>
    internal LandingEngineOption[] Suggestions()
    {
        LandingEngineDot[] plane = Dots.Where(d => d.AftPlane).ToArray();
        if (plane.Length == 0)
            return [];
        float reach = plane.Max(d => MathF.Sqrt(d.X * d.X + d.Y * d.Y));
        float centerLimit = Math.Max(0.15f * reach, 0.05f);
        string[] center = plane.Where(d => Radius(d) < centerLimit).Select(d => d.Key).ToArray();
        var rings = new List<LandingEngineDot[]>();
        foreach (LandingEngineDot dot in plane.Where(d => Radius(d) >= centerLimit).OrderBy(Radius))
        {
            if (rings.Count > 0 && Radius(dot) - Radius(rings[^1][0]) <= Math.Max(0.1f * reach, 0.05f))
                rings[^1] = [.. rings[^1], dot];
            else
                rings.Add([dot]);
        }

        var parts = new List<(string Name, string[] Keys)>();
        for (int i = 0; i < rings.Count; i++)
        {
            LandingEngineDot[] ring = rings[i].OrderBy(d => MathF.Atan2(d.Y, d.X)).ToArray();
            int n = ring.Length;
            string prefix = rings.Count > 1 ? (i == 0 ? "INNER " : i == rings.Count - 1 ? "OUTER " : $"RING {i + 1} ") : "";
            if (n % 2 == 0 && n > 2)
                parts.Add((prefix + "PAIR", [ring[0].Key, ring[n / 2].Key]));
            if (n % 3 == 0 && n > 3)
                parts.Add((prefix + "TRIANGLE", [ring[0].Key, ring[n / 3].Key, ring[2 * n / 3].Key]));
            if (n % 4 == 0 && n > 4)
                parts.Add((prefix + "CROSS", [ring[0].Key, ring[n / 4].Key, ring[n / 2].Key, ring[3 * n / 4].Key]));
            parts.Add((prefix.Length > 0 ? prefix.TrimEnd() + $" {n}" : $"RING {n}", ring.Select(d => d.Key).ToArray()));
        }

        // All engines first, so that its name wins over a center and ring choice with the same engines.
        var choices = new List<(string Name, string[] Keys)> { ($"ALL {plane.Length}", plane.Select(d => d.Key).ToArray()) };
        if (center.Length > 0)
            choices.Add(("CENTER", center));
        foreach (var part in parts)
        {
            if (center.Length > 0)
                choices.Add(("CENTER + " + part.Name, [.. center, .. part.Keys]));
            choices.Add((part.Name, part.Keys));
        }

        var seen = new HashSet<string>(StringComparer.Ordinal);
        var options = new List<LandingEngineOption>();
        foreach (var choice in choices)
        {
            string id = string.Join("|", choice.Keys.Order(StringComparer.Ordinal));
            if (!seen.Add(id))
                continue;
            LandingEngineStats stats = Stats(choice.Keys);
            if (stats.Verdict == LandingEngineVerdict.OffAxis)
                continue;
            options.Add(new LandingEngineOption(choice.Name, choice.Keys.Order(StringComparer.Ordinal).ToArray(), stats, false));
        }
        options.Sort((a, b) => a.Stats.Engines != b.Stats.Engines ? a.Stats.Engines.CompareTo(b.Stats.Engines)
            : b.Stats.Gimbals.CompareTo(a.Stats.Gimbals));
        int best = options.FindIndex(o => o.Stats.Verdict == LandingEngineVerdict.Good && o.Stats.Gimbals > 0);
        if (best < 0)
            best = options.FindIndex(o => o.Stats.Verdict == LandingEngineVerdict.NoHover);
        if (best >= 0)
            options[best] = options[best] with { Recommended = true };
        return options.ToArray();
    }

    /// <summary>
    /// The suggestion these engines are, or null for an unbalanced choice made by hand. A choice counts as a suggestion when it has the same
    /// engines, or when it is balanced and has the same number of engines at each distance from the axis, for example another opposite pair.
    /// </summary>
    internal LandingEngineOption? Matching(LandingEngineOption[] options, IReadOnlyCollection<string> keys)
    {
        LandingEngineOption? exact = options.FirstOrDefault(o => o.Keys.Length == keys.Count && o.Keys.All(keys.Contains));
        if (exact != null || keys.Count == 0 || Stats(keys).OffsetM > BalancedOffsetM)
            return exact;
        string signature = Signature(keys);
        return options.FirstOrDefault(o => o.Keys.Length == keys.Count && Signature(o.Keys) == signature);
    }

    // The distances of the engines from the axis, in steps of a twentieth of the cluster's reach, so that rotated choices compare equal.
    private string Signature(IReadOnlyCollection<string> keys)
    {
        LandingEngineDot[] plane = Dots.Where(d => d.AftPlane).ToArray();
        float reach = Math.Max(plane.Length == 0 ? 0f : plane.Max(Radius), 0.01f);
        return string.Join(",", Dots.Where(d => keys.Contains(d.Key))
            .Select(d => d.AftPlane ? ((int)MathF.Round(20f * Radius(d) / reach)).ToString(System.Globalization.CultureInfo.InvariantCulture) : "up:" + d.Key)
            .Order(StringComparer.Ordinal));
    }

    private static float Radius(LandingEngineDot dot) => MathF.Sqrt(dot.X * dot.X + dot.Y * dot.Y);
}
