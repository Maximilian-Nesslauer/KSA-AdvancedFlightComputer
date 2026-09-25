using System.Text.Json;
using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Numerics.Flight;

namespace AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// An ascent problem as plain JSON, so a problem the game built can be solved again offline - the test console's --ascent-replay reads it back. Every number is SI. The drag coefficient is written as a curve against Mach at the attitude the dynamics read it at, which is all of the table they use.
/// </summary>
public static class AscentProblemFile
{
    private sealed class StageDto
    {
        public double Thrust { get; set; }
        public double MassFlow { get; set; }
        public double PropellantMass { get; set; }
        public double JettisonMass { get; set; }
        public double DragArea { get; set; }
        public double[]? PressureGrid { get; set; }
        public double[]? ThrustAtPressure { get; set; }
        public double? ThrottleMin { get; set; }
        public double? FixedBurnTime { get; set; }
        public bool LiquidCarriesOver { get; set; }
        public double? SeedThrottle { get; set; }
        // The solids, as AscentSolidBurn holds them: stage time, mass flow, and thrust row-major against the pressure grid.
        public double[]? SolidTime { get; set; }
        public double[]? SolidMassFlow { get; set; }
        public double[]? SolidPressureGrid { get; set; }
        public double[]? SolidThrust { get; set; }
    }

    private sealed class ProblemDto
    {
        public double Mu { get; set; }
        public double BodyRadius { get; set; }
        public double Omega { get; set; }
        public double[] R0 { get; set; } = [];
        public double[] V0 { get; set; } = [];
        public double M0 { get; set; }
        public StageDto[] Stages { get; set; } = [];
        public double[]? Atmosphere { get; set; }       // rho0, p0, scale height, gamma; null for none
        public double[]? CdMach { get; set; }
        public double[]? Cd { get; set; }
        public double GroundRadius { get; set; }
        public double TargetRadius { get; set; }
        public double TargetSpeed { get; set; }
        public double TargetRadialRate { get; set; }
        public double[] PlaneNormal { get; set; } = [];
        public double QMax { get; set; }
        public double QAlphaMax { get; set; }
    }

    private static readonly double[] MachSamples = AeroTable.DefaultMachBreakpoints;

    public static string ToJson(AscentProblem p)
    {
        var dto = new ProblemDto
        {
            Mu = p.Mu,
            BodyRadius = p.BodyRadius,
            Omega = p.Omega,
            R0 = p.R0,
            V0 = p.V0,
            M0 = p.M0,
            Stages = p.Stages.Select(s => new StageDto
            {
                Thrust = s.Thrust,
                MassFlow = s.MassFlow,
                PropellantMass = s.PropellantMass,
                JettisonMass = s.JettisonMass,
                DragArea = s.DragArea,
                PressureGrid = s.PressureGrid,
                ThrustAtPressure = s.ThrustAtPressure,
                ThrottleMin = double.IsNaN(s.ThrottleMin) ? null : s.ThrottleMin,
                FixedBurnTime = s.IsPinned ? s.FixedBurnTime : null,
                LiquidCarriesOver = s.LiquidCarriesOver,
                SeedThrottle = s.SeedThrottle < 1.0 ? s.SeedThrottle : null,
                SolidTime = s.Solid?.Time,
                SolidMassFlow = s.Solid?.MassFlow,
                SolidPressureGrid = s.Solid?.PressureGrid,
                SolidThrust = s.Solid?.Thrust,
            }).ToArray(),
            GroundRadius = p.GroundRadius,
            TargetRadius = p.TargetRadius,
            TargetSpeed = p.TargetSpeed,
            TargetRadialRate = p.TargetRadialRate,
            PlaneNormal = p.PlaneNormal,
            QMax = p.QMax,
            QAlphaMax = p.QAlphaMax,
        };
        if (p.Atmosphere is KsaAscentAtmosphere ksa)
        {
            ExponentialAtmosphere a = ksa.Atmosphere;
            dto.Atmosphere = [a.SeaLevelDensity, a.SeaLevelPressure, a.ScaleHeight, a.Gamma];
            dto.CdMach = MachSamples;
            dto.Cd = MachSamples.Select(m => ksa.DragCoefficient(new Dual(m)).V).ToArray();
        }
        return JsonSerializer.Serialize(dto, new JsonSerializerOptions { WriteIndented = true });
    }

    public static AscentProblem FromJson(string json)
    {
        ProblemDto dto = JsonSerializer.Deserialize<ProblemDto>(json)
            ?? throw new InvalidDataException("not an ascent problem");
        AscentAtmosphere air = VacuumAscentAtmosphere.Instance;
        if (dto.Atmosphere is { Length: 4 } a)
        {
            var atm = new ExponentialAtmosphere(a[0], a[1], a[2], a[3]);
            AeroTable? table = null;
            if (dto.CdMach is { Length: >= 2 } mach && dto.Cd is { } cd && cd.Length == mach.Length)
            {
                // The curve is all the dynamics read, so it is laid down flat across alpha.
                double[] alpha = [0.0, 180.0];
                var values = new double[mach.Length * 2];
                for (int i = 0; i < mach.Length; i++)
                    values[i * 2] = values[i * 2 + 1] = cd[i];
                table = new AeroTable(mach, alpha, values);
            }
            air = new KsaAscentAtmosphere(atm, table);
        }
        return new AscentProblem
        {
            Mu = dto.Mu,
            BodyRadius = dto.BodyRadius,
            Omega = dto.Omega,
            R0 = dto.R0,
            V0 = dto.V0,
            M0 = dto.M0,
            Stages = dto.Stages.Select(s => new AscentStage(s.Thrust, s.MassFlow, s.PropellantMass, s.JettisonMass,
                                                            s.DragArea, s.PressureGrid, s.ThrustAtPressure,
                                                            s.ThrottleMin ?? double.NaN,
                                                            s.SolidTime is { } st && s.SolidMassFlow is { } sm
                                                                && s.SolidPressureGrid is { } sg && s.SolidThrust is { } sf
                                                                ? new AscentSolidBurn(st, sm, sg, sf) : null,
                                                            s.FixedBurnTime ?? double.NaN, s.LiquidCarriesOver,
                                                            s.SeedThrottle ?? 1.0)).ToArray(),
            Atmosphere = air,
            GroundRadius = dto.GroundRadius,
            TargetRadius = dto.TargetRadius,
            TargetSpeed = dto.TargetSpeed,
            TargetRadialRate = dto.TargetRadialRate,
            PlaneNormal = dto.PlaneNormal,
            QMax = dto.QMax,
            QAlphaMax = dto.QAlphaMax,
        };
    }
}
