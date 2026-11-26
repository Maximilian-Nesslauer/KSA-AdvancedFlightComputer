#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Guidance.Numerics.Flight;

/// <summary>
/// THE SWEEP STORE: each craft's sweep saved to disk when it ends, and loaded instead of sweeping again, in this game
/// session or a later one (2026-11-25, the user: "such a faff to have to wait for it to reload every time" - two
/// minutes for a Starship's body and four flaps).
///
/// WHERE: under KSA.Constants.DocumentsFolderPath\afc-aero-tables (per-instance under StarMap/Borea when
/// -InstancePath or STARMAP_INSTANCE_PATH is active, otherwise Documents\My Games\Kitten Space Agency\afc-aero-tables),
/// one gzipped JSON file per craft, geometry, surface provider/limits, body and belly side. THE NAME is the key
/// (<see cref="StorePathFor"/>): the craft, the aero mesh hash (KSAero takes it with every surface at rest), a
/// fingerprint of the live aero mesh's panelling (<see cref="MeshFingerprint"/>), a fingerprint of the active control
/// surface providers and deflection limits (<see cref="SurfaceFingerprint"/>), the body and the belly side. A
/// whole-craft sweep holds every table (TryStart), so the name says all a request needs.
///
/// WHAT: every table and figure of <see cref="Result"/> but the atmosphere, which is sampled again from the body
/// (BuildAtmosphere, as the sweep does), and the surfaces' parts, which are found again by PartKey on the craft as it
/// is (KsaAeroSweepCache.ViewOf, with the moments moved to its centre of mass). <see cref="StoreVersion"/> changes
/// when the sweep's tables change, and a file of another version is swept again.
/// </summary>
public static partial class KsaAeroSweep
{
    /// <summary>The stored format and sweep version: a file of any other is not read.</summary>
    private const int StoreVersion = 1;

    private static string StoreDirectory => Path.Combine(KSA.Constants.DocumentsFolderPath, "afc-aero-tables");

    private static readonly JsonSerializerOptions StoreJson = new()
    {
        IncludeFields = true,
        NumberHandling = JsonNumberHandling.AllowNamedFloatingPointLiterals,
        DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
    };

    /// <summary>The live aero mesh's panelling - panel width to 0.1 mm and the panel counts - folded into one number, or 0 with no mesh. KSAero turns surface panels in place, so a flap's angle does not change it. Main thread.</summary>
    internal static ulong MeshFingerprint(Vehicle vehicle)
    {
        var mesh = vehicle?.Parts?.Aerodynamics?.Mesh;
        if (mesh == null)
            return 0UL;
        ulong h = 1469598103934665603UL;
        void Mix(long v) { h ^= (ulong)v; h *= 1099511628211UL; }
        Mix((long)Math.Round(mesh.PanelWidth * 1e4));
        Mix(mesh.SolidPanels.Length);
        Mix(mesh.SheetPanels.Length);
        return h;
    }

    /// <summary>Fingerprint of the active control surface providers and each surface's deflection range and rate. Changes when a provider is added/removed or a flap's limits change. Main thread.</summary>
    internal static ulong SurfaceFingerprint(Vehicle vehicle, string booster)
    {
        ulong h = 1469598103934665603UL;
        void Mix(ulong v) { h ^= v; h *= 1099511628211UL; }
        void MixStr(string s)
        {
            if (string.IsNullOrEmpty(s)) { Mix(0); return; }
            foreach (char c in s) Mix(c);
            Mix(0xFFUL);
        }
        if (vehicle == null || booster.Length > 0 || !AdvancedFlightComputer.Features.AeroControl.ControlSurfaces.Any)
        {
            Mix(0xA5A5UL);
            return h;
        }
        var surfaces = new System.Collections.Generic.List<AdvancedFlightComputer.Features.AeroControl.ControlSurface>();
        AdvancedFlightComputer.Features.AeroControl.ControlSurfaces.Collect(vehicle, surfaces);
        Mix((ulong)surfaces.Count);
        foreach (var s in surfaces)
        {
            MixStr(s.Provider?.Name);
            MixStr(PartKey(s.Part));
            Mix((ulong)Math.Round(s.MinDeg * 100.0));
            Mix((ulong)Math.Round(s.MaxDeg * 100.0));
            Mix((ulong)Math.Round(s.RateDegS * 100.0));
        }
        return h;
    }

    private static string SanitizeFilePart(string raw, string fallback)
    {
        if (string.IsNullOrEmpty(raw))
            return fallback;
        var safe = new StringBuilder(raw.Length);
        foreach (char c in raw)
            safe.Append(char.IsLetterOrDigit(c) || c == '-' ? c : '_');
        return safe.Length > 0 ? safe.ToString() : fallback;
    }

    /// <summary>The file a craft's sweep is kept in, or null where it cannot be (no mesh, no hash). Main thread.</summary>
    private static string StorePathFor(Vehicle vehicle, IParentBody parent, ulong hash, string booster, int side)
    {
        ulong fingerprint = MeshFingerprint(vehicle);
        if (hash == 0 || fingerprint == 0)
            return null;
        ulong surfaceHash = SurfaceFingerprint(vehicle, booster);
        string craft = SanitizeFilePart(booster.Length > 0 ? booster : vehicle?.Id, "craft");
        string body = SanitizeFilePart(BodyKey(parent), "body");
        return Path.Combine(StoreDirectory, $"{craft}_{hash:x16}_{fingerprint:x16}_{surfaceHash:x16}_{body}_s{side}.aero.gz");
    }

    // ---- the stored form ----

    private sealed class StoredTable
    {
        public double[] M, A, B, V, F;
        public bool S;
    }

    private sealed class StoredBelly
    {
        public double[] Axis, Mach, Alpha, Cd, Cl, Cm;
        public double Area, Length;
    }

    private sealed class StoredSurface
    {
        public string Name, Key;
        public double Min, Max, Millis;
        public double[] Deg;
        public StoredTable[] F, Mo, NF, NM, BF, BM, SF, SM, DF, DM;
    }

    private sealed class Stored
    {
        public int Version;
        public string Craft, BodyName;
        public ulong Hash;
        public double[] MachGrid, AlphaGridDeg, CdTable, LiftTable, MomentTable, ArmTable, CentreOfPressureM;
        public double ReferenceLength, ReferenceArea, BalanceWorstM, BalanceWorstMach, Mass, CdTailFirst, CdBroadside, CdNoseFirst,
            PointingSensitivity, AttitudeSensitivity, RollSpread, MachSpread, LiftToDragMax, AtmosphereMirrorError, SampledAt, SweepMillis;
        public double[] Com, Box, SampledExtents;
        public StoredTable Body, BodyM, Nose, NoseM, Belly, BellyM, Sphere, SphereM, Band, BandM;
        public int BellySide;
        public StoredBelly[] BellyRows;
        public StoredSurface[] Surfaces;
    }

    private static double[] V3(double3 v) => [v.X, v.Y, v.Z];
    private static double3 D3(double[] a) => a is { Length: 3 } ? new double3(a[0], a[1], a[2]) : default;

    private static StoredTable Store(BodyAeroTable t) => t == null ? null : new StoredTable
    {
        M = t.MachGrid, A = t.AlphaGridDeg, B = t.BetaGridDeg, V = t.Values, S = t.Sphere,
        F = [t.Frame.LX, t.Frame.LY, t.Frame.LZ, t.Frame.AX, t.Frame.AY, t.Frame.AZ, t.Frame.BX, t.Frame.BY, t.Frame.BZ],
    };

    private static BodyAeroTable Table(StoredTable s)
    {
        if (s == null)
            return null;
        if (s.V == null || s.M == null || s.A == null || s.B == null || s.F is not { Length: 9 })
            throw new InvalidDataException("Stored aero table is truncated.");
        return new BodyAeroTable(s.M, s.A, s.B, s.V, new BodyAeroFrame(s.F[0], s.F[1], s.F[2], s.F[3], s.F[4], s.F[5], s.F[6], s.F[7], s.F[8]), s.S);
    }

    private static StoredTable[] Store(BodyAeroTable[] t) => t?.Select(Store).ToArray();
    private static BodyAeroTable[] Tables(StoredTable[] s)
    {
        if (s == null)
            return [];
        var result = new BodyAeroTable[s.Length];
        for (int i = 0; i < s.Length; i++)
            result[i] = Table(s[i]) ?? throw new InvalidDataException("Stored surface table entry is null.");
        return result;
    }

    /// <summary>A finished sweep written to its file, replacing any before. Any thread; a failure is logged and the sweep stands.</summary>
    internal static void SaveSweep(string path, Result r, string craft, System.Action<string> log)
    {
        if (path == null || r == null)
            return;
        try
        {
            var s = new Stored
            {
                Version = StoreVersion, Craft = craft, BodyName = r.BodyName, Hash = r.MeshHash,
                MachGrid = r.MachGrid, AlphaGridDeg = r.AlphaGridDeg, CdTable = r.CdTable, LiftTable = r.LiftTable,
                MomentTable = r.MomentTable, ArmTable = r.ArmTable, CentreOfPressureM = r.CentreOfPressureM,
                ReferenceLength = r.ReferenceLength, ReferenceArea = r.ReferenceArea, BalanceWorstM = r.BalanceWorstM, BalanceWorstMach = r.BalanceWorstMach,
                Mass = r.Mass, CdTailFirst = r.CdTailFirst, CdBroadside = r.CdBroadside, CdNoseFirst = r.CdNoseFirst,
                PointingSensitivity = r.PointingSensitivity, AttitudeSensitivity = r.AttitudeSensitivity, RollSpread = r.RollSpread,
                MachSpread = r.MachSpread, LiftToDragMax = r.LiftToDragMax, AtmosphereMirrorError = r.AtmosphereMirrorError,
                SampledAt = r.SampledAt, SweepMillis = r.SweepMillis,
                Com = V3(r.BodyMomentComAsmb), Box = V3(r.BoxExtents), SampledExtents = V3(r.SampledExtents),
            };
            s.Body = Store(r.BodyAero); s.BodyM = Store(r.BodyMoment);
            s.Nose = Store(r.NoseBodyAero); s.NoseM = Store(r.NoseBodyMoment);
            s.Belly = Store(r.BellyBodyAero); s.BellyM = Store(r.BellyBodyMoment);
            s.Sphere = Store(r.SphereBodyAero); s.SphereM = Store(r.SphereBodyMoment);
            s.Band = Store(r.BandBodyAero); s.BandM = Store(r.BandBodyMoment);
            s.BellySide = r.BellySide;
            s.BellyRows = r.Belly?.Select(b => new StoredBelly
            {
                Axis = V3(b.Axis), Mach = b.MachGrid, Alpha = b.AlphaDeg, Cd = b.Cd, Cl = b.Cl, Cm = b.Cm, Area = b.ReferenceArea, Length = b.ReferenceLength,
            }).ToArray();
            SurfaceTable[] surfaces = System.Threading.Volatile.Read(ref r.Surfaces) ?? [];
            s.Surfaces = surfaces.Select(t => new StoredSurface
            {
                Name = t.Name, Key = t.PartKey, Min = t.MinDeg, Max = t.MaxDeg, Millis = t.SweepMillis, Deg = t.DeflectionDeg,
                F = Store(t.Force), Mo = Store(t.Moment), NF = Store(t.NoseForce), NM = Store(t.NoseMoment),
                BF = Store(t.BellyForce), BM = Store(t.BellyMoment), SF = Store(t.SphereForce), SM = Store(t.SphereMoment),
                DF = Store(t.BandForce), DM = Store(t.BandMoment),
            }).ToArray();

            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            string temp = $"{path}.{Guid.NewGuid():N}.tmp";
            try
            {
                using (var file = File.Create(temp))
                using (var zip = new GZipStream(file, CompressionLevel.Optimal))
                    JsonSerializer.Serialize(zip, s, StoreJson);
                File.Move(temp, path, overwrite: true);
            }
            catch
            {
                try { if (File.Exists(temp)) File.Delete(temp); } catch (Exception) { }
                throw;
            }
            log?.Invoke($"[AFC] Aero tables of '{craft}' saved: {Path.GetFileName(path)}, {surfaces.Length} control surfaces, {new FileInfo(path).Length / 1024} KB.");
        }
        catch (Exception ex)
        {
            log?.Invoke($"[AFC] Aero tables of '{craft}' could not be saved to {path} ({ex.GetType().Name}: {ex.Message}); they are swept again next time.");
        }
    }

    /// <summary>A saved sweep read back, its atmosphere sampled again from <paramref name="parent"/>, its surfaces without parts (ViewOf re-links them). Null, with the reason, where the file cannot be read or is of another version. Any thread.</summary>
    internal static Result LoadSweep(string path, IParentBody parent, out string why)
    {
        why = "";
        Stored s;
        try
        {
            using var file = File.OpenRead(path);
            using var zip = new GZipStream(file, CompressionMode.Decompress);
            s = JsonSerializer.Deserialize<Stored>(zip, StoreJson);
        }
        catch (Exception ex)
        {
            why = $"could not read {Path.GetFileName(path)} ({ex.GetType().Name}: {ex.Message})";
            return null;
        }
        if (s == null || s.Version != StoreVersion)
        {
            why = $"{Path.GetFileName(path)} is of another version ({s?.Version})";
            return null;
        }
        Result r;
        try
        {
            r = new Result
            {
                MachGrid = s.MachGrid, AlphaGridDeg = s.AlphaGridDeg, CdTable = s.CdTable, LiftTable = s.LiftTable,
                MomentTable = s.MomentTable, ArmTable = s.ArmTable, CentreOfPressureM = s.CentreOfPressureM,
                ReferenceLength = s.ReferenceLength, ReferenceArea = s.ReferenceArea, BalanceWorstM = s.BalanceWorstM, BalanceWorstMach = s.BalanceWorstMach,
                Mass = s.Mass, CdTailFirst = s.CdTailFirst, CdBroadside = s.CdBroadside, CdNoseFirst = s.CdNoseFirst,
                PointingSensitivity = s.PointingSensitivity, AttitudeSensitivity = s.AttitudeSensitivity, RollSpread = s.RollSpread,
                MachSpread = s.MachSpread, LiftToDragMax = s.LiftToDragMax, SampledAt = s.SampledAt, SweepMillis = s.SweepMillis,
                BodyMomentComAsmb = D3(s.Com), BoxExtents = D3(s.Box), SampledExtents = D3(s.SampledExtents),
                BodyAero = Table(s.Body), BodyMoment = Table(s.BodyM), NoseBodyAero = Table(s.Nose), NoseBodyMoment = Table(s.NoseM),
                BellyBodyAero = Table(s.Belly), BellyBodyMoment = Table(s.BellyM), SphereBodyAero = Table(s.Sphere), SphereBodyMoment = Table(s.SphereM),
                BandBodyAero = Table(s.Band), BandBodyMoment = Table(s.BandM), BellySide = s.BellySide,
                MeshHash = s.Hash, Parent = parent, BodyName = s.BodyName ?? "",
            };
            if (s.MachGrid != null && s.AlphaGridDeg != null)
            {
                if (s.CdTable != null) r.Table = new AeroTable(s.MachGrid, s.AlphaGridDeg, s.CdTable);
                if (s.LiftTable != null) r.Lift = new AeroTable(s.MachGrid, s.AlphaGridDeg, s.LiftTable);
                if (s.MomentTable != null && s.ReferenceLength > 0.0) r.Moment = new AeroTable(s.MachGrid, s.AlphaGridDeg, s.MomentTable);
            }
            r.Belly = s.BellyRows?.Select(b => new BellySweep
            {
                Axis = D3(b.Axis), MachGrid = b.Mach, AlphaDeg = b.Alpha, Cd = b.Cd, Cl = b.Cl, Cm = b.Cm, ReferenceArea = b.Area, ReferenceLength = b.Length,
            }).ToArray();
            r.Surfaces = (s.Surfaces ?? []).Select(t => new SurfaceTable
            {
                Name = t.Name ?? "", PartKey = t.Key ?? "", MinDeg = t.Min, MaxDeg = t.Max, SweepMillis = t.Millis, DeflectionDeg = t.Deg ?? [],
                Force = Tables(t.F), Moment = Tables(t.Mo), NoseForce = Tables(t.NF), NoseMoment = Tables(t.NM),
                BellyForce = Tables(t.BF), BellyMoment = Tables(t.BM), SphereForce = Tables(t.SF), SphereMoment = Tables(t.SM),
                BandForce = Tables(t.DF), BandMoment = Tables(t.DM),
            }).ToArray();
            if (parent != null)
                BuildAtmosphere(parent, r);
        }
        catch (Exception ex)
        {
            why = $"invalid table data in {Path.GetFileName(path)} ({ex.GetType().Name}: {ex.Message})";
            return null;
        }
        return r;
    }
}
