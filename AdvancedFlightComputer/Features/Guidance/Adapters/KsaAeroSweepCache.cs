#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using System.Globalization;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.AeroControl;
using AdvancedFlightComputer.Guidance.Numerics.Flight;

/// <summary>
/// THE SWEEP CACHE: a craft is swept once per geometry, this game session, and every mode takes that sweep.
///
/// WHY. Each mode kept its own sweep on the vehicle's state (the aim's, the entry's, DIAG's, touchdown's), and a reload
/// started from nothing: 2026-11-25, three whole-craft sweeps of the Starship in a minute, each with 40 s of flaps.
///
/// THE KEY is the craft's aero mesh hash (AeroMeshSource.Hash, which KSAero takes with every surface at rest, so a
/// flap's angle never changes it), the body flown over, and for a booster its name. Not the bounding box: a part's
/// bounds follow its subparts' poses, so a flap out can move the box when the mesh has not changed.
///
/// A HIT (<see cref="TryCached"/>) is a result that holds every table asked for. While its sweep still runs, the same
/// craft is handed the same job. Once it has ended, the same craft with its centre of mass where the sweep saw it is
/// handed the same result; any other is handed a view of it on its own thread (<see cref="ViewOf"/>): its surface
/// tables re-linked to the craft's parts as they are now (a reload makes new Part objects, and tables are matched to
/// surfaces by part), and every moment moved to the centre of mass the craft has now.
///
/// A MISS sweeps what was asked for plus what was swept before on the same geometry (<see cref="CacheUnion"/>), so
/// the newest result serves every mode that has asked. Kept in memory only, the last <see cref="CacheKept"/> sweeps.
/// </summary>
public static partial class KsaAeroSweep
{
    private sealed class CacheEntry
    {
        public ulong Hash;
        public string Body = "";
        public string Booster = "";
        public bool Nose, Trim;
        public int Side = -1;
        public Job Job;
        public WeakReference<Vehicle> Owner;
        public string Craft = "";
    }

    /// <summary>Sweeps the cache keeps, newest first: a few craft and their table sets.</summary>
    private const int CacheKept = 8;

    /// <summary>A centre of mass this far from the one a sweep saw, m, has its moments moved; closer, the sweep's are taken as they are. Own choice: 5 cm on a 50 m craft is under 0.1 % of its length.</summary>
    public const double CacheComToleranceM = 0.05;

    private static readonly List<CacheEntry> Cache = new();

    private static string BodyKey(IParentBody parent) => (parent as Astronomical)?.Id ?? "";

    /// <summary>A part as a sweep found it: its template and its place on the craft, to a tenth of a metre. A reloaded craft's parts have new objects but the same keys.</summary>
    internal static string PartKey(Part part)
    {
        if (part == null)
            return "";
        double3 at = part.PositionVehicleAsmb;
        return string.Create(CultureInfo.InvariantCulture,
            $"{part.Template?.Id}@{Math.Round(at.X * 10.0):0},{Math.Round(at.Y * 10.0):0},{Math.Round(at.Z * 10.0):0}");
    }

    private static bool Covers(CacheEntry e, bool nose, bool trim, int side)
        => (!nose || e.Nose) && (!trim || e.Trim) && (side < 0 || e.Side == side);

    private static string CacheTablesText(CacheEntry e)
    {
        if (e.Booster.Length > 0)
            return e.Booster;
        string text = "TAIL";
        if (e.Nose)
            text += " + NOSE";
        if (e.Trim)
            text += " + BELLY TRIM";
        if (e.Side >= 0 && e.Side < BellyAxes.Length)
        {
            double3 a = BellyAxes[e.Side];
            text += " + BELLY " + (a.Y + a.Z > 0 ? "+" : "-") + (a.Y != 0 ? "Y" : "Z");
        }
        return text;
    }

    /// <summary>Drops sweeps that failed. Main thread.</summary>
    private static void CachePrune()
    {
        Cache.RemoveAll(e => e.Job.Finished && e.Job.Peek == null);
        while (Cache.Count > CacheKept)
            Cache.RemoveAt(0);
    }

    /// <summary>The newest sweep of this geometry that holds every table asked for, if there is one: <paramref name="job"/> hands it over (see the class summary). Main thread.</summary>
    private static bool TryCached(Vehicle vehicle, IParentBody parent, ulong hash, string booster, bool nose, bool trim, int side, double3 comNow, out Job job)
    {
        job = null;
        if (hash == 0)
            return false;
        CachePrune();
        string body = BodyKey(parent);
        CacheEntry hit = null;
        for (int i = Cache.Count - 1; i >= 0 && hit == null; i--)
        {
            CacheEntry e = Cache[i];
            if (e.Hash == hash && e.Body == body && e.Booster == booster && Covers(e, nose, trim, side))
                hit = e;
        }
        if (hit == null)
            return TryStored(vehicle, parent, hash, booster, side, comNow, out job);

        bool sameCraft = hit.Owner.TryGetTarget(out Vehicle owner) && ReferenceEquals(owner, vehicle);
        if (!hit.Job.Finished)
        {
            // Still sweeping: the same craft waits on the same sweep. A reloaded one waits for all of it, surfaces too, and takes a view.
            if (sameCraft)
            {
                job = hit.Job;
                return true;
            }
        }
        else if (sameCraft && hit.Job.Peek is { } done && ReferenceEquals(done.Parent, parent)
                 && Distance(done.BodyMomentComAsmb, comNow) <= CacheComToleranceM)
        {
            job = Job.Completed(done);
            int surfaces = done.Surfaces?.Length ?? 0;
            SweepProgress row = TrackCached(vehicle?.Id ?? "", CacheTablesText(hit) + " (CACHED)", surfaces);
            row.Handed(surfaces, surfaces);
            row.Enter(SweepProgress.Stage.Done);
            return true;
        }

        // A view: re-linked when the craft is not the one swept, moved when its centre of mass is not where the sweep saw it.
        Dictionary<string, (Part Part, string Name)> parts = sameCraft || booster.Length > 0 ? null : SurfacePartsByKey(vehicle);
        Job source = hit.Job;
        SweepProgress progress = TrackCached(vehicle?.Id ?? "", CacheTablesText(hit) + (sameCraft ? " (CACHED, NEW CoM)" : " (CACHED, RELOADED CRAFT)"), 0);
        progress.Enter(SweepProgress.Stage.Body);
        job = Job.Derived(() =>
        {
            source.WaitFinished();
            Result src = source.Peek;
            if (src == null)
                return (null, "the cached sweep failed");
            Result view = ViewOf(src, parts, comNow, parent);
            int swept = src.Surfaces?.Length ?? 0, kept = view.Surfaces?.Length ?? 0;
            progress.Handed(swept, kept);
            if (kept < swept)
                progress.Note($"{swept - kept} of {swept} surface tables found no part on the craft now");
            return (view, "");
        }, progress);
        // The view is this craft's from now on: the next ask, with the centre of mass where it is, takes it as it is.
        Cache.Add(new CacheEntry
        {
            Hash = hit.Hash, Body = hit.Body, Booster = hit.Booster, Nose = hit.Nose, Trim = hit.Trim, Side = hit.Side,
            Job = job, Owner = new WeakReference<Vehicle>(vehicle), Craft = vehicle?.Id ?? "",
        });
        CachePrune();
        return true;
    }

    /// <summary>
    /// A saved sweep of this geometry (KsaAeroSweepStore), when the cache has none: read on its own thread, re-linked to the craft's parts and moved to its centre of mass, and kept in the cache from then on. A whole-craft file holds every table, so only its belly side must match, and the file name says it. A file that cannot be read is deleted, so the next ask sweeps. Main thread.
    /// </summary>
    private static bool TryStored(Vehicle vehicle, IParentBody parent, ulong hash, string booster, int side, double3 comNow, out Job job)
    {
        job = null;
        string path = StorePathFor(vehicle, parent, hash, booster, booster.Length > 0 ? -1 : side);
        if (path == null || !System.IO.File.Exists(path))
            return false;
        Dictionary<string, (Part Part, string Name)> parts = booster.Length > 0 ? null : SurfacePartsByKey(vehicle);
        var entry = new CacheEntry
        {
            Hash = hash, Body = BodyKey(parent), Booster = booster,
            Nose = booster.Length == 0, Trim = booster.Length == 0, Side = booster.Length > 0 ? -1 : side,
            Owner = new WeakReference<Vehicle>(vehicle), Craft = vehicle?.Id ?? "",
        };
        SweepProgress progress = TrackCached(vehicle?.Id ?? "", CacheTablesText(entry) + " (SAVED)", 0);
        progress.Enter(SweepProgress.Stage.Body);
        job = Job.Derived(() =>
        {
            Result src = LoadSweep(path, parent, out string why);
            if (src == null)
            {
                try { System.IO.File.Delete(path); } catch (Exception) { }
                return (null, "saved aero tables: " + why);
            }
            Result view = ViewOf(src, parts, comNow, parent);
            int swept = src.Surfaces?.Length ?? 0, kept = view.Surfaces?.Length ?? 0;
            progress.Handed(swept, kept);
            if (kept < swept)
                progress.Note($"{swept - kept} of {swept} surface tables found no part on the craft now");
            return (view, "");
        }, progress);
        entry.Job = job;
        Cache.Add(entry);
        CachePrune();
        return true;
    }

    /// <summary>A new sweep of a geometry the cache has: what the newest one swept is swept again with what is asked for, so the new result covers both. A different belly side replaces the old one. Main thread.</summary>
    private static void CacheUnion(ulong hash, IParentBody parent, string booster, ref bool nose, ref bool trim, ref int side)
    {
        if (hash == 0)
            return;
        string body = BodyKey(parent);
        for (int i = Cache.Count - 1; i >= 0; i--)
        {
            CacheEntry e = Cache[i];
            if (e.Hash != hash || e.Body != body || e.Booster != booster || (e.Job.Finished && e.Job.Peek == null))
                continue;
            nose |= e.Nose;
            trim |= e.Trim;
            if (side < 0)
                side = e.Side;
            return;
        }
    }

    /// <summary>A sweep started: kept for the next ask. Main thread.</summary>
    private static void CacheAdd(Vehicle vehicle, IParentBody parent, ulong hash, string booster, bool nose, bool trim, int side, Job job)
    {
        if (hash == 0 || job == null)
            return;
        Cache.Add(new CacheEntry
        {
            Hash = hash, Body = BodyKey(parent), Booster = booster, Nose = nose, Trim = trim, Side = side,
            Job = job, Owner = new WeakReference<Vehicle>(vehicle), Craft = vehicle?.Id ?? "",
        });
        CachePrune();
    }

    /// <summary>The craft's control surfaces by PartKey, with their names now. Main thread.</summary>
    private static Dictionary<string, (Part Part, string Name)> SurfacePartsByKey(Vehicle vehicle)
    {
        var map = new Dictionary<string, (Part, string)>();
        if (vehicle == null || !ControlSurfaces.Any)
            return map;
        var surfaces = new List<ControlSurface>();
        ControlSurfaces.Collect(vehicle, surfaces);
        foreach (ControlSurface s in surfaces)
            map[PartKey(s.Part)] = (s.Part, s.Name);
        return map;
    }

    private static double Distance(double3 a, double3 b) => (a - b).Length();

    /// <summary>
    /// A sweep as another craft object or another centre of mass sees it: a shallow copy sharing every table it can.
    /// Surfaces are re-linked by <see cref="SurfaceTable.PartKey"/> where <paramref name="parts"/> is given (one with no
    /// part now is dropped). Moments are moved from the sweep's centre of mass c0 to <paramref name="comNow"/> c1:
    /// M1 = M0 + (c0 - c1) x F, which for coefficients (moment per q S L, force per q S) is C_M1 = C_M0 + (c0 - c1) x C_F / L,
    /// node by node, for every body-axis force and moment pair: the body, nose, belly, sphere and band tables and each
    /// surface's per-deflection changes. Pure: any thread.
    /// </summary>
    internal static Result ViewOf(Result src, Dictionary<string, (Part Part, string Name)> parts, double3 comNow, IParentBody parent)
    {
        Result v = src.ShallowCopy();
        if (parent != null)
            v.Parent = parent;
        double3 d = src.BodyMomentComAsmb - comNow;
        bool move = d.Length() > 1e-6;
        double length = src.ReferenceLength;
        if (move)
        {
            v.BodyMoment = MovedMoment(src.BodyAero, src.BodyMoment, d, length);
            v.NoseBodyMoment = MovedMoment(src.NoseBodyAero, src.NoseBodyMoment, d, length);
            v.BellyBodyMoment = MovedMoment(src.BellyBodyAero, src.BellyBodyMoment, d, length);
            v.SphereBodyMoment = MovedMoment(src.SphereBodyAero, src.SphereBodyMoment, d, length);
            v.BandBodyMoment = MovedMoment(src.BandBodyAero, src.BandBodyMoment, d, length);
            v.BodyMomentComAsmb = comNow;
            MoveRollAveraged(src, v, -d.X);
            if (src.Belly != null)
            {
                v.Belly = new BellySweep[src.Belly.Length];
                for (int k = 0; k < src.Belly.Length; k++)
                    v.Belly[k] = MovedTrimRows(src.Belly[k], d);
            }
        }

        SurfaceTable[] tables = System.Threading.Volatile.Read(ref src.Surfaces) ?? [];
        var kept = new List<SurfaceTable>(tables.Length);
        foreach (SurfaceTable t in tables)
        {
            SurfaceTable c = t;
            if (parts != null)
            {
                if (!parts.TryGetValue(t.PartKey ?? "", out (Part Part, string Name) now))
                    continue;
                c = t.ShallowCopy();
                c.Part = now.Part;
                c.Name = now.Name;
            }
            if (move)
            {
                if (ReferenceEquals(c, t))
                    c = t.ShallowCopy();
                c.Moment = MovedMoments(t.Force, t.Moment, d, length);
                c.NoseMoment = MovedMoments(t.NoseForce, t.NoseMoment, d, length);
                c.BellyMoment = MovedMoments(t.BellyForce, t.BellyMoment, d, length);
                c.SphereMoment = MovedMoments(t.SphereForce, t.SphereMoment, d, length);
                c.BandMoment = MovedMoments(t.BandForce, t.BandMoment, d, length);
            }
            kept.Add(c);
        }
        v.Surfaces = kept.ToArray();
        return v;
    }

    /// <summary>A moment table moved by d = c0 - c1 (see ViewOf, AeroMomentMove), with its force table beside it on the same grid; the table as it was where either is missing.</summary>
    internal static BodyAeroTable MovedMoment(BodyAeroTable force, BodyAeroTable moment, double3 d, double length)
        => AeroMomentMove.Moved(force, moment, d.X, d.Y, d.Z, length);

    private static BodyAeroTable[] MovedMoments(BodyAeroTable[] force, BodyAeroTable[] moment, double3 d, double length)
    {
        if (moment == null || moment.Length == 0 || force == null || force.Length != moment.Length)
            return moment;
        var moved = new BodyAeroTable[moment.Length];
        for (int k = 0; k < moment.Length; k++)
            moved[k] = MovedMoment(force[k], moment[k], d, length);
        return moved;
    }

    /// <summary>
    /// The roll-averaged pitch moment and its centres of pressure, for the centre of mass <paramref name="towardsNoseM"/>
    /// further towards the nose (+x): a centre of pressure measured from the centre of mass is that much less, and the
    /// moment coefficient changes by -s times the arm table over the reference length (ArmTable). Only the move along the
    /// long axis: these tables are averaged over roll.
    /// </summary>
    private static void MoveRollAveraged(Result src, Result v, double towardsNoseM)
    {
        if (src.CentreOfPressureM != null)
        {
            v.CentreOfPressureM = new double[src.CentreOfPressureM.Length];
            for (int i = 0; i < v.CentreOfPressureM.Length; i++)
                v.CentreOfPressureM[i] = src.CentreOfPressureM[i] - towardsNoseM;
            v.BalanceWorstM = src.BalanceWorstM - towardsNoseM;
        }
        if (src.MomentTable != null && src.ArmTable != null && src.ArmTable.Length == src.MomentTable.Length && src.ReferenceLength > 0.0)
        {
            var cm = new double[src.MomentTable.Length];
            for (int i = 0; i < cm.Length; i++)
                cm[i] = src.MomentTable[i] - towardsNoseM * src.ArmTable[i] / src.ReferenceLength;
            v.MomentTable = cm;
            v.Moment = new AeroTable(src.MachGrid, src.AlphaGridDeg, cm);
        }
    }

    /// <summary>A belly-first trim row set (BellySweep) moved by d = c0 - c1 (AeroMomentMove.MovedTrimCm); the rows as they were where they cannot be.</summary>
    internal static BellySweep MovedTrimRows(BellySweep rows, double3 d)
    {
        double[] cm = rows == null ? null : AeroMomentMove.MovedTrimCm(rows.Cd, rows.Cl, rows.Cm, rows.AlphaDeg,
            rows.Axis.X, rows.Axis.Y, rows.Axis.Z, d.X, d.Y, d.Z, rows.ReferenceLength);
        if (cm == null)
            return rows;
        return new BellySweep
        {
            Axis = rows.Axis, MachGrid = rows.MachGrid, AlphaDeg = rows.AlphaDeg, Cd = rows.Cd, Cl = rows.Cl, Cm = cm,
            ReferenceArea = rows.ReferenceArea, ReferenceLength = rows.ReferenceLength,
        };
    }
}
