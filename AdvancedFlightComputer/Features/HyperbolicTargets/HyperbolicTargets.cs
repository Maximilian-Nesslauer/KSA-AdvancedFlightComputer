using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.HyperbolicTargets;

/// <summary>
/// Lets the Transfer Planner target bodies on unbound orbits, which are the
/// interstellar comets. The Lambert solver handles any geometry, but the code
/// around it assumes a finite Period, a positive semi major axis and a usable
/// SOI. The asset file Content/HyperbolicBodies.xml gives the comets a mass and an SOI.
/// The patches here cover the rest, namely the target listing, the time of
/// flight estimate, the departure alignment, the encounter detection, and the
/// stock closest approach search, which throws on an unbound body.
/// </summary>
internal static class HyperbolicTargets
{
    /// <summary>Transfer window as fractions of the Hohmann estimate. Stock sizes
    /// the window from the target's Period, which is NaN on an unbound orbit.</summary>
    internal const double MinTofRatio = 0.3;
    internal const double MaxTofRatio = 4.0;

    /// <summary>True once every patch of this block applied. Without them stock cannot size or solve a transfer to an unbound body, so no page lists one.</summary>
    internal static bool Enabled { get; set; }

    public static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(Patch_PopulateWithPlanets)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_HohmannFlight)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_SetTransferInfo)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_AlignmentTime)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_TryFindIntercept)).Patch();

        // The asset file gives the comets an SOI whether or not this guard applies, and without the guard stock's encounter scan throws on them, so a missing anchor is an error rather than a quiet degrade.
        if (Patch_FindClosestApproaches.IsAnchorPresent)
            harmony.CreateClassProcessor(typeof(Patch_FindClosestApproaches)).Patch();
        else
            DefaultCategory.Log.Error(
                "[AFC] HyperbolicTargets: PatchedConic.FindClosestApproaches not found. " +
                "Stock's encounter search may throw on a comet that has an SOI; " +
                "re-verify this feature before flying to a comet.");
    }

    /// <summary>The body a transfer from <paramref name="source"/> is solved around.
    /// <c>TransferPlanner.SetTransferInfo</c> departs from the source's parent when
    /// that parent orbits something, so the Lambert solve is about the parent's
    /// parent, and otherwise from the vehicle itself about its own parent.</summary>
    internal static IParentBody? TransferFrame(Vehicle source)
        => (source.Parent as IOrbiter)?.Parent ?? source.Parent;

    /// <summary>Adds the bodies on unbound orbits around the transfer frame of <paramref name="source"/> that have an SOI to stock's DESTINATION list, until the list is full.</summary>
    internal static void AddUnboundTargets(Vehicle source, Span<TransferObject> list, ref int count)
    {
        if (TransferFrame(source) is not IParentBody frame || Universe.CurrentSystem == null)
            return;
        ReadOnlySpan<Astronomical> all = Universe.CurrentSystem.All.AsSpan();
        for (int i = 0; i < all.Length && count < list.Length; i++)
        {
            if (IsUnboundTarget(all[i], source, frame))
                list[count++] = new TransferObject(all[i]);
        }
    }

    /// <summary>Adds the unbound targets to the TRANSFER page list. It runs every frame the page draws, so it reads the system span without a copy.</summary>
    internal static void AddUnboundTargets(Vehicle source, List<TransferObject> list)
    {
        if (!Enabled || TransferFrame(source) is not IParentBody frame || Universe.CurrentSystem == null)
            return;
        ReadOnlySpan<Astronomical> all = Universe.CurrentSystem.All.AsSpan();
        for (int i = 0; i < all.Length; i++)
        {
            if (IsUnboundTarget(all[i], source, frame))
                list.Add(new TransferObject(all[i]));
        }
    }

    private static readonly HashSet<string> _alertedTargets = new();

    /// <summary>Forgets which targets were already alerted, so the next load alerts again.</summary>
    internal static void ResetAlerts() => _alertedTargets.Clear();

    /// <summary>Says once per target that the ideal departure, one transfer time
    /// before the target's periapsis, is already behind the sim time, which is
    /// why the porkchop window looks degenerate. <see cref="Patch_AlignmentTime"/>
    /// evaluates the same condition but also runs on the porkchop worker, and
    /// <c>TimedAlert.Create</c> mutates an unsynchronized list the draw thread walks,
    /// so only draw-thread callers raise it: stock's SetTransferInfo and the TRANSFER page.</summary>
    internal static void MaybeAlertDepartureWindowPast(IOrbiter target, UniverseTime hohmannToF)
    {
        UniverseTime ideal = target.Orbit.TimeAtPeriapsis - hohmannToF;
        if (!(ideal < Universe.GetElapsedTime())) return;

        string targetId = target.Id;
        if (!_alertedTargets.Add(targetId)) return;

        // An unbound target has one periapsis passage, and TransferTask.Run sweeps from the current sim time whatever AlignmentTime returned, so this does not promise a later window.
        AfcAlerts.Info(
            $"{targetId}: the ideal departure, one transfer time before its periapsis, " +
            "has already passed. The porkchop window starts from the current time.",
            Color.Yellow, 6.0);
    }

    private static bool IsUnboundTarget(Astronomical body, Vehicle source, IParentBody frame)
    {
        if (body is Vehicle || body is not IOrbiter orbiter) return false;
        if (orbiter.Orbit == null || orbiter.Orbit.Eccentricity < 1.0) return false;
        if (orbiter.Parent != frame) return false;
        if (orbiter.Id == source.Id || orbiter.Id == source.Parent?.Id) return false;

        if (orbiter.SphereOfInfluence <= 0.0 || double.IsNaN(orbiter.SphereOfInfluence))
        {
            LogHelper.WarnOnce($"soi-missing-{orbiter.Id}",
                $"[AFC] {orbiter.Id} has no SOI, Content/HyperbolicBodies.xml may not have loaded");
            return false;
        }
        return true;
    }
}
