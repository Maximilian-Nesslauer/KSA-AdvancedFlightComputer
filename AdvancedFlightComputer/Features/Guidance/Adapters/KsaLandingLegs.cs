#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using KSA;

/// <summary>
/// The landing legs of a craft, as KSA models them: a leg is a part with a two-state deploy and retract animation (KeyframeAnimationModule with ShowDeployRetract), and the part carries the editor tag Landing. The game has no leg module of its own, so this is the one place that decides what a leg is, for the deploy of a landing and for the height the landing plans with (KsaVehicleHeight).
///  A parachute bay door is tagged Landing too, but it belongs to its parachute (Parachute.SetBayDoorsOpen opens and closes it), so a part with a parachute is not a leg. A sun-tracking animation (a solar panel) has an extension that turns it on its own, and a slider actuator has no deployed end. Ramps, dishes, booms and wings are not tagged Landing. A part from a mod that has no Landing tag counts as a leg when its template or the animation key names a landing leg.
///  A deploy is what the part menu does (KeyframeAnimationModule.ShowContextMenu): the goal time goes to the end of the animation, and KeyframeAnimationModule.UpdateModules moves the legs there by the simulation step, so a leg takes the length of its animation to come down.
/// </summary>
internal static class KsaLandingLegs
{
    private static readonly EditorTag LandingTag = new("Landing");

    // A save keeps the goal time with four decimals, so a deployed leg of a loaded craft reads a hair under the duration.
    private const float DeployedToleranceS = 0.001f;

    public static bool IsLeg(KeyframeAnimationModule anim)
    {
        if (anim == null || !anim.ShowDeployRetract || anim.Extension != null || anim.Shared == null || anim.Shared.Duration <= 0f)
            return false;
        Part part = anim.Parent;
        Part full = part?.FullPart;
        if (full == null || full.SubtreeModules.Get<Parachute>().Length > 0)
            return false;
        return Tagged(part) || Tagged(full) || NamesLeg(anim.TemplateId) || NamesLeg(full.Template?.Id);
    }

    /// <summary>The goal of the leg is its deployed end, whether it is there yet or on its way.</summary>
    public static bool DeployCommanded(KeyframeAnimationModule anim) => anim.TimeGoal >= anim.Shared.Duration - DeployedToleranceS;

    public static int Count(Vehicle vehicle) => Count(vehicle, out _);

    /// <summary>The landing legs of <paramref name="vehicle"/>, and through <paramref name="stowed"/> how many of them are not commanded to their deployed end.</summary>
    public static int Count(Vehicle vehicle, out int stowed)
    {
        stowed = 0;
        PartTree tree = vehicle?.Parts;
        if (tree == null)
            return 0;
        int legs = 0;
        foreach (KeyframeAnimationModule anim in tree.Modules.Get<KeyframeAnimationModule>())
        {
            if (!IsLeg(anim))
                continue;
            legs++;
            if (!DeployCommanded(anim))
                stowed++;
        }
        return legs;
    }

    /// <summary>Commands every landing leg of <paramref name="vehicle"/> that is not on its way down to its deployed end, and returns how many it commanded. It never retracts one.</summary>
    public static int DeployAll(Vehicle vehicle)
    {
        PartTree tree = vehicle?.Parts;
        if (tree == null)
            return 0;
        int commanded = 0;
        foreach (KeyframeAnimationModule anim in tree.Modules.Get<KeyframeAnimationModule>())
        {
            if (!IsLeg(anim) || DeployCommanded(anim))
                continue;
            anim.TimeGoal = anim.Shared.Duration;
            commanded++;
        }
        return commanded;
    }

    /// <summary>The longest deploy animation of the legs of <paramref name="vehicle"/>, s, or 0 with no legs.</summary>
    public static double DeploySeconds(Vehicle vehicle)
    {
        PartTree tree = vehicle?.Parts;
        if (tree == null)
            return 0.0;
        double longest = 0.0;
        foreach (KeyframeAnimationModule anim in tree.Modules.Get<KeyframeAnimationModule>())
        {
            if (IsLeg(anim))
                longest = Math.Max(longest, anim.Shared.Duration);
        }
        return longest;
    }

    private static bool Tagged(Part part) => part?.Template != null && part.Template.HasEditorTag(LandingTag);

    private static bool NamesLeg(string id) => id != null && id.Contains("LandingLeg", StringComparison.OrdinalIgnoreCase);
}
