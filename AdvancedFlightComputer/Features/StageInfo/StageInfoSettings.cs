using System.Collections.Generic;
using KSA;

namespace AdvancedFlightComputer.Features.StageInfo;

public enum StageDisplayMode
{
    Auto,
    Vac,
    Asl,
    VacAsl,
    Planning
}

public readonly record struct AnalysisEnvironment(
    float PrimaryPressure,
    float? PrimarySurfaceGravity,
    float? SecondaryPressure,
    float? SecondarySurfaceGravity,
    string PrimaryLabel,
    string? SecondaryLabel,
    bool IsPrimaryCurrentCondition
);

public static class StageInfoSettings
{
    public static StageDisplayMode Mode = StageDisplayMode.Auto;
    public static string? SelectedBodyId;

    private static readonly List<Astronomical> _bodiesCache = new();

    public static AnalysisEnvironment ResolveEnvironment(Vehicle vehicle)
    {
        float currentPressure = vehicle.LastKinematicStates.AtmosphericPressure;
        bool inAtmosphere = currentPressure > 0f;

        return Mode switch
        {
            StageDisplayMode.Auto => new AnalysisEnvironment(
                PrimaryPressure: currentPressure,
                PrimarySurfaceGravity: null,
                SecondaryPressure: null,
                SecondarySurfaceGravity: null,
                PrimaryLabel: inAtmosphere ? "(ATM)" : "(VAC)",
                SecondaryLabel: null,
                IsPrimaryCurrentCondition: true),

            StageDisplayMode.Vac => new AnalysisEnvironment(
                PrimaryPressure: 0f,
                PrimarySurfaceGravity: null,
                SecondaryPressure: null,
                SecondarySurfaceGravity: null,
                PrimaryLabel: "(VAC)",
                SecondaryLabel: null,
                IsPrimaryCurrentCondition: !inAtmosphere),

            StageDisplayMode.Asl => new AnalysisEnvironment(
                PrimaryPressure: GetSeaLevelPressure(vehicle.Parent),
                PrimarySurfaceGravity: null,
                SecondaryPressure: null,
                SecondarySurfaceGravity: null,
                PrimaryLabel: "(ASL)",
                SecondaryLabel: null,
                IsPrimaryCurrentCondition: false),

            StageDisplayMode.VacAsl => new AnalysisEnvironment(
                PrimaryPressure: 0f,
                PrimarySurfaceGravity: null,
                SecondaryPressure: GetSeaLevelPressure(vehicle.Parent),
                SecondarySurfaceGravity: null,
                PrimaryLabel: "[VAC]",
                SecondaryLabel: "[ASL]",
                IsPrimaryCurrentCondition: !inAtmosphere),

            StageDisplayMode.Planning => ResolvePlanningEnvironment(vehicle),

            _ => new AnalysisEnvironment(0f, null, null, null, "(VAC)", null, true)
        };
    }

    public static List<Astronomical> GetCelestialBodies()
    {
        _bodiesCache.Clear();

        if (Universe.CurrentSystem == null)
            return _bodiesCache;

        foreach (Astronomical astro in Universe.CurrentSystem.All.GetList())
        {
            if (astro is Vehicle)
                continue;
            if (astro is IParentBody)
                _bodiesCache.Add(astro);
        }

        return _bodiesCache;
    }

    public static float ComputeSurfaceGravity(IParentBody body)
    {
        double r = body.MeanRadius;
        if (r <= 0.0)
            return 0f;
        return (float)(Constants.GRAVITATIONAL_CONSTANT * body.Mass / (r * r));
    }

    public static float GetSeaLevelPressure(IParentBody? body)
    {
        if (body is Astronomical astro)
        {
            var atmo = astro.GetAtmosphereReference();
            if (atmo != null)
                return (float)(double)atmo.Physical.SeaLevelPressure;
        }
        return 0f;
    }

    public static void Reset()
    {
        Mode = StageDisplayMode.Auto;
        SelectedBodyId = null;
        _bodiesCache.Clear();
    }

    private static AnalysisEnvironment ResolvePlanningEnvironment(Vehicle vehicle)
    {
        IParentBody? body = FindSelectedBody();
        if (body == null)
        {
            return new AnalysisEnvironment(0f, null, null, null, "(VAC)", null, true);
        }

        float pressure = GetSeaLevelPressure(body);
        float gravity = ComputeSurfaceGravity(body);
        bool hasAtmosphere = pressure > 0f;
        string bodyName = (body as Astronomical)?.Id ?? "?";
        string label = hasAtmosphere
            ? $"({bodyName} ASL)"
            : $"({bodyName})";

        return new AnalysisEnvironment(
            PrimaryPressure: pressure,
            PrimarySurfaceGravity: gravity,
            SecondaryPressure: null,
            SecondarySurfaceGravity: null,
            PrimaryLabel: label,
            SecondaryLabel: null,
            IsPrimaryCurrentCondition: false);
    }

    private static IParentBody? FindSelectedBody()
    {
        if (SelectedBodyId == null || Universe.CurrentSystem == null)
            return null;

        foreach (Astronomical astro in Universe.CurrentSystem.All.GetList())
        {
            if (astro is Vehicle)
                continue;
            if (astro is IParentBody body && astro.Id == SelectedBodyId)
                return body;
        }

        return null;
    }
}
