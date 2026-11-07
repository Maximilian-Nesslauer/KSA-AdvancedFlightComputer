#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using KSA;

/// <summary>
/// One ascent method as the LAUNCH page shows it. The page draws the parts every method shares
/// (target orbit, launch window, the profile picture, the commit and the stop) and asks the selected
/// from the method's entry in GuidanceModes. A method that registers through GuidanceWindow.RegisterMode
/// replaces the entry with its mode id here through GuidanceModeHooks.AscentSetup.
/// </summary>
internal sealed record AscentMethodUi(
    GuidanceWindow.AscentMethod Id, GuidanceModeId ModeId,
    Action<Vehicle, Orbit, IParentBody, double, float> DrawPlayer,
    Func<float, FoldPart[]> ExpertParts, Action DrawTuning,
    Func<string> PhaseLabel)
{
    public string Label => GuidanceModes.Of(ModeId)?.MethodLabel ?? ModeId.ToString().ToUpperInvariant();

    public bool IsAvailable => GuidanceModes.Of(ModeId)?.Available ?? false;

    public string UnavailableReason => GuidanceModes.Of(ModeId)?.UnavailableReason ?? GuidanceModes.GravityTurnUnavailable;
}

public static partial class GuidanceWindow
{
    private static AscentMethodUi[] _ascentMethodUis;

    internal static AscentMethodUi[] AscentMethodUis => _ascentMethodUis ??= BuildAscentMethodUis();

    private static AscentMethodUi[] BuildAscentMethodUis() => new[]
    {
        new AscentMethodUi(AscentMethod.CatsUpfg, GuidanceModeId.AscentUpfg,
            DrawPlayer: (vehicle, orbit, parent, bodyRadius, innerW) => { },
            ExpertParts: CatsUpfgParts,
            DrawTuning: () => { },
            PhaseLabel: AscentPhaseLabel),
        // The gravity turn slot. Until its mode registers as available through GuidanceWindow.RegisterMode, its segment shows but does not select.
        new AscentMethodUi(AscentMethod.GravityTurn, GuidanceModeId.AscentGravityTurn,
            DrawPlayer: (vehicle, orbit, parent, bodyRadius, innerW) => { },
            ExpertParts: _ => [],
            DrawTuning: () => { },
            PhaseLabel: () => "IDLE"),
    };

    internal static AscentMethodUi CurrentAscentMethodUi()
    {
        foreach (AscentMethodUi method in AscentMethodUis)
            if (method.Id == _s.AscentMethod && method.IsAvailable)
                return method;
        return AscentMethodUis[0];
    }

    internal static AscentChoice CurrentAscentChoice()
        => CurrentAscentMethodUi().Id == AscentMethod.GravityTurn ? AscentChoice.GravityTurn : AscentChoice.CatsUpfg;

    private const string AscentLockedText = "Locked while an ascent flies, is armed or plans.";

    internal static void DrawAscentMethodRow()
    {
        AscentMethodUi[] methods = AscentMethodUis;
        var names = new string[methods.Length];
        var availableAt = new bool[methods.Length];
        var held = new string[methods.Length];
        var cards = new string[methods.Length];
        int active = 0;
        AscentMethodUi current = CurrentAscentMethodUi();
        bool locked = _s.Running || _s.LaunchArmed || _s.ConvexLaunchPending || AscentPlanBusy;
        int available = 0;
        for (int i = 0; i < methods.Length; i++)
        {
            availableAt[i] = methods[i].IsAvailable;
            if (availableAt[i])
                available++;
            names[i] = methods[i].Label;
            if (ReferenceEquals(methods[i], current))
                active = i;
            cards[i] = PageText.AscentCards[methods[i].Id == AscentMethod.CatsUpfg ? 0 : 1];
        }
        for (int i = 0; i < methods.Length; i++)
            held[i] = !methods[i].IsAvailable ? methods[i].UnavailableReason
                : locked && i != active ? AscentLockedText
                : "";

        bool cats = current.Id == AscentMethod.CatsUpfg;
        bool row = FormRules.AscentRowShows(available);
        ConsoleUi.NoteShape("AfcAscentRow", row ? 1 : 0);
        if (row)
        {
            string[] labels = PageText.AscentSegments(names, availableAt, segments => ConsoleUi.ChoiceRowFits("ASCENT", segments));
            int picked = ConsoleUi.ChoiceRow("ASCENT", "AfcAscentMethod", labels, active, held, cards,
                cats ? "" : cards[active], "How the craft climbs to orbit. Each segment says what it flies.");
            if (!locked && picked >= 0 && picked < methods.Length && methods[picked].IsAvailable)
                _s.AscentMethod = methods[picked].Id;
        }

        if (!cats)
            return;
        int climb = _s.FlyConvexAscent ? 0 : 1;
        string[] climbHeld = locked
            ? new[] { climb == 0 ? "" : AscentLockedText, climb == 1 ? "" : AscentLockedText }
            : null;
        int pickedClimb = ConsoleUi.ChoiceRow("CLIMB", "AfcAscentClimb", PageText.ClimbLabels, climb, climbHeld, PageText.ClimbCards,
            PageText.ClimbLines[climb], PageText.ClimbTip(current.Label), feedsPicture: true);
        if (!locked && pickedClimb >= 0)
            _s.FlyConvexAscent = pickedClimb == 0;
    }

    private static FoldPart[] CatsUpfgParts(float innerW)
    {
        Vehicle vehicle = Program.ControlledVehicle;
        Orbit orbit = vehicle?.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return [];
        return
        [
            new FoldPart(FoldNames.Turn, true, InExpert(() => DrawAscentSettingsSection(innerW))),
            new FoldPart(FoldNames.ConvexPlan, true, InExpert(() =>
            {
                ConsoleUi.KeepSectionHead(true);
                DrawConvexAscentSection(vehicle, orbit, parent, innerW);
                ConsoleUi.KeepSectionHead(false);
            })),
        ];
    }
}
