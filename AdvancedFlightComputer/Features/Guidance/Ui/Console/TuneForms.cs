#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using KSA;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

public static partial class GuidanceWindow
{
    private static readonly string[] TuneLandingLabels = { "APPROACH", "ENTRY", "TOUCHDOWN", "G-FOLD", "HOVER" };
    // In TuneDiagnostics order.
    private static readonly string[] TuneDiagnosticsLabels = { "ASCENT", "BOOSTBACK", "AERO", "ATMOSPHERE", "G-FOLD", "TOUCHDOWN", "FLIGHT PLAN" };

    /// <summary>The RESET of a changed value that could be pressed on the set of TUNE in this frame (ResetTail.Draw).</summary>
    internal static int TuneRowResets { get; private set; }

    internal static void DrawTuneGuidanceSet(Vehicle vehicle, float innerW, TuneSet set, ref TuneLanding landing, ref TuneDiagnostics diagnostics)
    {
        TuneRowResets = 0;
        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;
        double bodyRadius = parent.MeanRadius;

        if (set != TuneSet.Diag)
            ConsoleUi.MutedWrapped(PageText.TuningRule);

        switch (set)
        {
            case TuneSet.Ascent:
                bool qAlphaWaits = _s.Running && HasPhysicalAtmosphere(parent);
                ConsoleUi.NoteShape("AfcTuneQAlphaWaits", qAlphaWaits ? 1 : 0);
                if (qAlphaWaits)
                    ConsoleUi.MutedWrapped(PageText.QAlphaNextLaunch);
                using (TierScope.Push(UiTier.Tuning))
                {
                    DrawConvexAscentSection(vehicle, orbit, parent, innerW);
                    foreach (AscentMethodUi method in AscentMethodUis)
                        method.DrawTuning();
                }
                break;

            case TuneSet.Recover:
                using (TierScope.Push(UiTier.Tuning))
                    DrawBoostbackContent(vehicle, orbit, parent, innerW);
                break;

            case TuneSet.Landing:
            {
                var counts = new int[TuneLandingLabels.Length];
                for (int i = 0; i < counts.Length; i++)
                    counts[i] = CountTuneChanged(TuneSet.Landing, (TuneLanding)i);
                int picked = ConsoleUi.PlaceTabs("PART", "AfcTuneLanding", TuneLandingLabels, (int)landing, PageText.TuneLandingCards, counts: counts, small: true);
                if (picked >= 0)
                    landing = (TuneLanding)picked;
                using (TierScope.Push(UiTier.Tuning))
                {
                    switch (landing)
                    {
                        case TuneLanding.Approach:
                            DrawDescentContent(innerW);
                            break;
                        case TuneLanding.Entry:
                            Draw3DofLandingContent(vehicle, innerW);
                            break;
                        case TuneLanding.Gfold:
                            DrawGfoldLandingContent(innerW);
                            break;
                        case TuneLanding.SixDof:
                            Draw6DofLandingContent(vehicle, innerW);
                            break;
                        default:
                            DrawHoverContent(vehicle, orbit, parent, parent.Mu, innerW);
                            break;
                    }
                }
                break;
            }

            default:
            {
                if (OwnerThreadViolation.Length > 0)
                    ConsoleUi.DangerWrapped(ConsoleUi.EscapeFormat("THREADING: " + OwnerThreadViolation));
                if (_s.GuidanceError.Length > 0)
                    ConsoleUi.WarningWrapped(ConsoleUi.EscapeFormat($"LAST STEP ERROR ({_s.FailStreak} IN A ROW): {_s.GuidanceError}"));
                ModeDiagnostics modeView = DrawDiagnosticsTabs(TuneDiagnosticsLabels, ref diagnostics);
                ImGui.Spacing();
                if (modeView != null)
                    modeView.Draw(vehicle, innerW);
                else
                    DrawDiagnostics(vehicle, orbit, parent, innerW, diagnostics);
                break;
            }
        }
    }

    private static void DrawDiagnostics(Vehicle vehicle, Orbit orbit, IParentBody parent, float innerW, TuneDiagnostics diagnostics)
    {
        // The aero model is sampled by the guidance step while a page asks for it, and these pages show it.
        if (diagnostics is TuneDiagnostics.Boostback or TuneDiagnostics.Aero)
            ViewDemand.Mark(vehicle, ViewDemand.Kind.Aero);

        if (diagnostics == TuneDiagnostics.Gfold)
        {
            DrawGfoldDebugInline();
            return;
        }
        if (diagnostics == TuneDiagnostics.FlightPlan)
        {
            Ui.Console.Pages.TunePage.DrawFlightPlanView(vehicle);
            return;
        }
        // TOUCHDOWN: the live figures of touchdown guidance and its telemetry log.
        if (diagnostics == TuneDiagnostics.Log)
        {
            using (TierScope.Push(UiTier.Diagnostics))
                Draw6DofLandingContent(vehicle, innerW);
            Draw6DofLogDiag(vehicle, innerW);
            return;
        }

        long rowsBefore = ConsoleUi.RowsDrawn;
        using (TierScope.Push(UiTier.Diagnostics))
        {
            switch (diagnostics)
            {
                case TuneDiagnostics.Ascent:
                    DrawConvexAscentSection(vehicle, orbit, parent, innerW);
                    DrawAscentOptionsSection(parent.MeanRadius);
                    break;
                case TuneDiagnostics.Boostback:
                    DrawBoostbackGuidanceSection(vehicle, innerW);
                    DrawBoostbackImpactSection(vehicle, parent, innerW);
                    break;
                case TuneDiagnostics.Aero:
                    DrawBoostbackSurrogateSection(vehicle, innerW);
                    DrawBoostbackProfileSection(innerW);
                    break;
                default:
                    DrawBoostbackAtmosphereSection(vehicle, orbit, parent, innerW);
                    break;
            }
        }
        if (ConsoleUi.RowsDrawn == rowsBefore)
            ConsoleUi.MutedWrapped(PageText.DiagEmpty(diagnostics));
    }

    /// <summary>The tuning values of the set on screen, for the footer count and its reset.</summary>
    internal static (SettingSpec[] Doubles, BoolSpec[] Bools) TuneSetSpecs(TuneSet set, TuneLanding landing) => set switch
    {
        TuneSet.Ascent => (Specs.TuningAscent, null),
        TuneSet.Recover => (Specs.TuningBoostback, null),
        TuneSet.Landing => landing switch
        {
            TuneLanding.Approach => (Specs.TuningApproach, null),
            TuneLanding.Entry => (Specs.TuningThreeDof, null),
            TuneLanding.Gfold => (Specs.TuningGfold, null),
            TuneLanding.SixDof => (Specs.TuningSixDof, Specs.TuningSixDofBools),
            _ => (Specs.TuningHover, null),
        },
        _ => (null, null),
    };

    internal static int CountTuneChanged(TuneSet set, TuneLanding landing)
    {
        (SettingSpec[] doubles, BoolSpec[] bools) = TuneSetSpecs(set, landing);
        return SettingAccess.CountChanged(_s, doubles, bools);
    }

    internal static int CountTuneSetChanged(TuneSet set)
        => set == TuneSet.Landing
            ? SettingAccess.CountChanged(_s, Specs.TuningLanding, Specs.TuningSixDofBools)
            : CountTuneChanged(set, TuneLanding.Approach);

    /// <summary>Why RESET THIS SET cannot run now, or empty when it can.</summary>
    internal static string TuneSetResetLock(TuneSet set, TuneLanding landing)
    {
        (SettingSpec[] doubles, BoolSpec[] bools) = TuneSetSpecs(set, landing);
        return ResetLockReason(doubles, bools);
    }

    internal static void ConfirmResetTuneSet(Vehicle vehicle, TuneSet set, TuneLanding landing)
    {
        (SettingSpec[] doubles, BoolSpec[] bools) = TuneSetSpecs(set, landing);
        int changed = SettingAccess.CountChanged(_s, doubles, bools);
        if (changed == 0 || ResetLockReason(doubles, bools).Length > 0)
            return;
        string name = set == TuneSet.Landing ? TuneLandingLabels[(int)landing] : ConsolePages.TuneSetLabel(set);
        ResetPopup.Create(ResetFor(vehicle, doubles, bools), "RESET SET", PageText.ResetSetQuestion(changed, name));
    }
}
