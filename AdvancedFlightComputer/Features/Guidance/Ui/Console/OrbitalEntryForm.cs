#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using KSA;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

// The rows of LAND > ENTRY > ORBITAL ENTRY, by tier: what the entry does now and its limits on the page, its hand-over and bellyflop in HAND-OVERS, its attitude and bank in the GNC fold, its tracking and re-plans on TUNE > LANDING > ORBITAL ENTRY, and the aim of DEORBIT ONLY's entry burn in the GNC fold of DEORBIT.
public static partial class GuidanceWindow
{
    /// <summary>The way content of ORBITAL ENTRY on LAND, and its TUNE part: each row draws in its own tier's scope only.</summary>
    private static void DrawOrbitalEntryContent(Vehicle vehicle, float innerW)
    {
        var st = _s.OrbitalEntry;
        if (TierScope.IsPrimaryPass)
        {
            // A refusal or an end the entry left stays on the page while nothing flies, as the head shows it for a while.
            if (!OrbitalEntryLive && !st.EngagePending && st.StatusWarn && st.Status.Length > 0)
                ConsoleUi.Warning(Fmt.Sentence(st.Status).AsSpan());
            if (OrbitalEntryLive)
                DrawOrbitalEntryLiveRows(st);
        }

        // The plan's limits: the player's, read at each re-plan.
        GaugeRow(Specs.OrbitalEntryQMaxKPa, ref _s.OrbitalEntryQMaxKPa);
        GaugeRow(Specs.OrbitalEntryGMax, ref _s.OrbitalEntryGMax);
        GaugeRow(Specs.OrbitalEntryHeatMaxKWm2, ref _s.OrbitalEntryHeatMaxKWm2);
        // What this craft can hold at best, once the entry has its model: a MAX HEATING under it cannot be met.
        if (TierScope.IsPrimaryPass && OrbitalEntryLive && double.IsFinite(st.HeatFloorKWm2))
        {
            string least = FormattableString.Invariant($"{st.HeatFloorKWm2:F0} kW/m^2 AT {st.HeatFloorSpeedMs:F0} m/s");
            if (_s.OrbitalEntryHeatMaxKWm2 > 0 && _s.OrbitalEntryHeatMaxKWm2 < st.HeatFloorKWm2)
                GaugeRowText("LEAST HEATING", least + ", OVER THE LIMIT", ConsoleStyle.Danger);
            else
                GaugeRowText("LEAST HEATING", least);
        }

        if (GuidanceRegion.Begin("ORBITAL ENTRY PARAMETERS", GuidanceRegion.Tuning))
        {
            GaugeRow(Specs.OrbitalEntryTrackOmega, ref _s.OrbitalEntryTrackOmega);
            GaugeRow(Specs.OrbitalEntryMeasureSmoothS, ref _s.OrbitalEntryMeasureSmoothS);
            GaugeRow(Specs.OrbitalEntryGainDragFloor, ref _s.OrbitalEntryGainDragFloor);
            GaugeRow(Specs.OrbitalEntrySmoothS, ref _s.OrbitalEntrySmoothS);
            GaugeRow(Specs.OrbitalEntryCorridorWeight, ref _s.OrbitalEntryCorridorWeight);
            GaugeRow(Specs.OrbitalEntryResolveS, ref _s.OrbitalEntryResolveS);
            GaugeRow(Specs.OrbitalEntryBellyflopResolveS, ref _s.OrbitalEntryBellyflopResolveS);
            GuidanceRegion.End();
        }
    }

    // What the entry flies now, the readouts of the ENTRY TRAJ window that a pilot reads in the console: where it is, what it banks, how it tracks.
    private static void DrawOrbitalEntryLiveRows(OrbitalEntryState st)
    {
        var o = st.Last;
        if (st.Phase == OrbitalEntryPhase.Bellyflop)
        {
            var bf = st.Bellyflop;
            GaugeRowText("BELLYFLOP", FormattableString.Invariant($"PITCH {bf.PitchDeg:F0}, BANK {bf.BankDeg:F0}, YAW {bf.YawDeg:F0} deg"));
            if (bf.Plan is { } fall)
                GaugeRowText("PLAN LANDS", Fmt.Distance(fall.MissM) + " FROM ITS AIM");
            return;
        }
        if (st.Phase == OrbitalEntryPhase.Sweeping)
            return;
        GaugeRowText("RANGE TO GO", FormattableString.Invariant($"{st.RangeM / 1000:F0} km, {st.CrossrangeM / 1000:F1} km ACROSS"));
        GaugeRowText("MACH, ALTITUDE", FormattableString.Invariant($"{st.Mach:F1}, {st.AltitudeM / 1000:F1} km"));
        GaugeRowText("BANK", FormattableString.Invariant($"{(o.BankDeg >= 0 ? "R" : "L")}{Math.Abs(o.BankDeg):F0} deg (PLAN {o.BankRefDeg:F0}){(o.Overbanked ? ", OVERBANK" : "")}"));
        GaugeRowText("DRAG", FormattableString.Invariant($"{st.DragMs2:F1} OF {o.DragRefMs2:F1} m/s^2"));
        GaugeRowText("ANGLE OF ATTACK", FormattableString.Invariant($"{st.AlphaFlownDeg:F0} deg (ASKED {st.AlphaDeg:F0})"));
        if (_s.OrbitalEntryBellyflop && double.IsFinite(st.Bellyflop.TerminalRangeM) && st.Bellyflop.TerminalRangeM > 0)
            GaugeRowText("BELLYFLOP FALL", FormattableString.Invariant($"{st.Bellyflop.TerminalRangeM / 1000:F1} km FROM THE HAND-OVER"));
    }

    /// <summary>The HAND-OVERS fold of ORBITAL ENTRY: where the entry ends, and bellyflop after it.</summary>
    private static void DrawOrbitalEntryHandOverRows()
    {
        GaugeRow(Specs.OrbitalEntryHandoverMach, ref _s.OrbitalEntryHandoverMach);
        GaugeRow(Specs.OrbitalEntryHandoverAltKm, ref _s.OrbitalEntryHandoverAltKm);
        // Bellyflop is chosen at the take-over, so a switch in the bellyflop phase would change nothing.
        using (new ImGuiDisabledScope(_s.OrbitalEntry.Phase == OrbitalEntryPhase.Bellyflop))
            GaugeRowCheck(Specs.OrbitalEntryBellyflop, ref _s.OrbitalEntryBellyflop);
        using (new ImGuiDisabledScope(!_s.OrbitalEntryBellyflop))
        {
            GaugeRow(Specs.OrbitalEntryBellyflopArmMach, ref _s.OrbitalEntryBellyflopArmMach);
            GaugeRow(Specs.OrbitalEntryBellyflopAimUprangeM, ref _s.OrbitalEntryBellyflopAimUprangeM);
            GaugeRowCheck(Specs.OrbitalEntryBellyflopTouchdown, ref _s.OrbitalEntryBellyflopTouchdown);
            using (new ImGuiDisabledScope(!_s.OrbitalEntryBellyflopTouchdown))
                GaugeRow(Specs.OrbitalEntryBellyflopTouchdownS, ref _s.OrbitalEntryBellyflopTouchdownS);
        }
    }

    /// <summary>GNC > ATTITUDE of ORBITAL ENTRY: the angle of attack it flies, the belly that faces the air, and what turns the craft.</summary>
    private static void DrawOrbitalEntryAttitudeRows()
    {
        GaugeRowCheck(Specs.OrbitalEntryTrimAoa, ref _s.OrbitalEntryTrimAoa);
        GaugeRow(Specs.OrbitalEntryAoaDeg, ref _s.OrbitalEntryAoaDeg);
        using (new ImGuiDisabledScope(!_s.OrbitalEntryTrimAoa))
        {
            GaugeRow(Specs.OrbitalEntryAoaMinDeg, ref _s.OrbitalEntryAoaMinDeg);
            GaugeRow(Specs.OrbitalEntryAoaMaxDeg, ref _s.OrbitalEntryAoaMaxDeg);
        }
        GaugeRow(Specs.OrbitalEntryBellyAxis, ref _s.OrbitalEntryBellyAxis);
        GaugeRow(Specs.OrbitalEntryNoseRadiusM, ref _s.OrbitalEntryNoseRadiusM);
        GaugeRowCheck(Specs.OrbitalEntryOwnAttitude, ref _s.OrbitalEntryOwnAttitude);
        using (new ImGuiDisabledScope(!_s.OrbitalEntryOwnAttitude))
        {
            GaugeRowCheck(Specs.OrbitalEntryUseRcs, ref _s.OrbitalEntryUseRcs);
            using (new ImGuiDisabledScope(!_s.OrbitalEntryUseRcs))
                GaugeRow(Specs.OrbitalEntryRcsDeadbandDeg, ref _s.OrbitalEntryRcsDeadbandDeg);
        }
        GaugeRow(Specs.OrbitalEntryBellyflopPitchUpDeg, ref _s.OrbitalEntryBellyflopPitchUpDeg);
        GaugeRow(Specs.OrbitalEntryBellyflopAngleMaxDeg, ref _s.OrbitalEntryBellyflopAngleMaxDeg);
        GaugeRow(Specs.OrbitalEntryBellyflopBankMaxDeg, ref _s.OrbitalEntryBellyflopBankMaxDeg);
    }

    /// <summary>GNC > BANK of ORBITAL ENTRY: the bank the plan and the tracker may use, the reversals, and where the tracker takes hold.</summary>
    private static void DrawOrbitalEntryBankRows()
    {
        GaugeRow(Specs.OrbitalEntryBankMinDeg, ref _s.OrbitalEntryBankMinDeg);
        GaugeRow(Specs.OrbitalEntryBankMaxDeg, ref _s.OrbitalEntryBankMaxDeg);
        GaugeRow(Specs.OrbitalEntryOverbankDeg, ref _s.OrbitalEntryOverbankDeg);
        GaugeRow(Specs.OrbitalEntryReversalBandDeg, ref _s.OrbitalEntryReversalBandDeg);
        GaugeRow(Specs.OrbitalEntryRollRateDegS, ref _s.OrbitalEntryRollRateDegS);
        GaugeRow(Specs.OrbitalEntryTrackRollRateDegS, ref _s.OrbitalEntryTrackRollRateDegS);
        GaugeRowCheck(Specs.OrbitalEntryEngageInAir, ref _s.OrbitalEntryEngageInAir);
        using (new ImGuiDisabledScope(_s.OrbitalEntryEngageInAir))
            GaugeRow(Specs.OrbitalEntryStartDragMs2, ref _s.OrbitalEntryStartDragMs2);
    }

    /// <summary>GNC > ENTRY AIM of DEORBIT: where DEORBIT ONLY's entry burn puts the site in orbital entry's corridor.</summary>
    private static void DrawEntryAimRows()
    {
        using (new ImGuiDisabledScope(!_s.OrbitalEntryAimDeorbit))
        {
            GaugeRow(Specs.OrbitalEntryAimFraction, ref _s.OrbitalEntryAimFraction);
            GaugeRow(Specs.OrbitalEntryAimMaxCrossKm, ref _s.OrbitalEntryAimMaxCrossKm);
            GaugeRow(Specs.OrbitalEntryAimHorizonH, ref _s.OrbitalEntryAimHorizonH);
        }
    }
}
