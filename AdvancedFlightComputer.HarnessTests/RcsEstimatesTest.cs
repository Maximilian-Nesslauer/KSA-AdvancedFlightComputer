using AdvancedFlightComputer.Features.RcsTranslation;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;

namespace AdvancedFlightComputer.HarnessTests;

// Pure tests for the Auto attitude decision: the propellant a strategy would
// need (RcsEstimates.RequiredPropellantKg, which the sufficiency warning and
// the activation alert share) and the Hold-vs-Align resolution
// (RcsExecutor.ResolveStrategy), including the preference margin that keeps
// Auto from slewing for a marginal saving. Estimates are hand-built, so no
// vehicle is involved.
public sealed class RcsEstimatesTest : IHarnessTest
{
    public string Name => "afc-rcs-estimates";

    public int Run(HeadlessSession session)
    {
        bool ok = true;
        ok &= CheckRequiredPropellant();
        ok &= CheckResolveStrategy();
        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    // Hold 100 kg, Align 40 + 20 = 60 kg total, both feasible.
    private static RcsEstimates BothFeasible() => new()
    {
        Valid = true,
        HoldFeasible = true,
        HoldPropellantKg = 100.0,
        HoldDurationSec = 50.0,
        AlignFeasible = true,
        AlignPropellantKg = 40.0,
        AlignSlewPropellantKg = 20.0,
        AlignDurationSec = 30.0,
        AlignSlewDurationSec = 10.0,
        AlignAxis = 0,
    };

    private bool CheckRequiredPropellant()
    {
        bool ok = true;
        RcsEstimates est = BothFeasible();

        ok &= Near("hold required", est.RequiredPropellantKg(RcsAttitudeStrategy.Hold), 100.0);
        ok &= Near("align required", est.RequiredPropellantKg(RcsAttitudeStrategy.Align), 60.0);
        // Auto takes the cheaper feasible strategy.
        ok &= Near("auto required", est.RequiredPropellantKg(RcsAttitudeStrategy.Auto), 60.0);

        // Only Hold feasible: Auto falls to Hold, Align reports nothing.
        RcsEstimates holdOnly = est;
        holdOnly.AlignFeasible = false;
        ok &= Near("hold-only auto", holdOnly.RequiredPropellantKg(RcsAttitudeStrategy.Auto), 100.0);
        ok &= Near("hold-only align zero", holdOnly.RequiredPropellantKg(RcsAttitudeStrategy.Align), 0.0);

        // Only Align feasible: Auto falls to Align, Hold reports nothing.
        RcsEstimates alignOnly = est;
        alignOnly.HoldFeasible = false;
        ok &= Near("align-only auto", alignOnly.RequiredPropellantKg(RcsAttitudeStrategy.Auto), 60.0);
        ok &= Near("align-only hold zero", alignOnly.RequiredPropellantKg(RcsAttitudeStrategy.Hold), 0.0);

        // Neither feasible: nothing is required.
        RcsEstimates none = est;
        none.HoldFeasible = false;
        none.AlignFeasible = false;
        ok &= Near("none required", none.RequiredPropellantKg(RcsAttitudeStrategy.Auto), 0.0);
        return ok;
    }

    private bool CheckResolveStrategy()
    {
        bool ok = true;
        RcsEstimates est = BothFeasible();

        // Explicit Hold is always Hold.
        var (s, ax) = RcsExecutor.ResolveStrategy(RcsAttitudeStrategy.Hold, in est);
        ok &= Check("explicit hold", s == RcsAttitudeStrategy.Hold && ax == -1);

        // Explicit Align with a feasible axis aligns.
        (s, ax) = RcsExecutor.ResolveStrategy(RcsAttitudeStrategy.Align, in est);
        ok &= Check("explicit align feasible", s == RcsAttitudeStrategy.Align && ax == 0);

        // Explicit Align with no feasible axis degrades to Hold.
        RcsEstimates noAlign = est;
        noAlign.AlignFeasible = false;
        (s, ax) = RcsExecutor.ResolveStrategy(RcsAttitudeStrategy.Align, in noAlign);
        ok &= Check("explicit align infeasible", s == RcsAttitudeStrategy.Hold && ax == -1);

        // Auto: Align total 60 clears 0.9 x Hold (90), so Auto slews.
        (s, ax) = RcsExecutor.ResolveStrategy(RcsAttitudeStrategy.Auto, in est);
        ok &= Check("auto align cheaper", s == RcsAttitudeStrategy.Align && ax == 0);

        // Auto: Align total 95 does not clear the 90 margin, so Auto holds
        // rather than pay a slew for a marginal saving.
        RcsEstimates marginal = est;
        marginal.AlignPropellantKg = 75.0;
        marginal.AlignSlewPropellantKg = 20.0;
        (s, ax) = RcsExecutor.ResolveStrategy(RcsAttitudeStrategy.Auto, in marginal);
        ok &= Check("auto margin holds", s == RcsAttitudeStrategy.Hold && ax == -1);

        // Auto: Hold infeasible but Align feasible forces the slew.
        RcsEstimates noHold = est;
        noHold.HoldFeasible = false;
        (s, ax) = RcsExecutor.ResolveStrategy(RcsAttitudeStrategy.Auto, in noHold);
        ok &= Check("auto hold infeasible", s == RcsAttitudeStrategy.Align && ax == 0);

        // Auto: neither feasible degrades to Hold (the activation feasibility
        // check refuses it afterwards).
        RcsEstimates none = est;
        none.HoldFeasible = false;
        none.AlignFeasible = false;
        (s, ax) = RcsExecutor.ResolveStrategy(RcsAttitudeStrategy.Auto, in none);
        ok &= Check("auto none feasible", s == RcsAttitudeStrategy.Hold && ax == -1);
        return ok;
    }

    private bool Near(string label, double actual, double expected)
    {
        bool ok = Math.Abs(actual - expected) < 1e-6 * Math.Max(1.0, Math.Abs(expected));
        if (!ok)
            HarnessLog.Line($"[{Name}] TEST {label}: got {actual}, expected {expected} => FAIL");
        return ok;
    }

    private bool Check(string label, bool condition)
    {
        if (!condition)
            HarnessLog.Line($"[{Name}] TEST {label} => FAIL");
        return condition;
    }
}
