using System.Collections.ObjectModel;
using System.Reflection;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.Ui.Console;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The planner key routing and the pure stock planner. With the routing switch on, a true write to the
// public TransferPlanner.ShowPlanWindow setter from a closed planner leaves the stock window closed and
// raises a console toggle; the STOCK PLANNER link opens the stock window; every close passes; a write
// that bypasses the setter is caught by the frame check. With the switch off the setter is stock. The
// only AFC patches on planner members are the setter prefix and the unbound-target data patches.
public sealed class PlannerRouteTest : AfcTest
{
    private const string AfcHarmonyPrefix = "com.maxi.advancedflightcomputer";

    public override string Name => "afc-planner-route";

    protected override void Execute(TestContext t)
    {
        // The harness never runs Mod.OnFullyLoaded, so the test applies the setter prefix on its own owner and removes it at the end.
        var harmony = new Harmony(AfcHarmonyPrefix + ".harnesstests.plannerroute");
        bool appliedBefore = PlannerRoute.PrefixApplied;
        Vehicle? controlledBefore = Program.ControlledVehicle;
        if (!appliedBefore)
            PlannerRoute.ApplyPatch(harmony);
        try
        {
            CheckPureStock(t);
            // The key routes only with a craft under control.
            foreach (Astronomical body in t.System.All.AsSpan())
            {
                if (body is Vehicle vehicle && vehicle is not KittenEva)
                {
                    Program.ControlledVehicle = vehicle;
                    break;
                }
            }
            if (Program.ControlledVehicle == null)
            {
                t.Fail("controlled craft", "the system has no craft to control");
                return;
            }
            CheckRouting(t);
        }
        finally
        {
            Program.ControlledVehicle = controlledBefore;
            harmony.UnpatchAll(harmony.Id);
            if (!appliedBefore)
                AccessTools.Field(typeof(PlannerRoute), "_prefixApplied").SetValue(null, false);
        }
    }

    private static void CheckRouting(TestContext t)
    {
        FieldInfo? showField = AccessTools.Field(typeof(TransferPlanner), "_showPlanWindow");
        if (showField == null)
        {
            t.Fail("planner field", "TransferPlanner._showPlanWindow not found");
            return;
        }

        bool wasOpen = TransferPlanner.ShowPlanWindow;
        bool wasActive = PlannerRoute.Active;
        try
        {
            if (wasOpen)
                TransferPlanner.ShowPlanWindow = false;
            PlannerRoute.ConsumeToggle(out _);

            PlannerRoute.Active = true;
            if (PlannerRoute.PrefixApplied)
            {
                TransferPlanner.ShowPlanWindow = true;
                t.Check("a routed open leaves the stock window closed", !(bool)showField.GetValue(null)!);
                t.Check("a routed open raises a console toggle", PlannerRoute.ConsumeToggle(out _));
            }
            else
            {
                t.Check("the setter prefix is applied", false, "the frame check routes the key alone");
            }

            PlannerRoute.OpenStockPlanner();
            t.Check("the STOCK PLANNER link opens the stock window",
                TransferPlanner.ShowPlanWindow && PlannerRoute.StockLinkActive && !PlannerRoute.ConsumeToggle(out _));
            PlannerRoute.FrameCheck();
            t.Check("the frame check leaves a linked stock window open", TransferPlanner.ShowPlanWindow);

            TransferPlanner.ShowPlanWindow = false;
            t.Check("a close passes and ends the link", !TransferPlanner.ShowPlanWindow && !PlannerRoute.StockLinkActive);

            showField.SetValue(null, true);
            PlannerRoute.FrameCheck();
            t.Check("the frame check closes a stock window opened past the setter",
                !TransferPlanner.ShowPlanWindow && PlannerRoute.ConsumeToggle(out _));

            PlannerRoute.Active = false;
            TransferPlanner.ShowPlanWindow = true;
            t.Check("with routing off the setter opens the stock window",
                TransferPlanner.ShowPlanWindow && !PlannerRoute.ConsumeToggle(out _));
            TransferPlanner.ShowPlanWindow = false;
        }
        finally
        {
            PlannerRoute.Active = false;
            TransferPlanner.ShowPlanWindow = false;
            PlannerRoute.ConsumeToggle(out _);
            PlannerRoute.OnStockWindowClosed();
            if (wasOpen)
                showField.SetValue(null, true);
            PlannerRoute.Active = wasActive;
        }
    }

    private static void CheckPureStock(TestContext t)
    {
        string[] allowed = ["set_ShowPlanWindow", "PopulateWithPlanets", "SetTransferInfo"];
        var foreign = new List<string>();
        foreach (MethodBase method in Harmony.GetAllPatchedMethods())
        {
            if (method.DeclaringType != typeof(TransferPlanner))
                continue;
            Patches? info = Harmony.GetPatchInfo(method);
            if (info == null || !OwnedByAfc(info))
                continue;
            if (Array.IndexOf(allowed, method.Name) < 0)
                foreign.Add(method.Name);
        }
        t.Check("AFC patches no planner member besides the setter and the unbound-target data", foreign.Count == 0,
            string.Join(", ", foreign));

        MethodInfo? setter = PlannerRoute.Setter;
        Patches? setterInfo = setter == null ? null : Harmony.GetPatchInfo(setter);
        t.Check("the setter carries one AFC prefix and nothing else from AFC",
            setterInfo != null && Count(setterInfo.Prefixes) == 1 && Count(setterInfo.Postfixes) == 0
            && Count(setterInfo.Transpilers) == 0 && Count(setterInfo.Finalizers) == 0);

        string[] afcKeys = [ManeuverTools.KeySetPeriapsis, ManeuverTools.KeySetApoapsis,
            ManeuverTools.KeyMatchInclination, ManeuverTools.KeySetInclination];
        bool anyAfcType = TransferPlanner.TransferTypes.Exists(type => Array.IndexOf(afcKeys, type.GetKey()) >= 0);
        t.Check("the stock planner lists only its own plan types", !anyAfcType,
            string.Join(", ", TransferPlanner.TransferTypes.ConvertAll(type => type.GetKey())));
    }

    private static bool OwnedByAfc(Patches info)
        => Count(info.Prefixes) + Count(info.Postfixes) + Count(info.Transpilers) + Count(info.Finalizers) > 0;

    private static int Count(ReadOnlyCollection<Patch> patches)
        => patches.Count(patch => patch.owner.StartsWith(AfcHarmonyPrefix, StringComparison.Ordinal));
}
