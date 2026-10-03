using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.PlanWindow;
using AdvancedFlightComputer.HarnessTests.Framework;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class BurnMenuLauncherTest : AfcTest
{
    public override string Name => "afc-burn-menu-launcher";

    protected override void Execute(TestContext t)
    {
        Vehicle? source = null;
        foreach (Astronomical body in t.System.All.AsSpan())
        {
            if (body is Vehicle vehicle && vehicle is not KittenEva && !MultiPassRegistry.Has(vehicle.Id))
            {
                source = vehicle;
                break;
            }
        }
        if (source == null)
        {
            t.Fail("source vehicle", "no vehicle without an active multi-pass execution");
            return;
        }
        if (StockPlanner.TransferBeingCalculated)
        {
            t.Fail("planner idle", "a stock transfer calculation is running");
            return;
        }

        using var state = new SavedState();
        string[] keys = [ManeuverTools.KeySetApoapsis, ManeuverTools.KeySetPeriapsis,
            ManeuverTools.KeyMatchInclination, ManeuverTools.KeySetInclination];
        ManeuverTools.InjectTransferTypes();
        CheckTypeOrder(t);
        foreach (string key in keys)
            CheckShortcut(t, key, source);
        CheckInterstellarReset(t);

        TransferPlanner.TransferTypes.RemoveAll(type => type.GetKey() == ManeuverTools.KeySetApoapsis);
        StockPlanner.TransferType = new TransferType("S3-sentinel", "Sentinel");
        StockPlanner.TransferCalculated = true;
        StockPlanner.ShowPlanWindow = false;
        TransferObject sourceBefore = StockPlanner.SourceBody;
        BurnMenuLauncher.OpenPlanner(ManeuverTools.KeySetApoapsis, source);
        t.Check("removed shortcut leaves planner unchanged",
            !TransferPlanner.ShowPlanWindow && StockPlanner.TransferCalculated
            && StockPlanner.TransferTypeKey == "S3-sentinel"
            && StockPlanner.SourceBody.GetKey() == sourceBefore.GetKey());

        CheckWindowClose(t, source);
    }

    // The orbit tools sit right after stock's circularize entries and ahead of the interstellar planner.
    private static void CheckTypeOrder(TestContext t)
    {
        List<string> keys = TransferPlanner.TransferTypes.ConvertAll(type => type.GetKey());
        int circularize = keys.IndexOf(ManeuverTools.KeyStockCircularizePeriapsis);
        int first = keys.IndexOf(ManeuverTools.KeySetPeriapsis);
        int last = keys.IndexOf(ManeuverTools.KeySetInclination);
        int interstellar = keys.IndexOf(ManeuverTools.KeyStockInterstellar);
        t.Check("AFC plan types follow the stock circularize entries, ahead of Interstellar",
            circularize >= 0 && first == circularize + 1 && last == first + 3 && (interstellar < 0 || interstellar > last),
            string.Join(", ", keys));
    }

    private static void CheckShortcut(TestContext t, string key, Vehicle source)
    {
        Field(typeof(ManeuverToolsWindow), "_defaultsInitialized").SetValue(null, true);
        Field(typeof(ManeuverToolsWindow), "_nodeDefaultInitialized").SetValue(null, true);
        StockPlanner.ShowPlanWindow = false;
        StockPlanner.SourceBody = new TransferObject(-1);
        StockPlanner.TransferCalculated = true;

        BurnMenuLauncher.OpenPlanner(key, source);
        t.Check(key + " opens the selected tool",
            TransferPlanner.ShowPlanWindow && StockPlanner.TransferTypeKey == key
            && ReferenceEquals(StockPlanner.SourceBody.Body, source)
            && !StockPlanner.TransferCalculated);
        t.Check(key + " resets input defaults",
            !(bool)Field(typeof(ManeuverToolsWindow), "_defaultsInitialized").GetValue(null)!
            && !(bool)Field(typeof(ManeuverToolsWindow), "_nodeDefaultInitialized").GetValue(null)!);
        t.Check(key + " applies the stock type reset",
            DestinationKey() == "N/A" && Field(typeof(TransferPlanner), "_selectedEntry").GetValue(null) == null
            && SelectedTimeUnit() == TimeUnit.Minutes,
            $"destination {DestinationKey()}, time unit {SelectedTimeUnit()}");
    }

    // TransferPlanner.DrawPlanWindow starts an interstellar plan in years and every other type in minutes.
    private static void CheckInterstellarReset(TestContext t)
    {
        int index = TransferPlanner.TransferTypes.FindIndex(
            type => type.GetKey() == ManeuverTools.KeyStockInterstellar);
        if (index < 0)
        {
            t.Fail("interstellar type", "the stock plan types have no " + ManeuverTools.KeyStockInterstellar + " entry");
            return;
        }
        StockPlanner.SelectType(TransferPlanner.TransferTypes[index]);
        t.Check("an interstellar switch selects years and clears the destination",
            SelectedTimeUnit() == TimeUnit.Years && DestinationKey() == "N/A"
            && Field(typeof(TransferPlanner), "_selectedEntry").GetValue(null) == null,
            $"destination {DestinationKey()}, time unit {SelectedTimeUnit()}");
    }

    private static string DestinationKey()
        => ((TransferObject)Field(typeof(TransferPlanner), "_destinationBody").GetValue(null)!).GetKey();

    private static TimeUnit SelectedTimeUnit()
        => ((TimeObject)Field(typeof(TransferPlanner), "_selectedTimeUnit").GetValue(null)!).Unit;

    private static void CheckWindowClose(TestContext t, Vehicle source)
    {
        FieldInfo lastSource = Field(typeof(Patch_DrawPlanWindow), "_lastSource");
        FieldInfo lastEntry = Field(typeof(Patch_DrawPlanWindow), "_lastEntry");
        var entry = new OrbitalTransfers.PorkChopEntry(new OrbitalTransfers.TransferData(), FlightPlan.CreateUninitialized(source.Hash));
        lastSource.SetValue(null, source);
        lastEntry.SetValue(null, entry);
        StockPlanner.ShowPlanWindow = true;
        Patch_DrawPlanWindow.TickWindowState();
        t.Check("open window retains plan state",
            ReferenceEquals(lastSource.GetValue(null), source) && ReferenceEquals(lastEntry.GetValue(null), entry));

        StockPlanner.ShowPlanWindow = false;
        StockPlanner.TransferType = new TransferType("S3-unhandled", "Unhandled");
        // An unhandled type must clear the state of a closed window before returning without a viewport.
        MethodInfo postfix = typeof(Patch_OnPreRender).GetMethod("Postfix", BindingFlags.Static | BindingFlags.NonPublic)!;
        postfix.Invoke(null, [null]);
        t.Check("closed window clears state before the type gate",
            lastSource.GetValue(null) == null && lastEntry.GetValue(null) == null);
    }

    private static FieldInfo Field(Type type, string name)
        => type.GetField(name, BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic)
            ?? throw new MissingFieldException(type.FullName, name);

    private sealed class SavedState : IDisposable
    {
        private readonly List<(FieldInfo Field, object? Value)> _fields = new();
        private readonly TransferType[] _types = TransferPlanner.TransferTypes.ToArray();

        public SavedState()
        {
            Save(typeof(TransferPlanner), "_showPlanWindow", "_sourceBody", "_transferType", "_transferCalculated",
                "_destinationBody", "_selectedEntry", "_selectedTimeUnit", "_transferInfo", "_selectedMinTime", "_selectedMaxTime");
            Save(typeof(ManeuverToolsWindow), "_defaultsInitialized", "_nodeDefaultInitialized", "_lastTargetParentId");
            Save(typeof(Patch_DrawPlanWindow), "_ourBurn", "_lastEntry", "_lastSource");
        }

        private void Save(Type type, params string[] names)
        {
            foreach (string name in names)
            {
                FieldInfo field = Field(type, name);
                _fields.Add((field, field.GetValue(null)));
            }
        }

        public void Dispose()
        {
            TransferPlanner.TransferTypes.Clear();
            TransferPlanner.TransferTypes.AddRange(_types);
            foreach (var saved in _fields)
                saved.Field.SetValue(null, saved.Value);
        }
    }
}
