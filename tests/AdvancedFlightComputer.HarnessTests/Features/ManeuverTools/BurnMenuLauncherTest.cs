using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Ui.Console;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.HarnessTests.Framework;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Every orbit right-click entry opens the AFC console on ORBIT with its tool and craft and resets the
// inputs of that tool, and the plan state of the ORBIT page lives only while the page is on screen.
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

        using var state = new SavedState();
        string[] keys = [ManeuverTools.KeySetApoapsis, ManeuverTools.KeySetPeriapsis,
            ManeuverTools.KeyMatchInclination, ManeuverTools.KeySetInclination,
            ManeuverTools.KeyCircularizeApoapsis, ManeuverTools.KeyCircularizePeriapsis];
        foreach (string key in keys)
            CheckShortcut(t, key, source);
        CheckPageClose(t, source);
    }

    private static void CheckShortcut(TestContext t, string key, Vehicle source)
    {
        Field(typeof(OrbitToolForm), "_defaultsInitialized").SetValue(null, true);
        Field(typeof(OrbitToolForm), "_nodeDefaultInitialized").SetValue(null, true);
        AfcConsole.IsOpen = false;
        AfcConsole.Page = ConsolePage.Tune;

        BurnMenuLauncher.OpenOrbitTool(key, source);
        t.Check(key + " opens the console on ORBIT with the tool",
            AfcConsole.IsOpen && AfcConsole.Page == ConsolePage.Orbit
            && OrbitToolsPanel.ToolKey == key && ReferenceEquals(OrbitToolsPanel.Source, source),
            $"open {AfcConsole.IsOpen}, page {AfcConsole.Page}, tool {OrbitToolsPanel.ToolKey}");
        t.Check(key + " resets input defaults",
            !(bool)Field(typeof(OrbitToolForm), "_defaultsInitialized").GetValue(null)!
            && !(bool)Field(typeof(OrbitToolForm), "_nodeDefaultInitialized").GetValue(null)!);
    }

    // The plan state lives only while the ORBIT page is on screen.
    private static void CheckPageClose(TestContext t, Vehicle source)
    {
        FieldInfo lastSource = Field(typeof(OrbitToolsPanel), "_lastSource");
        FieldInfo lastEntry = Field(typeof(OrbitToolsPanel), "_lastEntry");
        var entry = new OrbitalTransfers.PorkChopEntry(new OrbitalTransfers.TransferData(), FlightPlan.CreateUninitialized(source.Hash));
        lastSource.SetValue(null, source);
        lastEntry.SetValue(null, entry);
        OrbitToolsPanel.PreviewVisible = true;
        OrbitToolsPanel.Tick();
        t.Check("a shown page retains plan state",
            ReferenceEquals(lastSource.GetValue(null), source) && ReferenceEquals(lastEntry.GetValue(null), entry));

        OrbitToolsPanel.PreviewVisible = false;
        // The console frame ticks the panel every frame, page or not, so a hidden page drops its plan state.
        OrbitToolsPanel.Tick();
        t.Check("a hidden page clears its plan state",
            lastSource.GetValue(null) == null && lastEntry.GetValue(null) == null);
    }

    private static FieldInfo Field(Type type, string name)
        => type.GetField(name, BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic)
            ?? throw new MissingFieldException(type.FullName, name);

    private sealed class SavedState : IDisposable
    {
        private readonly List<(FieldInfo Field, object? Value)> _fields = new();
        private readonly bool _enabled = AfcConsole.Enabled;
        private readonly bool _open = AfcConsole.IsOpen;
        private readonly ConsolePage _page = AfcConsole.Page;
        private readonly string _toolKey = OrbitToolsPanel.ToolKey;
        private readonly Vehicle? _source = OrbitToolsPanel.Source;
        private readonly bool _previewVisible = OrbitToolsPanel.PreviewVisible;

        public SavedState()
        {
            AfcConsole.Enabled = true;
            Save(typeof(OrbitToolForm), "_defaultsInitialized", "_nodeDefaultInitialized", "_lastTargetParentId");
            Save(typeof(OrbitToolsPanel), "_ourBurn", "_lastEntry", "_lastSource");
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
            foreach (var saved in _fields)
                saved.Field.SetValue(null, saved.Value);
            OrbitToolsPanel.Select(_toolKey, _source);
            OrbitToolsPanel.PreviewVisible = _previewVisible;
            AfcConsole.Enabled = _enabled;
            AfcConsole.IsOpen = _open;
            AfcConsole.Page = _page;
            // Opening the console saved its page to ui.toml, so the saved page is put back too.
            AfcConsole.SavePreferences();
        }
    }
}
