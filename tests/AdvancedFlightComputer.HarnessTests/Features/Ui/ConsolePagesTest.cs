using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.Ui.Console;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.HarnessTests.Framework;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The pages of the AFC console, without a draw because the harness has no ImGui context. Every page
// opens through the one entry that the AFC menu, the planner key and the helpers share, and is saved
// to ui.toml and read back. The cue targets of the helpers and the head reach their sub-pages, an
// unknown cue changes nothing, and the console opens nothing while it is off, as during an unload.
// The stock planner keeps only its own plan types.
public sealed class ConsolePagesTest : AfcTest
{
    public override string Name => "afc-console-pages";

    protected override void Execute(TestContext t)
    {
        bool open = AfcConsole.IsOpen;
        bool enabled = AfcConsole.Enabled;
        bool drawer = AfcConsole.DrawerOpen;
        bool route = PlannerRoute.Active;
        ConsolePage page = AfcConsole.Page;
        LandWay way = AfcConsole.LandWay;
        try
        {
            AfcConsole.Enabled = true;
            foreach (ConsolePage p in ConsolePages.All)
            {
                AfcConsole.IsOpen = false;
                AfcConsole.Open(p);
                t.Check($"{ConsolePages.Label(p)} opens and is saved", AfcConsole.IsOpen && AfcConsole.Page == p
                    && UiConfig.GetString(AfcConsole.KeyPage, "") == ConsolePages.Key(p));
            }

            AfcConsole.Open(ConsolePage.Orbit);
            AfcConsole.Page = ConsolePage.Launch;
            AfcConsole.LoadPreferences();
            t.Check("the saved page is read back", AfcConsole.Page == ConsolePage.Orbit);

            AfcConsole.LandWay = LandWay.FromOrbit;
            AfcConsole.Navigate(ConsolePages.CueTarget(ConsolePage.Land, ConsolePages.EnumKey(LandWay.Hover)));
            t.Check("a LAND/HOVER cue opens the hover", AfcConsole.Page == ConsolePage.Land && AfcConsole.LandWay == LandWay.Hover);

            AfcConsole.LandWay = LandWay.FromOrbit;
            AfcConsole.Navigate(ConsolePages.CueTarget(ConsolePage.Land, ConsolePages.EnumKey(LandWay.Boostback)));
            t.Check("a LAND/BOOSTBACK cue opens the boostback",
                AfcConsole.Page == ConsolePage.Land && AfcConsole.LandWay == LandWay.Boostback);

            AfcConsole.LandWay = LandWay.FromOrbit;
            AfcConsole.Navigate(ConsolePages.RecoverAlias + "/AFTERSEPARATION");
            t.Check("an old booster cue after separation opens LAND > BOOSTBACK",
                AfcConsole.Page == ConsolePage.Land && AfcConsole.LandWay == LandWay.Boostback);

            AfcConsole.Navigate(ConsolePages.RecoverAlias + "/BEFORELAUNCH");
            t.Check("an old booster cue before launch opens LAUNCH", AfcConsole.Page == ConsolePage.Launch);

            AfcConsole.DrawerOpen = true;
            AfcConsole.Navigate(ConsolePages.CueTarget(ConsolePage.Staging));
            t.Check("a page cue closes the drawer, which would hide that page", !AfcConsole.DrawerOpen && AfcConsole.Page == ConsolePage.Staging);

            AfcConsole.Navigate(ConsolePages.CueTarget(ConsolePage.Land, ConsolePages.EnumKey(LandWay.Boostback)));
            AfcConsole.Navigate("NO SUCH PAGE");
            t.Check("an unknown cue changes nothing", AfcConsole.Page == ConsolePage.Land && AfcConsole.LandWay == LandWay.Boostback);

            AfcConsole.IsOpen = false;
            AfcConsole.Enabled = false;
            AfcConsole.Open(ConsolePage.Tune);
            t.Check("the console opens nothing while it is off", !AfcConsole.IsOpen && AfcConsole.Page == ConsolePage.Land);
            AfcConsole.Enabled = true;

            string[] afcKeys = [ManeuverTools.KeySetPeriapsis, ManeuverTools.KeySetApoapsis,
                ManeuverTools.KeyMatchInclination, ManeuverTools.KeySetInclination];
            bool anyAfcType = TransferPlanner.TransferTypes.Exists(type => Array.IndexOf(afcKeys, type.GetKey()) >= 0);
            t.Check("the stock planner lists only its own plan types", !anyAfcType,
                string.Join(", ", TransferPlanner.TransferTypes.ConvertAll(type => type.GetKey())));
        }
        finally
        {
            AfcConsole.Enabled = enabled;
            AfcConsole.IsOpen = open;
            AfcConsole.DrawerOpen = drawer;
            AfcConsole.Page = page;
            AfcConsole.LandWay = way;
            AfcConsole.SavePreferences();
            // LoadPreferences reads the routing switch again, so the state from before the test is put back.
            PlannerRoute.Active = route;
        }
    }
}
