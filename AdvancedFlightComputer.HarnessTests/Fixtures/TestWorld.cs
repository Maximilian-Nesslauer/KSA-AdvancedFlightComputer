using AdvancedFlightComputer.HarnessTests.Framework;
using KSA;

namespace AdvancedFlightComputer.HarnessTests.Fixtures;

// Nothing here names a body: tests assert against whatever the loaded system provides, so the suite
// still runs on a modded star system.
public static class TestWorld
{
    // A system without one is a failure, not a skip: there is nothing left to run against.
    public static bool RequireHome(TestContext t, out IParentBody home)
    {
        if (t.System.HomeBody is IParentBody resolved)
        {
            home = resolved;
            return true;
        }
        home = null!;
        t.Fail("home body", "the loaded system has no home body");
        return false;
    }

    // First child with a usable sphere of influence, for the tests that need a second gravity well.
    public static Celestial? FindMoon(IParentBody home)
    {
        foreach (IOrbiter child in home.Children)
        {
            if (child is Celestial moon && moon is IParentBody body
                && body.SphereOfInfluence > 0.0 && !double.IsNaN(body.SphereOfInfluence))
                return moon;
        }
        return null;
    }
}
