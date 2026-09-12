using StarMap.API;

namespace AdvancedFlightComputer.HarnessTests;

// Marks the assembly as a StarMap mod so the deployed folder is a valid mod install. No lifecycle
// hooks: the DLL only carries IHarnessTest classes, which HeadlessHarness loads and runs itself
// during a headless run; on a normal (GPU) launch this mod does nothing.
[StarMapMod]
public sealed class TestMod
{
}
