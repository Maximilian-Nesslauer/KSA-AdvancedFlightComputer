using AdvancedFlightComputer.Features.Guidance;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests.Framework;

// Base for every test here. Exceptions are deliberately not caught: HarnessRunner classifies a
// MissingMemberException or TypeLoadException escaping a test as game-API drift, an infrastructure
// failure, and catching it would downgrade that to an ordinary FAIL.
public abstract class AfcTest : IHarnessTest
{
    // Also the KSA_HEADLESS_TESTS filter key, so a rename breaks existing invocations.
    public abstract string Name { get; }

    public virtual bool OptIn => false;

    public int Run(HeadlessSession session)
    {
        TestContext t = new TestContext(Name, session);
        // The harness steps the simulation far faster than real time, so the guidance gates read the
        // simulation time and advance as they do at 1x in game, not with how fast this machine runs.
        GuidanceClock.Source = static () => (long)(Universe.GetElapsedSeconds() * 1000.0);
        try
        {
            Execute(t);
        }
        finally
        {
            GuidanceClock.Reset();
        }
        return t.Finish();
    }

    protected abstract void Execute(TestContext t);
}
