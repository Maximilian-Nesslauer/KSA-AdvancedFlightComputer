using HeadlessHarness.Harness;

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
        Execute(t);
        return t.Finish();
    }

    protected abstract void Execute(TestContext t);
}
