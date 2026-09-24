using System.Reflection;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Guidance.Conic;
using AdvancedFlightComputer.Guidance.Gfold;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;

namespace AdvancedFlightComputer.HarnessTests;

// Calls the G-FOLD re-plan with the parameter build, the solver and the flight-time search replaced by stubs, so the propellant each plan needs is chosen here and no native solver runs. After a plan is refused for propellant, the searches rest while the committed plan keeps flying.
public sealed class GuidanceGfoldBackoffTest : AfcTest
{
    public override string Name => "afc-guidance-gfold-backoff";

    private const BindingFlags PrivateStatic = BindingFlags.NonPublic | BindingFlags.Static;
    private const double FuelAboard = 100.0;
    private const double Short = 200.0;
    private const double Fits = 50.0;

    private static double _solveFuel;
    private static double _searchFuel;
    private static int _solves;
    private static int _searches;

    protected override void Execute(TestContext t)
    {
        FieldInfo ambient = typeof(GuidanceWindow).GetField("_s", PrivateStatic)!;
        object? previousAmbient = ambient.GetValue(null);
        double? previousLimit = GfoldPlanner.SolveTimeLimitS;
        double rest = (double)typeof(GuidanceWindow).GetField("GfoldFuelSearchRetryS", PrivateStatic)!.GetRawConstantValue()!;
        var harmony = new Harmony("com.maxi.afc.harnesstests.guidance.gfoldbackoff");
        try
        {
            harmony.Patch(AccessTools.Method(typeof(KsaGfold), "BuildParams"), prefix: Prefix(nameof(StubParams)));
            harmony.Patch(AccessTools.Method(typeof(GfoldPlanner), "Solve"), prefix: Prefix(nameof(StubSolve)));
            harmony.Patch(AccessTools.Method(typeof(GfoldPlanner), nameof(GfoldPlanner.SearchMinFuel)),
                prefix: Prefix(nameof(StubSearch)));

            GfoldTrajectory committed = Plan(10.0);
            var state = new VehicleAutopilotState { GfoldPlan = committed, GfoldArrivalTime = 130.0 };
            _solves = _searches = 0;

            // A committed plan whose re-plan comes back short, from the single solve and from both searches.
            _solveFuel = _searchFuel = Short;
            double now = 100.0;
            Replan(ambient, state, now);
            t.Check("a re-plan short of propellant runs both searches once", _searches == 2, $"searches={_searches}");
            t.Check("the refusal starts the rest", state.GfoldSearchRetryTime == now + rest, $"retry={state.GfoldSearchRetryTime}");
            t.Check("the committed plan keeps flying", ReferenceEquals(state.GfoldPlan, committed));
            t.Check("the refusal is visible", state.LandingStatus.Contains("propellant"), state.LandingStatus);

            int solvesBefore = _solves;
            for (int i = 1; i < 8; i++)
                Replan(ambient, state, now + rest * i / 8.0);
            t.Check("the searches rest after the refusal", _searches == 2, $"searches={_searches}");
            t.Check("the single solve still runs on every cadence", _solves == solvesBefore + 7, $"solves={_solves - solvesBefore}");

            now += rest + 0.25;
            Replan(ambient, state, now);
            t.Check("the searches run again once the rest is over", _searches == 4, $"searches={_searches}");

            _solveFuel = Fits;
            Replan(ambient, state, now + 0.25);
            t.Check("a single solve that fits again is taken during the rest",
                !ReferenceEquals(state.GfoldPlan, committed) && _searches == 4, $"searches={_searches}");
            t.Check("taking a plan ends the rest",
                double.IsNegativeInfinity(state.GfoldSearchRetryTime) && state.GfoldFailStreak == 0);

            // A new refusal, then a retarget inside its rest, through the real retarget path. The retarget's search flag stays set until a plan is committed, so a refused retarget has to rest like any other search.
            _solveFuel = Short;
            now += 0.5;
            Replan(ambient, state, now);
            int afterRefusal = _searches;
            state.LandingPhase = GuidanceWindow.LandingPhase.GfoldDescent;
            ambient.SetValue(null, state);
            Method("RetargetLandingSite").Invoke(null, [0.0, 0.0]);
            t.Check("a retarget ends the rest", double.IsNegativeInfinity(state.GfoldSearchRetryTime));

            Replan(ambient, state, now + 0.25);
            t.Check("a retarget searches straight away during the rest", _searches == afterRefusal + 1,
                $"searches={_searches - afterRefusal}");
            Replan(ambient, state, now + 0.5);
            t.Check("a refused retarget rests like any other search", _searches == afterRefusal + 1,
                $"searches={_searches - afterRefusal}");
            Replan(ambient, state, now + 0.5 + rest);
            t.Check("the retarget searches again after its rest", _searches == afterRefusal + 2,
                $"searches={_searches - afterRefusal}");

            state.GfoldPlan = null;
            Replan(ambient, state, now + 0.75 + rest);
            t.Check("a craft with no plan searches during the rest", _searches == afterRefusal + 3,
                $"searches={_searches - afterRefusal}");
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            GfoldPlanner.SolveTimeLimitS = previousLimit;
            ambient.SetValue(null, previousAmbient);
        }
    }

    private static void Replan(FieldInfo ambient, VehicleAutopilotState state, double now)
    {
        ambient.SetValue(null, state);
        Method("SolveGfoldPlan").Invoke(null, [null, null, null, null, null, now]);
    }

    private static MethodInfo Method(string name) =>
        typeof(GuidanceWindow).GetMethod(name, PrivateStatic)
        ?? throw new MissingMethodException(nameof(GuidanceWindow), name);

    private static HarmonyMethod Prefix(string name) => new(typeof(GuidanceGfoldBackoffTest), name);

    // Two nodes are enough for FuelUsed, which is the first mass minus the last.
    private static GfoldTrajectory Plan(double fuel) => new()
    {
        Status = ConicStatus.Optimal,
        Dt = 1.0,
        Position = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        Velocity = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        AccelCmd = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        Sigma = [0.0, 0.0],
        Mass = [1000.0, 1000.0 - fuel],
        LandingPoint = [0.0, 0.0, 0.0],
        LandingErrorNorm = 0.0,
        Iterations = 1,
    };

    private static bool StubParams(ref GfoldParams __result, ref string refusal)
    {
        __result = new GfoldParams { FuelMass = FuelAboard };
        refusal = "";
        return false;
    }

    private static bool StubSolve(ref GfoldTrajectory __result)
    {
        _solves++;
        __result = Plan(_solveFuel);
        return false;
    }

    private static bool StubSearch(ref GfoldPlanner.SearchResult? __result)
    {
        _searches++;
        __result = new GfoldPlanner.SearchResult(Plan(_searchFuel), 20.0, _searchFuel, 1);
        return false;
    }
}
