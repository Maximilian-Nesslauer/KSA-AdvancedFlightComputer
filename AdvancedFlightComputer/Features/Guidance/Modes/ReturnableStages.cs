#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Numerics;

// Which stages can return under guidance, and their estimated return cost.
//
// The parts that leave together need able crew in a command zone or a usable probe core.
//
// Why the Jacobian is computed once for every stage.
// ImpactSteering.Correction needs an impact prediction and velocity Jacobian that depend on the vehicle state, not the landing target.
// Only the miss depends on the target.
// So one prediction and one Jacobian serve every stage in the list, and each extra target is a 3x3 solve.
// Listing five stages costs what listing one does.
//
// The value estimates the impulse that would return a stage if it separated now.
// It is exact only for the next separation and remains hypothetical for stages below it.
// The estimate has two important approximations:
//
// - The coast uses the current stack's aerodynamic surrogate because an attached subtree has no separate bounding box.
// - A separated booster has a smaller CdA, so this value is a gauge rather than a flight plan.
//
// - The estimate is impulsive, while a real return burn takes time and costs more.
// - BoostbackShooter plans the actual burn after separation.
// - This value says whether to stage yet, not how to fly the burn.
public static partial class GuidanceWindow
{
    /// <summary>How often the returnable-stage costs are re-solved, ms.</summary>
    private const long ReturnDvIntervalMs = 1000;

    /// <summary>
    /// One separable stage with usable control, and what returning would cost it.
    /// </summary>
    public sealed class ReturnableStage
    {
        /// <summary>The detached subtree's root part. Its InstanceId is the stage's
        /// identity - stable for the life of the craft, which is what lets a target set
        /// on the pad still be attached to the right stage at separation.</summary>
        public uint RootId;
        internal string EngineGroupScope = "";

        /// <summary>The root part's display name, for the list.</summary>
        public string Name = "";

        /// <summary>Which staging sequence separates it.</summary>
        public int SequenceIndex;

        /// <summary>The stock Sequence.Number of that sequence, the number the stock staging list shows. It is not the list index.</summary>
        public int SequenceNumber;

        /// <summary>True when this is the NEXT thing to separate - the one the number
        /// below is actually about rather than hypothetical.</summary>
        public bool IsNext;

        /// <summary>A landing site set by the player (VehicleAutopilotState.StageTargets), at <see cref="TargetLatDeg"/>, <see cref="TargetLonDeg"/>.</summary>
        public bool HasTarget;
        public double TargetLatDeg, TargetLonDeg;

        /// <summary>Set to optimal return (a drone ship, VehicleAutopilotState.StageOptimalReturn): it comes home with no site set, to where its coast from the plan's separation lands.</summary>
        public bool OptimalReturn;

        /// <summary>It is brought home, to a site or by an optimal return: the plan keeps a reserve for it and it is handed over at separation. Otherwise it is expendable.</summary>
        public bool ComesHome => HasTarget || OptimalReturn;

        /// <summary>How it comes home, as the plan's return paths have it.</summary>
        public AdvancedFlightComputer.Guidance.Scvx.Ascent.BoosterReturnMode Mode
            => HasTarget ? AdvancedFlightComputer.Guidance.Scvx.Ascent.BoosterReturnMode.Site
             : OptimalReturn ? AdvancedFlightComputer.Guidance.Scvx.Ascent.BoosterReturnMode.OptimalReturn
             : AdvancedFlightComputer.Guidance.Scvx.Ascent.BoosterReturnMode.Expendable;

        /// <summary>Impulse to put the ballistic impact on this stage's site from the
        /// current state, m/s. NaN when it has no target or nothing landed.</summary>
        public double RequiredDvMs = double.NaN;

        /// <summary>Great-circle miss between where the vehicle would come down now and
        /// this stage's site, m. The distance the dV above is buying.</summary>
        public double MissM = double.NaN;
    }

    /// <summary>
    /// Rebuild the list of stages that could fly themselves home.
    ///
    /// Rides the stage-model tick so changes in crew or probe power reach the return list.
    /// </summary>
    private static void RefreshReturnableStages(Vehicle vehicle)
    {
        // REUSED, NOT REBUILT.
        // This runs at 4 Hz and the costs are solved at 1 Hz, so handing out fresh objects would blank three readings in four and make a live number look like it was failing.
        // Reuse also means the 4 Hz tick allocates nothing on the sim path after the first pass.
        List<ReturnableStage> list = _s.ReturnableStages;
        Dictionary<uint, ReturnableStage> cache = _s.ReturnableStageCache;
        list.Clear();
        _s.ReturnControlRefusals.Clear();

        PartTree tree = vehicle?.Parts;
        SequenceList sequenceList = tree?.SequenceList;
        if (tree == null || sequenceList == null)
            return;

        Span<Control> controls = tree.Modules.Get<Control>();
        if (controls.Length == 0)
            return;      // nothing aboard is controllable, so nothing separating is

        Span<EngineController> engines = tree.Modules.Get<EngineController>();
        _returnDropsSoFar.Clear();
        _returnExcludedRoots.Clear();
        bool first = true;
        ReadOnlySpan<Sequence> sequences = sequenceList.Sequences;
        for (int i = 0; i < sequences.Length; i++)
        {
            if (sequences[i].Activated || sequences[i].Parts.IsEmpty)
                continue;

            SequenceRoots(sequences[i], _returnRoots);
            if (_returnRoots.Count == 0)
                continue;   // an ignition-only sequence: it separates nothing
            foreach (Part root in _returnRoots)
            {
                CollectSubtree(root, _returnDropsSoFar);
                if (!_returnExcludedRoots.Contains(root))
                    _returnExcludedRoots.Add(root);
            }

            // NOT ONE THAT REACHES ORBIT. A stage that separates once the climb is over - an upper stage leaving its payload, a capsule - is already going orbital speed, and no boostback brings that home, command pod or not. The climb is over when nothing left aboard lights at this sequence or later: the booster's own separation lights the stage above it, and a strap-on's leaves the core and the upper stage still to fire. Hot staging, an upper stage lit a sequence before the booster leaves, would read as orbital here.
            bool climbGoesOn = LightsLater(engines, i);

            // ONE ENTRY PER SEPARATED ROOT, not per sequence (#130). Two side boosters leaving on one sequence are two craft, each needing its own target. Each root is judged on the part of its subtree it takes with it: a root nested inside another's subtree on the same row leaves as a craft of its own.
            for (int r = 0; r < _returnRoots.Count; r++)
            {
                Part root = _returnRoots[r];
                if (!climbGoesOn)
                    continue;
                _returnDrops.Clear();
                CollectOwnSubtree(root, root, _returnExcludedRoots, _returnDrops);
                string name = string.IsNullOrEmpty(root.DisplayName) ? "stage" : root.DisplayName;
                VehicleControl? control = StageControl.Evaluate(vehicle, _returnDrops);
                if (control != VehicleControl.Controllable)
                {
                    if (control.HasValue)
                        _s.ReturnControlRefusals.Add((root.InstanceId, name, sequences[i].Number, control.Value));
                    continue;
                }

                uint rootId = root.InstanceId;
                if (!cache.TryGetValue(rootId, out ReturnableStage stage))
                {
                    stage = new ReturnableStage { RootId = rootId };
                    cache[rootId] = stage;
                }
                stage.Name = name;
                stage.EngineGroupScope = LandingEngineGroups.PartAddress(root, tree.Root) ?? "";
                stage.SequenceIndex = i;
                stage.SequenceNumber = sequences[i].Number;
                // Every craft the first separating row drops is next, and none of a later row's: a dumb interstage coming off first does not make the booster behind it next.
                stage.IsNext = first;
                stage.HasTarget = false;
                stage.OptimalReturn = false;
                if (_s.StageTargets.TryGetValue(stage.RootId, out double2 site))
                {
                    stage.HasTarget = true;
                    stage.TargetLatDeg = site.X;
                    stage.TargetLonDeg = site.Y;
                }
                else if (_s.StageOptimalReturn.Contains(stage.RootId))
                {
                    stage.OptimalReturn = true;
                    // The drone ship's point once a plan has found it; NaN until then.
                    bool known = _s.OptimalReturnSites.TryGetValue(stage.RootId, out double2 ship);
                    stage.TargetLatDeg = known ? ship.X : double.NaN;
                    stage.TargetLonDeg = known ? ship.Y : double.NaN;
                }
                list.Add(stage);
            }
            first = false;
        }
        _returnDrops.Clear();
        _returnRoots.Clear();
        _returnExcludedRoots.Clear();
        _returnDropsSoFar.Clear();
    }

    /// <summary>The distinct roots of the subtrees that <paramref name="sequence"/> separates, in the order its decouplers come; empty for a row that separates nothing.</summary>
    private static void SequenceRoots(Sequence sequence, List<Part> roots)
    {
        roots.Clear();
        ReadOnlySpan<Part> parts = sequence.Parts;
        for (int j = 0; j < parts.Length; j++)
        {
            foreach (ISequenced module in parts[j].InSequence(sequence.Number))
            {
                if (module is not Decoupler decoupler || !StagingHooks.CanActivate(decoupler))
                    continue;
                Part root = DetachedRoot(decoupler);
                if (root != null && !roots.Contains(root))
                    roots.Add(root);
            }
        }
    }

    /// <summary>
    /// The parts that leave with the booster rooted at <paramref name="root"/> when its row fires (sequence list index <paramref name="sequenceIndex"/>): its subtree, short of every subtree a row up to and including its own separates from it first or beside it. Side boosters hang off the core in the part tree; by the time the core leaves they have gone.
    /// </summary>
    internal static HashSet<Part> PartsLeavingWith(Vehicle vehicle, Part root, int sequenceIndex)
    {
        var others = new List<Part>();
        var rowRoots = new List<Part>();
        ReadOnlySpan<Sequence> sequences = vehicle.Parts.SequenceList.Sequences;
        for (int i = 0; i <= sequenceIndex && i < sequences.Length; i++)
        {
            if (sequences[i].Activated || sequences[i].Parts.IsEmpty)
                continue;
            SequenceRoots(sequences[i], rowRoots);
            foreach (Part r in rowRoots)
                if (r != root && !others.Contains(r))
                    others.Add(r);
        }
        var parts = new HashSet<Part>();
        CollectOwnSubtree(root, root, others, parts);
        return parts;
    }

    private static HashSet<Part> PartsLeavingOnActivation(Vehicle vehicle, Part root, Decoupler decoupler)
    {
        var roots = new List<Part>();
        // A delayed row is already activated. Its pending modules are the live batch.
        if (StagingDetector.PendingDecouplersFor(vehicle, decoupler) is { } pending)
        {
            foreach (ISequenced module in pending)
                if (module is Decoupler joint && ReferenceEquals(joint.Parent.FullPart.Tree, vehicle.Parts)
                    && StagingHooks.CanActivate(joint) && DetachedRoot(joint) is Part departing)
                    roots.Add(departing);
        }
        else
        {
            foreach (Sequence row in vehicle.Parts.SequenceList.Sequences)
            {
                if (row.Number != decoupler.Sequence)
                    continue;
                SequenceRoots(row, roots);
                break;
            }
        }

        var parts = new HashSet<Part>();
        CollectOwnSubtree(root, root, roots, parts);
        return parts;
    }

    /// <summary>The parts that leave with <paramref name="root"/> itself: its subtree, short of any other separated root's, which leaves as a craft of its own.</summary>
    private static void CollectOwnSubtree(Part node, Part root, List<Part> roots, HashSet<Part> into)
    {
        if (node != root && roots.Contains(node))
            return;
        if (!into.Add(node))
            return;
        foreach (Part child in node.TreeChildren)
            CollectOwnSubtree(child, root, roots, into);
    }

    // Scratch for the subtree walk.
    // Same contract as _stagingDropped: filled and consumed inside one call, never read across calls.
    private static readonly HashSet<Part> _returnDrops = new HashSet<Part>();
    // The roots one sequence separates.
    private static readonly List<Part> _returnRoots = new List<Part>();
    // Use the same exclusions as PartsLeavingWith for the current and earlier rows.
    private static readonly List<Part> _returnExcludedRoots = new List<Part>();
    // Everything the separations so far in the walk drop, this one included.
    private static readonly HashSet<Part> _returnDropsSoFar = new HashSet<Part>();

    /// <summary>True when an engine still aboard after the separations so far (<see cref="_returnDropsSoFar"/>) is lit by sequence <paramref name="sequence"/> or a later one: the climb goes on after that separation.</summary>
    private static bool LightsLater(Span<EngineController> engines, int sequence)
    {
        for (int e = 0; e < engines.Length; e++)
            if (engines[e].Sequence >= sequence && !_returnDropsSoFar.Contains(engines[e].Parent.FullPart))
                return true;
        return false;
    }

    /// <summary>
    /// Re-solve what it would cost each listed stage to come home from here.
    ///
    /// ONE prediction and ONE Jacobian for the whole list - see the header. Throttled to
    /// 1 Hz because a Jacobian is three seeded RK4 sweeps and this runs through an
    /// ascent, and because the answer moves on the timescale of the trajectory rather
    /// than the frame rate.
    /// </summary>
    private static void UpdateReturnDv(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        List<ReturnableStage> list = _s.ReturnableStages;
        if (list.Count == 0)
            return;

        long now = GuidanceClock.NowMs;
        if (now - _s.ReturnDvTick < ReturnDvIntervalMs)
            return;
        _s.ReturnDvTick = now;

        for (int i = 0; i < list.Count; i++)
        {
            list[i].RequiredDvMs = double.NaN;
            list[i].MissM = double.NaN;
        }

        KsaAeroSweep.Result aero = _s.Aero;
        double mass = vehicle.TotalMass;
        if (aero?.Table == null || !(mass > 0.0))
        {
            _s.ReturnDvNote = _s.AeroError.Length > 0 ? _s.AeroError : "NO AERO SURROGATE";
            return;
        }

        var sys = new DragCoastSystem
        {
            Mu = parent.Mu,
            OmegaZ = parent.GetAngularVelocity(),
            MeanRadius = parent.MeanRadius,
            AreaOverMass = aero.ReferenceArea / mass,
            Alpha = GlideNominalAlpha(aero),
            Table = aero.Table,
            LiftTable = aero.Lift,
            Atmosphere = aero.Atmosphere,
        };

        var opt = ImpactOptions.Default(parent.MeanRadius);
        opt.MaxTime = ImpactHorizonMinutes * 60.0;
        opt.PathStride = 0;

        double3 r0 = orbit.StateVectors.PositionCci;
        double3 v0 = orbit.StateVectors.VelocityCci;
        Span<double> x0 = stackalloc double[ImpactPredictor.N]
            { r0.X, r0.Y, r0.Z, v0.X, v0.Y, v0.Z };

        _s.ReturnScratch ??= new Dual[ImpactPredictor.ScratchLength];

        Span<double> dG = stackalloc double[9];
        ImpactPrediction nom = ImpactPredictor.VelocityJacobian(
            sys, x0, opt, _s.ReturnScratch, dG, default, default);
        if (!nom.Hit)
        {
            // Normal, not a fault: a vehicle still climbing has no ballistic impact inside the horizon, so there is nothing yet to drag onto a site.
            _s.ReturnDvNote = nom.Status == ImpactStatus.NoImpactWithinHorizon
                ? "NO BALLISTIC IMPACT WITHIN " + Ui.Logic.Fmt.Time(ImpactHorizonMinutes * 60.0)
                : nom.Status.ToString();
            return;
        }
        _s.ReturnDvNote = "";

        double3 hitF = new double3(nom.Fx.V, nom.Fy.V, nom.Fz.V);
        double hitLen = hitF.Length();
        if (!(hitLen > 0.0))
            return;
        double3 nrm = hitF / hitLen;

        Span<double> nrmSpan = stackalloc double[3] { nrm.X, nrm.Y, nrm.Z };
        Span<double> missSpan = stackalloc double[3];
        Span<double> dv = stackalloc double[3];
        Span<double> greedy = stackalloc double[3];

        // MEAN RADIUS ON BOTH SIDES, deliberately.
        // The prediction above terminates at MeanRadius, so placing the sites at MeanRadius + terrain would put a terrain height into the miss that is an artefact of mixing two surfaces rather than a real distance.
        // Sampling terrain per site would not fix it either: one prediction serves every stage, so its termination radius has to be common.
        // Self-consistent is what this number needs; the boostback overlay is where the low-passed terrain height belongs.
        double siteRadius = parent.MeanRadius;

        for (int i = 0; i < list.Count; i++)
        {
            ReturnableStage stage = list[i];
            if (!stage.HasTarget)
                continue;

            // The site in the frame the Jacobian's rows are written in - the co-rotating one.
            // Same conversion UpdateSteering makes, and getting it backwards would report a cost that is right in size and wrong by the body's rotation.
            double3 siteCcf = SiteDirCcfAt(stage.TargetLatDeg, stage.TargetLonDeg) * siteRadius;
            double3 siteF = siteCcf.Transform(parent.GetCcf2Cci());
            double3 missF = hitF - siteF;

            missSpan[0] = missF.X; missSpan[1] = missF.Y; missSpan[2] = missF.Z;
            if (!ImpactSteering.Correction(dG, missSpan, nrmSpan, ImpactSteering.DefaultLambda,
                                           dv, greedy))
                continue;

            stage.RequiredDvMs = Math.Sqrt(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]);

            // Great-circle rather than the chord, so it is the number a map would give.
            double cosang = Math.Clamp(double3.Dot(nrm, double3.Normalize(siteF)), -1.0, 1.0);
            stage.MissM = Math.Acos(cosang) * parent.MeanRadius;
        }
    }

    /// <summary>True when at least one returnable stage has somewhere to go. Nothing is
    /// solved, and no aero surrogate is fitted, until one does.</summary>
    private static bool AnyStageTargeted()
    {
        List<ReturnableStage> list = _s.ReturnableStages;
        for (int i = 0; i < list.Count; i++)
            if (list[i].HasTarget)
                return true;
        return false;
    }

    /// <summary>A site's body-fixed direction, for a lat/lon that is not the vehicle's
    /// own. KSA's convention, the same one SiteDirCcf uses.</summary>
    private static double3 SiteDirCcfAt(double latDeg, double lonDeg)
    {
        double lat = latDeg * Math.PI / 180.0;
        double lon = lonDeg * Math.PI / 180.0;
        return new double3(
            Math.Cos(lat) * Math.Cos(lon),
            Math.Cos(lat) * Math.Sin(lon),
            Math.Sin(lat));
    }

}
