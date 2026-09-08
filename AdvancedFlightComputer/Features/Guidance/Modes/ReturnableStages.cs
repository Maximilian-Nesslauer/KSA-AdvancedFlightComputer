using System;
using System.Collections.Generic;
using Brutal.Numerics;
using KSA;
using PoweredGuidance.Flight;
using PoweredGuidance.Numerics;

// WHICH STAGES CAN FLY THEMSELVES HOME, AND WHAT IT WOULD COST THEM.
//
// A stage is worth returning if it can be flown, and it can be flown if it has a
// command pod — Vehicle.IsControllable is Parts.Controls.NumModules > 0, and every
// attitude profile in the flight computer gates on it. So the test is not "is this a
// big expensive-looking booster" but the same one the game applies: does the subtree
// that separates carry a Control module. An interstage does not. A booster with
// avionics does.
//
// WHY THE JACOBIAN IS COMPUTED ONCE FOR ALL OF THEM. The cost of coming back is
// ImpactSteering.Correction — the impulse that drags the ballistic impact point onto a
// site — and that decomposes cleanly: the impact prediction and its velocity Jacobian
// depend only on where the VEHICLE is and how it flies, not on where anyone wants to
// land. Only the miss depends on the target. So one prediction and one Jacobian serve
// every stage in the list, and each extra target is a 3x3 solve. Listing five stages
// costs what listing one does.
//
// WHAT THE NUMBER MEANS, AND WHAT IT DOES NOT. It is "if this separated NOW, what
// impulse puts it on its site" — exactly the question for the next separation, and a
// hypothetical for anything further down the stack, which has no business separating
// here. Two approximations are worth naming rather than burying:
//
//   THE BALLISTIC COEFFICIENT IS THE STACK'S. The coast is integrated with the aero
//   surrogate fitted to the vehicle as it is now and its current mass, because a
//   subtree that has not separated has no bounding box of its own to sweep — KSA
//   computes CdA from the live assembly. A booster alone is shorter, so its CdA is
//   smaller and its true return cost differs. The nose area is much the same, and drag
//   moves this number by single-digit percent on a boostback-shaped arc (--impact
//   prices it), so it is a good gauge and not a plan.
//
//   IT IS AN IMPULSE. A real return burn lasts tens of seconds and costs more — the
//   shooter measures 18% more than the impulsive figure on the reference arc, and the
//   impulsive answer points at the ground besides. BoostbackShooter is what plans the
//   actual burn once the booster exists. This is the number that says whether to stage
//   yet, not the number that flies it.
public static partial class PoweredGuidanceWindow
{
    /// <summary>How often the returnable-stage costs are re-solved, ms.</summary>
    private const long ReturnDvIntervalMs = 1000;

    /// <summary>
    /// One separable stage that carries a command pod, and what returning would cost it.
    /// </summary>
    public sealed class ReturnableStage
    {
        /// <summary>The detached subtree's root part. Its InstanceId is the stage's
        /// identity — stable for the life of the craft, which is what lets a target set
        /// on the pad still be attached to the right stage at separation.</summary>
        public uint RootId;

        /// <summary>The root part's display name, for the list.</summary>
        public string Name = "";

        /// <summary>Which staging sequence separates it.</summary>
        public int SequenceIndex;

        /// <summary>True when this is the NEXT thing to separate — the one the number
        /// below is actually about rather than hypothetical.</summary>
        public bool IsNext;

        public bool HasTarget;
        public double TargetLatDeg, TargetLonDeg;

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
    /// Rides the stage-model tick: its input is the part tree, which only changes when
    /// staging does, and StageModelDirty already marks exactly that.
    /// </summary>
    private static void RefreshReturnableStages(Vehicle vehicle)
    {
        // REUSED, NOT REBUILT. This runs at 4 Hz and the costs are solved at 1 Hz, so
        // handing out fresh objects would blank three readings in four and make a live
        // number look like it was failing. Reuse also means the 4 Hz tick allocates
        // nothing on the sim path after the first pass.
        List<ReturnableStage> list = _s.ReturnableStages;
        Dictionary<uint, ReturnableStage> cache = _s.ReturnableStageCache;
        list.Clear();

        PartTree tree = vehicle?.Parts;
        SequenceList sequenceList = tree?.SequenceList;
        if (tree == null || sequenceList == null)
            return;

        Span<Control> controls = tree.Modules.Get<Control>();
        if (controls.Length == 0)
            return;      // nothing aboard is controllable, so nothing separating is

        bool first = true;
        ReadOnlySpan<Sequence> sequences = sequenceList.Sequences;
        for (int i = 0; i < sequences.Length; i++)
        {
            if (sequences[i].Activated || sequences[i].Parts.IsEmpty)
                continue;

            _returnDrops.Clear();
            ReadOnlySpan<Part> parts = sequences[i].Parts;
            Part detached = null;
            for (int j = 0; j < parts.Length; j++)
            {
                Span<Decoupler> decouplers = parts[j].SubtreeModules.Get<Decoupler>();
                for (int k = 0; k < decouplers.Length; k++)
                {
                    Part root = DetachedRoot(decouplers[k]);
                    if (root == null)
                        continue;
                    detached ??= root;
                    CollectSubtree(root, _returnDrops);
                }
            }
            if (detached == null || _returnDrops.Count == 0)
                continue;   // an ignition-only sequence: it separates nothing

            // THE TEST. A command pod on the side that leaves is what makes the thing
            // that leaves a vehicle rather than debris.
            bool controllable = false;
            for (int c = 0; c < controls.Length && !controllable; c++)
                controllable = _returnDrops.Contains(controls[c].Parent.FullPart);
            if (!controllable)
            {
                // Still a separation, so the NEXT flag has to move past it: a dumb
                // interstage coming off first does not make the booster behind it next.
                first = false;
                continue;
            }

            uint rootId = detached.InstanceId;
            if (!cache.TryGetValue(rootId, out ReturnableStage stage))
            {
                stage = new ReturnableStage { RootId = rootId };
                cache[rootId] = stage;
            }
            stage.Name = string.IsNullOrEmpty(detached.DisplayName) ? "stage" : detached.DisplayName;
            stage.SequenceIndex = i;
            stage.IsNext = first;
            stage.HasTarget = false;
            if (_s.StageTargets.TryGetValue(stage.RootId, out double2 site))
            {
                stage.HasTarget = true;
                stage.TargetLatDeg = site.X;
                stage.TargetLonDeg = site.Y;
            }
            list.Add(stage);
            first = false;
        }
    }

    // Scratch for the subtree walk. Same contract as _stagingDropped: filled and
    // consumed inside one call, never read across calls.
    private static readonly HashSet<Part> _returnDrops = new HashSet<Part>();

    /// <summary>
    /// Re-solve what it would cost each listed stage to come home from here.
    ///
    /// ONE prediction and ONE Jacobian for the whole list — see the header. Throttled to
    /// 1 Hz because a Jacobian is three seeded RK4 sweeps and this runs through an
    /// ascent, and because the answer moves on the timescale of the trajectory rather
    /// than the frame rate.
    /// </summary>
    private static void UpdateReturnDv(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        List<ReturnableStage> list = _s.ReturnableStages;
        if (list.Count == 0)
            return;

        long now = Environment.TickCount64;
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
            _s.ReturnDvNote = _s.AeroError.Length > 0 ? _s.AeroError : "no aero surrogate";
            return;
        }

        var sys = new DragCoastSystem
        {
            Mu = parent.Mu,
            OmegaZ = parent.GetAngularVelocity(),
            MeanRadius = parent.MeanRadius,
            AreaOverMass = aero.ReferenceArea / mass,
            Alpha = 0.0,
            Table = aero.Table,
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
            // Normal, not a fault: a vehicle still climbing has no ballistic impact
            // inside the horizon, so there is nothing yet to drag onto a site.
            _s.ReturnDvNote = nom.Status == ImpactStatus.NoImpactWithinHorizon
                ? $"no ballistic impact within {ImpactHorizonMinutes:F0} min"
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

        // MEAN RADIUS ON BOTH SIDES, deliberately. The prediction above terminates at
        // MeanRadius, so placing the sites at MeanRadius + terrain would put a terrain
        // height into the miss that is an artefact of mixing two surfaces rather than a
        // real distance. Sampling terrain per site would not fix it either: one
        // prediction serves every stage, so its termination radius has to be common.
        // Self-consistent is what this number needs; the boostback overlay is where the
        // low-passed terrain height belongs.
        double siteRadius = parent.MeanRadius;

        for (int i = 0; i < list.Count; i++)
        {
            ReturnableStage stage = list[i];
            if (!stage.HasTarget)
                continue;

            // The site in the frame the Jacobian's rows are written in - the co-rotating
            // one. Same conversion UpdateSteering makes, and getting it backwards would
            // report a cost that is right in size and wrong by the body's rotation.
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

    /// <summary>The stage-specific target a separating subtree should be handed, falling
    /// back to the vehicle's own site when none was set for it.</summary>
    private static void ResolveStageTarget(uint rootId, out double latDeg, out double lonDeg)
    {
        if (_s.StageTargets.TryGetValue(rootId, out double2 site))
        {
            latDeg = site.X;
            lonDeg = site.Y;
            return;
        }
        latDeg = _s.SiteLatDeg;
        lonDeg = _s.SiteLonDeg;
    }
}
