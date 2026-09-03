# Changelog

## [Unreleased]

- **Ascent leaves the booster propellant to come home with**, and hands it over.
  A new per-vehicle **Booster reserve dV (m/s)** on the ascent tab; at zero nothing
  changes.

  **A dV rather than a mass, because the arithmetic is the whole point.** The
  propellant a reserve costs is `m_dry · (exp(dv/ve) − 1)` — the *booster's* dry
  mass — and the upper stage is not in it. That is exact, not an approximation, and
  it is why sizing against the mass actually to hand at staging is a trap. On a 20 t
  booster under a 40 t upper stage, a 500 m/s reserve is 3.63 t; measured against the
  60 t stack it is 10.88 t, which is 7.25 t of ascent propellant given up for nothing
  and 1303 m/s where 500 was asked for. `m_dry` is read straight off the stage model:
  the mass that goes overboard at the first jettison, the same discontinuity
  `Coalesce` already uses to decide a stage boundary is one.

  **Two levers, and both are needed.** UPFG never commands staging — it plans around
  boundaries, but the event is fired by the tanks running dry. So the reserve (1)
  raises stage 0's `MassDry` on the ascent's *copy* of the model, which shrinks
  `burnTimes[0]` and with it `L`, the dV UPFG believes is aboard, so it hands the
  difference to the upper stage; and (2) adds a third cue to `AutoSequence`. Where
  the existing two wait for propellant to run out, this one fires while the stage is
  still perfectly able to burn, because what is left is spoken for. Lever 1 alone
  makes UPFG plan a shorter stage and then burn straight through the reserve.

  **The guard that keeps it off a strap-on stack.** "The next thing that separates"
  is not the same as "the booster": while solids burn beside a core, the next
  separation is the casings, and reserving there would end the whole first stage
  early to leave propellant in something about to be thrown away. The reserve arms
  only when the next separation takes *every* engine currently producing thrust with
  it, which becomes true on its own once the strap-ons are gone. It also will not
  fire during the vertical rise, so a reserve larger than the stage holds stages the
  vehicle early rather than on the pad — and the readout says so before it bites.

  **Auto hand-over.** A separated booster is a *new* `Vehicle` with no entry in the
  state table, and the sweep drops unknown unfocused craft on the floor — correct for
  every other vehicle in the universe and exactly wrong for this one. So the parts the
  decoupler is about to detach are recorded *before* the split, and the first unknown
  vehicle carrying one of them is adopted: given state, given the ascent's landing
  site, and started on boostback. The engage is retried rather than done on the
  adoption frame, because that is the frame the part tree is least settled and the
  aero sweep has nothing to fit a surrogate to.

  One at a time: the hand-over record is a static, because it has to be read from a
  vehicle that does not exist yet and so cannot live on either party. A pair of side
  boosters separating together would need one record each. It expires after 30 s.

- **`Navbox.Numerics` is `PoweredGuidance.Numerics`**, and `Navbox.Flight` is
  `PoweredGuidance.Flight`. The name predates the mod and named nothing that still
  exists; the shipped assembly, the mod folder and the `PoweredGuidance.Upfg`
  namespace were already the other name, so this just stops the bottom of the stack
  disagreeing with the rest of it. Project, directory, root namespace, both solution
  files and the packaging targets moved with it, `git mv` so the history follows.

  The guidance panel moved too: id `"NavboxGuidance"` → `"PoweredGuidance"`, title
  "Powered Guidance". The id is what a persisted window layout is filed under, so
  **this panel's saved position resets once** — drag it back and it stays.

  `legacy/navbox/` deliberately did not move: that is the actual old navbox, a
  different program and correctly named.

  If you have deployed the mod before, an orphaned `Navbox.Numerics.dll` will be left
  in `Documents\My Games\Kitten Space Agency\mods\PoweredGuidance\`. Nothing
  references it, so it is clutter rather than a fault, but it can go.

- **Boostback is a guidance mode now**, not just a workbench: EXECUTE on the
  Boostback tab starts a four-phase machine (`Guidance/Boostback.cs`) that runs
  from the per-vehicle sim step, so a booster flies itself home whether or not
  anyone is watching it.

  - *Separation* — 2 s at the vehicle's own minimum throttle, holding the attitude
    separation left it in. Read from `PartTree.EngineThrottleMin` rather than
    `Ksa6DofSetup.VehicleThrottleFloor`, which reports 1.0 for an engine that can
    be commanded to zero — the right conservative direction for a descent solver
    sizing its authority, and exactly the wrong one for a settling burn.
  - *Rotation* — the commanded thrust axis slews onto the boostback dV at a fixed
    rate, with the engine **still lit at minimum throttle**: the gimbals are what
    turn a booster this size, and a gimbal makes torque in proportion to the
    thrust it deflects, so shutting down for the flip leaves only the authority
    that cannot do it. The dV the sweeping thrust axis lays down costs propellant
    but not accuracy — the correction is re-derived from the live state and the
    burn does not start until the turn is done. The flight computer gets the
    attitude **and the rate it is turning at** (`KsaAttitudeRate`), so it tracks
    the slew instead of nulling a sequence of stationary targets. It hands over
    when both the command and the vehicle are inside 2°, not just the command.
  - *Boostback* — full throttle in three stages. Most of it flies the **shot plan**
    (`BoostbackShooter`), re-solved every 2 s on a receding horizon: each solve
    plans a burn starting *now* from the live state, and the vehicle flies the head
    of the freshest one. At T-5 s the plan **hands over to the impulsive
    correction**, re-solved at 10 Hz; at T-2 s that command **freezes** and the
    rest runs open loop, cutting off on sensed dV against the dV owed at the
    freeze. The two laws swap jobs because the impulsive model's error scales with
    burn *duration* — over five seconds it is nearly exact, and it is cheap enough
    to keep the loop closed where the plan is not.
  - *Entry orientation* — slews to **surface** retrograde and holds. Surface, not
    inertial, because KSA's atmosphere co-rotates rigidly with the body.

  The burn attitude and the prediction's assumption agree by construction rather
  than by luck: `DragCoastSystem` holds alpha 0 for the whole coast, and both the
  boostback dV and the entry command put KSA's body +x aft. The rotation is the
  only stretch that is not near alpha 0, and nothing is predicted through it.

  Guards, because a closed loop with no convergence proof will otherwise burn to
  depletion pointed at whatever the last solve produced: a burn-duration backstop
  at 3x the ignition estimate, a propellant check (armed after one second, since
  the master switch does not reach the engine until the foot of the same step),
  and a skip straight to the entry attitude when the correction is under 5 m/s.
  Auto-staging is deliberately *not* run alongside it — the machine cuts the
  engine at boostback cutoff and coasts from there, and the auto-stager reads no
  thrust as a cue to fire the next sequence, which on a returning first stage
  means the sequences that separate it.

  EXECUTE and ABORT no longer stripe out on this tab, RETARGET works there (the
  site is what the correction aims at), and the landing-site marker is drawn
  alongside the impact marker so the miss line has both ends.

- **Direct shooting on the boostback burn** (`PoweredGuidance.Flight.BoostbackShooter`,
  `Scvx.Console --shoot`). A burn is five numbers — pitch, yaw, pitch rate, yaw
  rate, duration — flown as a linear tangent law `unit(lambda + lambdadot*tau)`
  through the real powered arc (mass depletion, drag at the angle of attack the
  burn actually flies) and then coasting retrograde to the ground. Two knobs are
  driven to hit the site, three are optimised.

  The inner/outer split is the design, not a convenience: **duration → downrange,
  yaw → crossrange**, each the strongest and most independent lever on one
  component of the miss, and **exactly two**. Given three the inner solve would be
  underdetermined and would need a minimum-norm choice — which is precisely how the
  impulsive law ends up pointing at the ground. Restricting it to two keeps that
  decision in the outer loop where it is optimised rather than stumbled into.

  **The finite-burn optimum points 25° ABOVE the horizon**, where the impulsive
  correction points 33° below:

  | pitch | burn | propellant | note |
  |---|---|---|---|
  | −5° | — | — | not enough propellant (0.3 km short at the tank limit) |
  | 0° | 58.9 s | 16017 kg | |
  | **25°** | **48.1 s** | **13088 kg** | cheapest |
  | 40° | 50.7 s | 13774 kg | |

  So the nose-down answer was a modelling artefact of pretending a 50 s burn is an
  impulse — confirmed rather than argued. Lofting is **18.3% cheaper** than a level
  burn, and below 0° the problem is not merely expensive but *unsolvable* for this
  vehicle: the tank limit is reached with tens of km still to go. The pitch floor
  added earlier is therefore compensating for a modelling error, not overriding a
  genuine optimum.

  The turn rates earn almost nothing here (48.12 s against 48.13 s, optimal pitch
  rate −0.03°/s), so the fixed-direction burn is essentially optimal on this arc and
  the extra two parameters could be dropped. ~900 ms for the full three-parameter
  search — a solve-at-ignition-and-track computation, not a per-cycle one.

  **This is what the boostback burn now flies.** `Guidance/Boostback.cs` re-solves
  it every 2 s from the live state and points along `SteeringAt(tau)` — the law
  evaluated at the elapsed time, not a stationary sample of it — with a fresh
  off-cadence solve forced at ignition, since a plan's duration is measured from
  when it was solved and flying a two-second-old one would cut the burn short by
  exactly that.

  The impulsive correction stays, doing the job it is good at: it is cheap, it is
  re-solved at 10 Hz, and it answers *is there anything worth lighting an engine
  for, and is there anything left to correct*. It no longer says which way to point.
  The **Min pitch** knob moved with it, from a null-space nudge bought after the
  fact to a hard bound inside the optimisation — the same knob, now constraining
  the burn that is actually flown, and costing nothing when the unconstrained
  optimum already clears it.

  Measured closed loop, against an engine 3% down on the thrust the plan was built
  on (`--shoot` now flies all three stages):

  | | miss | burn |
  |---|---|---|
  | one plan, open loop | 6.91 km | 48.1 s |
  | plan, frozen for the last 5 s | 0.87 km | 49.4 s |
  | **plan → impulsive at T-5, froze T-2** | **0.54 km** | 49.5 s |
  | … and no freeze at all | 0.52 km | 49.5 s |
  | *[ref]* plan re-solved to cutoff | 0.24 km | 49.5 s |

  So the loop absorbs the modelling error, and the terminal handover recovers about
  **half** of what freezing the plan cost — 337 m of 635, not all of it. Two
  seconds of open-loop tail costs 15 m where five cost 337.

  The reference row is the honest ceiling and is recorded rather than omitted: the
  impulsive law is an approximation and the plan is not, so re-solving the plan
  straight through to cutoff is *more* accurate than any terminal scheme here. What
  it is not is flyable, for a reason this check cannot reproduce — its prediction is
  perfect, so it prices what an open-loop tail costs without reproducing the
  low-passed terrain height and the ratio-of-two-small-numbers correction that are
  the reason for having one.

  Cost on the sim thread: a warm re-solve is **2.3 ms in Release**, which is what
  the packaged mod is built as, against 24.6 ms in Debug — a tenfold gap worth
  knowing before reading a profile. `BoostbackPlanSolveMs` is on the tab, so a
  vehicle where this stops being affordable says so rather than quietly
  stuttering. Getting there was measurement rather than guesswork: `BurnNodes`
  went 60 → 16 after `--shoot` showed **8 nodes landing in the identical place as
  240**, the in-flight coast is stepped four times coarser than the overlay's, and
  the pattern search starts at 2° warm instead of 8°.

- **Flight-path-angle shaping** on the boostback correction, using the free
  direction. The targeting correction minimises dV and nothing else, and the
  cheapest way to drag an impact point backwards is often to thrust *downward* —
  measured on the reference arc, 94.5 m/s of a 173.7 m/s correction points **33
  degrees below the horizon**. That is the dive, and it is the documented failure
  of instantaneous impact-point guidance (Jo, Han & Ahn §2.4).

  `ImpactSteering.FreeDirection` returns the velocity change that moves the impact
  point *nowhere* — J maps three velocity components onto two impact directions,
  so exactly one combination is free. Spending dV along it re-aims the burn
  without disturbing the target: the standard task-priority arrangement from
  redundant-manipulator control, primary task untouched by construction. Measured:
  a 60 m/s nudge moves the impact **47 m**, against 7.8 km if the same dV went into
  steering — 0.6%, so "free" is accurate to first order and the leak is real but
  small.

  Exposed as one knob, **Min pitch (deg)**, per vehicle, and it is honoured
  whatever it costs. Zero means "never thrust below the horizon", positive lofts
  the burn — which is what buys flight time for a low-thrust vehicle — and very
  negative switches shaping off. A *floor*: a burn already above target is left
  alone.

  **There is no dV cap**, and adding one was a mistake worth recording. Moving
  along the free direction does not change where the vehicle lands, so any pitch is
  reachable by moving further along it; the only limit is geometric, because the
  command asymptotes to the free direction and so cannot pitch above *its* pitch
  (52.4° on the reference arc). An earlier version capped the spend at 60 m/s,
  which pinned the achievable pitch near −15° whatever the knob said. Shaping still
  fades out with the burn without a cap, because the dV for a given pitch is
  proportional to the targeting correction — the ratio depends only on the angle
  and the geometry.

  Measured across the range, and the cost is the thing to watch rather than the
  angle:

  | target | shaping dV | total dV | ratio | impact leak |
  |---|---|---|---|---|
  | −20° | 42 m/s | 179 m/s | 0.24× | 0.02 km |
  | 0° | 119 m/s | 211 m/s | 0.69× | 0.20 km |
  | +20° | 251 m/s | 305 m/s | 1.45× | 0.97 km |
  | +40° | 722 m/s | 743 m/s | 4.16× | 11.4 km |

  The leak grows with the square of the nudge (the free direction is free only to
  first order); the closed loop absorbs it as targeting error the next cycle.

  It targets the **commanded direction**, not the vehicle's flight path angle. The
  distinction is expensive: levelling the command costs 119 m/s, levelling the
  velocity would cost **441.9 m/s**. A booster past apogee is descending whatever
  happens and cancelling that is not the boostback's job. On the reference arc it
  pulls the burn from −33.0° to
  −14.8° for 5.8% more dV at a 0° target. The knob is the equivalent of the paper's
  offline-tuned FPA-rate threshold and wants tuning against flight results.

  `SteerShape` is deliberately **not** folded into `SteerDv`: the latter's
  magnitude is what tells Boostback there is targeting work left, and a shaping
  term holding that number up after the miss was nulled would stop the burn ever
  ending on its own. `SteerCommand` is what to fly and how long to burn for;
  `SteerDv` alone is what ends it.

- Added a **Boostback** tab to the guidance panel, next to Ascent. It is an aero
  workbench rather than a guidance mode for now: opening it samples the focused
  vehicle's drag off KSA's *own* `BoundingBoxCdA.ComputeCdA`, adds the skin term
  the game applies separately, averages over roll azimuth, and fits the
  `Cd(Mach, alpha)` surrogate. Nothing is re-derived, so the table cannot drift
  from what the vehicle actually flies through. It resamples only when the
  bounding box changes, which is the only thing that can change the answer. The
  guidance above is built on it; the 6-DOF dynamics still have no aero term.

- Added a **drag-integrated impact point** to the Boostback tab, behind a toggle,
  drawn as a world overlay alongside the existing G-FOLD and 6-DOF ones. It RK4s
  the vehicle's current state to the terrain through KSA's own drag model and the
  mirrored atmosphere, holding the retrograde attitude, and marks where it lands
  with lat/lon, time to impact, ground downrange, impact speed and the miss
  against the landing site. The drawn track is de-rotated by the body's spin, so
  it ends on the marker rather than tens of kilometres away from it.

- Added `PoweredGuidance.Numerics.Rk4`, a shared fourth-order integrator generic over an
  `IOdeSystem` struct, and `PoweredGuidance.Flight.ImpactPredictor` on top of it. Both
  carry `Dual`s throughout, so `d(impact point, time of flight)/d(initial state)`
  comes out of a seeded sweep — verified against central differences by
  `Scvx.Console --impact`, along with fourth-order convergence and energy
  conservation. That includes the derivative **through the stopping condition**:
  the surface crossing is solved by Newton in Dual arithmetic rather than by
  bisection, because a bracket method's iterate differentiates to a secant
  estimate of the sensitivity rather than the exact one. Getting that wrong left
  `d(impact)/d(v0)` with the wrong sign on one column.

  The overlay is ~4x cheaper than it first landed (3.7 ms -> 0.9 ms per
  prediction): `AeroTable.Cd(Dual, Dual)` skips the spline's gradient when neither
  input is seeded (bit-identical, 43% off an unseeded drag evaluation), and the
  fine integration step now starts eight scale heights up rather than at the
  atmosphere's nominal 167 km top — which had put essentially every suborbital
  trajectory on the fine step for its whole flight. Default steps still land
  within 2 mm of a converged answer.

  The marker no longer skips: the path and impact point are stored body-fixed so
  they stay glued to the ground between recomputes, the terrain height feeding the
  target radius is low-passed (it is sampled at a moving point, and a shallow
  descent turns a few hundred metres of height into kilometres of lateral
  movement), and the drawn point blends toward each new prediction instead of
  teleporting. It is red now, with a ringed crosshair and drop-shadowed text.

  `ImpactPredictor.VelocityJacobian` gives `d(impact)/d(v0)` as a 3x3 in three
  seeded sweeps (~3.6 ms) — the drag-integrated replacement for the Keplerian
  `d` vectors a closed-form IIP steering law inverts. It reports the impact point
  in the **co-rotating** frame, computed inside the Dual chain, because the
  rotation angle is `omega * (time of flight)` and the time of flight depends on
  the state: doing that rotation on the `.V` parts outside the chain drops
  `d(tof)/dv` and is wrong by **50%** on this trajectory (measured, 39.4 m/(m/s)
  against a column of 78.9). `Dual` gained `Sin`/`Cos` for it, and the overlay now
  reads the predictor's own ground-frame answer so the picture and the
  sensitivities cannot disagree.

  `ImpactSteering.Correction` turns that Jacobian into the velocity change that
  moves the impact onto the landing site, drawn as an arrow from the vehicle
  behind its own toggle. It never forms an inverse: it solves
  `(J J^T + lam^2 I) y = m` and returns `-J^T y`, the damped right pseudo-inverse.
  The damping is what makes that well posed — J is structurally rank 2, and rank 1
  in-plane, since a planar trajectory can only slide its impact along its own
  ground track. One step removes 94% of a 21.5 km miss; the same dV along the
  greedy direction leaves 2.3x as much.

  Measured, and it matters for the boostback plan: on a boostback-shaped coast
  the drag landing point is **long** of the vacuum impact point, not short — drag
  delays the descent (which gravity keeps re-accelerating) more than it kills
  downrange speed (which nothing does). The sign flips only for shallow, fast
  entries. At KSA's real drag the error in the vacuum IIP is tens of percent.

- **Angle of attack is now retrograde-first**: alpha = 0 is flying tail-first
  with the engine into the wind, 180 is nose-first. Every phase the surrogate
  exists to serve is flown engine-first, so this puts the small angles where the
  vehicle actually sits instead of at the far edge of the table. The alpha axis
  now spans the full 0–180 rather than stopping at 30, and the placeholder table
  was regenerated on the new convention — leaving prograde 0–30 data under a
  retrograde label would have been silently wrong.

- Added `PoweredGuidance.Flight.ExponentialAtmosphere`, a self-contained mirror of KSA's
  atmosphere — `rho0 exp(-h/H)` above *mean* radius, the same hard cutoff, the
  same sub-sea-level clamp — so a solve can plan through the air without
  referencing the game. Speed of sound is *derived* rather than assumed: the
  model is isothermal, so `a = sqrt(gamma P0/rho0)` is constant with altitude
  (340.3 m/s for Earth). The Boostback tab re-verifies the mirror against KSA's
  own `GetAtmosphericDensityAtAltitude` on every resample. `Dual` gained `Exp`.

  Two findings worth recording, both from the game rather than from us: KSA has
  **no Mach dependence at all**, so the surrogate's Mach axis is flat and kept
  only as the interface the solver wants; and KSA's isotropic skin term
  (`0.1 x bounding-box surface area`) *dominates* form drag for a slender stack,
  roughly 33:1 nose-on — so drag is nearly attitude-independent near alpha = 0
  while still varying ~4x across the full range.

- Removed the SCS backend from the G-FOLD descent. It was added in 0.4.0 as a
  cross-check while Clarabel was unproven; Clarabel has since been the only
  backend that flies, and carrying a second solver cost a per-vehicle selector,
  a tolerance knob that only one of them used, and two comparison harnesses.
  `Gfold.Console --ab` is gone with it, and `--ab-rt` is now `--frame` — the
  question it answers, whether a solve fits in a sim frame, was never about the
  A/B. The `SparseCcs.VStack` self-test it used to run has moved to
  `--clarabel-smoke`, since that conversion is on Clarabel's path too. SCS is
  still vendored and still shipped: Scvx.Core uses it for the 6-DOF subproblem.

- Added a tabulated aero surrogate: `lib/PoweredGuidance.Numerics` holds an N-d
  tensor-product cubic B-spline with analytic gradients, and `PoweredGuidance.Flight.AeroTable`
  fits Cd(Mach, alpha) to it. It composes with the existing forward-mode AD, so
  aero can be written inline in Dual-valued dynamics rather than differentiated
  by hand. `Dual` and the aero model moved into the new shared library; nothing
  consumes the table in flight yet. `Scvx.Console --aero` checks it.

## [0.4.0]

Relicensed to MIT. ECOS is gone, replaced by Clarabel.

- Replaced the ECOS conic solver with [Clarabel](https://github.com/oxfordcontrol/Clarabel.rs) for the G-FOLD powered descent. Clarabel is an interior-point method like ECOS and reproduces its answer exactly — same time of flight to 0.01 s, same fuel to 0.01 kg, offline and at the shape the mod flies — while being Apache-2.0 rather than GPLv3.

- **The project is now MIT licensed.** ECOS was the only copyleft dependency and forced the whole work to be GPLv3. Every remaining vendored component is permissive: Clarabel Apache-2.0, SCS MIT, their shared AMD BSD-3 and QDLDL Apache-2.0. Attributions are in THIRD-PARTY-NOTICES.md. Releases up to and including v0.3.1 remain GPLv3.

- Added an SCS backend alongside Clarabel, selectable per vehicle, as an independent cross-check on the descent solution. It agrees to a fraction of a kilogram but costs several times as much on a problem this shape, and is not the default.

- Added a solve time limit and a solve-cost readout to the descent. G-FOLD runs synchronously on the sim thread every 0.25 s, so a solve that outruns its frame is visible now rather than felt.

- `Gfold.Console` gained `--ab` and `--ab-rt` to compare the backends on the identical assembled problem, offline and at the in-flight problem size.

## [0.3.1]

Fixed the ascent attitude jerk, aimed the launch plane ahead of the pad, and caught up with KSA 2026.8.22.

- Fixed the attitude jerking on the way up: the commanded roll came from cross(steering, position), which is degenerate on the pad and snapped 18 degrees at pitch-over. An ascent now holds the roll it lifted off with.

- Steering is flown as the linear-tangent law rather than a once-a-second sample of it, and the turning rate it implies is handed to the flight computer as the target's own rate.

- "LAN from position" and launch-to-target now lead the pad by three minutes, so the plane crossing lands inside the ascent rather than at lift-off.

- Fixed EXECUTE arming a launch that never fired while the panel was folded or on another tab.

- Added a Force roll checkbox and roll angle to Ascent settings.

- Stage list no longer splits one physical stage across several rows, and names the sequence, engine count and G-limited segments.

- KSA 2026.8.22 compatibility: fixed the panel drawing at half width, and guarded the stage model against the drain simulation double-counting an engine.

## [0.3.0]

Added sequential convex programming and UI updates.

- Implemented Convex Approximation for Trajectories - Sequential Algorithm (CAT-S), allowing accurate control of vehicles with high rotational interia, and paving the way for aerodynamic entries.

- Added new UI designed for the end-user. New UI elements such as UPFG visualiser, landing visualiser and descent pass planner.

- Added multi-vehicle control with independent PoweredGuidance instances.

- Tweaked orbit overlay.

- New logo. 

## [0.2.1]
HOTFIX

- Fixed flight computer not resetting properly.

## [0.2.0]

Updated compatability with new staging menu, added ascent overlay, tweaked landing parameters

- Ascent params moved into the main Ascent tab (no more separate popup), expanded
  by default alongside the target orbit.
- Added a map/world overlay for ascent: target orbit in cyan, flown trajectory in
  magenta. Toggleable, and hidden while the Landing tab is open.
- Stage table now shows delta-v per stage, and is visible before you hit EXECUTE
  so you can check your staging on the pad.
- Moved the G-FOLD-to-hover handoff setting into the Deorbit tab, next to the rest
  of the approach settings.
- Touchdown is now detected by actual ground contact instead of an altitude
  guess, for both G-FOLD descent and terminal hover — engines cut the instant any
  part of the vehicle touches down.
- Fixed: solid rocket boosters were treated as having no propellant, so their
  fuel and thrust were missing from the ascent stage list.
- Auto-staging now also drops burned-out boosters that are still attached to a
  live core, not just at total engine flameout — and won't ever separate the part
  of the vehicle you're actually flying.
- Fixed: disengaging the autopilot could leave the vehicle spinning uncontrollably,
  and could make the Strict/Balanced/Relaxed attitude presets stop responding.

## [0.1.0] - 2026-06-18

Initial release.

- UPFG ascent guidance to a target orbit.
- G-FOLD powered descent guidance for landing.
- Terminal hover mode for the final touchdown.
- Automatic engine control and staging for both ascent and landing.
