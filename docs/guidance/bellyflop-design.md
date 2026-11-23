# Bellyflop: terminal descent for belly-first craft

Design notes, 2026-11-23. Status: agreed in outline, being built on `feature/orbital-entry`.

## The chain

| Phase | Solver | Frame | From → to |
|---|---|---|---|
| Orbital entry | energy-domain SCvx (existing) | trim angle of attack, bank | entry interface → Mach 3 |
| **Bellyflop** | **new 3-DOF SCvx** | **belly axes: pitch, bank, yaw** | **Mach 3 → the flip** |
| Flip and landing burn | 6-DOF touchdown (existing), in the background, choosing the flip | thrust axis | flip → touchdown |

Bellyflop solves in the background from Mach 4 and takes over at Mach 3 once its plan has converged and lands within tolerance of the site; until then the entry keeps flying. The touchdown solver runs in the background during bellyflop, as it does under a booster's glide, and takes over at the flip it chooses.

## Bellyflop's formulation

**Frame.** The site frame of the 3-DOF solver (rotating with the body, origin at the site, z up).

**State** `x = [r(3), v(3)]`. No mass state: nothing burns. The craft's mass is a constant.

**Attitude, belly axes.** At zero angles the belly normal points straight into the airflow (`-v̂`). From there:
- **pitch θ** turns the belly about the span axis, the nose down into the airflow or up out of it: the belly angle of attack, the sweep's `BellyBodyAero` alpha (90° less the angle from nose-first);
- **bank φ** turns it about the nose axis, one wing into the airflow: the sweep's beta;
- **yaw ψ** turns it about the belly normal, which points the nose, and with it the direction pitch lifts, over the ground.

**The reference frame, per node, from the velocity.** The angles are aero-relative, not inertial Euler angles: each node's reference body frame is built from its own `v̂` (part of the state). Belly normal `−v̂`; reference nose `n̂ = P⊥v̂(ĉ + û)` normalised, where `ĉ` is the horizontal bearing to the site when bellyflop takes over (fixed) and `û` local up; span `n̂ × belly`. In shallow flight (`v̂` near `ĉ`) the nose stands up the vertical plane through the bearing, as the entry's nose does at 90° angle of attack; falling near-vertically it lies along the bearing. It is singular only for velocity along `ĉ + û`, climbing at 45°, which a descent never does. ψ = 0 is this reference: "yaw locked to the initial bearing".

**Why angles and not a vector or quaternion.** Bellyflop has no rotational dynamics (attitude is a control) and its angles stay within ±20° of a reference that is never near an Euler singularity (±90°). The limits are then boxes, convex as they stand; the angles map almost one to one onto the belly tables' own α and β; and nothing needs a unit-norm constraint. A unit belly vector (as the 3-DOF's thrust axis) would make the separate pitch and bank limits nonlinear and need a second vector with an orthogonality constraint for yaw; a quaternion adds a norm constraint and nonlinear limits for no benefit at these angles. The flip, where attitude swings 90° and rotational dynamics matter, is the 6-DOF's, which already carries quaternions. The body axes (and the quaternion the attitude controller is given) are built from the reference frame and the three angles at output.

The body axes are built from the reference frame and the three angles; the air force comes from `BellyBodyAero.Coefficients(M, α, β)` (body-axis `C_x, C_y, C_z`, on a B-spline with exact gradients through `Dual`) rotated into the site frame. Pitch moves the lift along the nose's direction, bank across it.

**Control** `u = [θ, φ, ψ]`, rad.

**Dynamics.** `ṙ = v`, `v̇ = g(r) − 2ω×v − ω×(ω×r) + F_aero(r, v, u)/m`, with density and Mach from the sweep's atmosphere. Jacobians by forward-mode `Dual`, as the 3-DOF does.

**Discretisation.** N nodes over a free duration σ, trapezoidal collocation with virtual control, an L1 trust region; the 3-DOF solver's SCvx loop (linearise, SCS conic subproblem, ratio test).

**Constraints.**
- Pitch and bank: `|θ|, |φ| ≤ 20°` throughout (a setting). At node 0 the limit is widened to admit the attitude the craft is in, and closes at the attitude rate limit: the entry hands over at about 50° belly angle of attack, and bellyflop pitches the craft belly-first from there.
- Yaw: free, but with an L1 price far above bank's, so the solver turns the yaw only once bank is pinned at its limit ("yaw locked to the initial bearing, free when bank is pinned").
- Attitude rates: `|θ_{k+1} − θ_k|`, `|φ_{k+1} − φ_k| ≤ rate · σ/(N−1)`, a rate the flaps can fly.
- Terminal: ground track ends at the site at ground level (soft L1 on the horizontal miss, hard altitude); final velocity free.

**Objective.** `w_miss |r_xy,N| + Σ (θ_k² + φ_k²) + w_ψ Σ |ψ_k| + w_rate Σ (Δθ² + Δφ²) + virtual control and slack prices`.

**Seed.** Integrate belly-first (θ = φ = ψ = 0) from the start state to the ground; σ from that.

## The entry's range target

The entry still plans in energy down to its hand-over at Mach 3, but its range to go there is no longer zero. It is the range bellyflop will cover from the hand-over to the ground, predicted by integrating the belly-first descent (θ = φ = 0) from the plan's own predicted hand-over state, and refreshed each re-plan. So the entry aims at the ground through bellyflop's glide, without running its energy-domain model where it breaks down (near the ground the speed stops falling, the flight path turns vertical and the drag terms its dynamics divide by blow up).

## Hand-over to the touchdown solver

As under a booster's glide (`StepEntryTouchdownPrepare`, `StepSixDofStandby`, `CheckPendingEntryTouchdownHandoff`), with two changes for a belly-first craft:
- the seed path and the predicted state at ignition give the 6-DOF the **thrust axis** (the nose direction from bellyflop's attitude), not a 3-DOF thrust command;
- the predicted ignition attitude is belly-first; the 6-DOF admits it (its tilt cone widens to the entry attitude; 6dof.py's own case arrives at 92°).

**Known gap.** The 6-DOF touchdown solver has no belly-first aerodynamics: its body tables are tail-first and fade out past 28° off engine-first, so through the flip it sees only roll-averaged drag and lift on the thrust axis, with no air moment and no flap authority. It will plan the flip on RCS and gimbals alone. Adding the belly tables to `BodyAero6Dof` is follow-up work.

## Attitude

Bellyflop's attitude goes to AFC's own attitude controller as the entry's does (`FlyOwnAttitude` with the belly roll reference): flaps and RCS by authority, the RCS on the stock phase plane.

## Offline checks

A closed loop like `--orbital-entry`'s: truth at 50 Hz on the synthetic belly tables, bellyflop re-solving at 1-2 Hz, from Mach 3 hand-over states with dispersions in range, crossrange and flight-path angle; pass on a miss under a few hundred metres at the flip altitude, within the angle limits.
