# Parked SCvx issues

Known issues in the landing guidance (entry guidance, 3-DOF; touchdown guidance, 6-DOF) that are **parked**. They are understood well enough to describe, but none has shown up as a problem in flight, and the guidance flies well as it is. Pick one up if a flight shows it. The issues worth acting on are in #103.

Status as of `feature/3dof_aero` at c58e171 (2026-10-14).

## Offline checks that fail

- **`--path` "acceleration binds".** With `Scvx6DofConfig.AccelWeight` at 300, the acceleration limit does not bind on the check's case: peak 24.1 m/s² against a 12.3 m/s² limit, unconverged. 300 was chosen so touchdown guidance brakes past the limit rather than arriving at 10 m/s (288dd62); 1e3 is an untested middle.
- **`--3dof-mpc` "lift −30 %" and "combined".** Both land off target. They predate the 2026-10-13 work.
- **The acceleration slack is not in the merit.** `Scvx6DofSubproblemScs` prices `AccelWeight` x slack in the convex objective, but `Scvx6DofSolver.Iterate`'s predicted merit and `TrueCost` leave it out. So the ratio test cannot credit a step for reducing the acceleration-limit violation, and may refuse one that trades fuel for it. This may be why "acceleration binds" stays unconverged. Adding it changes which steps are accepted on every landing, so it wants a flight test; see #105.

## Model and estimation

- **The thrust-bias estimator double-counts the gimbals' side force.** `UpdateAccelBias` subtracts only the axial thrust from the measured acceleration, so the side force leaks into the bias, on top of being in the model.
- **A warm seed's end moves with re-anchoring.** `ColdReferenceShift` carries the velocity offset through the horizon, which moves the seed's terminal point by dv × sigma. The solve pulls it back onto the target, but it might contribute to warm refusals.

## Hand-overs

- **Retarget during the 6-DOF standby.** A target move while the 3-DOF flies the burn makes the standby cold-restart on drift. It has never been examined.

## Performance and logging

- **The synchronous cold solve fails on a mass defect.** It reports −2640 kg at interval 0. Flight uses the spread cold solve, which does not.
- **Engaging stalls the main thread for about 0.78 s.** This is the KSAero sweep.
- **The cycle log's `twrMin` uses linear thrust.** It reads the throttle floor × capability, not the engine's thrust curve.
