# Changelog

## [Unreleased]

Ascent guidance: fixed the attitude jerking on the way up, and stopped the launch
plane being seeded under the pad instead of ahead of it.

- The commanded attitude no longer takes its ROLL REFERENCE from
  cross(steering, position). Those two are the same vector on the pad and within a
  float epsilon of it through the start of the pitch-over, so KSA substituted an
  arbitrary axis and then snapped out of it the moment the pitch program made the
  cross product representable — an 18 deg attitude jump in one 10 ms step on the
  test geometry, with several tenths of a degree of noise per step either side of
  it. The ascent now rolls to its own target plane, which is well conditioned from
  the pad to cutoff, and the roll it holds is the one the vehicle ALREADY HAD when
  guidance engaged — measured off the vehicle's own body axes at that moment, so an
  ascent is commanded a thrust direction and nothing else. Where the stock
  construction was fine the two agree exactly (`tools/convtest` checks all of it).
- UPFG now solves once per second instead of once per SIM STEP. It is a recursive
  once-per-guidance-cycle algorithm whose corrections integrate across cycles, and
  the reference implementation it was ported from ran at exactly this cadence;
  calling it at 60 Hz wound those corrections up sixty times faster than they are
  damped for. Its convergence test — "tgo settled between calls" — is now measured
  against how far tgo *should* have moved in the interval, so it means something at
  any call rate rather than passing on the second step.
- The commanded direction slews toward the solution at up to 5 deg/s rather than
  jumping to it, so a guidance cycle landing, the hand-over out of the open-loop
  turn and a staging transient all reach the flight computer as attitude motion
  instead of an attitude step.
- Engine cutoff is timed from the solve that produced tgo rather than from the step
  that noticed it.
- The LAN seeded by "from position" is now the plane over where the pad WILL be
  three minutes from now, not the one overhead at the moment the button is pressed.
  The pad is carried east the whole time the vehicle is still climbing through the
  vertical rise and the turn, so the plane that was overhead at lift-off is one the
  ascent has already gone past by the time it is flying, and the guidance yaws to
  chase it — overshooting the plane it was aimed at.
- Launch-to-target ignites on the same lead: the window countdown now runs to the
  launch that puts the plane crossing three minutes into the ascent, rather than to
  the crossing itself. Both readouts name the lead, so T-0 arriving before the site
  is in the plane reads as intended rather than as an error.

- Ascent steering is now the linear-tangent LAW rather than a once-a-second sample of
  it. UPFG computes lambda and lambdadot and the port used only i_f = unit[lambda -
  (J/L)lambdadot], the value at the solve instant; the law is evaluated every step now
  (`UpfgGuidance.SteeringAt`), so the commanded direction is continuous between solves
  and the guidance cadence no longer has to be hidden from the attitude.
- The turning rate that law implies is published to KSA's flight computer as the
  target's own rate (`KsaAttitudeRate`, a postfix on `FlightComputer.UpdateAttitudeTarget`).
  `AttitudeTarget.RatesCci` is a feedforward the FC already subtracts from the measured
  body rate, but the Custom branch fills it with the reference frame's rate — nil for
  the inertial frame we steer in — so guidance was declaring a moving target stationary
  on every step and the controller nulled the error instead of tracking the motion.- Stage list: adjacent arcs with the same thrust, the same Isp and no mass gap

- Ascent settings gain a **Force roll** checkbox and a roll-angle box, greyed until it
  is ticked. Left alone the ascent still commands a thrust direction only and holds
  whatever roll the vehicle lifted off with; ticked, it commands the angle given,
  measured about the thrust axis from the target plane's normal. Ticking it also takes
  the flight computer out of decoupled roll mode — `UpdateAttitudeTrackError` computes
  no roll term while decoupled (the default) and would otherwise discard the commanded
  roll entirely. Un-ticking holds the roll the vehicle is in at that moment rather than
  rolling back to lift-off.
- Release packaging: the `PackageRelease` target copied `CHANGELOG.md` from beside the
  csproj, where it has never been. `dotnet build -c Release` failed at MSB3030 on every
  run; it now takes the file from the repo root.  between them are now merged back into one stage, and arcs carrying under 1 m/s

- Fixed EXECUTE arming a launch that never fired. The armed auto-launch — tracking the
  window, keeping the chase orbit on the target, and pressing EXECUTE at the window —
  all ran from the DRAW, so it was gated on the panel being open, the Ascent tab being
  the selected one, and the "Target orbit" fold being expanded. Collapse the fold or
  look at another tab while waiting and the launch never happened, which from the seat
  is indistinguishable from a dead button. `StepLaunchWindow` owns it from the sim step
  now; the draw only prints the status.
- Fixed the launch lead wrapping a near window into the next revolution. Inside the
  lead window the ideal ignition is behind us while the plane crossing is still ahead,
  and the wrap reported the following day's launch. It now goes immediately — at worst
  the lead late, which only means the crossing lands earlier in the ascent.
- Fixed the panel drawing at 56% width on 16:9. KSA 2026.8.22 changed `ScreenReference`
  to normalise UV by window HEIGHT on both axes (`UvToPixelsNormalized`), where it had
  been width; every authored dimension in the gauge shrank by the aspect ratio at a
  stroke. `GaugeUnit` reads the viewport width directly now, and `GaugeScreenUv`
  converts at the one boundary that still speaks ScreenReference's convention.
- Guarded the stage model against KSA double-counting engines. Module-level sequencing
  lets one part sit in several sequences, and the drain simulation's registration loop
  walks sequences 0..k registering every EngineController it finds, with no
  de-duplication — so a part carrying an engine module in one sequence and a decoupler
  in another has its cores registered twice. Thrust and mass flow double while the mass
  ratio does not, so the stock delta-v looks right while thrust reads double and burn
  time reads half: exactly the two figures UPFG steers on. The adapter now cross-checks
  each phase against the vacuum capability of the engines the game says are in it, and
  believes the engines. Conservative by construction — vacuum sequences only, no
  solids, and only for a discrepancy far larger than modelling noise.  are dropped. KSA's drain simulation splits a burn wherever its engine-core count
  changes for even one iteration, and its inner loop steps from one TANK emptying
  to the next, so a stack with several tanks can produce several rows for one
  physical stage. (The acceleration limit also splits a stage in two, by design —
  that one is now marked G in the readout rather than looking like an extra stage.)
- The staging readout now shows which sequence each stage came from, how many
  engines the game had burning across it, and a G on any segment the acceleration
  limit split off; and it shows KSA's own total dV alongside ours whenever the two
  disagree by more than a percent.

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
