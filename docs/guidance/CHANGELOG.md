# Changelog

## [Unreleased]

- Only stages that separate during the climb are listed under **Returnable stages**. A stage with a command pod that separates once nothing is left to fire, such as an upper stage leaving its payload, is going orbital speed and cannot fly back, so it can no longer be given a landing target.

- Fixed a boostback with no burn plan ("the plan did not converge") when the burn is long and near the propellant aboard. The planner gave up if no burn solved at its first guess of pitch, 10 degrees, even where one at 30 degrees lands with propellant to spare; it now looks either side of the guess first. A plan that cannot be found now says why, such as not enough propellant.

- The boostback flip turns as fast as the booster can: the command is slewed at **Slew rate** only until the booster is within 135 degrees of the burn direction, so it starts turning the right way, then goes straight to it and the flight computer turns the booster at whatever its gimbals give. The boostback's settling burn is 0.2 s instead of 2 s, and **EXECUTE** on the Boostback tab no longer restarts a boostback that is already flying: each press put it back to settling, which cost a booster coming home from 3 km/s ten seconds.

- The Ascent tab's **Target orbit** box is split in three: the target, periapsis, apoapsis and inclination at the top with no header to fold them away, then **Returnable stages**, which does not fold away either and has one row per stage, then **Orbit settings** with the rest: LAN, argument of periapsis, the insertion options, the chase offset, auto warp and staging, and the launch window. The stages' live return dV is no longer shown.

- The convex ascent now **sizes the booster reserve itself** (#84), and there is no longer a reserve to set: **Booster reserve dV** is gone. Give the booster a landing site with **Set target** on its row in **Returnable stages**, under the inclination, and the plan works out from nothing what the booster needs from wherever it separates: the smallest boostback that lands it on that site, plus a landing burn of 1.3 times its terminal velocity, with 10 % on the boostback for a real burn. It then chooses the separation and the reserve together for the most mass to orbit, and the booster stages at the planned reserve. The Convex ascent section shows what the booster keeps on a **Booster** row. A core under strap-on boosters works the same way: give the core the target, and the plan keeps its reserve in the core's own stage, staged once the strap-ons are gone. The boostback is priced from 15 seconds after separation, where the booster actually lights after clearing the upper stage, settling and flipping, with 25 % on it for a real burn: from 3.3 km/s the booster's own burn planner needed 16 % more than the impulse. With no target, or without a usable convex plan, the booster keeps nothing and burns out as before. Moving the target asks for a new plan. The plan is one solve, straight from the seed, and takes seconds rather than most of a minute: the solver is now built optimised even in a Debug build.

- The convex ascent now chooses its own launch timing instead of a fixed 3-minute lead. With a target it picks when to lift off, within 10 minutes of the launch window, as part of the fuel optimisation, and EXECUTE arms for that time; the launch window row shows it. Without a target it picks the LAN for the plane over the pad, and the panel's LAN is set to it. The best launch was 2 to 7 minutes ahead of the pad passing under the plane in every case measured: 40 to 80 kg more into orbit for the KSA rockets tried. Lifting off off that time costs more the further off it is: a Saturn V into 51.6 degrees loses 4.5 t of its 142.6 t when it lifts off 15 minutes late. **Choose launch time** (with a target) or **Choose LAN** (without) in the Convex ascent section turns it off.

- The convex ascent can now plan a launch from a pad that is off the target orbit's plane. From about 7 degrees off it used to fail at once with "the solver keeps failing at the smallest trust region", whatever the vehicle. Leaving the plane still costs propellant: on a Saturn V 10 degrees costs about 44 t of the 148 t it puts into orbit, and a vehicle that cannot afford the turn now runs out of propellant short of the plane, with the gates reported not met (#90).

- The convex ascent now plans the **booster reserve**. On the pad the next staging row is usually the ignition, so the reserve only armed once the engines were lit, after EXECUTE had already planned the whole first stage. The reserve then staged the booster early, and the vehicle flew below its plan all the way to the UPFG hand-over, where it pitched up sharply. The reserve now arms on the pad, so the plan burns the first stage that much shorter and drops the booster when the plan expects it.

- The convex ascent no longer refuses to launch when its solve stalls on a plan that already passes every feasibility check: the dynamics, the max-q, max q-alpha and acceleration limits, and the insertion. Such a stall leaves the plan short only of proving it optimal, and it is now flown, provided the solver still expected less than 0.1 % of the lift-off mass in savings. The status says how many kilograms that was. A very high-thrust single-stage vehicle hit this after its max q was relaxed: the plan it refused was the optimum to within about 2 kg (#120).

- The **AFC Guidance** menu no longer disappears when another mod, such as ModMenu, patches the menu bar. With ModMenu it is under **Mods > AFC Guidance**.

- The optional dependency is now **KittenExtensions Continued** 0.5.3 or later instead of the original KittenExtensions, which does not load on KSA 2026.10.19.5638 and later. It applies the hyperbolic-targets and AUTOSTAGE button patches. Remove the original KittenExtensions before you install it.

- UPFG, G-FOLD and the convex ascent plan within the lower of the g-limit and the structural load limit at which KSA holds the throttle back. Stages with solid motors or without throttle are planned at full thrust.

- Added **argument of periapsis** to the ascent target. Tick **Fix arg. of Pe** and the ascent holds the whole ellipse in place, inserting wherever along it the burn ends instead of always at periapsis. While it flies, an **Insertion** row shows how far round from periapsis that is, and turns amber once that starts to cost real dV, or when the insertion is before periapsis, which a burn that starts from orbit may not converge on. For its last 45 seconds the burn holds the insertion it is aiming at rather than following the cutoff, so it cannot chase itself round the ellipse. A burn from orbit can only reach an argument of periapsis that puts the insertion a little past where it would naturally end, up to about 20 degrees; further round it runs out of propellant just as a free insertion placed there would. Left unticked, the argument of periapsis falls wherever the burn ends, as before.

- With the argument of periapsis free, the ascent now inserts where the burn is shortest instead of always at periapsis. Through the first half of the burn it prices every insertion from periapsis to 45 degrees past it every ten seconds, each on a copy of the live solution solved to convergence, and moves the insertion to the cheapest when that saves at least a fifth of a second of burn. The saving is a few m/s into a low orbit and can reach hundreds on a long burn into a very eccentric one. The Insertion row shows it in seconds of burn and m/s, and **Optimise insertion** switches it off.

- **Insert before apoapsis** moves a free insertion to 5 degrees short of apoapsis, and with Optimise insertion on searches from 45 to 5 degrees before it instead of past periapsis. The burn always ends still climbing, so the vehicle coasts up to apoapsis and can circularise there. It is off by default: for most vehicles a burn that ends high costs more than one that ends near periapsis.

- A target entered with its apoapsis below its periapsis now swaps the two when the field is left, so periapsis is always the lower.

- Launch to target now waits for whichever plane crossing comes next, ascending or descending, instead of the one picked by hand. The node choice is greyed out while a target is picked and stays yours for an ascent without one.

- **Launch now**, beside Warp and Stay at 1x when an armed launch offers to warp to its window, starts the ascent to the target's chase orbit straight away instead - for a window just missed. It cancels the armed countdown and heads for the nearest crossing; UPFG steers out the difference in plane on the way up, which costs more dV the further from the window the launch is.

- Launch to target now flies a **co-elliptic** chase orbit, with the target's eccentricity and argument of periapsis on a semi-major axis the offset below its own, instead of a circular one. Clear **Copy target arg. Pe** to leave the argument of periapsis free.

- An ascent to a periapsis inside the atmosphere now inserts where the orbit climbs out of it, rather than cutting the engines in the air. An ascent to an orbit entirely inside the atmosphere is refused.

- The target orbit overlay draws periapsis where a fixed argument of periapsis puts it. With it free, periapsis no longer rides along with the vehicle: it sits where the ascent's predicted cutoff and insertion angle put it, and after cutoff on the orbit actually reached.

## [0.4.0]

Boosters fly themselves home. Returnable stages with their own landing sites, and a move to MIT.

- Added **boostback guidance**. EXECUTE on the new Boostback tab flies a separated booster back toward its landing site through a settling burn, flip, boostback burn, and entry attitude, and it keeps flying whether or not you are watching it. RETARGET works there, so the site can be moved mid-flight.

- Boosters hand themselves over at separation: the stage that drops away starts its own boostback with its own landing site while you stay with the upper stage.

- Added **returnable stages** to the Ascent tab. Any stage that separates carrying a command pod is listed with its own **Set target** button and a live return cost that falls as you climb, so you can see when bringing it back becomes affordable.

- Added **Booster reserve dV**, per vehicle. Ascent leaves that much propellant in the booster and stages on it, rather than burning the stage dry and leaving nothing to come home on.

- The Boostback tab draws where the vehicle will actually hit the ground through the atmosphere rather than in vacuum, with time to impact, impact speed, and the miss against your landing site.

- **The project is now MIT licensed.** ECOS was the only copyleft dependency and forced the whole work to be GPLv3. It has been replaced by [Clarabel](https://github.com/oxfordcontrol/Clarabel.rs), which reproduces its landing solutions exactly. Attributions are in THIRD-PARTY-NOTICES.md; releases up to and including v0.3.1 remain GPLv3.

- KSA 2026.9.7.5402 compatibility.

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

- Implemented Convex Approximation for Trajectories - Sequential Algorithm (CAT-S), allowing accurate control of vehicles with high rotational inertia and paving the way for aerodynamic entries.

- Added a new user interface with a UPFG visualizer, landing visualizer, and descent pass planner.

- Added multi-vehicle control with independent PoweredGuidance instances.

- Tweaked orbit overlay.

- Added a new logo.

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
- Touchdown is now detected by actual ground contact instead of an altitude guess, for both G-FOLD descent and terminal hover. Engines cut the instant any part of the vehicle touches down.
- Fixed: solid rocket boosters were treated as having no propellant, so their
  fuel and thrust were missing from the ascent stage list.
- Auto-staging now also drops burned-out boosters that are still attached to a live core, not just at total engine flameout, and will not separate the part of the vehicle you are actually flying.
- Fixed: disengaging the autopilot could leave the vehicle spinning uncontrollably,
  and could make the Strict/Balanced/Relaxed attitude presets stop responding.

## [0.1.0] - 2026-06-18

Initial release.

- UPFG ascent guidance to a target orbit.
- G-FOLD powered descent guidance for landing.
- Terminal hover mode for the final touchdown.
- Automatic engine control and staging for both ascent and landing.
