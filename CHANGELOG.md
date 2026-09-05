# Changelog

## [Unreleased]

## [0.4.0]

Boosters fly themselves home. Returnable stages with their own landing sites, and a move to MIT.

- Added **boostback guidance**. EXECUTE on the new Boostback tab flies a separated booster back toward its landing site — settling burn, flip, boostback burn, then entry attitude — and it keeps flying whether or not you are watching it. RETARGET works there, so the site can be moved mid-flight.

- Boosters hand themselves over at separation: the stage that drops away starts its own boostback with its own landing site while you stay with the upper stage.

- Added **returnable stages** to the Ascent tab. Any stage that separates carrying a command pod is listed with its own **Set target** button and a live return cost that falls as you climb, so you can see when bringing it back becomes affordable.

- Added **Booster reserve dV**, per vehicle. Ascent leaves that much propellant in the booster and stages on it, rather than burning the stage dry and leaving nothing to come home on.

- The Boostback tab draws where the vehicle will actually hit the ground — through the atmosphere rather than in vacuum — with time to impact, impact speed, and the miss against your landing site.

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
