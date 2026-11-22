# Maneuvers and transfers

## Maneuver Quick-Tools

On the **MANEUVER** page of the console:

- **Circularize** - at apoapsis or periapsis.
- **Set Periapsis / Set Apoapsis** - single burn at the opposite apse to raise or lower one apse to a target altitude.
- **Match Inclination** - plane-change burn at AN or DN to align with a target orbit's plane. The target starts on the craft's own target, and **SET AS CRAFT TARGET** in **GNC** makes the picked one the craft's target.
- **Set Inclination** - plane-change burn at AN or DN to set an absolute inclination angle. The reference plane is selectable: **Equatorial** (parent body's equator, which matches `Orbit.Inclination`, because KSA's CCI +Z is the spin axis of the body) or **Ecliptic** (the plane of the system, KSA's CCE +Z). The row starts on **Equatorial**. For Earth the two differ by the ~23.4 degree obliquity.

The page starts on **Circularize**, or on **Set Apoapsis** when the orbit is already circular. A target field starts on an even value that differs from the orbit now, and it gets a new value when the situation, the body or the planned burns change. After **CREATE BURN** the page shows the burn that was made, with **FLY BURN** and **DELETE BURN** in the footer and **NEXT TOOL** to plan another burn from the orbit after it. **FLIGHT PLAN** is in the **PARAMETERS** fold.

When an action ends, the head offers one next step with a link: **CIRCULARIZE** after the ascent, the next pass after a pass, the orbit and **MANEUVER** after a burn. The console does not change page by itself.

Right-click the controlled vehicle's orbit and open **Advanced Flight Computer** to select a quick-tool.
The **At Periapsis** submenu offers **AFC: Set Apoapsis...**, and **At Apoapsis** offers **AFC: Set Periapsis...**.
These shortcuts open the console on **MANEUVER** with the tool selected, and they do not place a burn at the clicked point.
The stock Transfer Planner keeps its own plan types, including its own **Circularize**.

Quick-tools can plan a single burn after the last planned burn that changes the trajectory.
For example, plan a burn to raise apoapsis, then use **Set Periapsis** on the resulting orbit.
The preview and created node use that post-burn trajectory.
Multi-pass cannot start on a pending burn's trajectory.
Shortcuts wait until an active multi-pass execution has finished.

## Flyby Targeting

The stock planner aims every transfer at the target body's center, so a well-timed Hohmann arrives as an impact and the flyby has to be set up afterwards as a separate correction. Tick **TARGET FLYBY PERIAPSIS** on the **TRANSFER** page of the console and the departure burn is aimed to arrive at a periapsis you choose instead, so **CREATE FLYBY DEPARTURE** fires the flyby directly.

- **Periapsis** is entered against a selectable **REFERENCE**: **Surface** (altitude above the mean radius), **Center** (radius straight from the body center), or **Atmosphere** (altitude above the atmosphere boundary). The Atmosphere option only appears for bodies that have one, and a request below the safe floor is refused.
- **FLYBY SIDE** picks which side of the body you pass, named in the target's own orbital frame: **Inner** (toward its parent), **Outer** (away from it), **North**, or **South**. The aim offset has to stay perpendicular to the approach, so a side whose axis lies along the approach direction cannot be reached and is greyed out.
- Works for moon flybys and for interplanetary targets, either as a single burn or split across multi-pass passes.
- The page reports the approach speed, the impact parameter, the departure delta-V next to the impact-aimed one, and the periapsis the propagated trajectory actually reaches. The side column of the picture shows the flyby from above the target, and **SHOW IN WORLD** draws the retargeted trajectory in the world.
- For multiple passes, the readout uses the selected departure after any schedule shift and the periapsis from the final preview trajectory.
- If propagation cannot confirm a periapsis, the readout shows **No prediction** and its reason, such as a departure time outside the flight plan or no resolved target encounter.

- The preview shows the burn that **CREATE** makes, refined for the finite burn. **CREATE** refuses a flyby that is not armed or has no solution, and says why.
- **Course correction.** After a flyby departure that **CREATE** made to a moon of the craft's parent body, AFC follows the burn, checks the periapsis on the flight plan, and adds up to two small correction burns when the periapsis is off by more than 10 percent (at least 10 km). They stay in Manual, and you fly them with **FLY BURN**. Stock AUTO misses a long departure burn by more at a higher simulation speed, and the correction cancels that miss.

**Limitations:**
- The course correction covers a single flyby of a moon of the craft's parent body, and a save load ends it. For an interplanetary flyby or a multi-pass departure, trim the periapsis with a small correction burn yourself.

## Multi-Pass Burns

Any of the quick-tools and the transfers of the **TRANSFER** page can be split into multiple burns across successive orbits.
Instead of one long burn that sweeps a large arc away from periapsis, the engine fires in shorter bursts near periapsis on each orbit, reducing finite-burn loss.

**Maneuvers that can be split:**
- Hohmann transfers
- Set Periapsis / Set Apoapsis
- Match Inclination / Set Inclination
- Circularize at apoapsis / periapsis

**How to use:**
1. Select a tool on the **MANEUVER** page, or calculate and select a transfer on the **TRANSFER** page, and configure the maneuver as usual. The **PER PASS** choice is in the **GNC** fold of **MANEUVER** and **TRANSFER**.
2. Use the **PASSES** slider to choose how many passes (2-10). The burn bars under the picture show each pass and the stage that pays for it.
3. Click **CREATE n PASSES**. The first pass burn is placed in the burn plan.
4. Click **FLY BURN**, or enable **Auto** on the stock BURN gauge. Each pass fires automatically, with the main engine or as an RCS burn, and the next pass is scheduled after completion.
5. The head of the console shows the running passes and stops them with **STOP PASSES**.

**Why it helps:**
When burn duration is a significant fraction of the orbital period, a single burn wastes fuel by thrusting far from periapsis. Splitting across N passes keeps each burn near periapsis where the Oberth effect is strongest.
Real lunar kick stages use the same technique, several perigee burns instead of one long injection. It helps most for low-TWR spacecraft such as ion engines, small kick stages and nuclear tugs, where a single departure burn can take tens of minutes.

**Hands-free execution:**
Multi-pass works best with the built-in [automatic staging](burns-and-staging.md#automatic-staging) switched on (handles staging between passes) and the built-in [automatic burn removal](burns-and-staging.md#automatic-burn-removal) left on (cleans up completed burns). A multi-pass execution then runs hands-free from first ignition to final departure.

**Limitations:**
- Same-parent transfers (e.g., LEO to Luna) shift the final burn forward by a few parking periods to fit the K-schedule. The shift is shown under **PASSES** on **TRANSFER**.
- A change to the flyby side or altitude updates the selected departure even during thrust.
- Very high-energy departures from small SOIs (e.g., low Mars orbit to Saturn) may auto-clamp to fewer passes because intermediate orbits would escape the SOI.
- A pass that runs out of propellant waits in Manual so you can stage or refuel; if nothing aboard can thrust any more, the execution is cancelled with an alert.

## Hyperbolic Targets

The stock Transfer Planner filters out bodies with eccentricity >= 1. This mod lets both the stock planner and the **TRANSFER** page target interstellar comets (Oumuamua, 2I/Borisov, 3I/ATLAS) by patching the time-of-flight and alignment math to handle unbound orbits. The mod also gives the comets a mass and a sphere of influence, which the planner needs to target a body.
