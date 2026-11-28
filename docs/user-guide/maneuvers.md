# Maneuvers and transfers

## Maneuver quick-tools

The **MANEUVER** page provides single-burn orbit tools:

- **Circularize** - circularize at apoapsis or periapsis.
- **Set Periapsis / Set Apoapsis** - raise or lower one apse with a single burn at the opposite apse.
- **Match Inclination** - plane-change burn at AN or DN to match a target orbit's plane. The target defaults to the craft's current target; **SET AS CRAFT TARGET** in **GNC** updates the craft's target to the selected one.
- **Set Inclination** - plane-change burn at AN or DN to reach an absolute inclination angle. Select **Equatorial** (parent body's equator, matching `Orbit.Inclination`, where KSA CCI +Z is the spin axis) or **Ecliptic** (system plane, KSA CCE +Z). The row starts on **Equatorial**. For Earth the two differ by about 23.4 deg.

When multiple craft are loaded, a **CRAFT** row at the top lets you plan maneuvers for another craft without switching control.

The page defaults to **Circularize**, or **Set Apoapsis** if the orbit is already circular. Target fields start on a rounded value that differs from the current orbit and refresh when the flight state, body or burn plan changes. After **CREATE BURN**, the page shows the created burn with **FLY BURN** and **DELETE BURN** in the footer, plus **NEXT TOOL** to plan another burn on the post-burn orbit. Inside **PARAMETERS**, the **FLIGHT PLAN** link opens the stock flight plan table on **TUNE > DIAG > FLIGHT PLAN**.

When an action finishes, the console head suggests one next step with a link (such as **CIRCULARIZE** after ascent, the next pass after a multi-pass burn, or **MANEUVER** after a burn). The console never switches pages by itself.

### Orbit right-click shortcuts

Right-click the controlled craft's orbit and open **Advanced Flight Computer** to pick a quick-tool. **At Periapsis** offers **AFC: Set Apoapsis...**, and **At Apoapsis** offers **AFC: Set Periapsis...**. These shortcuts open **MANEUVER** with that tool selected; they do not place a burn at the clicked point. The stock Transfer Planner keeps its own plan types, including its own **Circularize**.

### Chaining burns

Quick-tools can plan a single burn after the last planned burn that changes the trajectory (for example, raise apoapsis first, then use **Set Periapsis** on the resulting orbit). The preview and created node use the post-burn trajectory. Multi-pass cannot start on a pending burn's trajectory, and shortcuts wait until an active multi-pass execution completes.

## Transfers

The **TRANSFER** page computes body-to-body transfer windows on an AFC porkchop plot and draws the selected transfer in the map:

- Pick a **DESTINATION** (and **CRAFT** when multiple craft are loaded) and click **SOLVE** (or **RE-SOLVE** after changing settings).
- Click anywhere on the porkchop plot to inspect a departure, or use **SELECT** (**MIN DV** / **MIN ARRIVAL**) and **MOVE PICK** (`^ v < >`) to step across the grid.
- Open the **SEARCH** fold to set **MIN FLIGHT TIME**, **MAX FLIGHT TIME** and **SET AS CRAFT TARGET**.
- If probe battery power will run out before the departure burn or during the coast, AFC warns on **TRANSFER** (and in the **MORE** drawer).

## Flyby targeting

The stock planner aims every transfer at the target body's center, so a well-timed Hohmann arrives as an impact and requires a separate correction burn. Tick **TARGET FLYBY PERIAPSIS** on **TRANSFER** to aim the departure burn directly at a chosen flyby periapsis, then click **CREATE FLYBY DEPARTURE**.

- **Periapsis reference:** Choose **Surface** (altitude above mean radius), **Center** (distance from body center) or **Atmosphere** (altitude above the atmosphere boundary, shown only for atmospheric bodies). Requests below the safe floor are refused.
- **FLYBY SIDE:** Pick which side of the target to pass in the target's orbital frame: **Inner** (toward parent), **Outer** (away from parent), **North** or **South**. Because the aim offset must stay perpendicular to the approach, any side aligned with the approach is greyed out.
- **Scope:** Works for moon flybys and interplanetary targets, as a single burn or split with multi-pass.
- **Readout and preview:** Shows approach speed, impact parameter, departure delta-v beside the impact-aimed value, and the propagated periapsis. The plot sidebar sketches the flyby from above the target, and **SHOW IN WORLD** draws the retargeted trajectory in the world. For multi-pass departures, the readout reflects the scheduled departure and final preview trajectory. If propagation cannot confirm a periapsis, it reports **No prediction** and the reason.
- **Finite-burn refinement:** The preview shows the exact burn **CREATE** will place, refined for finite burn duration. **CREATE** refuses an unarmed or unsolved flyby and states why.
- **Course correction:** After a moon flyby departure created by **CREATE**, AFC monitors the burn, checks the resulting periapsis on the flight plan, and adds up to two small correction burns if periapsis is off by more than 10 percent (at least 10 km). Those corrections stay in Manual for you to fly with **FLY BURN**, cancelling the miss that stock AUTO introduces on long burns at high simulation speeds.

**Limitations:**

- Automatic course correction applies only to single-burn flybys of a moon of the craft's parent body, and a save load ends tracking. For interplanetary flybys or multi-pass departures, trim the periapsis manually.

## Multi-pass burns

Any quick-tool on **MANEUVER** and any transfer on **TRANSFER** can be split into multiple burns across successive orbits. Firing in shorter bursts near periapsis reduces finite-burn steering losses and preserves the Oberth effect, especially for low-TWR craft (ion engines, small kick stages, nuclear tugs).

**Maneuvers that can be split:**

- Hohmann transfers
- Set Periapsis / Set Apoapsis
- Match Inclination / Set Inclination
- Circularize at apoapsis / periapsis

**How to use:**

1. Configure a tool on **MANEUVER** or a solved transfer on **TRANSFER**. The **PER PASS** option lives in the **GNC** fold.
2. Set the **PASSES** slider (`2-10`). The burn bars under the plot show each pass and the stage that pays for it.
3. Click **CREATE n PASSES**. AFC places the first pass burn in the burn plan.
4. Click **FLY BURN** (or enable **Auto** on the stock BURN gauge). Each pass executes automatically (main engine or RCS) and schedules the next pass on completion.
5. Watch progress in the console head and stop early with **STOP PASSES**.

With [automatic staging](burns-and-staging.md#automatic-staging) armed and [automatic burn removal](burns-and-staging.md#automatic-burn-removal) enabled, a multi-pass departure runs hands-free from first ignition to final escape.

**Limitations:**

- Same-parent transfers (such as LEO to Luna) shift the final burn forward by a few parking periods to fit the schedule; the shift is shown under **PASSES** on **TRANSFER**.
- Changing the flyby side or altitude updates the selected departure even during thrust.
- High-energy departures from small SOIs (such as low Mars orbit to Saturn) may auto-clamp to fewer passes so intermediate orbits do not escape the SOI.
- If a pass runs out of propellant, it pauses in Manual so you can stage or refuel; if nothing aboard can thrust, execution cancels with an alert.
- A multi-pass transfer needs a near-circular parking orbit (eccentricity at most 0.05).
- After a load, a saved multi-pass run whose burn is missing, or was edited out of tolerance, is cancelled with a warning.

## Hyperbolic targets

The stock Transfer Planner filters out bodies with eccentricity `>= 1`. AFC patches the time-of-flight and alignment math so both the stock planner and the **TRANSFER** page can target interstellar comets (`Oumuamua`, `2I/Borisov`, `3I/ATLAS`), and assigns them the mass and sphere of influence the planner requires.
