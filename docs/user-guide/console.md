# The AFC console

Every AFC feature that you start, monitor or stop lives in one window: the **AFC console**.

## Opening the console

You can open the console from several places:

- Press **F4** or select **View > Transfer Planner** (opens where the stock Transfer Planner opens).
- Open the **AFC** menu in the top bar (includes one item per page and **RELEASE ALL**).
- Open **Settings > Mods** and click **OPEN AFC**.
- Use AFC links next to stock UI: the orbit right-click menu, the delay block in a part window, or the RCS block under the burn editor.

The stock Transfer Planner remains intact. Click **PLANNER** (with the STOCK chip) on the **TRANSFER** page to open it, for example for Interstellar plans.

To give **F4** and **View > Transfer Planner** back to the stock planner, switch **F4 OPENS AFC** off on **TUNE > PREFERENCES** or in the **AFC** menu. The **AFC** menu still opens the console. In flight with no controlled craft, **F4** opens the stock planner, and while the stock planner is open the console steps aside. If the console fails to draw repeatedly (for example after a game update), AFC hands **F4** back to the stock planner automatically for the rest of the session.

In the **Editor** and during **EVA**, only the pages that apply are active (`STAGING`, `TUNE > PREFERENCES`, and the stock `PLANNER` link on `TRANSFER`).

## Pages

One row of tabs under the head lists the seven pages in mission order, with a divider before the two setup pages. A dot after a tab name means something is running on that page.

- **LAUNCH** - Ascent planning and execution: **ASCENT** and **CLIMB** settings, target orbit, launch window and the flight profile plot with the convex plan, flown track and staging ticks. **REACH** shows the plane-change cost from the launch site, and **TWR** warns when the first stage cannot lift off. **THEN** picks what follows the ascent (*Stay in orbit*, moon flyby or moon orbit; see [Launch to a moon](powered-guidance.md#launch-to-a-moon)). The **BOOSTER RETURN** section under the plot sets the **SITE** (or **OPTIMAL RETURN**) for each returnable stage and shows the **RESERVE** sized by the solved climb.
- **MANEUVER** - Orbit quick-tools with an orbit or plane sketch, burn bars and multi-pass previews.
- **TRANSFER** - Transfers to other bodies on an AFC porkchop plot, with multi-pass splitting, flyby targeting and moon course corrections.
- **BURNS** - Execution method for planned burns (main engine or RCS), RCS attitude strategy, propellant estimates, warp to the turn and **AUTO REMOVE FINISHED BURNS**.
- **LAND** - Six ways down in mission order: **DEORBIT** (aimed entry burn then orbital entry in air, or braking descent on airless bodies), **BOOSTBACK** (return a spent booster to its site), **ENTRY/BRAKING** (orbital entry in air, or braking burn on airless bodies, without a deorbit burn first), **GLIDE** (atmospheric descent via **FLIP LANDING** or **RETRO BURN**), **LAND NOW** (powered descent from the current state) and **HOVER**. Every way shares the landing site and shows a clickable path strip whose steps open their settings inline. Under the site, **LAND** shows the ground slope and the craft's tip-over slope, and **FIND FLAT** searches nearby for flatter ground. The **LEGS** fold deploys landing legs by time to touchdown (`DEPLOY TIME` + `BUFFER`). **STEER BURN WITH GIMBAL** (in **LAND > DEORBIT > GNC**) switches RCS off during gimballed landing burns so the gimbal steers alone.
- **PILOT strip (stock gauge panel)** - Sits above the stock Autopilot Settings panel with **STOCK**, **AFC MANUAL INTERVENTION**, and **RATE** (`30%`, `50%`, `80%`). In **AFC MANUAL INTERVENTION**, pitch/yaw/roll keys command turn rates through AFC's attitude controller (rate command, attitude hold), stop any running AFC programme or stock **AUTO** burn, and hold the stock flight computer in **MANUAL**.
- **STAGING** - AutoStage switches and per-sequence / per-part staging delays.
- **TUNE** - Guidance tuning parameters, solver diagnostics (**DIAG**) and AFC preferences (**PREFERENCES**). Each tab shows the count of non-default values, and each changed row shows its default and a **RESET** action.

## Head, drawer and footer

Every page shares the same frame:

- **Idle head:** When nothing runs, one row shows the craft state (**LANDED**, **ORBIT**, **ASCENT**, **COAST**, **FALL**, **DESCENT**, or **ESCAPE**) and the current body.
- **Active head:** While something runs, the head shows who flies the controlled craft (**AFC** in blue, **STOCK AUTO** outlined, **AFC PLANS + STOCK AUTO** with both, or **MANUAL** in grey) plus a single stop button at the right. The stop acts on the highest-priority active item: a guidance mode first, then an RCS burn, then a multi-pass run, then a stock AUTO node. Main numbers and the current phase sit directly below.
- **Messages:** A message row appears only when there is a message and clears when a new mode starts or a commit button is pressed. Once nothing flies, a note clears from the head after 15 s and a warning after 30 s; the drawer keeps the history. Solver logs stay on **TUNE > DIAG**. A persistent warning row shows whenever **AFC MAY STEER** or **AFC MAY USE ENGINES** is switched off.
- **MORE drawer:** Click **MORE** in the head to open the drawer with the craft's **GNC SWITCHES** (**AFC MAY STEER**, **AFC MAY USE ENGINES**, **AUTOSTAGE**), individual stop buttons for everything running, **RELEASE ALL**, other craft currently under AFC control and the message history.
- **Inputs and folds:** Settings that change *what* the craft does sit directly on the page. Settings that tune *how* it flies live in short folds such as **GNC**, **LIMITS** or **LEGS**, whose header summarizes active values. **PARAMETERS** holds readouts, and controller gains live on **TUNE**. Every setting tooltip ends with how long that value is kept.
- **Footer:** The footer states what is true now, describes in one sentence what the commit will do, and provides one full-width commit button (for example **LAUNCH**, **BOOSTBACK**, **CREATE BURN**, **DEORBIT + LAND** or **TOUCHDOWN**). The answer line ends by naming who will fly after the click. On **MANEUVER**, **TRANSFER** and **BURNS**, **DELETE BURN** sits next to **FLY BURN**. A commit that would take the craft from another running guidance mode asks for confirmation first.

All pages remain readable in every flight state:

- A gold commit button fits the current flight state.
- An outlined commit button shows why it does not fit (for example **LAUNCH** in orbit shows "Not on the ground", or **CREATE TRANSFER** shows the delta-v shortfall). You can still press it unless a refusal blocks it, and a link beside it jumps to a better-fitting page.
- While a page's own mode is flying, its commit button is hidden and replaced by live status.
- On **TRANSFER**, the commit button is **SOLVE** until a porkchop plot exists.
- On **LAND**, a small ring highlights the way down that fits the current state. Opening the console for the first time while falling lands directly on **LAND**.

## Window layout and strip

Window height adjusts to the active page with the top-right corner pinned. It grows immediately, and shrinks only on a page change or when a fold closes, so live readouts do not make the footer jump. By default it stops above the stock autopilot panel (shrinking the plot first, then scrolling); dragging the resize handle overrides the maximum height.

Click the expand icon beside **X** to toggle **WIDE** mode (plot on the left, inputs on the right). Pages without a plot, such as **TUNE**, keep the narrow layout.

Close the console with **X** or **F4**. While AFC is flying the craft, the head stays on screen as a compact strip with the stop button; click **OPEN AFC >** at the bottom left of the strip to reopen the page of the running mode.

The active page, **WIDE** state and window position are saved in `ui.toml` in the mod folder. Use **RESET WINDOW POSITION** in the **AFC** menu or **WINDOW POSITION > RESET** on **TUNE > PREFERENCES** to dock the console again. **TUNE > PREFERENCES** also holds **OPEN AFC AT FLIGHT START**, **FOLDS OPEN**, **SHOW STAGING DIAGNOSTICS**, **F4 OPENS AFC**, **GUIDANCE ENABLED** and **NUMPAD NUDGES THE HOVER**.

## Failure and separation bands

- **Red failure band:** Appears under the stock altitude counter even when the console is closed if a guidance mode gives up while the craft still needs flying, or stays without a plan for 10 s. It shows what happened, what the engine is doing, who has control and the time to impact when known. It stays until you click **OK**. Normal completions, player or stock AUTO takeovers, and brief no-plan states do not trigger the band.
- **Green separation band:** Appears under the stock altitude counter (or below the red failure band if both are active) in the final seconds before a stage separates (`SEPARATION T-`), pulsing just before separation, even with the console closed.
