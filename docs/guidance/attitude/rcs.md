# RCS mixing

`KsaRcsTorqueControl` gives a craft a torque from its RCS thrusters, the cheapest way its layout allows.
A caller asks for a torque; the mixer solves `RcsTorqueAllocator` for each thruster's duty on the main thread, and the vehicle worker turns the duties into pulses.
It works for any layout and any mode, and it does not go through the stock flight computer's RCS logic at all.

The attitude controller, entry and boostback glides and touchdown guidance all fly their RCS torque through it.
The shared problem it solves is set out in [The common torque-allocation problem](index.md#the-common-torque-allocation-problem).

## The thrusters as effectors

`KsaRcsTorqueControl.Measure` lists the craft's staged thrusters from its parts and measures each one from its own nozzles (`RcsWrenchTable.ComputeLive`), at the live centre of mass and the ambient pressure now.
For thruster $i$ at full thrust it records

- the force $\mathbf f_i$, N;
- the torque about the centre of mass $\mathbf t_i = \mathbf r_i \times \mathbf f_i$, N m;
- the propellant flow $\dot m_i$, kg/s.

All three are projected onto the caller's frame, which is the model body axes for every caller today.
A thruster with no flow, no force or no torque is left out.
Because the measurement is redone every step, a craft that burns down its propellant, stages or loses a thruster is allocated on what it has now.

## The programme

The command is each thruster's **duty** $d_i$, the share of the control period it fires.
In the terms of the common problem, $\mathbf u = \mathbf d$, $B = [\mathbf t_1 \cdots \mathbf t_n]$, the bounds are $0 \le d_i \le 1$, and the cost of a command is its propellant:

$$
\begin{aligned}
\min_{\mathbf d,\,\mathbf e^{\pm}}\quad & \sum_i \dot m_i\, d_i + \sum_a p\,w_a\,(e^{+}_a + e^{-}_a) \\
\text{s.t.}\quad & \sum_i \mathbf t_i\, d_i + \mathbf e^{+} - \mathbf e^{-} = \boldsymbol\tau \\
& d_i + s_i = 1,\qquad \mathbf d,\ \mathbf s,\ \mathbf e^{\pm} \ge 0
\end{aligned}
$$

| Common term | RCS |
| --- | --- |
| commands $\mathbf u$ | duties $d_i$, one per thruster |
| effectiveness $B$ | each thruster's full-thrust torque $\mathbf t_i$ |
| bounds | $0 \le d_i \le 1$, through a slack $s_i$ |
| command cost $\mathbf c$ | full-thrust flow $\dot m_i$, kg/s |
| shortfall price $p$ | $10^3 \times \max_i \dot m_i / \lVert\mathbf t_i\rVert$: a thousand times the dearest thruster's propellant per newton metre |
| priorities $w_a$ | $(1, 1, 0.3)$ from every caller: pitch and yaw before roll |
| side force | $\sum_i \mathbf f_i d_i$, left free and reported |

The programme has $3 + n$ rows (three torque rows and one bound per thruster) and $2n + 6$ columns (duties, slacks and six shortfalls).
`RcsLpSolver.Solve` runs it with a two-phase simplex.
Duties under $10^{-4}$ are dropped: they would ask for a puff shorter than any thruster's minimum pulse.

This is the jet-select programme of Bergmann and Weiler (AIAA 1983-2209), which AFC's RCS executor already runs for translation, with force and torque swapping roles and an upper bound on each duty.

### The free side force

The side force is not a constraint.
Thrusters at the top of a booster cannot turn it without pushing it sideways: each newton metre of pitch comes with about $1/h$ newtons of push, $h$ being the thrusters' height over the centre of mass.
Forbidding that push would leave such a craft no pitch at all.
The mixer reports it instead (`Report.ForceN`), and touchdown guidance carries it in its plan.

### Which axis gives way

The shortfall is priced so high that the mixer gives every newton metre it can before it leaves any unmet.
When the demand is out of reach, the priorities decide what gives: roll, at 0.3, goes first.
A large pitch demand with a little roll on top is met in pitch, and the roll waits.

<!-- viz: rcs-mixing 620 -->
*Move the torque demand and watch which thrusters fire and for how long, what side force comes with it, and where roll gives way when the demand leaves the attainable set.*

## What it can reach

`RcsTorqueAllocator.Characterise(axis, sign)` is the common problem's characterise step.
It maximises $t$ subject to $\sum_i \mathbf t_i d_i = \sigma\,t\,\hat{\mathbf e}_a$ and $0 \le d_i \le 1$, with a cost of $10^{-6}/(\max_i \dot m_i/|\mathbf t_i|)$ per kg/s so that a tie goes to the cheaper pattern.
It then allocates half that reach and reports, per newton metre, the propellant flow and the side force.
`KsaRcsTorqueControl.TryCharacterise` runs it for all six directions, and `Ksa6DofSetup.RcsModel` turns them into touchdown guidance's RCS model: each axis's two bounds, and the side force and propellant per newton metre.

The plan therefore uses the same thrusters, measured the same way, as the mixer that flies it.

## Pulses

A thruster is on or off.
The vehicle worker turns each duty into a pulse in `KsaRcsTorqueControl.OnComputeControl`, which runs from `VehicleCommandSink.Run` after stock's `FlightComputer.ComputeControl`:

1. The control period is $T = 0.1$ s (`PeriodSec`).
2. At the start of each period, thruster $i$ wants $d_i T + c_i$ seconds of firing, $c_i$ being what it carried from the last period.
3. If that is at least the thruster's own minimum pulse (`ThrusterController.MinimumPulseTime`), it fires for $\min(d_i T + c_i,\ T)$ and carries any excess, up to one period.
4. If it is shorter, it fires nothing and carries the whole amount.
5. During the period each firing thruster gets the time it has left, written into `ThrusterControllerState.CommandPulseTime` as the longer of stock's pulse and this one.
6. The worker asks for its next control step at the start of the next period, no sooner than 10 ms. Asking for a step at the end of each pulse, as a first version did, made the game take extra physics steps and stutter.

So a small duty comes out as an occasional pulse with the right average, not a minimum pulse every period and not nothing.
A thruster without propellant is skipped.

## Outside the stock flight computer

Nothing here reads the stock flight computer's RCS mode, thruster configuration, authority figures or attitude modes.
The thrusters come from the craft's parts, their thrust and torque from their nozzles, and the mixer's state is keyed on the thruster modules.
The one point of contact is where the pulses are delivered: stock rewrites every thruster's pulse on every control step, zero when it fires nothing, so this writes after it.

That means:

- The stock RCS switch does not stop it. Touchdown guidance has its own switch, `SixDofRcs` (**RCS TORQUE**), and a landing burn that the stock flight computer steers keeps the stock RCS switched off on purpose.
- A craft that is not controllable, or whose control the kFS vehicle computer owns, fires nothing.
- Duties solved for one set of thrusters find no match after staging, or when a thruster runs dry, and fire nothing until the next command. `KsaRcsTorqueControl.Reached` reports how many thrusters the worker matched; zero with a command standing means the pulses reach nothing.
- `KsaRcsTorqueControl.Release` stops the writes, and stock's own command stands from the next step.

See [Vehicle control writes](../../control-ownership.md#active-rcs-control) for the other RCS writer, the RCS executor.

### Why not stock's RCS

Stock steers one axis at a time.
Every thruster tagged to an axis fires for the same time, the axis's impulse over its authority, and only the tagged part of each thruster's torque counts (`FlightComputer.SelectJetsToFire`).
It cannot be asked for a torque, it ignores what a thruster does to the other axes and the side force it brings, and it fires every tagged thruster whether or not a cheaper set would do.

## What you see in flight

- **On a glide, the RCS fires only for what the fins cannot give.** It shares one programme with the fins (see [Fin mixing](fins.md#fins-and-rcs-in-one-programme)), so it fires when the fins are at their stops or still travelling, when the air is too thin for the fins, or when the craft is more than 20 deg off engine first.
- **A booster's top thrusters push it sideways as they turn it.** Touchdown guidance plans with that push; the glides' attitude controller does not model it.
- **Small, steady demands come out as occasional pulses**, not a constant puffing.
- **Roll is the first to give way** when a demand is too big.

## Code map

| File | Main type or method |
| --- | --- |
| [RcsTorqueAllocator.cs](../../../AdvancedFlightComputer/Features/RcsTorque/RcsTorqueAllocator.cs) | `RcsTorqueAllocator.Allocate`, `Characterise` |
| [KsaRcsTorqueControl.cs](../../../AdvancedFlightComputer/Features/RcsTorque/KsaRcsTorqueControl.cs) | `KsaRcsTorqueControl.Measure`, `Command`, `TryCharacterise`, `OnComputeControl`, `Release` |
| [RcsLpSolver.cs](../../../AdvancedFlightComputer/Features/RcsTranslation/RcsLpSolver.cs) | `RcsLpSolver.Solve`, two-phase simplex with Bland's rule |
| [RcsWrenchTable.cs](../../../AdvancedFlightComputer/Features/RcsTranslation/RcsWrenchTable.cs) | `RcsWrenchTable.ComputeLive`, a thruster's live force, torque and flow |
| [Ksa6DofSetup.cs](../../../AdvancedFlightComputer/Features/Guidance/Adapters/Scvx/SixDof/Ksa6DofSetup.cs) | `Ksa6DofSetup.RcsModel`, `RcsModelFrom` |
| [SixDof.cs](../../../AdvancedFlightComputer/Features/Guidance/Modes/SixDof.cs) | `GuidanceWindow.FlyPlanRcs` |
| [AttitudeController.cs](../../../AdvancedFlightComputer/Features/Guidance/Control/AttitudeController.cs) | `AttitudeController.Step`, which fires the RCS for what the fins leave |
