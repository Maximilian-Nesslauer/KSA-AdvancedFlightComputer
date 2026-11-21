# Gimbal mixing

`KsaTvcAllocator` turns a body torque in newton metres into a deflection for every engine gimbal.
Touchdown guidance flies its plan's torque through it, together with its attitude feedback's share.
The attitude controller on main does not use it: it flies a craft whose engine is off.

The stock flight computer has its own allocation (`TvcAllocation.Solve`), but it takes a normalised demand per axis, not newton metres.
A 6-DOF plan needs to command a torque it trusts and to know what the gimbals can give, so AFC has its own.
The shared problem is set out in [The common torque-allocation problem](index.md#the-common-torque-allocation-problem); the gimbals solve it without a linear programme.

## Where it runs

`KsaGimbalControl.SetLsq` publishes a torque for one vehicle on the main thread.
The vehicle worker applies it in `KsaGimbalControl.OnComputeControl`, from `VehicleCommandSink.Run`, after stock's `FlightComputer.ComputeControl`, so it is the last word on the gimbals whatever the attitude mode.

- It reads each gimbal's thrust from the last step's state, as stock does. An unlit gimbal gets zero thrust and no share while another is lit. If none is lit, every gimbal stands in at its nameplate `Data.MaximumThrust`, so the allocation can be previewed on the pad and an off-centre engine can be put at its trim before ignition.
- It writes `CommandY` and `CommandZ`, each a fraction of the gimbal's range in $[-1, 1]$, and sets `NaturalFrequencyLimit` to infinity so the gimbal follows at its own natural frequency rather than at a limit stock left in the state.
- It writes nothing while guidance is switched off, the craft is not controllable, or the kFS vehicle computer owns it. See [Vehicle control writes](../../control-ownership.md#guidance-control).

## Effectiveness: lever arms and the thrust moment

Each gimbal has two commands, $u_{2i}$ and $u_{2i+1}$, one per axis.
For command $(u_y, u_z)$ the thrust direction is the rest direction turned by the deflection, $\hat{\mathbf d}(u) = R(u)\,\hat{\mathbf d}_0$ with angles $u_y\theta_{y,\text{max}}$ and $u_z\theta_{z,\text{max}}$, and the thrust acts at the pivot plus the nozzle's offset, swung with it:

$$
\boldsymbol\tau_i(u) = \big(\mathbf p_i + R(u)\,\boldsymbol\ell_i - \mathbf r_\text{cm}\big) \times T_i\,R(u)\,\hat{\mathbf d}_0
$$

$\mathbf p_i$ is the pivot, $\boldsymbol\ell_i$ the offset from pivot to thrust point, $\mathbf r_\text{cm}$ the centre of mass and $T_i$ the thrust.
`KsaTvcAllocator.FillColumns` differentiates this whole, by central differences in each command, to get two columns of $B$ (N m per unit command) and two of the force Jacobian $B_f$ (N per unit command):

$$
B_{:,c} = \frac{\boldsymbol\tau_i(+h\,\hat{\mathbf e}_c) - \boldsymbol\tau_i(-h\,\hat{\mathbf e}_c)}{2h},
\qquad h = 0.5
$$

The step is half the range: large enough to beat float noise, small enough that the slope error at a 10° gimbal is under 0.2 %.

The swing of the thrust point matters.
It moves by the nozzle's length times the deflection while the full thrust acts on it, which is the same order as the side force, and the two together act on the arm to the **pivot**: the nozzle's length cancels.
Taken at the rest thrust point instead, the arm was a nozzle too long, and a booster's torques came out 1.37 times what its engines delivered.

`B` is an incremental model about rest.
The torque the craft gets is the engines' **rest moment** plus what the allocator was asked for.
For a balanced cluster the rest moment is zero.

## The solve

The allocator solves the regularised minimum-norm problem in closed form:

$$
\mathbf u = B^\mathsf{T}\big(BB^\mathsf{T} + \lambda I_3\big)^{-1}\boldsymbol\tau,
\qquad \lambda = 10^{-6}\,\operatorname{tr}(BB^\mathsf{T})
$$

$BB^\mathsf{T}$ is only 3×3, so this is cheap enough for every control step.
Minimum norm spreads effort across the gimbals in proportion to their effectiveness: the verniers take roll and the main engine pitch and yaw, with no setting per craft.
The small $\lambda$ keeps an axis with no authority at all, such as roll on a single central engine, from making the inverse blow up; an unreachable torque then gives a small command, not a huge one.

### The weak axis gives way

The gimbals' travel is one budget.
Roll is made by deflecting them against each other, pitch and yaw by deflecting them together, and roll is often hundreds of times weaker: one craft measured 2.8 kN m of roll against 1902 kN m of pitch.
Scaling the whole answer to fit then let a roll demand a few percent over its limit cut pitch and yaw to a third.

So any axis whose capability is under a quarter of the strongest's (`WeakAxisShare`) is the **weak axis**.
It is clamped to its capability and allocated **last**, in the travel the other two leave: the largest share of its own solution that keeps every gimbal inside its range.
This is the gimbals' form of the common problem's axis priorities.

### Redistribution at the stops

If any command exceeds 1, the whole strong-axis solution is first scaled to the first gimbal at its stop, which keeps the torque's direction.
Then `KsaTvcAllocator.Redistribute` carries on (a cascaded generalised inverse):

1. Hold every gimbal axis at its stop.
2. Solve the part of the strong axes' demand still missing on the gimbal axes left free, with the same regularised inverse.
3. Move the free axes along that solution until the next one reaches a stop or the demand is met.
4. Repeat.

Without this the smallest actuator set the whole craft's torque.
On one landing on one engine of three, the main gimbal sat at a third of its travel while a 70° vernier was at its stop, the allocation delivered 51–69 % of the demand for the last 1.5 s, and the craft touched down at 4.5 m/s sideways.

The result (`TvcAllocationResult`) reports the torque and side force the linear model says the commands give, the saturation scale, and the share of the weak axis that fitted.

## Off-centre engines

A landing on part of a stage's engines can put the thrust off the centre of mass.
`KsaThrustMoment.Rest` measures the engines' moment with every gimbal at rest, at the same thrust as `Data.MaximumThrust`, so it is on the gimbals' scale; moment over force is the moment per newton at any throttle.
One engine of a ring 0.8 m out on a 10 m arm needs about 4.6° of a 6° gimbal just to hold it.

The rest moment per newton, $\mathbf m$ (`Ksa6DofSetup.ThrustMomentPerNewton`), sets two things.

**The trim.** The deflection that holds the engines' own moment needs a torque per newton of $(-m_x, -m_y, 0)$ (`Ksa6DofGuidance.TrimTorquePerNewton`).
Touchdown guidance asks for it in its torque from 3 s before ignition (`TrimPrepositionLeadS`), so the nozzles are there when the engine lights.

**The coupled roll.** An engine off the axis rolls the craft as it steers.
With $L$ the arm from the centre of mass to the gimbal pivots along the thrust axis, a lateral torque $(\tau_x, \tau_y)$ brings a roll of

$$
\tau_{z,\text{coupled}} = \frac{m_x\,\tau_y - m_y\,\tau_x}{L}
$$

`Ksa6DofGuidance.CoupledRollNm` adds it to the demand, and `SetLsq` names it apart as `coupled`.
The allocator then solves it **with** pitch and yaw rather than treating it as weak-axis demand.
Otherwise the minimum-norm step would hold roll at zero with the verniers, the weak step would put it back, and the verniers' stops would be the whole craft's limit.

The landing engine picker marks a choice whose thrust centre is more than 5 cm off the cluster's axis as off axis (`LandingEngineLayout`).
In flight, `LandingEngineSelection` lands on all engines instead when the moment is over 80 % of what the gimbals can hold against it (`KsaThrustMoment.LateralAgainstGimbals`).

## Range and slew

- **Range.** Each gimbal axis's command is a fraction of its `MaxAngle`, so the bound is $|u| \le 1$. The smallest range on the craft is `Ksa6DofSetup.GimbalLimitDeg`.
- **Slew.** The allocator assumes the gimbal goes where it is told. The game moves it there at its natural frequency, with a rate cap of `Gimbal.RATE_CAP_PER_NATURAL_FREQUENCY` ranges per second per unit of natural frequency. Touchdown guidance's plan holds its lateral thrust to that rate (`Ksa6DofSetup.GimbalSlewPerS`, `Scvx6DofConfig.GimbalSlewPerS`), and its attitude feedback's 1.5 rad/s bandwidth sits well under a full deflection in half a second.

`KsaGimbalControl.Slot.Trace` logs the gimbal with the most thrust: an angle trailing its command by the slew cap is the gimbal's rate limit, and one on its command with the torque still short is the allocation.

## The trim-centred cone

Touchdown guidance plans the lateral thrust $(t_{dx}, t_{dy})$ inside a cone, centred on the trim rather than on zero (`Scvx6DofConfig.GimbalTrimX`, `GimbalTrimY`):

$$
\big\|(t_{dx}, t_{dy}) - T\,(g_x, g_y)\big\| \le T\tan\theta_\text{eff},
\qquad g_x = \frac{m_y}{L},\quad g_y = -\frac{m_x}{L}
$$

The cone's width $\theta_\text{eff}$ is what the allocator really delivers, not the hardware's nameplate range:

1. Ask the allocator for far more than it has, along each lateral axis (`Ksa6DofSetup.TorqueAlong`), and take the smaller: the **zero-roll** width, `LateralTorqueLimit`.
2. With the trim off the axis, ask again with the coupled roll named (`CoupledLateralTorqueLimit`). The width becomes the larger of the zero-roll width and that capability less the trim, never past the hardware's range beyond the trim.
3. Shrink it to the plan's share, $\tan\theta_\text{eff} \to (1 - \text{margin})\tan\theta_\text{eff}$, so the attitude feedback has room (`SixDofTorqueMargin`, 20 %).

Centred on zero, one 10° engine of three needed 324 % of the cone just for its trim: every plan asked for a deflection the craft could not have.

## How it fits the common problem

| Common term | Gimbals |
| --- | --- |
| commands $\mathbf u$ | two normalised deflections per gimbal |
| effectiveness $B$ | central-difference Jacobian of the torque about the centre of mass, from rest |
| bounds | $-1 \le u \le 1$ per gimbal axis |
| solve | regularised minimum norm, then scale and redistribute at the stops; no linear programme and no costs |
| priorities | the weak axis is allocated last, in the travel left; a coupled roll goes with the strong axes |
| side force | $B_f\,\mathbf u$, reported (`AchievedForce`) |
| characterise | `TvcAllocationResult.MaxTorque` (each axis's minimum-norm solution driven to its first stop) and `Ksa6DofSetup.TorqueAlong` (over-ask and read back) |

The attainable set is the gimbal box mapped through $B$, a zonotope.
The minimum-norm solution scaled to its first stop reaches only part of it; redistribution reaches further, towards its edge.
That is why touchdown guidance sizes its cone with `TorqueAlong` and not with `MaxTorque`: on one booster `MaxTorque` read twice what the allocator delivered in any one direction, and the plan's torques were scaled to about half at every step.

<!-- viz: gimbal-mixing 620 -->
*Choose which engines are lit, move the torque demand, and see each gimbal's deflection, the stops being hit one by one, and the roll an off-centre engine brings.*

## What you see in flight

- **A balanced cluster** gimbals together for pitch and yaw and against each other for roll, the verniers doing most of the roll.
- **One engine off the axis** sits at its trim deflection from just before ignition, steers around it, and rolls the craft a little as it does; the plan expects that roll.
- **A gimbal at its stop** holds there while the others carry on, so the torque does not fall to what the weakest gimbal could give.
- **A roll demand too large for the gimbals** is cut back on its own; pitch and yaw stay whole.

## Code map

| File | Main type or method |
| --- | --- |
| [TvcAllocator.cs](../../../AdvancedFlightComputer/Features/Guidance/Control/TvcAllocator.cs) | `KsaTvcAllocator.Solve`, `FillColumns`, `Redistribute`; `TvcAllocationResult` |
| [GimbalControl.cs](../../../AdvancedFlightComputer/Features/Guidance/Control/GimbalControl.cs) | `KsaGimbalControl.SetLsq`, `ApplyLsq`, `PlanningGimbals`, `LitGimbalThrust` |
| [ThrustMoment.cs](../../../AdvancedFlightComputer/Features/Guidance/Control/ThrustMoment.cs) | `KsaThrustMoment.Rest`, `LateralAgainstGimbals` |
| [LandingEngineLayout.cs](../../../AdvancedFlightComputer/Features/Guidance/Control/LandingEngineLayout.cs) | `LandingEngineLayout.Build`, the landing engine picker's off-axis check |
| [EnginePerf.cs](../../../AdvancedFlightComputer/Features/Guidance/Control/EnginePerf.cs) | `KsaEnginePerf.ThrottleForThrust`, the throttle for the plan's thrust that sets each gimbal's thrust |
| [Ksa6DofSetup.cs](../../../AdvancedFlightComputer/Features/Guidance/Adapters/Scvx/SixDof/Ksa6DofSetup.cs) | `Ksa6DofSetup.TorqueAlong`, `LateralTorqueLimit`, `CoupledLateralTorqueLimit`, `ThrustMomentPerNewton`, `GimbalSlewPerS` |
| [Ksa6DofGuidance.cs](../../../AdvancedFlightComputer/Features/Guidance/Adapters/Scvx/SixDof/Ksa6DofGuidance.cs) | `Ksa6DofGuidance.Command`, `CoupledRollNm`, `TrimTorquePerNewton` |
| [Scvx6DofConfig.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/Scvx6DofConfig.cs) | `GimbalTrimX`, `GimbalTrimY`, `GimbalMaxDeg`, `GimbalSlewPerS` |
| [SixDof.cs](../../../AdvancedFlightComputer/Features/Guidance/Modes/SixDof.cs) | `GuidanceWindow.ApplyPlanCommand` |
