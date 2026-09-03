# Steering on the drag-integrated impact point

What `ImpactPredictor.VelocityJacobian` and `ImpactSteering.Correction` actually
compute, and how the pseudo-inverse law compares with using the Jacobian alone for
instantaneous greedy guidance.

All numbers below are measured by `Scvx.Console --impact` on a boostback-shaped
reference arc: 90 km altitude, $v_0 = (-350,\ 1500,\ 0)$ m/s, KSA-Earth gravity and
atmosphere, an F9-class booster held retrograde.

---

## 0. The idea in words

`J` answers the **forward** question: *push the velocity by $\Delta v$, and the impact
point moves by $J\,\Delta v$.* Its entries are metres of impact per m/s of velocity.

What we want is the **backward** question: *what push moves the impact by $-m$, so the
miss goes to zero?* That is just a linear system:

$$
J\,\Delta v = -m .
$$

If `J` were an ordinary invertible matrix that would be the end of it —
$\Delta v = -J^{-1}m$, one shot, nothing clever. **Everything complicated below exists
because `J` is not invertible.**

### Why it isn't invertible

Three velocity components go in, but the impact point can only move in **two**
independent ways.

Nothing is being *constrained* here, and it is worth being careful about that. We chose
to report the point where the trajectory **crosses** radius $R$ — and a crossing point
of a sphere is on that sphere. So $\lVert p\rVert = R$ for every initial state, not
because anyone imposed it, but because that is what the output *is*. A function whose
values all lie on a sphere has a derivative that lands in the sphere's tangent plane,
and three inputs onto two output directions cannot be invertible.

Report something else — position at a fixed time, say — and the structure disappears
along with the problem.

Break `J` into its independent input-output pairs and measure the **gain** of each —
how many metres the impact moves per m/s of push along that direction:

| direction | gain |
|---|---|
| 1 | 129 m per m/s |
| 2 | 102 m per m/s |
| 3 | **0** |

That third one is the whole problem: there is a direction you can push the velocity in
that moves the impact point *not at all*. Two consequences:

- **Not every miss is reachable.** You can only produce what directions 1 and 2 can
  produce, so in general you get as close as possible rather than exactly there.
- **The answer is not unique.** For a miss you *can* fix, add any amount of direction 3
  and you get the same impact point. So infinitely many $\Delta v$ work.

### "Minimum $\Delta v$" is the answer to the second point

Among all the pushes that get you as close as possible, take the **shortest one**. That
is the whole definition, and it is what a pseudo-inverse computes. It puts zero along
direction 3, because spending fuel there buys nothing.

### Why the formula gives you the shortest one for free

You have three knobs: $v_x$, $v_y$, $v_z$. But one particular *combination* of them —
here $(0.79,\ -0.61,\ 0)$ — is a **dud**: turn it and the impact point does not move at
all. Measured, it moves the impact by $7\times10^{-15}$ m per m/s.

So of your three knobs, two do something and one is wasted fuel. "Shortest $\Delta v$"
just means: **don't touch the dud.**

Any push splits into a useful part and a dud part, and those two are at right angles, so

$$
\text{total}^2 = \text{useful}^2 + \text{dud}^2 .
$$

The useful part is *fixed* — it is the bit that actually moves the impact where you want
it, so you have no choice about it. The dud part is *free*. Setting it to zero is
obviously best, and that is the whole of "minimum $\Delta v$".

Now the one piece of structure worth knowing: writing the answer as

$$
\Delta v = -J^{\top} y
$$

**cannot** contain any of the dud, whatever $y$ is. The columns of $J^{\top}$ span
exactly the useful directions, so anything built out of them has zero dud component
automatically. Measured on the real answer: the dud component is $8\times10^{-17}$ of
its length.

That is why the formula has a $J^{\top}$ on the outside. It is not decoration and it is
not there to make the algebra work — **it is the minimisation**. Nothing in the code
searches for a minimum, because the form of the answer already guarantees it.

The rest is bookkeeping: put $\Delta v = -J^{\top}y$ into $J\,\Delta v = -m$, get
$(JJ^{\top})\,y = m$, solve that little $3\times3$ for $y$, multiply back by $-J^{\top}$.

### Doing it without dividing by zero

Naively you would take each direction's share of the miss and divide by its gain. That
works for gains of 129 and 102, and explodes for a gain of 0. So instead of $1/\text{gain}$
use

$$
\frac{\text{gain}}{\text{gain}^{2} + \lambda^{2}} ,
$$

which is very nearly $1/\text{gain}$ when the gain is large, and falls to zero when the
gain is small. At $\lambda = 1$:

| gain | $1/\text{gain}$ | damped |
|---|---|---|
| 129 | 0.00774 | 0.00774 |
| 102 | 0.00984 | 0.00984 |
| 0 | $\infty$ | **0** |

$\lambda$ is a "don't bother" threshold in the same units as the gains — metres of
impact per m/s. Setting it to 1 says: *ignore any direction that moves the impact less
than about a metre per m/s of fuel.*

In code this is a $3\times3$ solve, $(JJ^{\top} + \lambda^{2}I)\,y = m$ followed by
$\Delta v = -J^{\top}y$. No inverse is ever formed.

### Why it still takes a few goes

`J` is measured where you are **now**, and it is a straight-line stand-in for a
relationship that curves. It is accurate for small changes and less so for a 21.5 km
correction. So: take the step, re-measure `J`, repeat. In practice
21.5 km → 1.35 km → 5 m → 0.

### And the greedy alternative, in the same words

Greedy uses `J` the other way round: $J^{\top}m$ says which push makes the miss shrink
fastest *right now*. It never consults the gains, so it will happily push in a
low-gain direction just because that direction points most directly at the target.

Picture a long thin valley with the target at the far end. Greedy walks straight at the
target — across the valley and up the side, then back down. The minimum-$\Delta v$
answer walks along the valley floor. Both arrive; one takes a longer route.

Everything from here down is the same story with the working shown.

---

## 1. Setup

State $x = (r, v) \in \mathbb{R}^6$ in a body-centred **inertial** frame whose $+z$ is
the spin axis. The coast obeys

$$
\dot r = v, \qquad
\dot v = -\frac{\mu}{\lVert r\rVert^{3}} r
\;-\; \tfrac12 \rho(h)\, \lVert v_{\text{air}}\rVert\, C_d(M,\alpha)\, \frac{A}{m}\, v_{\text{air}},
$$

$$
v_{\text{air}} = v - \boldsymbol\omega \times r,
\qquad \boldsymbol\omega = (0,0,\omega),
\qquad h = \lVert r\rVert - R_{\text{mean}} .
$$

Impact time $t_f$ is defined **implicitly** by

$$
g\bigl(t_f, x_0\bigr) \;=\; \bigl\lVert r(t_f; x_0) \bigr\rVert - R_{\text{target}} \;=\; 0 .
$$

The ground impact point is the inertial one carried back by the rotation that happens
during the flight, so that it is fixed to the terrain:

$$
p(x_0) \;=\; R_z\!\bigl(-\omega t_f\bigr)\, r\bigl(t_f; x_0\bigr) .
$$

The miss against a target $p_{\text{t}}$ is

$$
m \;=\; p - p_{\text{t}} \;\in\; \mathbb{R}^3 \quad [\mathrm{m}].
$$

---

## 2. The Jacobian

$$
J \;=\; \frac{\partial p}{\partial v_0} \;\in\; \mathbb{R}^{3\times3},
\qquad J_{ij} = \frac{\partial p_i}{\partial v_{0j}} .
$$

Because $t_f$ itself depends on $v_0$, the chain rule has two terms:

$$
J \;=\;
\underbrace{R_z(-\omega t_f)\,\frac{\partial r}{\partial v_0}\bigg|_{t_f}}_{\text{trajectory}}
\;+\;
\underbrace{\left[\frac{\partial R_z}{\partial t_f}\, r \;+\; R_z(-\omega t_f)\, \dot r\right]
\frac{\partial t_f}{\partial v_0}}_{\text{event}},
$$

with the event sensitivity from the implicit function theorem:

$$
\frac{\partial t_f}{\partial v_0}
= -\frac{\partial g/\partial v_0}{\partial g/\partial t},
\qquad
\frac{\partial g}{\partial t} = \frac{r \cdot v}{\lVert r\rVert}.
$$

Both terms come out of the same forward-mode AD sweep — we never write them down. The
event term is obtained by running the surface-crossing Newton iteration
$h \leftarrow h - g/\dot g$ **in dual arithmetic**: differentiating that update and
using $g\to0$ at convergence leaves exactly $-\,(\partial g/\partial v_0)/\dot g$,
independent of whatever derivative the starting guess carried.

Dropping the event term is not a small error. Measured: it is worth **39.4 m per m/s**
against a column magnitude of 78.9, i.e. **50%**.

### Units, and a sanity anchor

$$
[J] \;=\; \frac{\mathrm{m}}{\mathrm{m/s}} \;=\; \mathrm{s}.
$$

**The Jacobian is in seconds.** For a drag-free, gravity-free coast $p = r_0 + v_0 t$,
so $J = t\,I$ and every singular value equals the time of flight. Measured here:

| | value |
|---|---|
| $\sigma_1,\ \sigma_2,\ \sigma_3$ | $129.18,\ 101.63,\ 0$ s |
| time of flight $t_f$ | $144.33$ s |

Same order, as it must be. If a singular value ever came out wildly away from $t_f$,
something is wrong.

### $J$ is rank 2, and there is only one reason

It comes entirely from what the predictor **reports**, not from any constraint and not
from the trajectory's shape. $\lVert p \rVert = R_{\text{target}}$ for *every*
perturbed trajectory — that is the definition of a crossing point — so

$$
\frac{\partial}{\partial v_0}\lVert p\rVert = \hat n^{\top} J = 0,
\qquad \hat n = \frac{p}{\lVert p\rVert}.
$$

$\hat n$ is a **left null vector** of $J$ — measured $\lVert \hat n^\top J\rVert / \lVert J\rVert_F = 1.4\times10^{-17}$.
Any radial miss component is unreachable, and is annihilated by both $J^\top m$ and the
damped solve on their own. (The explicit tangent projection in the code is therefore
defensive, not load-bearing: removing it changes $\Delta v$ by $10^{-14}$ m/s.)

Hence $\sigma_3 = 0$ and $J^{-1}$ does not exist.

**Planarity is not a second cause.** An earlier version of this document said the
surface and the trajectory being planar were two independent reasons that stacked up.
They cannot be: two independent deficiencies would leave rank 1 and *two* zero singular
values, and there is only one. The measured $2\times2$ in-plane block does have rows
parallel to six significant figures ($-1.3011/-1.6879 = 0.770840$ against
$78.8552/102.2979 = 0.770839$) — but that is the same tangency fact restricted to the
plane, not an extra one. Tested directly on an inclined, non-planar arc:

$$
\sigma = (130.89,\ 101.66,\ 0) \quad\text{— still rank 2.}
$$

Planarity fixes *where* the null direction points, not that there is one.

---

## 3. Law A — pure Jacobian, instantaneous greedy

Minimise the linearised miss

$$
\Phi(\Delta v) = \tfrac12 \bigl\lVert m + J\,\Delta v \bigr\rVert^2 .
$$

Its gradient at $\Delta v = 0$ is

$$
\nabla \Phi \big|_{0} = J^{\top} m,
$$

so the steepest-descent (greedy) direction is $-J^{\top}m$. **This is the transpose,
not the Jacobian itself** — $J$ maps $\Delta v \mapsto \Delta p$, so $J m$ is not even
dimensionally meaningful as a velocity.

A direction is not a step. Exact line search along it:

$$
\alpha^\star \;=\; \arg\min_\alpha \bigl\lVert m - \alpha J J^{\top} m \bigr\rVert^2
\;=\; \frac{\lVert J^{\top} m\rVert^{2}}{\lVert J J^{\top} m\rVert^{2}},
\qquad
\boxed{\;\Delta v = -\,\alpha^\star J^{\top} m\;}
$$

Units: $[J^\top m] = \mathrm{s\,m}$ and $[\alpha^\star] = \mathrm{s}^{-2}$, giving
m/s — but **$\alpha$ has to come from somewhere**. Either you do the line search above
(a second matrix–vector product, cheap) or you hand-tune a gain in units of
$\mathrm{s}^{-2}$ that must be retuned as $t_f$ changes, since $\sigma_i \sim t_f$.

**Convergence is linear**, at rate

$$
\left(\frac{\kappa - 1}{\kappa + 1}\right)^{2},
\qquad \kappa = \frac{\sigma_1^2}{\sigma_2^2} = \left(\frac{129.18}{101.63}\right)^{2} = 1.616 ,
$$

predicting $0.055$ of the miss surviving per iteration. Measured: $0.057$. The theory
holds, and the rate is good **because the reachable 2-D subspace is well conditioned** —
$\sigma_1/\sigma_2 = 1.27$. Steepest descent is not badly behaved here; it would be on a
problem with a worse spread.

### Why does greedy need to iterate at all?

$-J^{\top}m$ *is* the direction that maximises the reduction in miss — but only in the
limit of an **infinitesimal** step. It is a statement about the derivative at the
current point:

$$
\left.\frac{\mathrm{d}}{\mathrm{d}\epsilon}\right|_{\epsilon=0}
\tfrac12\bigl\lVert m + J(\epsilon\, u)\bigr\rVert^{2} = \epsilon\, u^{\top} J^{\top} m ,
$$

minimised over unit $u$ by $u = -J^{\top}m / \lVert J^{\top}m\rVert$. Guidance does not
apply an infinitesimal $\Delta v$, and two separate things break the hope of one shot.

**(i) Conditioning — and this one has nothing to do with nonlinearity.**
Freeze $J$ and $m$ and consider the *exactly* linear problem
$\min_{\Delta v}\lVert m + J\Delta v\rVert$. Its level sets are ellipses whose axes are
the singular values of $J$ — here 129.18 and 101.63. The gradient is **perpendicular to
the level set**, and perpendicular-to-an-ellipse points at its centre only when the
ellipse is a **circle**, i.e. when $J^{\top}J \propto I$. Otherwise the gradient
overshoots along one axis and undershoots along another, and the iterations exist to
work off that mismatch. Measured on the frozen linear problem:

```
Gauss-Newton, ONE step:      residual    36.44 m     (from 21.541 km)
steepest descent, step 1:    residual  2959.47 m
              …  step 2:     residual   408.17 m
              …  step 3:     residual    66.67 m
              …  step 4:     residual    37.21 m
              …  step 5:     residual    36.43 m
```

No nonlinearity anywhere in that table. Gauss-Newton solves the quadratic exactly
because it *is* the exact minimiser of the quadratic; steepest descent takes five steps
to reach the same place.

(The ~36 m both settle on is not a failure. $m$ is the **chord** between two points on
the sphere, and a chord is not tangent: its radial part is
$\text{chord}^2/2R = 21541^2/(2 \cdot 6371000) = 36.4$ m, which no tangent displacement
can produce. It disappears in the nonlinear iteration because as the miss shrinks the
chord shortens and its radial part falls off as the square.)

**(ii) Nonlinearity.** $J$ is a linearisation of a map that curves, so even the exact
minimiser of the linear model leaves a second-order residual: Gauss-Newton's first step
left 1.349 km of 21.541 km, or 6.3%.

So **Gauss-Newton iterates only because of (ii); greedy iterates because of (i) *and*
(ii)** — and here (i) dominates, which is why greedy's measured per-iteration factor on
the full nonlinear problem (0.057) matches the *linear* theory (0.055) so closely. Its
slowness is conditioning, not curvature.

**When the distinction stops mattering.** In a closed loop applying small corrections at
a high rate and re-measuring each cycle, you never take a big step — so "best direction
right now" is close to all you need, and greedy is defensible. The distinction bites
when you want to compute the whole correction in one shot, which is exactly what an
open-loop boostback targeting solution wants to do.

---

## 4. Law B — Gauss-Newton / pseudo-inverse

Instead of descending on $\Phi$, set the linearised miss to **zero**:

$$
m + J\,\Delta v = 0 .
$$

$J$ is rank deficient, so take the minimum-norm solution, $\Delta v = -J^{+}m$. Since
$JJ^{\top}$ is singular this needs regularising — Levenberg–Marquardt:

$$
\boxed{\;\Delta v \;=\; -\,J^{\top}\bigl(J J^{\top} + \lambda^{2} I\bigr)^{-1} m\;}
$$

In the SVD $J = U\Sigma V^{\top}$ this is

$$
\Delta v \;=\; -\,V \,\operatorname{diag}\!\left(\frac{\sigma_i}{\sigma_i^{2} + \lambda^{2}}\right) U^{\top} m ,
$$

i.e. each singular direction is passed through with **filter factor**

$$
f_i = \frac{\sigma_i^{2}}{\sigma_i^{2} + \lambda^{2}} .
$$

At $\lambda = 1$ s against $\sigma = (129.18,\ 101.63,\ 0)$:

$$
f = (0.99994,\ 0.99990,\ 0).
$$

The two useful directions pass essentially untouched; the null direction is annihilated
rather than amplified. As $\lambda \to 0$ this tends to Gauss-Newton, as
$\lambda \to \infty$ to steepest descent with step $1/\lambda^2$ — the two laws are the
ends of one family.

No inverse is formed in code: the $3\times3$ system $(JJ^\top + \lambda^2 I)y = m$ is
solved by Gaussian elimination with partial pivoting and the result multiplied by
$J^\top$.

**Nothing here is geometry-aware.** This is the ordinary damped pseudo-inverse applied
to whatever $J$ arrives — no term encodes the sphere, the tangent plane or the rotating
frame. As $\lambda \to 0$ it becomes the plain Moore-Penrose pseudo-inverse, including
for rank-deficient $J$. The only reason $\lambda$ is not zero is that the exact
right-inverse form $J^\top (JJ^\top)^{-1}$ needs $J$ to have full row rank, which it
does not. The geometry shows up only in the *values* $J$ takes.

### Where the minimisation actually comes from

No optimiser runs anywhere in `Correction`. The minimisation is in the *shape* of the
formula, and it is worth seeing exactly where.

**Two subspaces.** $J$ has rank 2, so velocity space splits into two orthogonal pieces:

$$
\mathbb{R}^{3} \;=\; \operatorname{row}(J) \;\oplus\; \ker(J),
\qquad \dim = 2 + 1 .
$$

Pushes in $\ker(J)$ do nothing at all. Measured, with
$n = (0.7920,\ -0.6105,\ 0)$ spanning it:

$$
\lVert J n \rVert = 7.2\times10^{-15}\ \text{m per m/s}.
$$

**So solutions come in lines, not points.** If $\Delta v_0$ solves
$J\,\Delta v = -m$, then so does $\Delta v_0 + t\,n$ for every $t$ — same impact point,
different fuel. Measured:

| $t$ added along $n$ | $\lVert\Delta v\rVert$ | impact shift |
|---|---|---|
| 0 m/s | 173.67 m/s | 0.000000 m |
| 25 m/s | 175.46 m/s | 0.000000 m |
| 50 m/s | 180.73 m/s | 0.000000 m |
| 100 m/s | 200.40 m/s | 0.000000 m |

**Which one is shortest.** Split any $\Delta v$ into its two orthogonal parts. By
Pythagoras,

$$
\lVert \Delta v \rVert^{2} \;=\; \lVert \Delta v_{\text{row}} \rVert^{2}
\;+\; \lVert \Delta v_{\text{null}} \rVert^{2},
$$

and since $J\,\Delta v = J\,\Delta v_{\text{row}}$, the row part is **forced** by the
equation while the null part is **free**. The norm is therefore smallest when

$$
\Delta v_{\text{null}} = 0 ,
$$

i.e. when $\Delta v$ lies entirely in $\operatorname{row}(J)$. That is the entire
content of "minimum $\Delta v$".

**And that is what $J^{\top}$ on the outside buys you.** Because

$$
\operatorname{row}(J) = \operatorname{col}(J^{\top}),
$$

*any* vector of the form $\Delta v = -J^{\top}y$ lies in the row space, whatever $y$ is
— so it has no null component by construction. Measured on the computed answer:

$$
\frac{n \cdot \Delta v}{\lVert \Delta v\rVert} = 8.2\times10^{-17}.
$$

So the algorithm reads:

1. **Restrict** the search to $\Delta v = -J^{\top}y$ — *this is the minimisation*, done
   structurally rather than by searching.
2. Substitute into $J\,\Delta v = -m$, giving $\bigl(JJ^{\top}\bigr) y = m$.
3. Solve that $3\times3$ for $y$.
4. Recover $\Delta v = -J^{\top}y$.

There is no minimisation step because the $J^{\top}$ is the minimisation.

**"We inverted the Jacobian" — not quite.** $J$ is singular and was never inverted. What
is inverted is $JJ^{\top} + \lambda^{2}I$, a different and genuinely invertible matrix.
Note also that $JJ^{\top}$ acts on the *impact* side, not the velocity side: $y$ is an
intermediate living in impact space, and $J^{\top}$ is what carries it back to a
velocity. The $\lambda^{2}I$ is there because $JJ^{\top}$ inherits $J$'s rank 2 and is
singular on its own.

Adding $\lambda$ also changes the question slightly, from "hit it exactly, with least
fuel" to

$$
\min_{\Delta v}\ \bigl\lVert m + J\,\Delta v \bigr\rVert^{2} + \lambda^{2}\lVert \Delta v\rVert^{2},
$$

a trade between accuracy and fuel. At $\lambda = 1$ against gains of 129 and 102 the
trade is negligible on the directions that work and total on the one that does not.

**Units:** $[J^{+}] = \mathrm{s}^{-1}$, so $\Delta v$ comes out in m/s **with no gain to
tune**. That is the practical difference from Law A, and it is not cosmetic — the
scaling adapts automatically as the time of flight shrinks through the coast.

**Convergence is quadratic** (this is a zero-residual problem: a reachable target can be
hit exactly).

---

## 5. Measured comparison

Same target — 20 km back along track, 8 km across, projected onto the surface. Miss
21.541 km.

| | Law A: greedy $-\alpha^\star J^\top m$ | Law B: damped $-J^{+}m$ |
|---|---|---|
| iterations to < 10 m | 4 | 3 |
| total $\Delta v$ | **204.44 m/s** | **186.16 m/s** |
| first step $\Delta v$ | 168.08 m/s | 173.67 m/s |
| miss after first step | 3.291 km | 1.349 km |
| convergence | linear, factor 0.057 | quadratic |
| step scaling | needs $\alpha$ (line search or tuned gain) | none |
| cost per iteration | $J$, plus one extra mat–vec | $J$, plus a $3\times3$ solve |

Miss sequences:

```
Law A   21.541 km -> 3.291 km -> 0.187 km -> 0.011 km -> 0.001 km
Law B   21.541 km -> 1.349 km -> 0.005 km -> 0.000 km
```

**Greedy costs about 10% more $\Delta v$ and one extra iteration.** That is the honest
number. An earlier note in this repo claimed "2.3× worse", which came from giving
steepest descent an arbitrary step length equal to the Gauss-Newton one rather than its
own optimal $\alpha^\star$ — a straw man.

Both need the same expensive thing, $J$ (three seeded sweeps, ~3.6 ms). Neither is
meaningfully cheaper than the other; the $3\times3$ solve is free next to the
integration.

---

## 6. What neither of these is

Both laws answer: *what is the smallest instantaneous impulse, applied now, that moves
the predicted impact toward the target*. That is greedy in the sense that matters for
propellant, and no amount of iterating fixes it. Neither law can see:

- **Burn duration.** Both assume an impulse. A real boostback burn has duration, during
  which $r$ moves, so the achieved impact differs from the impulsive prediction.
- **Cost along the trajectory.** No gravity loss, no drag loss during the burn, no
  throttle or attitude rate limit. $\lVert\Delta v\rVert$ is not propellant.
- **When to burn.** $\sigma_i \sim t_f$, so the *same* miss costs less $\Delta v$ the
  earlier it is corrected. A law that only ever nulls the miss *now* is structurally
  blind to this, and it is where the real fuel saving lives.
- **The null direction as a resource.** Minimum norm puts zero component along
  $\ker J$. But that direction is *free* with respect to the impact point — a spare
  degree of freedom that could be spent on entry velocity or attitude alignment.
  Both laws discard it. (`ImpactSteering.FreeDirection` and the **Min pitch** knob
  now spend it; see §8.)

Jo & Ahn's closed-form boost-back guidance is what encodes the finite-burn and fuel
structure — the velocity-frame reparameterisation, the thrust-magnitude sphere, and the
closed-form pick on the resulting great circle. Its $d$ vectors are **sensitivities of
the impact point**, exactly the $J$ above, and the whole point of the drag-aware
variant is to substitute this integrated $J$ for their Keplerian one.

So: $J$ is the deliverable. `ImpactSteering` is a reference implementation and the
arrow on the overlay, not the guidance law.

---

## 7. What flies instead

The first two bullets of §6 turned out to be the whole story, and they were fixed by
changing the question rather than by improving the answer.

`BoostbackShooter` describes a burn by five numbers — pitch, yaw, pitch rate, yaw rate,
duration — flown as a **linear tangent law** $\hat{i}_F(\tau) = \widehat{\lambda +
\dot\lambda\,\tau}$ through the real powered arc, mass depletion and all, then
coasting retrograde to the ground. Two of the five are driven to hit the site (duration
→ downrange, yaw → crossrange); the other three are optimised for propellant. So the
burn *has* a duration, and the cost being minimised is propellant rather than
$\lVert\Delta v\rVert$.

The difference is not academic. On the reference arc the impulsive correction points
**33° below the horizon** and the finite-burn optimum points **25° above** it, for
18.3% less propellant — and everything below the horizon is not merely dear but
*infeasible* for the vehicle, which runs the tanks dry with tens of kilometres still to
go. The nose-down answer was an artefact of pretending a 48-second burn is an impulse.

`Guidance/Boostback.cs` flies the plan on a **receding horizon**: re-solve from the live
state every 2 s, fly the head of the freshest plan, freeze inside T-5 s and evaluate the
law out to cutoff. Against an engine 3% down on thrust, `--shoot` measures the burn
missing by **6.91 km** flown open loop, **875 m** re-solved every 2 s, and **240 m**
re-solved right through to cutoff — so the loop absorbs the modelling error and the
remaining residue is the price of the freeze window.

The impulsive law did not go away, because it answers a question the shot does not:
*is there anything worth lighting an engine for, and is there anything left to correct.*
It is cheap, it is re-solved at 10 Hz, and it starts and ends the burn. What it no
longer does is say which way to point.

---

## 8. Where this lives

| | |
|---|---|
| `lib/Navbox.Numerics/Math/Rk4.cs` | RK4 over `IOdeSystem`, dual-valued throughout |
| `lib/Navbox.Numerics/Flight/ImpactPredictor.cs` | `DragCoastSystem`, `Predict`, `VelocityJacobian` |
| `lib/Navbox.Numerics/Flight/ImpactSteering.cs` | `Correction`, `FreeDirection` — both laws |
| `lib/Navbox.Numerics/Flight/PoweredBurn.cs` | `BoostbackShooter` — the burn that actually flies |
| `ksamod/Guidance/Boostback.cs` | the four-phase machine and the receding horizon |
| `scvx/Scvx.Console/ImpactCheck.cs` | `--impact`, every number in §1–§6 |
| `scvx/Scvx.Console/ShootCheck.cs` | `--shoot`, every number in §7 |
