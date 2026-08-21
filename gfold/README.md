# gfold

G-FOLD (convex powered-descent guidance, Açıkmeşe/Blackmore) in C#, intended as
the terminal-guidance trajectory generator for the KSA mod (`../ksamod`). The
mod consumes this as compiled artifacts only — `Gfold.Core.dll` + `ecos.dll`
dropped next to the mod DLL — so none of the optimization machinery leaks into
the mod project.

## Mathematics

G-FOLD casts powered-descent guidance as a **second-order cone program**
(SOCP): a convex problem with a linear objective, linear equality
constraints, and second-order (Lorentz) cone constraints. In standard form,

$$
\begin{aligned}
\min_{x} \quad & f^\top x \\
\text{s.t.} \quad & \lVert A_i x + b_i \rVert_2 \le c_i^\top x + d_i, \quad i = 1, \dots, k \\
& F x = g
\end{aligned}
$$

where $`x \in \mathbb{R}^n`$ is the decision vector and there are $`k`$ conic
constraints. Each one confines $`(A_i x + b_i,\ c_i^\top x + d_i)`$ to a
second-order cone $`\mathcal{Q} = \lbrace (y, \tau) : \lVert y \rVert_2 \le \tau \rbrace`$.

The problem is assembled and then solved by the ECOS solver; its output is the
decision vector $`x`$, which stacks every per-node variable — position, velocity,
thrust acceleration $`\mathbf{u}`$, log-mass $`z`$, and slack $`\sigma`$ — into one
vector (see Notation).

The sections below build up to that form: the convexity rules an SOCP must obey,
notation, the continuous dynamics, the convexification that makes it an SOCP, the
discretized problem with its constraint tables, and finally how it is assembled
for the solver (down to the full matrices for a tiny case).

### Convexity: what an SOCP allows

A convex program can be solved to a *global* optimum in polynomial time precisely
because its feasible set is convex and its objective convex — the interior-point
method can never get stuck in a false local minimum. An SOCP is the special case
assembled from only these ingredients:

- **Objective** — linear, $`f^\top x`$. (A convex objective is made linear by the
  epigraph trick: $`\min\, t`$ subject to $`f(x) \le t`$ — exactly how P3 handles its
  norm objective.)
- **Equality constraints** — must be **affine**, $`Fx = g`$. A nonlinear equality is
  never allowed: it traces a curved surface, not a solid convex region. (This is
  why the mass equality $`\dot m = -\alpha \lVert \mathbf{T}_c \rVert`$ had to be
  relaxed — see Convexification.)
- **Inequality constraints** — must be **convex**, and in an SOCP take exactly two
  shapes:
  - linear, $`a^\top x \le b`$ (the nonnegative orthant $`\mathbb{R}_+`$);
  - second-order cone, $`\lVert A x + b \rVert_2 \le c^\top x + d`$.

**Rules of thumb.** Write every inequality as $`g(x) \le 0`$; it is a convex
constraint **iff $`g`$ is convex**. Useful facts:

- affine functions $`a^\top x + b`$ are convex *and* concave → fine on either side
  of an inequality, and the only thing allowed in an equality;
- norms $`\lVert \cdot \rVert`$, exponentials $`e^{az}`$, and squares are convex;
- a sum of convex functions, or the pointwise max of convex functions, is convex;
- "convex $`\le`$ affine" is convex (e.g. $`\lVert \mathbf{u} \rVert \le \sigma`$, or
  the throttle floor $`\rho_1 e^{-z} \le \sigma`$); the reverse "affine $`\le`$ convex"
  is **non-convex** (e.g. the throttle ceiling $`\sigma \le \rho_2 e^{-z}`$, which is
  why it must be linearized);
- the intersection of convex sets is convex, so stacking more valid constraints
  keeps the whole problem convex.

**Worked example — why the log-mass box is convex.** The box
$`z_{0,n} \le z_n \le z_{1,n}`$ looks like it might inherit the exponential
trouble, but $`z_{0,n}`$ and $`z_{1,n}`$ are precomputed *constants* (the min/max-mass
envelope at node $`n`$), not functions of the variables. So it is just two affine
bounds, $`z_n - z_{1,n} \le 0`$ and $`z_{0,n} - z_n \le 0`$ — two half-spaces whose
intersection (a slab) is convex. No $`e^{-z}`$ appears in the box itself, which is
why it is plain *linear*, not a cone.

### Notation

| Symbol | Meaning |
| --- | --- |
| $`n`$ | node index, $`0 \dots N-1`$ ($`N`$ nodes, step $`\Delta t = t_f/(N-1)`$) |
| $`\mathbf{r} = (r_x, r_y, r_z)`$ | position; $`r_x`$ is the vertical (altitude) component |
| $`(\cdot)_{yz}`$ | the horizontal $`(r_y, r_z)`$ components of a vector |
| $`\mathbf{v}`$ | velocity |
| $`\mathbf{x} = [\mathbf{r}; \mathbf{v}]`$ | per-node 6-D dynamical state (distinct from the stacked SOCP decision vector $`x`$ above) |
| $`\mathbf{u} = \mathbf{T}_c / m`$ | thrust acceleration — the control |
| $`z = \ln m`$ | log-mass (not a position component) |
| $`\sigma`$ | thrust-magnitude slack, $`\lVert \mathbf{u} \rVert \le \sigma`$; equals $`\Gamma / m`$ |
| $`t`$ | P3 landing-error epigraph variable |
| $`\mathbf{g}`$ | constant gravity vector |
| $`\alpha = 1/(I_{sp}\, g_0)`$ | mass-flow (fuel-consumption) rate |
| $`\rho_1, \rho_2`$ | min / max thrust (force) bounds |
| $`\Gamma`$ | thrust-magnitude slack in force units (paper's variable; $`\sigma = \Gamma/m`$) |
| $`\theta`$ | max thrust pointing angle from vertical |
| $`\gamma_{gs}`$ | glideslope angle |
| $`V_\text{max}`$ | max speed |
| $`\mathbf{r}_f, \mathbf{v}_f`$ | target landing position / velocity |

### State and dynamics

The vehicle state stacks position and velocity (both in the surface frame):

$$
\mathbf{x} = \begin{bmatrix} \mathbf{r} \\ \mathbf{v} \end{bmatrix} \in \mathbb{R}^6,
\qquad
\mathbf{r}, \mathbf{v} \in \mathbb{R}^3
$$

Its derivative is the translational equation of motion, with thrust
acceleration $`\mathbf{T}_c / m`$ and gravity $`\mathbf{g}`$ as inputs:

$$
\dot{\mathbf{x}} = A(\omega) \mathbf{x} + B \left( \mathbf{g} + \frac{\mathbf{T}_c}{m} \right)
$$

$$
A(\omega) = \begin{bmatrix} 0 & I \\ -S(\omega)^2 & -2 S(\omega) \end{bmatrix},
\qquad
B = \begin{bmatrix} 0 \\ I \end{bmatrix}
$$

Here $`S(\omega)`$ is the skew-symmetric cross-product matrix of the planet's
angular velocity $`\omega`$, so $`-S(\omega)^2 \mathbf{r}`$ is the centripetal and
$`-2 S(\omega) \mathbf{v}`$ the Coriolis term. This implementation takes
$`\omega = 0`$ (non-rotating frame), reducing $`A`$ to the double integrator
$`\left[ \begin{smallmatrix} 0 & I \\ 0 & 0 \end{smallmatrix} \right]`$.

### Convexification

In physical variables $`(\mathbf{r}, \mathbf{v}, m, \mathbf{T}_c)`$ the problem is
**non-convex** on three counts: the acceleration $`\mathbf{T}_c / m`$ is bilinear,
the mass depletion $`\dot m = -\alpha \lVert \mathbf{T}_c \rVert`$ is nonlinear,
and the engine's lower throttle limit $`\lVert \mathbf{T}_c \rVert \ge \rho_1 > 0`$
carves out a non-convex set. Two changes of variable ($`\mathbf{u}`$, $`z`$), one
slack relaxation ($`\sigma`$), and one epigraph variable ($`t`$) turn it into the
SOCP standard form above. (The mass/discretization part is from Açıkmeşe & Ploen
2007; the lossless control relaxation from Açıkmeşe, Carson & Blackmore 2013 —
see references.)

| New variable | Definition | Removes | Replaces |
| --- | --- | --- | --- |
| $`\mathbf{u}`$ | $`\mathbf{T}_c / m`$ (thrust acceleration) | $`\mathbf{T}_c/m`$ division → dynamics become $`\dot{\mathbf{v}} = \mathbf{u} + \mathbf{g}`$ | thrust force |
| $`z`$ | $`\ln m`$ (log-mass) | nonlinear mass ODE → $`\dot z = -\alpha \sigma`$ | mass |
| $`\sigma`$ | slack with $`\lVert \mathbf{u} \rVert \le \sigma`$ | non-convex thrust lower bound $`\lVert \mathbf{T}_c \rVert \ge \rho_1`$ (handled *losslessly*) | $`\lVert \mathbf{u} \rVert`$ |
| $`t`$ | epigraph with $`\lVert \mathbf{r}_{N-1} - \mathbf{r}_f \rVert \le t`$ | norm in the P3 objective | — (P3 only) |

**Lossless convexification.** The thrust magnitude is relaxed from
$`\lVert \mathbf{u} \rVert = \sigma`$ to the cone $`\lVert \mathbf{u} \rVert \le \sigma`$.
The slack $`\sigma`$ here is the paper's $`\Gamma`$ normalized by mass,
$`\sigma = \Gamma / m`$ — it carries the magnitude into the (now linear) mass
dynamics and throttle bounds. The relaxation is provably *tight* at the optimum
($`\lVert \mathbf{u} \rVert = \sigma`$), so nothing is lost.

**Taylor-linearized throttle bounds.** Dividing the throttle limits by mass
gives $`\rho_1 e^{-z} \le \sigma \le \rho_2 e^{-z}`$. The upper bound is
"$`\sigma \le`$ (convex)", a non-convex region, so $`e^{-z}`$ is linearized about a
per-node point $`z_{0,n}`$:

$$
\sigma_n \le \mu_{2,n}\,(1 - (z_n - z_{0,n})), \qquad \mu_{2,n} = \rho_2 e^{-z_{0,n}} .
$$

The expansion point is the **minimum possible mass at node $`n`$** — the lightest
the vehicle can be is full-throttle burn since launch:

$$
z_{0,n} = \ln\!\left( m_\text{wet} - \alpha \rho_2 \, t_n \right), \qquad t_n = n\,\Delta t .
$$

$`e^{-z}`$ is convex (its second derivative $`\tfrac{d^2}{dz^2} e^{-z} = e^{-z} > 0`$
everywhere), so its tangent lies below the curve and the affine bound is exact at
$`z_{0,n}`$ and conservative above it (it never permits an infeasible throttle). The
box $`z_{0,n} \le z_n \le z_{1,n}`$ keeps $`z_n`$ near $`z_{0,n}`$ so the linearization
stays tight.

**Upper vs lower bound — expanded for different reasons.** The two throttle bounds
are not the same kind of constraint:

- **Upper** $`\sigma \le \rho_2 e^{-z}`$ is the region *below* a convex curve (a
  hypograph) — **non-convex**. The first-order linearization replaces it with an
  affine constraint to *make it convex*.
- **Lower** $`\sigma \ge \rho_1 e^{-z}`$ is the region *above* a convex curve (an
  epigraph) — already **convex**, so it needs no fixing for convexity. It is
  expanded only because it is an *exponential-cone* shape, and ECOS handles only
  linear and second-order cones. A second-order Taylor cut,
  $`\sigma \ge \mu_{1,n}(1 - w + \tfrac{1}{2} w^2)`$ with $`w = z - z_{0,n} \ge 0`$,
  turns it into a quadratic — a rotated second-order cone ECOS *can* take.
  Truncating the alternating series after the $`+\tfrac{1}{2}w^2`$ term
  over-estimates $`e^{-w}`$, so the cut stays conservative (never under-thrusts).
  This bound is optional (`EnforceLowerThrust`).

This is what trades the *unfixable* non-convexity in the dynamics for a *mild,
conservative* linearization on the throttle bound — the reason the log-mass form
is preferred even though it (unlike the force form) needs the Taylor step.

### G-FOLD Problem Formulation

Time of flight $`t_f`$ is **not** a decision variable in the convex program: each
solve fixes $`t_f`$ (so $`\Delta t = t_f/(N-1)`$ is constant), and an outer
golden-section search picks the best $`t_f`$. For a fixed $`t_f`$ the code solves one
of two problems:

$$
\textbf{P3 (min landing error):} \quad \min \; \lVert \mathbf{r}_{N-1} - \mathbf{r}_f \rVert
$$

$$
\textbf{P4 (min fuel):} \quad \max \; z_{N-1}
$$

The classic G-FOLD objective is written $`\min \int_0^{t_f} \Gamma\, dt`$ (minimize
integrated thrust = fuel). The code does **not** optimize that integral directly;
it maximizes the final log-mass $`z_{N-1}`$, which is **equivalent**: fuel burned is
$`m_\text{wet} - e^{z_{N-1}}`$, so maximizing $`z_{N-1}`$ minimizes fuel, and fuel
$`= \alpha \int_0^{t_f} \Gamma\, dt`$. P3's norm objective is made linear with the
epigraph variable $`t`$ ($`\lVert \mathbf{r}_{N-1} - \mathbf{r}_f \rVert \le t`$,
minimize $`t`$).

Subject to equality and inequality constraints.

#### Equality Constraints

Our equality constraints are our initial and terminal conditions, as well as the
dynamics that link one node to another.

| Constraint | Expression | Description |
| --- | --- | --- |
| Initial position | $`\mathbf{r}_0 = \mathbf{r}_\text{init}`$ | Trajectory starts at the measured position |
| Initial velocity | $`\mathbf{v}_0 = \mathbf{v}_\text{init}`$ | Trajectory starts at the measured velocity |
| Terminal velocity | $`\mathbf{v}_{N-1} = \mathbf{v}_f`$ | Arrive at the target touchdown velocity $`\mathbf{v}_f`$ |
| Initial mass | $`z_0 = \ln m_\text{wet}`$ | Log-mass begins at the wet mass |
| Terminal position | $`r_{x,N-1} = r_{f,x}`$ (P3); $`\quad \mathbf{r}_{N-1} = \mathbf{r}_f`$ (P4) | P3 pins altitude only (floats the landing point); P4 pins the full point |
| Endpoint thrust direction | $`\mathbf{u}_0 = \sigma_0\,\hat{\mathbf{e}}, \quad \mathbf{u}_{N-1} = \sigma_{N-1}\,\hat{\mathbf{e}}, \quad \hat{\mathbf{e}} = (1,0,0)`$ | Thrust vertical at the endpoints (initial dropped in free-initial-thrust mode) |
| Terminal thrust slack | $`\sigma_{N-1} = 0`$ | Thrust magnitude goes to zero at touchdown |
| Velocity dynamics | $`\mathbf{v}_{n+1} = \mathbf{v}_n + \tfrac{1}{2}(\mathbf{a}_n + \mathbf{a}_{n+1})\,\Delta t, \quad \mathbf{a} = \mathbf{u} + \mathbf{g}`$ | Trapezoidal link for $`\mathbf{v}`$ (thrust accel + gravity) |
| Position dynamics | $`\mathbf{r}_{n+1} = \mathbf{r}_n + \tfrac{1}{2}(\mathbf{v}_n + \mathbf{v}_{n+1})\,\Delta t`$ | Trapezoidal integral of $`\mathbf{v}`$ |
| Mass dynamics | $`z_{n+1} = z_n - \tfrac{\alpha\,\Delta t}{2}(\sigma_n + \sigma_{n+1})`$ | Log-mass decreases with integrated thrust magnitude $`\sigma`$ |

#### Inequality Constraints

| Constraint | Expression | Cone | Description |
| --- | --- | --- | --- |
| Thrust magnitude | $`\lVert \mathbf{u}_n \rVert \le \sigma_n`$ | SOC | Lossless convexification: slack $`\sigma`$ bounds the thrust acceleration |
| Thrust pointing | $`\hat{\mathbf{e}}_1^\top \mathbf{u}_n \ge \sigma_n \cos\theta`$ | linear | Thrust stays within angle $`\theta`$ of vertical |
| Thrust upper bound | $`\sigma_n \le \mu_{2,n}\,(1 - (z_n - z_{0,n}))`$ | linear | Throttle ceiling $`\rho_2 e^{-z}`$, linearized about $`z_{0,n}`$, $`\ \mu_{2,n} = \rho_2 e^{-z_{0,n}}`$ |
| Thrust lower bound | $`\sigma_n \ge \mu_{1,n}\,(1 - w + \tfrac{1}{2}w^2), \ w = z_n - z_{0,n}`$ | SOC | Throttle floor $`\rho_1 e^{-z}`$, 2nd-order cut as a rotated cone (optional) |
| Log-mass box | $`z_{0,n} \le z_n \le z_{1,n}`$ | linear | Mass stays in the reachable envelope, keeping the linearization tight |
| Glideslope | $`\lVert (\mathbf{r}_n - \mathbf{r}_f)_{yz} \rVert \le \cot\gamma_{gs}\,(r_{x,n} - r_{f,x})`$ | SOC | Stay above the approach cone toward the pad |
| Velocity cap | $`\lVert \mathbf{v}_n \rVert \le V_\text{max}`$ | SOC | Speed stays below $`V_\text{max}`$ |
| Ground | $`r_{x,n} \ge 0`$ | linear | Altitude stays non-negative |
| Landing-error epigraph | $`\lVert \mathbf{r}_{N-1} - \mathbf{r}_f \rVert \le t`$ | SOC | P3 only — bound the miss distance, minimized as the objective |

### What GfoldPlanner hands the solver

`GfoldPlanner.Solve` turns the formulation above into the arrays
`EcosSolver.Solve` expects — an objective `c`, equality matrices `A, b`,
inequality matrices `G, h`, and the cone-size declarations `PositiveOrthantDim`
and `SocDims` — then reads the answer back. Here is what each piece is and where
it comes from.

**The decision vector `x`.** The trajectory is sampled at `N` nodes
($`t_n = n\,\Delta t`$) and every variable at every node becomes an unknown
(direct transcription). They are packed into one vector `x` of length
$`11N`$ (or $`11N+1`$ for P3), blocked by type then node. The index helpers are
the single source of truth for which slot is what:

| Block | Index helper | Size | Contents |
| --- | --- | --- | --- |
| state | `IX(n,i) = 6n + i` | $`6N`$ | position ($`i=0,1,2`$), velocity ($`i=3,4,5`$) |
| control | `IU(n,i) = 6N + 3n + i` | $`3N`$ | thrust acceleration $`\mathbf{u}`$ |
| log-mass | `IZ(n) = 9N + n` | $`N`$ | $`z`$ |
| slack | `IS(n) = 10N + n` | $`N`$ | $`\sigma`$ |
| epigraph | `IT = 11N` | $`1`$ | $`t`$ (P3 only) |

**The objective `c`.** Same length as `x`, all zeros except one entry:
- P4 (min fuel): `c[IZ(N-1)] = -1` — maximize final log-mass.
- P3 (min error): `c[IT] = 1` — minimize the epigraph variable `t`.

**The equalities `A, b`.** `A` has one column per entry of `x` and one **row per
scalar equation**; `b` is the right-hand side. A row is mostly zeros — only the
variables that appear in that equation are nonzero. Each entry in the
equality-constraints table contributes its rows: a *vector* condition expands to
three (one per x/y/z component), the dynamics links couple node `n` to `n+1`.

For example, "initial position $`\mathbf{r}_0 = \mathbf{r}_\text{init}`$" is three
rows; the first (`GfoldPlanner.cs:245`) is

```csharp
A.Add(row, IX(0, 0), 1); b[row] = r0[0];   // 1 * x[IX(0,0)] = r0[0]
```

— a single `1` in the column for the initial-altitude slot, the measured value in
`b`.

**The inequalities `G, h` and cone sizes.** These use $`G x + s = h`$, i.e.
$`s = h - G x`$, with `s` required to stay in its cone (`≥ 0` for linear,
$`s_0 \ge \lVert(s_1,\dots)\rVert`$ for a cone). So for each row: **`h` holds the
constant part** of the quantity you want `s` to equal, and **`G` holds the
*negated* coefficients** of the variables (because of the minus sign).

*Linear example — ground, $`r_{x,n} \ge 0`$.* Altitude is the variable
`x[IX(n,0)]`; we want the slack to be that altitude (`s ≥ 0`). Constant part is
`0`, the coefficient `+1` negates to `-1` (`GfoldPlanner.cs:363`):

```csharp
G.Add(row, IX(n, 0), -1); h[row] = 0;      // s = 0 - (-1)*r_x = r_x >= 0
```

*Cone example — velocity cap, $`\lVert \mathbf{v}_n \rVert \le V_\text{max}`$.* A
second-order cone of size 4: we want `s = (V_max, v_x, v_y, v_z)`, so that
$`s_0 \ge \lVert(s_1,s_2,s_3)\rVert`$ reads $`V_\text{max} \ge \lVert\mathbf{v}_n\rVert`$.
Row 0 is the constant bound (no variables); rows 1–3 pull in the velocity
components (`GfoldPlanner.cs:384`):

```csharp
h[row++] = vMax;                            // s0 = V_max
for (int i = 0; i < 3; i++)
{ G.Add(row, IX(n, 3 + i), -1); h[row++] = 0; }   // s_{1..3} = v
soc.Add(4);                                 // declare: these 4 rows are one cone
```

The solver reads the rows as a sequence of blocks, so `Solve` **emits them in a
fixed order** and reports the sizes:
- the **linear** rows first — thrust pointing, throttle ceiling, log-mass box,
  ground — and their total count is `PositiveOrthantDim`;
- then each **cone** as one contiguous block — glideslope, velocity cap, thrust
  magnitude, optional throttle floor, P3 epigraph — with each block's size pushed
  onto `SocDims` in that order.

The order and the sizes must match the rows, or constraints get silently
misassigned; that is the only reason `Solve` builds the rows in this particular
sequence.

**Sparse format and the call.** `A` and `G` are accumulated as
`(row, col, value)` triplets and converted to compressed-column form by
`SparseCcs` (the layout the native solver reads). Arrays are pinned across the
P/Invoke boundary because ECOS keeps the caller's pointers alive from setup
through cleanup.

**Reading the result.** `EcosSolver.Solve` returns the optimal `x`; `Extract`
pulls each block out by the same indices and converts back to physical quantities
(`m = e^z`, and un-scaling — see below).

**Nondimensionalization.** One thing `Solve` does *before* assembly: scale the
problem to $`O(1)`$. The raw SI problem — metre-scale positions against unit-scale
log-mass rows — makes the solver numerically unhappy ("unreliable search
direction"). So lengths are scaled by $`L = \max(1000, \lVert \mathbf{r}_0 \rVert)`$,
time by $`T = \sqrt{L/g}`$ (hence velocity $`L/T`$, acceleration $`L/T^2`$); mass stays
in kg since it only enters through $`z = \ln m`$ and the scale-invariant product
$`\alpha\,\rho\,t`$. `Extract` un-scales everything on the way out.

### The constraint matrices, spelled out

The matrices are `11N` columns wide, so rather than a literal grid, each row group
is listed as a *stencil*: the columns it touches, the coefficient on each, and the
right-hand side. With the column ordering below, this fully determines `A`, `b`,
`G`, `h`. Shown for the **reference configuration** (no relaxations, throttle
floor off); $`i \in \{0,1,2\}`$ is the axis, $`n`$ the node, $`\Delta t`$ the step.

**Column ordering of `x`** (length `11N`, or `11N+1` for P3):

```
[ r_0 v_0 | r_1 v_1 | ... | r_{N-1} v_{N-1} | u_0 ... u_{N-1} | z_0 ... z_{N-1} | s_0 ... s_{N-1} | t ]
  └──────── state, 6N ─────────────────────┘ └── control 3N ─┘ └─ log-mass N ─┘ └── slack N ───┘ └P3┘
   cols 0 .. 6N-1                              6N .. 9N-1        9N .. 10N-1      10N .. 11N-1     11N
```

**Equality rows `A x = b`** (every row is one scalar equation):

| # rows | constraint | columns : coefficient | `b` |
| --- | --- | --- | --- |
| 3 | initial position | `IX(0,i):1` | `r_init[i]` |
| 3 | initial velocity | `IX(0,3+i):1` | `v_init[i]` |
| 3 | terminal velocity | `IX(N-1,3+i):1` | `v_f[i]` |
| 1 | initial mass | `IZ(0):1` | `ln m_wet` |
| 3 / 1 | terminal position | P4: `IX(N-1,i):1`; P3: only `IX(N-1,0):1` | `land[i]` / `r_f[0]` |
| 3 | initial thrust dir | `IU(0,0):1, IS(0):-1` ; `IU(0,1):1` ; `IU(0,2):1` | `0` |
| 3 | terminal thrust dir | `IU(N-1,0):1, IS(N-1):-1` ; `IU(N-1,1):1` ; `IU(N-1,2):1` | `0` |
| 1 | terminal slack | `IS(N-1):1` | `0` |
| 3(N-1) | velocity dynamics, `n=0..N-2` | `IX(n+1,3+i):1, IX(n,3+i):-1, IU(n,i):-Δt/2, IU(n+1,i):-Δt/2` | `Δt·g[i]` |
| 3(N-1) | position dynamics, `n=0..N-2` | `IX(n+1,i):1, IX(n,i):-1, IX(n+1,3+i):-Δt/2, IX(n,3+i):-Δt/2` | `0` |
| N-1 | mass dynamics, `n=0..N-2` | `IZ(n+1):1, IZ(n):-1, IS(n):αΔt/2, IS(n+1):αΔt/2` | `0` |

**Inequality rows `G x + s = h`** — linear block first (their count is
`PositiveOrthantDim`), then the cone blocks (sizes go to `SocDims`). Each cone is
listed row-by-row:

| # rows | constraint (linear) | columns : coefficient | `h` |
| --- | --- | --- | --- |
| N-1 | thrust pointing, `n=0..N-2` | `IS(n):cosθ, IU(n,0):-1` | `0` |
| N-2 | thrust upper bound, `n=1..N-2` | `IS(n):1, IZ(n):μ₂,ₙ` | `μ₂,ₙ(1+z₀,ₙ)` |
| N-2 | log-mass lower, `n=1..N-2` | `IZ(n):-1` | `-z₀,ₙ` |
| N-2 | log-mass upper, `n=1..N-2` | `IZ(n):1` | `z₁,ₙ` |
| N-1 | ground, `n=0..N-2` | `IX(n,0):-1` | `0` |

| cone × count | constraint (SOC) | rows (columns : coefficient) | `h` per row |
| --- | --- | --- | --- |
| Q3 × (N-1) | glideslope, `n=0..N-2` | `IX(n,0):-cotγ` / `IX(n,1):-1` / `IX(n,2):-1` | `-cotγ·r_f[0]` / `-r_f[1]` / `-r_f[2]` |
| Q4 × (N-1) | velocity cap, `n=0..N-2` | (none) / `IX(n,3+i):-1` | `V_max` / `0,0,0` |
| Q4 × (N-1) | thrust magnitude, `n=0..N-2` | `IS(n):-1` / `IU(n,i):-1` | `0` / `0,0,0` |
| Q4 × 1 | landing epigraph (P3) | `IT:-1` / `IX(N-1,i):-1` | `0` / `-r_f[i]` |

**Shape of it.** The boundary rows are just identity entries pinning single
variables. The dynamics rows are **bidiagonal in the node index** — each couples
node `n` to `n+1` only — so `A` is mostly a banded ladder down the diagonal. The
path-constraint rows of `G` are **block-diagonal by node** (each touches one
node's variables), except the epigraph, which ties the final node to `t`. That
sparsity is exactly what `SparseCcs` stores and what makes the solve fast.

### The full matrices for N = 5

A concrete instance: `N = 5` nodes (`n = 0..4`), `Δt = t_f/4`, problem **P4**
(min fuel), reference configuration. The decision vector has `11N = 55` entries.
Unlike a 2-node toy, the interior nodes `n = 1, 2, 3` now carry the throttle-ceiling
and log-mass-box rows, so every row family is visible.

Legend: `.` = 0, `p` = +1, `m` = −1, `D` = −Δt/2, `a` = +αΔt/2, `c` = cos θ,
`k` = −cot γ_gs, `U` = μ₂,ₙ (a positive per-node constant). The `b/h` column holds
the right-hand side.

Columns of `x` (0–54): node blocks `6n..6n+5` are `(rₙ, vₙ)` — so `0-5` = node 0,
`6-11` = node 1, `12-17` = node 2, `18-23` = node 3, `24-29` = node 4; then
`30-44` = controls `u₀..u₄` (3 each); `45-49` = `z₀..z₄`; `50-54` = `σ₀..σ₄`.

**Equality system `A x = b`** (48 rows × 55 cols):

```
       0 1 2 3 4 5 6 7 8 9 101112131415161718192021222324252627282930313233343536373839404142434445464748495051525354 |  b/h
r00:   p . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | r0x
r01:   . p . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | r0y
r02:   . . p . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | r0z
r03:   . . . p . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | v0x
r04:   . . . . p . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | v0y
r05:   . . . . . p . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | v0z
r06:   . . . . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . . . . . . . . . . . . . . . . . . . . . . | vfx
r07:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . . . . . . . . . . . . . . . . . . . . . | vfy
r08:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . . . . . . . . . . . . . . . . . . . . | vfz
r09:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . p | 0
r10:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . . . . . . . . . . . . . . m . . . . | 0
r11:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . . . . . . . . . . . . . . . . . . | 0
r12:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . . . . . . . . . . . . . . . . . | 0
r13:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . . . . . . m | 0
r14:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . . . . . . | 0
r15:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . . . . . | 0
r16:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . . . . | ln m_wet
r17:   . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | land_x
r18:   . . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | land_y
r19:   . . . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . . . . . . . . . . . . . . . . . . . . . . . | land_z
r20:   . . . m . . . . . p . . . . . . . . . . . . . . . . . . . . D . . D . . . . . . . . . . . . . . . . . . . . . | dt*gx
r21:   . . . . m . . . . . p . . . . . . . . . . . . . . . . . . . . D . . D . . . . . . . . . . . . . . . . . . . . | dt*gy
r22:   . . . . . m . . . . . p . . . . . . . . . . . . . . . . . . . . D . . D . . . . . . . . . . . . . . . . . . . | dt*gz
r23:   m . . D . . p . . D . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
r24:   . m . . D . . p . . D . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
r25:   . . m . . D . . p . . D . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
r26:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m p . . . a a . . . | 0
r27:   . . . . . . . . . m . . . . . p . . . . . . . . . . . . . . . . . D . . D . . . . . . . . . . . . . . . . . . | dt*gx
r28:   . . . . . . . . . . m . . . . . p . . . . . . . . . . . . . . . . . D . . D . . . . . . . . . . . . . . . . . | dt*gy
r29:   . . . . . . . . . . . m . . . . . p . . . . . . . . . . . . . . . . . D . . D . . . . . . . . . . . . . . . . | dt*gz
r30:   . . . . . . m . . D . . p . . D . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
r31:   . . . . . . . m . . D . . p . . D . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
r32:   . . . . . . . . m . . D . . p . . D . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
r33:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m p . . . a a . . | 0
r34:   . . . . . . . . . . . . . . . m . . . . . p . . . . . . . . . . . . . . D . . D . . . . . . . . . . . . . . . | dt*gx
r35:   . . . . . . . . . . . . . . . . m . . . . . p . . . . . . . . . . . . . . D . . D . . . . . . . . . . . . . . | dt*gy
r36:   . . . . . . . . . . . . . . . . . m . . . . . p . . . . . . . . . . . . . . D . . D . . . . . . . . . . . . . | dt*gz
r37:   . . . . . . . . . . . . m . . D . . p . . D . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
r38:   . . . . . . . . . . . . . m . . D . . p . . D . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
r39:   . . . . . . . . . . . . . . m . . D . . p . . D . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
r40:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m p . . . a a . | 0
r41:   . . . . . . . . . . . . . . . . . . . . . m . . . . . p . . . . . . . . . . . D . . D . . . . . . . . . . . . | dt*gx
r42:   . . . . . . . . . . . . . . . . . . . . . . m . . . . . p . . . . . . . . . . . D . . D . . . . . . . . . . . | dt*gy
r43:   . . . . . . . . . . . . . . . . . . . . . . . m . . . . . p . . . . . . . . . . . D . . D . . . . . . . . . . | dt*gz
r44:   . . . . . . . . . . . . . . . . . . m . . D . . p . . D . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
r45:   . . . . . . . . . . . . . . . . . . . m . . D . . p . . D . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
r46:   . . . . . . . . . . . . . . . . . . . . m . . D . . p . . D . . . . . . . . . . . . . . . . . . . . . . . . . | 0
r47:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m p . . . a a | 0
```

**Inequality system `G x + s = h`** (61 rows × 55 cols), with
`PositiveOrthantDim = 17` and `SocDims = [3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4]`
(rows g00–g16 are the orthant block; then four `SOC(3)` glideslope cones, four
`SOC(4)` velocity cones, four `SOC(4)` thrust-magnitude cones):

```
       0 1 2 3 4 5 6 7 8 9 101112131415161718192021222324252627282930313233343536373839404142434445464748495051525354 |  b/h
g00:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . c . . . . | 0
g01:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . c . . . | 0
g02:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . c . . | 0
g03:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . c . | 0
g04:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . U . . . . p . . . | mu2_1(1+z0_1)
g05:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . | -z0_1
g06:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . . . | z1_1
g07:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . U . . . . p . . | mu2_2(1+z0_2)
g08:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . | -z0_2
g09:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . . | z1_2
g10:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . U . . . . p . | mu2_3(1+z0_3)
g11:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . | -z0_3
g12:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . p . . . . . . | z1_3
g13:   m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g14:   . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g15:   . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g16:   . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g17:   k . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | -k*r_fx
g18:   . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | -r_fy
g19:   . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | -r_fz
g20:   . . . . . . k . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | -k*r_fx
g21:   . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | -r_fy
g22:   . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | -r_fz
g23:   . . . . . . . . . . . . k . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | -k*r_fx
g24:   . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | -r_fy
g25:   . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | -r_fz
g26:   . . . . . . . . . . . . . . . . . . k . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | -k*r_fx
g27:   . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | -r_fy
g28:   . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | -r_fz
g29:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | V_max
g30:   . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g31:   . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g32:   . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g33:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | V_max
g34:   . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g35:   . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g36:   . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g37:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | V_max
g38:   . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g39:   . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g40:   . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g41:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | V_max
g42:   . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g43:   . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g44:   . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | 0
g45:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . | 0
g46:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . . | 0
g47:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . . | 0
g48:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . . | 0
g49:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . | 0
g50:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . . | 0
g51:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . . | 0
g52:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . . | 0
g53:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . | 0
g54:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . . | 0
g55:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . . | 0
g56:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . . | 0
g57:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . | 0
g58:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . . | 0
g59:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . . | 0
g60:   . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . m . . . . . . . . . . . . . | 0
```

**Objective** `c` is all zeros except `c[49] = -1` (i.e. `IZ(N-1)` → maximize
final log-mass `z₄`).

Notice the structure the stencils predicted: identity entries for the boundary
rows, the dynamics rows marching down the diagonal in `±1 / D` triples (one node
to the next), the interior-node throttle/box rows (`g04`–`g12`), and each cone as
a tight little block touching a single node.

For **P3** instead: add a 56th column for `t`, change the objective to
`c[55] = 1`, replace the three terminal-position rows (r17–r19) with the single
altitude row `IX(4,0):p | r_fx`, and append a 4-row `SOC(4)` epigraph block
(`t` vs `r₄ − r_f`) — making `SocDims = [3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4]`.

### References

- Açıkmeşe & Ploen, *Convex Programming Approach to Powered Descent Guidance for[README.md](../README.md)
  Mars Landing*, JGCD, 2007 (docs/reference/_Convexprogramming_pdgforMarslanding.pdf)  — log-mass change of variables, discretization, and
  the Taylor-linearized throttle bounds (eqs. 34-36).
- Açıkmeşe, Carson & Blackmore, *Lossless Convexification of Nonconvex Control
  Bound and Pointing Constraints of the Soft Landing Optimal Control Problem*,
  IEEE TCST, 2013 (`docs/reference/gfold.pdf`) — the lossless-convexification
  proof for the thrust bounds and pointing constraint.

## Layout

- `ecos/` — vendored ECOS conic solver sources (embotech/ecos, develop branch;
  see `ecos/ECOS-VERSION.txt` for the pinned commit). GPLv3.
- `build-ecos.ps1` — compiles `native/ecos.dll` with Zig as a drop-in C
  compiler (`zig cc`, no MSVC needed). Expects `zig` on PATH; pass
  `-ZigExe <path>` to point at a portable copy.
- `native/` — build output (gitignored). Rebuild with the script.
- `shim/ecos_shim.c` — accessors for `pwork` internals compiled into
  ecos.dll, so managed code never depends on the struct layout.
- `Gfold.Core/` — the managed library: `EcosSolver.Solve(EcosProblem)`
  (standard conic form: min c'x s.t. Ax=b, Gx+s=h, s in R+^l x SOC(q...)),
  `SparseCcs` triplet->CCS builder, P/Invoke bindings with pinned-array
  lifetime management (ECOS retains caller pointers from setup to cleanup).
- `Gfold.Console/` — runs the P3 -> P4 flow on the reference "Numerical
  Example 1" case, verifies the result physically (dynamics replay, bounds),
  writes CSVs. `--check <csv>` audits any trajectory against the constraint
  set; `--verbose` shows ECOS iterations.
- `python_ref/` — CVXPY/Clarabel replica of the original Python for
  cross-validation (`gfold_ref.py [tf] [N] [--scaled] [--feascheck csv]`).


## Build notes

ECOS is built without `DLONG`, so the C `idxint` is a 32-bit `int` — the C#
interop maps `idxint -> int`, `pfloat -> double`. `CTRLC=0` keeps console
signal handlers out of the game process. Verbosity is a runtime setting
(`settings.verbose`), not a compile-time one.
[README.md](../README.md)
Smoke test (PowerShell):

```powershell
Add-Type -TypeDefinition 'using System; using System.Runtime.InteropServices;
public static class E { [DllImport("native\\ecos.dll")] public static extern IntPtr ECOS_ver(); }'
[Runtime.InteropServices.Marshal]::PtrToStringAnsi([E]::ECOS_ver())  # -> 2.0.10
```
