# Reference for the 3-DOF glide-and-burn SCvx loop, for validating Scvx/ThreeDof.
#
# 3dof.py's loop verbatim (same model, seed, weights, trust-region schedule and
# convergence test), minus the plots, dumping the converged trajectory and the
# per-iteration trace. As with loop_ref.py, the converged point is the real test;
# agreeing iteration by iteration only holds while both sides take the same
# accept/reject decisions, and this side solves with CLARABEL where C# uses SCS.
#
# Writes threedof_ref.csv:
#   line 1: x0(7)
#   line 2: xf(6)  terminal position and velocity
#   line 3: converged X (N*7)
#   line 4: converged B (N*3)
#   line 5: converged throttle fraction Tm/Tmax (N)
#   line 6: sigma_coast, sigma_burn, iterations, final cost, final defect_norm
#   line 7+: per-iteration trace rows "it,rho,accepted,tr,sigma_coast,sigma_burn,step,defect,cost"
#
#   usage: python threedof_ref.py      (needs cvxpy with CLARABEL, and jax)
import os
import time
import numpy as np
import cvxpy as cp
import jax
import jax.numpy as jnp

jax.config.update("jax_enable_x64", True)   # match cvxpy's float64 -- avoids noisy Jacobians

# =====================================================================
# 3DOF powered-descent LANDING via SCvx, with JAX-computed Jacobians.
#
# ATTITUDE is an independent control. The body axis b (unit vector) sets BOTH:
#   * thrust direction:  T = Tm * b     (Tm = scalar throttle magnitude)
#   * angle of attack:   alpha = angle(b, v)  ->  lift, INDEPENDENT of thrust.
# So the vehicle makes lift via attitude even with the engine OFF (entry/coast).
#
# TWO PHASES, each with its own free duration (time-dilation):
#   1. COAST -- engine off (Tm=0), but attitude still makes lift/drag.
#   2. BURN  -- single continuous landing burn, Tmin <= Tm <= Tmax.
#
# Separating attitude from thrust makes the 40% floor CONVEX (Tmin <= Tm <= Tmax
# is a scalar box). The only non-convexity left is |b| = 1, handled by the SCvx
# linearisation  bbar . b = 1  (tangent plane to the unit sphere; exact at the fixpoint).
#
#   state    X = [ r(3), v(3), m(1) ]
#   control  b(3) (attitude, |b|=1)  and  Tm (scalar throttle)
#   times    sigma_coast, sigma_burn  (both free)
#
#   rdot = v
#   vdot = g + (1/m)(Tm*b + D + L)
#   mdot = -alpha * Tm
#
# We hand-write only f(); jax.jacobian gives every slope.
# =====================================================================

## ---- vehicle / environment constants ----
g_vec = jnp.array([0.0, 0.0, -9.81])
rho0  = 1.225
H     = 8.5e3
S     = 10.8
Cd0   = 0.8
k     = 0.5
Cl_a  = 2.0
Isp   = 282.0
alpha = 1.0 / (Isp * 9.81)

##Placeholder wave drag
gamma = 1.4
R     = 287
M_p   = 1.1
M_W   = 0.3        # renamed from W -- 'W' is the virtual-control cp.Variable below
M_A   = 1.25

Tmax = 845000.0
Tmin = 0.40 * Tmax        # 40% throttle floor (when the engine is lit)
m0   = 28000.0

alpha_max = np.radians(10.0)
sin_max   = np.sin(alpha_max)
V_THRESH  = 5.0

V_EPS2 = 1e-4


def f(X, b, Tm):
    """Unified continuous dynamics. b = body axis (unit), Tm = throttle magnitude.
    Lift comes from the attitude b (works with Tm=0 -> coast lift). The ONLY hand-written physics."""
    r, v, m = X[:3], X[3:6], X[6]
    h = r[2]

    rho   = rho0 * jnp.exp(-h / H)
    vmag2 = jnp.dot(v, v)
    vmag  = jnp.sqrt(vmag2 + V_EPS2)
    vhat  = v / vmag
    q     = 0.5 * rho * vmag2
    
    a    = jnp.sqrt(gamma* R) * jnp.sqrt(jnp.maximum(216.65, 288.15 - 0.0065 * h))
    Mach = jnp.linalg.norm(vmag)/a

    bv = jnp.dot(b, vhat)                      # cos(angle between body axis and velocity)
    p  = b - bv * vhat                         # body axis projected _|_ v ; |p| = sin(alpha)
    s  = bv / jnp.sqrt(bv * bv + 1e-6)         # smooth sign(b.v): lift opposes tilt in retrograde
    pp = jnp.dot(p, p)                         # sin^2(alpha); smooth (no bare sqrt)
    L  = q * S * Cl_a * s * p                  # lift _|_ v, from ATTITUDE (independent of thrust)
    Cd = Cd0 + k * Cl_a**2 * pp
    Cd = Cd * (1 + M_A * jnp.exp(-(((Mach - M_p) / M_W) ** 2)))
    D  = -q * S * Cd * vhat

    T  = Tm * b                                # thrust along the body axis
    rdot = v
    vdot = g_vec + (T + D + L) / m
    mdot = -alpha * Tm
    return jnp.concatenate([rdot, vdot, jnp.atleast_1d(mdot)])


def p_vec(X, b):
    """Body axis projected _|_ v; |p| = sin(angle of attack). Used for the stall limit."""
    v = X[3:6]
    vhat = v / jnp.sqrt(jnp.dot(v, v) + V_EPS2)
    return b - jnp.dot(b, vhat) * vhat


f_batch     = jax.jit(jax.vmap(f))
jac_dyn     = jax.jit(jax.vmap(jax.jacobian(f, argnums=(0, 1, 2))))   # A(7,7), Bb(7,3), Bt(7,)
p_batch     = jax.jit(jax.vmap(p_vec))
p_jac_batch = jax.jit(jax.vmap(jax.jacobian(p_vec, argnums=(0, 1))))


## ---- scenario (landing) ----
NX = 7
N    = 30
K    = 12                 # split: coast = intervals 0..K-1 (nodes 0..K); burn = K..N-2
dtau_c = 1.0 / K
dtau_b = 1.0 / (N - 1 - K)

r0 = np.array([2000.0, 0.0, 20000.0])
rf = np.array([0.0, 0.0, 0.0])
approach = (rf - r0) / np.linalg.norm(rf - r0)
approach = np.array([0, 0, -1])
speed0 = 600.0
v0 = speed0 * approach
vf = np.array([0.0, 0.0, 0.0])

## ---- seed: straight-line in position/velocity (hits BOTH hard boundary constraints exactly,
## which is what matters with the hard terminal + trust region), with a high-drag sigma guess. ----
Xbar = np.zeros((N, NX))
Xbar[:, :3] = np.linspace(r0, rf, N)
Xbar[:, 3:6] = np.linspace(v0, vf, N)
Xbar[:K, 6] = m0
Xbar[K:, 6] = np.linspace(m0, 0.85 * m0, N - K)
Bbar = np.zeros((N, 3))                           # body-axis seed: retrograde of the seed velocity
for n in range(N):
    vv = Xbar[n, 3:6]; nv = np.linalg.norm(vv)
    Bbar[n] = -vv / nv if nv > 1.0 else -approach
Tmbar = np.zeros(N)
Tmbar[K:] = 0.6 * Tmax
sigc_bar = 22.0     # high drag -> long coast (let the air brake), short late burn
sigb_bar = 27.0
X_seed = Xbar.copy()

## ---- scaling ----
Xscale = np.array([1000., 1000., 6000., 300., 300., 300., m0])
SIG_SCALE = 30.0

## ---- SCvx weights + trust-region radius update ----
RHO_VC = 1e5
W_DTHR = 0.05     # throttle-RATE smoothing (penalises |dTm| changes, NOT magnitude -> no long-burn bias)
W_DATT = 0.10     # attitude-RATE smoothing (penalises |db| changes -> no attitude chatter)
tol      = 8e-3
iters_max = 120

tr             = 0.1
TR_MIN, TR_MAX = 1e-3, 0.1
RHO0, RHO1, RHO2 = 0.0, 0.25, 0.7
SHRINK, GROW   = 0.5, 1.5
COAST_MIN, COAST_MAX = 0.0, 40.0
BURN_MIN, BURN_MAX   = 5.0, 50.0

## ---- decision variables ----
X = cp.Variable((N, NX))
B = cp.Variable((N, 3))                   # attitude (body axis)
Tm = cp.Variable(N)                       # throttle magnitude
W = cp.Variable((N - 1, NX))
sig_coast = cp.Variable(nonneg=True)
sig_burn  = cp.Variable(nonneg=True)

Zg = np.zeros(N)


def true_cost(Xv, Bv, Tmv, sgc, sgb):
    """Merit = min-fuel + smoothing + RHO_VC*(TRUE nonlinear defect), normalised.
    Coast intervals use real f with Tm=0 (attitude lift still on); burn uses real f with Tm."""
    fc = np.asarray(f_batch(jnp.asarray(Xv), jnp.asarray(Bv), jnp.asarray(Zg)))
    fb = np.asarray(f_batch(jnp.asarray(Xv), jnp.asarray(Bv), jnp.asarray(Tmv)))
    d = np.zeros((N - 1, NX))
    for n in range(K):
        d[n] = Xv[n + 1] - Xv[n] - 0.5 * dtau_c * sgc * (fc[n] + fc[n + 1])
    for n in range(K, N - 1):
        d[n] = Xv[n + 1] - Xv[n] - 0.5 * dtau_b * sgb * (fb[n] + fb[n + 1])
    fuel   = (m0 - Xv[-1, 6]) / m0
    smooth = W_DTHR * np.sum((np.diff(Tmv[K:]) / Tmax) ** 2) + W_DATT * np.sum(np.diff(Bv, axis=0) ** 2)
    defect = RHO_VC * np.sum((d / Xscale) ** 2)
    return fuel + smooth + defect, np.max(np.abs(d / Xscale))


history = []
trace = []
J_ref, _ = true_cost(Xbar, Bbar, Tmbar, sigc_bar, sigb_bar)
loop_t0 = time.perf_counter()
it = 0
while it < iters_max:
    Xj, Bj, Tmj = jnp.asarray(Xbar), jnp.asarray(Bbar), jnp.asarray(Tmbar)
    ## coast linearisation (Tm=0) and burn linearisation (Tm), both via JAX.
    ## Both carry the attitude (b) Jacobian, so lift responds to attitude in BOTH phases.
    f0c = np.asarray(f_batch(Xj, Bj, jnp.asarray(Zg)))
    Ac, Bbc, _ = jac_dyn(Xj, Bj, jnp.asarray(Zg)); Ac, Bbc = np.asarray(Ac), np.asarray(Bbc)
    f0b = np.asarray(f_batch(Xj, Bj, Tmj))
    Ab, Bbb, Bt = jac_dyn(Xj, Bj, Tmj); Ab, Bbb, Bt = np.asarray(Ab), np.asarray(Bbb), np.asarray(Bt)
    p0 = np.asarray(p_batch(Xj, Bj))
    JpX, Jpb = p_jac_batch(Xj, Bj); JpX, Jpb = np.asarray(JpX), np.asarray(Jpb)

    gc = {n: sig_coast * f0c[n] + sigc_bar * (Ac[n] @ (X[n] - Xbar[n]) + Bbc[n] @ (B[n] - Bbar[n]))
          for n in range(K + 1)}
    gb = {n: sig_burn * f0b[n] + sigb_bar * (Ab[n] @ (X[n] - Xbar[n])
                                             + Bbb[n] @ (B[n] - Bbar[n])
                                             + Bt[n] * (Tm[n] - Tmbar[n]))
          for n in range(K, N)}

    con = [
        X[0, :3] == r0, X[0, 3:6] == v0, X[0, 6] == m0,
        X[N - 1, :3] == rf, X[N - 1, 3:6] == vf,
    ]
    for n in range(K):
        con += [X[n + 1] == X[n] + 0.5 * dtau_c * (gc[n] + gc[n + 1]) + W[n]]
    for n in range(K, N - 1):
        con += [X[n + 1] == X[n] + 0.5 * dtau_b * (gb[n] + gb[n + 1]) + W[n]]

    ## unit attitude (linearised sphere: tangent plane at Bbar) -- all nodes
    for n in range(N):
        con += [Bbar[n] @ B[n] == 1.0]
    ## engine off in coast; convex throttle box in burn (no LCvx slack needed now)
    for n in range(K):
        con += [Tm[n] == 0]
    for n in range(K, N):
        con += [Tm[n] >= Tmin, Tm[n] <= Tmax]

    ## stall limit + retrograde body, ALL nodes with meaningful speed (coast steers too)
    for n in range(N):
        vb = Xbar[n, 3:6]; sp = np.linalg.norm(vb)
        if sp > V_THRESH:
            p_lin = p0[n] + JpX[n] @ (X[n] - Xbar[n]) + Jpb[n] @ (B[n] - Bbar[n])
            con += [cp.norm(p_lin) <= sin_max]
            con += [B[n] @ (vb / sp) <= 0]
            
    ## monotonic approach - do not overshoot the pad and return by too much
    for n in range(N):
        con += [X[n, 3] <= 2]
        con += [X[n, 0] >= -10]

    ## trust region (normalised box) over X, attitude, throttle, and both sigmas
    con += [cp.abs(cp.multiply(X - Xbar, 1.0 / Xscale)) <= tr]
    con += [cp.abs(B - Bbar) <= tr]                       # b is O(1); tr ~ rotation per iter
    con += [cp.abs((Tm - Tmbar) / Tmax) <= tr]
    con += [cp.abs((sig_coast - sigc_bar) / SIG_SCALE) <= tr]
    con += [cp.abs((sig_burn - sigb_bar) / SIG_SCALE) <= tr]
    con += [sig_coast >= COAST_MIN, sig_coast <= COAST_MAX,
            sig_burn >= BURN_MIN, sig_burn <= BURN_MAX]

    objective = cp.Minimize(
        (m0 - X[N - 1, 6]) / m0
        + W_DTHR * cp.sum_squares(cp.diff(Tm[K:]) / Tmax)        # throttle-rate (no magnitude bias)
        + W_DATT * cp.sum_squares(cp.diff(B, axis=0))            # attitude-rate
        + RHO_VC * cp.sum_squares(cp.multiply(W, 1.0 / Xscale[None, :]))
    )
    problem = cp.Problem(objective, con)

    t0 = time.perf_counter()
    solved = True
    try:
        problem.solve(solver=cp.CLARABEL, verbose=False)
    except cp.error.SolverError:
        solved = False
    wall_ms = (time.perf_counter() - t0) * 1e3
    if not solved or X.value is None or problem.status not in ("optimal", "optimal_inaccurate"):
        tr = max(TR_MIN, tr * SHRINK)
        why = "exception" if not solved else problem.status
        print(f"iter {it}: solve failed ({why}) -> shrink tr={tr:.3f}")
        it += 1
        if tr <= TR_MIN * 1.001:
            print("trust region collapsed -- stopping"); break
        continue

    ## ratio test
    fuel   = (m0 - X.value[N - 1, 6]) / m0
    smooth = W_DTHR * np.sum((np.diff(Tm.value[K:]) / Tmax) ** 2) + W_DATT * np.sum(np.diff(B.value, axis=0) ** 2)
    J_lin = fuel + smooth + RHO_VC * np.sum((W.value / Xscale[None, :]) ** 2)
    J_true, defect_n = true_cost(X.value, B.value, Tm.value, float(sig_coast.value), float(sig_burn.value))
    pred_red = J_ref - J_lin
    act_red  = J_ref - J_true
    rho = act_red / pred_red if abs(pred_red) > 1e-9 else (1.0 if act_red >= 0 else -1.0)

    dX = np.max(np.abs((X.value - Xbar) / Xscale))
    dB = np.max(np.abs(B.value - Bbar))
    dT = np.max(np.abs((Tm.value - Tmbar) / Tmax))
    ds = max(abs(float(sig_coast.value) - sigc_bar), abs(float(sig_burn.value) - sigb_bar)) / SIG_SCALE
    used = max(dX, dB, dT, ds)
    if rho > RHO0:
        Xbar, Tmbar = X.value.copy(), Tm.value.copy()
        Bbar = B.value / np.linalg.norm(B.value, axis=1, keepdims=True)   # re-project onto |b|=1
        sigc_bar, sigb_bar = float(sig_coast.value), float(sig_burn.value)
        J_ref = J_true
        history.append((X.value.copy(), Bbar.copy(), Tm.value.copy()))
        tag, step = "accept", max(dX, ds)
    else:
        tag, step = "REJECT", 0.0

    if rho < RHO1:
        tr = max(TR_MIN, tr * SHRINK)
    elif rho >= RHO2 and used >= 0.8 * tr:
        tr = min(TR_MAX, tr * GROW)

    trace.append((it, rho, 1.0 if tag == "accept" else 0.0, tr, sigc_bar, sigb_bar, step, defect_n, J_true))
    print(f"iter {it}: rho={rho:+.2f} {tag}  tr={tr:.3f}  tc={sigc_bar:.1f} tb={sigb_bar:.1f}s  "
          f"step={step:.2e}  defect_n={defect_n:.2e}  J={J_true:.3e}  wall={wall_ms:.1f} ms")
    it += 1
    if tag == "accept" and step < tol and defect_n < 1e-3:
        print(f"converged after {it} iters (step={step:.2e}, defect_n={defect_n:.2e})")
        break


## ---- dump ----
Xc, Bc, Tmc = history[-1]
J_fin, d_fin = true_cost(Xc, Bc, Tmc, sigc_bar, sigb_bar)
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "threedof_ref.csv")
fmt = lambda a: ",".join(repr(float(z)) for z in np.ravel(a))
with open(out, "w") as w:
    w.write(f"# N={N} K={K} converged_iters={it} solver=CLARABEL\n")
    w.write(f"# consts tmax={Tmax} floor={Tmin / Tmax} isp={Isp} area={S} cd0={Cd0} k={k} cl_a={Cl_a} "
            f"rho0={rho0} h={H} alpha_max_deg={np.degrees(alpha_max)} v_thresh={V_THRESH} "
            f"rho_vc={RHO_VC} w_dthr={W_DTHR} w_datt={W_DATT} sig_scale={SIG_SCALE}\n")
    w.write(fmt(np.concatenate([r0, v0, [m0]])) + "\n")
    w.write(fmt(np.concatenate([rf, vf])) + "\n")
    w.write(fmt(Xc) + "\n")
    w.write(fmt(Bc) + "\n")
    w.write(fmt(Tmc / Tmax) + "\n")
    w.write(fmt([sigc_bar, sigb_bar, it, J_fin, d_fin]) + "\n")
    for row in trace:
        w.write(fmt(row) + "\n")
print(f"wrote {out}")
