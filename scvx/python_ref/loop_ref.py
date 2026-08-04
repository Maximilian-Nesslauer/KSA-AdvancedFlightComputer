# Reference for the full SCvx LOOP, for validating Scvx.Core/Scvx6DofSolver.cs.
#
# Replicates 6dof.py's outer loop exactly (same seed, weights, trust-region
# schedule and convergence test) and dumps the converged trajectory plus the
# per-iteration trace. The trace matters as much as the final answer: SCvx is a
# nonconvex local method, so two implementations agreeing on the converged point
# is the real test, while agreeing iteration-by-iteration is a bonus that only
# holds while both take the same accept/reject decisions.
#
# Writes loop_ref.csv:
#   line 1: x0(14)
#   line 2: xf(13)
#   line 3: converged X (N*14)
#   line 4: converged U (N*4)
#   line 5: sigma, iterations, final cost, final defect_norm
#   line 6+: per-iteration trace rows "it,rho,accepted,tr,sigma,step,defect,cost"

import os
import numpy as np
import cvxpy as cp
import jax
import jax.numpy as jnp

jax.config.update("jax_enable_x64", True)

g_vec = jnp.array([0.0, 0.0, -9.81]); g0 = 9.81
m0 = 250000.0; Isp = 330.0; alpha = 1.0 / (Isp * g0)
Tmax = 6.0e6; Tmin = 0.40 * Tmax
L_arm = 25.0
I_vec = jnp.array([1.0e8, 1.0e8, 2.5e6]); I_inv = 1.0 / I_vec
delta_max = np.radians(10.0); tan_delta = float(np.tan(delta_max))
tau_roll_max = 1.0e5
tilt_max = np.radians(30.0); cos_tilt = float(np.cos(tilt_max))


def quat_mul(q, p):
    qw, qx, qy, qz = q; pw, px, py, pz = p
    return jnp.array([qw*pw - qx*px - qy*py - qz*pz,
                      qw*px + qx*pw + qy*pz - qz*py,
                      qw*py - qx*pz + qy*pw + qz*px,
                      qw*pz + qx*py - qy*px + qz*pw])


def quat_to_R(q):
    qw, qx, qy, qz = q
    return jnp.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qw*qz),   2*(qx*qz+qw*qy)],
        [2*(qx*qy+qw*qz),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qw*qx)],
        [2*(qx*qz-qw*qy),   2*(qy*qz+qw*qx),   1-2*(qx*qx+qy*qy)]])


def f(X, u):
    v = X[3:6]; q = X[6:10]; w = X[10:13]; m = X[13]
    tdx, tdy, T, tau_roll = u[0], u[1], u[2], u[3]
    T_body = jnp.array([tdx, tdy, T])
    rdot = v
    vdot = g_vec + (quat_to_R(q) @ T_body) / m
    qdot = 0.5 * quat_mul(q, jnp.array([0.0, w[0], w[1], w[2]]))
    tau = jnp.array([L_arm*tdy, -L_arm*tdx, tau_roll])
    wdot = I_inv * (tau - jnp.cross(w, I_vec * w))
    mdot = -alpha * T
    return jnp.concatenate([rdot, vdot, qdot, wdot, jnp.atleast_1d(mdot)])


f_batch = jax.jit(jax.vmap(f))
jac_dyn = jax.jit(jax.vmap(jax.jacobian(f, argnums=(0, 1))))

NX, NU, N = 14, 4, 30
dtau = 1.0 / (N - 1)

r0 = np.array([100.0, 0.0, 300.0]); v0 = np.array([0.0, 0.0, -50.0])
q0 = np.array([1.0, 0.0, 0.0, 0.0]); w0 = np.zeros(3)
rf = np.zeros(3); vf = np.zeros(3); qf = np.array([1.0, 0.0, 0.0, 0.0]); wf = np.zeros(3)

Xbar = np.zeros((N, NX))
Xbar[:, :3] = np.linspace(r0, rf, N)
Xbar[:, 3:6] = np.linspace(v0, vf, N)
Xbar[:, 6:10] = q0
Xbar[:, 13] = np.linspace(m0, 0.92*m0, N)
Ubar = np.zeros((N, NU)); Ubar[:, 2] = 1.05*m0*g0
sig_bar = 12.0

Xscale = np.array([100., 100., 300., 50., 50., 50., 1., 1., 1., 1., 1., 1., 1., m0])
Uscale = np.array([Tmax*tan_delta, Tmax*tan_delta, Tmax, tau_roll_max])
SIG_SCALE = 12.0
RHO_VC = 1e5; W_DU = 0.2; W_W = 1.0
tol = 8e-3; iters_max = 150
tr = 0.1
TR_MIN, TR_MAX = 1e-3, 0.1
RHO0, RHO1, RHO2 = 0.0, 0.25, 0.7
SHRINK, GROW = 0.5, 1.5
SIG_MIN, SIG_MAX = 5.0, 25.0

X = cp.Variable((N, NX)); U = cp.Variable((N, NU))
Wv = cp.Variable((N - 1, NX)); sigma = cp.Variable(nonneg=True)


def smoothing_np(Uv, Xv):
    return (W_DU * np.sum((np.diff(Uv, axis=0) / Uscale[None, :]) ** 2)
            + W_W * np.sum(Xv[:, 10:13] ** 2))


def true_cost(Xv, Uv, sg):
    fv = np.asarray(f_batch(jnp.asarray(Xv), jnp.asarray(Uv)))
    d = np.zeros((N - 1, NX))
    for n in range(N - 1):
        d[n] = Xv[n + 1] - Xv[n] - 0.5 * dtau * sg * (fv[n] + fv[n + 1])
    fuel = (m0 - Xv[-1, 13]) / m0
    defect = RHO_VC * np.sum((d / Xscale) ** 2)
    return fuel + smoothing_np(Uv, Xv) + defect, np.max(np.abs(d / Xscale))


trace = []
J_ref, _ = true_cost(Xbar, Ubar, sig_bar)
it = 0
while it < iters_max:
    Xj, Uj = jnp.asarray(Xbar), jnp.asarray(Ubar)
    f0 = np.asarray(f_batch(Xj, Uj))
    A, B = jac_dyn(Xj, Uj); A, B = np.asarray(A), np.asarray(B)

    g = {n: sigma*f0[n] + sig_bar*(A[n] @ (X[n]-Xbar[n]) + B[n] @ (U[n]-Ubar[n]))
         for n in range(N)}
    con = [X[0, :3] == r0, X[0, 3:6] == v0, X[0, 6:10] == q0,
           X[0, 10:13] == w0, X[0, 13] == m0,
           X[N-1, :3] == rf, X[N-1, 3:6] == vf,
           X[N-1, 6:10] == qf, X[N-1, 10:13] == wf]
    for n in range(N-1):
        con += [X[n+1] == X[n] + 0.5*dtau*(g[n] + g[n+1]) + Wv[n]]
    for n in range(N):
        qb = Xbar[n, 6:10]
        con += [qb @ X[n, 6:10] == 1.0]
        con += [U[n, 2] >= Tmin, U[n, 2] <= Tmax]
        con += [cp.norm(U[n, 0:2]) <= tan_delta*U[n, 2]]
        con += [cp.abs(U[n, 3]) <= tau_roll_max]
        qx, qy = qb[1], qb[2]
        R22 = 1 - 2*(qx*qx + qy*qy)
        dR22 = np.array([0.0, -4*qx, -4*qy, 0.0])
        con += [R22 + dR22 @ (X[n, 6:10] - qb) >= cos_tilt]
        con += [X[n, 2] >= -1.0]
    con += [cp.abs(cp.multiply(X - Xbar, 1.0/Xscale)) <= tr]
    con += [cp.abs(cp.multiply(U - Ubar, 1.0/Uscale)) <= tr]
    con += [cp.abs((sigma - sig_bar)/SIG_SCALE) <= tr]
    con += [sigma >= SIG_MIN, sigma <= SIG_MAX]

    objective = cp.Minimize(
        (m0 - X[N-1, 13])/m0
        + W_DU*cp.sum_squares(cp.multiply(cp.diff(U, axis=0), 1.0/Uscale[None, :]))
        + W_W*cp.sum_squares(X[:, 10:13])
        + RHO_VC*cp.sum_squares(cp.multiply(Wv, 1.0/Xscale[None, :])))
    prob = cp.Problem(objective, con)

    solved = True
    try:
        prob.solve(solver=cp.CLARABEL, verbose=False)
    except cp.error.SolverError:
        solved = False
    if not solved or X.value is None or prob.status not in ("optimal", "optimal_inaccurate"):
        tr = max(TR_MIN, tr * SHRINK)
        trace.append((it, float("nan"), 0, tr, sig_bar, 0.0, float("nan"), J_ref))
        it += 1
        if tr <= TR_MIN * 1.001:
            break
        continue

    fuel = (m0 - X.value[N-1, 13])/m0
    J_lin = fuel + smoothing_np(U.value, X.value) + RHO_VC*np.sum((Wv.value/Xscale[None, :])**2)
    J_true, defect_n = true_cost(X.value, U.value, float(sigma.value))
    pred_red = J_ref - J_lin
    act_red = J_ref - J_true
    rho = act_red/pred_red if abs(pred_red) > 1e-9 else (1.0 if act_red >= 0 else -1.0)

    dX = np.max(np.abs((X.value - Xbar)/Xscale))
    dU = np.max(np.abs((U.value - Ubar)/Uscale))
    ds = abs(float(sigma.value) - sig_bar)/SIG_SCALE
    used = max(dX, dU, ds)

    accepted = rho > RHO0
    step = 0.0
    if accepted:
        Xbar, Ubar = X.value.copy(), U.value.copy()
        Xbar[:, 6:10] /= np.linalg.norm(Xbar[:, 6:10], axis=1, keepdims=True)
        sig_bar = float(sigma.value)
        J_ref = J_true
        step = max(dX, ds)

    if rho < RHO1:
        tr = max(TR_MIN, tr*SHRINK)
    elif rho >= RHO2 and used >= 0.8*tr:
        tr = min(TR_MAX, tr*GROW)

    trace.append((it, rho, 1 if accepted else 0, tr, sig_bar, step, defect_n, J_ref))
    print(f"iter {it}: rho={rho:+.2f} {'accept' if accepted else 'REJECT'}  tr={tr:.3f}  "
          f"sig={sig_bar:.1f}  step={step:.2e}  defect={defect_n:.2e}  J={J_ref:.3e}")
    it += 1
    if accepted and step < tol and defect_n < 1e-3:
        print(f"converged after {it} iters")
        break

final_defect = trace[-1][6]
print(f"\nsigma={sig_bar:.6f}  iters={it}  cost={J_ref:.6e}  prop_used={m0 - Xbar[-1,13]:.0f} kg")

here = os.path.dirname(os.path.abspath(__file__))
out = os.path.join(here, "loop_ref.csv")


def row(a):
    return ",".join(repr(float(z)) for z in np.asarray(a).ravel())


with open(out, "w") as fh:
    fh.write(f"# N={N} converged_iters={it} solver=CLARABEL\n")
    fh.write(row(np.concatenate([r0, v0, q0, w0, [m0]])) + "\n")
    fh.write(row(np.concatenate([rf, vf, qf, wf])) + "\n")
    fh.write(row(Xbar) + "\n")
    fh.write(row(Ubar) + "\n")
    fh.write(row([sig_bar, it, J_ref, final_defect]) + "\n")
    for t in trace:
        fh.write(row(t) + "\n")
print(f"wrote {out}")
