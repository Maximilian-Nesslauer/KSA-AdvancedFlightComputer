# Reference solution of ONE SCvx convex subproblem, for validating
# Scvx.Core/Scvx6DofSubproblem.cs.
#
# 6dof.py builds this problem with CVXPY and lets it canonicalise to cone form
# invisibly; the C# side writes that cone form out by hand. Solving the very same
# subproblem — same reference trajectory, same trust radius, same linearisation —
# and comparing the optimal X, U and sigma is what proves the hand-assembly is
# right.
#
# Deliberately solved with ECOS, not Clarabel: the C# side calls ECOS, so using
# the same solver removes "is the port wrong or is it just a different solver"
# from the diff. Falls back to Clarabel if this cvxpy build dropped ECOS.
#
# Writes sub_ref.csv:  one header line of metadata, then
#   line 1: x0(14)
#   line 2: xf(13)
#   line 3: xbar flattened (N*14)
#   line 4: ubar flattened (N*4)
#   line 5: sig_bar, tr
#   line 6: solved X flattened (N*14)
#   line 7: solved U flattened (N*4)
#   line 8: solved sigma, objective

import os
import sys
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
RHO_VC = 1e5; W_DU = 0.2; W_W = 1.0
SIG_SCALE = 12.0; SIG_MIN, SIG_MAX = 5.0, 25.0
Xscale = np.array([100., 100., 300., 50., 50., 50., 1., 1., 1., 1., 1., 1., 1., m0])
Uscale = np.array([Tmax*tan_delta, Tmax*tan_delta, Tmax, tau_roll_max])

# --- the subproblem instance: the SCvx seed, i.e. exactly iteration 0 of 6dof.py ---
r0 = np.array([100.0, 0.0, 300.0]); v0 = np.array([0.0, 0.0, -50.0])
q0 = np.array([1.0, 0.0, 0.0, 0.0]); w0 = np.zeros(3)
rf = np.zeros(3); vf = np.zeros(3); qf = np.array([1.0, 0.0, 0.0, 0.0]); wf = np.zeros(3)
x0 = np.concatenate([r0, v0, q0, w0, [m0]])
xf = np.concatenate([rf, vf, qf, wf])          # 13, mass free

Xbar = np.zeros((N, NX))
Xbar[:, :3] = np.linspace(r0, rf, N)
Xbar[:, 3:6] = np.linspace(v0, vf, N)
Xbar[:, 6:10] = q0
Xbar[:, 13] = np.linspace(m0, 0.92*m0, N)
Ubar = np.zeros((N, NU)); Ubar[:, 2] = 1.05*m0*g0
sig_bar = 12.0
tr = 0.1

f0 = np.asarray(f_batch(jnp.asarray(Xbar), jnp.asarray(Ubar)))
A, B = jac_dyn(jnp.asarray(Xbar), jnp.asarray(Ubar))
A, B = np.asarray(A), np.asarray(B)

X = cp.Variable((N, NX)); U = cp.Variable((N, NU))
Wv = cp.Variable((N-1, NX)); sigma = cp.Variable(nonneg=True)

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
solver = None
for cand in ("ECOS", "CLARABEL"):
    if cand in cp.installed_solvers():
        solver = cand
        break
prob.solve(solver=getattr(cp, solver), verbose=False)
print(f"solver={solver}  status={prob.status}  objective={prob.value:.10e}")
if X.value is None:
    sys.exit("subproblem did not solve")

here = os.path.dirname(os.path.abspath(__file__))
out = os.path.join(here, "sub_ref.csv")


def row(a):
    return ",".join(repr(float(z)) for z in np.asarray(a).ravel())


with open(out, "w") as fh:
    fh.write(f"# N={N} NX={NX} NU={NU} solver={solver} status={prob.status}\n")
    fh.write(row(x0) + "\n")
    fh.write(row(xf) + "\n")
    fh.write(row(Xbar) + "\n")
    fh.write(row(Ubar) + "\n")
    fh.write(row([sig_bar, tr]) + "\n")
    fh.write(row(X.value) + "\n")
    fh.write(row(U.value) + "\n")
    fh.write(row([float(sigma.value), float(prob.value)]) + "\n")
    # Virtual control too: it lets the C# side reconstruct the FULL primal point
    # and audit every constraint row against a known-feasible solution, which
    # separates "my formulation is wrong" from "the solver struggled".
    fh.write(row(Wv.value) + "\n")
print(f"wrote {out}")
