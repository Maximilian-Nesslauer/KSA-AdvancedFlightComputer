# Reference dynamics + Jacobians for validating Scvx.Core/Dynamics6Dof.cs.
#
# Model identical to 6dof.py (that file is the source of truth); this one only
# evaluates f, A = df/dX and B = df/du at a set of test points and writes them
# for the C# side to diff against. JAX supplies every slope by autodiff, so
# agreement validates the hand-written forward-mode AD in C#.
#
# Test points deliberately span the regimes the SCvx loop actually visits:
# an identity-attitude hover, a tilted vehicle with body rates (where R(q),
# q(x)w and w x Iw are all non-trivial), the gimbal at its stop, and pseudo-
# random states — the last catching sign errors a symmetric case would hide.
#
# Usage:  python dyn_ref.py            (writes dyn_ref.csv next to this file)

import os
import numpy as np
import jax
import jax.numpy as jnp

jax.config.update("jax_enable_x64", True)

g_vec = jnp.array([0.0, 0.0, -9.81])
g0 = 9.81
Isp = 330.0
alpha = 1.0 / (Isp * g0)
L_arm = 25.0
I_vec = jnp.array([1.0e8, 1.0e8, 2.5e6])
I_inv = 1.0 / I_vec


def quat_mul(q, p):
    qw, qx, qy, qz = q
    pw, px, py, pz = p
    return jnp.array([
        qw * pw - qx * px - qy * py - qz * pz,
        qw * px + qx * pw + qy * pz - qz * py,
        qw * py - qx * pz + qy * pw + qz * px,
        qw * pz + qx * py - qy * px + qz * pw,
    ])


def quat_to_R(q):
    qw, qx, qy, qz = q
    return jnp.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz),     2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy),     2 * (qy * qz + qw * qx),     1 - 2 * (qx * qx + qy * qy)],
    ])


def f(X, u):
    v = X[3:6]
    q = X[6:10]
    w = X[10:13]
    m = X[13]
    tdx, tdy, T, tau_roll = u[0], u[1], u[2], u[3]
    T_body = jnp.array([tdx, tdy, T])
    thrust_in = quat_to_R(q) @ T_body
    rdot = v
    vdot = g_vec + thrust_in / m
    qdot = 0.5 * quat_mul(q, jnp.array([0.0, w[0], w[1], w[2]]))
    tau = jnp.array([L_arm * tdy, -L_arm * tdx, tau_roll])
    wdot = I_inv * (tau - jnp.cross(w, I_vec * w))
    mdot = -alpha * T
    return jnp.concatenate([rdot, vdot, qdot, wdot, jnp.atleast_1d(mdot)])


jac = jax.jit(jax.jacobian(f, argnums=(0, 1)))

NX, NU = 14, 4
m0 = 250000.0
Tmax = 6.0e6
tan_delta = float(np.tan(np.radians(10.0)))


def unit(q):
    return q / np.linalg.norm(q)


cases = []

# 1. hover, identity attitude, zero rates -- the seed the SCvx loop starts from
cases.append((
    np.concatenate([[100.0, 0.0, 300.0], [0.0, 0.0, -50.0], [1.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0], [m0]]),
    np.array([0.0, 0.0, 1.05 * m0 * g0, 0.0])))

# 2. tilted with body rates: R(q), q(x)w and w x (Iw) all non-trivial at once
cases.append((
    np.concatenate([[40.0, -15.0, 120.0], [3.0, -2.0, -30.0],
                    unit(np.array([0.966, 0.129, -0.224, 0.048])),
                    [0.05, -0.03, 0.02], [0.86 * m0]]),
    np.array([0.10 * Tmax * tan_delta, -0.22 * Tmax * tan_delta, 0.62 * Tmax, 2.5e4])))

# 3. gimbal on its stop, roll torque saturated, near touchdown
cases.append((
    np.concatenate([[2.0, 1.0, 12.0], [-0.4, 0.2, -3.0],
                    unit(np.array([0.999, -0.02, 0.03, -0.01])),
                    [-0.01, 0.015, -0.004], [0.83 * m0]]),
    np.array([Tmax * tan_delta * 0.40, Tmax * tan_delta * 0.40, 0.40 * Tmax, 1.0e5])))

# 4-8. pseudo-random -- asymmetric in every component, so a transposed index or
#      a flipped sign cannot cancel out the way it can in a symmetric case
rng = np.random.default_rng(20260803)
for _ in range(5):
    q = unit(rng.normal(size=4))
    cases.append((
        np.concatenate([rng.normal(0, 200, 3), rng.normal(0, 60, 3), q,
                        rng.normal(0, 0.15, 3), [m0 * rng.uniform(0.7, 1.0)]]),
        np.array([rng.normal(0, Tmax * tan_delta * 0.5),
                  rng.normal(0, Tmax * tan_delta * 0.5),
                  Tmax * rng.uniform(0.4, 1.0),
                  rng.normal(0, 5e4)])))

here = os.path.dirname(os.path.abspath(__file__))
out = os.path.join(here, "dyn_ref.csv")
with open(out, "w") as fh:
    fh.write(f"# {len(cases)} cases; per case: X({NX}), U({NU}), f({NX}), "
             f"A({NX}x{NX} row-major), B({NX}x{NU} row-major)\n")
    for X, U in cases:
        fv = np.asarray(f(jnp.asarray(X), jnp.asarray(U)))
        A, B = jac(jnp.asarray(X), jnp.asarray(U))
        A, B = np.asarray(A), np.asarray(B)
        row = np.concatenate([X, U, fv, A.ravel(), B.ravel()])
        fh.write(",".join(repr(float(z)) for z in row) + "\n")

print(f"wrote {len(cases)} cases -> {out}")
print(f"  per row: {NX + NU + NX + NX * NX + NX * NU} values")
