# Reference implementation for validating Gfold.Core: a Python 3 port of the
# original GFOLD_static_p3p4.py (P3 minimum landing error -> P4 minimum fuel),
# with parameters identical to GfoldParams defaults and the same N / tf
# parametrization as GfoldPlanner (dt = tf / (N-1)).
#
# Solved with CVXPY + Clarabel — deliberately a DIFFERENT solver from the C#
# side's ECOS, so agreement validates the formulation, not just the binding.
#
# Matches the C# port's two documented deviations from the original script:
# glideslope cone over the horizontal components (y,z), and no thrust
# lower-bound rows (the original omitted them despite its comment).
#
# Usage:  python gfold_ref.py [tf] [N]
# Writes py_p3.csv / py_p4.csv, then compares against ../gfold_p3.csv and
# ../gfold_p4.csv if present.

import sys
import os
import numpy as np
import cvxpy as cp

# --- parameters: G-FOLD-Python "Numerical Example 1", in lockstep with
# --- Gfold.Core/GfoldParams.cs ---
g0 = 9.80665
m_dry = 2000.0
m_fuel = 300.0
m_wet = m_dry + m_fuel
Isp = 203.94
T_max = 24000.0
r1 = 0.2 * T_max
r2 = 0.8 * T_max
V_max = 90.0
y_gs = np.radians(30.0)        # glideslope
p_cs = np.radians(45.0)        # pointing
alpha = 1.0 / (Isp * g0)
g = np.array([-3.71, 0.0, 0.0])
r0 = np.array([2400.0, 2000.0, 0.0])
v0 = np.array([-40.0, 30.0, 0.0])
rf = np.array([0.0, 0.0, 0.0])
vf = np.array([0.0, 0.0, 0.0])

ORIG_GS = "--orig-gs" in sys.argv  # reproduce the repo's [0:2] glideslope quirk
sys.argv = [a for a in sys.argv if a != "--orig-gs"]

# --scaled: nondimensionalize exactly like GfoldPlanner (length scale = |r0|,
# time scale sqrt(L/g)) before solving. Mass terms are invariant under this
# scaling, so reported fuel numbers stay comparable.
if "--scaled" in sys.argv:
    sys.argv = [a for a in sys.argv if a != "--scaled"]
    _L = max(1000.0, np.linalg.norm(r_ := np.array([2400.0, 2000.0, 0.0])))
    _T = np.sqrt(_L / 3.71)
    _V, _A = _L / _T, _L / _T ** 2
    g = g / _A
    r0 = r0 / _L
    v0 = v0 / _V
    rf = rf / _L
    vf = vf / _V
    V_max = V_max / _V
    r1 = r1 / _A
    r2 = r2 / _A
    alpha = alpha * _A * _T
    SCALE_T, SCALE_L, SCALE_V = _T, _L, _V
else:
    SCALE_T, SCALE_L, SCALE_V = 1.0, 1.0, 1.0

FEASCHECK = None
if "--feascheck" in sys.argv:
    i = sys.argv.index("--feascheck")
    FEASCHECK = sys.argv[i + 1]
    del sys.argv[i:i + 2]

_numeric = [a for a in sys.argv[1:] if a.replace(".", "", 1).isdigit()]
tf = (float(_numeric[0]) if len(_numeric) > 0 else 81.0) / SCALE_T
N = int(float(_numeric[1])) if len(_numeric) > 1 else 120
dt = tf / (N - 1)


def build(program, rf_fixed=None):
    x = cp.Variable((6, N))
    u = cp.Variable((3, N))
    z = cp.Variable(N)
    s = cp.Variable(N)

    con = [
        x[0:3, 0] == r0,
        x[3:6, 0] == v0,
        x[3:6, N - 1] == vf,
        s[N - 1] == 0,
        u[:, 0] == s[0] * np.array([1, 0, 0]),
        u[:, N - 1] == s[N - 1] * np.array([1, 0, 0]),
        z[0] == np.log(m_wet),
    ]
    if program == 3:
        con += [x[0, N - 1] == 0]
    else:
        con += [x[0:3, N - 1] == rf_fixed]

    for n in range(N - 1):
        con += [x[3:6, n + 1] == x[3:6, n] + (dt / 2) * ((u[:, n] + g) + (u[:, n + 1] + g))]
        con += [x[0:3, n + 1] == x[0:3, n] + (dt / 2) * (x[3:6, n + 1] + x[3:6, n])]
        if ORIG_GS:
            # the repo's literal line: norms components [0:2] = (altitude, y)
            con += [cp.norm(x[0:2, n] - rf[0:2]) - (x[0, n] - rf[0]) / np.tan(y_gs) <= 0]
        else:
            # glideslope over horizontal components (y,z) against altitude x
            con += [cp.norm(x[1:3, n] - rf[1:3]) - (x[0, n] - rf[0]) / np.tan(y_gs) <= 0]
        con += [cp.norm(x[3:6, n]) <= V_max]
        con += [z[n + 1] == z[n] - (alpha * dt / 2) * (s[n] + s[n + 1])]
        con += [cp.norm(u[:, n]) <= s[n]]
        con += [u[0, n] >= np.cos(p_cs) * s[n]]

        if n > 0:
            z0_term = m_wet - alpha * r2 * n * dt
            z1_term = m_wet - alpha * r1 * n * dt
            z0 = np.log(z0_term)
            z1 = np.log(z1_term)
            mu_2 = r2 / z0_term
            con += [s[n] <= mu_2 * (1 - (z[n] - z0))]
            con += [z[n] >= z0]
            con += [z[n] <= z1]

    con += [x[0, 0:N - 1] >= 0]

    if program == 3:
        objective = cp.Minimize(cp.norm(x[0:3, N - 1] - rf))
    else:
        objective = cp.Maximize(z[N - 1])
    return cp.Problem(objective, con), x, u, z, s


VERBOSE = "--verbose" in sys.argv
sys.argv = [a for a in sys.argv if a != "--verbose"]


def solve(program, rf_fixed=None):
    problem, x, u, z, s = build(program, rf_fixed)
    problem.solve(solver=cp.CLARABEL, verbose=VERBOSE)
    return problem, x.value, u.value, z.value, s.value


def feascheck(csv_path):
    # Evaluate a foreign trajectory against THESE constraint objects: the
    # violations come from CVXPY itself, so this cannot repeat a hand-port
    # misreading of the constraint set.
    d = np.genfromtxt(csv_path, delimiter=",", names=True)
    xv = np.vstack([d["rx"], d["ry"], d["rz"], d["vx"], d["vy"], d["vz"]])
    uv = np.vstack([d["ux"], d["uy"], d["uz"]])
    zv = np.log(d["mass"])
    sv = d["sigma"]
    rf_fixed = xv[0:3, -1]
    problem, x, u, z, s = build(4, rf_fixed)
    x.value, u.value, z.value, s.value = xv, uv, zv, sv
    worst = 0.0
    worst_c = None
    for c in problem.constraints:
        v = float(np.max(c.violation()))
        if v > worst:
            worst, worst_c = v, c
    print(f"{csv_path}: fuel {m_wet - np.exp(zv[-1]):.2f} kg, "
          f"worst CVXPY constraint violation {worst:.3e}")
    if worst_c is not None and worst > 1e-5:
        print(f"  violated: {worst_c}")
    print("  FEASIBLE under the CVXPY constraint set" if worst < 1e-5
          else "  INFEASIBLE under the CVXPY constraint set")


if FEASCHECK:
    feascheck(FEASCHECK)
    sys.exit(0)


def unscale(xv, uv, sv):
    # back to physical units for CSV/compare regardless of --scaled
    xs = xv.copy()
    xs[0:3, :] *= SCALE_L
    xs[3:6, :] *= SCALE_V
    a = SCALE_L / SCALE_T ** 2
    return xs, uv * a, sv * a


def write_csv(path, xv, uv, zv, sv):
    xv, uv, sv = unscale(xv, uv, sv)
    m = np.exp(zv)
    with open(path, "w") as f:
        f.write("t,rx,ry,rz,vx,vy,vz,ux,uy,uz,sigma,mass,thrustN\n")
        for n in range(N):
            row = [n * dt * SCALE_T, *xv[0:3, n], *xv[3:6, n], *uv[:, n], sv[n], m[n], sv[n] * m[n]]
            f.write(",".join(f"{v:.9g}" for v in row) + "\n")


def compare(py_x, py_z, csharp_csv, label):
    py_x, _, _ = unscale(py_x, np.zeros((3, N)), np.zeros(N))
    if not os.path.exists(csharp_csv):
        print(f"  ({csharp_csv} not found, skipping diff)")
        return
    cs = np.genfromtxt(csharp_csv, delimiter=",", names=True)
    if cs.shape[0] != N:
        print(f"  ({csharp_csv} has {cs.shape[0]} nodes, expected {N} — rerun the C# console with matching tf/N)")
        return
    cs_r = np.vstack([cs["rx"], cs["ry"], cs["rz"]])
    cs_v = np.vstack([cs["vx"], cs["vy"], cs["vz"]])
    cs_m = cs["mass"]
    dr = np.max(np.linalg.norm(cs_r - py_x[0:3, :], axis=0))
    dv = np.max(np.linalg.norm(cs_v - py_x[3:6, :], axis=0))
    dm = np.max(np.abs(cs_m - np.exp(py_z)))
    print(f"  {label} max |dr| {dr:.4g} m | max |dv| {dv:.4g} m/s | max |dm| {dm:.4g} kg")


here = os.path.dirname(os.path.abspath(__file__))
os.chdir(here)
print(f"CVXPY {cp.__version__} / Clarabel | tf={tf}s N={N} dt={dt:.3f}s")

prob3, x3, u3, z3, s3 = solve(3)
land = x3[0:3, N - 1]
fuel3 = m_wet - np.exp(z3[N - 1])
print(f"P3 [{prob3.status}] landing ({land[0]:.2f}, {land[1]:.2f}, {land[2]:.2f}) "
      f"error {np.linalg.norm(land - rf):.4g} m, fuel {fuel3:.2f} kg")
write_csv("py_p3.csv", x3, u3, z3, s3)

prob4, x4, u4, z4, s4 = solve(4, rf_fixed=land)
fuel4 = m_wet - np.exp(z4[N - 1])
print(f"P4 [{prob4.status}] fuel {fuel4:.2f} kg")
write_csv("py_p4.csv", x4, u4, z4, s4)

print("\nC# (ECOS) vs Python (Clarabel):")
compare(x3, z3, os.path.join("..", "gfold_p3.csv"), "P3")
compare(x4, z4, os.path.join("..", "gfold_p4.csv"), "P4")
