# Renders the C# SCvx result using the SAME plots 6dof.py produces, with the
# Python reference overlaid so the two can be compared by eye.
#
# The two runs converge to DIFFERENT local optima and that is expected, not a
# bug: SCvx is a local method on a nonconvex problem, and the reference run hits
# six convex-solver failures (see loop_ref.csv) which shrink its trust region and
# arrest sigma near 17 s. The C# run has none, so sigma grows to ~24 s. Both are
# feasible; the C# one has the lower merit. These plots are for judging whether
# the two are the same KIND of trajectory, not whether they are identical.
#
# Inputs:  loop_cs.csv  (written by Scvx.Console --loop)
#          loop_ref.csv (written by loop_ref.py)
# Outputs: compare_results.png, compare_iters.png, 6dof_cs_landing.gif

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

NX, NU = 14, 4
Tmax = 6.0e6
L_arm = 25.0
L_body = 71.0
delta_max_deg = 10.0
tilt_max_deg = 30.0
m0 = 250000.0

here = os.path.dirname(os.path.abspath(__file__))


def quat_to_R(q):
    qw, qx, qy, qz = q
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qw*qz),   2*(qx*qz+qw*qy)],
        [2*(qx*qy+qw*qz),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qw*qx)],
        [2*(qx*qz-qw*qy),   2*(qy*qz+qw*qx),   1-2*(qx*qx+qy*qy)]])


def load_cs(path):
    rows = [l for l in open(path) if l.strip() and not l.startswith("#")]
    X = np.array([float(v) for v in rows[0].split(",")]).reshape(-1, NX)
    U = np.array([float(v) for v in rows[1].split(",")]).reshape(-1, NU)
    meta = [float(v) for v in rows[2].split(",")]
    trace = np.array([[float(v) for v in r.split(",")] for r in rows[3:]])
    return X, U, meta[0], int(meta[1]), meta[2], meta[3], trace


def load_ref(path):
    rows = [l for l in open(path) if l.strip() and not l.startswith("#")]
    X = np.array([float(v) for v in rows[2].split(",")]).reshape(-1, NX)
    U = np.array([float(v) for v in rows[3].split(",")]).reshape(-1, NU)
    meta = [float(v) for v in rows[4].split(",")]
    trace = np.array([[float(v) for v in r.split(",")] for r in rows[5:]])
    return X, U, meta[0], int(meta[1]), meta[2], meta[3], trace


def diagnostics(Xc, Uc):
    n = Xc.shape[0]
    q = Xc[:, 6:10]
    tdx, tdy, T = Uc[:, 0], Uc[:, 1], Uc[:, 2]
    thrust_in = np.zeros((n, 3)); nose_in = np.zeros((n, 3))
    for k in range(n):
        R = quat_to_R(q[k])
        thrust_in[k] = R @ np.array([tdx[k], tdy[k], T[k]])
        nose_in[k] = R @ np.array([0.0, 0.0, 1.0])
    throttle = T / Tmax * 100.0
    gimbal = np.degrees(np.arctan2(np.sqrt(tdx**2 + tdy**2), T))
    tilt = np.degrees(np.arccos(np.clip(nose_in[:, 2], -1, 1)))
    return thrust_in, nose_in, throttle, gimbal, tilt


Xc, Uc, sig_c, it_c, cost_c, def_c, tr_c = load_cs(os.path.join(here, "loop_cs.csv"))
Xr, Ur, sig_r, it_r, cost_r, def_r, tr_r = load_ref(os.path.join(here, "loop_ref.csv"))
N = Xc.shape[0]

Tv_c, nose_c, thr_c, gim_c, tlt_c = diagnostics(Xc, Uc)
Tv_r, nose_r, thr_r, gim_r, tlt_r = diagnostics(Xr, Ur)
t_c = sig_c * np.arange(N) / (N - 1)
t_r = sig_r * np.arange(N) / (N - 1)

for tag, X, U, sig, it, cost, dfc, thr, gim, tlt in [
        ("C#  ", Xc, Uc, sig_c, it_c, cost_c, def_c, thr_c, gim_c, tlt_c),
        ("py  ", Xr, Ur, sig_r, it_r, cost_r, def_r, thr_r, gim_r, tlt_r)]:
    print(f"{tag} burn {sig:5.1f}s  prop {m0 - X[-1,13]:6.0f} kg  iters {it:3d}  "
          f"merit {cost:.4e}  defect {dfc:.2e}  "
          f"peak tilt {tlt.max():4.1f}d  peak gimbal {gim.max():4.1f}d  "
          f"throttle {thr.min():.0f}-{thr.max():.0f}%")

# ---------- same 2x3 panel as 6dof.py, both runs overlaid ----------
STYLE = {"C#": dict(color="tab:blue", marker="o", ms=3, lw=1.8),
         "py": dict(color="tab:red", marker="s", ms=2.5, lw=1.2, alpha=0.65, ls="--")}

fig, axs = plt.subplots(2, 3, figsize=(16, 9))

for lab, X, t, thr, gim, tlt in [("C#", Xc, t_c, thr_c, gim_c, tlt_c),
                                 ("py", Xr, t_r, thr_r, gim_r, tlt_r)]:
    st = STYLE[lab]
    r, v, w, m = X[:, :3], X[:, 3:6], X[:, 10:13], X[:, 13]

    axs[0, 0].plot(r[:, 0], r[:, 2], label=lab, **st)
    for i, ax in [(0, axs[0, 1]), (1, axs[0, 1]), (2, axs[0, 1])]:
        pass
    for i, cl in enumerate("xyz"):
        axs[0, 1].plot(t, r[:, i], label=f"{lab} r_{cl}",
                       color=st["color"], ls=st.get("ls", "-"),
                       alpha=0.35 + 0.3 * i, lw=st["lw"])
        axs[0, 2].plot(t, v[:, i], label=f"{lab} v_{cl}",
                       color=st["color"], ls=st.get("ls", "-"),
                       alpha=0.35 + 0.3 * i, lw=st["lw"])
    axs[1, 0].plot(t, thr, label=f"{lab} throttle %", color=st["color"],
                   ls=st.get("ls", "-"), lw=st["lw"])
    axs[1, 0].plot(t, gim, label=f"{lab} gimbal deg", color=st["color"],
                   ls=":", lw=st["lw"])
    axs[1, 1].plot(t, tlt, label=f"{lab} tilt deg", color=st["color"],
                   ls=st.get("ls", "-"), lw=st["lw"])
    axs[1, 1].plot(t, np.degrees(np.linalg.norm(w, axis=1)), label=f"{lab} |w| deg/s",
                   color=st["color"], ls=":", lw=st["lw"])
    axs[1, 2].plot(t, m, label=f"{lab} mass", **st)

axs[0, 0].scatter([0], [0], c="k", marker="*", s=140, label="pad", zorder=5)
axs[0, 0].set_xlabel("downrange x [m]"); axs[0, 0].set_ylabel("altitude z [m]")
axs[0, 0].set_title("flight path (x-z)"); axs[0, 0].axis("equal")
axs[0, 1].set_title("position [m]")
axs[0, 2].set_title("velocity [m/s]")
axs[1, 0].axhline(40, ls="--", color="0.5", alpha=0.6)
axs[1, 0].axhline(delta_max_deg, ls="--", color="0.5", alpha=0.6)
axs[1, 0].set_title("throttle [%] (solid) & gimbal [deg] (dotted)\ndashed grey = 40% floor, 10 deg limit")
axs[1, 1].axhline(tilt_max_deg, ls="--", color="0.5", alpha=0.6)
axs[1, 1].set_title("tilt [deg] (solid) & |body rate| [deg/s] (dotted)\ndashed grey = 30 deg limit")
axs[1, 2].set_title("mass [kg]")
for ax in axs.ravel():
    ax.grid(True, alpha=0.3); ax.legend(loc="best", fontsize=7)
for ax in axs[:, 1:].ravel():
    ax.set_xlabel("time [s]")

fig.suptitle(
    f"6DOF min-fuel landing — C# (blue, SCS) vs Python reference (red dashed, Clarabel)\n"
    f"C#: burn {sig_c:.1f}s, prop {m0-Xc[-1,13]:.0f} kg, {it_c} iters, merit {cost_c:.3e}   |   "
    f"py: burn {sig_r:.1f}s, prop {m0-Xr[-1,13]:.0f} kg, {it_r} iters, merit {cost_r:.3e}",
    fontsize=11)
fig.tight_layout()
fig.savefig(os.path.join(here, "compare_results.png"), dpi=120)
print("saved -> compare_results.png")

# ---------- convergence traces ----------
fig2, ax2 = plt.subplots(1, 4, figsize=(18, 4))
for lab, tr in [("C#", tr_c), ("py", tr_r)]:
    st = STYLE[lab]
    it, rho, acc, trr, sg, step, dfc, cst = (tr[:, i] for i in range(8))
    ax2[0].semilogy(it, np.maximum(cst, 1e-12), label=lab, **st)
    ax2[1].plot(it, sg, label=lab, **st)
    ax2[2].semilogy(it, trr, label=lab, **st)
    ax2[3].semilogy(it, np.maximum(dfc, 1e-12), label=lab, **st)
    # mark rejected / failed iterations — this is the whole story of why the two
    # runs diverge: the reference's rejections are convex-solver failures.
    rej = acc < 0.5
    if rej.any():
        ax2[1].scatter(it[rej], sg[rej], marker="x", s=60, color=st["color"],
                       label=f"{lab} rejected/failed", zorder=5)
ax2[0].set_title("merit J (log)"); ax2[1].set_title("sigma (burn time) [s]")
ax2[2].set_title("trust region (log)"); ax2[3].set_title("true defect (log)")
for a in ax2:
    a.set_xlabel("SCvx iteration"); a.grid(True, alpha=0.3); a.legend(fontsize=8)
fig2.suptitle("SCvx convergence — note the reference's 6 solver failures (x) which "
              "shrink its trust region and arrest sigma near 17 s", fontsize=11)
fig2.tight_layout()
fig2.savefig(os.path.join(here, "compare_iters.png"), dpi=120)
print("saved -> compare_iters.png")

# ---------- 3D animation of the C# solution, same as 6dof.py's ----------
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

r = Xc[:, :3]; v = Xc[:, 3:6]
base = r - L_arm * nose_c
tip = r + (L_body - L_arm) * nose_c
Tmag = np.linalg.norm(Tv_c, axis=1)
ascale = (0.6 * L_body) / max(Tmag.max(), 1.0)
ends = np.vstack([base, tip, base - Tv_c * ascale])
lims = [(ends[:, i].min() - 30, ends[:, i].max() + 30) for i in range(3)]
span = max(hi - lo for lo, hi in lims)
ctr = [0.5 * (lo + hi) for lo, hi in lims]
lims = [(c - span/2, c + span/2) for c in ctr]

fig3 = plt.figure(figsize=(9, 8))
ax3 = fig3.add_subplot(111, projection="3d")


def draw(k):
    ax3.cla()
    ax3.plot(r[:, 0], r[:, 1], r[:, 2], color="0.8", lw=1)
    ax3.plot(r[:k+1, 0], r[:k+1, 1], r[:k+1, 2], color="tab:blue", lw=2)
    ax3.scatter([0], [0], [0], c="k", marker="*", s=150, label="pad")
    ax3.plot([base[k, 0], tip[k, 0]], [base[k, 1], tip[k, 1]], [base[k, 2], tip[k, 2]],
             color="tab:gray", lw=6, solid_capstyle="round")
    ax3.scatter(*tip[k], c="tab:gray", s=20)
    a = -Tv_c[k] * ascale
    ax3.quiver(base[k, 0], base[k, 1], base[k, 2], a[0], a[1], a[2],
               color="tab:orange", lw=3, arrow_length_ratio=0.15, label="thrust")
    ax3.set_xlim(*lims[0]); ax3.set_ylim(*lims[1]); ax3.set_zlim(*lims[2])
    ax3.set_xlabel("x [m]"); ax3.set_ylabel("y [m]"); ax3.set_zlabel("altitude z [m]")
    ax3.set_title(f"6DOF landing (C# / SCS)   t = {t_c[k]:.1f} s")
    ax3.legend(loc="upper left")
    ax3.text2D(0.02, 0.80,
               f"throttle = {thr_c[k]:4.0f} %\n"
               f"gimbal   = {gim_c[k]:4.1f} deg\n"
               f"tilt     = {tlt_c[k]:4.1f} deg\n"
               f"speed    = {np.linalg.norm(v[k]):4.1f} m/s",
               transform=ax3.transAxes, family="monospace", fontsize=9, va="top")
    ax3.view_init(elev=14, azim=-70)


anim = FuncAnimation(fig3, draw, frames=N, interval=160)
anim.save(os.path.join(here, "6dof_cs_landing.gif"), writer=PillowWriter(fps=6))
print("saved -> 6dof_cs_landing.gif")
