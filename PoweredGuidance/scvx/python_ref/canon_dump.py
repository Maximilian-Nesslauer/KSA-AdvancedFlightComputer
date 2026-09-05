# Dumps CVXPY's OWN canonicalisation of the reference subproblem, in the same
# cone-program format Scvx.Core writes.
#
# INCONCLUSIVE AS A HEAD-TO-HEAD TEST — but it found something better.
#
# The intent was: ECOS fails on the C#-assembled matrices while Clarabel and SCS
# solve them, so hand CVXPY's own canonicalisation to the C# ECOS binding and see
# whether ECOS copes with that. It cannot be done this way, because CVXPY does
# NOT canonicalise this problem into a pure SOCP at all: it emits a QUADRATIC
# objective matrix P (612 nnz, |P| in [4e-1, 2e5]) alongside only 30 SOCs — the
# gimbal cones. The three sum_squares penalties live in P, not in cones.
#
# ECOS has no P: it minimises c'x only. So targeting ECOS forces the epigraph
# reformulation the C# side does, which is what introduces the three large extra
# cones (dims 408, 118, 92) and the 618 extra rows. In other words the natural
# form of this problem is a QP-with-cones, and ECOS is the one solver in play
# that cannot express it.
#
# Dumping without P would silently drop every quadratic penalty and solve a
# different problem, so this script is kept for the structural comparison it
# prints, not as a solver benchmark. Reproducing it against ECOS needs the python
# `ecos` package, which has no wheel for 3.14 here and no MSVC to build one.
# CVXPY's SCS form is
#     minimize c'x  s.t.  Ax + s = b,  s in {0}^z x R+^l x SOC(q...)
# which maps onto ECOS form by splitting the zero-cone rows off as equalities:
#     A_eq = A[:z], b_eq = b[:z];  G = A[z:], h = b[z:], with cones l and q.
#
# Usage:  python canon_dump.py     (writes canon_dump.txt)

import os
import numpy as np
import cvxpy as cp
import scipy.sparse as sp

import sub_ref  # builds and solves the reference subproblem on import

prob = sub_ref.prob
data, chain, inv = prob.get_problem_data(cp.SCS)

A = sp.csc_matrix(data["A"])
b = np.asarray(data["b"]).ravel()
c = np.asarray(data["c"]).ravel()
dims = data["dims"]

z = int(dims.zero)
l = int(dims.nonneg)
q = [int(v) for v in dims.soc]
n = A.shape[1]
m_total = A.shape[0]

assert z + l + sum(q) == m_total, f"{z}+{l}+{sum(q)} != {m_total}"
print(f"CVXPY canonical: n={n} zero={z} nonneg={l} socs={len(q)} total rows={m_total}")
print(f"  |c| range [{np.abs(c[c != 0]).min():.2e}, {np.abs(c).max():.2e}]")
print(f"  |A| range [{np.abs(A.data).min():.2e}, {np.abs(A.data).max():.2e}]")

Aeq = A[:z].tocoo()
beq = b[:z]
G = A[z:].tocoo()
h = b[z:]

here = os.path.dirname(os.path.abspath(__file__))
out = os.path.join(here, "canon_dump.txt")
with open(out, "w") as fh:
    fh.write(f"n {n}\n")
    fh.write(f"p {z}\n")
    fh.write(f"m {m_total - z}\n")
    fh.write(f"l {l}\n")
    fh.write("q " + " ".join(str(v) for v in q) + "\n")
    fh.write(f"c {len(c)}\n" + ",".join(repr(float(v)) for v in c) + "\n")
    fh.write(f"b {len(beq)}\n" + ",".join(repr(float(v)) for v in beq) + "\n")
    fh.write(f"h {len(h)}\n" + ",".join(repr(float(v)) for v in h) + "\n")
    fh.write(f"A {Aeq.nnz}\n")
    for r, cc, v in zip(Aeq.row, Aeq.col, Aeq.data):
        fh.write(f"{r},{cc},{float(v)!r}\n")
    fh.write(f"G {G.nnz}\n")
    for r, cc, v in zip(G.row, G.col, G.data):
        fh.write(f"{r},{cc},{float(v)!r}\n")

print(f"wrote {out}")
print(f"reference objective (CVXPY) = {prob.value!r}")
