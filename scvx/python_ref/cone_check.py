# Solves the cone program the C# side assembled, using a different solver.
#
# The point is to separate two very different failures. The C# assembly has
# already been shown to be correct in the ways a formulation check can show:
# the reference optimum is feasible in it, and c'x at that point reproduces the
# reference objective to 1e-15. If Clarabel then solves these exact matrices and
# recovers the same optimum, the assembly is right and ECOS is simply failing on
# a problem it does not like. If Clarabel fails too, something structural is
# wrong that a feasibility audit cannot see.
#
# Usage:  python cone_check.py [cone_dump.txt]

import os
import sys
import numpy as np
import scipy.sparse as sp
import cvxpy as cp

here = os.path.dirname(os.path.abspath(__file__))
path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "cone_dump.txt")

with open(path) as fh:
    lines = [l.rstrip("\n") for l in fh]

i = 0
def tok():
    global i
    s = lines[i]; i += 1
    return s

n = int(tok().split()[1])
p = int(tok().split()[1])
m = int(tok().split()[1])
l = int(tok().split()[1])
q = [int(z) for z in tok().split()[1:]]


def vector(tag):
    global i
    head = tok().split()
    assert head[0] == tag, f"expected {tag}, got {head[0]}"
    cnt = int(head[1])
    vals = np.array([float(z) for z in tok().split(",")])
    assert len(vals) == cnt
    return vals


def triplets(tag, rows, cols):
    global i
    head = tok().split()
    assert head[0] == tag, f"expected {tag}, got {head[0]}"
    nnz = int(head[1])
    r = np.empty(nnz, dtype=int); c = np.empty(nnz, dtype=int); v = np.empty(nnz)
    for k in range(nnz):
        a, b, d = lines[i].split(","); i += 1
        r[k] = int(a); c[k] = int(b); v[k] = float(d)
    return sp.csc_matrix((v, (r, c)), shape=(rows, cols))


c = vector("c")
b = vector("b")
h = vector("h")
A = triplets("A", p, n)
G = triplets("G", m, n)

print(f"n={n} p={p} m={m} l={l} cones={len(q)} "
      f"nnz(A)={A.nnz} nnz(G)={G.nnz}")
print(f"|c| range [{np.abs(c[c != 0]).min():.2e}, {np.abs(c).max():.2e}]")
print(f"|A| range [{np.abs(A.data).min():.2e}, {np.abs(A.data).max():.2e}]")
print(f"|G| range [{np.abs(G.data).min():.2e}, {np.abs(G.data).max():.2e}]")

x = cp.Variable(n)
cons = [A @ x == b]
if l > 0:
    cons.append(G[:l] @ x <= h[:l])
off = l
for d in q:
    Gi = G[off:off + d]
    hi = h[off:off + d]
    # s = h - Gx must lie in the second-order cone: s0 >= ||s[1:]||
    cons.append(cp.SOC(hi[0] - Gi[0] @ x, hi[1:] - Gi[1:] @ x))
    off += d
assert off == m

prob = cp.Problem(cp.Minimize(c @ x), cons)
for cand in ("CLARABEL", "SCS", "ECOS"):
    if cand not in cp.installed_solvers():
        continue
    try:
        prob.solve(solver=getattr(cp, cand), verbose=False)
        print(f"{cand:9s} status={prob.status:22s} objective={prob.value!r}")
    except Exception as e:
        print(f"{cand:9s} FAILED: {type(e).__name__}: {e}")
