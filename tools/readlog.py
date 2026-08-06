"""Summarise a navbox 6-DOF telemetry run.

    python tools/readlog.py                 # newest run
    python tools/readlog.py 20260806-0920   # a specific run (prefix is enough)

Reads the three files SixDofLog writes and answers, in order, the questions that
actually decide what is wrong:

  1. Did the guidance keep re-solving, or was it flying a stale plan? A refused
     re-solve leaves the previous plan in place, so a run of refusals means the
     vehicle was open loop no matter how healthy everything else looks.
  2. Did the commanded thrust get delivered, and was it ever saturated?
  3. Was the landing reachable at all -- stopping distance against altitude.
  4. Does the PLAN curve, or does the vehicle diverge from a straight plan?
     Only the plan snapshots can separate those, and they have opposite causes.
"""
import csv
import glob
import math
import os
import sys

LOGDIR = os.path.expandvars(
    r"%USERPROFILE%\Documents\My Games\Kitten Space Agency\navbox-logs")


def load(path):
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def num(row, key, default=0.0):
    v = row.get(key, "")
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def bar(frac, width=28):
    frac = max(0.0, min(1.0, frac))
    n = int(round(frac * width))
    return "#" * n + "." * (width - n)


def main():
    if not os.path.isdir(LOGDIR):
        sys.exit(f"no log directory at {LOGDIR}")

    pattern = sys.argv[1] if len(sys.argv) > 1 else ""
    runs = sorted(glob.glob(os.path.join(LOGDIR, f"*{pattern}*-cycle.csv")))
    if not runs:
        sys.exit(f"no runs matching {pattern!r} in {LOGDIR}")
    cyc_path = runs[-1]
    stamp = os.path.basename(cyc_path)[: -len("-cycle.csv")]
    base = os.path.join(LOGDIR, stamp)

    rows = load(cyc_path)
    if not rows:
        sys.exit("cycle file is empty")

    print(f"RUN {stamp}   {len(rows)} cycles")
    ev_path = base + "-events.log"
    if os.path.exists(ev_path):
        for line in open(ev_path):
            line = line.rstrip()
            if line.startswith("# vehicle"):
                print("  " + line[2:])

    t0, t1 = num(rows[0], "t"), num(rows[-1], "t")
    print(f"  sim time {t0:.1f} -> {t1:.1f} s   "
          f"alt {num(rows[0],'alt'):.0f} -> {num(rows[-1],'alt'):.0f} m")

    # ---- 1. re-solve health -------------------------------------------------
    attempted = [r for r in rows if r.get("status") != "(no solve this cycle)"]
    solved = [r for r in attempted if r.get("solved") == "1"]
    refused = [r for r in attempted if r.get("solved") != "1"]
    print(f"\n1. RE-SOLVE HEALTH   {len(solved)}/{len(attempted)} accepted"
          f"   {bar(len(solved)/max(len(attempted),1))}")
    if refused:
        print(f"   REFUSED {len(refused)} times. The vehicle flew a STALE plan on those cycles.")
        seen = {}
        for r in refused:
            msg = (r.get("error") or "?").strip()
            seen[msg] = seen.get(msg, 0) + 1
        for msg, n in sorted(seen.items(), key=lambda kv: -kv[1])[:4]:
            print(f"     {n:4d} x  {msg[:100]}")
        # Longest unbroken run of refusals: one is noise, twenty is open loop.
        worst = cur = 0
        for r in attempted:
            cur = cur + 1 if r.get("solved") != "1" else 0
            worst = max(worst, cur)
        print(f"   longest unbroken refusal streak: {worst} cycles")
    age = [num(r, "planElapsed") for r in rows]
    print(f"   plan age: median {sorted(age)[len(age)//2]:.2f} s, max {max(age):.2f} s")
    dm = [num(r, "defectM") for r in rows if r.get("defectM")]
    if dm:
        lim = num(rows[-1], "defectLimitM", 1.0)
        print(f"   defect: median {sorted(dm)[len(dm)//2]:.2f} m, max {max(dm):.2f} m (limit {lim:.2f} m)")

    # ---- 2. thrust ----------------------------------------------------------
    dem = [num(r, "thrustDemandN") for r in rows]
    cap = [num(r, "capabilityN") for r in rows]
    sat = sum(1 for r in rows if r.get("saturated") == "1")
    ratio = [d / c for d, c in zip(dem, cap) if c > 1.0]
    print("\n2. THRUST")
    if ratio:
        print(f"   demand/capability: min {min(ratio):.2f}  median "
              f"{sorted(ratio)[len(ratio)//2]:.2f}  max {max(ratio):.2f}")
    print(f"   capability {min(cap)/1e6:.2f} - {max(cap)/1e6:.2f} MN"
          f"   saturated on {sat} cycles")
    thr = [num(r, "throttle") for r in rows]
    print(f"   throttle: min {min(thr):.2f}  median {sorted(thr)[len(thr)//2]:.2f}  max {max(thr):.2f}")

    # ---- 3. reachability ----------------------------------------------------
    print("\n3. REACHABILITY (stopping distance vs altitude remaining)")
    bad = [r for r in rows
           if num(r, "descentRate") > 1.0 and num(r, "stopDistM") > num(r, "altToGo") > 0]
    twr = [num(r, "twr") for r in rows if num(r, "twr") > 0]
    if twr:
        print(f"   TWR {min(twr):.2f} - {max(twr):.2f}")
    if bad:
        w = max(bad, key=lambda r: num(r, "stopDistM") - num(r, "altToGo"))
        print(f"   UNREACHABLE on {len(bad)}/{len(rows)} cycles; worst at t={num(w,'t'):.1f}s: "
              f"needs {num(w,'stopDistM'):.0f} m to stop, {num(w,'altToGo'):.0f} m left")
    else:
        print("   always able to stop in the altitude remaining")

    # ---- 4. does the PLAN curve, or does the VEHICLE diverge? ---------------
    plan_path = base + "-plan.csv"
    print("\n4. PLAN SHAPE vs FLOWN PATH")
    if not os.path.exists(plan_path):
        print("   (no plan snapshots)")
    else:
        snaps = {}
        for p in load(plan_path):
            snaps.setdefault(p["t"], []).append(p)
        print(f"   {len(snaps)} snapshots")
        print(f"   {'t':>7} {'nodes':>6} {'plan len':>9} {'direct':>8} {'ratio':>7}  shape")
        for t in sorted(snaps, key=float)[:14]:
            pts = sorted(snaps[t], key=lambda p: int(p["node"]))
            xs = [(num(p, "x"), num(p, "y"), num(p, "z")) for p in pts]
            length = sum(math.dist(a, b) for a, b in zip(xs, xs[1:]))
            direct = math.dist(xs[0], xs[-1])
            ratio = length / direct if direct > 1e-6 else 0.0
            shape = ("straight" if ratio < 1.10 else
                     "curved" if ratio < 1.6 else "LOOPING")
            print(f"   {float(t):7.1f} {len(pts):6d} {length:8.0f} m {direct:7.0f} m"
                  f" {ratio:7.2f}  {shape}")
        worst = max(
            (sum(math.dist(a, b) for a, b in zip(v, v[1:])) /
             max(math.dist(v[0], v[-1]), 1e-6))
            for v in ([(num(p, "x"), num(p, "y"), num(p, "z"))
                       for p in sorted(s, key=lambda q: int(q["node"]))]
                      for s in snaps.values()))
        print(f"   worst plan path/direct ratio: {worst:.2f}")
        print("   >1.6 means the PLAN itself loops (a solver/constraint problem).")
        print("   Near 1.0 while the vehicle wanders means TRACKING (stale plans, thrust, torque).")

    # flown path, for the same comparison
    pts = [(num(r, "rx"), num(r, "ry"), num(r, "rz")) for r in rows]
    flown = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    direct = math.dist(pts[0], pts[-1])
    print(f"   FLOWN: {flown:.0f} m over a {direct:.0f} m displacement"
          f"  ratio {flown/max(direct,1e-6):.2f}")

    # ---- events -------------------------------------------------------------
    if os.path.exists(ev_path):
        keep = [l.rstrip() for l in open(ev_path)
                if l.strip() and not l.startswith("#") and "RE-SOLVE REFUSED" not in l]
        if keep:
            print("\nEVENTS")
            for line in keep[:25]:
                print("  " + line)


if __name__ == "__main__":
    main()
