#!/usr/bin/env python3
"""Join the Pure/ runtimes onto the root sweep counters, then fit  t = A*x + B.

The root sweeps carry the exact work counters (arcs_scanned, layer_iterations) but no
usable clock; Pure/ carries solve_seconds measured on a known machine but no counters.
This writes Merged/sweep_<SOLVER>.csv with both, one row per instance, then fits

    t  =  A * x  +  B          x = layer_iterations   (deque)
                               x = arcs_scanned       (Bellman)

A is seconds per unit of work -- the amortised per-iteration cost.
B is the fixed per-call cost that does not scale with the work term: allocation,
   extractSolution, timer overhead.  If solve_seconds really excludes process start-up,
   B should be small and positive.

Note x is the RAW counter, not eff_K or B-per-start: layer_iterations == eff_K * n and
arcs_scanned == B * n already, so the product is the counter itself.

  python3 merge_and_fit.py
"""
import csv
import math
import os
import statistics

ROOT = {
    "PTVRP_LAYERED": "layer_iterations",
    "PTVRP_LAYERED_SAFE": "layer_iterations",
    "PTVRP_LAYERED_CONT_FIX": "layer_iterations",
    "PTVRP_LAYERED_CONT": "layer_iterations",
    "PTVRP_CONT_FIX": "arcs_scanned",
    "PTVRP_CONT": None,          # no arc counter of its own
    "PTVRP": None,               # counter is trace-gated, absent from the sweep
    "PTVRP_LINEAR": None,
}
OUT = "Merged"


def load(path):
    with open(path, newline="") as fh:
        return {r["instance"].replace("\\", "/"): r for r in csv.DictReader(fh)}


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def ols(xs, ys):
    """Least squares y = A x + B, with R^2 and the standard error of A."""
    n = len(xs)
    mx, my = statistics.mean(xs), statistics.mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    A = sxy / sxx
    B = my - A * mx
    pred = [A * x + B for x in xs]
    ss_res = sum((y - p) ** 2 for y, p in zip(ys, pred))
    ss_tot = sum((y - my) ** 2 for y in ys)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    se_A = math.sqrt((ss_res / (n - 2)) / sxx) if n > 2 and sxx > 0 else float("nan")
    return A, B, r2, se_A, pred


def main():
    os.makedirs(OUT, exist_ok=True)
    base = load("sweep_PTVRP.csv")
    feas = {k for k, r in base.items() if num(r["cost"]) is not None}

    print("Merged/ written with pure_solve_seconds + pure_wall_seconds joined onto each root row.\n")
    print(f"{'solver':>24s}{'work counter':>19s}{'n':>7s}"
          f"{'A (ns/unit)':>14s}{'+/- se':>9s}{'B (us)':>10s}{'R2':>9s}")

    for solver, xcol in ROOT.items():
        R = load(f"sweep_{solver}.csv")
        P = load(f"Pure/sweep_{solver}.csv")

        # merge: every root row gains the two pure timing columns
        fields = list(next(iter(R.values())).keys()) + ["pure_solve_seconds", "pure_wall_seconds"]
        with open(os.path.join(OUT, f"sweep_{solver}.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            w.writeheader()
            for k, row in R.items():
                out = dict(row)
                p = P.get(k)
                out["pure_solve_seconds"] = p["solve_seconds"] if p else ""
                out["pure_wall_seconds"] = p["wall_seconds"] if p else ""
                w.writerow(out)

        if xcol is None:
            print(f"{solver:>24s}{'(none recorded)':>19s}{'-':>7s}{'-':>14s}{'-':>9s}{'-':>10s}{'-':>9s}")
            continue

        pts = []
        for k in feas:
            x = num(R[k].get(xcol))
            t = num(P[k]["solve_seconds"]) if k in P else None
            if x is not None and t is not None and x > 0 and t > 0:
                pts.append((x, t))
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        A, B, r2, se, _ = ols(xs, ys)
        print(f"{solver:>24s}{xcol:>19s}{len(pts):7d}"
              f"{A * 1e9:14.3f}{se * 1e9:9.3f}{B * 1e6:10.2f}{r2:9.5f}")

    print("\nA is seconds per unit of work; B is the fixed per-call cost.")


if __name__ == "__main__":
    main()
