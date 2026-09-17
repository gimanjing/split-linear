#!/usr/bin/env python3
"""Summarise a Program/pure sweep. Run from the repository root after run_pure.py.

Writes three files into the sweep folder (default Pure/) :

  summary.csv        cat, solver, match, feasible, infeasible, match ratio, solve seconds, and the
                     spread of the repeated measurements. Match is agreement with PTVRP_CONT (the
                     oracle), counting a shared cost and a shared NO SOLUTION alike; the oracle's own
                     match is '-'.

  by_capacity.csv    the same timings broken down by vehicle capacity Q, which is the axis the
                     instances actually vary along. Vidal's originals put a ten-rung capacity ladder
                     in the _01.._10 suffix and the three folders scale that ladder by 1, 0.20 and
                     0.10, so a folder is a scale factor, not a capacity. Grouping by folder -- which
                     this script used to do -- mixes every rung together and says nothing.

  check_vs_original.csv   for each solver, how many costs equal the root sweep_<SOLVER>.csv. The
                     recording-free build must reproduce every answer the instrumented one gives.
                     Skipped for any solver with no root sweep present.

Quote solve_seconds, never wall_seconds : wall_seconds includes process start-up, whose floor is a
couple of milliseconds, and most instances solve in less than that.
"""
import argparse
import csv
import os
import statistics
import sys

ROWS = [("bellman", "PTVRP"), ("bellman", "PTVRP_CONT"), ("bellman", "PTVRP_CONT_FIX"),
        ("deque", "PTVRP_LINEAR"), ("deque", "PTVRP_LAYERED"), ("deque", "PTVRP_LAYERED_SAFE"),
        ("deque", "PTVRP_LAYERED_CONT"), ("deque", "PTVRP_LAYERED_CONT_FIX")]
ORACLE = "PTVRP_CONT"


def load(path):
    with open(path, newline="") as fh:
        return {r["instance"].replace("\\", "/"): r for r in csv.DictReader(fh)}


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def same(a, b):
    """Same verdict : equal costs, or both without a solution."""
    x, y = num(a), num(b)
    if x is None or y is None:
        return x is None and y is None
    return abs(x - y) < 1e-6


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep-dir", default="Pure")
    ap.add_argument("--original-dir", default=".")
    args = ap.parse_args()

    data = {}
    for _, s in ROWS:
        p = os.path.join(args.sweep_dir, f"sweep_{s}.csv")
        if not os.path.exists(p):
            sys.exit(f"missing {p}")
        data[s] = load(p)
    oracle = data[ORACLE]
    caps = sorted({num(r["Q"]) for r in oracle.values() if num(r["Q"]) is not None})

    summary = [["cat", "solver", "match", "feasible", "infeasible", "match ratio",
                "total solve (s)", "median solve (ms)", "median repeat spread", "repeats"]]
    by_cap = [["cat", "solver"] + [f"Q={c:g} median ms" for c in caps] + ["total solve (s)"]]
    check = [["solver", "instances", "same as original", "different", "missing",
              "pure total solve (s)"]]

    for cat, s in ROWS:
        d = data[s]
        secs, spreads, reps = [], [], []
        feas = infeas = 0
        percap = {c: [] for c in caps}
        for inst, orow in oracle.items():
            r = d[inst]
            t = num(r.get("solve_seconds"))
            if t is not None:
                secs.append(t)
                q = num(r["Q"])
                if q in percap:
                    percap[q].append(t)
            sp = num(r.get("solve_seconds_spread"))
            if sp is not None:
                spreads.append(sp)
            rp = num(r.get("repeats"))
            if rp is not None:
                reps.append(rp)
            if same(orow["cost"], r["cost"]):
                if num(r["cost"]) is None:
                    infeas += 1
                else:
                    feas += 1

        if s == ORACLE:
            own = sum(1 for r in d.values() if num(r["cost"]) is not None)
            match, ratio, f, i = "-", "-", own, len(d) - own
        else:
            match = feas + infeas
            ratio = f"{100 * match / len(oracle):.2f}%"
            f, i = feas, infeas

        summary.append([
            cat, s, match, f, i, ratio,
            f"{sum(secs):.3f}",
            f"{1000 * statistics.median(secs):.4f}" if secs else "",
            f"{statistics.median(spreads):.4f}" if spreads else "",
            f"{int(statistics.median(reps))}" if reps else "",
        ])
        by_cap.append([cat, s] + [
            f"{1000 * statistics.median(percap[c]):.4f}" if percap[c] else "" for c in caps
        ] + [f"{sum(secs):.3f}"])

        orig_path = os.path.join(args.original_dir, f"sweep_{s}.csv")
        if os.path.exists(orig_path):
            orig = load(orig_path)
            eq = sum(1 for inst in d if inst in orig and same(orig[inst]["cost"], d[inst]["cost"]))
            missing = sum(1 for inst in d if inst not in orig)
            check.append([s, len(d), eq, len(d) - eq - missing, missing, f"{sum(secs):.3f}"])

    for name, rows in (("summary.csv", summary), ("by_capacity.csv", by_cap),
                       ("check_vs_original.csv", check)):
        with open(os.path.join(args.sweep_dir, name), "w", newline="") as fh:
            csv.writer(fh).writerows(rows)
        print(f"== {name}")
        for r in rows:
            print(",".join(str(x) for x in r))
        print()


if __name__ == "__main__":
    main()
