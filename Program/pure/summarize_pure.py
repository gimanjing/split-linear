#!/usr/bin/env python3
"""Summarise a Program/pure sweep. Run from the repository root after run_pure.py.

Writes two files into the sweep folder (default Pure/) :

  summary.csv            cat, solver, match, feasible, infeasible, match ratio, instance 1/2/3 (s), total (s)
                         match is agreement with PTVRP_CONT (the oracle), counting a shared cost and a
                         shared NO SOLUTION alike; the oracle's own match is '-'. Times are solve_seconds.

  check_vs_original.csv  for each solver, how many costs equal the original sweep_<SOLVER>.csv at the
                         repository root -- the recording-free build must reproduce every answer.
"""
import argparse, csv, os, sys

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


def folder(inst):
    return int(inst.split("Instances ")[1][0])


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

    summary = [["cat", "solver", "match", "feasible", "infeasible", "match ratio",
                "instance 1 (s)", "instance 2 (s)", "instance 3 (s)", "total (s)"]]
    check = [["solver", "instances", "same as original", "different", "missing",
              "original total wall (s)", "pure total solve (s)", "pure total wall (s)"]]

    for cat, s in ROWS:
        d = data[s]
        t = {1: 0.0, 2: 0.0, 3: 0.0}
        feas = infeas = 0
        for inst, orow in oracle.items():
            r = d[inst]
            t[folder(inst)] += num(r["solve_seconds"]) or 0.0
            if same(orow["cost"], r["cost"]):
                if num(r["cost"]) is None:
                    infeas += 1
                else:
                    feas += 1
        if s == ORACLE:
            own_feas = sum(1 for r in d.values() if num(r["cost"]) is not None)
            row_match, ratio, f, i = "-", "-", own_feas, len(d) - own_feas
        else:
            row_match, ratio, f, i = feas + infeas, f"{100 * (feas + infeas) / len(oracle):.2f}%", feas, infeas
        summary.append([cat, s, row_match, f, i, ratio,
                        f"{t[1]:.3f}", f"{t[2]:.3f}", f"{t[3]:.3f}", f"{sum(t.values()):.3f}"])

        orig_path = os.path.join(args.original_dir, f"sweep_{s}.csv")
        if os.path.exists(orig_path):
            orig = load(orig_path)
            eq = sum(1 for inst in d if inst in orig and same(orig[inst]["cost"], d[inst]["cost"]))
            missing = sum(1 for inst in d if inst not in orig)
            orig_wall = sum(num(r["seconds"]) or 0.0 for r in orig.values())
            check.append([s, len(d), eq, len(d) - eq - missing, missing, f"{orig_wall:.1f}",
                          f"{sum(num(r['solve_seconds']) or 0.0 for r in d.values()):.3f}",
                          f"{sum(num(r['wall_seconds']) or 0.0 for r in d.values()):.1f}"])

    for name, rows in (("summary.csv", summary), ("check_vs_original.csv", check)):
        with open(os.path.join(args.sweep_dir, name), "w", newline="") as fh:
            csv.writer(fh).writerows(rows)
        print(f"== {name}")
        for r in rows:
            print(",".join(str(x) for x in r))


if __name__ == "__main__":
    main()
