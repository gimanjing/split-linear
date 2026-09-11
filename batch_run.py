#!/usr/bin/env python3
"""Batch runner for the Split solvers. Writes one CSV row per (instance, solver).

Examples
--------
  # every instance in one folder, exact PT-VRP solver
  python3 batch_run.py --dir "Instances/Instances 1" --solver PTVRP --out results.csv

  # compare solvers on the same instances (one row each, plus a gap column)
  python3 batch_run.py --dir "Instances/Instances 3" --solver PTVRP PTVRP_LAYERED --out cmp.csv

  # sample 20 instances instead of all 1050, for a quick look
  python3 batch_run.py --dir "Instances/Instances 2" --solver PTVRP --limit 20 --out quick.csv

Open the CSV in Excel. Columns: instance, n, Q, horizon, solver, cost, templates,
max_m, seconds, plus the layer diagnostics when the layered solver is used, and
gap_vs_first when more than one solver is given.
"""
import argparse, csv, glob, os, random, re, subprocess, sys, time

FIELDS = ["instance", "n", "Q", "horizon", "solver", "cost", "templates", "max_m",
          "seconds", "K_full_tour", "K_allocated", "max_layer_used", "layer_iterations",
          "eff_K", "gap_vs_first"]


def read_header(path):
    """n, Q and the horizon straight from the instance file."""
    try:
        t = open(path).read().split()
        n = int(t[t.index("DIMENSION") + 2])
        Q = float(t[t.index("CAPACITY") + 2])
        horizon = float(t[t.index("MAX_ROUTE") + 2]) if "MAX_ROUTE" in t else \
                  (float(t[t.index("HORIZON") + 2]) if "HORIZON" in t else "")
        return n, Q, horizon
    except Exception:
        return "", "", ""


def run_one(binary, path, solver, timeout):
    t0 = time.time()
    try:
        out = subprocess.run([binary, path, "-solver", solver],
                             capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return {"cost": "TIMEOUT", "seconds": timeout}
    elapsed = time.time() - t0

    def grab(pattern, cast=float):
        m = re.search(pattern, out)
        return cast(m.group(1)) if m else ""

    trips = [int(x) for x in re.findall(r"m\(sigma\)=(\d+)", out)]
    row = {
        "cost": grab(r"SOLUTION COST : ([\d.eE+-]+)"),
        "templates": grab(r"NB (?:TEMPLATES|ROUTES) : (\d+)", int),
        "max_m": max(trips) if trips else "",
        "seconds": round(elapsed, 4),
        "K_full_tour": grab(r"K FULL TOUR\s+: (\d+)", int),
        "K_allocated": grab(r"LAYERS ALLOCATED \(K\) : (\d+)", int),
        "max_layer_used": grab(r"MAX LAYER USED\s+: (\d+)", int),
        "layer_iterations": grab(r"LAYER ITERATIONS\s+: (\d+)", int),
        "eff_K": grab(r"effective K = ([\d.]+)"),
    }
    if row["cost"] == "":
        # the solver refused the instance (infeasible, or unreadable)
        row["cost"] = "NO SOLUTION"
    return row


def main():
    ap = argparse.ArgumentParser(description="Run Split solvers over many instances into a CSV.")
    ap.add_argument("--dir", action="append", required=True,
                    help="instance folder; repeat the flag for several folders")
    ap.add_argument("--solver", nargs="+", required=True,
                    help="one or more solver names, e.g. PTVRP PTVRP_LAYERED")
    ap.add_argument("--out", default="results.csv", help="CSV to write (default results.csv)")
    ap.add_argument("--limit", type=int, help="run only this many instances per folder")
    ap.add_argument("--seed", type=int, default=0, help="sampling seed used with --limit")
    ap.add_argument("--timeout", type=float, default=300, help="per-run timeout in seconds")
    ap.add_argument("--binary", default=os.path.join("Program", "split"))
    args = ap.parse_args()

    if not os.path.exists(args.binary):
        sys.exit(f"binary not found at {args.binary} -- run 'cd Program && make' first")

    files = []
    for d in args.dir:
        found = sorted(glob.glob(os.path.join(d, "*.gt")))
        if not found:
            print(f"warning: no .gt files in {d}", file=sys.stderr)
        if args.limit:
            random.Random(args.seed).shuffle(found)
            found = found[:args.limit]
            found.sort()
        files += found

    if not files:
        sys.exit("no instances found")

    print(f"{len(files)} instances x {len(args.solver)} solver(s) -> {args.out}")
    with open(args.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        for idx, path in enumerate(files, 1):
            n, Q, horizon = read_header(path)
            baseline = None
            for solver in args.solver:
                row = run_one(args.binary, path, solver, args.timeout)
                if baseline is None:
                    baseline = row["cost"]
                    row["gap_vs_first"] = ""
                elif isinstance(row["cost"], float) and isinstance(baseline, float) and baseline:
                    row["gap_vs_first"] = round((row["cost"] - baseline) / baseline * 100, 4)
                else:
                    row["gap_vs_first"] = ""
                row.update({"instance": os.path.relpath(path), "n": n, "Q": Q,
                            "horizon": horizon, "solver": solver})
                w.writerow({k: row.get(k, "") for k in FIELDS})
            if idx % 25 == 0 or idx == len(files):
                print(f"  {idx}/{len(files)}")
                fh.flush()
    print(f"done -> {args.out}")


if __name__ == "__main__":
    main()
