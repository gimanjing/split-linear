#!/usr/bin/env python3
"""Sweep the recording-free PT-VRP solvers (Program/pure) into one CSV per solver.

Run from the repository root, after 'make -C Program/pure':

  python3 Program/pure/run_pure.py                                  # all 8 solvers, all 3,150 instances
  python3 Program/pure/run_pure.py --solver PTVRP_CONT_FIX --limit 20

Writes <out-dir>/sweep_<SOLVER>.csv, one row per instance. Columns : instance, n, Q, horizon, solver,
cost (or NO SOLUTION / TIMEOUT), templates, solve_seconds (the solver's own clock around solve() only),
wall_seconds (the whole process, including start-up and parsing).

Runs are strictly sequential so that no two solves compete for the machine.
"""
import argparse, csv, glob, os, random, re, subprocess, sys, time

ALL_SOLVERS = ["PTVRP", "PTVRP_CONT_FIX", "PTVRP_LINEAR", "PTVRP_LAYERED", "PTVRP_LAYERED_SAFE",
               "PTVRP_LAYERED_CONT_FIX", "PTVRP_CONT", "PTVRP_LAYERED_CONT"]   # slow ones last
DIRS = ["Instances/Instances 1", "Instances/Instances 2", "Instances/Instances 3"]
FIELDS = ["instance", "n", "Q", "horizon", "solver", "cost", "templates", "solve_seconds", "wall_seconds"]


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
    t0 = time.perf_counter()
    try:
        proc = subprocess.run([binary, path, "-solver", solver],
                              capture_output=True, text=True, timeout=timeout or None)
    except subprocess.TimeoutExpired:
        return {"cost": "TIMEOUT", "templates": "", "solve_seconds": "", "wall_seconds": timeout}
    wall = time.perf_counter() - t0
    out = proc.stdout
    if proc.returncode != 0:
        sys.exit(f"solver failed on {path} ({solver}): {proc.stderr.strip()}")

    def grab(pattern):
        m = re.search(pattern, out)
        return m.group(1) if m else ""

    cost = grab(r"SOLUTION COST : ([\d.eE+-]+)")
    return {
        "cost": float(cost) if cost else "NO SOLUTION",
        "templates": grab(r"NB TEMPLATES : (\d+)"),
        "solve_seconds": grab(r"SOLVE TIME : ([\d.eE+-]+)"),
        "wall_seconds": round(wall, 6),
    }


def main():
    ap = argparse.ArgumentParser(description="Sweep the Program/pure PT-VRP solvers.")
    ap.add_argument("--solver", nargs="+", default=ALL_SOLVERS)
    ap.add_argument("--dir", action="append", help="instance folder; repeat for several (default: Instances 1/2/3)")
    ap.add_argument("--out-dir", default="Pure")
    ap.add_argument("--binary", default=os.path.join("Program", "pure", "split"))
    ap.add_argument("--limit", type=int, help="run only this many instances per folder")
    ap.add_argument("--seed", type=int, default=0, help="sampling seed used with --limit")
    ap.add_argument("--timeout", type=float, default=0, help="per-run timeout in seconds, 0 for no limit")
    args = ap.parse_args()

    if not os.path.exists(args.binary):
        sys.exit(f"binary not found at {args.binary} -- run 'make -C Program/pure' first")

    files = []
    for d in (args.dir or DIRS):
        found = sorted(glob.glob(os.path.join(d, "*.gt")))
        if args.limit:
            random.Random(args.seed).shuffle(found)
            found = sorted(found[:args.limit])
        files += found
    if not files:
        sys.exit("no instances found")

    headers = {path: read_header(path) for path in files}
    os.makedirs(args.out_dir, exist_ok=True)

    for solver in args.solver:
        out = os.path.join(args.out_dir, f"sweep_{solver}.csv")
        print(f"{solver}: {len(files)} instances -> {out}", flush=True)
        t_start = time.time()
        with open(out, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=FIELDS)
            w.writeheader()
            for idx, path in enumerate(files, 1):
                n, Q, horizon = headers[path]
                row = run_one(args.binary, path, solver, args.timeout)
                row.update({"instance": os.path.relpath(path), "n": n, "Q": Q, "horizon": horizon, "solver": solver})
                w.writerow(row)
                fh.flush()
                if idx % 525 == 0 or idx == len(files):
                    print(f"  {idx}/{len(files)}  ({time.time() - t_start:.0f} s)", flush=True)
    print("done", flush=True)


if __name__ == "__main__":
    main()
