#!/usr/bin/env python3
"""Rewrite instance demands along the giant-tour order, keeping geometry and total load fixed.

Only the demand column changes. Coordinates, the tour order, the depot legs, the capacity and the
horizon are all left alone, and the total demand is rescaled back to the original, so ceil(q_tot/Q)
-- the naive layer count -- is unchanged. What changes is only *where along the chromosome* the load
sits, which is what the eff_K model predicts should matter.

Profiles are Beta shapes over normalised tour position x = (i-0.5)/n, w_i propto x^(a-1) (1-x)^(b-1):

  flat      a=1   b=1     the reference (matches the original instance up to rounding)
  left      a=1   b=3     load concentrated at the start of the tour
  right     a=3   b=1     load concentrated at the end
  centre    a=3   b=3     load concentrated in the middle
  ends      a=0.5 b=0.5   load at both ends, light middle

Usage:
  python3 skew_instances.py --src "Instances/Instances 3" --out Instances/skew --limit 30
  python3 batch_run.py --dir Instances/skew/right --solver PTVRP_LAYERED --out right.csv
"""
import argparse, math, os, random, re, sys

PROFILES = {"flat": (1.0, 1.0), "left": (1.0, 3.0), "right": (3.0, 1.0),
            "centre": (3.0, 3.0), "ends": (0.5, 0.5)}


def parse(path):
    tok = open(path).read().split()
    hdr = {}
    i = 0
    while i < len(tok) and tok[i] != "GIANT_TOUR_SECTION":
        if tok[i] in ("DIMENSION", "CAPACITY", "MAX_ROUTE", "HORIZON", "SPEED"):
            j = i + 1
            if j < len(tok) and tok[j] == ":":
                j += 1
            hdr[tok[i]] = tok[j]
            i = j + 1
            continue
        i += 1
    n = int(hdr["DIMENSION"])
    i += 1
    dem, dret, dnext = [], [], []
    for k in range(1, n + 1):
        i += 1                       # index column
        dem.append(float(tok[i])); i += 1
        dret.append(tok[i]); i += 1
        if k < n:
            dnext.append(tok[i]); i += 1
    return hdr, n, dem, dret, dnext


def reweight(dem, a, b, floor=1.0):
    """Beta-shaped weights over tour position, rescaled to the original total demand."""
    n = len(dem)
    total = sum(dem)
    w = []
    for i in range(n):
        x = (i + 0.5) / n
        x = min(max(x, 1e-6), 1 - 1e-6)
        w.append(x ** (a - 1.0) * (1.0 - x) ** (b - 1.0))
    s = sum(w)
    out = [max(floor, v / s * total) for v in w]
    # rescaling after the floor, so the total is preserved rather than merely approximated
    for _ in range(50):
        cur = sum(out)
        if abs(cur - total) <= 1e-6 * max(1.0, total):
            break
        scale = total / cur
        out = [max(floor, v * scale) for v in out]
    return [round(v, 3) for v in out]


def write(path, hdr, n, dem, dret, dnext, name):
    lines = [f"NAME : {name}", "TYPE : GIANT_TOUR", f"DIMENSION : {n}",
             f"CAPACITY : {hdr['CAPACITY']}"]
    if "MAX_ROUTE" in hdr:
        lines.append(f"MAX_ROUTE : {hdr['MAX_ROUTE']}")
    elif "HORIZON" in hdr:
        lines.append(f"HORIZON : {hdr['HORIZON']}")
    if "SPEED" in hdr:
        lines.append(f"SPEED : {hdr['SPEED']}")
    lines.append("GIANT_TOUR_SECTION")
    for k in range(n):
        if k < n - 1:
            lines.append(f"{k+1} {dem[k]} {dret[k]} {dnext[k]}")
        else:
            lines.append(f"{k+1} {dem[k]} {dret[k]}")
    lines.append("EOF")
    open(path, "w").write("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser(description="Skew instance demands along the giant-tour order.")
    ap.add_argument("--src", required=True, help="source instance folder")
    ap.add_argument("--out", required=True, help="output root; one subfolder per profile")
    ap.add_argument("--limit", type=int, help="use only this many source instances")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--profiles", nargs="+", default=list(PROFILES),
                    help=f"subset of {list(PROFILES)}")
    args = ap.parse_args()

    import glob
    files = sorted(glob.glob(os.path.join(args.src, "*.gt")))
    if args.limit:
        random.Random(args.seed).shuffle(files)
        files = sorted(files[:args.limit])
    if not files:
        sys.exit(f"no .gt files under {args.src}")

    for prof in args.profiles:
        os.makedirs(os.path.join(args.out, prof), exist_ok=True)
    written = 0
    for src in files:
        hdr, n, dem, dret, dnext = parse(src)
        base = os.path.basename(src)
        for prof in args.profiles:
            a, b = PROFILES[prof]
            newdem = reweight(dem, a, b)
            dst = os.path.join(args.out, prof, base)
            write(dst, hdr, n, newdem, dret, dnext, f"{base[:-3]}_{prof}")
            written += 1
    print(f"{len(files)} source instances x {len(args.profiles)} profiles -> {written} files under {args.out}")


if __name__ == "__main__":
    main()
