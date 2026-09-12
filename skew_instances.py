#!/usr/bin/env python3
"""Rewrite instance demands along the giant-tour order, keeping geometry and total load fixed.

Only the demand column changes. Coordinates, tour order, depot legs, capacity and horizon are left
alone and the total demand is rescaled back to the original, so ceil(q_tot/Q) -- the naive layer
count -- does not move. Only *where along the chromosome* the load sits changes.

Skew is a two-factor design rather than a set of named cases, so eff_K can be regressed on it:

  mu    demand centroid, sum(i*q_i)/(n*sum q_i) in [0,1].  0.5 balanced, <0.5 head-loaded,
        >0.5 tail-loaded. This is the axis the eff_K asymmetry lives on.
  s     concentration. Larger s piles the load more tightly around mu; s=2 with mu=0.5 is
        the flat reference.

Weights are Beta over normalised tour position, a = mu*s and b = (1-mu)*s, so the two factors move
independently. Every generated file is logged with its *measured* centroid, Gini and coefficient of
variation -- the nominal knob and the achieved statistic are not the same thing once the positivity
floor and the integer rounding bite, and the achieved one is what belongs on the x-axis.

Usage:
  # full matrix, 5 positions x 4 concentrations
  python3 skew_instances.py --src "Instances/Instances 1" --out Instances/skew --limit 40 \
      --mu 0.1 0.3 0.5 0.7 0.9 --conc 2 4 10 30

  # one cell
  python3 skew_instances.py --src "Instances/Instances 1" --out Instances/skew --mu 0.9 --conc 10

Each cell lands in <out>/mu<mu>_s<s>/ and <out>/manifest.csv records the achieved statistics.
"""
import argparse, csv, math, os, random, sys

def cell_name(mu, s):
    return f"mu{mu:g}_s{s:g}"


def stats(dem):
    """Measured centroid, Gini and CV of a demand vector laid out in tour order."""
    n = len(dem); tot = sum(dem)
    centroid = sum((i + 0.5) * q for i, q in enumerate(dem)) / (n * tot) if tot else 0.0
    srt = sorted(dem)
    gini = (2.0 * sum((i + 1) * q for i, q in enumerate(srt)) / (n * tot) - (n + 1.0) / n) if tot else 0.0
    mean = tot / n
    var = sum((q - mean) ** 2 for q in dem) / n
    cv = (var ** 0.5) / mean if mean else 0.0
    return centroid, gini, cv


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
    ap.add_argument("--out", required=True, help="output root; one subfolder per (mu, s) cell")
    ap.add_argument("--limit", type=int, help="use only this many source instances")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--mu", nargs="+", type=float, default=[0.1, 0.3, 0.5, 0.7, 0.9],
                    help="target demand centroids in (0,1); 0.5 is balanced")
    ap.add_argument("--conc", nargs="+", type=float, default=[2, 4, 10, 30],
                    help="concentrations s = a+b; s=2 at mu=0.5 is the flat reference")
    args = ap.parse_args()

    import glob
    files = sorted(glob.glob(os.path.join(args.src, "*.gt")))
    if args.limit:
        random.Random(args.seed).shuffle(files)
        files = sorted(files[:args.limit])
    if not files:
        sys.exit(f"no .gt files under {args.src}")
    for mu in args.mu:
        if not 0.0 < mu < 1.0:
            sys.exit(f"--mu must lie strictly inside (0,1), got {mu}")

    cells = [(mu, s) for s in args.conc for mu in args.mu]
    for mu, s in cells:
        os.makedirs(os.path.join(args.out, cell_name(mu, s)), exist_ok=True)

    os.makedirs(args.out, exist_ok=True)
    man = open(os.path.join(args.out, "manifest.csv"), "w", newline="")
    mw = csv.writer(man)
    mw.writerow(["cell", "mu_target", "s", "instance", "n",
                 "centroid_measured", "gini_measured", "cv_measured",
                 "centroid_source", "gini_source", "total_demand_error"])

    written = 0
    for src in files:
        hdr, n, dem, dret, dnext = parse(src)
        base = os.path.basename(src)
        c0, g0, _ = stats(dem)
        for mu, s in cells:
            a, b = mu * s, (1.0 - mu) * s
            newdem = reweight(dem, a, b)
            cell = cell_name(mu, s)
            write(os.path.join(args.out, cell, base), hdr, n, newdem, dret, dnext,
                  f"{base[:-3]}_{cell}")
            c, g, cv = stats(newdem)
            mw.writerow([cell, mu, s, base, n, round(c, 5), round(g, 5), round(cv, 5),
                         round(c0, 5), round(g0, 5),
                         round(abs(sum(newdem) - sum(dem)) / max(1.0, sum(dem)), 9)])
            written += 1
    man.close()
    print(f"{len(files)} instances x {len(cells)} cells -> {written} files under {args.out}")
    print(f"achieved statistics per file logged in {os.path.join(args.out, 'manifest.csv')}")


if __name__ == "__main__":
    main()
