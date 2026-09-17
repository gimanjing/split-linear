#!/usr/bin/env python3
"""Restore Vidal's capacity ladder in Instances/Instances {1,2,3}, scaled per folder.

Vidal's original files (Instances/*.gt) encode a ten-rung capacity ladder in the _01.._10
suffix -- {100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000} -- identical for
every one of the 105 tour families. The derived folders flattened that axis, forcing a single
capacity on all ten files (100, 20 and 10 respectively), which left rho = qbar/Q uncontrolled
and correlated with n, since qbar itself scales as roughly sqrt(n) across families.

This script puts the ladder back, scaled per folder:

    Instances 1 : Q = Q_original          (100 .. 100,000)
    Instances 2 : Q = Q_original * 0.20   ( 20 ..  20,000)
    Instances 3 : Q = Q_original * 0.10   ( 10 ..  10,000)

Only the CAPACITY line is touched. DIMENSION, MAX_ROUTE (the horizon the folders add and the
originals lack), the giant tour, the demands and the geometry are all preserved byte for byte.

  python3 rescale_instances.py --dry-run     # report what would change
  python3 rescale_instances.py               # rewrite, then verify
"""
import argparse
import glob
import os
import re
import sys

SCALE = {1: 1.0, 2: 0.20, 3: 0.10}
CAP_RE = re.compile(r"^(\s*CAPACITY\s*:\s*)(\S+)(.*)$")


def header_value(path, key):
    """The value of a header key, read token-wise the way Pb_Data.cpp reads it."""
    tok = open(path).read().split()
    if key not in tok:
        return None
    i = tok.index(key) + 1
    if i < len(tok) and tok[i] == ":":
        i += 1
    return float(tok[i]) if i < len(tok) else None


def demands(path):
    """The demand column, so we can prove it never moved."""
    tok = open(path).read().split()
    n = int(tok[tok.index("DIMENSION") + 2])
    pos = tok.index("GIANT_TOUR_SECTION") + 1
    out = []
    for k in range(1, n + 1):
        pos += 1
        out.append(tok[pos])
        pos += 2
        if k < n:
            pos += 1
    return out


def fmt(q):
    """Integer when the scaled capacity is integral, as every rung x {1, .2, .1} is."""
    return str(int(round(q))) if abs(q - round(q)) < 1e-9 else repr(q)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="Instances")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    changed = skipped = 0
    for folder, scale in sorted(SCALE.items()):
        d = os.path.join(args.root, f"Instances {folder}")
        files = sorted(glob.glob(os.path.join(d, "*.gt")))
        if not files:
            sys.exit(f"no .gt files under {d}")
        seen = {}
        for path in files:
            original = os.path.join(args.root, os.path.basename(path))
            if not os.path.exists(original):
                sys.exit(f"no original for {path}")
            q_orig = header_value(original, "CAPACITY")
            if q_orig is None:
                sys.exit(f"{original} has no CAPACITY")
            q_new = q_orig * scale

            lines = open(path).read().split("\n")
            for i, line in enumerate(lines):
                m = CAP_RE.match(line)
                if m:
                    lines[i] = f"{m.group(1)}{fmt(q_new)}{m.group(3)}"
                    break
            else:
                sys.exit(f"{path} has no CAPACITY line")

            if not args.dry_run:
                with open(path, "w") as fh:
                    fh.write("\n".join(lines))
            seen.setdefault(q_orig, 0)
            seen[q_orig] += 1
            changed += 1
        rungs = ", ".join(f"{fmt(q * scale)}" for q in sorted(seen))
        print(f"Instances {folder} (x{scale:g}): {len(files)} files, ladder -> {rungs}")

    if args.dry_run:
        print(f"\ndry run: {changed} files would be rewritten")
        return

    # Verify: capacity is the scaled original, and nothing else moved.
    bad = 0
    for folder, scale in sorted(SCALE.items()):
        for path in sorted(glob.glob(os.path.join(args.root, f"Instances {folder}", "*.gt"))):
            original = os.path.join(args.root, os.path.basename(path))
            want = header_value(original, "CAPACITY") * scale
            got = header_value(path, "CAPACITY")
            if got is None or abs(got - want) > 1e-9:
                print(f"  CAPACITY mismatch {path}: {got} != {want}")
                bad += 1
            if demands(path) != demands(original):
                print(f"  demand column moved in {path}")
                bad += 1
            if header_value(path, "DIMENSION") != header_value(original, "DIMENSION"):
                print(f"  DIMENSION moved in {path}")
                bad += 1
            if header_value(path, "MAX_ROUTE") is None:
                print(f"  MAX_ROUTE lost in {path}")
                bad += 1
    print(f"\nrewrote {changed} files; verification {'FAILED' if bad else 'passed'}"
          f"{f' ({bad} problems)' if bad else ''}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
