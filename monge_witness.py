"""
Counterexamples behind NOTES.md 5.7 : the PT-VRP cost matrix is neither Monge
nor inverse-Monge.

For i < i' and j < j' the discrete mixed second difference is

    Delta = c(i,j) + c(i',j') - c(i,j') - c(i',j)

    Delta == 0 everywhere  <=>  c is separable, c(i,j) = A[i] + B[j]   (Vidal)
    Delta <= 0 everywhere  <=>  c is Monge                             (SMAWK / ASK)
    Delta >= 0 everywhere  <=>  c is inverse-Monge

PT-VRP has c(i,j) = m(i,j) * (A[i] + B[j]) with m = max(1, ceil((L[j]-L[i])/Q)).
The load gap l(i,j) = L[j] - L[i] is modular, so f(l) is Monge iff f is convex;
ceil is a staircase, not convex.  Both signs of Delta occur on strictly
increasing load profiles, so the matrix lies in neither cone.

Run:  python3 monge_witness.py
"""

import itertools
import math
import random


def trips(li, lj, Q):
    return max(1, math.ceil((lj - li) / Q))


def delta(L, A, B, i, ip, j, jp, Q):
    """Mixed second difference of c(i,j) = m(i,j) * (A[i] + B[j])."""
    c = lambda a, b: trips(L[a], L[b], Q) * (A[a] + B[b])
    return c(i, j) + c(ip, jp) - c(i, jp) - c(ip, j)


def hand_built():
    """The two closed-form witnesses quoted in NOTES.md 5.7."""
    Q = 10
    cases = [
        ("step at the near corner", [0, 5, 15, 20], [0.0, 0.0], [10.0, 10.0]),
        ("step at the far corner",  [0, 4,  8, 12], [0.0, 0.0], [10.0, 10.0]),
    ]
    print("Hand-built witnesses (Q = %d)\n" % Q)
    for name, L, (a1, a2), (b1, b2) in cases:
        A = [a1, a2, 0.0, 0.0]          # A[i], A[i'] ; A non-increasing
        B = [0.0, 0.0, b1, b2]          # B[j], B[j'] ; B non-decreasing
        d = delta(L, A, B, 0, 1, 2, 3, Q)
        m = [trips(L[0], L[2], Q), trips(L[0], L[3], Q),
             trips(L[1], L[2], Q), trips(L[1], L[3], Q)]
        print("  %-24s L=%-16s m(i,j),(i,j'),(i',j),(i',j') = %s" % (name, L, m))
        print("  %-24s Delta = %+.1f  ->  %s\n"
              % ("", d, "Monge VIOLATED" if d > 0 else "inverse-Monge VIOLATED"))


def random_scan(Q=10, n=9, trials=200_000, seed=1):
    """Sample load/cost profiles under the monotonicity constraints of 3.3."""
    rng = random.Random(seed)
    worst_monge = worst_inverse = 0.0
    for _ in range(trials):
        L = sorted(rng.sample(range(0, 60), n))
        A = sorted((rng.uniform(-30, 5) for _ in range(n)), reverse=True)
        B = sorted(rng.uniform(0, 40) for _ in range(n))
        for i, ip, j, jp in itertools.combinations(range(n), 4):
            # i < i' < j < j' so that the arc (i',j) exists
            if min(A[i] + B[j], A[i] + B[jp],
                   A[ip] + B[j], A[ip] + B[jp]) < 0:
                continue                      # g must be a real route cost
            d = delta(L, A, B, i, ip, j, jp, Q)
            worst_monge = max(worst_monge, d)
            worst_inverse = max(worst_inverse, -d)
    print("Random scan (%d profiles, n = %d, Q = %d)\n" % (trials, n, Q))
    print("  largest Delta > 0 (Monge violated)         : %+.3f" % worst_monge)
    print("  largest Delta < 0 (inverse-Monge violated) : %+.3f" % -worst_inverse)
    print("\n  Both signs occur -> the matrix is in neither cone.")


if __name__ == "__main__":
    hand_built()
    random_scan()
