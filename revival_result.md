# Revival: results of the full layered sweep

`revival.md` showed that stopping on time is unsound: a route can go over the horizon and a longer
one can come back under it. It had one control covering the whole benchmark, `PTVRP_CONT` (Bellman,
every pair scanned), and a layered control, `PTVRP_LAYERED_CONT`, that at `O(n²K)` only reached the
480 instances with `n <= 600`.

This file reports the rebuilt layered control over **all 3,150 instances**, and what it says about
revival, exactness and cost. Data: `sweep_PTVRP_LAYERED_CONT.csv` (this run), compared with
`sweep_PTVRP_CONT.csv`, `sweep_PTVRP_LAYERED.csv` and `sweep_PTVRP.csv` (commit `5049f5b`).

---

## 1. What changed in the solver

`PTVRP_LAYERED_CONT` is now **`PTVRP_LAYERED` with its two time pointers removed** and nothing else
changed (`Program/Split_Layered_PTVRP_cont.{h,cpp}`):

| piece | `PTVRP_LAYERED` | `PTVRP_LAYERED_CONT` (new) |
|---|---|---|
| layer windows by load, `firstLoadLE[k]` | yes | yes, unchanged |
| Vidal deque per layer, key `p[i] + k·A[i]` | yes | yes, unchanged |
| front eviction on load | yes | yes, unchanged |
| `firstFeasible` — column frontier, one-way | yes | **removed** |
| `firstTimeLE[k]` — per-layer time suffix, one-way | yes | **removed** |
| layer count per column | `trips(firstFeasible, j)` | `trips(0, j)` — no horizon bound |
| query | deque front | **first start from the front that fits the horizon** |

An infeasible start is **stepped over, never popped**. If it revives in a later column it is still
in the deque. Keys increase from front to back, so the first feasible start is the cheapest feasible
start in the deque.

No penalty is involved: the horizon is still a hard constraint, tested start by start.

The old pointers are replayed alongside, for counting only, so the solver can say when its layer
winner sits where `PTVRP_LAYERED` could not have looked.

New counters, all captured by `batch_run.py`:

| counter | meaning |
|---|---|
| `REVIVED STARTS` | layer winners behind where `PTVRP_LAYERED`'s pointers stood |
| `REVIVED IMPROVING` | those that also beat the label at that moment |
| `INFEASIBLE SKIPS` | deque entries stepped over because they did not fit — the price of not popping |
| `UNSAFE POPS` | back-pops that are not provably safe for feasibility (§3) |

---

## 2. Counterexamples

| instance | optimum | `PTVRP` | `PTVRP_CONT` | `PTVRP_LAYERED` | `PTVRP_LAYERED_CONT` |
|---|---|---|---|---|---|
| `ce_rounding` | **80** | 101 | 80 | 101 | **80** |
| `ce_blatant` | **70** | 120 | 70 | 120 | **70** |
| `ce_multiplier` | **240** | 241 | 240 | 241 | **240** |
| `ce_infeasible` | **70** | NO SOLUTION | 70 | 70 | **70** |

Right on all four. The `ce_rounding` trace shows the mechanism directly:

```
  +-- p[2] : layers 1..1   (pruned solver : feasible starts begin at 1, layers 1..1)
  |  layer k=1 : starts [0,2)  deque [ 0 1 ]  first feasible=1   cand = p[1] + 1*d(1,2) = 102   <- best so far

  +-- p[3] : layers 1..1   (pruned solver : feasible starts begin at 1, layers 1..1)
  |  layer k=1 : starts [0,3)  deque [ 0 1 2 ]  first feasible=0   cand = p[0] + 1*d(0,3) = 80   [behind the frontier]   <- best so far
```

At `p[2]` start 0 is over the horizon (81 > 80). It is skipped, **but it stays in the deque**. At
`p[3]` it fits again (80), it is at the front, and it wins. `PTVRP_LAYERED` had already moved its
frontier past it.

---

## 3. Exactness on the benchmark

| | instances |
|---|---|
| cost identical to `PTVRP_CONT` | **3,150 / 3,150** (2,829 feasible, 321 infeasible) |
| cost identical to `PTVRP_LAYERED` | **3,150 / 3,150** |
| number of templates, `max m` identical to `PTVRP_LAYERED` | 3,150 / 3,150 |
| timeouts | 0 |
| **`UNSAFE POPS`** | **0 on every instance** |

### Why zero unsafe pops is a certificate, not just agreement

The deque drops a start in only two ways:

1. **From the front, on load.** Load only grows, so a start that leaves the layer's window never
   comes back. Always safe.
2. **From the back, when a newer start `i` has a key at least as small as an older start `b`.** In
   the capacity-only problem that is enough. With a horizon, `i` also has to fit **whenever `b`
   fits**. At layer `k` a start fits when `(At[start] + Bt[j])·k <= T_H`, so that means
   `At[i] <= At[b]`.

   Under the triangle inequality `At` is non-increasing, so this holds automatically. On rounded
   instances it can fail. `UNSAFE POPS` counts every back-pop with `At[i] > At[b]`.

If a run reports **zero** unsafe pops, every dropped start was replaced by one that is at least as
cheap **and** at least as feasible. Two more facts close the gap:

- **The replacement stays in scope as long as `b` would.** `i > b`, so it leaves the load window no
  earlier.
- **Chains of pops stay safe.** Both inequalities are transitive.

So every feasible start the deque ever held is represented by a feasible start that is no worse. The
first feasible entry from the front is then the exact best of the layer.

**On all 3,150 instances the layered control is exact by this argument**, independently of the
agreement with `PTVRP_CONT`. The two confirm each other.

Why unsafe pops are so rare, as a plausible reason rather than a proof: a pop with
`At[i] > At[b]` needs `p[i] + k·A[i] <= p[b] + k·A[b]` with `A[i] > A[b]`. That means
`p[i] < p[b]`: serving a *longer* prefix strictly cheaper than a shorter one. That in turn needs a
rounding violation at exactly the right place, *and* one large enough to beat `k` times the gap in
`A`.

---

## 4. How revival behaves on real instances

### 4.1 Counts

| | `PTVRP_CONT` (Bellman) | `PTVRP_LAYERED_CONT` |
|---|---|---|
| what is counted | every arc behind the `break` | the **winner** of a layer, when behind the frontier |
| total | 177 revived arcs, 172 improving | 18 revived winners, 2 improving |
| instances | 103 | 18 |
| overlap | — | **all 18 are among Bellman's 103** |
| answer changed | 0 | 0 |

Every instance where a revived start wins a layer is one where Bellman also sees a revived arc. The
layered view never finds revival that Bellman misses.

The reverse gap, 85 instances, has two possible reasons, and this run does not separate them:

- **Different pointers.** `PTVRP_LAYERED`'s window works per layer on `At[i]` alone. It is not the
  same pointer as Bellman's forward `break`, and it can re-admit a start in a later column
  (`revival.md` §5, `ce_infeasible`).
- **Not the winner.** The counter here fires only when the revived start is the *best feasible*
  start of its layer. A revived start that is feasible but beaten by a cheaper one in the same
  layer is not counted.

### 4.2 Where it happens

| size | instances | revived winners | improving |
|---|---|---|---|
| n < 500 | 1,380 | 1 | 0 |
| 500 – 2,000 | 780 | 5 | 0 |
| 2,000 – 10,000 | 600 | 3 | 0 |
| n >= 10,000 | 390 | **9** | **2** |

More vendors means more horizon crossings, and more chances. The same trend as `revival.md` §6.

The two improving cases are both `ch71009` (n = 71,008) at very large capacity:

| instance | Q | layered revived / improving | Bellman arcs / improving | cost |
|---|---|---|---|---|
| `Instances 1/ch71009_09` | 50,000 | 1 / 1 | 3 / 3 | 6,671,779 — same as `PTVRP_LAYERED` |
| `Instances 1/ch71009_10` | 100,000 | 1 / 1 | 5 / 5 | 6,650,715 — same as `PTVRP_LAYERED` |

A revived start improved a label mid-sweep, and a later start reached the same label just as
cheaply. Improving a label is necessary, not sufficient, for a wrong answer. That remains the
whole story on this benchmark.

### 4.3 Conclusion on revival

The statement from `revival.md` now holds for **both** decoders over **all** instances:

> The early stop is **unsound** (four counterexamples, one at rounding scale), and on this benchmark
> it **never changes an answer**. For the layered decoder this is now shown on all 3,150 instances,
> not only on the 480 small ones.

---

## 5. Complexity and run time

### 5.1 The work is `n · K_full / 2` layer visits

Without a frontier, column `j` visits every layer up to `trips(0, j)`, which grows linearly to
`K_full = ceil(total demand / Q)`. Summed over `j`, that is about `n · K_full / 2`.

| | median | mean | max |
|---|---|---|---|
| `eff_K` = layer iterations / n, `PTVRP_LAYERED_CONT` | **188** | 2,692 | 90,602 |
| `eff_K`, `PTVRP_LAYERED` (with frontier) | **15** | 21.5 | 127 |
| `eff_K / K_full` | **0.503** | | |

Run time follows it closely: **seconds ∝ (n · K_full)^0.95, r² = 0.979** (922 runs over 0.05 s). Cost
per layer iteration is **21 ns** median (p10 15, p90 30). That includes the `trips()` division and
the infeasible skips.

### 5.2 The horizon bound, not the layering, is what makes `PTVRP_LAYERED` fast

`PTVRP_LAYERED`'s `eff_K` of 15 comes from two things at once:

- the layer partition, which is kept here;
- the horizon cutting the layer loop at `trips(firstFeasible, j)`, which is gone here.

Removing the frontier costs **12× in `eff_K` at the median**. The partition alone does not bound the
layer count. The horizon does.

### 5.3 `O(n · K_full)` against Bellman's `O(n²)`

`K_full > n` whenever mean demand exceeds `Q`. That is true on **2,099 of 3,150** instances: all
1,050 at Q = 10, and 1,049 of 1,050 at Q = 20. On those, `n · K_full > n²`, and the layered control
does more work than the plain Bellman control.

Median time ratio `PTVRP_LAYERED_CONT / PTVRP_CONT`, instances with n >= 3,000:

| Q | 10 | 20 | 100 | 200 | 500 | 1,000 | 2,000 | 5,000 | ≥ 10,000 |
|---|---|---|---|---|---|---|---|---|---|
| ratio | **7.35** | **5.02** | 1.32 | 0.70 | 0.33 | 0.19 | 0.12 | 0.07 | 0.05 – 0.06 |
| layered faster | 0/290 | 0/290 | 3/29 | 29/29 | 29/29 | 29/29 | 29/29 | 29/29 | 29/29 |

**Crossover between Q = 100 and Q = 200.** That is the same place as the `PTVRP_LAYERED` vs
`PTVRP` crossover in NOTES §5.5 (Q ≈ 100), and it follows from the same `K ∝ 1/Q` relation. Above
it the layered control is up to 20× faster than Bellman; below it, 5–7× slower.

Total solver time:

| folder | Q | `PTVRP_LAYERED_CONT` | `PTVRP_CONT` | `PTVRP_LAYERED` |
|---|---|---|---|---|
| Instances 1 | 100 – 100,000 | 89 s | 314 s | 8 s |
| Instances 2 | 20 | 1,599 s | 324 s | 11 s |
| Instances 3 | 10 | 2,354 s | 295 s | 11 s |
| **all** | | **4,042 s** | 934 s | 31 s |

The slowest single run is `Instances 3/ch71009_07`: n = 71,008, K_full = 180,802, 117 s,
NO SOLUTION.

### 5.4 Infeasible instances are the expensive ones

The 321 NO SOLUTION instances take **2,519 s of the 4,042 s (62%)**. `PTVRP_LAYERED` abandons a
column the moment its frontier reaches `j`. The continuous control cannot know that nothing will
revive, so it keeps visiting every layer to the end of the tour.

This is the honest cost of not breaking. On an instance where nothing is feasible past some point,
**no-break means no shortcut**.

### 5.5 The price of stepping over instead of popping

| | |
|---|---|
| infeasible skips, total | 33.0 × 10⁹, against 195.3 × 10⁹ layer iterations |
| skips per layer iteration | median **0.36**, mean 1.16, max **432** |

The extremes are all at `Q = 100,000` with `K_full <= 4`: `ca4663_10` (432), `ca4663_09` (197),
`kz9976_10` (149), `rl5934_10`, `d15112_10`.

With a huge `Q` the load window spans almost the whole prefix. Old starts whose templates are far
too long pile up at the front of the deque, and every column walks over them again. So the bound is
`O(n · K_full + skips)`, and in the worst case the skips alone approach `O(n²)`. On this benchmark
they stay below one per layer visit at the median.

---

## 6. The bound comes back: `PTVRP_LAYERED_CONT_FIX`

§5.2 concluded that the horizon frontier, not the layering, is what makes `PTVRP_LAYERED` fast — and
that the frontier is the unsound part. That framing was right about the mechanism and wrong about the
price. The frontier did not need to be unsound. It needed to prune on a different quantity.

**Prune on the path out**, the route without the leg home:

```
    path_out(i,j) = A[i] + sumDistance[j]
```

`sumDistance[j]` is unconditionally non-decreasing — extending a template appends an arc and removes
nothing, because the leg home is exactly what has been excluded. `Bt[j]` is not. Since
`d(i,j) >= path_out(i,j)` and `trips(i,j)` only grows, `path_out(i,j)·trips(i,j) > T_H` proves every
longer template from that start is infeasible for good. Revival lives entirely in the leg home, which
this bound ignores, so it cannot escape it.

So **both** one-way pointers come back, testing `Bout_t` instead of `Bt`:

| | `PTVRP_LAYERED` | `PTVRP_LAYERED_CONT` | `..._CONT_FIX` |
|---|---|---|---|
| column frontier | `Bt` — unsound | removed | `Bout_t` — **sound** |
| per-layer suffix `firstTimeLE[k]` | `Bt` — unsound | removed | `Bout_t` — **sound** |
| feasibility at the deque front | pops on time | steps over, exact | steps over, exact |
| layer bound | horizon | **none** (`K_full`) | horizon |

Pruning is conservative; the feasibility test stays exact on the true route time.

### 6.1 It is exact, and the bound is recovered

All 3,150 instances against `PTVRP_CONT` (`sweep_both_fixes.csv`): **2,829 identical, 321
both-infeasible, 0 disagreements, 0 `UNSAFE POPS`.**

Median `eff_K`:

| set | `CONT` (no bound) | **`FIX` (sound bound)** | `LAYERED` (unsound) |
|---|---|---|---|
| `Instances 1` | 4.0 | **2.0** | 1.9 |
| `Instances 2` | 499.7 | **24.2** | 18.5 |
| `Instances 3` | 998.9 | **30.2** | 29.2 |

On `Instances 3` the sound bound cuts the layer count to **3.0%** of the unbounded version, landing
within 3% of what the unsound frontier achieved.

### 6.2 Bellman gets the same rule, in one line

`PTVRP_CONT_FIX` (`Program/Split_Bellman_PTVRP_cont_fix.{h,cpp}`). The Bellman loop's running `time`
already *is* the path out — the return leg only enters `tau_ij` — so:

```cpp
if (pathOut * m > myData->horizon + 1.e-9)
    break ;
```

Its own counters give the price in arcs. Twelve largest instances per set, arcs scanned per start
against where the unsound stop would have ended:

| set | sound | unsound | overhead | no stop (`n/2`) |
|---|---|---|---|---|
| `Instances 1` | 518.2 | 463.4 | 1.12× | 35,504 |
| `Instances 2` | 11.5 | 6.6 | 1.73× | 35,504 |
| `Instances 3` | 6.5 | 3.8 | 1.72× | 35,504 |

The ratio is worse where capacity is low, but the absolute numbers there are 6–12 arcs per start.

### 6.3 What soundness actually costs

All five solvers, one machine, all 3,150 instances:

| solver | total | |
|---|---|---|
| `PTVRP_LAYERED` | 23.7 s | unsound frontier |
| **`PTVRP_LAYERED_CONT_FIX`** | **28.0 s** | **exact** |
| `PTVRP` | 34.0 s | unsound early stop |
| **`PTVRP_CONT_FIX`** | **38.4 s** | **exact** |
| `PTVRP_CONT` | 781.8 s | no stop — the oracle |
| `PTVRP_LAYERED_CONT` | 4,041.5 s | unbounded control (your i7, ≈3,496 s here) |

**1.18× for the layered decoder, 1.13× for Bellman.** Both about 20× faster than the oracle.

This supersedes §5.2's "the frontier is worth 12× and it is the unsound part". The correct statement
is: *the frontier is worth 12×, and a sound frontier recovers it for 18%.*

---

## 7. Summary

1. **Correct.** Right on all four counterexamples. Identical to the exact `PTVRP_CONT` on all 3,150
   instances. With zero unsafe pops everywhere, it is exact by construction on this benchmark (§3).
2. **Revival is real but never decisive here.** 18 instances have a layer won by a revived start,
   2 of those improve a label, and 0 change an answer. All 18 are inside Bellman's 103.
3. **Without a bound, cost is `Θ(n · K_full)`, measured.** Exponent 0.95, 21 ns per layer visit.
   Faster than Bellman's `O(n²)` control for Q >= 200, slower for Q <= 100, where `K_full` exceeds `n`.
4. **The frontier is worth 12× in `eff_K` — and it need not be unsound.** Pruning on the path out
   instead of the full route restores the bound with no assumption about the instance (§6). Median
   `eff_K` on `Instances 3` goes 998.9 -> 30.2, against 29.2 for the unsound frontier.
5. **Soundness costs 1.18× (layered) and 1.13× (Bellman)**, not the 169× the unbounded control needed.
   Both remain ~20× faster than the oracle that assumes nothing (§6.3).
6. **The remaining assumption is the back-pop.** It is counted (`UNSAFE POPS`) and never fired here,
   in either the unbounded control or the fixed solver. `PTVRP_CONT_FIX` has no deque and carries no
   such assumption at all.

---

## 8. Open, following from this

- **Split the 85-instance gap (§4.1).** Count *every* feasible start behind the replayed frontier, not
  only layer winners. That separates "different pointer" from "not the winner".
- **Look for an instance with `UNSAFE POPS > 0`.** Construct one, in the style of `ce_rounding`, to show
  whether the back-pop can actually lose the answer or is only unprovable.
- **A sound frontier would recover most of the 12× — ANSWERED (§6).** Not by the suffix sum `D[j]`
  that `revival.md` §9 first proposed, but by pruning on the path out, which needs nothing
  precomputed and applies unchanged to both decoders. Measured at 1.18× and 1.13×.
- **Infeasible instances (§5.4) — largely answered.** The path-out bound is exactly such a lower
  bound on time, and it fires on instances where nothing can fit: the 321 infeasible instances now
  cost the fixed solvers roughly what feasible ones do, instead of a full unbounded scan.
- **Does the fix hold off this benchmark?** Every result here is `T_H = 86,400` and TSPLIB rounding.
  The path-out argument is instance-independent by construction, so it should hold anywhere — but
  "should" is not "measured". The two-horizon rerun (`NOTES.md` §7.3) would test it cheaply.

---

## 9. Reproducing

```bash
cd Program && make && cd ..

# counterexamples
for f in Instances/Counterexamples/*.gt; do
  for s in PTVRP PTVRP_CONT PTVRP_CONT_FIX PTVRP_LAYERED PTVRP_LAYERED_CONT_FIX; do
    echo -n "$(basename $f) $s "; Program/split "$f" -solver $s 2>&1 | grep -E "SOLUTION COST|no Split"
  done
done

# the revival trace of §2
Program/split Instances/Counterexamples/ce_rounding.gt -solver PTVRP_LAYERED_CONT -trace 1

# the sweep (three folders in parallel, ~70 CPU-minutes, the Q = 10 folder dominates)
for d in 1 2 3; do
  python3 batch_run.py --dir "Instances/Instances $d" --solver PTVRP_LAYERED_CONT \
      --timeout 1800 --out lc_$d.csv &
done; wait
head -1 lc_1.csv > sweep_PTVRP_LAYERED_CONT.csv
for d in 1 2 3; do tail -n +2 lc_$d.csv >> sweep_PTVRP_LAYERED_CONT.csv; done
```

Timing note: this sweep ran three processes in parallel on 8 threads under WSL2. The other solvers'
seconds come from the earlier sweep in commit `5049f5b`. Treat the ratios in §5.3 as indicative to
within tens of percent, not exact. Every conclusion above rests on ratios of 5× or more, or on cost
agreement, not on small timing differences.
