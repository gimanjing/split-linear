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

`PTVRP_LAYERED_SAFE` (§8.2) adds two more, since it *prevents* those pops rather than counting them:

| counter | meaning |
|---|---|
| `BLOCKED POPS` | back-pops the joint-dominance guard refused — the ones `UNSAFE POPS` would have counted |
| `UNSORTED QUERIES` | queries in a layer that a blocked pop left un-key-sorted, so the `O(1)` front query was replaced by a scan |

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

> **This certificate is conditional, and the condition turned out to be load-bearing.** It certifies
> *these runs*, because `UNSAFE POPS` happened to be zero on them. §8 exhibits instances where the
> counter fires, and §8.1 a five-vendor instance where the back-pop loses the answer outright.
> `PTVRP_LAYERED_SAFE` (§8.2) makes the second condition hold by construction rather than by
> observation, so it needs no certificate at all.

Why unsafe pops are so rare **on this benchmark**, as a plausible reason rather than a proof: a pop with
`At[i] > At[b]` needs `p[i] + k·A[i] <= p[b] + k·A[b]` with `A[i] > A[b]`. That means
`p[i] < p[b]`: serving a *longer* prefix strictly cheaper than a shorter one. That in turn needs a
violation at exactly the right place, *and* one large enough to beat `k` times the gap in `A`. A
**rounding** violation is capped at one unit, so the second requirement almost never holds. That cap
is the whole reason this is zero — and it is a property of the benchmark, not of the algorithm. Where
the violation is unbounded (§8) the counter fires on every instance.

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

### 6.4 Head-to-head rerun: the sound frontier against the unsound one

§6.3 reads the two solvers off separate sweeps. This is the direct comparison: both solvers on the
same instance in the same process, `PTVRP_LAYERED_CONT_FIX` first so `gap_vs_first` is the unsound
decoder's error against the sound one. 6,300 rows in `sweep_layered_vs_contfix.csv`, 34 s wall on the
machine in §11, three folders in parallel.

| | result |
|---|---|
| identical cost | **3,150 / 3,150** (2,829 feasible, 321 infeasible) |
| `gap_vs_first` non-zero | **0 rows** |
| `PTVRP_LAYERED_CONT_FIX` vs the `PTVRP_CONT` oracle | **3,150 / 3,150** identical |
| `UNSAFE POPS` | 0 everywhere |

**What soundness costs, measured side by side: 1.08× overall** (39.6 s against 36.7 s). The ratio
grows with instance size, because at small `n` both runs are a few milliseconds and process startup
dominates:

| size | `..._CONT_FIX` | `PTVRP_LAYERED` | ratio |
|---|---|---|---|
| n < 500 (1,380) | 7.9 s | 8.1 s | 0.96× |
| 500 – 2,000 (780) | 6.3 s | 6.2 s | 1.02× |
| 2,000 – 10,000 (600) | 9.6 s | 8.8 s | 1.10× |
| n >= 10,000 (390) | 15.8 s | 13.6 s | **1.16×** |

Per-instance medians at `n >= 3,000` are 1.18× at `Q = 10`, 1.19× at `Q = 20` and 1.23× at `Q = 100`,
falling to 1.0–1.1× above that. So **§6.3's 1.18× is the right figure for instances where the work is
real**, and the 1.08× total is diluted by the 2,160 instances that finish in milliseconds. Quote the
per-size number, not the total.

The source of the difference is unchanged from §6.1 — the sound frontier bounds the layer count
slightly less tightly:

| | median `eff_K` | mean | max | median `K_allocated` |
|---|---|---|---|---|
| `PTVRP_LAYERED_CONT_FIX` | **17.3** | 25.5 | 135 | 33 |
| `PTVRP_LAYERED` | **15.0** | 21.5 | 127 | 27 |

One caveat on that comparison: `PTVRP_LAYERED` throws before printing its diagnostics on the 321
infeasible instances, so its `eff_K` statistics cover 2,829 instances against the fixed solver's
3,150.

Counters on this run: revived layer winners on **20** instances, 2 improving a label, 0 answers
changed; `INFEASIBLE SKIPS` 60.9 M across 2,653 instances — three orders of magnitude below the
33.0 × 10⁹ of the unbounded control in §5.5, which is the bound doing its work. The 20 instances
against §4.1's 18 for `PTVRP_LAYERED_CONT` is a small unexplained difference: the two solvers replay
the frontier under different layer bounds, so they do not have to agree, but it has not been checked.

The slowest single run is 0.13 s (`ch71009`, n = 71,008), against 117 s for the unbounded control.

---

## 7. Summary

1. **Correct.** Right on all four counterexamples. Identical to the exact `PTVRP_CONT` on all 3,150
   instances. With zero unsafe pops everywhere, it is exact by construction **on this benchmark**
   (§3) — a conditional statement, and §8 is where the condition fails.
2. **Revival is real but never decisive here.** 18 instances have a layer won by a revived start,
   2 of those improve a label, and 0 change an answer. All 18 are inside Bellman's 103.
3. **Without a bound, cost is `Θ(n · K_full)`, measured.** Exponent 0.95, 21 ns per layer visit.
   Faster than Bellman's `O(n²)` control for Q >= 200, slower for Q <= 100, where `K_full` exceeds `n`.
4. **The frontier is worth 12× in `eff_K` — and it need not be unsound.** Pruning on the path out
   instead of the full route restores the bound with no assumption about the instance (§6). Median
   `eff_K` on `Instances 3` goes 998.9 -> 30.2, against 29.2 for the unsound frontier.
5. **Soundness costs 1.18× (layered) and 1.13× (Bellman)**, not the 169× the unbounded control needed.
   Both remain ~20× faster than the oracle that assumes nothing (§6.3).
6. **The back-pop was the remaining assumption, and it is now closed too.** It is counted
   (`UNSAFE POPS`) and never fires on this benchmark — but it fires on every structurally non-metric
   instance, and on `ce_unsafe_pop_5v` it loses the answer outright, turning an optimum of 658 into
   `NO SOLUTION` (§8.1). `PTVRP_LAYERED_SAFE` replaces the key-only eviction with a joint-dominance
   one (§8.2): exact, free on this benchmark, and 60/60 where `PTVRP_LAYERED_CONT_FIX` was 59/60.
   `PTVRP_CONT_FIX` never carried the assumption at all — it has no deque.

7. **The framing is not about rounding.** Rounding is the cheapest witness, not the mechanism. The
   mechanism is that the pruning is justified by the triangle inequality, a property of *distance*,
   while every constraint here is on *duration* — and the two coincide only when duration is one fixed
   function of distance. §8 is what happens when it is not, which is the case any real travel-time
   model presents.

---

## 8. Where the fix stopped being unconditional -- and how that was closed

Everything above uses rounding as the source of the violation, because it is the cheapest witness:
already present in the data, needing no modelling assumption to defend. It is also the **smallest**
violation of its family, and this section is where that matters.

The triangle inequality is a statement about **distance**. Every pruning rule and every deque
invariant here is applied to **duration**. They coincide only when duration is one fixed function of
distance -- which these instances satisfy only because they set `tau = 2d`, a constant speed.
Anything that prices the leg home independently of the arcs that replace it separates them:

| source | violation |
|---|---|
| independent integer rounding | exactly **1 unit**, never more |
| time-dependent travel -- the leg home driven at a different hour, hence a different speed | unbounded |
| queueing or service at the depot, folded into the return leg | unbounded |
| driver-hours, breaks, shift ends charged wherever the leg home falls | unbounded |
| asymmetric networks -- one-way systems, turn restrictions, tolls, ferries | unbounded |

So the path-out bound, being instance-independent by construction, was worth testing on data that
breaks the inequality *structurally* rather than by one unit -- what any of rows 2-5 would produce.

`Instances/Counterexamples/structural_violation.gt` is such an instance: legs home drawn
independently of the inter-vendor arcs, so **46% of positions violate the triangle inequality, by a
median of 156 units and up to 482**. TSPLIB violations are always exactly 1.

Over 60 instances of that kind:

| | result |
|---|---|
| instances with `UNSAFE POPS` > 0 | **60 / 60** (5,304 pops) |
| `PTVRP_CONT_FIX` wrong | **0 / 60** |
| `PTVRP_LAYERED_CONT_FIX` wrong | **1 / 60** (oracle 88,554, returned 88,909) |

**The Bellman fix is unconditional.** Its only assumption is the path-out bound, which is arithmetic
-- append a non-negative leg, remove nothing -- and holds however the leg is priced.

**The layered fix is not -- and the defect is inherited, not introduced.** On the failing instance
`PTVRP_LAYERED_CONT` returns the *same* wrong answer (88,909) despite having no path-out bound at
all, while fully-unsound `PTVRP_LAYERED` returns 97,017 against the true 88,554. So the bound changed
the speed, not the answer: it removes 96% of the error (9.56% -> 0.40%) and cannot remove the rest.

The residue is the deque's key-only back-pop, shared by every layered solver here: an older start is discarded
when a newer one has a key at least as small, which is safe for feasibility only if the newer start
also fits the horizon whenever the older does, i.e. `At[new] <= At[old]`. That needs `At`
non-increasing, which needs the triangle inequality. Rounding never triggered it -- **zero across all
3,150 benchmark instances**. Structural violations trigger it on every instance, and occasionally
decisively.

`UNSAFE POPS` fired on the failing instance -- 78 in the fixed solver, 103 in the unbounded control
-- so the counter is a working alarm rather than a decoration, and §3's exactness certificate
correctly declines to certify there. The certificate was always conditional on that counter being
zero; this is the instance showing the condition is load-bearing. Three ways to use it, of which the
third turned out to be the right one:

1. **Prefer `PTVRP_CONT_FIX`** wherever duration is not a function of distance. It carries no deque
   and no such assumption.
2. **Treat `UNSAFE POPS > 0` as a fallback trigger** -- rerun that instance on the Bellman fix.
3. **Make the back-pop feasibility-aware** -- only evict when `At[new] <= At[old]` as well as on key.
   **Implemented as `PTVRP_LAYERED_SAFE` (§8.2).** The worry that it would cost deque length was
   unfounded: on this benchmark the guard never blocks a single pop, so the cost is nil, and off it
   the guard is the difference between 59/60 and 60/60.

### 8.1 A five-vendor instance where the back-pop loses the answer

`Instances/Counterexamples/ce_unsafe_pop_5v.gt`. Small enough to check by hand, and the failure is
the worst kind: **`NO SOLUTION` on an instance that is feasible at 658.**

```
CAPACITY 10   MAX_ROUTE 200

  vendor   demand   home   to next
       1        5     79         1
       2       11     42         7
       3        2     74        12
       4        7      8         7
       5       11     77         -
```

| solver | result |
|---|---|
| `PTVRP`, `PTVRP_CONT`, `PTVRP_CONT_FIX`, `PTVRP_LAYERED` | **658** (exhaustive enumeration agrees) |
| `PTVRP_LAYERED_CONT`, `PTVRP_LAYERED_CONT_FIX` | **NO SOLUTION**, `UNSAFE POPS` = 1 |
| `PTVRP_LAYERED_SAFE` | **658**, `BLOCKED POPS` = 1 (§8.2) |

The deque ranks starts by `key(i,k) = p[i] + k·A[i]`, where `A[i] = d(v_{i+1},depot) − path(v_1..v_{i+1})`.
The triangle inequality would force `A` to be non-increasing. Here it is not: `A = [79, 41, 66, −12, 50]`,
violated at `i=2` and again at `i=4`.

At column `j=5`, layer `k=2` holds the starts needing exactly two trips:

| start | `p[i]` | `A[i]` | key | its route | fits `T_H = 200`? |
|---|---|---|---|---|---|
| 2 | 326 | 66 | 458 | `d=170`, `d·2=340` | no |
| 3 | 474 | −12 | **450** | `d=92`, `d·2=184` | **yes** |
| 4 | 296 | 50 | **396** | `d=154`, `d·2=308` | no |

The deque evicts the back whenever the newcomer's key is no larger:

```
  add 2  ->  [2]                     key 458
  add 3  ->  450 <= 458, pop 2       A[3] = -12 < A[2] = 66   newcomer fits MORE easily -- safe
  add 4  ->  396 <= 450, pop 3       A[4] =  50 > A[3] = -12  newcomer fits LESS easily -- UNSAFE
```

The front is now start 4, whose route is `308 > 200`. It is skipped as infeasible, the deque is
exhausted, and layer 2 yields nothing — so `p[5]` never gets a label.

Start 3 was feasible at `184`, and `p[3] + 184 = 474 + 184 = 658` is the optimum. It was discarded by
a start that is **cheaper on paper and does not fit**. The key ranks on cost alone; feasibility is
not in it. Under the triangle inequality that never matters, because a later start always fits at
least as easily. Here it does not.

Reproduce:

```bash
for s in PTVRP_CONT PTVRP_CONT_FIX PTVRP_LAYERED_CONT_FIX PTVRP_LAYERED_SAFE; do
  echo -n "$s "
  Program/split Instances/Counterexamples/ce_unsafe_pop_5v.gt -solver $s 2>&1 \
    | grep -oE "SOLUTION COST : [0-9.]+|UNSAFE POPS : [0-9]+|BLOCKED POPS : [0-9]+|no Split solution"; echo
done
```

### 8.2 `PTVRP_LAYERED_SAFE`: sound eviction, and the last assumption goes

The trace above names its own repair. The key compares cost; feasibility is not in it. So put it in:

```
    evict b   when   key(i) <= key(b)   AND   At[i] <= At[b]
```

**The second clause is exact, not conservative.** Feasibility at column `j` in layer `k` is
`(At[start] + Bt[j])·k <= T_H`. Comparing two starts at the **same** `j` cancels `Bt[j]`, exactly as
it cancels in the cost, so `At[i] <= At[b]` means "wherever `b` fits, `i` fits" — at every `j`, for
good. That is the one `j`-independent comparison that still accounts for the leg home, which is why
this repair belongs in the **eviction** and could never have gone into the pruning: there the leg home
is precisely what has to be dropped (§6). The two fixes are complementary, not alternatives.

Blocking a pop can leave a layer no longer sorted by key, so `sortedByKey[k]` is tracked per layer.
While a layer is sorted the query is the original `O(1)` "first feasible from the front"; once a pop
there has been blocked, that layer scans for the cheapest feasible entry instead. So the redesign is a
per-layer flag rather than a rewrite, and it is confined to layers where the guard actually fires.
`BLOCKED POPS` and `UNSORTED QUERIES` report both.

| | `PTVRP_LAYERED_CONT_FIX` | `PTVRP_LAYERED_SAFE` |
|---|---|---|
| `ce_unsafe_pop_5v` (optimum 658) | `NO SOLUTION` | **658**, 1 pop blocked |
| the four §2 counterexamples, both demos | correct | correct |
| 60 structurally non-metric instances | 59 / 60 | **60 / 60**, 5,205 pops blocked |
| 3,150 benchmark instances vs `PTVRP_CONT` | 2,829 identical, 321 both-infeasible, 0 differ | **identical** |
| `BLOCKED POPS` / `UNSORTED QUERIES` on the benchmark | — | **0 / 0** |
| total, median `eff_K` | 28.0 s, 17.3 | 28.0 s, 17.3 |

Read the last three rows together, because the stronger claim is in them. On this benchmark the guard
costs nothing **because it never has to act** — `UNSORTED QUERIES = 0` means no layer ever left the
original `O(1)` query. So the guard does not rescue `PTVRP_LAYERED_CONT_FIX` here; it *confirms* that
`PTVRP_LAYERED_CONT_FIX` was already correct here. The two diverge only once the data stops being
metric, which per §8 is where every real duration model lives.

**The layered decoder now carries no assumption about the instance**, and §3's conditional — "exact
provided `UNSAFE POPS` is zero" — can be retired: the condition is enforced rather than observed.
`PTVRP_CONT_FIX` never carried one, having no deque. `sweep_layered_safe.csv` holds the run.

Reproduce the 60-instance sweep:

```bash
for s in PTVRP_CONT PTVRP_CONT_FIX PTVRP_LAYERED_CONT_FIX PTVRP_LAYERED_SAFE; do
  echo -n "$s "
  Program/split Instances/Counterexamples/structural_violation.gt -solver $s 2>&1 \
    | grep -oE "SOLUTION COST : [0-9.]+|UNSAFE POPS : [0-9]+|BLOCKED POPS : [0-9]+" | tr '\n' ' '; echo
done
```

---

## 9. Open, following from this

- **Split the 85-instance gap (§4.1).** Count *every* feasible start behind the replayed frontier, not
  only layer winners. That separates "different pointer" from "not the winner". **Still open.**
- **Look for an instance with `UNSAFE POPS > 0` — ANSWERED (§8, §8.1), and then patched (§8.2).**
  `structural_violation.gt` fires the counter on 60 of 60 instances; `ce_unsafe_pop_5v.gt` is five
  vendors where the back-pop turns an optimum of 658 into `NO SOLUTION`. So the answer is that the
  back-pop can genuinely lose the answer, not merely that it is unprovable. `PTVRP_LAYERED_SAFE`
  closes it with a joint-dominance eviction that is exact and, on this benchmark, free.
- **A sound frontier would recover most of the 12× — ANSWERED (§6).** Not by the suffix sum `D[j]`
  that `revival.md` §9 first proposed, but by pruning on the path out, which needs nothing
  precomputed and applies unchanged to both decoders. Measured at 1.18× and 1.13×.
- **Infeasible instances (§5.4) — largely answered.** The path-out bound is exactly such a lower
  bound on time, and it fires on instances where nothing can fit: the 321 infeasible instances now
  cost the fixed solvers roughly what feasible ones do, instead of a full unbounded scan.
- **Does the fix hold off this benchmark? — partly measured (§8).** Off *metric* data it now is: 60
  structurally non-metric instances, `PTVRP_CONT_FIX` 60/60 and `PTVRP_LAYERED_SAFE` 60/60. What is
  still untested is a different **horizon** — every run here is `T_H = 86,400`. The two-horizon rerun
  (`NOTES.md` §7.3) would cover it cheaply, and would also settle §7.3's own prediction. Note there is
  no `-horizon` flag yet; the cheap route is to add one rather than rewrite 6,300 instance files.
- **Measure the guard where it fires.** `BLOCKED POPS` and `UNSORTED QUERIES` are 0 on the whole
  benchmark, so the cost of a layer leaving the `O(1)` query has only been observed on the 60
  structural instances. How the scan-for-cheapest-feasible fallback scales when most layers are
  unsorted is unmeasured.

---

## 10. Reproducing

```bash
cd Program && make && cd ..

# counterexamples -- only PTVRP_CONT_FIX and PTVRP_LAYERED_SAFE are right on all of them
for f in Instances/Counterexamples/*.gt; do
  for s in PTVRP PTVRP_CONT PTVRP_CONT_FIX PTVRP_LAYERED PTVRP_LAYERED_CONT_FIX PTVRP_LAYERED_SAFE; do
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

# the head-to-head of §6.4 (~34 s) -- the sound frontier against the unsound one,
# both solvers on the same instance, so gap_vs_first is the error of the second
for d in 1 2 3; do
  python3 batch_run.py --dir "Instances/Instances $d" \
      --solver PTVRP_LAYERED_CONT_FIX PTVRP_LAYERED --timeout 1800 --out cf_$d.csv &
done; wait
head -1 cf_1.csv > sweep_layered_vs_contfix.csv
for d in 1 2 3; do tail -n +2 cf_$d.csv >> sweep_layered_vs_contfix.csv; done
```

---

## 11. Machine and environment

Every timing in this file comes from one machine:

| | |
|---|---|
| CPU | **Intel Core i7-9700K @ 3.60 GHz**, 8 cores / 8 threads (no SMT) |
| RAM | 16 GB host; ~7 GB visible to the VM |
| OS | Windows 11 Pro 22621, WSL2 Ubuntu, kernel 6.18.33.2-microsoft-standard-WSL2 |
| compiler | g++ (Ubuntu) 15.2.0, `-O3` |
| harness | `batch_run.py`, one process per instance folder, three folders in parallel |

Three parallel processes on 8 threads leave the cores oversubscribed only by memory bandwidth, not by
count, but the runs are not isolated: an instance timed alongside two others is slower than the same
instance timed alone. Single-instance measurements quoted in the text (§5.1's 21 ns per layer visit,
§6.4's slowest run) were taken with nothing else running.

Timing note, carried forward: the solver seconds in §5.3's table come from the earlier sweep in commit
`5049f5b`, which ran under the same setup but not in the same session. Treat those ratios as
indicative to within tens of percent. Every conclusion in this file rests on ratios of 5× or more, on
the same-process comparison of §6.4, or on cost agreement — not on small timing differences.
