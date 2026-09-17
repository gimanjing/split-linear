# Revival: results of the full layered sweep

`revival.md` showed that stopping on time is unsound: a route can go over the horizon and a longer
one can come back under it. It had one control covering the whole benchmark, `PTVRP_CONT` (Bellman,
every pair scanned), and a layered control, `PTVRP_LAYERED_CONT`, that at `O(n²K)` only reached the
480 instances with `n <= 600`.

This file reports the rebuilt layered control over **all 3,150 instances**, and what it says about
revival, exactness and cost. Data: the nine root `sweep_<SOLVER>.csv` files, all measured on the
restored capacity ladder (`Instances 1` at Vidal's `Q` = 100 … 100,000, `Instances 2` and
`Instances 3` at 0.20× and 0.10× of it; 16 distinct capacities, 10 to 100,000). `seconds` in
those files is whole-process wall clock from `batch_run.py`, one solver per sweep, on the machine
noted in §11; the counters are exact.

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
| cost identical to `PTVRP_CONT` | **3,150 / 3,150** (3,060 feasible, 90 infeasible) |
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
| total | 425 revived arcs, 408 improving | 36 revived winners, 2 improving |
| instances | 238 | 33 |
| overlap | — | **all 33 are among Bellman's 238** |
| answer changed | 0 | 0 |

Every instance where a revived start wins a layer is one where Bellman also sees a revived arc. The
layered view never finds revival that Bellman misses. (`PTVRP_CONT_FIX` reports the same 425 arcs on
the same 238 instances, so the Bellman count is independent of which stop is in place.)

The reverse gap, 205 instances, has two possible reasons, and this run does not separate them:

- **Different pointers.** `PTVRP_LAYERED`'s window works per layer on `At[i]` alone. It is not the
  same pointer as Bellman's forward `break`, and it can re-admit a start in a later column
  (`revival.md` §5, `ce_infeasible`).
- **Not the winner.** The counter here fires only when the revived start is the *best feasible*
  start of its layer. A revived start that is feasible but beaten by a cheaper one in the same
  layer is not counted.

### 4.2 Where it happens

| size | instances | revived winners | improving |
|---|---|---|---|
| n < 500 | 1,380 | 0 | 0 |
| 500 – 2,000 | 780 | 8 | 0 |
| 2,000 – 10,000 | 600 | 5 | 0 |
| n >= 10,000 | 390 | **23** | **2** |

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
| `eff_K` = layer iterations / n, `PTVRP_LAYERED_CONT` (3,150 rows) | **13.3** | 535 | 90,127 |
| `eff_K`, `PTVRP_LAYERED` (with frontier; 3,060 feasible rows) | **3.8** | 8.1 | 126 |
| `eff_K / K_full` | **0.511** | | |

Run time follows it closely: **seconds ∝ (n · K_full)^0.93, r² = 0.984** (556 runs over 0.05 s). Cost
per layer iteration on those runs is **44 ns** median (p10 30, p90 56), whole-process wall clock on
the machine of §11. That includes the `trips()` division and the infeasible skips.

### 5.2 The horizon bound, not the layering, is what makes `PTVRP_LAYERED` fast

`PTVRP_LAYERED`'s `eff_K` of 3.8 comes from two things at once:

- the layer partition, which is kept here;
- the horizon cutting the layer loop at `trips(firstFeasible, j)`, which is gone here.

Removing the frontier costs **3.5× in `eff_K` at the median** (13.3 against 3.8; the per-instance
median of the ratio is 2.5×). On the ladder the gap is widest at the bottom rungs — 35.1 against
5.9 on `Instances 3` — and closes toward 1 at the top, where `K_full` is itself a handful. The
partition alone does not bound the layer count. The horizon does.

### 5.3 `O(n · K_full)` against Bellman's `O(n²)`

`K_full > n` whenever mean demand exceeds `Q`. That is true on **315 of 3,150** instances: all
105 at Q = 10 and all 210 at Q = 20, and none above. On those, `n · K_full > n²`, and the layered
control does more work than the plain Bellman control.

Median time ratio `PTVRP_LAYERED_CONT / PTVRP_CONT`, instances with n >= 3,000 (all rows,
infeasible included):

| Q | 10 | 20 | 40 | 50 | 100 | 200 | 400 | 500 |
|---|---|---|---|---|---|---|---|---|
| ratio | **7.66** | **5.25** | **3.10** | **2.55** | 1.28 | 0.67 | 0.36 | 0.29 |
| layered faster | 0/29 | 0/58 | 0/29 | 0/29 | 3/87 | 87/87 | 29/29 | 58/58 |

| Q | 1,000 | 2,000 | 4,000 | 5,000 | 10,000 | 20,000 | 50,000 | 100,000 |
|---|---|---|---|---|---|---|---|---|
| ratio | 0.16 | 0.10 | 0.06 | 0.05 | 0.04 | 0.03 | 0.02 | 0.02 |
| layered faster | 87/87 | 87/87 | 29/29 | 58/58 | 87/87 | 58/58 | 29/29 | 29/29 |

**Crossover between Q = 100 and Q = 200.** That is the same place as the `PTVRP_LAYERED` vs
`PTVRP` crossover in NOTES §5.5 (Q ≈ 100), and it follows from the same `K ∝ 1/Q` relation. Above
it the layered control is up to 50× faster than Bellman; at `Q <= 50` it is 2.5–7.7× slower.

Total solver time:

| folder | Q | `PTVRP_LAYERED_CONT` | `PTVRP_CONT` | `PTVRP_LAYERED` |
|---|---|---|---|---|
| Instances 1 | 100 – 100,000 | 112 s | 432 s | 7 s |
| Instances 2 | 20 – 20,000 | 470 s | 425 s | 9 s |
| Instances 3 | 10 – 10,000 | 759 s | 424 s | 9 s |
| **all** | | **1,341 s** | 1,280 s | 25 s |

The two `O(n²)`-class controls now cost about the same in total: the ladder's upper rungs make
`K_full` small on most files, so the unbounded layered control is no longer dominated by the flat
`Q = 10` and `Q = 20` folders. The slowest single run is `Instances 3/ch71009_01`: n = 71,008,
K_full = 180,818, 142.6 s, NO SOLUTION.

### 5.4 Infeasible instances are the expensive ones

The 90 NO SOLUTION instances take **512 s of the 1,341 s (38%)**, 5.7 s each against 0.27 s for a
feasible one. `PTVRP_LAYERED` abandons a column the moment its frontier reaches `j`. The continuous
control cannot know that nothing will revive, so it keeps visiting every layer to the end of the
tour.

This is the honest cost of not breaking. On an instance where nothing is feasible past some point,
**no-break means no shortcut**.

### 5.5 The price of stepping over instead of popping

| | |
|---|---|
| infeasible skips, total | 19.1 × 10⁹, against 38.8 × 10⁹ layer iterations |
| skips per layer iteration | median **0.78**, mean 1.50, max **432** |

The extremes are all in `Instances 1` at `Q >= 50,000` with `K_full <= 4`: `ca4663_10` (432),
`ca4663_09` (197), `kz9976_10` (149), `rl5934_10` (103), `d15112_10` (98).

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

All 3,150 instances against `PTVRP_CONT` (`sweep_PTVRP_LAYERED_CONT_FIX.csv`): **3,060 identical,
90 both-infeasible, 0 disagreements, 0 `UNSAFE POPS`.**

Median `eff_K` (`CONT` and `FIX` over all 1,050 rows of a set; `LAYERED` over its feasible rows,
since it prints no diagnostics on an infeasible instance):

| set | `CONT` (no bound) | **`FIX` (sound bound)** | `LAYERED` (unsound) |
|---|---|---|---|
| `Instances 1` | 4.0 | **2.0** | 1.9 |
| `Instances 2` | 17.8 | **4.9** | 4.5 |
| `Instances 3` | 35.1 | **7.1** | 5.9 |

On `Instances 3` the sound bound cuts the layer count to **20%** of the unbounded version, landing
within 20% of what the unsound frontier achieved; on `Instances 1` the two frontiers are within 3%.

### 6.2 Bellman gets the same rule, in one line

`PTVRP_CONT_FIX` (`Program/Split_Bellman_PTVRP_cont_fix.{h,cpp}`). The Bellman loop's running `time`
already *is* the path out — the return leg only enters `tau_ij` — so:

```cpp
if (pathOut * m > myData->horizon + 1.e-9)
    break ;
```

Its own counters give the price in arcs. Twelve largest instances per set, total arcs scanned over
total starts, against where the unsound stop would have ended:

| set | sound | unsound | overhead | no stop (`n/2`) |
|---|---|---|---|---|
| `Instances 1` | 518.2 | 463.4 | 1.12× | 35,504 |
| `Instances 2` | 240.7 | 205.5 | 1.17× | 35,504 |
| `Instances 3` | 167.6 | 138.6 | 1.21× | 35,504 |

Over all 3,150 instances the sound stop scans 5,174 M arcs against 4,797 M, 1.08×. By capacity the
median overhead per instance runs from 1.40× at `Q = 10` (13 against 9 arcs per start) down to
1.00× from `Q = 10,000` up, where the horizon ends every scan before capacity does.

### 6.3 What soundness actually costs

All six solvers, one machine, all 3,150 instances (`seconds` summed over each `sweep_<SOLVER>.csv`):

| solver | total | |
|---|---|---|
| `PTVRP_LAYERED` | 24.9 s | unsound frontier |
| **`PTVRP_LAYERED_CONT_FIX`** | **27.7 s** | **exact** |
| `PTVRP` | 70.4 s | unsound early stop |
| **`PTVRP_CONT_FIX`** | **83.6 s** | **exact** |
| `PTVRP_CONT` | 1,280.1 s | no stop — the oracle |
| `PTVRP_LAYERED_CONT` | 1,340.8 s | unbounded control |

**1.11× for the layered decoder, 1.19× for Bellman.** The Bellman fix is 15× faster than the oracle
and the layered fix 46×. The unbounded layered control is 48× slower than its bounded version.

This supersedes §5.2's "the horizon bound is what makes `PTVRP_LAYERED` fast, and it is the unsound
part". The correct statement is: *the frontier is worth 3.5× in `eff_K` at the median on the ladder
(48× in wall clock), and a sound frontier recovers it for 11%.*

### 6.4 The sound frontier against the unsound one, per size and per capacity

The same-process head-to-head that this section first reported (`sweep_layered_vs_contfix.csv`) was
measured on the pre-ladder instances and is no longer in the repository. What follows compares the
two separate root sweeps, `sweep_PTVRP_LAYERED_CONT_FIX.csv` and `sweep_PTVRP_LAYERED.csv`, instance
by instance.

| | result |
|---|---|
| identical cost | **3,150 / 3,150** (3,060 feasible, 90 infeasible) |
| identical template count and `max m` | **3,150 / 3,150** |
| `PTVRP_LAYERED_CONT_FIX` vs the `PTVRP_CONT` oracle | **3,150 / 3,150** identical |
| `UNSAFE POPS` | 0 everywhere |

**What soundness costs: 1.11× overall** (27.7 s against 24.9 s). The ratio grows with instance
size, because at small `n` both runs are a few milliseconds and process startup dominates:

| size | `..._CONT_FIX` | `PTVRP_LAYERED` | ratio |
|---|---|---|---|
| n < 500 (1,380) | 3.5 s | 3.5 s | 1.00× |
| 500 – 2,000 (780) | 3.3 s | 3.2 s | 1.06× |
| 2,000 – 10,000 (600) | 7.1 s | 6.3 s | 1.13× |
| n >= 10,000 (390) | 13.8 s | 12.0 s | **1.15×** |

Per-instance medians at `n >= 3,000` are 1.21× at `Q = 10`, 1.21× at `Q = 20`, 1.23× at `Q = 40`
and 1.19× at `Q = 100`, falling through 1.16× at `Q = 200` to 1.04–1.08× from `Q = 4,000` up. So
**1.2× is the right figure for the low-capacity instances where the layers do real work**, and the
1.11× total is diluted both by the 2,160 instances that finish in milliseconds and by the upper
rungs, where the layer count is near 1 for either frontier. Quote the per-size, per-capacity
number, not the total.

The source of the difference is unchanged from §6.1 — the sound frontier bounds the layer count
slightly less tightly (both rows over the 3,060 feasible instances, the only ones on which
`PTVRP_LAYERED` prints diagnostics):

| | median `eff_K` | mean | max | median `K_allocated` |
|---|---|---|---|---|
| `PTVRP_LAYERED_CONT_FIX` | **4.12** | 9.50 | 133 | 6 |
| `PTVRP_LAYERED` | **3.78** | 8.09 | 126 | 5 |

Counters on this run: revived layer winners on **39** instances (42 winners), 2 improving a label,
0 answers changed; `INFEASIBLE SKIPS` 81.7 M across 2,178 instances — more than two orders of
magnitude below the 19.1 × 10⁹ of the unbounded control in §5.5, which is the bound doing its work.
The 39 instances against §4.1's 33 for `PTVRP_LAYERED_CONT` is a small unexplained difference: the
two solvers replay the frontier under different layer bounds, so they do not have to agree, but it
has not been checked. All 39 are among Bellman's 238.

The slowest single run is 0.19 s (`Instances 3/ch71009_03`, n = 71,008), against 142.6 s for the
unbounded control.

---

## 7. Summary

1. **Correct.** Right on all four counterexamples. Identical to the exact `PTVRP_CONT` on all 3,150
   instances. With zero unsafe pops everywhere, it is exact by construction **on this benchmark**
   (§3) — a conditional statement, and §8 is where the condition fails.
2. **Revival is real but never decisive here.** 33 instances have a layer won by a revived start,
   2 of those improve a label, and 0 change an answer. All 33 are inside Bellman's 238.
3. **Without a bound, cost is `Θ(n · K_full)`, measured.** Exponent 0.93, 44 ns per layer visit on
   the machine of §11. Faster than Bellman's `O(n²)` control for Q >= 200, slower for Q <= 100;
   `K_full` exceeds `n` only at Q <= 20.
4. **The frontier is worth 3.5× in `eff_K` at the median, 48× in wall clock — and it need not be
   unsound.** Pruning on the path out instead of the full route restores the bound with no
   assumption about the instance (§6). Median `eff_K` on `Instances 3` goes 35.1 -> 7.1, against 5.9
   for the unsound frontier.
5. **Soundness costs 1.11× (layered) and 1.19× (Bellman)**, not the 48× the unbounded control needs.
   The fixed solvers remain 46× and 15× faster than the oracle that assumes nothing (§6.3).
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

> The full formal treatment of this defect -- the dominance condition, why capacity-only Split is
> immune to it unconditionally, the Pareto structure the deque degenerates into, and a four-vendor
> duration-constrained CVRP with no multiplier where it fails completely -- is in **`pop.md`**.


The eviction discards an older start when a newer one has a key at least as small -- a comparison on
**cost only**. Feasibility has its own `j`-independent ranking, and it is `At[i]`: feasibility at
column `j` in layer `k` is `At[i] + Bt[j] <= T_H/k`, and comparing two starts at the SAME `j` cancels
`Bt[j]` exactly as it cancels in the cost. So

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
| 3,150 benchmark instances vs `PTVRP_CONT` | 3,060 identical, 90 both-infeasible, 0 differ | **identical** |
| `BLOCKED POPS` / `UNSORTED QUERIES` on the benchmark | — | **0 / 0** |
| total, median `eff_K` (3,060 feasible) | 27.7 s, 4.12 | 27.7 s, 4.12 |

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

- **Split the 205-instance gap (§4.1).** Count *every* feasible start behind the replayed frontier,
  not only layer winners. That separates "different pointer" from "not the winner". **Still open.**
- **Look for an instance with `UNSAFE POPS > 0` — ANSWERED (§8, §8.1), and then patched (§8.2).**
  `structural_violation.gt` fires the counter on 60 of 60 instances; `ce_unsafe_pop_5v.gt` is five
  vendors where the back-pop turns an optimum of 658 into `NO SOLUTION`. So the answer is that the
  back-pop can genuinely lose the answer, not merely that it is unprovable. `PTVRP_LAYERED_SAFE`
  closes it with a joint-dominance eviction that is exact and, on this benchmark, free.
- **A sound frontier would recover most of what the unsound one gives — ANSWERED (§6).** Not by the
  suffix sum `D[j]` that `revival.md` §9 first proposed, but by pruning on the path out, which needs
  nothing precomputed and applies unchanged to both decoders. Measured at 1.11× (layered) and 1.19×
  (Bellman) in wall clock, 1.08× in arcs for Bellman.
- **Infeasible instances (§5.4) — largely answered.** The path-out bound is exactly such a lower
  bound on time, and it fires on instances where nothing can fit: the 90 infeasible instances cost
  the fixed layered solvers 14 ms each against 9 ms for a feasible one, instead of the 5.7 s an
  unbounded scan spends on them.
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

**Timings in the current revision (§5, §6) come from the root `sweep_*.csv` files, which were run
after the capacity ladder was restored on a machine other than the one below; the commit messages
record it as roughly 1.5× slower on identical single runs.** They are whole-process wall clock from
`batch_run.py`, one solver per sweep, and are meant for ratios between solvers on the same
instances, not for absolute figures. The recording-free build in `Program/pure/`, which prints its
own solve time, has not yet been swept on the ladder; when it is, on the machine below, its
`solve_seconds` supersede these.

Every timing in the earlier revision of this file came from one machine:

| | |
|---|---|
| CPU | **Intel Core i7-9700K @ 3.60 GHz**, 8 cores / 8 threads (no SMT) |
| RAM | 16 GB host; ~7 GB visible to the VM |
| OS | Windows 11 Pro 22621, WSL2 Ubuntu, kernel 6.18.33.2-microsoft-standard-WSL2 |
| compiler | g++ (Ubuntu) 15.2.0, `-O3` |
| harness | `batch_run.py`, one process per instance folder, three folders in parallel |

Three parallel processes on 8 threads leave the cores oversubscribed only by memory bandwidth, not by
count, but the runs are not isolated: an instance timed alongside two others is slower than the same
instance timed alone. Single-instance measurements quoted in that revision (its 21 ns per layer
visit in §5.1, its slowest run in §6.4) were taken with nothing else running.

Timing note, carried forward: every conclusion in this file rests on ratios of 5× or more, on
per-instance counters, or on cost agreement — not on small timing differences. The soundness ratios
of §6.3 and §6.4 (1.1–1.2×) are the one place a small timing difference is quoted, and there the
arc and layer counters of §6.1–§6.2 carry the claim.

---

## 12. Note: earlier figures (pre-ladder instance set)

This file was re-measured on 2026-09-17 after `rescale_instances.py` restored Vidal's capacity
ladder to `Instances 2` and `Instances 3` (scaled by 0.20 and 0.10). Before that, those folders
were flat at `Q = 20` and `Q = 10` for all ten suffixes, `Instances 1` already carried the ladder,
and 2,829 of 3,150 instances were feasible (321 not). `Instances 1` is byte-identical on both
layouts and its rows did not move. What the earlier text said, for the record:

- **§4.** 177 revived arcs (172 improving) on 103 instances for Bellman; 18 revived layer winners
  on 18 instances (1 / 5 / 3 / 9 by size bucket), 2 improving, all inside Bellman's 103; reverse
  gap 85 instances.
- **§5.** `eff_K` of the unbounded control median 188, mean 2,692, max 90,602 against 15 / 21.5 /
  127 with the frontier; `eff_K / K_full` 0.503; seconds ∝ `(n·K_full)^0.95`, r² 0.979 over 922
  runs; 21 ns per layer iteration (p10 15, p90 30); the frontier worth 12× at the median.
  `K_full > n` on 2,099 instances (all 1,050 at `Q = 10`, 1,049 at `Q = 20`). Time ratio of the
  layered control to Bellman 7.35 / 5.02 / 1.32 / 0.70 / 0.33 / 0.19 / 0.12 / 0.07 / 0.05–0.06 at
  `Q` = 10 / 20 / 100 / 200 / 500 / 1,000 / 2,000 / 5,000 / ≥10,000, layered faster 0/290, 0/290,
  3/29 then 29/29; totals 89 / 1,599 / 2,354 s = 4,042 s against 314 / 324 / 295 = 934 s for
  Bellman and 8 / 11 / 11 = 31 s for `PTVRP_LAYERED`; slowest run `Instances 3/ch71009_07`,
  `K_full` 180,802, 117 s. The 321 infeasible instances took 2,519 s of 4,042 (62%). Infeasible
  skips 33.0 × 10⁹ against 195.3 × 10⁹ iterations, median 0.36 per iteration, mean 1.16.
- **§6.** Median `eff_K` 499.7 / 24.2 / 18.5 on `Instances 2` and 998.9 / 30.2 / 29.2 on
  `Instances 3` for no bound / sound bound / unsound, read as "3.0% of the unbounded version,
  within 3% of the unsound frontier". Arcs per start over the twelve largest instances 11.5 / 6.6
  (1.73×) on `Instances 2` and 6.5 / 3.8 (1.72×) on `Instances 3`. Totals 23.7 / 28.0 / 34.0 /
  38.4 / 781.8 / 4,041.5 s for `PTVRP_LAYERED`, `PTVRP_LAYERED_CONT_FIX`, `PTVRP`,
  `PTVRP_CONT_FIX`, `PTVRP_CONT`, `PTVRP_LAYERED_CONT`, read as **1.18× layered and 1.13×
  Bellman**, both ~20× faster than the oracle, and "a sound frontier recovers the 12× for 18%".
  The same-process head-to-head of §6.4 (`sweep_layered_vs_contfix.csv`, 6,300 rows, 34 s) gave
  1.08× overall, 0.96 / 1.02 / 1.10 / 1.16× by size bucket, 1.18–1.23× per instance at `Q <= 100`;
  median `eff_K` 17.3 (mean 25.5, max 135, `K_allocated` 33) against 15.0 (21.5, 127, 27);
  revived winners on 20 instances; infeasible skips 60.9 M across 2,653 instances; slowest run
  0.13 s.
- **§7, §8.2.** The same figures restated: 18 revived winners, exponent 0.95 and 21 ns, the
  frontier worth 12×, soundness 1.18× / 1.13× "not the 169× the unbounded control needed", and
  28.0 s / median `eff_K` 17.3 for both sound layered solvers.

Readings that changed, as against readings that merely rescaled:

1. **The two soundness costs swapped order.** Bellman now pays more (1.19×) than the layered
   decoder (1.11×). The upper rungs lengthen Bellman's per-start scan (median 320 arcs per start
   over all instances at `Q >= 20,000`, 1,900 on those with `n >= 3,000` at `Q = 100,000`) while the
   layered decoder's layer count falls toward 1 there, so the layered decoder has less to lose from
   a looser frontier.
2. **The sound frontier is looser than the unsound one by 20% on `Instances 3`, not 3%.** The old
   3% was measured at `Q = 10` only, where both frontiers allocate ~30 layers; on the ladder the
   relative gap is larger because the counts are small (7.1 against 5.9).
3. **The unbounded layered control is no longer the slow one.** It took 4.3× the oracle's time on
   the flat folders and takes 1.05× on the ladder, because `K_full > n` now holds on 315 instances
   rather than 2,099. The `Θ(n · K_full)` law itself is unchanged (exponent 0.93 against 0.95).
4. **Revival is seen more often, not less.** 238 instances against 103, and 29% of the largest
   instances against 13%. The ladder puts every geometry at high capacity, where `rho < 1` and
   the multiplier does not step at every vendor, which is the regime §5.9 of `NOTES.md` predicts
   revival lives in. The per-crossing rate either side of `rho = 1` moved from 38.2 / 1.1 to
   35.9 / 0.8.

Nothing in the correctness story moved: 0 disagreements, 0 unsafe pops, 0 blocked pops and 0
answers changed by revival on both layouts.
