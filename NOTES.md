# Splitting a Giant Tour When Route Cost Is Multiplicative

Working notes on the Periodic-Template VRP (PT-VRP): why Vidal's linear-time Split does not
apply to it, what does apply, what it costs, and what is still open.

These are research notes, not a finished paper. Every number below was produced by the code in
this repository and is reproducible by the commands in §9. Where an earlier claim of ours turned
out to be wrong, the correction is recorded in place rather than quietly removed (§10).

---

## 1. Background and problem statement

### 1.1 Split

Split is the decoder at the centre of route-first cluster-second heuristics and of HGS. It takes a
*giant tour* — a permutation `v_1 … v_n` of the customers with no vehicle assignment — and cuts it
into routes optimally, subject to the tour order being preserved. It is a shortest-path problem on
an auxiliary DAG with `n+1` nodes, where arc `(i, j)` represents the route serving `v_{i+1} … v_j`.

The naive dynamic program is

```
p[0] = 0
p[j] = min over feasible i < j of  p[i] + c(i, j)
```

which costs `O(n·B)`, where `B` is the average number of predecessors scanned before the capacity
or duration limit forces a break.

### 1.2 Vidal's linear Split

Vidal (2016, *Technical note: Split algorithm in O(n) for the capacitated vehicle routing problem*,
arXiv:1508.02759) observes that for the classic CVRP the arc cost is **separable**:

```
c(i, j) = d(v_1... depot legs and cumulative distance)  =  A[i] + B[j]
```

Consequently

> **Property 2.** For any two predecessors `i1`, `i2` and any `j` reachable from both,
> `c(i1, j) − c(i2, j) = A[i1] − A[i2] = K`, a constant independent of `j`.

Property 2 is the whole licence for the linear algorithm. It says the *ranking* of two candidate
predecessors never inverts as `j` advances. A predecessor that is dominated once is dominated
forever, so it can be discarded permanently, and the candidate set can be maintained as a monotone
deque with amortised `O(1)` work per column. Total: `O(n)`.

Note what this does *not* say. It does not say the deque is a heuristic that usually works. It says
that under separability, discarding is provably safe. Remove separability and the justification is
gone entirely — not weakened.

### 1.3 PT-VRP

In the PT-VRP a route is a **template** `sigma` — an ordered set of vendors — that is executed
repeatedly over a planning horizon. The number of executions is set by demand and capacity:

```
m(sigma) = ceil( q(sigma) / Q )                  trips required
cost     = sum over templates of  d(sigma) · m(sigma)
feasible iff  tau(sigma) · m(sigma) <= T_H       for every template
```

where `d(sigma)` is the template's travel distance, `tau(sigma)` its duration, `Q` the vehicle
capacity and `T_H` the horizon. A vendor may have `q_i > Q`; this is legal and simply forces
`m >= 2` for any template containing it. (Service time is modelled but fixed at zero — see §4.3.)

Two differences from the CVRP matter:

1. Capacity is no longer a **feasibility** constraint. It is a **cost multiplier**. Exceeding `Q`
   does not forbid a template; it makes it more expensive.
2. The binding feasibility constraint is the horizon, and it binds on the *product* `tau · m`, so
   it tightens non-linearly as a template grows.

---

## 2. Why linearity breaks

Under PT-VRP the arc cost is

```
c(i, j) = m(i, j) · ( A[i] + B[j] )
```

with `m(i, j) = max(1, ceil((L[j] − L[i]) / Q))` for cumulative load `L`. The multiplier depends on
**both** endpoints. Therefore

```
c(i1, j) − c(i2, j) = m(i1,j)·(A[i1] + B[j]) − m(i2,j)·(A[i2] + B[j])
```

which is a function of `j` whenever `m(i1,j) != m(i2,j)`. Property 2 fails.

Concretely: `m` is a step function of `j`, and the steps occur at *different* `j` for different `i`.
A predecessor that looks worse now becomes better the moment its rival crosses a capacity boundary
and its multiplier jumps. The failure is not asymptotic or measure-zero — it is generic.

The distinction worth stating precisely, because it is the mechanism of the whole repair:

- the multiplier **multiplies** rather than **adds**;
- an additive endpoint-dependent term preserves Property 2, a multiplicative one does not;
- but `m` is **integer-valued**, so conditioning on it partitions the predecessors into finitely
  many classes inside each of which the multiplier is constant, hence additive, hence separable.

That last point is §3.3.

---

## 3. Three decoders

All three are in `Program/`, selected by `-solver`. All three share the same instance reader,
the same feasibility test and the same reporting, so their outputs are directly comparable.

### 3.1 `PTVRP` — exact reference DP

`Split_Bellman_PTVRP.{h,cpp}`. The definition, computed. For every start `i`, scan forward
accumulating load, distance and time; compute `m`; break when `tau·m > T_H`. Cost `O(n·B)`.
This is the ground truth every other answer is checked against.

### 3.2 `PTVRP_LINEAR` — the deque applied anyway (deliberately wrong)

`Split_Linear_PTVRP.{h,cpp}`. This is Vidal's `Split_Linear` with exactly two edits: the propagate
step scales by `m`, and the front-eviction test uses the horizon. The `dominates` and
`dominatesRight` predicates are byte-identical to the original.

It exists as a **measured control**. Without it, "the deque is invalid here" is an assertion about a
proof. With it, the assertion becomes a number (§5.2). It should never be used to produce results.

### 3.3 `PTVRP_LAYERED` — the repair

`Split_Layered_PTVRP.{h,cpp}`, ported from a Rust layered-K decoder. One monotone deque per trip
count `k`. Within layer `k` the multiplier is a fixed constant, so

```
cost_k(i, j) = k · d(i, j) = k·A[i] + k·B[j]
```

is separable again, `cost_k(i1,j) − cost_k(i2,j) = k·(A[i1] − A[i2])` is independent of `j`, and
Property 2 holds *inside the layer*. The answer is the best candidate over all layers.

The cost is `O(n·K)` rather than `O(n²K)` because **the layers partition the predecessors rather
than duplicating them**: for a given `j`, the starts requiring exactly `k` trips form the contiguous
range `[firstLoadLE[k], firstLoadLE[k−1])`, and each of those pointers only ever moves right as `j`
grows. Each index therefore enters each layer's deque at most once.

The worked 5-vendor example makes the partition visible. At `p[5]` the layers take starts
`[4,5)`, `[2,4)`, `[1,2)`, `[0,1)` — four disjoint windows tiling `[0,5)`.

**Monotonicity assumption.** The two-pointer windows require `At[]` non-increasing and `Bt[]`
non-decreasing, which follows from the triangle inequality on travel times. TSPLIB rounds each
distance independently and so violates it by up to one unit. Rather than refuse to run, the solver
counts violations and reports them; §5.1 measures whether they ever change an answer.

---

## 4. Experimental setup

### 4.1 Instances

| set | files | n | capacity |
|---|---|---|---|
| `Instances/Instances 1` | 1,050 | 28 – 71,008 | 100 … 100,000 (10 values, suffix `_01.._10`) |
| `Instances/Instances 2` | 1,050 | same points | Q = 20 |
| `Instances/Instances 3` | 1,050 | same points | Q = 10 |

3,150 files, horizon `MAX_ROUTE = 86400`, distances doubling as travel times. In sets 2 and 3 many
vendors have `q_i > Q`, so classic Split cannot solve them at all while PT-VRP can — those sets are
the reason the multiplier exists.

### 4.2 Two defects found before any result was trustworthy

These are worth recording because each would have silently corrupted everything downstream.

**(a) None of the 3,150 instance files were readable.** The original parser assumed a fixed header
order and a fixed column count. The real files vary in both. The fix was to rewrite the reader to
slurp all tokens and do a token-driven header scan that accepts `DIMENSION`, `CAPACITY`,
`MAX_ROUTE`/`HORIZON`, `SPEED` in any order and skips unknown keys (`Pb_Data.cpp`). The principle we
settled on and then followed: **fix the program to match the instances, not the instances to match
the parser.**

**(b) The makefile had no header prerequisites.** No `Split_*.o` declared `Split.h` or `Pb_Data.h`,
and `main.o`/`commandline.o` declared none at all. Editing a constant in a header rebuilt nothing;
the binary kept running the old value while appearing to have been rebuilt. This is the more
dangerous of the two, because it produces plausible wrong numbers rather than an error.

### 4.3 Service time

Fixed at `PTVRP_SERVICE_TIME = 0.0` in `Program/Pb_Data.h`, a single named constant with a comment,
because the instance files carry none and the value is expected to be constant across an instance
set. Scale note recorded at the constant: on these sets any value below roughly 3,600 leaves the
optimum unchanged and values above roughly 40,000 make the instances infeasible, so the horizon only
reacts to service times of that order.

### 4.4 Validation harness

A separate exhaustive brute-force splitter in Python (scratch, not committed) enumerates all
partitions for small `n` and compares against each solver. Used for correctness of the DP itself
before any of the scale results below.

---

## 5. Results

### 5.1 Exactness

Full sweep, all three solvers over all 3,150 files (`full_sweep.csv`, 9,450 rows).

- **2,829** instances are feasible under the horizon; 321 are not, in agreement between the exact
  and the layered solver.
- **Layered = exact on 2,829 / 2,829.** No disagreement anywhere, including on the instances where
  the monotonicity warning fires. The rounding violations are real but never changed an answer.

### 5.2 The cost of using the deque anyway

- **Deque = exact on 389 / 2,829 (13.8%).**
- Cost excess over the optimum: **median 27.0%, mean 85.8%, maximum 1,080%.**
- The deque never reports infeasibility (0 / 3,150), including on the 321 instances that genuinely
  have no feasible split. It is not merely inaccurate; it is unsound on feasibility too.

A sharper characterisation of when it happens to be right. Restrict to the 831 instances whose
*exact optimum* uses only unit multipliers (`max m(sigma) = 1`), i.e. the PT-VRP optimum coincides
with a plain CVRP optimum:

| deque's own `max m` on its answer | 1 | 2 | 3 | 4 | 5 | 6+ |
|---|---|---|---|---|---|---|
| deque correct | 389 | 0 | 0 | 0 | 0 | 0 |
| deque wrong | 14 | 110 | 85 | 64 | 39 | 130 |

So `max m = 1` at the optimum is **necessary but not sufficient**. In 442 of those 831 cases the
deque's own path wanders into multi-trip templates and pays for them. In a further 14 cases both
paths are all-unit and the deque is *still* wrong — the scaled propagate and horizon eviction can
discard a predecessor that would have led to a cheaper all-unit path. The corruption is in the
pruning, not only in the final answer.

### 5.3 What the layered repair actually costs

The solver reports several quantities all called "K", which are not the same thing. From
`a280_01.gt` at Q=2000:

```
K FULL TOUR          : 72     ceil(total demand / Q) -- the naive ceiling
LAYERS ALLOCATED (K) : 48     deepest layer reached in any column, after the horizon bound
MAX LAYER USED       : 3      deepest layer that ever improved a label
MAX LAYER ON PATH    : 2      largest m(sigma) in the answer
LAYER ITERATIONS     : 8981   effective K = 32.19 per column   <- the O(n·K) work
```

`eff_K = layer_iterations / n` is the quantity that belongs in a complexity claim; it is the direct
analogue of Vidal's `B`. Across the 2,829 feasible instances:

| quantity | median | mean | max |
|---|---|---|---|
| `K_full_tour` (naive) | 287 | 3,150 | 47,663 |
| `K_allocated` (after horizon bound) | 27 | 38 | 198 |
| **`eff_K`** (work done) | **15.0** | **21.5** | **127** |
| `max_layer_used` (work that mattered) | 9 | 9.8 | 60 |

The naive ceiling overstates the real work by a factor of ~19 at the median. The horizon bound does
most of that reduction; the two-pointer partition does the rest.

### 5.4 A closed form for `eff_K`

`eff_K` is not a free parameter. Let `rho = qbar/Q` be the mean demand per vendor in units of
capacity, `c` the mean inter-vendor arc, `dbar_0` the mean depot leg. Let `L` be the number of
vendors a template can hold before the horizon binds. A template of `L` vendors has duration
`tau ≈ 2·dbar_0 + (L−1)·c` and multiplier `m ≈ L·rho`, and the horizon binds when `tau·m = T_H`:

```
(c·rho) L² + ((2·dbar_0 − c)·rho) L − T_H = 0
```

Take the positive root and predict

```
    eff_K ≈ max(1, L·rho)
```

Validated against all 2,829 measured values:

- **r² = 0.934** in log-log, 0.729 on the raw scale
- **median predicted/observed = 1.025**, p10 0.913, p90 1.465

Two consequences follow directly and both are worth stating, because they say the two algorithms
cannot both be slow on the same instance:

```
    K / B  =  qbar / Q                (B ≈ L, K ≈ L·rho)
    B · K  =  L²·rho  ≈  T_H / c      measured median ratio 0.50, so Θ(T_H/c)
    min(B, K)  =  O( sqrt(T_H / c) )
```

A template can spend the horizon on *length* or on *trips*, not both. Bellman pays for length,
layered pays for trips, and their product is pinned by the horizon. Dropping the depot legs from the
derivation (setting `dbar_0 = 0`) over-predicts by a median factor of 1.71, so the depot terms are
not a refinement — they carry most of the accuracy.

### 5.5 Where the crossover is

Wall-clock, layered vs exact Bellman. Restricted to `n >= 3,000` because at small `n` the runs are
2–4 ms and process startup dominates.

| Q | instances | median `eff_K` | median `t_bellman` (s) | median `t_layered` (s) | **median speedup** |
|---|---|---|---|---|---|
| 10 | 150 | 28.2 | 0.0149 | 0.0214 | **0.70×** |
| 20 | 230 | 17.9 | 0.0134 | 0.0202 | **0.70×** |
| 100 | 27 | 13.0 | 0.0170 | 0.0170 | **1.03×** |
| 200 | 27 | 11.0 | 0.0164 | 0.0145 | **1.23×** |
| 500 | 27 | 8.3 | 0.0172 | 0.0103 | **1.64×** |
| 1,000 | 27 | 6.2 | 0.0225 | 0.0096 | **2.40×** |
| 2,000 | 27 | 4.7 | 0.0256 | 0.0085 | **3.22×** |
| 5,000 | 27 | 3.2 | 0.0427 | 0.0075 | **4.98×** |
| 10,000 | 27 | 2.3 | 0.0545 | 0.0068 | **7.29×** |
| 20,000 | 27 | 1.8 | 0.0670 | 0.0068 | **9.33×** |
| 50,000 | 27 | 1.0 | 0.0852 | 0.0066 | **16.18×** |
| 100,000 | 27 | 1.0 | 0.1255 | 0.0064 | **18.28×** |

Crossover sits near **Q ≈ 100**. Below it the layered decoder loses — the per-layer bookkeeping is
not repaid when `eff_K` is 20–30. Above it, `eff_K` collapses toward 1 while Bellman's `B` climbs,
exactly as §5.4 predicts, and layered wins by an increasing margin.

The practical reading: the layered decoder is the right choice when capacity is generous relative
to demand. At `Q = 10` and `Q = 20` — where the multiplier is *most* active and the PT-VRP is most
distinct from the CVRP — the exact DP is simply faster. That is an uncomfortable result and it is
not an artefact; it follows from `B·K ≈ T_H/c`.

### 5.6 Where the load sits along the tour

`skew_instances.py` rewrites only the demand column, holding geometry, tour order, capacity, horizon
and **total** demand fixed, so `ceil(q_tot/Q)` does not move. Demands are Beta-shaped over normalised
tour position with `a = mu·s`, `b = (1−mu)·s`, giving a two-factor design:

- `mu` — demand centroid along the tour. 0.5 balanced, <0.5 head-loaded, >0.5 tail-loaded.
- `s` — concentration. Larger piles the load more tightly around `mu`; `s=2, mu=0.5` is flat.

`manifest.csv` logs the *measured* centroid, Gini and CV of every generated file; the positivity
floor pulls the nominal extremes inward, so regress on the measured statistic, not the knob.

Matrix of 5 centroids × 4 concentrations over 40 source instances; 34 are feasible in every cell.
Median `eff_K` relative to the flat reference `mu=0.5, s=2`:

**All 34 feasible instances**

| | mu=0.1 | mu=0.3 | mu=0.5 | mu=0.7 | mu=0.9 |
|---|---|---|---|---|---|
| s=2 | 1.00 | 1.02 | 1.00 | 0.87 | 0.66 |
| s=30 | 1.00 | 1.00 | 1.00 | 0.91 | 0.67 |

**Restricted to the 23 instances with `eff_K > 1` at the reference** — i.e. those where layering is
doing anything at all

| | mu=0.1 | mu=0.3 | mu=0.5 | mu=0.7 | mu=0.9 |
|---|---|---|---|---|---|
| s=2 | **1.20** | 1.15 | 1.00 | 0.82 | **0.57** |
| s=4 | 1.21 | 1.18 | 1.01 | 0.85 | 0.56 |
| s=10 | 1.27 | 1.17 | 1.03 | 0.83 | 0.60 |
| s=30 | 1.25 | 1.18 | 1.05 | 0.82 | 0.62 |

Findings:

1. **Position is the effect, and it is monotone and two-sided.** Tail-loading removes ~40% of
   `eff_K`; head-loading *adds* ~20–25%. The mechanism: the layer count needed at column `j` depends
   on the load accumulated before `j`. Put the heavy vendors last and most columns are processed at
   low `k`, with the deep layers reached only near the end. Put them first and every subsequent
   column pays.
2. **The unfiltered table hides half of it.** The 11 instances stuck at `eff_K = 1` cannot move and
   pin the left-hand columns to exactly 1.00. Filtering to the instances that can respond is what
   reveals the head-loading penalty. (This is the error corrected in §10.1.)
3. **Concentration is a weak second-order effect** — at most 0.07 across a 15× range of `s` — and its
   sign is not uniform: it amplifies the head-load penalty (1.20 → 1.25) and damps the tail-load
   benefit (0.57 → 0.62). Unevenness alone is not the mechanism; position is.
4. Skewing at low capacity removes feasibility from many instances, since concentrated load makes
   `tau·m` exceed the horizon locally.

---

## 6. A line of attack that did not survive contact

The §5.6 result suggests an obvious move: if tail-loading cuts `eff_K` by 40%, reorder the
chromosome so the heavy vendors come last. We pursued this and abandoned it. The reasoning is
recorded because the conclusion is not obvious in advance.

### 6.1 The arrangement is not free

The giant tour *is* the chromosome — the object HGS is searching over, and Split's job is to score
it. Reorder it before scoring and you are scoring a different candidate. Only reorderings that
provably preserve the answer are admissible, which is a far smaller set than "reorderings that
reduce `eff_K`."

### 6.2 Reversal: correct, but there is nothing to reverse

Reversing the giant tour is answer-preserving under symmetric distances. Measured: **0 cost changes
in 117 instances**, and `eff_K` drops to **0.27–0.45×** on the instances where the orientation rule
fires. A centroid-based decision rule (flip when the demand centroid is below ~0.45) picked the
better orientation **71 / 79** times.

It buys nothing on real data. Measured over 40 TSPLIB-derived instances, the demand centroid is
**median 0.5002, range 0.468 – 0.521**. These instances are balanced to three decimal places. A
transformation that exploits skew, applied to data with no skew, is worth zero. It also requires
symmetric distances, which is a modelling restriction one may not wish to assume.

### 6.3 Rotation: not answer-preserving, and not expressible

Rotation was proposed as the asymmetry-safe alternative, since it never traverses an arc backwards.
It is asymmetry-safe and it is not answer-safe.

**Rotating the tour to start at position `k` is exactly equivalent to forbidding any template from
spanning the gap between `v_{k−1}` and `v_k`.** It forces a cut. Different rotations force different
cuts and admit different partitions, so cost can change. Reversal preserves the solution set;
rotation restricts it. The two feel like the same class of move and are not.

Two qualifications, one in each direction:

- In rotation's favour: linear Split *already* forces an arbitrary cut, at the chromosome boundary.
  So this is a choice between arbitrary constraints, not between free and constrained. The honest
  version is `min` over all `n` rotations, which is the true cyclic optimum and costs `n×`.
- Against: the `.gt` format stores only `n−1` forward arcs. There is no closing arc `d(v_n, v_1)`.
  The giant tour in this data is a **path**, not a cycle. Rotation is not expressible without adding
  coordinates to the format.

### 6.4 The general objection

Every reordering trick attacks the **constant**, not the exponent. Halving `eff_K` from 15 to 7.5
buys 2×; the algorithm is still `O(n·K)`. That is worth having only if it is cheap, and each of
these costs either a correctness guarantee or a modelling restriction.

### 6.5 A related negative result on search difficulty

A separate question: does clustered demand make the giant-tour search easier or harder? Skew along
the tour is not an instance property (reordering changes it); the instance property is whether heavy
customers sit near each other in *space*. Fixing a point set and assigning the same demand multiset
either at random or clustered into one region, then measuring 2-opt descent under the PT-VRP
objective, gives 2-opt final/start cost **0.477 → 0.501**. Clustered demand makes the landscape
*flatter*, so rearranging buys less, not more. (`rearrange.py`, scratch.)

---

## 7. Open questions

1. **Is the PT-VRP cost matrix Monge within a layer?** This is the load-bearing question for §8.3.
   Within layer `k` the cost is `k·A[i] + k·B[j]`, which is trivially Monge; the difficulty is that
   the *layer membership boundary* moves with `j`, so the effective matrix over all predecessors is
   a patchwork. Whether the patchwork is totally monotone is not established either way.
2. **Is there a tighter a priori bound on `K` than the horizon frontier?** `K_allocated` (median 27)
   already improves on `ceil(q_tot/Q)` (median 287) by ~10×, but `eff_K` (median 15) and
   `max_layer_used` (median 9) show there is more slack. A bound computable before the sweep would
   let layers be allocated once rather than grown.
3. **Does the crossover at `Q ≈ 100` move under a different horizon?** `B·K ≈ T_H/c` says the
   trade-off is governed by `T_H/c`, so a shorter horizon should move the crossover. Untested —
   every instance here has `T_H = 86400`.
4. **Does the monotonicity violation ever matter?** Zero effect in 2,829 instances is strong evidence
   but not a proof. A constructed adversarial instance would settle whether the counter is a genuine
   safety net or permanently decorative.
5. **Does any of this survive inside HGS?** Every result here is on a static giant tour. Split inside
   HGS is called on tours that are themselves evolving, and the distribution of those tours is not
   the distribution of TSPLIB orders.

---

## 8. Next steps, in order

### 8.1 Skip layers that cannot contribute — engineering, ~1.7× median

The diagnostics say the work is not where the cost is: median `eff_K` is 15.0 while median
`max_layer_used` is 9. The ratio `eff_K / max_layer_used` has **median 1.65, mean 2.55, p90 5.9,
max 16.0**. On the README's example instance it is 7. Deep layers are allocated, visited and
contribute nothing.

The fix is bookkeeping: track which layers are live per column and skip the rest. Exactness is
preserved by construction (a layer that cannot improve any label cannot change the minimum), and it
is verifiable against the existing brute-force harness.

Note the ratio is a *lower* bound on an oracle's win, since `max_layer_used` is a global maximum and
the per-column useful depth is usually shallower. It is also not fully realisable, since liveness
must be detected rather than known. The realisable fraction is unknown until implemented — which is
the argument for implementing it first: it is the cheapest way to find out.

### 8.2 Incremental layer structure between adjacent columns — engineering, ~2–3×

Moving from column `j` to `j+1` shifts the layer boundaries by one vendor's demand. The structure is
currently rebuilt per column; it should be slid. This is the same amortised-pointer argument Vidal
uses inside a single deque, applied across layers. Constant factor, more intricate than §8.1.

### 8.3 Monge / K-link path machinery — research, could change the exponent

The only item here that is not a constant. There is a body of work on shortest paths in DAGs whose
cost matrices are Monge or totally monotone, including the *K-link path* problem, which is close to
what layering produces. Aggarwal, Schieber and Tokuyama is the reference, and **Vidal cites it in
his own conclusion** as the direction he did not take.

If the answer to open question 7.1 is yes, that machinery can beat a linear scan per layer and lower
the exponent rather than shaving the constant. If the answer is no, this line closes. It is the only
item that can fail outright, so it wants a proof attempt before any code.

**Recommended order: 8.1 now** (contained, measurable, verifiable), 8.2 if another constant is wanted,
8.3 as a separate question with a proof in front of it.

### 8.4 Repository debt

- README §"Skewing the load along the tour" still carries the superseded skew table and the claim
  that concentration barely registers. Superseded by §5.6 and §10.1.
- `full_sweep.csv` (9,450 rows) and `per_instance_K.csv` (2,829 rows) are currently gitignored.
  Decision pending: commit raw, commit a summary, or leave out and regenerate.
- The orientation rule of §6.2 is designed but not implemented. Given §6.2's conclusion, it should
  probably stay unimplemented.

---

## 9. Reproducing

```bash
cd Program && make

# the three decoders on the worked example (exact 80, deque 148, layered 80)
./split ../Instances/ptvrp_demo_5v.gt -solver PTVRP         -trace 1
./split ../Instances/ptvrp_demo_5v.gt -solver PTVRP_LINEAR  -trace 1
./split ../Instances/ptvrp_demo_5v.gt -solver PTVRP_LAYERED -trace 1

# the full sweep behind §5.1, §5.2, §5.3 (9,450 rows, ~20 min)
python3 batch_run.py --dir "Instances/Instances 1" --dir "Instances/Instances 2" \
    --dir "Instances/Instances 3" \
    --solver PTVRP PTVRP_LINEAR PTVRP_LAYERED --out full_sweep.csv

# the skew matrix behind §5.6
python3 skew_instances.py --src "Instances/Instances 1" --out Instances/skew --limit 40 \
    --mu 0.1 0.3 0.5 0.7 0.9 --conc 2 4 10 30
python3 batch_run.py $(for d in Instances/skew/mu*; do echo -n "--dir $d "; done) \
    --solver PTVRP_LAYERED --out skew_results.csv
```

---

## 10. Corrections

Recorded rather than removed, since the mechanism of each error is itself informative.

**10.1 "Skew only lowers `eff_K`; concentration doesn't matter."** Wrong in both halves, and from the
same cause: the 11 instances pinned at `eff_K = 1` cannot respond to reordering, and including them
dragged every median toward 1.00. Head-loading *raises* `eff_K` (to 1.20–1.27), and concentration is
a real if weak effect with opposite signs on the two sides. Corrected table in §5.6. The general
lesson: when a metric has a floor, instances sitting on the floor are not evidence of no effect.

**10.2 "Skipping empty layers is worth ~7×."** That figure came from a single instance (the README
example, `eff_K = 104` vs `max_layer_used = 15`). The population median is **1.65**, mean 2.55. The
effort is still worth doing and is still first in §8, but on a ~1.7× expectation, not 7×. Corrected
in §8.1.

**10.3 `T_H / (2·min d_ret)` as a bound on `K`.** Proposed and discarded: `min d_ret` is tiny on
these instances, so the bound barely improved on `ceil(q_tot/Q)`. Replaced by the horizon-frontier
bound now reported as `K_allocated`.

**10.4 Speedup figures quoted from memory as "12.85× at Q=100,000."** The measured value depends on
which subset is taken — median over all instances is 1.67× (startup-dominated at small `n`), median
over `n >= 3,000` is 18.28×, mean over all is 6.21×. §5.5 states the subset explicitly. Small-`n`
timings should not be quoted at all at these run lengths.
