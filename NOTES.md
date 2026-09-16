# Splitting a Giant Tour When Route Cost Is Multiplicative

Working notes on the Periodic-Template VRP (PT-VRP): why Vidal's linear-time Split does not
apply to it, what does apply, what it costs, and what is still open.

`revival.md` is the standalone, step-by-step account of the early-stop finding (§5.8-§5.10 here),
written for a reader who wants the mechanism rather than the survey.

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
3. **`d` and `tau` are separate quantities.** The cost is on distance, the constraint is on duration.
   Every instance here happens to set `tau = 2d` — a constant speed — but nothing in the model
   requires `tau` to be any function of `d` at all, and the moment it is not, the geometric facts
   that justify the decoders' pruning stop applying to the constraint that uses them. This is the
   whole of §5.8–§5.10 and §7.4, and §3.3 is where it is stated precisely.

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
non-decreasing, which follows from the triangle inequality on travel times. Rather than refuse to run,
the solver counts violations and reports them; §5.1 measures whether they ever change an answer.

**When that assumption fails, and by how much.** This is the hinge of §5.8–§5.10 and §7.4, so it is
worth stating once, in general, rather than as a property of TSPLIB. The triangle inequality is a
statement about **distances**; every constraint and pointer here is on **duration**. The two coincide
only when duration is a function of distance and it is the *same* function on every leg. Anything that
prices the leg home independently of the arcs that replace it breaks them apart:

| source | violation | exhibited here |
|---|---|---|
| **independent integer rounding** — each distance rounded to a whole number on its own | exactly **1 unit**, never more | all 3,150 benchmark instances (`revival_arcs.md`) |
| **time-dependent travel** — the leg home is driven at a different hour, hence a different speed | unbounded | `Instances/Counterexamples/structural_violation.gt` |
| **queueing or service at the depot**, folded into the return leg | unbounded | — |
| **driver-hours, breaks, shift ends** charged wherever the leg home falls | unbounded | — |
| **asymmetric networks** — one-way systems, turn restrictions, tolls, ferries | unbounded | — |

Rounding is the version worth *demonstrating* on, because it assumes no modelling choice at all: it is
already in the data and one unit suffices (§5.8). It is also the weakest, and the difference is
load-bearing rather than cosmetic. **The magnitude of the violation determines which mechanism
breaks**, and the two break at different scales:

| violation | early stop / frontier | deque back-pop |
|---|---|---|
| ≤ 1 unit (rounding) | **breaks** — 4 counterexamples, §5.8 | survives; 0 unsafe pops on 3,150 instances |
| unbounded (structural) | **breaks** | **breaks** — `NO SOLUTION` on a feasible instance, §7.4d |

So a decoder hardened only against rounding is hardened against the half that never mattered.
Measured on `structural_violation.gt`: 46% of positions violated, median 156 units, maximum 482,
against TSPLIB's invariable 1. Both mechanisms are repaired in §7.4c and §7.4d.

### 3.4 `PTVRP_CONT_FIX`, `PTVRP_LAYERED_CONT_FIX`, `PTVRP_LAYERED_SAFE` — the sound decoders

Added after §5.8 established the defect. `PTVRP_CONT_FIX` (`Split_Bellman_PTVRP_cont_fix.{h,cpp}`) is
`PTVRP` stopping on the path out instead of the full route; `PTVRP_LAYERED_CONT_FIX`
(`Split_Layered_PTVRP_cont_fix.{h,cpp}`) applies the same quantity to both of the layered decoder's
one-way pointers; `PTVRP_LAYERED_SAFE` (`Split_Layered_PTVRP_safe.{h,cpp}`) adds joint-dominance
eviction on top, which is what the second row of the table above requires. Derivations, costs and
sweeps in §7.4c–d. **These are the solvers to use.** `PTVRP` and `PTVRP_LAYERED` are retained as the
unsound baselines they are measured against.

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

### 5.7 What the layering actually restores: the mixed second difference

The cleanest statement of both the obstruction and the repair is through the discrete mixed second
difference, for `i < i'` and `j < j'`:

```
Delta(i,i'; j,j')  =  c(i,j) + c(i',j')  -  c(i,j')  -  c(i',j)
```

Three conditions on a cost matrix, in strictly decreasing order of strength:

| condition | requirement on `Delta` | what it buys |
|---|---|---|
| separable, `c = A[i] + B[j]` | `= 0` everywhere | permanent discarding; a monotone deque, `O(n)` |
| Monge | `<= 0` everywhere | argmin monotone in `j`; SMAWK / LARSCH |
| totally monotone | weaker still | SMAWK |

The first two are related by an equivalence that is easy to miss: **a matrix is separable if and
only if `Delta` vanishes on every 2x2 submatrix.** Forward is one line of algebra; backward, set
`A[i] = c(i,j0) - c(i0,j0)` and `B[j] = c(i0,j)`, and the vanishing condition reconstructs
`c(i,j) = A[i] + B[j]`. So "every submatrix is Monge with equality" and "separable" are the same
property under two names. Separability sits *on the boundary* of the Monge cone, not inside it.

This settles what Vidal's result rests on. His cost is separable by construction -- the depot legs
attach entirely to one endpoint each and the interior cost is a difference of prefix sums -- so
`Delta == 0` identically, and Property 2 is that fact restated. The deque with permanent discarding
is available *because the boundary case is attained*. The machinery of Aggarwal, Schieber and
Tokuyama addresses the strictly larger class `Delta <= 0`, which contains matrices such as
`c(i,j) = -i*j` that admit no separation at all. Separability implies Monge; Monge does not imply
separability, and the implication does not run backwards (§10.5).

**Under the multiplicative objective, `Delta` neither vanishes nor keeps a fixed sign.** Write
`g(i,j) = A[i] + B[j] >= 0` and `l(i,j) = L[j] - L[i]`. Because `l` is *modular* --
`l(i,j) + l(i',j') = l(i,j') + l(i',j)` exactly -- a function `f(l)` is Monge iff `f` is convex, and
`m = max(1, ceil(l/Q))` is a staircase, not convex. Two realisable configurations, both on strictly
increasing loads at `Q = 10`:

| configuration | loads `L[i], L[i'], L[j], L[j']` | `m` at `(i,j),(i,j'),(i',j),(i',j')` | `Delta` |
|---|---|---|---|
| step at the near corner | 0, 5, 15, 20 | 2, 2, 1, 2 | `+g(i',j) > 0` |
| step at the far corner | 0, 4, 8, 12 | 1, 2, 1, 1 | `-g(i,j') < 0` |

The sign changes within a single instance. The PT-VRP cost matrix therefore lies in **neither** the
Monge nor the inverse-Monge cone, and the `max(1, .)` floor does not rescue it. A random scan over
200,000 load/cost profiles satisfying `At` non-increasing, `Bt` non-decreasing and `g >= 0` finds
violations of both directions in bulk (`monge_witness.py`, §9).

**What layering does, precisely.** It does not weaken the requirement to Monge and then apply weaker
machinery. It partitions the predecessor set until `Delta == 0` holds exactly on each part -- the
original, strongest property, restored piecewise. Within layer `k` the cost is `k*A[i] + k*B[j]` and
`Delta == 0` identically; the moving window does not break this, because if `(i,j')` and `(i',j)`
both lie in layer `k`'s band then `lo(j) <= lo(j') <= i` and `i' < hi(j) <= hi(j')`, so all four
corners are finite. The right description of the decoder is **separability restoration**, not a
Monge technique.

**Two gaps, stated rather than hidden.**

- Total monotonicity is *weaker* than Monge, so "not Monge" does not formally imply "not totally
  monotone". Ruling out TM specifically has not been done. §8.3 is probably closed; it is not
  provably closed until it is.
- The two witnesses are constructed on the abstract matrix under the monotonicity constraints of
  §3.3. Whether such `A`/`B` values arise from an actual point set and tour order is unverified. A
  referee will ask. The fix is cheap: scan the existing instances for a real sign-changing 4-tuple.

---

### 5.8 The early stop is unsound: four counterexamples

The Bellman DP and the layered decoder both terminate a scan early. `Split_Bellman_PTVRP.cpp:77`
breaks the forward scan at the first `j` with `tau(i,j)*m > T_H`; `Split_Layered_PTVRP.cpp:123`
advances `firstTimeLE[k]` past starts failing the same test. Both rest on `tau(i,j)*m` being
non-decreasing in `j`.

**It is not.** Extending a template from `v_j` to `v_{j+1}` *drops* the return leg `d(v_j, depot)`
and adds two legs, so the duration changes by

```
    d(v_j, v_{j+1}) + d(v_{j+1}, depot) - d(v_j, depot)
```

which is non-negative only under the triangle inequality. Where that fails, a longer template can be
*shorter*, and if the shorter one fits the horizon the early stop has already walked away from it.

**The counterexamples below use rounding; the claim is not about rounding.** Per §3.3, the triangle
inequality on durations fails whenever duration is not one fixed function of distance — hourly traffic,
depot queueing, driver-hours, one-way networks. Rounding is chosen here only because it is the cheapest
witness available: it is already in the benchmark and needs no modelling assumption to defend. It is
also the *smallest* violation of the family, which is why these four instances all turn on a margin of
one unit. On a duration model that varies with the clock the margin is unbounded, and a second
mechanism fails as well (§7.4d).

`PTVRP_LAYERED_CONT` (`Split_Layered_PTVRP_cont.{h,cpp}`) is the layered counterpart: it keeps the
layer partition, which is load-based and unconditionally exact, and drops only the two one-way time
pointers. The first version scanned every start in every layer, `O(n^2 K)`, and agreed with
`PTVRP_CONT` on the 480 instances with `n <= 600`. It has since been rebuilt on `PTVRP_LAYERED`'s own
deques: infeasible starts are stepped over at query time instead of discarded, `O(n·K_full)`, which
makes the full 3,150-instance sweep feasible. It returns the true optimum on all four counterexamples.
Full-sweep results and the one assumption the deque still carries are in `revival_result.md`.

`PTVRP_CONT` (`Split_Bellman_PTVRP_cont.{h,cpp}`) is the control: the same DP with `continue` in
place of `break`, so every pair `(i,j)` is evaluated. It counts **revived arcs** — arcs that are
feasible although an earlier one was not, i.e. exactly what the early stop never sees.

Four instances in `Instances/Counterexamples/`, each `n = 3`, each verified against exhaustive
enumeration of all partitions:

| instance | true optimum | `PTVRP` | `PTVRP_CONT` | `PTVRP_LAYERED` | `PTVRP_LAYERED_CONT` | triangle violation |
|---|---|---|---|---|---|---|
| `ce_rounding` | **80** | 101 (+26%) | 80 | 101 (+26%) | 80 | **1 unit** |
| `ce_blatant` | **70** | 120 (+71%) | 70 | 120 (+71%) | 70 | 40 units |
| `ce_multiplier` | **240** | 241 | 240 | 241 | 240 | 1 unit, with `m = 2` |
| `ce_infeasible` | **70** | `NO SOLUTION` | 70 | 70 | 70 | 40 units |

Three things these establish that the 3,150-instance sweep could not.

**(a) The violation need only be one unit — and one unit is the floor, not the typical case.**
`ce_rounding` has `d(v_2, depot) = 21` against `d(v_2, v_3) + d(v_3, depot) = 20`, exactly the
magnitude independent integer rounding produces. The horizon is then placed so that
`d(0,2) = 81 > T_H = 80 >= d(0,3) = 80`. This is not an artefact of pathological geometry; it is
reachable from the rounding already present in the instance set. Read the other way, it is the
*strongest* form of the claim: if one unit suffices, then every larger violation — which is to say
every non-trivial duration model, §3.3 — suffices too.

**(b) The multiplier does not protect.** `ce_multiplier` keeps `m = 2` constant across the revival
step, so the failure survives into the regime PT-VRP actually cares about. The margin is 1 unit in
240, which is the honest size: revival needs `m` *not* to step at that position (§5.9), and that
constrains how much cost can ride on the revived arc.

**(c) The failure mode includes false infeasibility.** `ce_infeasible` is feasible at cost 70 and
`PTVRP` reports no solution at all. For a decoder feeding a penalty-based fitness this is worse than
a suboptimal cost — it misreports the feasible region.

**The two decoders fail on overlapping but different sets.** `PTVRP_LAYERED` reproduces `PTVRP`'s
error on the first three and is *correct* on `ce_infeasible`. The mechanisms are not the same bug:
`break` abandons a start permanently, whereas `firstTimeLE[k]` is a window over predecessors that a
later column can still re-admit. This is the direct evidence on "which pointer" that §7.4 asked for
and that cost agreement alone could not supply.

**What the sweep says anyway.** On all 3,150 real instances the three solvers agree exactly
(§5.1), and the early stop skipped an improving arc on 103 of them without ever changing an answer.
So the correct statement is not "the early stop is safe" but:

> The early stop is **unsound**, demonstrably so at rounding scale, and **empirically harmless on
> every instance in this benchmark set**.

### 5.9 Where revival lives, and why it is horizon-free

Revival requires two things at the step where a start crosses the horizon:

1. **`m` must not step there.** If the multiplier increments, `tau*m` jumps up by a whole `d*Delta m`
   and swamps any backward wobble. Probability that it does not step is roughly `(1 - rho)^+` for
   `rho = qbar/Q`.
2. **The wobble must beat the drift.** Forward drift per step is about `m*c` for mean arc `c`;
   backward wobble is at most `m*delta` for one rounding unit `delta`. **The `m` cancels**, leaving
   the condition `delta/c` — free of both horizon and capacity.

There is also a clean counting fact: **every start crosses the horizon exactly once**, so there are
exactly `n` chances per instance regardless of `T_H`. The horizon moves *where* along the scan the
crossing happens, not *how many* crossings there are.

Measured over the 3,150-instance sweep:

| | revived arcs per million crossings |
|---|---|
| `rho < 1` (`m` does not step at every vendor) | **38.2** |
| `rho >= 1` (`m` steps at every vendor) | **1.1** |

A 35x cliff at `rho = 1`, exactly where the gate predicts it. Per-instance rates rise 0.2% -> 12.6%
with `n` simply because there are more crossings, not because the process differs. Caveat: the
per-crossing rate is not flat across `n` (6.7 / 77 / 33 / 35 per million by size bucket); short tours
are lower, most likely because many starts never reach the horizon at all.

**Superseded as a repair, not as a measurement.** The suffix bound this section suggested is sound
but needs an O(n) array rebuilt whenever the tour order changes. The path-out rule (§7.4c) needs
nothing precomputed and is what was implemented. The rate measurements below stand.

**This is the same dimensionless group as §7.3.** That section derives `B` and `K` both scaling as
`sqrt(T_H)`, so the ratio `B/K = Q/qbar = 1/rho` is horizon-free. Revival is governed by the same
`rho` and is horizon-free for the same reason: `T_H` cancels. The two questions are separate — §7.3
is performance, §5.9/§7.4 is correctness — but one experiment tests both (§7.3's two-horizon rerun).

### 5.10 Vidal's original Split is not affected

Worth stating explicitly, because the counterexamples above invite the inference that the same defect
sits in the classic algorithm. It does not. Every termination and eviction condition in Vidal's
originals is **load-based**:

```
Split_Bellman.cpp:36           for (... ; j <= nbNodes && load <= vehCapacity ; j++)
Split_Bellman_Bounded.cpp:45   for (... ; j <= nbNodes && load <= vehCapacity ; j++)
Split_Linear.cpp:56            while (sumLoad[i+1] - sumLoad[front] > vehCapacity + 0.0001)
Split_Linear_Bounded.cpp:75    while (sumLoad[i+1] - sumLoad[front] > vehCapacity + 0.0001)
```

Load is a cumulative sum of non-negative demands, hence strictly increasing in `j`
**unconditionally** — no geometry, no triangle inequality, nothing for rounding to perturb. Once
`load > Q` it stays `> Q`. Vidal's early termination is sound by construction and revival is
impossible in his setting.

His own source corroborates the principle. In `Split_Bellman_Soft.cpp:37`, where capacity becomes a
penalty rather than a hard limit and the monotone stopping rule no longer applies, the guard is
commented out and the scan runs to `n`:

```cpp
for (int j = i+1 ; j <= myData->nbNodes /* && load <= 4.0 * myData->vehCapacity */ ; j++)
```

The same reasoning as `PTVRP_CONT`, reached independently: when the quantity you would break on stops
being monotone, stop breaking on it.

**The transferable claim is therefore about the constraint, not the author.** Any Split that
terminates on accumulated **time** rather than accumulated **load** inherits this defect — the
distance-constrained VRP, VRPTW, any duration-limited variant. Load is monotone by construction;
duration is monotone only under the triangle inequality, which is a property of *distances*. Without
a `_cont`-style control there is nothing to observe, which may be why it appears to be unreported.

**And the exposure grows with the realism of the duration model, not with the size of the instance.**
§3.3 is the ladder: a purely academic benchmark supplies a 1-unit violation through rounding, which is
enough to make the defect real but keeps every margin tiny. Each step toward a duration a dispatcher
would recognise — travel time that depends on departure hour, waiting at the depot gate, a break
charged against the drive home, a one-way ring road — prices the leg home independently of the arcs
replacing it and removes the bound on the violation entirely. So the small margins measured in §5.8
and the zero decisive cases in §7.4a are properties of *this benchmark being metric*, not properties
of the defect. A duration-constrained Split run against real travel times has no such margin, and
needs both repairs (§7.4c, §7.4d), not just the first.

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

Status, so the list is readable at a glance:

| | question | state |
|---|---|---|
| 7.1 | Monge structure / what layering restores | **resolved** (§5.7) |
| **7.2** | **a priori bound on `K`** | **open** |
| **7.3** | **does the crossover move with `T_H`?** | **open** — prediction made, untested |
| 7.4 | is the early stop unsound, and what to do | **resolved** (§5.8, §7.4c, §7.4d) |
| 7.5 | does any of it survive inside HGS | **deferred** — later, not now |

So the live work is **7.2 and 7.3**, and only 7.3 has a ready experiment.

1. **Is the PT-VRP cost matrix Monge within a layer? -- RESOLVED, see §5.7.** Within a layer the
   cost is separable, which is the *boundary case* `Delta == 0` of the Monge condition rather than a
   weaker consequence of it, and the moving window does not break it. The full matrix is neither
   Monge nor inverse-Monge, since `Delta` takes both signs within one instance. The right description
   of the decoder is **separability restoration**, not a Monge technique, and that is what the
   question was asking.

   Two residues are parked rather than counted as open here, because neither can change the decoder:
   **(i)** whether the full matrix is *totally monotone* — TM is weaker than Monge so the
   counterexamples do not exclude it — which is the last door for §8.3 and belongs there, since it
   would only matter if someone reopened the exponent line; **(ii)** whether the witness 4-tuples are
   realisable from actual geometry rather than from the abstract monotonicity constraints of §3.3.
   (ii) is a referee-proofing chore — scan the instances for a real sign-changing 4-tuple — not a
   question whose answer would change anything here.
2. **Is there a tighter a priori bound on `K` than the horizon frontier? -- OPEN. Still open, but the
   frontier is now sound.** §7.4c replaces the unsound frontier with a path-out one at no cost in the
   bound (median `eff_K` 30.2 against the unsound 29.2), so the question is no longer entangled with
   correctness. What is still missing is a bound computable *before* the sweep rather than advanced
   during it. `K_allocated` (median 27)
   already improves on `ceil(q_tot/Q)` (median 287) by ~10×, but `eff_K` (median 15) and
   `max_layer_used` (median 9) show there is more slack. A bound computable before the sweep would
   let layers be allocated once rather than grown.
3. **Does the crossover at `Q ≈ 100` move under a different horizon? -- OPEN. The §5.4 algebra says no.**
   Solving `B·K ≈ T_H/c` together with `K/B = rho = qbar/Q` gives `K ≈ sqrt(rho·T_H/c)` and
   `B ≈ sqrt(T_H/(c·rho))`: both grow as `sqrt(T_H)`, so the *ratio* `B/K = Q/qbar` carries no `T_H`
   at all. With per-unit implementation constants `alpha` (Bellman, per predecessor scanned) and
   `beta` (layered, per layer visited), the crossover condition `alpha·B = beta·K` gives

   ```
       Q* = qbar · (beta / alpha)          independent of T_H
   ```

   The horizon is a budget spent on the *product* `tau·m`; stretching it inflates `B` and `K` by the
   same factor and leaves the race unchanged. **Prediction: rerunning §5.5 at `T_H = 43,200` and
   `T_H = 172,800` leaves `Q*` near 100 while both timing columns scale as `sqrt(2)`.** Untested —
   every instance here has `T_H = 86,400`. This is the cheapest available test of §5.4 on a
   dimension it was not fitted to, and it supersedes the earlier guess in this slot that a shorter
   horizon *should* move the crossover (§10.6).
4. **Does the monotonicity violation ever matter? -- RESOLVED: yes, and both halves are now patched.**
   The defect is established (§5.8), its two mechanisms are separated (a/b), and each has a sound
   replacement that is implemented, swept and measured (c for the pruning, d for the eviction). The
   only residue is 7.4a — how often the *unsound* version would actually bite — which the fix makes
   moot in practice and which is kept below as a measurement, not a blocker.

   The early stop is **unsound**, not merely unproven. Four counterexamples in
   `Instances/Counterexamples/` drive
   `PTVRP` to a wrong answer, one of them from a triangle violation of **exactly one unit** -- the
   magnitude TSPLIB's independent integer rounding already produces -- and one of them to a false
   `NO SOLUTION` on a feasible instance. All four verified against exhaustive enumeration.
   `PTVRP_LAYERED` fails on three of the four and is correct on the fourth, so `break` and
   `firstTimeLE[k]` are related but not identical defects: `break` abandons a start permanently,
   whereas the layered window can re-admit one in a later column.

   What follows is in four parts: (a) how often it bites, (b) the layered decoder's counter,
   (c) the repair to the pruning, (d) the repair to the eviction. Only (a) is still a measurement.

   **(a) Frequency on realistic geometry.** The counterexamples are constructed: the horizon is
   placed deliberately in the one-unit gap. On the 3,150-instance benchmark the early stop skipped an
   improving arc 177 times across 103 instances and **never** changed an answer. So the gap between
   "unsound" and "harmful" is entirely a question of how often `T_H` lands in that window, and this
   benchmark says: not once. Whether that survives a different horizon, a different rounding
   convention, or non-Euclidean travel times is untested. §5.9 predicts the rate is horizon-free;
   that prediction has not been run.

   **(b) The layered decoder has no consequence counter -- ANSWERED.** It now has one.
   `monotonicityViolations` (`Split_Layered_PTVRP.cpp:51`) is only a **precondition** check: it scans
   `At[]`/`Bt[]` once and counts where the arrays are non-monotone. Measured against `PTVRP_CONT`'s
   consequence counter on `Instances 1`, 24,050 flagged positions produced 167 revived arcs -- zero
   misses, but a 144:1 false-alarm rate, tripping on 740 of 1,050 instances. Useless as evidence.

   `PTVRP_LAYERED_CONT` and `PTVRP_LAYERED_CONT_FIX` both report `REVIVED STARTS` / `REVIVED
   IMPROVING` -- starts that won a layer from behind where the pruned solver's frontier stood. Over
   all 3,150 instances: **18 revived winners on 18 instances, 2 improving, 0 changing an answer**
   (20 / 2 / 0 with the bound in place). All 18 sit inside Bellman's 103. So the layered half of this
   question no longer rests on cost agreement.

   **(c) What to do about it -- ANSWERED: patch it, and it is nearly free.** None of the three options
   listed here was taken. The rule that works is simpler than all of them: **prune on the route
   without the leg home.**

   ```
       path_out(i,j) = A[i] + sumDistance[j]          the route minus the return leg
       stop when   path_out(i,j) * m(i,j) > T_H
   ```

   Sound with no assumption about the instance whatever -- not the triangle inequality, not rounding,
   not symmetry. `sumDistance` only grows (extending a template appends an arc and removes nothing,
   the leg home being exactly what is excluded), `m` only grows, and `d(i,j) >= path_out(i,j)`.
   Revival lives entirely in the leg home, so a bound that ignores it cannot be escaped. The test
   deciding whether an arc may be *used* is unchanged and exact; an arc that fails it is skipped, not
   stopped on.

   Two solvers implement it, `PTVRP_CONT_FIX` (`Split_Bellman_PTVRP_cont_fix.{h,cpp}`, a one-line
   change since the Bellman loop's running `time` already is the path out) and
   `PTVRP_LAYERED_CONT_FIX` (`Split_Layered_PTVRP_cont_fix.{h,cpp}`, where the same quantity makes
   *both* one-way pointers sound because `sumDistance` is monotone where `Bt` is not).

   All 3,150 instances against `PTVRP_CONT` (`sweep_both_fixes.csv`, 15,750 rows, one machine):
   **2,829 identical, 321 both-infeasible, 0 disagreements, 0 `UNSAFE POPS`.**

   | solver | total | |
   |---|---|---|
   | `PTVRP_LAYERED` | 23.7 s | unsound frontier |
   | `PTVRP_LAYERED_CONT_FIX` | **28.0 s** | exact |
   | `PTVRP` | 34.0 s | unsound early stop |
   | `PTVRP_CONT_FIX` | **38.4 s** | exact |
   | `PTVRP_CONT` | 781.8 s | no stop, the oracle |

   **Soundness costs 1.13x in Bellman and 1.18x in the layered decoder**, and both stay ~20x faster
   than the oracle. For the layered decoder the bound also restores `eff_K`: median 998.9 -> 30.2 on
   `Instances 3`, against 29.2 for the unsound frontier. Details in `revival.md` §9 and
   `revival_result.md` §6.

   This supersedes §5.9's proposed suffix bound and the suffix-minimum refinement that followed it.
   Both are sound, and the suffix minimum is tighter, but both need an O(n) array rebuilt whenever
   the tour order changes -- which inside HGS is every iteration. The path-out rule needs nothing
   precomputed.

   **(d) The other mechanism -- the deque back-pop -- ANSWERED: it also breaks, and it is also
   patched.** (c) closes the *pruning*. It does not close the *deque*, and §3.3 says exactly when the
   difference shows: at rounding scale the back-pop survives (0 unsafe pops on 3,150 instances), at
   structural scale it does not.

   The deque discards an older start `b` when a newer start `i` has a key at least as small. With a
   horizon, `i` must also **fit whenever `b` fits**, and the key -- `p[i] + k·A[i]` -- ranks on cost
   alone. Under the triangle inequality `At` is non-increasing and a later start always fits at least
   as easily, so the gap never opens. Off it, the deque can discard the only start that fits.
   `ce_unsafe_pop_5v.gt` is five vendors with `A = [79, 41, 66, -12, 50]`, and every layered solver
   without the guard -- `PTVRP_LAYERED_CONT_FIX` included -- returns **`NO SOLUTION` on an instance
   feasible at 658**. That is a worse failure mode than any in §5.8.

   `PTVRP_LAYERED_SAFE` (`Split_Layered_PTVRP_safe.{h,cpp}`) adds one clause, and it is exact rather
   than conservative:

   ```
       evict b   when   key(i) <= key(b)   AND   At[i] <= At[b]
   ```

   Feasibility at column `j` in layer `k` is `(At[start] + Bt[j])·k <= T_H`. Comparing two starts at
   the **same** `j` cancels `Bt[j]`, exactly as it cancels in the cost, so `At[i] <= At[b]` means
   "wherever `b` fits, `i` fits", at every `j`, permanently. That is the one `j`-independent
   comparison that still accounts for the leg home, which is why this repair belongs in the eviction
   and could not have gone into the pruning -- where the leg home is precisely what must be dropped.
   The guard can leave a layer unsorted by key, so `sortedByKey[k]` is tracked per layer: sorted
   layers keep the original `O(1)` "first feasible from the front" query, and a layer whose pop was
   blocked scans for the cheapest feasible entry instead. `BLOCKED POPS` and `UNSORTED QUERIES`
   report both.

   | | `PTVRP_LAYERED_CONT_FIX` | `PTVRP_LAYERED_SAFE` |
   |---|---|---|
   | `ce_unsafe_pop_5v` (optimum 658) | `NO SOLUTION` | **658** |
   | 60 structurally non-metric instances | 59 / 60 | **60 / 60**, 5,205 pops blocked |
   | 3,150 benchmark instances vs `PTVRP_CONT` | 2,829 identical, 321 both-infeasible, 0 differ | **identical** |
   | `BLOCKED POPS` / `UNSORTED QUERIES` on the benchmark | -- | **0 / 0** |
   | total, median `eff_K` | 28.0 s, 17.3 | 28.0 s, 17.3 |

   The last two rows are the point, and the stronger reading is not that the guard rescued instances
   here. It is that the guard **never fired**: no layer ever left the `O(1)` query, so the guard is
   free on metric-up-to-rounding data, and it confirms that `PTVRP_LAYERED_CONT_FIX` was already
   correct on this benchmark rather than rescuing it. The two diverge only once the data stops being
   metric, which is where §3.3 says real duration models live.

   **With (c) and (d) both in, the layered decoder carries no assumption about the instance at all**,
   and the conditional on its exactness -- "exact provided `UNSAFE POPS` is zero" -- is gone.
   `PTVRP_CONT_FIX` never carried one: it has no deque.

5. **Does any of this survive inside HGS? -- DEFERRED, for later.** Every result here is on a static giant tour. Split inside
   HGS is called on tours that are themselves evolving, and the distribution of those tours is not
   the distribution of TSPLIB orders. Two separable halves, neither attempted. **(a)** Does the
   demand centroid stay near 0.5 under OX/PMX crossover, which is position-based and demand-blind?
   If so, §5.6 and §5.5 carry over unchanged. **(b)** Split is called 10^5-10^6 times on tours
   differing by a few moves, so reuse *across calls* becomes a lever with no analogue in the static
   setting (§8.3). A third issue is unspecified anywhere: how a horizon-infeasible tour is reported
   to a penalty-based fitness, given that 321 instances are genuinely infeasible and §5.2 shows the
   deque never detects infeasibility at all.

### Status ledger (2026-09-16)

A snapshot of what is settled and what is not, so the distinction survives the next gap in work.
"Settled" means the argument has been made and checked here; it does not mean peer-reviewed.

**Settled**

| claim | evidence | caveat |
|---|---|---|
| Within a layer the cost is separable; Property 2 holds | algebraic | the moving window does not break it — all four corners stay finite (§5.7) |
| separable `<=>` `Delta == 0` on every 2x2 | both directions shown | classical; §10.5 rests on it |
| `separable => Monge`, and not conversely | counterexample `c = -i*j` | the implication direction was the error in §10.5 |
| The full matrix is neither Monge nor inverse-Monge | 2 closed-form witnesses + 200k random scan | witnesses are abstract, not yet exhibited on real geometry |
| Vidal's deque needs separability, not Monge | §5.7 | three unrelated senses of "monotonicity" are in play — see §7.4 |
| Layer partition is rounding-safe; only the horizon frontier is at risk | read from the source | `firstLoadLE` vs `firstTimeLE` (§7.4) |
| §7.2 asks for a *bound*; §5.4 supplies an *estimate* | — | reframe only, no new result |
| `Q* = qbar·(beta/alpha)`, horizon-free | algebraic, from §5.4 | a prediction; untested (§7.3) |
| The early stop (`break`, `firstTimeLE`) is **unsound** | 4 counterexamples vs exhaustive enumeration (§5.8) | one needs a violation of only **1 unit**; one yields false `NO SOLUTION` |
| `break` and `firstTimeLE` are different defects | `ce_infeasible`: `PTVRP` wrong, `PTVRP_LAYERED` right | answers "which pointer" (§7.4) |
| All 3 PT-VRP solvers agree on 3,150/3,150 real instances | full sweep, identical cost strings | agreement is not soundness (§5.8) |
| Revival is gated by `rho = qbar/Q`, horizon-free | 35x rate cliff at `rho = 1` (§5.9) | per-crossing rate not flat in `n` |
| Vidal's original Split is unaffected | every guard is load-based (§5.10) | load is monotone by construction |
| The unsound early stop can be made **exact** by pruning on the path out | 3,150 instances, 0 disagreements vs `PTVRP_CONT` (§7.4c) | needs no assumption and nothing precomputed |
| Soundness costs 1.13x (Bellman) / 1.18x (layered) | `sweep_both_fixes.csv`, one machine | ~20x faster than the oracle either way |
| A sound frontier restores the layer bound | median `eff_K` 998.9 -> 30.2 on `Instances 3` | within 3% of what the unsound frontier gave |
| `firstTimeLE` does mis-step on real instances | 18 revived winners on 18 instances, 2 improving, 0 decisive | answers §7.4b; all 18 inside Bellman's 103 |
| The defect is about duration not being a function of distance; rounding is its weakest instance | §3.3 ladder; `structural_violation.gt` violates 46% of positions by a median 156 vs TSPLIB's 1 | the non-rounding sources (traffic, queueing, driver-hours) are argued from the model, not instrumented |
| The deque back-pop **is** decisive off metric data | `ce_unsafe_pop_5v`: every unguarded layered solver returns `NO SOLUTION` on an instance feasible at 658 | five vendors, checkable by hand (§7.4d) |
| Joint-dominance eviction closes it, exactly and for free | `PTVRP_LAYERED_SAFE`: 60/60 structural, 3,150/3,150 benchmark, 28.0 s, guard fired 0 times | `At[i] <= At[b]` is exact because `Bt[j]` cancels at fixed `j` |
| The layered decoder now carries **no** instance assumption | (c) + (d) together | the "provided `UNSAFE POPS` = 0" qualifier is retired |

**Open**

| question | state | what is missing |
|---|---|---|
| **A priori bound on `K` (§7.2)** | **open** | reframed, not answered; no candidate beyond what `K_allocated` already computes |
| **Does the crossover move with `T_H` (§7.3)?** | **open — prediction only** | the two-horizon rerun; the one live question with an experiment already specified |
| How often would the *unsound* early stop bite? (§7.4a) | open, and now academic | a horizon sweep. Both decoders are patched, so this only sizes the defect in code that has not been patched |
| Survival inside HGS (§7.5) | **deferred — later** | centroid stability; cross-call reuse; infeasibility reporting |
| Realisable fraction of §8.1 | open | unchanged — unknown until implemented |

**Parked** — residues that cannot change the decoder, kept so they are not rediscovered as new

| question | where it belongs |
|---|---|
| Is the full matrix totally monotone? | §8.3, the exponent line. TM is weaker than Monge so §5.7 does not exclude it; only matters if that line is reopened |
| Are the §5.7 witnesses geometrically realisable? | referee-proofing; scan the instances for a real sign-changing 4-tuple |
| Does the path-out fix hold off this benchmark? | **largely answered** by the 60 structural instances (`PTVRP_CONT_FIX` 60/60). Untested only across horizons, which §7.3's rerun would cover |

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

### 8.3 Monge / K-link path machinery — closed, pending one check

**Superseded by §5.7.** This was the only item that could change the exponent rather than the
constant, and the proof attempt §7.1 asked for has been made and came back negative. The reasoning
is worth keeping. The machinery of Aggarwal, Schieber and Tokuyama — including the *K-link path*
problem, which Vidal cites in his own conclusion as the direction he did not take — applies to
matrices with `Delta <= 0`. The PT-VRP matrix has `Delta` of both signs within a single instance, so
there is no weaker structural property to fall back on once separability is lost.

The deeper reason this line was never as promising as it looked: layering is not a way of exploiting
Monge structure, it is a way of *restoring separability* on a partition, which is the strictly
stronger property. Having restored it, there is nothing further for SMAWK or LARSCH to extract — a
separable matrix is Monge with equality, degenerate, and a linear scan per layer is already optimal
on it.

The one remaining door is **total monotonicity**, which is weaker than Monge and therefore not
excluded by the counterexamples. If anyone reopens this line, that is the proof to attempt, and it
should still be attempted before any code.

**Recommended order: 8.1 now** (contained, measurable, verifiable), 8.2 if another constant is
wanted. With the exponent line closed, the remaining upside is entirely in how *few* layers are
touched (§8.1, §8.2) and — newly — in reuse across Split calls inside HGS (§7.5b), which has no
analogue in the static benchmark and is not yet a numbered item anywhere.

### 8.4 Repository debt

- ~~README's solver table stops at `PTVRP_LAYERED_CONT`~~ — **cleared 2026-09-16.** It now lists all
  eight PT-VRP solvers, marks which are unsound, and says plainly to use `PTVRP_CONT_FIX` or
  `PTVRP_LAYERED_SAFE`. The instance list gained `Instances/Counterexamples/`.
- ~~`revival_result.md` presents the back-pop as an open assumption~~ — **cleared 2026-09-16.** §8.2
  documents `PTVRP_LAYERED_SAFE`, §8 option 3 no longer says "not yet implemented", §3's certificate
  is explicitly marked conditional with a pointer to where the condition fails, and §9's "look for an
  instance with `UNSAFE POPS > 0`" is marked answered by its own §8.1.
- ~~README §"Skewing the load along the tour" carries the superseded skew table~~ — already corrected;
  it now shows the filtered 5x4 matrix and points at §10.1. Nothing to do.
- `-horizon` is not a command-line flag. `Pb_Data.cpp:56` takes the horizon from the instance file's
  `MAX_ROUTE`, and `commandline.cpp` parses only `-solver -veh -pen -trace`. §7.3's two-horizon rerun
  needs this; adding the flag is far cheaper than rewriting 6,300 instance files.
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

# the four counterexamples of §5.8 -- PTVRP is wrong on all four, the two fixed solvers right
for f in Instances/Counterexamples/*.gt; do
  for s in PTVRP PTVRP_CONT PTVRP_CONT_FIX PTVRP_LAYERED PTVRP_LAYERED_SAFE; do
    echo -n "$(basename $f) $s "; Program/split "$f" -solver $s 2>&1 | grep -E "SOLUTION COST|no Split"
  done
done

# §7.4d -- the back-pop, where only PTVRP_LAYERED_SAFE survives among the layered solvers
for s in PTVRP_CONT PTVRP_CONT_FIX PTVRP_LAYERED_CONT_FIX PTVRP_LAYERED_SAFE; do
  echo -n "$s "
  Program/split Instances/Counterexamples/ce_unsafe_pop_5v.gt -solver $s 2>&1 \
    | grep -oE "SOLUTION COST : [0-9.]+|UNSAFE POPS : [0-9]+|BLOCKED POPS : [0-9]+|no Split solution" \
    | tr '\n' ' '; echo
done

# the skew matrix behind §5.6
python3 skew_instances.py --src "Instances/Instances 1" --out Instances/skew --limit 40 \
    --mu 0.1 0.3 0.5 0.7 0.9 --conc 2 4 10 30
python3 batch_run.py $(for d in Instances/skew/mu*; do echo -n "--dir $d "; done) \
    --solver PTVRP_LAYERED --out skew_results.csv

# the counterexamples behind §5.7 : both QI directions, hand-built + 200k random scan
python3 monge_witness.py
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

**10.5 "Separability comes from the Monge property."** Backwards, and worth keeping because the
reconciliation is the useful part. The implications run `separable => Monge => totally monotone`,
one direction only: `c(i,j) = -i*j` is Monge and admits no decomposition `A[i] + B[j]`. The
intuition behind the error — that Vidal's matrix is built out of Monge submatrices — is *correct*,
but circular without announcing itself, because those submatrices are Monge **with equality**, and
"`Delta` vanishes on every 2x2" is precisely the definition of separable. There is nothing hidden
inside the Monge structure to be found by inspecting the node arithmetic; at equality the Monge
structure *is* the separability. Consequence for the write-up: "separability comes from Monge" is
safe only if "Monge" means the equality case, in which case it is a tautology, and a reader who
takes "Monge" in the SMAWK sense (`Delta <= 0`) will read it as false. State the condition on
`Delta` and the ambiguity disappears. See §5.7.

**10.6 "A shorter horizon should move the crossover."** Stated in §7.3 as a consequence of
`B·K ≈ T_H/c`, and contradicted by that same relation once it is solved rather than quoted: `B` and
`K` both scale as `sqrt(T_H)`, so the ratio deciding the race is horizon-free and
`Q* = qbar·(beta/alpha)`. The general lesson: a relation on a *product* constrains the two factors
jointly but says nothing about their ratio, and it is the ratio the crossover depends on. Corrected
in §7.3; still untested.
