# Reducing the layered decoder

`NOTES.md` §7.2 asks for *a tighter a priori bound on `K` than the horizon frontier*. This file
answers that question, and then asks the larger one behind it: whether `O(n·K)` can be beaten at all,
or whether the layer partition is the end of the line.

**Short version.** Three results, one negative and two positive.

1. **Exact separability cannot be restored across layers.** It is not a gap in our construction; the
   mixed second difference is non-zero, and separability is *equivalent* to that difference
   vanishing. §4.1 is the proof.
2. **What replaces it is a single-crossing structure.** At a fixed column, every layer contributes
   exactly *one affine function of `t = B[j]`*, with slope `k`. So the per-column answer is the lower
   envelope of `K` lines at a parameter that advances monotonically. Cross-layer order can flip, but
   only once per pair. §4.2–§4.3.
3. **The rounding, not the multiplier, is what breaks Monge.** Replace `m = ceil(l/Q)` by `l/Q` and
   the matrix becomes Monge — provably, under exactly the monotonicity the solver already checks.
   Keep the `max(1, ·)` floor and it is still Monge. Keep the ceiling and drop the floor and it is
   not. §4.4.

And one measurement that matters more than any of them for practical work: **at `Q = 10` the layer
partition degenerates to one start per layer** (§2.4). The deque compresses nothing there, which is
why the decoder loses to Bellman at exactly the capacities where the PT-VRP is most distinctive.

Every number below comes from the nine root `sweep_*.csv` files and from the scripts in §7.

---

## 1. What "reduction" can mean here

Three different things get called a reduction, and they have different prices.

| target | from | to | what it needs |
|---|---|---|---|
| **tighter bound** | `K_allocated` | a per-column `K(j)` | arithmetic already in the loop (§3) |
| **fewer visits** | `eff_K` per column | the layers that can win | a liveness test (§5.1) |
| **lower exponent** | `O(n·K)` | `O(n polylog)` | not touching every (start, layer) pair (§5.3) |

Only the third changes the asymptotics. The first two are constants, and §5.3 argues the third is
blocked by something structural rather than by missing cleverness.

---

## 2. Where the work actually is

### 2.1 The five K's are two quantities

`K_allocated` and `eff_K` have Spearman correlation **0.99** (0.97 in log space) and a median ratio
of 1.36. They are the same quantity read twice. `max_layer_used` and `max_m` agree exactly on
**69.1%** of feasible instances, and `max_layer_used >= max_m` always (947 instances strictly above,
0 below). So the five collapse to:

- `K_full_tour` — the naive ceiling, irrelevant to the work after the horizon bound;
- `K_allocated ~ eff_K` — **the work**;
- `max_layer_used ~ max_m` — **the work that mattered**.

The gap between the last two is the whole opportunity: median `eff_K / max_layer_used` is **1.89**,
p90 **5.36**, max **17.5**.

### 2.2 K is set by capacity, not by size

Spearman against the five K's:

| | `K_full` | `K_alloc` | `eff_K` | `max_layer_used` | `max_m` |
|---|---|---|---|---|---|
| `n` | 0.58 | **0.26** | **0.26** | 0.27 | 0.20 |
| `Q` | −0.78 | −0.90 | −0.88 | −0.84 | −0.82 |
| `rho = qbar/Q` | 0.78 | **0.90** | **0.88** | 0.84 | 0.82 |

In log space the `n` correlation of `eff_K` falls to **0.07**. So after the horizon bound, **the work
per column is essentially independent of instance size** — it is a function of `rho` alone. The naive
ceiling is the only K that grows with `n`, and it is the one the algorithm does not pay.

Non-monotone in `n` at fixed `Q`, which is worth noticing because it contradicts the intuition that
bigger is worse (median `eff_K`):

| Q | n<500 | 500–2k | 2k–10k | >=10k |
|---|---|---|---|---|
| 10 | 22.2 | 36.0 | 31.7 | **18.3** |
| 100 | 8.4 | 15.0 | 15.7 | **12.4** |
| 1,000 | 2.0 | 5.0 | 6.4 | 6.2 |

It peaks in the middle. At very large `n` the depot legs are large enough that the horizon binds on
distance before capacity binds on load, so `m` collapses to 1 and the layers empty out.

### 2.3 Only a handful of layers are competitive

Per column, over 18 sampled instances: the number of layers holding a *feasible* candidate, against
how many come within 1% and 10% of that column's winner.

| instance | Q | layers/col | within 1% | within 10% |
|---|---|---|---|---|
| `gil262_01` | 10 | 40.76 | 4.72 | 14.48 |
| `linhp318_01` | 10 | 7.04 | 5.32 | 7.04 |
| `gil262_04` | 100 | 34.20 | **1.80** | 5.24 |
| `linhp318_01` | 100 | 10.32 | **2.04** | 6.08 |
| `gil262_03` | 500 | 6.52 | **1.00** | 1.32 |
| `lin318_05` | 2,000 | 2.00 | 1.00 | 1.00 |

From `Q = 500` up, **exactly one layer is within 1% of the winner** while 2–7 are visited. And the
winner sits shallow: over 4,350 sampled columns, **74.7% are won at `k = 1`**, 82.9% at `k <= 2`,
87.2% at `k <= 3`, with a maximum of 15.

### 2.4 The layer partition degenerates at both ends

The fraction of live start *pairs* at a column that share a layer:

| Q | 10 | 50 | 100 | 500 | 2,000 | 10,000 |
|---|---|---|---|---|---|---|
| same-layer pairs | **0.00** | 0.02–0.05 | 0.02–0.06 | 0.26–0.37 | 0.43–0.70 | **1.00** |

At `Q = 10` every vendor's demand exceeds the capacity, so **every start sits in its own layer**. A
deque over a one-element set does nothing: the layered decoder there is an `O(n·K)` re-derivation of
Bellman with per-layer bookkeeping on top, which is exactly why it loses (median speedup 0.72×). At
`Q >= 10,000` there is one layer and the decoder is Vidal's Split unchanged.

**The layering earns its keep only in the middle**, and the middle is where the crossover sits.

---

## 3. The a priori bound — answering §7.2

### 3.1 The rule

A layer `k` at column `j` can hold a feasible arc only if *some* start in its window does. The
window is `W_k(j) = { i : (k-1)Q < L[j] − L[i] <= kQ }`, and within it the path out

```
    path_out(i,j) = d(depot, v_{i+1}) + (sumDistance[j] − sumDistance[i+1])
```

is smallest for the **largest** index in the window, because a later start walks less of the tour. So
testing that one start decides the layer:

```
    K(j) = max { k :  path_out(last(W_k(j)), j) · k  <=  T_H }
```

This is sound and needs no new assumption. `path_out` is non-decreasing as the start moves back and
`k` grows with it, so once a layer fails, every deeper layer fails — the loop may stop, not merely
skip. It costs nothing to evaluate: the two-pointer structure already knows `last(W_k(j))`.

### 3.2 What it is worth

Measured against what the solver actually does (`PTVRP_LAYERED_CONT_FIX`):

| instance | Q | `eff_K` | `K_allocated` | **`K(j)` bound** | gain vs `eff_K` |
|---|---|---|---|---|---|
| `gil262_01` | 10 | 133.40 | 163 | **21.17** | **6.3×** |
| `linhp318_01` | 10 | 30.31 | 48 | **5.68** | **5.3×** |
| `pr299_01` | 10 | 24.09 | 37 | **5.18** | **4.7×** |
| `d198_03` | 50 | 21.04 | 33 | 20.51 | 1.03× |
| `gil262_04` | 100 | 31.64 | 51 | 31.64 | 1.00× |
| `pr152_03` | 100 | 5.33 | 8 | 5.34 | 1.00× |
| `kroA200_03` | 500 | 4.01 | 6 | 4.01 | 1.00× |
| `lin318_07` | 10,000 | 1.00 | 1 | 1.00 | 1.00× |

Median over the sample: **`K(j) / eff_K` = 1.00**.

Two comparisons, and they must not be mixed. `eff_K` and the `K(j)` column above are both *means
over columns*; `K_allocated` and `Kjmax` are both *maxima over columns*. Compared like with like:

- **mean against mean** — `K(j)` versus `eff_K` — is where the gain is, and it is **4–6× at
  `Q = 10`, 1.00× from `Q = 50` up**;
- **max against max** — `Kjmax` versus `K_allocated` — is **1.00 everywhere** (37/37, 49/48,
  166/163, 13/13, 51/51, …). The bound never allocates fewer layers.

So the bound does not shrink the allocation. It shrinks the **per-column visits**, and only where the
global frontier is loose — which is exactly the low-capacity regime where the layered decoder is
slowest and loses to Bellman. **It attacks the worst case, not the median, and the question §7.2 asks
is closed in the negative for `Q >= 50`.**

What it does supply is the other thing §7.2 wanted: a bound *computable before the column is
scanned* rather than advanced during it.

### 3.3 The closed form was already there

Within layer `k` the window spans more than `(k-1)Q` of demand, hence at least `(k-1)Q/qbar` vendors,
hence a path out of at least that many mean arcs `c`. The bound `path_out · k <= T_H` becomes

```
    k · (k-1) · Q · c / qbar  <=  T_H      =>      K = O( sqrt( rho · T_H / c ) )
```

which is `NOTES.md` §5.4's `eff_K ≈ L·rho` with the quadratic solved. **So §5.4's estimate and
§7.2's missing bound are the same object.** What §5.4 lacked was not a formula but the observation
that the same argument, applied per column with the real window rather than with averages, is exact
enough to run in the solver.

One caveat on §5.4 the ladder exposes: its `B·K ≈ T_H/c` is not constant across capacity. Measured
`K·B·c/T_H` runs **0.105 at `Q = 10` to 0.838 at `Q = 100,000`**, an 8× drift. The median 0.46 in
`NOTES.md` is an average over a moving quantity.

---

## 4. Can separability be restored across layers?

### 4.1 No, and the obstruction is exact

Write `Delta(i,i'; j,j') = c(i,j) + c(i',j') − c(i,j') − c(i',j)` for `i < i'`, `j < j'`.

**A matrix is separable if and only if `Delta` vanishes on every 2×2 submatrix** (`NOTES.md` §5.7;
forward is algebra, backward sets `A[i] = c(i,j0) − c(i0,j0)`, `B[j] = c(i0,j)`). So restoring
separability on a set of predecessors *means* making `Delta` vanish on it. There is no other route —
no change of variables, no reweighting, no potential function — because the condition is an identity
on the cost values themselves, not on how they are written.

On the PT-VRP matrix `Delta` takes both signs, measured over 200,000 random 4-tuples satisfying `A`
non-increasing, `B` non-decreasing, `d >= 0`:

```
    m = max(1, ceil(l/Q))     Delta > 0 : 43,382     Delta < 0 : 135,578     = 0 : 20,480
```

A partition that restores separability must therefore put every sign-changing 4-tuple across its
parts. Conditioning on `m` does exactly that, and **it is the coarsest such partition that works by
construction**, because `Delta == 0` inside a part requires the multiplier to be constant on that
part: with `m` constant `= k`, `c = k·A[i] + k·B[j]` and `Delta == 0` identically; with `m` taking
two values on a part, a 4-tuple straddling them has `Delta != 0` by the scan above.

**So layers cannot be merged while keeping separability. That question is closed.**

### 4.2 What survives: one line per layer

The useful structure is not separability but this. At column `j`, the candidates from layer `k` are
the starts in `W_k(j)`, and

```
    value(i, j)  =  p[i] + k·(A[i] + B[j])  =  key(i,k)  +  k·B[j]
```

The `i`-dependence and the `j`-dependence are *already* split inside the layer — that is the
separability the layer has. Taking the minimum over the window:

```
    best_k(j)  =  minkey_k(j)  +  k · B[j]
```

**Each layer contributes exactly one affine function of `t = B[j]`, with slope `k` and intercept
`minkey_k(j)`.** The DP's per-column answer is

```
    p[j]  =  min over k  [ minkey_k(j) + k·t ],       t = B[j]
```

the **lower envelope of at most `K` lines**, evaluated at a parameter `t` that is non-decreasing in
`j` — the same monotonicity of `B` the solver already checks and reports.

This is worth stating because it reframes the loop. The decoder currently evaluates all `K` lines and
takes the minimum. That is a linear scan of an envelope query.

### 4.3 Cross-layer order flips at most once

For two layers `k1 < k2`,

```
    best_{k1}(j) − best_{k2}(j)  =  [minkey_{k1} − minkey_{k2}]  +  (k1 − k2)·t
```

is affine in `t` with **negative** slope. Holding the intercepts fixed, it changes sign **at most
once**, at `t* = (minkey_{k2} − minkey_{k1}) / (k2 − k1)`.

Two consequences:

- **Why no permanent ranking exists** (§4.1, restated concretely): a deque needs "worse now implies
  worse forever", and here the order genuinely inverts at `t*`. Not an artefact.
- **Why a kinetic one does:** single-crossing is exactly the hypothesis of kinetic tournaments and
  Li Chao trees. Absent intercept changes, the argmin layer is **non-increasing in `j`** — as `t`
  grows the smallest slope wins. The measurement agrees: 74.7% of columns are won at `k = 1` and
  87.2% at `k <= 3` (§2.3).

The blocker is that the intercepts are not fixed. `minkey_k(j)` changes as starts enter and leave the
window, and — per `pop.md` — the minimum-key entry may be horizon-infeasible, so the per-layer
summary is not one number but "the cheapest *feasible* entry", which is a Pareto query over
`(key, At)`.

### 4.4 The relaxation *is* Monge, and the ceiling is the only reason the real thing is not

Replace the integer multiplier by `m̃ = l/Q`, `l = L[j] − L[i]`. Then, expanding,

```
    Q · Delta  =  (L[j'] − L[j]) · (A[i'] − A[i])  +  (L[i'] − L[i]) · (B[j] − B[j'])
```

Both load differences are `>= 0`. `A[i'] − A[i] <= 0` when `A` is non-increasing and
`B[j] − B[j'] <= 0` when `B` is non-decreasing — the two conditions the layered solvers already test
at start-up and report as the monotonicity warning. Hence **`Delta <= 0` everywhere: the relaxed
matrix is Monge**, and SMAWK/LARSCH applies to it.

The scan confirms it and localises the obstruction:

| multiplier | `Delta > 0` | `Delta < 0` | `= 0` | verdict |
|---|---|---|---|---|
| `max(1, ceil(l/Q))` — the real one | 43,382 | 135,578 | 20,480 | **neither** |
| `l/Q` | **0** | 199,494 | 0 | **Monge** |
| `max(1, l/Q)` | **0** | 196,551 | 2,878 | **Monge** |
| `1 + l/Q` | **0** | 199,409 | 0 | **Monge** |
| `ceil(l/Q)` — no floor | 43,821 | 135,014 | 20,604 | **neither** |

Read the last two rows together. **The `max(1, ·)` floor is harmless; the ceiling is the whole
problem.** `NOTES.md` §5.7 says the floor "does not rescue it", which is true but understates the
result: the floor was never the difficulty.

And the relaxation sandwiches the truth, since `x <= ceil(x) < x + 1` and `m = 1 > l/Q` when `l < Q`:

```
    c̃(i,j)  <=  c(i,j)  <  c̃(i,j) + d(i,j)
```

By induction the relaxed DP satisfies `p̃[j] <= p[j]` at every column (same feasible set, smaller
costs). So **`p̃` is a valid lower bound computable in `O(n)`** by LARSCH, and the error at any arc is
at most one traversal of the route. That is a certificate, not a heuristic: a layer whose relaxed
best already exceeds the incumbent cannot win, and may be skipped exactly.

Caveat, and it is the same one as everywhere else in this repository: the Monge proof above **needs
`A` non-increasing and `B` non-decreasing**, which is the triangle inequality on durations. Off
metric data the relaxation is not Monge either, and this whole route closes. See `pop.md` §12.

---

## 5. Where the `O(n·K)` barrier really is

### 5.1 Skipping dead layers is a constant

`NOTES.md` §8.1 proposes tracking live layers. §2.3 sizes it: median `eff_K / max_layer_used` is
1.89, and per column only 1–5 layers of 2–40 are within 1% of the winner. Combined with §3.2's bound
at low `Q`, the realistic ceiling on this line is **2–6×**, concentrated at `Q <= 20`.

### 5.2 The envelope query is also a constant

§4.2 says the per-column scan is an envelope query, answerable in `O(log K)` with a kinetic
tournament. But count both halves of the work:

```
    per-column queries    Sum_j K(j)              = n · eff_K       <- the O(log K) idea attacks this
    window insertions     Sum_i (layers i enters) = n · K-ish       <- untouched
```

Each start enters each layer's deque at most once (the two-pointer partition), so the insertion term
is `O(n·K)` regardless of how the query is answered. Replacing the query with an envelope takes
`2·n·eff_K` to `n·eff_K + n·log K` — **a factor of about 2, not an exponent.**

### 5.3 The actual barrier

To beat `O(n·K)` you must **avoid materialising the `(start, layer)` pairs at all**. The windows are
determined by one array `L` and the thresholds `L[j] − kQ` for `k = 1..K`. As `j` advances by one,
*every* threshold shifts by the same `q_j`. So in the **load domain** the `K` window boundaries move
in lockstep; in the **index domain** they do not, because the demands are unequal.

That suggests where to look:

- **Uniform demands.** If `q_i ≡ q` then `L[i] = i·q`, every window is an index interval of fixed
  length `Q/q`, and all `K` of them slide by exactly one index per column. The whole layer structure
  is then one sliding window plus `K` offsets, which is representable in `O(1)` space per layer and
  plausibly `O(n log n)` overall. **This special case is not worked out and is the most promising
  concrete question in this file.**
- **General demands.** The structure is `K` predecessor queries into `L` at an arithmetic progression
  of thresholds, all shifting by the same amount each step. Whether that admits a representation
  sublinear in `K` is, as far as we know, open.

Until one of those lands, `O(n·K)` with `K = O(sqrt(rho·T_H/c))` is the honest complexity, and
`min(B, K) = O(sqrt(T_H/c))` (`NOTES.md` §5.4) is the honest bound on *whichever decoder you pick*.

### 5.4 Which is why the cheapest real win is not an algorithm

`B·K = Theta(T_H/c)` means the two decoders cannot both be slow. Choosing the right one per instance
is free and is worth more than either optimisation above:

- layered is faster on **629 of 788** feasible instances with `n >= 3,000`, by up to **50×**
  (2,062 of 3,060 over all feasible instances, but below `n = 3,000` the clock is start-up);
- Bellman is faster on the other 159, by a median of 1.23× and at most 2.3×;
- the switch is at `eff_K / B ≈ 0.25`, and since `eff_K/B ≈ rho` (measured `(K/B)/rho` = 0.91–1.07
  for `Q <= 5,000`), that is `rho ≈ 0.25`;
- measured directly: layered never wins above `rho = 0.26`, Bellman never wins below `rho = 0.128`.

`rho = qbar/Q` is two header fields and one pass over the demand column. **A three-line selector
captures most of the available speed**, and needs no new proof, because both decoders are already
exact (§6 item 1).

---

## 6. To do, in order

1. **Decoder selector on `rho`.** Compute `rho = qbar/Q` at load time; dispatch to `PTVRP_CONT_FIX`
   when `rho > 0.25`, `PTVRP_LAYERED_SAFE` otherwise. Both are exact and unconditional, so this
   cannot change an answer — verify with a sweep that costs are identical to the current ones, and
   report the wall-clock saving. **Cheapest item here and the largest measured win.** Expect the
   `Q <= 50` instances (about 500 of 3,150) to drop by ~25% and nothing else to move.

2. **Per-layer path-out bound (§3.1).** In the layer loop, stop advancing `k` when
   `path_out(last(W_k(j)), j) · k > T_H`. One condition, no new state, sound by the same argument as
   the existing path-out frontier. Expect **4–6× fewer layer iterations at `Q = 10`, nothing above
   `Q = 50`**. Verify: `LAYER ITERATIONS` falls on the low-`Q` folders, costs unchanged on all 3,150.

3. **Report `K(j)` as a diagnostic first.** Before changing the loop, print `sum_j K(j) / n` beside
   `eff_K` so the gain is measured on the full benchmark rather than on the 18 instances here. This
   is how to find out whether §3.2's "nothing above `Q = 50`" holds generally.

4. **Live-layer skipping (`NOTES.md` §8.1).** Now bounded by §2.3's measurement rather than by the
   single-instance guess that §10.2 corrected: expect **1.9× median**, up to 5× at p90. Do it after
   item 2, since item 2 removes some of the same work and the two gains are not additive.

5. **Relaxation-based pruning (§4.4).** Run the `O(n)` LARSCH DP on the Monge relaxation first, use
   `p̃[j]` as a lower bound, and skip any layer whose relaxed best exceeds the incumbent. Exact by
   §4.4's sandwich. This is the only item that needs new machinery, and it should be attempted only
   if items 2 and 4 disappoint — its value is that it prunes on a *certificate*, so it degrades
   gracefully where the heuristics do not.

6. **Uniform-demand special case (§5.3).** The one open question here that could change the
   exponent. Worth a day of thought before any code.

Not on the list, deliberately: merging layers (§4.1 closes it), and anything that assumes `A`
non-increasing without checking (`pop.md`).

---

## 7. Reproducing

The measurements in §2–§4 come from two scripts, neither committed:

```bash
# §2, §5.4 : the five K's, their correlations, K against B, the crossover
#            reads the nine root sweep_*.csv plus the instance headers
python3 kb_ab.py

# §2.3, §2.4, §3.2, §4.1, §4.4 : the Monge scans and the per-instance experiments
python3 kb_c.py
```

`kb_c.py` runs an independent `O(n²)` Python DP with the path-out stop and checks its cost against
`sweep_PTVRP_CONT.csv` on every sampled instance (18/18 agree), so the a priori bound and the layer
statistics are measured against a solver-independent reference.

The Monge scans draw 200,000 random 4-tuples per multiplier under `A` non-increasing, `B`
non-decreasing, `d >= 0` — the same generator as `monge_witness.py`, extended to the relaxations.

---

## 8. The math, for later

Collected so the pieces are in one place, including the ones that did not lead anywhere yet.

**Separability.** `c` is separable on a set iff `Delta == 0` on every 2×2 inside it. `Delta` has both
signs on the PT-VRP matrix. Conditioning on `m` is therefore not one repair among several; it is the
coarsest partition that can work, since a part carrying two values of `m` contains a non-zero
`Delta`.

**The layer as a line.** `best_k(j) = minkey_k(j) + k·B[j]`, slope `k`, parameter `t = B[j]`
non-decreasing. `p[j]` is the lower envelope at `t`. Cross-layer differences are affine in `t` with
slope `k1 − k2 < 0`: single-crossing at `t* = (minkey_{k2} − minkey_{k1})/(k2 − k1)`.

**Monge relaxation.** With `m̃ = l/Q`,

```
    Q·Delta  =  (L[j'] − L[j])·(A[i'] − A[i])  +  (L[i'] − L[i])·(B[j] − B[j'])  <=  0
```

given `A` non-increasing and `B` non-decreasing. `c̃ <= c < c̃ + d`, so `p̃ <= p` pointwise and `p̃` is
an `O(n)` lower bound. The ceiling alone destroys this; the `max(1,·)` floor does not.

**Cross-layer permanent dominance, for reference.** For `i1 > i2`, `i1` dominates `i2` at column `j`
iff

```
    p[i1] − p[i2]  <=  delta(j)·(A[i1] + B[j])  +  m2(j)·(A[i2] − A[i1])
```

with `m2(j) = m(i2,j)`, `m1(j) = m(i1,j)`, `delta(j) = m2 − m1 ∈ { floor(Δ/Q), ceil(Δ/Q) }` for
`Δ = L[i1] − L[i2]`. Making this permanent requires minimising the right-hand side over all future
`j`: the first term is minimised at the current `j` (`B` increasing, `delta >= floor(Δ/Q)`), the
second at the current `m2` when `A[i2] >= A[i1]` and at `m2(n)` otherwise. Note `delta = 0` reduces
it to the ordinary per-layer key comparison — **the entire cross-layer gain lives in `delta >= 1`.**
Measured with that sufficient condition, survivors are 6.1% of live starts, and 0.61× the number the
per-layer rule keeps; but the rule is conservative and at `Q >= 2,000`, where there is only one
layer, it keeps *more* than the per-layer minimum rather than fewer. It is recorded here because the
`delta = 0` observation explains why: with one layer there is no `delta >= 1` pair to exploit, and
permanence is then a strictly stronger demand than the deque's per-column minimum.

**What is not known.** Whether the full matrix is totally monotone (weaker than Monge, so §4.1 does
not exclude it — `NOTES.md` §8.3 parks this); whether the uniform-demand case admits `O(n log n)`
(§5.3); and whether the `K` lockstep predecessor queries of §5.3 have a sublinear-in-`K`
representation for general demands.
