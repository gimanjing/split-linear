# The pop

**What it is:** the deque's back-eviction discards a start that is more expensive. Feasibility is not
in that comparison. When the cheaper start does not fit the horizon and the discarded one did, the
only usable candidate is gone.

**What it is not:** it is not revival (`revival.md`). Revival is a *pruning* defect — a start dropped
too early by a bound that can go back down. This is an *eviction* defect — a start dropped
permanently by a rule that only looks at cost. The two are independent: §11.

**What triggers it:** `A` not being non-increasing, which means the triangle inequality failing on the
leg home. Nothing else. Not the multiplier, not the layering, not the capacity.

**The fix:** evict only on joint dominance, `key(new) <= key(old)` **and** `At[new] <= At[old]`.
Under the triangle inequality the second clause is automatic, so it is free on metric data: §8.

Everything below is produced by the code in this repository. Every table and trace is copied from
real output or from `check_pop.py`; nothing is illustrative. Commands are in §13.

---

## 1. Setting and notation

A giant tour `v_1 … v_n` is fixed. Split chooses where to cut it. Write `p[j]` for the cheapest way
to cover `v_1 … v_j`, and let the arc `(i,j]` be the route serving `v_{i+1} … v_j`.

```
sumDistance[j] = sum_{t=1}^{j-1} d(v_t, v_{t+1})        the path along the tour, depot excluded
A[i]           = d(depot, v_{i+1}) - sumDistance[i+1]    the start's contribution
B[j]           = sumDistance[j] + d(v_j, depot)          the endpoint's contribution
d(i,j)         = A[i] + B[j]                             one trip of the template
```

`At` and `Bt` are the same quantities in time units (`/speed`, plus service time per vendor). For the
PT-VRP the template is executed

```
m(i,j) = max(1, ceil((L[j] - L[i]) / Q))          L = cumulative demand
```

times, so

```
cost(i,j)     = d(i,j)  * m(i,j)
duration(i,j) = tau(i,j) * m(i,j),   tau(i,j) = At[i] + Bt[j]
feasible      <=>  tau(i,j) * m(i,j) <= T_H
```

Setting `Q` above the total demand gives `m ≡ 1` and the whole thing collapses to a plain
duration-constrained CVRP. **Everything in this document survives that collapse.** §6 is an instance
where it does.

### 1.1 Separability is the whole reason a deque is possible

`d(i,j) = A[i] + B[j]` — the cost of an arc splits into a term that depends only on the start and a
term that depends only on the endpoint. This is Vidal's Property 2, and its consequence is that for
two starts compared **at the same endpoint**, the endpoint term cancels:

```
cost(i1,j) - cost(i2,j) = A[i1] - A[i2]        independent of j
```

So "`i1` is cheaper than `i2`" is a property of `i1` and `i2` alone, decided once and valid for
every future `j`. That is what licenses a monotone structure: an ordering established now cannot be
overturned later.

### 1.2 Layering restores separability under the multiplier

With `m(i,j)` in the cost the difference no longer cancels — `m` depends on both ends. The layered
decoder restores it by partitioning arcs on `k = m(i,j)` and running one deque per layer. Inside
layer `k` the cost is

```
p[i] + k*A[i] + k*B[j]
```

so with

```
key(i,k) = p[i] + k*A[i]              <- the only i-dependent part
```

we again get `cost_k(i1,j) − cost_k(i2,j) = key(i1,k) − key(i2,k)`, independent of `j`. Each layer
has its own deque and its own `nextAdd[k]`, so an eviction in layer `k` is an eviction from layer `k`
only. All the reasoning below is inside a fixed layer.

---

## 2. There are two j-independent rankings, not one

The cost ranking is `key(·,k)`. It is the one the deque uses. But feasibility has a j-independent
ranking too, and it is a *different* one.

Feasibility at column `j` in layer `k` is

```
(At[i] + Bt[j]) * k <= T_H     <=>     At[i] <= T_H/k - Bt[j]
```

The right-hand side depends only on `j` and `k`. So comparing two starts at the same `j`, `Bt[j]`
cancels exactly as it cancelled in the cost:

```
At[i1] <= At[i2]    =>    wherever i2 fits, i1 fits — at every j, for ever
```

**Two orders, both permanent, both legitimate, and in general they disagree.**

| | ranks by | small is | permanent? |
|---|---|---|---|
| `key(i,k) = p[i] + k·A[i]` | cost | cheaper | yes, `B[j]` cancels |
| `At[i]` | feasibility | easier to fit | yes, `Bt[j]` cancels |

The deque is a monotone structure under `key` alone. It has no representation of the second order.

---

## 3. Dominance, and when eviction is sound

**Definition.** In layer `k`, start `i1` *dominates* start `i2` if for every column `j` at which both
are live:

* (a) `i2` feasible ⟹ `i1` feasible, and
* (b) `cost_k(i1,j) ≤ cost_k(i2,j)`.

By §1.1, (b) ⟺ `key(i1,k) ≤ key(i2,k)`. By §2, `At[i1] ≤ At[i2]` ⟹ (a).

**Theorem (sound eviction).** Discarding `i2` on the arrival of `i1` is sound iff `i1` dominates `i2`,
i.e.

```
    key(i1,k) <= key(i2,k)     AND     At[i1] <= At[i2]
```

Both clauses are `j`-free, so a pop justified by them is justified for every future column — which is
what a permanent discard requires.

**The rule actually implemented** is clause (b) only. In the layered solvers
(`Split_Layered_PTVRP`, `..._CONT`, `..._CONT_FIX`) it is written directly:

```cpp
while (!dq.empty() && key(dq.back(), k) >= ki)
    dq.pop_back() ;
```

In `Split_Linear` it is the same comparison expanded. `dominatesRight(i,j)` reads

```cpp
potential[j] + cli[j+1].dreturn  <  potential[i] + cli[i+1].dreturn
                                    + sumDistance[j+1] - sumDistance[i+1] + 0.0001
```

which rearranges to `p[j] + A[j] < p[i] + A[i]`, i.e. `key(j,1) < key(i,1)`.

**Clause (a) is not checked anywhere. It is assumed.** §4 shows that in Vidal's setting the
assumption is discharged for free, and §5 shows what discharges it once a horizon replaces the
capacity.

---

## 4. Why capacity-only Split never needed the check

In Vidal's Split the only constraint is load. The set of feasible starts at column `j` is

```
    { i : L[j] - L[i] <= Q }
```

and `L` is non-decreasing because demands are non-negative. So the feasible set is **an up-set in
index order** — a suffix `{i : i >= threshold(j)}` — and the threshold is what the front pointer
chases.

The back-eviction only ever pops entries whose index is *smaller* than the newcomer's (entries are
pushed in increasing `i`, and the pop takes `dq.back()`). On a suffix, a larger index is feasible
whenever a smaller one is. So clause (a) holds automatically, for free, **with no assumption
whatsoever about the distances**.

> **This is why Vidal's Split is correct on non-metric, asymmetric, rounded, or adversarial data.**
> Its feasibility predicate is positional. The triangle inequality is nowhere in the argument.

Two jobs, cleanly separated:

```
front pop  : "this start can never be feasible again"     a feasibility statement
back pop   : "this start can never be cheapest again"     a cost statement
```

They are independent precisely because feasibility is a suffix in index order. The back pop was
always allowed to ignore feasibility because the front pop owned it entirely.

---

## 5. Why a duration limit breaks the separation

Replace the load constraint with a horizon. Even with `m ≡ 1`, feasibility at column `j` is

```
    { i : At[i] <= T_H - Bt[j] }          (= { i : A[i] <= T_H - B[j] } when time is distance)
```

This is a **sublevel set of `A`**, not a set of indices. It is a suffix in index order **iff `A` is
non-increasing**.

**Proposition.** The triangle inequality implies `A` is non-increasing.

```
A[i+1] - A[i] = d(0, v_{i+2}) - d(0, v_{i+1}) - d(v_{i+1}, v_{i+2})  <=  0
```

by `d(0, v_{i+2}) <= d(0, v_{i+1}) + d(v_{i+1}, v_{i+2})`.

So on metric data the duration constraint *also* yields an index-suffix, clause (a) is again free, and
the cost-only pop is sound. The triangle inequality had been silently doing the work the load
monotonicity used to do.

Break it and the feasible set is scattered. A start with a large index can fit *less* easily than one
with a small index, the back pop's implicit premise fails, and the deque discards usable candidates.

**In one sentence:** cost-only back-eviction is sound whenever the feasible start set is an up-set in
index order. For capacity that is unconditional. For a horizon it is exactly the triangle inequality.

---

## 6. The minimal example: no multiplier, no layers, plain duration-bounded CVRP

`Instances/Counterexamples/ce_dvrp_unsafe_pop_4v.gt`

```
CAPACITY 10000    MAX_ROUTE 160        (unit demands, so m(sigma) = 1 on every route)

  vendor   home   to next
       1     54        13
       2     82         8
       3     21        14
       4     83         -
```

Capacity is 10,000 against a total demand of 4. `K FULL TOUR : 1`. There is one layer. This is a
distance-constrained CVRP with four customers, and the defect is fully present.

**Where the triangle inequality fails** (two places, one of which bites):

```
i=0 :  d(0,v_2) = 82  >  d(0,v_1) + d(v_1,v_2) = 54 + 13 = 67
i=2 :  d(0,v_4) = 83  >  d(0,v_3) + d(v_3,v_4) = 21 + 14 = 35
```

so `A = [54, 69, 0, 48]` climbs twice instead of descending.

**The state at the last column.** `j = 4`, `k = 1`, `B[4] = 118`, `T_H = 160`:

| start | `p[i]` | `A[i]` | `key = p + A` | route `A[i] + 118` | fits 160? |
|---|---|---|---|---|---|
| 0 | 0 | 54 | 54 | 172 | no |
| 1 | 108 | 69 | 177 | 187 | no |
| 2 | 149 | **0** | 149 | **118** | **yes** |
| 3 | 96 | 48 | **144** | 166 | no |

Exactly one start is feasible, and it is not the cheapest. The two orders of §2 disagree at the one
column where it matters.

**The eviction trace:**

```
add 0  ->  [0]
add 1  ->  key 177 > 54,  no pop                                  ->  [0,1]
add 2  ->  key 149 <= 177, pop 1.   At[2]=0  <= At[1]=69   safe   ->  [0,2]
add 3  ->  key 144 <= 149, pop 2?   At[3]=48 >  At[2]=0    UNSAFE
```

Unguarded the deque becomes `[0,3]`. Both entries exceed 160, both are skipped as infeasible, the
scan exhausts, and `p[4]` never receives a label.

```
$ Program/split .../ce_dvrp_unsafe_pop_4v.gt -solver PTVRP_LAYERED_CONT_FIX -trace 1

  +-- p[4] : path-out frontier at 0, layers 1..1   (full-route frontier would stand at 2)

INFEASIBLE SKIPS : 2
UNSAFE POPS : 1
ERROR : no Split solution has been propagated until the last node
```

With the guard:

```
$ Program/split .../ce_dvrp_unsafe_pop_4v.gt -solver PTVRP_LAYERED_SAFE -trace 1

  +-- p[4] : path-out frontier at 0, layers 1..1   (full-route frontier would stand at 2)
  |  layer k=1 : starts [0,4)  deque [ 0 2 3 ]  first feasible=2   cand = p[2] + 1*d(2,4) = 267

BLOCKED POPS : 1
UNSORTED QUERIES : 1
SOLUTION COST : 267
```

**Scoreboard** (exhaustive enumeration gives 267: `{1,2}` at 149, `{3,4}` at 118):

| solver | result | |
|---|---|---|
| `PTVRP`, `PTVRP_CONT`, `PTVRP_CONT_FIX` | **267** | Bellman — no deque, nothing to evict |
| `PTVRP_LINEAR` | 262 | **infeasible**, and it has no layers at all |
| `PTVRP_LAYERED` | 262 | **infeasible** — its template runs `tau = 166 > 160` |
| `PTVRP_LAYERED_CONT` | `NO SOLUTION` | `UNSAFE POPS = 1` |
| `PTVRP_LAYERED_CONT_FIX` | `NO SOLUTION` | `UNSAFE POPS = 1` |
| `PTVRP_LAYERED_SAFE` | **267** | `BLOCKED POPS = 1` |

Two things this file settles:

1. **The multiplier is irrelevant.** Every route has `m = 1`. `K FULL TOUR = 1`. The defect does not
   need the PT-VRP.
2. **Layering is irrelevant.** `PTVRP_LINEAR` is Vidal's single deque with the horizon substituted for
   the capacity in the front pop, no layers, and it returns 262 — a partition containing a route of
   duration 166 against a limit of 160, which it detects and reports itself.

   Its trace shows the same eviction doing the damage:

   ```
     +-- p[3] : queue holds [ 0 2 ]  front=0     ->  p[3] = 96
     +-- p[4] : queue holds [ 3 ]    front=3     ->  p[4] = 262

   ERROR : template 1 violates the horizon constraint (166 > 160)
   ```

   Start 2 left by the back pop, start 0 by the front pop, and only the infeasible 3 remained. Block
   the back pop and the queue at `p[4]` is `[0,2]`; the front pop drops 0 as it already does, leaves
   2, and the answer is 267.

   `PTVRP_LINEAR` carries a second, independent unsoundness — its dominance functions are blind to
   the multiplier. On this instance `m ≡ 1`, so that one is inert, which is exactly what makes the
   attribution clean.

What is *not* irrelevant: a duration constraint, a cost-ranked deque, and a geometry where cheapest
and easiest-to-get-home disagree.

---

## 7. The same failure with the multiplier present

`Instances/Counterexamples/ce_unsafe_pop_5v.gt` — the PT-VRP version, failing inside layer `k = 2`.

```
CAPACITY 10   MAX_ROUTE 200

  vendor   demand   home   to next
       1        5     79         1
       2       11     42         7
       3        2     74        12
       4        7      8         7
       5       11     77         -
```

`A = [79, 41, 66, −12, 50]`, violated at `i = 1` and `i = 3`. At column `j = 5`, layer `k = 2`:

| start | `p[i]` | `A[i]` | key | its route | fits `T_H = 200`? |
|---|---|---|---|---|---|
| 2 | 326 | 66 | 458 | `d=170`, `d·2=340` | no |
| 3 | 474 | −12 | **450** | `d=92`, `d·2=184` | **yes** |
| 4 | 296 | 50 | **396** | `d=154`, `d·2=308` | no |

```
add 2  ->  [2]                  key 458
add 3  ->  450 <= 458, pop 2    At[3] = -12 < At[2] = 66    safe
add 4  ->  396 <= 450, pop 3    At[4] =  50 > At[3] = -12   UNSAFE
```

Start 3 was feasible at 184 and `p[3] + 184 = 658` is the optimum. It was discarded by a start that
is cheaper on paper and does not fit.

| solver | result |
|---|---|
| `PTVRP`, `PTVRP_CONT`, `PTVRP_CONT_FIX`, `PTVRP_LAYERED` | **658** |
| `PTVRP_LAYERED_CONT`, `PTVRP_LAYERED_CONT_FIX` | `NO SOLUTION`, `UNSAFE POPS = 1` |
| `PTVRP_LAYERED_SAFE` | **658**, `BLOCKED POPS = 1` |

Note that fully-unsound `PTVRP_LAYERED` gets this one right and the *fixed* solvers get it wrong.
The unsound frontier happens to keep a start that the eviction would otherwise have needed. Nothing
follows from that except that the two defects are independent (§11).

---

## 8. The repair

Add clause (a) to the eviction. `Split_Layered_PTVRP_safe.cpp:144-159`:

```cpp
double ki = key(i, k) ;
while (!dq.empty() && key(dq.back(), k) >= ki)
{
    if (At[i] > At[dq.back()] + 1.e-9)
    {
        // cheaper but harder to fit : keep the incumbent, and the layer is no longer
        // sorted by key, so its query has to scan.
        blockedPops ++ ;
        sortedByKey[k] = 0 ;
        break ;
    }
    dq.pop_back() ;
}
dq.push_back(i) ;
```

Three things to note.

**It belongs in the eviction and could not have gone in the pruning.** A prune is permanent across
*future* columns, so it needs a quantity monotone in `j`. The full route time `tau(i,j)` is not
monotone in `j` — that is exactly revival. The only monotone bound available is the path out, which
excludes the leg home by construction. The eviction compares two starts at the *same* `j`, where
`Bt[j]` cancels, and `At[i]` is the one `j`-independent comparison that still accounts for the leg
home. There is nowhere else to put it.

**It is a `break`, not a `continue`.** Once a pop is blocked the loop stops popping, so entries further
back that the newcomer really does dominate stay in. The deque is then a *superset* of the Pareto set
(§9), never a subset — which is the direction that preserves correctness.

**The query has to change with it.** While the layer is key-sorted, "first feasible from the front" is
the cheapest feasible entry, which is the original O(1) query. Once a pop has been blocked that
ordering is gone, so the query scans the whole deque for the cheapest feasible entry.
`sortedByKey[k]` tracks the regime per layer, so a layer that never blocks never pays.

---

## 9. What the deque is, once the guard fires

The query the algorithm actually needs is

```
    argmin key(i,k)   subject to   At[i] <= T,        T = T_H/k - Bt[j]
```

**Proposition.** The optimum is attained on the Pareto-minimal set of the live starts under
`(key, At)`. *Proof:* if `i` is optimal and dominated by `i'`, then `At[i'] ≤ At[i] ≤ T` so `i'` is
feasible, and `key(i') ≤ key(i)`, so `i'` is optimal too. ∎

**Proposition.** Sort that Pareto set by `key` ascending. Then `At` is strictly descending (otherwise
one element would dominate another). So `{i : At[i] ≤ T}` is a **suffix** of that order, and the
answer is its first element — reachable by binary search on `At` in `O(log)`.

**Corollary.** When `A` is non-increasing, the ordinary deque *is* this Pareto set: its entries have
ascending index, hence ascending `key` (the deque invariant) and descending `At` (monotonicity). The
feasible entries are a suffix, and "first feasible from the front" is precisely the binary-search
answer, computed by linear scan. **The existing algorithm was always the Pareto structure — in the
special case where the Pareto set happens to be contiguous in index order.**

This says what a proper non-metric implementation would cost: maintaining a Pareto set under
insertion needs an ordered container rather than a deque, so `O(log)` per insert and per query
instead of amortised `O(1)`. A log factor, not a linear blowup.

**This is not implemented.** `PTVRP_LAYERED_SAFE` keeps the deque and falls back to a linear scan,
which is correct but asymptotically worse than necessary on data where the guard fires often. It was
built that way because the guard never fires on the benchmark (§10), so the fast path is what
matters and the slow path only has to be right. See §12.

---

## 10. What it costs

**On the benchmark — nothing, and not because it fixed instances.** 3,150 instances, `Instances 1–3`:

| | `..._CONT_FIX` | **`..._SAFE`** |
|---|---|---|
| exact vs. Bellman | 2,829 + 321, exact | 2,829 + 321, exact |
| `UNSAFE POPS` / `BLOCKED POPS` | 0 | **0** |
| `UNSORTED QUERIES` | — | **0** |
| median `eff_K` | 17.26765 | **17.26765** (identical to 5 d.p.) |
| total wall clock | 28.0 s | **28.0 s** |

`UNSORTED QUERIES = 0` means no layer ever left the fast path. The guard is free here because it
never had to act — the 1,214 triangle violations in the TSPLIB tours (0.277% of positions, always
exactly one unit) never once produced an eviction where the newcomer was harder to fit.

**On structurally non-metric data — it fires constantly and still costs little.**
`structural_violation.gt`, `n = 400`, legs home drawn independently of the tour arcs so 46% of
positions violate the triangle inequality by a median of 156 units:

| | `..._CONT_FIX` | `..._SAFE` |
|---|---|---|
| answer | 46,605 | 46,605 (both match Bellman) |
| `UNSAFE POPS` / `BLOCKED POPS` | 165 | 165 |
| `UNSORTED QUERIES` | — | 1,992 of 2,018 layer iterations (98.7%) |
| `LAYER ITERATIONS` | 2,018 | 2,018 |
| `INFEASIBLE SKIPS` | 1,024 | 1,322 (+29%) |

The guard blocks 165 pops and puts 98.7% of queries on the scanning path, and the counted extra work
is 298 additional infeasible entries skipped. `INFEASIBLE SKIPS` does not count the feasible entries
the unsorted query also walks past, so the true extra is somewhat larger — but it is bounded by the
deque length per query, and the deques are short. Layer iterations are identical, so the guard costs
nothing structurally. Wall clock at `n = 400` is dominated by process start-up and the two solvers
are indistinguishable.

**Most unsafe pops are harmless.** 165 of them here and the answer is unchanged. An unsafe pop only
changes the result when the discarded start was the best *feasible* one at some column — otherwise it
discards something that would have lost anyway. That is why the defect is easy to carry for years
without noticing: it needs a triangle violation, and then it needs that violation to land on the one
start that mattered.

**Over 60 structurally non-metric instances:**

| | result |
|---|---|
| instances with `UNSAFE POPS > 0` | 60 / 60 (5,304 pops) |
| `PTVRP_CONT_FIX` wrong | 0 / 60 |
| `PTVRP_LAYERED_CONT_FIX` wrong | 1 / 60 (oracle 88,554, returned 88,909) |
| **`PTVRP_LAYERED_SAFE` wrong** | **0 / 60** (5,205 pops blocked) |

---

## 11. Relation to revival

They are different defects in different parts of the algorithm, and neither implies the other.

| | revival (`revival.md`) | the pop (here) |
|---|---|---|
| where | the horizon **prune** — front pointer / frontier | the **eviction** — back pop |
| claim it assumes | `tau(i,j)` is non-decreasing in `j` | `At` is non-increasing in `i` |
| what breaks it | the leg home shrinking as `j` grows | the leg home growing as `i` grows |
| symptom | a start dropped too early, answer too expensive | a start discarded for good, answer too expensive or absent |
| the fix | prune on the path out (`Bout_t`) | evict on joint dominance (`At`) |
| fires on TSPLIB | 177 revived arcs on 103 of 3,150 instances, 0 answers changed | never, on any of the 3,150 |
| affects Bellman? | **yes** — the early stop is Bellman's | **no** — Bellman has no deque |

Both trace back to the same root — the triangle inequality failing — but they fail through different
mechanisms and need separate fixes, and each can be present without the other showing.

In §7, `PTVRP_LAYERED` — unsound on *both* counts — returns the right answer 658, while
`PTVRP_LAYERED_CONT_FIX` — sound prune, unsafe pop — returns nothing. The unsound prune happens to
leave a start alive that the eviction would otherwise have needed. Fixing one defect exposed the
other.

In §6 they show as two different symptoms on one file: with the unsound prune the answer is 262 with
a route of duration 166 against a limit of 160; with the sound prune and the unsafe pop there is no
answer at all.

The solver ladder, and what each rung assumes:

| solver | prune | eviction | assumes |
|---|---|---|---|
| `PTVRP_LAYERED` | full route (unsound) | cost only | `tau` monotone in `j` **and** `At` monotone in `i` |
| `PTVRP_LAYERED_CONT` | none | cost only | `At` monotone in `i` |
| `PTVRP_LAYERED_CONT_FIX` | path out (sound) | cost only | `At` monotone in `i` |
| **`PTVRP_LAYERED_SAFE`** | path out (sound) | joint | **nothing** |
| `PTVRP_CONT_FIX` (Bellman) | path out (sound) | — | **nothing** |

---

## 12. Scope

**Affected:** any Split whose feasibility test is not positional and whose deque evicts on cost alone.
That is every duration-constrained, time-window-free Split built on Vidal's template, with or without
a multiplier, with or without layering. `PTVRP_LINEAR` (§6) is the smallest instance of it: Vidal's
Split with the horizon substituted for the capacity in the front pop, one deque, no layers, no
multiplier in play.

**Not affected:** Vidal's original Split (§4). Its feasibility is cumulative load, which is an up-set
in index order unconditionally. No assumption about the distance matrix is anywhere in its proof.

**Not affected:** any Bellman / `O(n²)` Split. There is no eviction, so there is nothing to lose.

**The condition to check** before trusting a cost-only eviction is not "is the instance metric" but
the weaker, directly testable one:

```
    At[i+1] <= At[i]   for all i
```

The layered solvers compute this at start-up and print a `WARNING` line counting the positions that
break it — together with the `Bt[j+1] >= Bt[j]` positions that revival needs, since the two are
reported in one figure. On `ce_dvrp_unsafe_pop_4v` it reads `3`: two `At` climbs and one `Bt` dip.

An instance can be wildly non-metric elsewhere and still satisfy this, because only the leg home
enters `A`.

**Practical generators of the failure**, beyond rounded TSPLIB coordinates: asymmetric travel times,
time-dependent or congestion-scaled legs, a depot cost that is not a travel time (gate fees, driver
sign-off, mandatory rest at the depot), and any objective where `d` and `tau` are priced differently.
In all of them `A` and `At` can climb.

---

## 13. Reproducing

```bash
make -C Program

# §6 -- the four-vendor duration-bounded CVRP, no multiplier
for s in PTVRP_CONT PTVRP_CONT_FIX PTVRP_LINEAR PTVRP_LAYERED \
         PTVRP_LAYERED_CONT PTVRP_LAYERED_CONT_FIX PTVRP_LAYERED_SAFE ; do
  printf '%-26s ' $s
  Program/split Instances/Counterexamples/ce_dvrp_unsafe_pop_4v.gt -solver $s 2>&1 \
    | grep -oE "SOLUTION COST : [0-9.]+|UNSAFE POPS : [0-9]+|BLOCKED POPS : [0-9]+|no Split solution" \
    | tr '\n' ' ' ; echo
done

# the two traces of §6
Program/split Instances/Counterexamples/ce_dvrp_unsafe_pop_4v.gt -solver PTVRP_LAYERED_CONT_FIX -trace 1
Program/split Instances/Counterexamples/ce_dvrp_unsafe_pop_4v.gt -solver PTVRP_LAYERED_SAFE      -trace 1

# §7 -- the five-vendor PT-VRP version
Program/split Instances/Counterexamples/ce_unsafe_pop_5v.gt -solver PTVRP_LAYERED_SAFE

# §10 -- the structural instance and its counters
for s in PTVRP_CONT_FIX PTVRP_LAYERED_CONT_FIX PTVRP_LAYERED_SAFE ; do
  printf '%-26s ' $s
  Program/split Instances/Counterexamples/structural_violation.gt -solver $s 2>&1 \
    | grep -oE "SOLUTION COST : [0-9.]+|UNSAFE POPS : [0-9]+|BLOCKED POPS : [0-9]+|UNSORTED QUERIES : [0-9]+" \
    | tr '\n' ' ' ; echo
done

# §10 -- the benchmark row
python3 batch_run.py --dir "Instances/Instances 1" --dir "Instances/Instances 2" \
    --dir "Instances/Instances 3" --solver PTVRP_LAYERED_SAFE --timeout 0 --out sweep_layered_safe.csv
```

The `A` / `B` / `p` tables, the triangle-violation positions and the exhaustive optima in §6 and §7
are recomputed independently of the solver by `check_pop.py`, which parses the instance and runs the
`O(n²)` recursion in Python:

```bash
python3 check_pop.py Instances/Counterexamples/ce_dvrp_unsafe_pop_4v.gt \
                     Instances/Counterexamples/ce_unsafe_pop_5v.gt
```

---

## 14. Open

1. **The `O(log)` Pareto query of §9 is not implemented.** The fallback is a linear scan. It only
   matters on data where the guard fires often, which the benchmark is not, but the asymptotics of
   `PTVRP_LAYERED_SAFE` on non-metric data are currently worse than they need to be.

2. **Is the guard ever *necessary* on real data?** Zero blocked pops across 3,150 TSPLIB-derived
   instances. The failure needs a triangle violation *and* that violation to land on the decisive
   start. On rounded Euclidean data the violations are all one unit, which is apparently never enough.
   Whether a non-Euclidean published benchmark exists where it fires is untested.

3. **What do published duration-constrained Split implementations do?** The cost-only eviction is
   inherited verbatim from Vidal's code, where it is correct. Whether anyone carrying it into a
   duration-constrained setting added clause (a) is an open literature question, and it decides
   whether §3 is a technical note or a footnote.

4. **Time windows.** With time windows the feasibility predicate is not a sublevel set of a single
   scalar, so `At` is no longer a sufficient statistic and §3's clause (a) has no one-line form. What
   replaces it is not worked out here.
