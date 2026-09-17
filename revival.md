# Revival

**What it is:** a route that does not fit the time limit, followed by a *longer* route that does.

**When it happens:** whenever a route's duration is not a function of its distance — so the leg home
can be priced independently of the two legs that replace it. Independent integer rounding is the
cheapest example and is already present in every TSPLIB instance, but it is the *smallest* one.
Traffic that varies by hour, queueing at the depot, driver-hours rules and one-way networks break the
same assumption by far more than one unit (§1.1).

**Why it matters:** every Split implementation that stops scanning at the first time-infeasible route
assumes this cannot happen. When it can, the answer comes out wrong.

**Who is affected:** not Vidal's Split (§7). Any Split that stops on accumulated *time* rather than
accumulated *load* (§8).

**The fix:** two parts, because the violation has two sizes. Stop on the route *without* the leg home,
and evict from the deque only on **joint** dominance. Sound with no assumption about the instance,
nothing precomputed, 1.19x in Bellman and 1.11x in the layered decoder (§9).

**The other defect:** fixing the stop does not make the layered decoder unconditional. Its deque
evicts on cost alone, which is a separate unsound step with a separate fix. That one has its own
note: `pop.md`.

**The other defect:** fixing the stop does not make the layered decoder unconditional. Its deque
evicts on cost alone, which is a separate unsound step with a separate fix. That one has its own
note: `pop.md`.

Everything below is produced by the code in this repository. Every trace is copied from real output;
nothing is illustrative. Commands are in §9.

---

## 1. The problem in one picture

You are growing a route by adding one vendor at a time, and you stop when it no longer fits the
time limit `T_H`. Adding a vendor looks like this:

```
before:   depot ──→ … ──→ v_j ──────────────→ depot
after:    depot ──→ … ──→ v_j ──→ v_{j+1} ──→ depot
```

You **remove** one leg, `d(v_j, depot)`, and **add** two, `d(v_j, v_{j+1}) + d(v_{j+1}, depot)`.
So the duration changes by

```
    d(v_j, v_{j+1})  +  d(v_{j+1}, depot)  −  d(v_j, depot)
```

Normally that is positive — the **triangle inequality** says a detour via `v_{j+1}` cannot be shorter
than going straight home. Routes only get longer as you add vendors, so once you are over `T_H` you
stay over, and stopping is safe.

**But the triangle inequality is a statement about distance, and the constraint is on duration.** The
two coincide only when duration is a function of distance and it is the *same* function on every leg.
Break that and `d(v_j, depot)` can exceed `d(v_j, v_{j+1}) + d(v_{j+1}, depot)`. Adding a vendor then
makes the route *shorter*, and a route that was over the limit comes back under — after the scan has
already walked away.

That is revival.

### 1.1 What breaks it, in increasing order of size

| source | violation | exhibited here |
|---|---|---|
| **independent integer rounding** — TSPLIB rounds each distance to a whole number on its own | exactly **1 unit**, never more | all 3,150 benchmark instances (`revival_arcs.md`) |
| **time-dependent travel** — the leg home is driven at a different hour than the arcs replacing it, so it is priced at a different speed | unbounded | `structural_violation.gt` |
| **queueing or service at the depot**, folded into the return leg | unbounded | — |
| **driver-hours, breaks, shift ends** charged wherever the leg home falls | unbounded | — |
| **asymmetric networks** — one-way systems, turn restrictions, tolls, ferries | unbounded | — |

Rounding is the one worth *demonstrating* on, because it assumes nothing: it is already in the data,
and one unit is enough (§3). But it is the weakest form, and the distinction is not cosmetic.

**The size of the violation decides how much breaks.** A one-unit wobble defeats the early stop and
nothing else — the deque's eviction rule survives it on all 3,150 instances. A structural violation
defeats the eviction rule as well, and *that* failure returns `NO SOLUTION` on a feasible instance
(§9). So a solver hardened only against rounding is hardened against the case that never mattered.
Everything in §§2–8 below uses rounding because it is the cheapest witness; §9 fixes both.

---

## 2. First, a normal instance where nothing goes wrong

`Instances/ptvrp_demo_10v.gt` — 10 vendors, `Q = 10`, `T_H = 80`, travel time = distance.

| vendor | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|---|---|---|---|
| demand `q` | 3 | 4 | 3 | 5 | 2 | 4 | 3 | 5 | 4 | 3 |
| to depot `dreturn` | 10 | 12 | 15 | 11 | 9 | 14 | 18 | 16 | 13 | 10 |
| to next `dnext` | 4 | 5 | 6 | 5 | 7 | 6 | 5 | 4 | 5 | — |

The triangle inequality holds at every position, so **revival is impossible here**:

```
j=1: 10 <= 4+12=16      j=6: 14 <= 6+18=24
j=2: 12 <= 5+15=20      j=7: 18 <= 5+16=21
j=3: 15 <= 6+11=17      j=8: 16 <= 4+13=17
j=4: 11 <= 5+9=14       j=9: 13 <= 5+10=15
j=5:  9 <= 7+14=21
```

**Reading a route.** `(i, j]` means the route serving vendors `i+1 … j`. Its distance is
`dreturn[i+1] + dnext[i+1] + … + dnext[j-1] + dreturn[j]` — out to the first, along the chain, back
from the last. Its load is the demands summed. Its **trip count** is `m = ceil(load / Q)`, and it
must satisfy `d · m <= T_H`. Its **cost** is `d · m`.

Example, route `(0,3]` = vendors 1,2,3:

```
    d = dreturn[1] + dnext[1] + dnext[2] + dreturn[3] = 10 + 4 + 5 + 15 = 34
    q = 3 + 4 + 3 = 10        m = ceil(10/10) = 1
    d·m = 34 <= 80 ✓          cost = 34
```

The true optimum, by enumerating all 512 partitions: **149**, cutting after vendors 3, 4, 7, 9.

### 2.1 Bellman — the definition, computed

For each endpoint `j`, try every possible start `i` and keep the best. `p[j]` is the cheapest way to
serve vendors `1…j`.

```
  +-- p[3] : cheapest way to serve vendors 1..3
  |  start=0 : template(1..3)  d=34  q=10  m=1  cost=34*1=34   ->  p[0]=0 + 34 = 34   <- best so far
  |  start=1 : template(2..3)  d=32  q=7   m=1  cost=32*1=32   ->  p[1]=20 + 32 = 52
  |  start=2 : template(3..3)  d=30  q=3   m=1  cost=30*1=30   ->  p[2]=26 + 30 = 56
  +-- p[3] = 34  (predecessor 0)
```

Three starts tried, best is `start=0`. Nothing clever happens. This is the reference answer:
**149**, and it is correct.

### 2.2 Vidal's deque — and why it breaks here

The deque keeps candidate starts ranked by their fixed cost, and trusts its **front**. That is valid
in the classic CVRP because of Vidal's **Property 2**: the cost difference between two starts is the
same for every endpoint, so a start that is worse once is worse forever and can be thrown away.

In PT-VRP the cost is `m(i,j) · d(i,j)`, and `m` depends on **both** ends. The ranking is no longer
stable. Watch it fail, from the real trace:

```
  +-- p[3] : queue holds [ 0 1 2 ]  front=0
  |  front=0 : d=34  m=1  cost=34*1   ->  p[3] = 34

  +-- p[4] : queue holds [ 0 1 3 ]  front=0
  |  front=0 : d=36  m=2  cost=36*2   ->  p[4] = 72
  |  *** but start=3 gives 56 -- the deque trusted the wrong front (off by 16) ***
```

At `p[3]`, start 0 was genuinely best. One vendor later the load crosses `Q = 10`, `m` jumps from 1
to 2, and start 0's cost **doubles** — while start 3's does not. The ranking inverted. The deque had
already committed.

Final answer: **182**, which is 22% worse than 149.

### 2.3 Layered — one deque per trip count

The repair: run a **separate deque for each value of `m`**. Inside one layer `m` is a fixed number,
so the cost is `k · d(i,j) = k·A[i] + k·B[j]` — separable again, Property 2 holds again, the deque is
valid again.

Same column, from the real trace:

```
  +-- p[4] : feasible starts begin at 0, layers 1..2
  |  layer k=1 : starts [2,4)  deque [ 3 ]    front=3   cand = p[3] + 1*d(3,4) = 56   <- best so far
  |  layer k=2 : starts [0,2)  deque [ 0 1 ]  front=0   cand = p[0] + 2*d(0,4) = 72
```

Two things to notice:

1. **The windows `[2,4)` and `[0,2)` are disjoint and cover `[0,4)` exactly.** Every start lives in
   exactly one layer — the layers *partition* the starts, they do not duplicate them. That is why
   the cost is `O(nK)` and not `O(n²K)`.
2. Layer 1 finds 56, which is what the deque missed. The answer is **149**, matching Bellman.

Which start goes in which layer is decided by **load** — `m = ceil(load/Q)` — and load is a running
sum of demands, so it only ever grows. **The layer partition is always correct.** Hold on to that
fact; §5 is about a different pointer entirely.

### 2.4 Scoreboard on the normal instance

| solver | answer | correct? |
|---|---|---|
| Bellman (`PTVRP`) | 149 | ✓ |
| Vidal deque (`PTVRP_LINEAR`) | 182 | ✗ +22% |
| Layered deque (`PTVRP_LAYERED`) | 149 | ✓ |
| exhaustive enumeration | 149 | — |

---

## 3. Now an instance where revival happens

`Instances/Counterexamples/ce_rounding.gt` — only **3 vendors**, so you can check every number by
hand. Capacity is large (`Q = 1000`) and every demand is 1, so **`m = 1` everywhere**. The multiplier
is switched off completely. This is deliberate: it shows the bug has nothing to do with PT-VRP.

| vendor | 1 | 2 | 3 |
|---|---|---|---|
| demand | 1 | 1 | 1 |
| `dreturn` | 30 | 21 | 10 |
| `dnext` | 30 | 10 | — |

Time limit `T_H = 80`.

**The violation.** At vendor 2:

```
    d(v2, depot) = 21        d(v2, v3) + d(v3, depot) = 10 + 10 = 20
```

21 > 20. Going home directly from vendor 2 is **one unit longer** than going via vendor 3. That is
geometrically impossible, and it is exactly what independent integer rounding produces.

**What it does to the routes from start 0:**

| route | vendors | distance | fits `T_H = 80`? |
|---|---|---|---|
| `(0,1]` | 1 | 30 + 30 = **60** | ✓ |
| `(0,2]` | 1,2 | 30 + 30 + 21 = **81** | ✗ over by 1 |
| `(0,3]` | 1,2,3 | 30 + 30 + 10 + 10 = **80** | ✓ **back under** |

Adding vendor 3 made the route **shorter**, from 81 to 80. That is revival.

**The true optimum is 80** — one route covering all three vendors. Verified by enumerating all 4
partitions.

---

## 4. The same instance, four ways, step by step

### 4.1 Bellman, normal (stops at the first infeasible route) → **101, wrong**

```
  +-- p[1] : cheapest way to serve vendors 1..1
  |  start=0 : template(1..1)  d=60  q=1  m=1  cost=60*1=60   ->  p[0]=0 + 60 = 60   <- best so far
  +-- p[1] = 60  (predecessor 0)

  +-- p[2] : cheapest way to serve vendors 1..2
  |  start=1 : template(2..2)  d=42  q=1  m=1  cost=42*1=42   ->  p[1]=60 + 42 = 102   <- best so far
  +-- p[2] = 102  (predecessor 1)

  +-- p[3] : cheapest way to serve vendors 1..3
  |  start=1 : template(2..3)  d=41  q=2  m=1  cost=41*1=41   ->  p[1]=60 + 41 = 101   <- best so far
  |  start=2 : template(3..3)  d=20  q=1  m=1  cost=20*1=20   ->  p[2]=102 + 20 = 122
  +-- p[3] = 101  (predecessor 1)
```

**Look at `p[3]`. `start=0` is missing.** It should be the first line. It is absent because when the
scan from start 0 reached `(0,2]` at distance 81 it exceeded 80, the loop **broke**, and start 0 was
abandoned for good. The route `(0,3]` at distance 80 was never even computed.

Answer: **101**. Optimum: 80. Wrong by 26%.

### 4.2 Bellman, continuous (never stops early) → **80, correct**

The identical algorithm with one word changed: `continue` instead of `break`.

```
  +-- p[3] : cheapest way to serve vendors 1..3
  |  start=0 : template(1..3)  d=80  q=3  m=1  cost=80*1=80   ->  p[0]=0 + 80 = 80   [revived]   <- best so far
  |  start=1 : template(2..3)  d=41  q=2  m=1  cost=41*1=41   ->  p[1]=60 + 41 = 101
  |  start=2 : template(3..3)  d=20  q=1  m=1  cost=20*1=20   ->  p[2]=102 + 20 = 122
  +-- p[3] = 80  (predecessor 0)
REVIVED ARCS : 1
REVIVED IMPROVING : 1
```

There is `start=0`, tagged **`[revived]`** — meaning "this route is feasible even though an earlier,
shorter one was not." It gives 80 and wins immediately.

`REVIVED ARCS : 1` is the counter: one route existed that the normal scan never looked at.
`REVIVED IMPROVING : 1` says that route was better than anything else available.

Answer: **80**. Correct.

### 4.3 Layered, normal (prunes by a moving frontier) → **101, wrong**

```
  +-- p[1] : feasible starts begin at 0, layers 1..1
  |  layer k=1 : starts [0,1)  deque [ 0 ]    front=0   cand = p[0] + 1*d(0,1) = 60   <- best so far

  +-- p[2] : feasible starts begin at 1, layers 1..1
  |  layer k=1 : starts [1,2)  deque [ 1 ]    front=1   cand = p[1] + 1*d(1,2) = 102  <- best so far

  +-- p[3] : feasible starts begin at 1, layers 1..1
  |  layer k=1 : starts [1,3)  deque [ 1 2 ]  front=1   cand = p[1] + 1*d(1,3) = 101  <- best so far

WARNING : 1 positions break the monotonicity the sliding windows assume (rounded distances)
```

Follow the phrase **"feasible starts begin at"**. It reads 0, then **1**, then **1**.

At `p[2]`, start 0 was infeasible (81 > 80), so the frontier moved from 0 to 1. At `p[3]` start 0 is
feasible again — but the frontier is a **one-way pointer**. It never moves back. So `p[3]` scans
`[1,3)` and start 0 is outside the window.

Note the failure is *not* in the layering. There is only one layer here (`m = 1` throughout), and
the layer partition is load-based and always correct. The bug is in a **separate** pointer that
prunes by time.

Answer: **101**. Same wrong answer as Bellman, reached by a different route.

### 4.4 Layered, continuous (no time pruning) → **80, correct**

Keeps the layers, drops the frontier. Every start in the load window is scanned and its feasibility
tested one at a time.

```
  +-- p[1] : scanning starts [0,1), layers 1..1   (pruned solver would start at 0)
  |  layer k=1 : best start 0   cand = p[0] + 1*d(0,1) = 60   <- best so far
  +-- p[1] = 60  (predecessor 0, layer 1)

  +-- p[2] : scanning starts [0,2), layers 1..1   (pruned solver would start at 1)
  |  layer k=1 : best start 1   cand = p[1] + 1*d(1,2) = 102  <- best so far
  +-- p[2] = 102  (predecessor 1, layer 1)

  +-- p[3] : scanning starts [0,3), layers 1..1   (pruned solver would start at 1)
  |  layer k=1 : best start 0   cand = p[0] + 1*d(0,3) = 80   [behind the frontier]   <- best so far
  +-- p[3] = 80  (predecessor 0, layer 1)
REVIVED STARTS : 1
REVIVED IMPROVING : 1
```

`(pruned solver would start at 1)` shows where the frontier *would* have been. Start 0 is tagged
**`[behind the frontier]`** — recovered from exactly the region the pruning throws away.

Answer: **80**. Correct.

### 4.5 Scoreboard on the revival instance

| solver | stops early? | answer | correct? |
|---|---|---|---|
| Bellman normal | yes, on time | **101** | ✗ +26% |
| Bellman continuous | no | **80** | ✓ |
| Layered normal | yes, on time | **101** | ✗ +26% |
| Layered continuous | no | **80** | ✓ |
| exhaustive enumeration | — | **80** | — |

Both fast solvers are wrong. Both controls are right. **The multiplier is 1 throughout, so PT-VRP is
not involved** — this is a plain duration-constrained routing instance.

---

## 5. The three more counterexamples

All in `Instances/Counterexamples/`, all `n = 3`, all checked against exhaustive enumeration.

| instance | optimum | Bellman | Bellman cont | Layered | Layered cont | what it shows |
|---|---|---|---|---|---|---|
| `ce_rounding` | **80** | 101 | 80 | 101 | 80 | a **1-unit** violation is enough |
| `ce_blatant` | **70** | 120 | 70 | 120 | 70 | the error can reach **+71%** |
| `ce_multiplier` | **240** | 241 | 240 | 241 | 240 | survives with **`m = 2`** active |
| `ce_infeasible` | **70** | `NO SOLUTION` | 70 | 70 | 70 | can report **false infeasibility** |

Two remarks.

**`ce_infeasible` is the worst failure mode.** The instance is solvable at cost 70 and normal Bellman
reports that no solution exists. Inside a metaheuristic that feeds a penalty-based fitness,
misreporting the feasible region is worse than returning a suboptimal number.

**Layered gets `ce_infeasible` right while Bellman gets it wrong.** So `break` and the frontier
pointer are *related but not identical* defects: `break` abandons a start permanently, while the
layered window is a range that a later column can sometimes re-enter. They fail on overlapping but
different sets of instances.

---

## 6. How often does this happen in practice?

All 3,150 instances in `Instances/Instances 1,2,3` on the restored capacity ladder (16 capacities,
10 to 100,000; `sweep_PTVRP.csv`, `sweep_PTVRP_CONT.csv`, `sweep_PTVRP_LAYERED.csv`).

**All three agree on all 3,150.** 3,060 feasible with identical cost strings, 90 infeasible
unanimously, zero disagreements. The layered control `PTVRP_LAYERED_CONT` agrees on all 3,150 as
well (`revival_result.md`).

**But the early stop fires on 238 of them.** On those, a route the scan skipped would have improved a
label when the control evaluated it: 425 revived arcs in all, 408 of them improving. It never
changed a final answer — improving a label mid-scan is necessary but not sufficient, because a later
start often reaches the same point just as cheaply.

Where it fires (share of instances with at least one revived arc):

| instance size | rate | | capacity `Q` | rate |
|---|---|---|---|---|
| n < 500 | 0.1% | | 10 | 0.0% |
| 500 – 2,000 | 3.2% | | 20 | 0.5% |
| 2,000 – 10,000 | 16.3% | | 40 – 50 | 1.9 – 5.7% |
| 10,000 – 80,000 | **29.2%** | | 100 – 100,000 | 5.4 – 11.9% |

**Size.** Each start crosses the time limit exactly once, so an instance gets `n` chances. More
vendors, more chances. Per million crossings the rate is 4.0, 68.1, 44.1 and 24.4 across the four
size buckets, so it is not flat, but it is of one order of magnitude from 500 vendors up.

**Capacity.** Revival needs `m` to stay the *same* across the step — if the trip count increments,
the product `d·m` jumps up and swamps the one-unit wobble. At `Q = 10` the load crosses a capacity
boundary at nearly every vendor, so the window barely exists. At large `Q` it rarely does.

Measured, with `rho = mean demand / Q`:

| | instances | revived routes per million crossings |
|---|---|---|
| `rho < 1` (trip count does not step every vendor) | 2,835 | **35.9** |
| `rho >= 1` (it does) | 315 | **0.8** |

A 45× cliff exactly where the mechanism predicts. On the ladder `rho >= 1` occurs only at `Q = 10`
and `Q = 20`, and those 315 instances produced one revived arc between them.

**So the honest summary is two sentences, and both matter:**

> The early stop is **unsound** — demonstrably, from a violation the size of one rounding unit.
> It is also **harmless on every instance in this benchmark**.

The second sentence does not rescue the first. A single counterexample settles a correctness claim;
3,150 agreements do not.

---

## 7. Vidal's Split is **not** affected

This needs saying plainly, because the counterexamples above invite the opposite conclusion.

Every stopping and eviction condition in the original library is **load-based**:

```
Split_Bellman.cpp:36           for (... ; j <= nbNodes && load <= vehCapacity ; j++)
Split_Bellman_Bounded.cpp:45   for (... ; j <= nbNodes && load <= vehCapacity ; j++)
Split_Linear.cpp:56            while (sumLoad[i+1] - sumLoad[front] > vehCapacity + 0.0001)
Split_Linear_Bounded.cpp:75    while (sumLoad[i+1] - sumLoad[front] > vehCapacity + 0.0001)
```

Load is a running sum of non-negative demands. It increases **unconditionally** — no geometry, no
triangle inequality, nothing for rounding to disturb. Once `load > Q` it stays `> Q`. **Vidal's early
termination is correct by construction, and revival cannot occur in his setting.**

His own source makes the same point. In `Split_Bellman_Soft.cpp:37`, where capacity becomes a
*penalty* instead of a hard limit and the monotone stopping rule no longer applies, the guard is
commented out and the scan runs to the end:

```cpp
for (int j = i+1 ; j <= myData->nbNodes /* && load <= 4.0 * myData->vehCapacity */ ; j++)
```

Same reasoning as our control solvers, reached independently: when the quantity you would stop on
stops being monotone, stop stopping on it.

---

## 8. Who *is* affected

The claim is about the **constraint**, not the author:

> Any Split that stops scanning on accumulated **time** rather than accumulated **load** inherits
> this defect.

That covers the distance-constrained VRP, the VRPTW, and any duration-limited variant, whether or not
a multiplier is involved — `ce_rounding` has `m = 1` throughout. Load is monotone by construction;
duration is monotone only under the triangle inequality, which is a property of *distances*. Without
a control that performs no early stop there is nothing to observe, which may be why it appears to be
unreported.

**And the more realistic the duration model, the worse it gets.** Reading §1.1 as a ladder: rounding
puts a 1-unit violation into data that is otherwise metric, and is the version a purely academic
benchmark exhibits. Every step toward a duration a dispatcher would recognise — travel times that
depend on departure hour, waiting at the depot gate, a break charged against the drive home, a
one-way ring road — prices the leg home independently of the arcs that replace it and removes the
bound on the violation entirely. Measured on `structural_violation.gt`: **46% of positions violated,
median 156 units, maximum 482**, against TSPLIB's invariable 1.

The practical reading is that this is not a benchmark artefact to be tolerated because the margins
are small. The margins are small *because the benchmark is metric*. A duration-constrained Split
deployed against real travel times has no such margin, and §9's second half is what it needs.

**Not yet verified, and it decides whether this is a result or a footnote:** whether published
duration-constrained Split implementations actually contain the pattern. Everything above
demonstrates it in *our* code. Until someone opens two or three implementations from the literature
and checks, the honest claim is narrower than the one in the box.

---

## 9. The repair

Implemented and measured. **Stop on the route *without* the leg home.**

Recall §1: adding a stop lengthens the path out to the new vendor, and *replaces* the leg home. The
path out can only grow — you append an arc and remove nothing. The leg home is the only piece that
can shrink, and it is exactly where revival lives. So exclude it:

```
    path out (i,j) = depot -> v_{i+1} -> ... -> v_j          no leg home

    stop when   path_out(i,j) · m(i,j)  >  T_H
```

Sound with **no assumption about the instance at all** — not the triangle inequality, not rounding,
not symmetry. Three facts, each immediate:

1. `path_out` is non-decreasing in `j` (append an arc, remove nothing);
2. `m` is non-decreasing in `j` (demand accumulates);
3. the full route is `path_out + leg home >= path_out`.

So once `path_out · m` exceeds the horizon, every longer template from that start is infeasible,
permanently. **Revival cannot escape a bound that ignores the leg home.**

The test that decides whether an arc may be *used* is unchanged and exact: `tau(i,j)·m <= T_H`, on
the full route. An arc failing it is **skipped, not stopped on**. Pruning is conservative;
feasibility is exact.

Check it on `ce_rounding` from §3. From start 0 at vendor 2: the path out is `30 + 30 = 60`, and
`60 · 1 = 60 <= 80`, so the scan does **not** stop. It continues, reaches vendor 3, and finds the
route of length 80. Correct — and note it never needed to know that a shorter route was coming.

### In Bellman it is one line

`PTVRP_CONT_FIX` (`Program/Split_Bellman_PTVRP_cont_fix.{h,cpp}`). The loop's running `time` already
*is* the path out — the return leg is only ever added into `tau_ij` — so the change is:

```cpp
if (pathOut * m > myData->horizon + 1.e-9)
    break ;                        // sound: pathOut and m only grow, and tau >= pathOut
```

### In the layered decoder it is the same quantity, twice

`PTVRP_LAYERED_CONT_FIX` (`Program/Split_Layered_PTVRP_cont_fix.{h,cpp}`). The path out is also
separable — `path_out(i,j) = A[i] + sumDistance[j]` — and `sumDistance[j]` is unconditionally
non-decreasing where `Bt[j]` is not. So **both** one-way pointers become sound by swapping which
quantity they test:

| | `PTVRP_LAYERED` | `PTVRP_LAYERED_CONT` | `..._CONT_FIX` | `..._SAFE` |
|---|---|---|---|---|
| column frontier | `Bt` — unsound | removed | `Bout_t` — **sound** | `Bout_t` — **sound** |
| per-layer suffix `firstTimeLE[k]` | `Bt` — unsound | removed | `Bout_t` — **sound** | `Bout_t` — **sound** |
| feasibility at the deque front | pops on time | steps over, exact | steps over, exact | steps over, exact |
| layer bound | horizon | **none** (`K_full`) | horizon | horizon |
| deque back-pop | key only — assumed | key only — assumed | key only — assumed | **joint** — sound |

### What it costs

All 3,150 instances on the restored capacity ladder, one sweep per solver (`sweep_<SOLVER>.csv`,
whole-process wall clock from `batch_run.py`, one machine). Exactness is against `PTVRP_CONT`, which
scans every pair and assumes nothing: **3,060 identical, 90 both-infeasible, 0 disagreements**, for
every solver.

| solver | total | |
|---|---|---|
| `PTVRP_LAYERED` | 24.9 s | unsound frontier |
| `PTVRP_LAYERED_CONT_FIX` | **27.7 s** | **sound frontier**, assumed eviction |
| `PTVRP_LAYERED_SAFE` | **27.7 s** | **sound frontier and sound eviction** |
| `PTVRP` | 70.4 s | unsound early stop |
| `PTVRP_CONT_FIX` | **83.6 s** | **sound early stop** |
| `PTVRP_CONT` | 1,280.1 s | no stop — the oracle |

**Soundness costs 1.19× in Bellman and 1.11× in the layered decoder.** The Bellman fix stays 15×
faster than the oracle and the layered one 46×. The eviction guard on top is free — 27.7 s either
way, identical `eff_K` — because on metric-up-to-rounding data it never has to act (below). The
totals are start-up dominated at small `n` (2,879 of the 3,150 `PTVRP_LAYERED` runs finish under
20 ms), so the per-size ratios in `revival_result.md` §6.4 are the ones to quote.

In arcs rather than seconds, over the twelve largest instances per set — total arcs scanned over
total starts, against where the unsound stop would have ended:

| set | sound | unsound | overhead | no stop (`n/2`) |
|---|---|---|---|---|
| `Instances 1` | 518.2 | 463.4 | 1.12× | 35,504 |
| `Instances 2` | 240.7 | 205.5 | 1.17× | 35,504 |
| `Instances 3` | 167.6 | 138.6 | 1.21× | 35,504 |

Over the whole sweep the sound stop scans 5,174 M arcs against 4,797 M for the unsound one, 1.08×.
The ratio is worst at the bottom of the ladder — 1.40× at `Q = 10`, 1.32× at `Q = 20` — where the
absolute numbers are 13 and 20 arcs per start, and it is 1.00× from `Q = 10,000` up, where the
horizon rather than capacity ends every scan.

And for the layered decoder, the bound recovers most of what the *unsound* frontier gave —
median `eff_K`:

| set | no bound | **sound bound** | unsound |
|---|---|---|---|
| `Instances 1` | 4.0 | **2.0** | 1.9 |
| `Instances 2` | 17.8 | **4.9** | 4.5 |
| `Instances 3` | 35.1 | **7.1** | 5.9 |

### What this supersedes

An earlier draft of this section proposed a suffix sum of the instance's own triangle violations,
`D[j]`, and stopping on `(tau(i,j) − D[j])·m > T_H`. That is also sound, and a later refinement using
the suffix *minimum* of the route length is tighter still. Both are superseded: they need an O(n)
precomputed array that must be rebuilt whenever the tour order changes — which, inside HGS, is every
iteration. The path-out rule needs nothing precomputed and is already a quantity the loop carries.

### The second half of the fix: sound eviction

The path-out bound closes the *pruning*. It does not close the *deque*, and §1.1 says exactly when
that matters. `PTVRP_LAYERED_SAFE` (`Program/Split_Layered_PTVRP_safe.{h,cpp}`) closes it.

The deque discards an older start `b` when a newer start `i` has a key at least as small. In the
capacity-only problem that is enough. With a horizon, `i` must also **fit whenever `b` fits** — and
the key ranks on cost alone, so it does not check that. Under the triangle inequality it never needs
to, because `At` is then non-increasing and a later start always fits at least as easily. Off it,
`At` is not monotone and the deque can throw away the only start that fits:

```
ce_unsafe_pop_5v.gt    A = [79, 41, 66, -12, 50]      violated at i=2 and i=4

  add 2  ->  [2]                  key 458
  add 3  ->  450 <= 458, pop 2    A[3] = -12 < A[2] = 66   newcomer fits MORE easily -- safe
  add 4  ->  396 <= 450, pop 3    A[4] =  50 > A[3] = -12  newcomer fits LESS easily -- UNSAFE
```

Start 3 was feasible and optimal at 658. Start 4 is cheaper on paper and does not fit. Every layered
solver without the guard — including `PTVRP_LAYERED_CONT_FIX` — returns `NO SOLUTION` on an instance
that is feasible. Five vendors; checkable by hand (`revival_result.md` §8.1).

**The repair is one extra clause on the eviction, and it is exact rather than conservative:**

```
    evict b   when   key(i) <= key(b)   AND   At[i] <= At[b]
```

Feasibility at column `j` in layer `k` is `(At[start] + Bt[j])·k <= T_H`. Comparing two starts at the
**same** `j` cancels `Bt[j]` exactly as it cancels in the cost, so `At[i] <= At[b]` means "wherever
`b` fits, `i` fits", at every `j`, permanently. That is the one `j`-independent comparison that still
accounts for the leg home — which is why the fix belongs in the eviction and could never have gone
into the pruning, where the leg home is precisely what must be dropped.

The guard can leave a layer no longer sorted by key, so `sortedByKey[k]` tracks it per layer: while
the layer is still sorted the query is the original `O(1)` "first feasible from the front"; once a pop
has been blocked, that layer scans for the cheapest feasible entry instead. `BLOCKED POPS` and
`UNSORTED QUERIES` report both.

| | `PTVRP_LAYERED_CONT_FIX` | `PTVRP_LAYERED_SAFE` |
|---|---|---|
| `ce_unsafe_pop_5v` (optimum 658) | `NO SOLUTION` | **658** |
| 60 structurally non-metric instances | 59 / 60 | **60 / 60**, 5,205 pops blocked |
| 3,150 benchmark instances vs `PTVRP_CONT` | 3,060 identical, 90 both-infeasible, 0 differ | **identical** |
| `BLOCKED POPS` / `UNSORTED QUERIES` on the benchmark | — | **0 / 0** |
| total | 27.7 s, median `eff_K` 4.1 | 27.7 s, median `eff_K` 4.1 |

Read the last two rows together. On this benchmark the guard costs nothing **because it never fires**:
no layer ever leaves the original `O(1)` query. That is the stronger statement — not that the guard
rescued instances here, but that it confirms `PTVRP_LAYERED_CONT_FIX` was already right here, while
being the difference between right and wrong the moment the data stops being metric.

**So the layered decoder now carries no assumption about the instance at all**, and the qualifier on
its exactness is gone. `PTVRP_CONT_FIX` never carried one: it has no deque.

---

## 10. Reproducing everything above

```bash
cd Program && make

# §2 -- the normal 10-vendor instance, three solvers, step by step
./split ../Instances/ptvrp_demo_10v.gt -solver PTVRP         -trace 1    # 149, correct
./split ../Instances/ptvrp_demo_10v.gt -solver PTVRP_LINEAR  -trace 1    # 182, deque fails
./split ../Instances/ptvrp_demo_10v.gt -solver PTVRP_LAYERED -trace 1    # 149, correct

# §4 -- the revival instance, four ways
cd ..
for s in PTVRP PTVRP_CONT PTVRP_LAYERED PTVRP_LAYERED_CONT; do
  echo "=== $s ==="
  Program/split Instances/Counterexamples/ce_rounding.gt -solver $s -trace 1
done

# §5, §9 -- all four counterexamples, unsound and fixed solvers side by side
for f in Instances/Counterexamples/*.gt; do
  for s in PTVRP PTVRP_CONT PTVRP_CONT_FIX PTVRP_LAYERED PTVRP_LAYERED_CONT_FIX; do
    echo -n "$(basename $f) $s "
    Program/split "$f" -solver $s 2>&1 | grep -E "SOLUTION COST|no Split"
  done
done

# §6 -- the full sweep (about 40 minutes)
python3 batch_run.py --dir "Instances/Instances 1" --dir "Instances/Instances 2" \
    --dir "Instances/Instances 3" \
    --solver PTVRP PTVRP_CONT PTVRP_LAYERED --out full_sweep.csv
```

| solver name | what it is |
|---|---|
| `PTVRP` | Bellman DP, stops at the first time-infeasible route. `O(nB)`. **Unsound** |
| `PTVRP_CONT` | same DP, never stops early. `O(n²)`. **Control / oracle** |
| `PTVRP_CONT_FIX` | same DP, stops on the path out. `O(nB)`. **Sound unconditionally** (§9) |
| `PTVRP_LINEAR` | Vidal's deque applied to PT-VRP. Wrong for a different reason (§2.2) |
| `PTVRP_LAYERED` | one deque per trip count, plus time pruning. `O(nK)`. **Unsound frontier** |
| `PTVRP_LAYERED_CONT` | layers and deques kept, time pruning dropped. `O(n·K_full)`. **Control** (see `revival_result.md`) |
| `PTVRP_LAYERED_CONT_FIX` | layers plus a path-out frontier. `O(nK)`. Sound pruning, **assumed eviction** |
| `PTVRP_LAYERED_SAFE` | the above plus joint-dominance eviction. `O(nK)`. **Sound unconditionally** (§9) |

Use `PTVRP_CONT_FIX` or `PTVRP_LAYERED_SAFE`. The others are here to be measured against.

---

## 11. Where the detail lives

`NOTES.md` is the full research write-up: why Vidal's Property 2 fails under a multiplicative
objective, what layering restores and why, the closed form for the layered decoder's cost, the
crossover against Bellman, and the open questions. Revival is §5.8–§5.10 and open question 7.4 there.

Vidal, T. (2016). *Technical note: Split algorithm in O(n) for the capacitated vehicle routing
problem.* Computers & Operations Research 69, 40–47. arXiv:1508.02759.

---

## 12. Note: earlier figures (pre-ladder instance set)

The benchmark numbers in §6 and §9 were re-measured on 2026-09-17, after `rescale_instances.py`
restored Vidal's capacity ladder to `Instances 2` and `Instances 3` (scaled by 0.20 and 0.10).
Before that those two folders were flat at `Q = 20` and `Q = 10` for all ten suffixes. The
counterexamples, traces and the argument of §1–§5 and §7–§8 do not depend on the benchmark and are
unchanged. What the earlier text said, for the record:

- **§6.** 2,829 feasible and 321 infeasible; the early stop fired on 103 instances (177 revived
  arcs, 172 improving); rates 0.1 / 1.4 / 6.8 / 12.6% by size bucket and 0.2% at `Q = 10`, 0.8% at
  `Q = 20`, 5.7–11.4% from `Q = 100` up; 38.2 against 1.1 revived routes per million crossings
  either side of `rho = 1`, a 35× cliff.
- **§9.** Totals 23.7 / 28.0 / 28.0 / 34.0 / 38.4 / 781.8 s for `PTVRP_LAYERED`,
  `PTVRP_LAYERED_CONT_FIX`, `PTVRP_LAYERED_SAFE`, `PTVRP`, `PTVRP_CONT_FIX`, `PTVRP_CONT`, read as
  **1.13× in Bellman and 1.18× in the layered decoder**, both ~20× faster than the oracle. Arcs per
  start on the twelve largest instances: 11.5 / 6.6 (1.73×) on `Instances 2` and 6.5 / 3.8 (1.72×)
  on `Instances 3`. Median `eff_K` 499.7 / 24.2 / 18.5 on `Instances 2` and 998.9 / 30.2 / 29.2 on
  `Instances 3` for no bound / sound bound / unsound. Median `eff_K` 17.3 for the two sound layered
  solvers.

Two readings moved. The order of the two soundness costs is reversed on the ladder — Bellman now
pays more (1.19×) than the layered decoder (1.11×) — because the upper rungs lengthen Bellman's
scan while the layered decoder's layer count collapses toward 1 there. And the sound layered bound
sits 20% above the unsound frontier's `eff_K` on `Instances 3` rather than 3%, though in absolute
terms that is 7.1 against 5.9 layers per column rather than 30.2 against 29.2. The `Instances 1`
rows are byte-identical on both layouts and did not move.
