# Revival

**What it is:** a route that does not fit the time limit, followed by a *longer* route that does.

**Why it matters:** every Split implementation that stops scanning at the first time-infeasible route
assumes this cannot happen. On rounded instances it can, and the answer comes out wrong.

**Who is affected:** not Vidal's Split (§7). Any Split that stops on accumulated *time* rather than
accumulated *load* (§8).

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

**But TSPLIB rounds every distance to a whole number, independently.** That rounding can make
`d(v_j, depot)` up to one unit bigger than the other two added together. Then adding a vendor makes
the route *shorter*. If it was over the limit by less than that, it comes back under — and the scan
already walked away.

That is revival.

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

All 3,150 instances in `Instances/Instances 1,2,3`, three solvers each.

**All three agree on all 3,150.** 2,829 feasible with identical cost strings, 321 infeasible
unanimously, zero disagreements. The layered control `PTVRP_LAYERED_CONT` agreed too on the
480 instances with `n <= 600`, where its first `O(n²K)` version was tolerable; the deque-based
rebuild covers all 3,150 (`revival_result.md`).

**But the early stop fires on 103 of them.** On those, a route the scan skipped would have improved a
label when the control evaluated it. It never changed a final answer — improving a label mid-scan is
necessary but not sufficient, because a later start often reaches the same point just as cheaply.

Where it fires:

| instance size | rate | | capacity `Q` | rate |
|---|---|---|---|---|
| n < 500 | 0.1% | | 10 | 0.2% |
| 500 – 2,000 | 1.4% | | 20 | 0.8% |
| 2,000 – 10,000 | 6.8% | | 100 – 100,000 | 5.7 – 11.4% |
| 10,000 – 80,000 | **12.6%** | | | |

**Size.** Each start crosses the time limit exactly once, so an instance gets `n` chances. More
vendors, more chances. The per-crossing rate is roughly constant.

**Capacity.** Revival needs `m` to stay the *same* across the step — if the trip count increments,
the product `d·m` jumps up and swamps the one-unit wobble. At `Q = 10` the load crosses a capacity
boundary at nearly every vendor, so the window barely exists. At large `Q` it rarely does.

Measured, with `rho = mean demand / Q`:

| | revived routes per million crossings |
|---|---|
| `rho < 1` (trip count does not step every vendor) | **38.2** |
| `rho >= 1` (it does) | **1.1** |

A 35× cliff exactly where the mechanism predicts.

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
duration is monotone only under the triangle inequality, and rounded benchmark instances do not
satisfy it. Without a control that performs no early stop there is nothing to observe, which may be
why it appears to be unreported.

**Not yet verified, and it decides whether this is a result or a footnote:** whether published
duration-constrained Split implementations actually contain the pattern. Everything above
demonstrates it in *our* code. Until someone opens two or three implementations from the literature
and checks, the honest claim is narrower than the one in the box.

---

## 9. A possible repair

Not yet proved, and not yet implemented. Recorded because it looks cheap.

Measured across 1,200 benchmark instances:

| | |
|---|---|
| largest triangle violation `delta` | **exactly 1.00** |
| mean inter-vendor distance `c` | 39.6 |
| `delta / c` | 0.020 median, 0.158 worst |

One step forward adds about 40. One violation takes back at most 1. The wobble is two orders of
magnitude below the drift, which suggests an exact stop is nearly free.

Precompute a suffix sum of the instance's own violations, once, in `O(n)`:

```
    D[j] = sum over j' >= j of  max(0, dreturn[j'] - dnext[j'] - dreturn[j'+1])
```

Then stop on `(tau(i,j) - D[j]) · m > T_H` instead of `tau(i,j) · m > T_H`. Since the trip count only
grows, no later route can fit once that fires — so the stop becomes **exact**. On an instance
satisfying the triangle inequality `D` is identically zero and the behaviour is unchanged.

Sanity check on `ce_rounding`: one violation of 1 at position 2, so `D[2] = 1`. At `p[2]` from start
0, `tau = 81` and `(81 − 1)·1 = 80 <= 80`, so the scan does **not** stop, continues, and finds the
route of length 80. Correct.

**Two caveats.** The multiplier interaction needs a careful re-derivation before anyone relies on
this. And the layered decoder prunes on `At[i]` rather than a forward-accumulating `tau`, so the same
construction has to be redone there — it is not obviously the same argument.

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

# §5 -- all four counterexamples
for f in Instances/Counterexamples/*.gt; do
  for s in PTVRP PTVRP_CONT PTVRP_LAYERED PTVRP_LAYERED_CONT; do
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
| `PTVRP` | Bellman DP, stops at the first time-infeasible route. `O(nB)` |
| `PTVRP_CONT` | same DP, never stops early. `O(n²)`. **Control** |
| `PTVRP_LINEAR` | Vidal's deque applied to PT-VRP. Wrong for a different reason (§2.2) |
| `PTVRP_LAYERED` | one deque per trip count, plus time pruning. `O(nK)` |
| `PTVRP_LAYERED_CONT` | layers and deques kept, time pruning dropped. `O(n·K_full)`. **Control** (see `revival_result.md`) |

---

## 11. Where the detail lives

`NOTES.md` is the full research write-up: why Vidal's Property 2 fails under a multiplicative
objective, what layering restores and why, the closed form for the layered decoder's cost, the
crossover against Bellman, and the open questions. Revival is §5.8–§5.10 and open question 7.4 there.

Vidal, T. (2016). *Technical note: Split algorithm in O(n) for the capacitated vehicle routing
problem.* Computers & Operations Research 69, 40–47. arXiv:1508.02759.
