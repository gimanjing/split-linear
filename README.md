# split-linear
Split Linearity Preserving for Multiplicative Function

Vidal's Split library, extended with the Periodic-Template VRP (PT-VRP), where a template is
executed `m(sigma) = ceil(q(sigma)/Q)` times over a horizon, so its cost is `d(sigma)*m(sigma)` and
its duration `tau(sigma)*m(sigma)` must fit `T_H`.

## Running it

In a Codespace everything is built on create. Otherwise:

```bash
cd Program && make
```

Then:

```bash
./split <instance> -solver <SOLVER> [-trace 1]
```

| solver | what it is |
|---|---|
| `BELLMAN`, `LINEAR`, and the `_SOFT` / `_BOUNDED` variants | Vidal's original CVRP solvers, unchanged |
| `PTVRP` | exact PT-VRP DP, `O(nB)`. The reference answer |
| `PTVRP_LINEAR` | Vidal's deque applied to PT-VRP anyway. **Deliberately wrong**, kept to measure the error |
| `PTVRP_LAYERED` | one deque per trip count, `O(nK)`. Exact |

## Learning the three PT-VRP algorithms

`-trace 1` prints the DP as it fills in. Use the 5-vendor instance — it is the worked example whose
numbers are small enough to check by hand.

```bash
./split ../Instances/ptvrp_demo_5v.gt -solver PTVRP         -trace 1
./split ../Instances/ptvrp_demo_5v.gt -solver PTVRP_LINEAR  -trace 1
./split ../Instances/ptvrp_demo_5v.gt -solver PTVRP_LAYERED -trace 1
```

Read them in that order.

**1. `PTVRP` — the exact DP.** Every candidate start is evaluated for every endpoint. For each `p[j]`
you see each template, its distance `d`, its load `q`, its trip count `m`, and the resulting
`d*m`. Nothing clever happens; it is the definition, computed. The count at the bottom is the `n*B`
work. Answer: 80.

**2. `PTVRP_LINEAR` — why the shortcut fails.** The deque keeps predecessors ranked by fixed cost and
commits to its front. Watch the front stay at 0 the whole way down while the trace reports what an
exhaustive scan would have found: off by 8, then 22, then 46. The ranking was established without
the multiplier, so it is a ranking of the wrong quantity. Answer: 148, which is 85% worse.

**3. `PTVRP_LAYERED` — the repair.** One deque per trip count. Look at `p[5]`: the layers take
starts `[4,5)`, `[2,4)`, `[1,2)`, `[0,1)` — disjoint windows that tile `[0,5)`. Each start appears
in exactly one layer, which is why the cost is `O(nK)` and not `O(n^2K)`. Inside a layer `m` is a
constant, so `k*d(i,j) = k*A[i] + k*B[j]` separates and the deque's ranking is valid again.
Answer: 80, matching the exact DP.

The one-line summary: the multiplier destroys the property the deque needs; conditioning on the
multiplier restores it exactly, because `m` is integer-valued and therefore supplies a finite,
natural partition of the predecessors.

## Reading the layer diagnostics

`PTVRP_LAYERED` reports several `K` values, which are not the same thing:

```
K FULL TOUR          : 1947   ceil(total demand / Q) -- the naive ceiling
LAYERS ALLOCATED (K) : 135    deepest layer reached in any column
MAX LAYER USED       : 15     deepest layer that improved a label
MAX LAYER ON PATH    : 15     largest m(sigma) in the answer
LAYER ITERATIONS     : 81445  effective K = 104.15 per column  <- the O(n*K) work
```

`LAYER ITERATIONS / n` is the quantity that belongs in a complexity claim; it is the analogue of
Vidal's `B`. The two move in opposite directions as capacity changes, because a template can spend
the horizon on length or on trips but not both, and their product is bounded by `T_H` over the mean
per-vendor travel time.

## Instances

- `Instances/*.gt` — the original Vidal files.
- `Instances/Instances 1/` — 105 instances at 10 capacities (100 to 100000), suffix `_01.._10`.
- `Instances/Instances 2/`, `Instances 3/` — the same instances at `Q=20` and `Q=10`. Many vendors
  there have `q_i > Q`, so classic Split cannot solve them at all while PT-VRP can.
- `Instances/ptvrp_demo_5v.gt` — the 5-vendor worked example used above.

Service time is fixed at `PTVRP_SERVICE_TIME` in `Program/Pb_Data.h`; the instance files carry none.
