# Where revival actually happens in TSPLIB

The counterexamples in `Instances/Counterexamples/` are constructed. These are not — they are the
real arcs, from the real benchmark files, at the positions where the route length dips.

**Reading the tables.** A `.gt` file stores the giant tour as arcs, not as TSPLIB city IDs: for each
position `j` it records `d(vj, depot)` — the leg home — and `d(vj, vj+1)` — the step to the next
vendor. "Vendor `j`" below is the position in the giant tour, not the original city number. The
route ending at `j` is `depot -> v1 -> ... -> vj -> depot`, i.e. the running prefix plus the leg home.

**The condition.** The route gets shorter by extending it when

```
    d(vj, depot)   >   d(vj, vj+1) + d(vj+1, depot)
```

— going straight home costs more than stopping at the next vendor and going home from there. In real
geometry that is impossible. TSPLIB rounds every distance to a whole number independently, and the
rounding makes it true by one unit.

---

## 1. One step: 63 instances, one unit every time

First occurrence in each instance, `Instances 1`:

| instance | at vendor `j` | `d(vj,0)` | `d(vj,vj+1)` | `d(vj+1,0)` | sum | loss |
|---|---|---|---|---|---|---|
| `ar9152_01` | 249 | 6263 | 186 | 6076 | 6262 | **1** |
| `bier127_01` | 90 | 1038 | 259 | 778 | 1037 | **1** |
| `bm33708_01` | 667 | 4080 | 12 | 4067 | 4079 | **1** |
| `brd14051_01` | 9 | 2177 | 23 | 2153 | 2176 | **1** |
| `ca4663_01` | 354 | 15812 | 121 | 15690 | 15811 | **1** |
| `ch150_01` | 40 | 185 | 23 | 161 | 184 | **1** |
| `ch71009_01` | 26 | 14724 | 23 | 14700 | 14723 | **1** |
| `d1291_01` | 137 | 737 | 25 | 711 | 736 | **1** |
| `d15112_01` | 271 | 8607 | 141 | 8465 | 8606 | **1** |
| `d1655_01` | 519 | 1045 | 25 | 1019 | 1044 | **1** |
| `d18512_01` | 34 | 2626 | 47 | 2578 | 2625 | **1** |
| `d2103_01` | 1343 | 1359 | 25 | 1333 | 1358 | **1** |

Always exactly 1 — the rounding unit. Never more, at a single step.

---

## 2. Two steps: `ar9152_01`, vendor 2102

The next stop is *equal*, not shorter. Anything that stops the moment the route stops shrinking
walks past this.

```
  vendor j   d(vj,depot)   d(vj,vj+1)   route len   vs 2102
      2102          3682            0      157698        +0    <- scan is here
      2103          3682          167      157698        +0    equal
      2104          3514            0      157697        -1    SHORTER : revival window
      2105          3514           94      157697        -1    SHORTER
```

The losing arc:

```
  at vendor 2103:  d(v2103,depot) = 3682  >  167 + 3514 = 3681       violation 1
```

---

## 3. Five steps: `d1655_01`, vendor 1256

The clearest picture of the mechanism, and the case that kills any fixed-depth lookahead. The route
goes **up** first, then drops below where it started, five stops later.

```
  vendor j   d(vj,depot)   d(vj,vj+1)   route len   vs 1256
      1256           222           45       46950        +0    <- scan is here
      1257           178           25       46951        +1    LONGER
      1258           152           25       46950        +0    equal
      1259           127           25       46950        +0    equal
      1260           102           25       46950        +0    equal
      1261            76           25       46949        -1    SHORTER : revival window
      1262            51           25       46949        -1    SHORTER
```

Two separate losing arcs contribute:

```
  at vendor 1257:  d(v1257,depot) = 178  >  25 + 152 = 177           violation 1
  at vendor 1260:  d(v1260,depot) = 102  >  25 + 76  = 101           violation 1
```

**Why this stretch is vulnerable.** Look at the leg-home column: 222, 178, 152, 127, 102, 76, 51 —
falling by about 25 each step, which is exactly `d(vj, vj+1)`. The tour is walking in a straight line
back toward the depot. Every vendor is *on the way home*, so stopping at it is almost free, and the
route length is nearly flat for six consecutive stops. When the independent rounding shaves one unit
off a leg home, the route dips below where it was.

Revival needs a stretch like this: the tour heading at the depot, several near-collinear vendors, and
the route length flat enough that one rounding unit decides the sign.

---

## 4. Why the fix does not care about any of this

The bound in `revival.md` §9 stops on the route **without** the leg home:

```
    path_out(i,j) = depot -> v_{i+1} -> ... -> v_j
```

Every number that moves in the tables above is a leg home. The path-out column would read
monotonically upward in all three cases — it only ever gains `d(vj, vj+1)` and never gives anything
back. That is why the rule needs no lookahead, no suffix array, and no assumption about the geometry:
it prunes on the one quantity revival cannot touch.

---

## 5. Reproducing

```bash
python3 - <<'EOF'
def load(p):
    t=open(p).read().split(); i=0
    while t[i]!="DIMENSION": i+=1
    n=int(t[i+2] if t[i+1]==":" else t[i+1])
    while t[i]!="GIANT_TOUR_SECTION": i+=1
    i+=1
    dret=[0.0]*(n+2); dnx=[0.0]*(n+2)
    for k in range(1,n+1):
        i+=2; dret[k]=float(t[i]); i+=1
        if k<n: dnx[k]=float(t[i]); i+=1
    R=[0.0]*(n+2); s=0.0
    for j in range(1,n+1):
        if j>1: s+=dnx[j-1]
        R[j]=s+dret[j]
    return n,dret,dnx,R
n,dret,dnx,R = load("Instances/Instances 1/d1655_01.gt")
for j in range(1256,1263):
    print(j, dret[j], dnx[j], R[j], R[j]-R[1256])
EOF
```
