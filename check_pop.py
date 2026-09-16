#!/usr/bin/env python3
"""Recompute A, B, key, feasibility and the exhaustive optimum for the two pop counterexamples."""
import math, sys

def parse(path):
    tok = open(path).read().split()
    hdr, i = {}, 0
    while tok[i] != "GIANT_TOUR_SECTION":
        if tok[i] in ("DIMENSION","CAPACITY","MAX_ROUTE","HORIZON","SPEED"):
            j = i+1
            if tok[j] == ":": j += 1
            hdr[tok[i]] = float(tok[j]); i = j+1; continue
        i += 1
    i += 1
    n = int(hdr["DIMENSION"]); dem=[0.0]*(n+1); dret=[0.0]*(n+1); dnext=[0.0]*(n+1)
    for v in range(1, n+1):
        i += 1                       # index
        dem[v]  = float(tok[i]); i += 1
        dret[v] = float(tok[i]); i += 1
        if v < n: dnext[v] = float(tok[i]); i += 1
    return n, hdr["CAPACITY"], hdr["MAX_ROUTE"], dem, dret, dnext

def tables(path):
    n, Q, TH, dem, dret, dnext = parse(path)
    sD=[0.0]*(n+1); sL=[0.0]*(n+1)
    for v in range(1,n+1):
        sL[v]=sL[v-1]+dem[v]; sD[v]=sD[v-1]+dnext[v-1]
    A=[dret[i+1]-sD[i+1] for i in range(n)]
    B=[0.0]+[sD[j]+dret[j] for j in range(1,n+1)]
    def m(i,j): return max(1, math.ceil((sL[j]-sL[i])/Q - 1e-9))
    return n,Q,TH,sD,sL,A,B,m

def exhaustive(path):
    n,Q,TH,sD,sL,A,B,m = tables(path)
    INF=float('inf'); p=[INF]*(n+1); p[0]=0.0; pr=[-1]*(n+1)
    for j in range(1,n+1):
        for i in range(j):
            if p[i]==INF: continue
            d=A[i]+B[j]; k=m(i,j)
            if d*k > TH + 1e-9: continue
            if p[i]+d*k < p[j]: p[j]=p[i]+d*k; pr[j]=i
    return p[n], p, pr, tables(path)

for path in sys.argv[1:]:
    opt,p,pr,(n,Q,TH,sD,sL,A,B,m) = exhaustive(path)
    print(f"=== {path}   n={n} Q={Q:g} T_H={TH:g} ===")
    print("  A =", [f"{a:g}" for a in A])
    print("  B =", [f"{b:g}" for b in B[1:]])
    print("  A monotone non-increasing ?",
          all(A[i+1] <= A[i] + 1e-9 for i in range(n-1)),
          "  violations at i =", [i for i in range(n-1) if A[i+1] > A[i]+1e-9])
    print("  triangle d(0,v_{i+2}) <= d(0,v_{i+1}) + d(v_{i+1},v_{i+2}) fails at i =",
          [i for i in range(n-1) if A[i+1] > A[i]+1e-9])
    print("  p  =", [("inf" if x==float('inf') else f"{x:g}") for x in p])
    print(f"  exhaustive optimum = {opt:g}")
    # decode
    cur=n; segs=[]
    while cur>0:
        i=pr[cur]; segs.append((i+1,cur,A[i]+B[cur],m(i,cur))); cur=i
    for a,b,d,k in reversed(segs):
        print(f"    vendors [{a}..{b}]  d={d:g}  m={k}  tau*m={d*k:g}  cost={d*k:g}")
    print()
