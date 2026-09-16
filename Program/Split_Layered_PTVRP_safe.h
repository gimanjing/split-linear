#ifndef SPLIT_LAYERED_PTVRP_SAFE_H
#define SPLIT_LAYERED_PTVRP_SAFE_H

#include "Split.h"
#include <deque>

// Layered-K Split for the PT-VRP, sound bound AND sound eviction : the last assumption removed.
//
// Split_Layered_PTVRP_cont_fix fixed the horizon PRUNING (path-out bound) but kept Vidal's eviction
// rule unchanged : an older start is discarded when a newer one has a key at least as small. That is
// a comparison on COST only. With a horizon the cheapest start need not be a usable one, so the
// eviction can throw away the only candidate that fits (Instances/Counterexamples/ce_unsafe_pop_5v.gt).
//
// The missing ingredient is that feasibility also has a j-independent ranking. Feasibility at column j
// in layer k is  At[i] + Bt[j] <= T_H/k , and comparing two starts at the SAME j cancels Bt[j] exactly
// as it cancels in the cost. So
//
//     At[new] <= At[old]   <=>   wherever old fits, new fits -- at every j, for ever
//
// Eviction therefore needs both columns:
//
//     evict old   when   key(new) <= key(old)   AND   At[new] <= At[old]
//
// Under the triangle inequality At is non-increasing, so the second clause is automatic, the guard
// never blocks, and the deque behaves exactly as before -- no cost on metric instances. Where it does
// block, the deque is no longer sorted by key, so the query switches from "first feasible from the
// front" to "cheapest feasible anywhere in the deque". sortedByKey[k] tracks which regime each layer
// is in, so the fast path is kept wherever the guard has never fired.
//
// BLOCKED POPS counts the evictions the guard prevented : the ones Split_Layered_PTVRP_cont_fix would
// have made and reported as UNSAFE POPS.
class Split_Layered_PTVRP_safe: public Split
{

private:

	vector < double > potential ;
	vector < int > pred ;
	vector < int > predK ;

	vector < double > sumDistance ;
	vector < double > sumLoad ;

	// d(i,j) = A[i] + B[j], tau(i,j) = At[i] + Bt[j], pathout_time(i,j) = At[i] + Bout_t[j]
	vector < double > A, B, At, Bt, Bout_t ;

	inline int trips(int i, int j)
	{
		int m = (int) ceil((sumLoad[j] - sumLoad[i]) / myData->vehCapacity - 1.e-9) ;
		return (m < 1) ? 1 : m ;
	}

	inline double key(int i, int k)
	{
		return potential[i] + (double) k * A[i] ;
	}

public:

	int layersFullTour ;
	int layersAllocated ;
	int maxLayerUsed ;
	int maxLayerOnPath ;
	long layerIterations ;
	long infeasibleSkips ;
	long unsafePops ;

	// Evictions the joint-dominance guard prevented (what cont_fix would have counted as unsafe)
	long blockedPops ;

	// Columns where a layer had to fall back to scanning the whole deque
	long unsortedQueries ;

	// Starts the PATH-OUT frontier kept alive that the full-route frontier of Split_Layered_PTVRP would
	// already have dropped : the ones that can still revive.
	long revivedStarts ;
	long revivedImproving ;

	int monotonicityViolations ;

	Split_Layered_PTVRP_safe(Pb_Data * myData) : Split(myData),
		layersFullTour(0), layersAllocated(0), maxLayerUsed(0), maxLayerOnPath(0), layerIterations(0),
		infeasibleSkips(0), unsafePops(0), blockedPops(0), unsortedQueries(0), revivedStarts(0), revivedImproving(0),
		monotonicityViolations(0) {}

	int solve();
};

#endif
