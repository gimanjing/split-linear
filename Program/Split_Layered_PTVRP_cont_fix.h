#ifndef SPLIT_LAYERED_PTVRP_CONT_FIX_H
#define SPLIT_LAYERED_PTVRP_CONT_FIX_H

#include "Split.h"
#include <deque>

// Layered-K Split for the PT-VRP with a SOUND horizon bound : O(n K) with K bounded by the horizon.
//
// Split_Layered_PTVRP prunes with tau(i,j) = At[i] + Bt[j], the full route time including the leg home.
// Bt[j] is not monotone on rounded instances -- the leg home can shrink by more than the detour costs --
// so its two one-way pointers can walk past a start that later fits again (revival.md).
//
// Split_Layered_PTVRP_cont answered that by deleting both pointers. Exact, but with no bound left on the
// layer count : every column visits every layer up to ceil(load(0,j)/Q), so the loop is O(n*K_full_tour),
// and on the low-capacity sets that is worse than brute-force Bellman.
//
// This solver keeps a bound and keeps exactness, by pruning on the PATH OUT instead of the full route :
//
//     pathout(i,j)  =  depot -> v_{i+1} -> ... -> v_j        the route WITHOUT the leg home
//                   =  A[i] + sumDistance[j]                 separable, same as d(i,j)
//
// Two facts make it a valid stopping rule with no assumption about the instance at all :
//   1. sumDistance[j] only grows -- extending a template appends an arc and removes nothing, because the
//      leg home is exactly what has been excluded. So pathout(i,j) is non-decreasing in j unconditionally.
//   2. d(i,j) = pathout(i,j) + dreturn[j] >= pathout(i,j), and trips(i,j) only grows.
// Hence  pathout(i,j)*trips(i,j) > T_H  proves every longer template from i is infeasible, for good.
// Revival cannot escape it : revival lives entirely in the leg home, which this bound ignores.
//
// So both pointers come back, testing Bout_t[j] (path out) rather than Bt[j] (full route) :
//   firstFeasible  -- column frontier, bounds the layer count at trips(firstFeasible, j) ;
//   firstTimeLE[k] -- per-layer suffix, At[i] > T_H/k - Bout_t[j] means i is dead in layer k and above.
// Both are one-way and both are sound, because Bout_t is monotone where Bt is not.
//
// The query is unchanged from Split_Layered_PTVRP_cont : walk the deque from the front and take the first
// start that fits the REAL horizon, tested with the true tau = At[i] + Bt[j]. A start that is temporarily
// over the horizon is stepped over, never popped, so it is still there when it revives.
//
// Pruning is therefore conservative and the feasibility test is exact. UNSAFE POPS still counts the one
// remaining assumption, the key-only back-pop, exactly as in Split_Layered_PTVRP_cont.
class Split_Layered_PTVRP_cont_fix: public Split
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

	// Starts the PATH-OUT frontier kept alive that the full-route frontier of Split_Layered_PTVRP would
	// already have dropped : the ones that can still revive.
	long revivedStarts ;
	long revivedImproving ;

	int monotonicityViolations ;

	Split_Layered_PTVRP_cont_fix(Pb_Data * myData) : Split(myData),
		layersFullTour(0), layersAllocated(0), maxLayerUsed(0), maxLayerOnPath(0), layerIterations(0),
		infeasibleSkips(0), unsafePops(0), revivedStarts(0), revivedImproving(0),
		monotonicityViolations(0) {}

	int solve();
};

#endif
