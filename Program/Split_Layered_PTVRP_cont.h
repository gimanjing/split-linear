#ifndef SPLIT_LAYERED_PTVRP_CONT_H
#define SPLIT_LAYERED_PTVRP_CONT_H

#include "Split.h"
#include <deque>

// Layered-K Split for the PT-VRP WITHOUT the horizon pruning, on the same deques : O(n K) layer visits.
//
// Split_Layered_PTVRP keeps two one-way pointers that assume tau(i,j)*m is monotone :
//   firstFeasible    (Split_Layered_PTVRP.cpp:83)  -- the column's feasibility frontier
//   firstTimeLE[k]   (Split_Layered_PTVRP.cpp:123) -- the per-layer feasible suffix
// Both only ever move right, so a start excluded once is excluded for good. On rounded instances a start
// can fall out of the horizon and come back in (revival.md), and by then the pointers have walked past it.
//
// This solver is Split_Layered_PTVRP with both pointers removed and nothing else changed :
//   - the layer windows [firstLoadLE[k], firstLoadLE[k-1]) are kept -- load-based, always exact ;
//   - each layer keeps Vidal's deque ranked by key = p[i] + k*A[i], and evicts from the front on load only ;
//   - the query walks the deque from the front and takes the first start that fits the horizon. Keys
//     increase front to back, so that start is the best feasible one in the deque. An infeasible start is
//     stepped over, never popped, so it is still there if it revives in a later column.
// With no frontier there is no horizon bound on the layer count either : every layer up to
// ceil(load(0,j)/Q) is visited in every column, so the layer loop is O(n * K_full_tour).
//
// One assumption is left, and it is counted rather than hidden. The back-pop discards an older start when
// a newer one has a key at least as small. That is safe for feasibility only if the newer start fits the
// horizon whenever the older one does, i.e. At[new] <= At[old] -- the same triangle inequality. UNSAFE
// POPS counts the pops where it does not hold. Whether any of them changes an answer is measured against
// PTVRP_CONT, which scans every pair and assumes nothing.
class Split_Layered_PTVRP_cont: public Split
{

private:

	vector < double > potential ;
	vector < int > pred ;

	// layer (trip count) of the arc that last improved potential[j]
	vector < int > predK ;

	vector < double > sumDistance ;
	vector < double > sumLoad ;

	// Separable decomposition : d(i,j) = A[i] + B[j], tau(i,j) = At[i] + Bt[j]
	vector < double > A, B, At, Bt ;

	inline int trips(int i, int j)
	{
		int m = (int) ceil((sumLoad[j] - sumLoad[i]) / myData->vehCapacity - 1.e-9) ;
		return (m < 1) ? 1 : m ;
	}

	// key ranking predecessors inside layer k; constant in j, which is what makes the deque valid here
	inline double key(int i, int k)
	{
		return potential[i] + (double) k * A[i] ;
	}

public:

	// Trips the whole giant tour would need, ceil(total demand / Q) : also the deepest layer allocated
	int layersFullTour ;

	// Largest layer that ever produced an accepted improvement anywhere in the DP
	int maxLayerUsed ;

	// Largest layer appearing on the final path (equals the largest m(sigma) in the solution)
	int maxLayerOnPath ;

	// Total (column, layer) pairs the loop executed
	long layerIterations ;

	// Deque entries stepped over because they did not fit the horizon at that (column, layer) : the price
	// of never popping on time
	long infeasibleSkips ;

	// Back-pops where the evicted start fit the horizon more easily than the newcomer (At[old] < At[new])
	long unsafePops ;

	// Layer winners sitting behind where Split_Layered_PTVRP's pointers would stand : a start the pruned
	// solver could not have picked. IMPROVING : the winner also beat the label at that moment.
	long revivedStarts ;
	long revivedImproving ;

	// Count of positions where At[] / Bt[] break the monotonicity the pruned solver assumes
	int monotonicityViolations ;

	Split_Layered_PTVRP_cont(Pb_Data * myData) : Split(myData),
		layersFullTour(0), maxLayerUsed(0), maxLayerOnPath(0), layerIterations(0), infeasibleSkips(0),
		unsafePops(0), revivedStarts(0), revivedImproving(0), monotonicityViolations(0) {}

	int solve();
};

#endif
