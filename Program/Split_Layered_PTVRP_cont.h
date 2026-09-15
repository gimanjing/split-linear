#ifndef SPLIT_LAYERED_PTVRP_CONT_H
#define SPLIT_LAYERED_PTVRP_CONT_H

#include "Split.h"

// Layered-K Split for the PT-VRP WITHOUT the horizon pruning : O(n^2 K).
//
// Split_Layered_PTVRP keeps two one-way pointers that assume tau(i,j)*m is monotone :
//   firstFeasible    (Split_Layered_PTVRP.cpp:83)  -- the column's feasibility frontier
//   firstTimeLE[k]   (Split_Layered_PTVRP.cpp:123) -- the per-layer feasible suffix
// Both only ever move right, so a start excluded once is excluded for good. That is exact only under
// the triangle inequality on travel times, which rounded instances do not satisfy.
//
// This solver keeps the LAYER STRUCTURE, which is sound -- inside layer k the multiplier is constant,
// so k*d(i,j) = k*A[i] + k*B[j] is separable and the layer partition by trips() is load-based and
// monotone unconditionally -- and drops only the horizon pruning : within a layer every start is
// scanned and its feasibility tested individually. It is therefore the layered counterpart of
// Split_Bellman_PTVRP_cont, and the reference for whether the pruning ever changes an answer.
//
// The cost is O(n^2 K) : this is a control, not a contender.
class Split_Layered_PTVRP_cont: public Split
{

private:

	vector < double > potential ;
	vector < int > pred ;
	vector < int > predK ;

	vector < double > sumDistance ;
	vector < double > sumLoad ;

	// Separable decomposition : d(i,j) = A[i] + B[j], tau(i,j) = At[i] + Bt[j]
	vector < double > A, B, At, Bt ;

	// Starts that are horizon-feasible at j although they sit behind where the monotone frontier of
	// Split_Layered_PTVRP would already have advanced : exactly the starts the pruning never sees.
	long revivedStarts ;
	long revivedImproving ;

	inline int trips(int i, int j)
	{
		int m = (int) ceil((sumLoad[j] - sumLoad[i]) / myData->vehCapacity - 1.e-9) ;
		return (m < 1) ? 1 : m ;
	}

public:

	Split_Layered_PTVRP_cont(Pb_Data * myData) : Split(myData), revivedStarts(0), revivedImproving(0) {}

	int solve();
};

#endif
