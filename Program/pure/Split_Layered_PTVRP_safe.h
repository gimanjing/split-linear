#ifndef SPLIT_LAYERED_PTVRP_SAFE_H
#define SPLIT_LAYERED_PTVRP_SAFE_H

#include "Split.h"
#include <deque>

// Layered-K Split for the PT-VRP, sound bound AND sound eviction : the last assumption removed.
//
// Split_Layered_PTVRP_cont_fix plus a sound back-pop. Feasibility at column j in layer k is
// At[i] + Bt[j] <= T_H/k, and comparing two starts at the same j cancels Bt[j], so
// At[new] <= At[old] means "wherever old fits, new fits" at every j. Eviction therefore needs both :
//
//     evict old   when   key(new) <= key(old)   AND   At[new] <= At[old]
//
// Where the guard blocks a pop the deque is no longer sorted by key, so that layer's query switches from
// "first feasible from the front" to "cheapest feasible anywhere in the deque". sortedByKey[k] tracks
// which regime each layer is in.
// Recording-free copy of Program/Split_Layered_PTVRP_safe : see there for the full argument.
class Split_Layered_PTVRP_safe: public Split
{

private:

	vector < double > potential ;
	vector < int > pred ;

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

	Split_Layered_PTVRP_safe(Pb_Data * myData) : Split(myData) {}

	int solve();
};

#endif
