#ifndef SPLIT_LAYERED_PTVRP_CONT_H
#define SPLIT_LAYERED_PTVRP_CONT_H

#include "Split.h"
#include <deque>

// Layered-K Split for the PT-VRP WITHOUT the horizon pruning, on the same deques : O(n K_full_tour).
//
// Split_Layered_PTVRP with both one-way time pointers removed and nothing else changed :
//   - the layer windows [firstLoadLE[k], firstLoadLE[k-1]) are kept -- load-based, always exact ;
//   - each layer keeps Vidal's deque ranked by key = p[i] + k*A[i], and evicts from the front on load only ;
//   - the query walks the deque from the front and takes the first start that fits the horizon. An
//     infeasible start is stepped over, never popped, so it is still there if it revives in a later column.
// With no frontier there is no horizon bound on the layer count : every layer up to ceil(load(0,j)/Q) is
// visited in every column. The key-only back-pop is still assumed safe for feasibility.
// Recording-free copy of Program/Split_Layered_PTVRP_cont.
class Split_Layered_PTVRP_cont: public Split
{

private:

	vector < double > potential ;
	vector < int > pred ;

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

	Split_Layered_PTVRP_cont(Pb_Data * myData) : Split(myData) {}

	int solve();
};

#endif
