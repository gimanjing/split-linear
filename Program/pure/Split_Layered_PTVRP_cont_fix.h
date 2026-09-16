#ifndef SPLIT_LAYERED_PTVRP_CONT_FIX_H
#define SPLIT_LAYERED_PTVRP_CONT_FIX_H

#include "Split.h"
#include <deque>

// Layered-K Split for the PT-VRP with a SOUND horizon bound : O(n K) with K bounded by the horizon.
//
// Prunes on the PATH OUT (the route without the leg home), pathout(i,j) = A[i] + sumDistance[j], which is
// non-decreasing in j unconditionally, so a start dropped on it can never fit again :
//   frontierPathOut -- column frontier, bounds the layer count at trips(frontierPathOut, j) ;
//   firstTimeLE[k]  -- per-layer suffix, At[i] > T_H/k - Bout_t[j] means i is dead in layer k and above.
// The query walks the deque from the front and takes the first start that fits the REAL horizon, tested
// with the true tau = At[i] + Bt[j]. Pruning is conservative, feasibility exact. The key-only back-pop is
// still assumed safe for feasibility.
// Recording-free copy of Program/Split_Layered_PTVRP_cont_fix : see there for the full argument.
class Split_Layered_PTVRP_cont_fix: public Split
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

	Split_Layered_PTVRP_cont_fix(Pb_Data * myData) : Split(myData) {}

	int solve();
};

#endif
