#ifndef SPLIT_LAYERED_PTVRP_H
#define SPLIT_LAYERED_PTVRP_H

#include "Split.h"
#include <deque>

// Layered-K Split for PT-VRP : one DP layer per integer trip count, O(n*K).
//
// Within layer k the multiplier is a constant, so cost_k(i,j) = k*A[i] + k*B[j] is separable, Vidal's
// Property 2 holds inside the layer, and one monotone deque per layer is valid. The answer is the best
// candidate over all layers. Two one-way time pointers prune on the full route time, which assumes the
// triangle inequality on travel times.
// Recording-free copy of Program/Split_Layered_PTVRP : see there for the full argument.
class Split_Layered_PTVRP: public Split
{

private:

	vector < double > potential ;
	vector < int > pred ;

	vector < double > sumDistance ;
	vector < double > sumLoad ;

	// Separable decomposition : d(i,j) = A[i] + B[j], tau(i,j) = At[i] + Bt[j]
	vector < double > A, B, At, Bt ;

	// m(i,j), at least one to match the Bellman solver's convention
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

	Split_Layered_PTVRP(Pb_Data * myData) : Split(myData) {}

	int solve();
};

// Hard cap on the number of layers. 0 means uncapped: allocate every layer up to the trip count the
// whole giant tour needs. Set it to 5 to admit only templates of at most five trips.
const int PTVRP_MAX_K = 0 ;

#endif
