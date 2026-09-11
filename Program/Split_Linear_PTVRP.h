#ifndef SPLIT_LINEAR_PTVRP_H
#define SPLIT_LINEAR_PTVRP_H

#include "Split.h"

// DELIBERATELY UNSOUND. Vidal's O(n) deque Split applied to the PT-VRP cost structure, which does not
// satisfy the property that makes the deque valid. It is a control, not a solver: run it against
// Split_Bellman_PTVRP to measure what the shortcut costs. Do not use its output as a solution.
//
// Why it cannot be right. The O(n) algorithm keeps a queue of non-dominated predecessors and commits to
// the front as the best predecessor for every remaining node. That commitment is only justified by
// Vidal's Property 2 : for two predecessors i1 < i2, the difference c(i1,j) - c(i2,j) must be a constant
// independent of j, so an ordering established once stays valid forever. PT-VRP cost is d(i,j)*m(i,j)
// with m(i,j) = ceil((Q[j]-Q[i])/Q), a step function whose breakpoints sit at different j for different
// i, so the difference jumps whenever either multiplier increments and the ordering can invert.
//
// The construction below is Split_Linear with exactly two edits, so the damage is attributable:
//   - propagate() multiplies the trip distance by m(i,j)
//   - the front-eviction test uses the PT-VRP duration constraint instead of the capacity constraint
// dominates() and dominatesRight() are left byte-identical to Split_Linear, i.e. they still rank
// predecessors on fixed cost alone and never look at the multiplier. That is the unsound step.
class Split_Linear_PTVRP: public Split
{

private:

	vector < double > potential ;
	vector < int > pred ;

	// sumDistance[i] for i > 1 contains the sum of distances : sum_{k=1}^{i-1} d_{k,k+1}
	vector < double > sumDistance ;

	// sumLoad[i] for i >= 1 contains the sum of loads : sum_{k=1}^{i} q_k
	vector < double > sumLoad ;

	// m(i,j) : number of executions the template i+1..j needs. At least one, matching the Bellman solver.
	inline int trips(int i, int j)
	{
		int m = (int) ceil((sumLoad[j] - sumLoad[i]) / myData->vehCapacity - 1.e-9) ;
		return (m < 1) ? 1 : m ;
	}

	// d(sigma) : distance of ONE trip of the template i+1..j
	inline double tripDistance(int i, int j)
	{
		return sumDistance[j] - sumDistance[i+1] + myData->cli[i+1].dreturn + myData->cli[j].dreturn ;
	}

	// tau(sigma) : duration of ONE trip of the template i+1..j, travel plus service of every vendor
	inline double tripDuration(int i, int j)
	{
		return tripDistance(i, j) / myData->speed + PTVRP_SERVICE_TIME * (double)(j - i) ;
	}

	// EDIT 1 of 2 versus Split_Linear : the trip distance is scaled by the multiplier
	inline double propagate(int i, int j)
	{
		return potential[i] + tripDistance(i, j) * (double) trips(i, j) ;
	}

	// EDIT 2 of 2 versus Split_Linear : a predecessor is dropped from the front once the template it
	// would form can no longer fit the horizon, rather than once it exceeds vehicle capacity
	inline bool exceedsHorizon(int i, int j)
	{
		return tripDuration(i, j) * (double) trips(i, j) > myData->horizon + 0.0001 ;
	}

	// UNCHANGED from Split_Linear -- and this is precisely the unsound part : both tests rank
	// predecessors by fixed cost and equal cumulative load, with no knowledge of the multiplier.
	inline bool dominates(int i, int j)
	{
		return (sumLoad[i] == sumLoad[j] && potential[j] + myData->cli[j+1].dreturn > potential[i] + myData->cli[i+1].dreturn + sumDistance[j+1] - sumDistance[i+1] - 0.0001);
	}

	inline bool dominatesRight(int i, int j)
	{
		return potential[j] + myData->cli[j+1].dreturn < potential[i] + myData->cli[i+1].dreturn + sumDistance[j+1] - sumDistance[i+1] + 0.0001 ;
	}

public:

	Split_Linear_PTVRP(Pb_Data * myData) : Split(myData) {}

	int solve();
};

#endif
