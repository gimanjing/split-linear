#ifndef SPLIT_LAYERED_PTVRP_H
#define SPLIT_LAYERED_PTVRP_H

#include "Split.h"
#include <deque>

// Layered-K Split for PT-VRP : one DP layer per integer trip count.
//
// The plain deque (Split_Linear_PTVRP) fails because d(i,j)*m(i,j) is not separable -- m steps at
// different j for different i, so an ordering of predecessors established once can invert later.
// Fixing k removes exactly that problem. Within layer k the multiplier is a constant, so
//
//     cost_k(i,j) = k * d(i,j) = k * A[i] + k * B[j]
//
// is separable again, the difference cost_k(i1,j) - cost_k(i2,j) = k*(A[i1]-A[i2]) is independent of j,
// and Vidal's Property 2 holds inside the layer. One monotone deque per layer is therefore valid, and
// the answer is the best candidate over all layers. Complexity is O(n*K) for K layers.
//
// The layers partition the predecessors rather than duplicating them : for a given j the starts needing
// exactly k trips form the contiguous range [firstLoadLE[k], firstLoadLE[k-1]), and every one of those
// pointers only ever moves right as j grows, so each index enters each layer's deque at most once.
//
// Ported from a Rust layered-K decoder. That version also carries soft trip/route-time penalties, which
// split each layer into up to three linear regimes; PT-VRP has a hard horizon instead of penalties, so
// the regimes collapse to one and the horizon appears only as a feasibility window on the predecessors.
//
// Monotonicity: the two-pointer windows assume At[] is non-increasing and Bt[] non-decreasing, which
// follows from the triangle inequality on travel times. The TSPLIB instances round each distance
// independently and so violate it by up to one unit; rather than refuse to run, the solver counts the
// violations and reports them, leaving the size of any resulting error to be measured against the
// exact Bellman DP.
class Split_Layered_PTVRP: public Split
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

	// Trips the whole giant tour would need, ceil(total demand / Q) : the naive layer count
	int layersFullTour ;

	// Layers the DP actually allocated, after the horizon bound (and PTVRP_MAX_K) cut the loop down
	int layersAllocated ;

	// Largest layer that ever produced an accepted improvement anywhere in the DP
	int maxLayerUsed ;

	// Largest layer appearing on the final path (equals the largest m(sigma) in the solution)
	int maxLayerOnPath ;

	// Total (column, layer) pairs the loop executed : this, divided by n, is the K that actually
	// drives the O(n*K) cost -- not the largest layer that happened to improve a label
	long layerIterations ;

	// Count of positions where the monotonicity the two-pointer windows assume does not hold
	int monotonicityViolations ;

	Split_Layered_PTVRP(Pb_Data * myData) : Split(myData),
		layersFullTour(0), layersAllocated(0), maxLayerUsed(0), maxLayerOnPath(0),
		layerIterations(0), monotonicityViolations(0) {}

	int solve();
};

// Hard cap on the number of layers. 0 means uncapped: allocate every layer up to the trip count the
// whole giant tour needs. Set it to 5 to admit only templates of at most five trips.
const int PTVRP_MAX_K = 0 ;

#endif
