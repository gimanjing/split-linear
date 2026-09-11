#ifndef SPLIT_BELLMAN_PTVRP_H
#define SPLIT_BELLMAN_PTVRP_H

#include "Split.h"

// Bellman-style Split for the Periodic-Template VRP (PT-VRP).
//
// A template (segment of the giant tour) sigma is executed m(sigma) = ceil(q(sigma)/Q) times over the
// horizon. Cost and duration are therefore d(sigma)*m(sigma) and tau(sigma)*m(sigma) rather than the
// plain d(sigma)/tau(sigma) of the classical CVRP Split. Since m(sigma) is a non-linear step function of
// the segment's cumulative demand, the O(n) dominance argument used by Split_Linear does not apply here
// (see Split_Bellman_PTVRP.cpp for the argument, and the accompanying counterexample). This class keeps
// the classical O(n) outer / O(B) inner Bellman structure and only swaps the feasibility test and the
// arc-cost formula.
class Split_Bellman_PTVRP: public Split
{

private:

	// Potential vector : potential[j] = min cost of a partition of vendors [1..j]
	vector < double > potential ;

	// Predecessor of j in an optimal partition
	vector < int > pred ;

	// Recorded d(sigma), tau(sigma), m(sigma) of the arc (pred[j], j) that last improved potential[j],
	// kept so the final solution can be reported without rescanning every template
	vector < double > predD ;
	vector < double > predT ;
	vector < int > predM ;

public:

	Split_Bellman_PTVRP(Pb_Data * myData) : Split(myData) {}

	int solve();
};

#endif
