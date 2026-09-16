#ifndef SPLIT_BELLMAN_PTVRP_H
#define SPLIT_BELLMAN_PTVRP_H

#include "Split.h"

// Bellman-style Split for the PT-VRP, O(n*B). The arc (i,j) costs d(i,j)*m(i,j) and exists only if
// tau(i,j)*m(i,j) <= horizon. Each start's scan stops at the first template over the horizon, which
// assumes tau(i,j) is non-decreasing in j (triangle inequality on travel times).
// Recording-free copy of Program/Split_Bellman_PTVRP : see there for the full argument.
class Split_Bellman_PTVRP: public Split
{

private:

	// Potential vector : potential[j] = min cost of a partition of vendors [1..j]
	vector < double > potential ;

	// Predecessor of j in an optimal partition
	vector < int > pred ;

public:

	Split_Bellman_PTVRP(Pb_Data * myData) : Split(myData) {}

	int solve();
};

#endif
