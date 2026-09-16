#ifndef SPLIT_BELLMAN_PTVRP_CONT_H
#define SPLIT_BELLMAN_PTVRP_CONT_H

#include "Split.h"

// Bellman-style Split for the PT-VRP WITHOUT the early stop : O(n^2). An infeasible template is skipped
// with "continue" instead of ending the scan, so every pair (i,j) is evaluated and no triangle-inequality
// assumption is made. The exact reference (oracle) for the other PT-VRP solvers.
// Recording-free copy of Program/Split_Bellman_PTVRP_cont.
class Split_Bellman_PTVRP_cont: public Split
{

private:

	// Potential vector : potential[j] = min cost of a partition of vendors [1..j]
	vector < double > potential ;

	// Predecessor of j in an optimal partition
	vector < int > pred ;

public:

	Split_Bellman_PTVRP_cont(Pb_Data * myData) : Split(myData) {}

	int solve();
};

#endif
