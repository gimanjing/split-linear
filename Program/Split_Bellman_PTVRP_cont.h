#ifndef SPLIT_BELLMAN_PTVRP_CONT_H
#define SPLIT_BELLMAN_PTVRP_CONT_H

#include "Split.h"

// Bellman-style Split for the PT-VRP WITHOUT the early stop : O(n^2).
//
// Identical to Split_Bellman_PTVRP except that an infeasible template (i,j) is skipped with "continue"
// instead of ending the scan for start i with "break". The early stop is exact only if tau(i,j) is
// non-decreasing in j, which needs the triangle inequality on travel times; TSPLIB rounds each distance
// independently and can violate it by one unit. This solver therefore makes no such assumption, and is
// the reference for checking whether the "break" in Split_Bellman_PTVRP -- and the matching pointer
// sweeps in Split_Layered_PTVRP -- ever change an answer on real instances (NOTES.md §7.4).
class Split_Bellman_PTVRP_cont: public Split
{

private:

	// Potential vector : potential[j] = min cost of a partition of vendors [1..j]
	vector < double > potential ;

	// Predecessor of j in an optimal partition
	vector < int > pred ;

	// Recorded d(sigma), tau(sigma), m(sigma) of the arc (pred[j], j) that last improved potential[j]
	vector < double > predD ;
	vector < double > predT ;
	vector < int > predM ;

	// Arcs (i,j) that are feasible although some (i,j') with j' < j was not : exactly the arcs the
	// early stop never evaluates. revivedImproving counts those that improved potential[j] when seen.
	long revivedArcs ;
	long revivedImproving ;

public:

	Split_Bellman_PTVRP_cont(Pb_Data * myData) : Split(myData), revivedArcs(0), revivedImproving(0) {}

	int solve();
};

#endif
