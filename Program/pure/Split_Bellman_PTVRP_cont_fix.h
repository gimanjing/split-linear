#ifndef SPLIT_BELLMAN_PTVRP_CONT_FIX_H
#define SPLIT_BELLMAN_PTVRP_CONT_FIX_H

#include "Split.h"

// Bellman-style Split for the PT-VRP with a SOUND early stop : O(n*B) with B bounded by the horizon.
// The scan stops on the PATH OUT (the route without the leg home), which never shrinks as the template
// grows, so pathout*m > T_H proves every longer template from i is infeasible. The test that decides
// whether an arc may be used stays exact, on the full route. Pruning is conservative, feasibility exact.
// Recording-free copy of Program/Split_Bellman_PTVRP_cont_fix : see there for the full argument.
class Split_Bellman_PTVRP_cont_fix: public Split
{

private:

	vector < double > potential ;
	vector < int > pred ;

public:

	Split_Bellman_PTVRP_cont_fix(Pb_Data * myData) : Split(myData) {}

	int solve();
};

#endif
