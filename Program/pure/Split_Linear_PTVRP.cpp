
//--------------------------------------------------------
//UNSOUND CONTROL : VIDAL'S O(n) DEQUE SPLIT APPLIED TO THE PT-VRP COST
//Extends the LIBRARY OF SPLIT ALGORITHM FOR VEHICLE ROUTING PROBLEMS (Thibaut VIDAL, 2015)
//See Split_Linear_PTVRP.h for why this cannot be correct.
//--------------------------------------------------------

#include "Split_Linear_PTVRP.h"

int Split_Linear_PTVRP::solve()
{
	// Initialization of the data structures -- identical to Split_Linear
	potential = vector <double> (myData->nbNodes+1, 1.e30) ;
	pred = vector <int> (myData->nbNodes+1, -1) ;
	sumDistance = vector <double> (myData->nbNodes+1, 0) ;
	sumLoad = vector <double> (myData->nbNodes+1, 0) ;
	potential[0] = 0 ;

	for (int i = 1 ; i <= myData->nbNodes ; i++)
	{
		sumLoad[i] = sumLoad[i-1] + myData->cli[i].demand ;
		sumDistance[i] = sumDistance[i-1] + myData->cli[i-1].dnext ;
	}

	Trivial_Deque queue = Trivial_Deque(myData->nbNodes+1, 0) ;

	// Main loop -- structurally identical to Split_Linear
	for (int i = 1 ; i <= myData->nbNodes ; i++)
	{
		// The front is taken as the best predecessor for i : the unjustified step under the PT-VRP cost
		potential[i] = propagate(queue.get_front(), i) ;
		pred[i] = queue.get_front() ;

		if (i < myData->nbNodes)
		{
			if (!dominates(queue.get_back(), i))
			{
				while (queue.size() > 0 && dominatesRight(queue.get_back(), i))
					queue.pop_back() ;
				queue.push_back(i) ;
			}
			// Drop predecessors that can no longer reach the next node inside the horizon. The guard on
			// size() keeps the queue from emptying, since the broken dominance can already have discarded
			// the node that would have survived.
			while (queue.size() > 1 && exceedsHorizon(queue.get_front(), i+1))
				queue.pop_front() ;
		}
	}

	extractSolution(potential, pred) ;
	return 0 ;
}
