
//--------------------------------------------------------
//UNSOUND CONTROL : VIDAL'S O(n) DEQUE SPLIT APPLIED TO THE PT-VRP COST
//Extends the LIBRARY OF SPLIT ALGORITHM FOR VEHICLE ROUTING PROBLEMS (Thibaut VIDAL, 2015)
//See Split_Linear_PTVRP.h for why this cannot be correct.
//--------------------------------------------------------

#include "Split_Linear_PTVRP.h"

int Split_Linear_PTVRP::solve()
{
	// Initialization of the data structures -- identical to Split_Linear
	potential = vector <double> (myData->nbNodes+1) ;
	pred = vector <int> (myData->nbNodes+1) ;
	sumDistance = vector <double> (myData->nbNodes+1) ;
	sumLoad = vector <double> (myData->nbNodes+1) ;
	potential[0] = 0 ;
	pred[0] = -1 ;
	sumDistance[0] = 0 ;
	sumLoad[0] = 0 ;

	for (int i = 1 ; i <= myData->nbNodes ; i++)
	{
		potential[i] = 1.e30 ;
		pred[i] = -1 ;
		sumLoad[i] = sumLoad[i-1] + myData->cli[i].demand ;
		sumDistance[i] = sumDistance[i-1] + myData->cli[i-1].dnext ;
	}

	Trivial_Deque queue = Trivial_Deque(myData->nbNodes+1, 0) ;

	// Main loop -- structurally identical to Split_Linear
	for (int i = 1 ; i <= myData->nbNodes ; i++)
	{
		// The front is taken as the best predecessor for i. Under the PT-VRP cost this is exactly the
		// unjustified step : the ordering that put this node at the front was established with the
		// multiplier ignored, so the front need not be the best predecessor here at all.
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
			// size() is needed here but not in Split_Linear : there the capacity precondition q_i <= Q
			// keeps the queue non-empty, whereas here the broken dominance can already have discarded
			// the node that would have survived, and emptying the queue would read past the front.
			while (queue.size() > 1 && exceedsHorizon(queue.get_front(), i+1))
				queue.pop_front() ;
		}
	}

	// THE CORE OF THE SPLIT ALGORITHM IS FINISHED HERE,
	// NOW JUST SWEEPING THE ROUTE in O(n) TO REPORT THE SOLUTION (IN THE GOOD DIRECTION)

	if (potential[myData->nbNodes] > 1.e29)
	{
		cout << "ERROR : no Split solution has been propagated until the last node" << endl ;
		throw string ("ERROR : no Split solution has been propagated until the last node");
	}

	myData->solutionNbRoutes = 0 ;
	int cour = myData->nbNodes ;
	while (cour != 0)
	{
		cour = pred[cour] ;
		myData->solutionNbRoutes ++ ;
	}

	myData->solutionTemplateDist = vector <double> (myData->solutionNbRoutes) ;
	myData->solutionTemplateTime = vector <double> (myData->solutionNbRoutes) ;
	myData->solutionTemplateTrips = vector <int> (myData->solutionNbRoutes) ;

	cour = myData->nbNodes ;
	for (int i = myData->solutionNbRoutes-1 ; i >= 0 ; i--)
	{
		int endNode = cour ;
		cour = pred[cour] ;
		myData->solution[i] = cour+1 ;
		myData->solutionTemplateDist[i]  = tripDistance(cour, endNode) ;
		myData->solutionTemplateTime[i]  = tripDuration(cour, endNode) ;
		myData->solutionTemplateTrips[i] = trips(cour, endNode) ;
	}

	myData->solutionCost = potential[myData->nbNodes] ;

	return 0 ;
}
