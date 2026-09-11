
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

	if (myData->trace)
		cout << endl << "=== DEQUE (UNSOUND) : one front is trusted for every endpoint ===" << endl
			 << "    the queue ranks predecessors by fixed cost only, with no knowledge of m" << endl ;

	// Main loop -- structurally identical to Split_Linear
	for (int i = 1 ; i <= myData->nbNodes ; i++)
	{
		// The front is taken as the best predecessor for i. Under the PT-VRP cost this is exactly the
		// unjustified step : the ordering that put this node at the front was established with the
		// multiplier ignored, so the front need not be the best predecessor here at all.
		potential[i] = propagate(queue.get_front(), i) ;
		pred[i] = queue.get_front() ;

		if (myData->trace)
		{
			cout << endl << "  +-- p[" << i << "] : queue holds [ " ;
			for (int q = 0 ; q < queue.size() ; q++) cout << queue.get_front_at(q) << " " ;
			cout << "]  front=" << queue.get_front() << endl ;
			int f = queue.get_front() ;
			cout << "  |  front=" << f << " : d=" << tripDistance(f,i) << "  m=" << trips(f,i)
				 << "  cost=" << tripDistance(f,i) << "*" << trips(f,i)
				 << "   ->  p[" << i << "] = " << potential[i] << endl ;
			// what an exhaustive scan would have found, to expose the committed error
			double best = 1.e30 ; int bestI = -1 ;
			for (int c = 0 ; c < i ; c++)
			{
				if (potential[c] > 1.e29) continue ;
				if (exceedsHorizon(c, i)) continue ;
				double v = potential[c] + tripDistance(c,i) * (double) trips(c,i) ;
				if (v < best) { best = v ; bestI = c ; }
			}
			if (bestI >= 0 && best < potential[i] - 1.e-9)
				cout << "  |  *** but start=" << bestI << " gives " << best
					 << " -- the deque trusted the wrong front (off by "
					 << potential[i] - best << ") ***" << endl ;
		}

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
