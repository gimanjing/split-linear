//--------------------------------------------------------
//LAYERED-K SPLIT FOR THE PT-VRP, NO HORIZON PRUNING
//Extends the LIBRARY OF SPLIT ALGORITHM FOR VEHICLE ROUTING PROBLEMS (Thibaut VIDAL, 2015)
//See Split_Layered_PTVRP_cont.h : the layer structure is kept, the two one-way pointers are not.
//--------------------------------------------------------

#include "Split_Layered_PTVRP_cont.h"

int Split_Layered_PTVRP_cont::solve()
{
	const int n = myData->nbNodes ;
	const double speed = myData->speed ;
	const double horizon = myData->horizon ;

	potential = vector <double> (n+1, 1.e30) ;
	pred = vector <int> (n+1, -1) ;
	predK = vector <int> (n+1, 0) ;
	sumDistance = vector <double> (n+1, 0) ;
	sumLoad = vector <double> (n+1, 0) ;
	revivedStarts = 0 ;
	revivedImproving = 0 ;

	for (int i = 1 ; i <= n ; i++)
	{
		sumLoad[i] = sumLoad[i-1] + myData->cli[i].demand ;
		sumDistance[i] = sumDistance[i-1] + myData->cli[i-1].dnext ;
	}
	potential[0] = 0 ;

	// Same separable decomposition as Split_Layered_PTVRP : d(i,j) = A[i] + B[j].
	A  = vector <double> (n, 0) ;
	At = vector <double> (n, 0) ;
	B  = vector <double> (n+1, 0) ;
	Bt = vector <double> (n+1, 0) ;
	for (int i = 0 ; i < n ; i++)
	{
		A[i]  = myData->cli[i+1].dreturn - sumDistance[i+1] ;
		At[i] = A[i] / speed - PTVRP_SERVICE_TIME * (double) i ;
	}
	for (int j = 1 ; j <= n ; j++)
	{
		B[j]  = sumDistance[j] + myData->cli[j].dreturn ;
		Bt[j] = B[j] / speed + PTVRP_SERVICE_TIME * (double) j ;
	}

	// Where Split_Layered_PTVRP's one-way frontier would stand. Tracked only to count the starts the
	// pruning would have thrown away; it never restricts the scan below.
	int monotoneFrontier = 0 ;

	if (myData->trace)
		cout << endl << "=== LAYERED, NO HORIZON PRUNING : every start scanned, feasibility tested one by one ===" << endl
			 << "    the layer partition by trip count is kept -- it is load-based and always monotone" << endl ;

	for (int j = 1 ; j <= n ; j++)
	{
		while (monotoneFrontier < j
		       && (At[monotoneFrontier] + Bt[j]) * (double) trips(monotoneFrontier, j) > horizon + 1.e-9)
			monotoneFrontier ++ ;

		// Deepest layer any start could need. No frontier bound : the whole prefix is in scope.
		int kHi = trips(0, j) ;

		if (myData->trace)
			cout << endl << "  +-- p[" << j << "] : scanning starts [0," << j << "), layers 1.." << kHi
				 << "   (pruned solver would start at " << monotoneFrontier << ")" << endl ;

		for (int k = 1 ; k <= kHi ; k++)
		{
			int bestI = -1 ;
			double bestCand = 1.e30 ;

			for (int i = 0 ; i < j ; i++)
			{
				if (trips(i, j) != k) continue ;        // layer membership : load-based, always exact
				if (potential[i] > 1.e29) continue ;

				double tau = At[i] + Bt[j] ;
				if (tau * (double) k > horizon + 1.e-9) continue ;   // tested per start, never pruned

				double cand = potential[i] + (double) k * (A[i] + B[j]) ;

				if (i < monotoneFrontier)
				{
					revivedStarts ++ ;
					if (cand < potential[j]) revivedImproving ++ ;
				}
				if (cand < bestCand) { bestCand = cand ; bestI = i ; }
			}

			if (bestI < 0) continue ;

			if (myData->trace)
				cout << "  |  layer k=" << k << " : best start " << bestI
					 << "   cand = p[" << bestI << "] + " << k << "*d(" << bestI << "," << j << ")"
					 << " = " << bestCand
					 << (bestI < monotoneFrontier ? "   [behind the frontier]" : "")
					 << ((bestCand < potential[j]) ? "   <- best so far" : "") << endl ;

			if (bestCand < potential[j])
			{
				potential[j] = bestCand ;
				pred[j] = bestI ;
				predK[j] = k ;
			}
		}

		if (myData->trace && pred[j] >= 0)
			cout << "  +-- p[" << j << "] = " << potential[j] << "  (predecessor " << pred[j]
				 << ", layer " << predK[j] << ")" << endl ;
	}

	// A revived start that improves a label is necessary, not sufficient, for the pruning to change the
	// answer : a later start may improve the same label further. Compare SOLUTION COST against
	// PTVRP_LAYERED to decide.
	cout << "REVIVED STARTS : " << revivedStarts << endl ;
	cout << "REVIVED IMPROVING : " << revivedImproving << endl ;

	if (potential[n] > 1.e29)
	{
		cout << "ERROR : no Split solution has been propagated until the last node "
			 << "(every vendor beyond this point is infeasible within the horizon)" << endl ;
		throw string ("ERROR : no Split solution has been propagated until the last node");
	}

	myData->solutionNbRoutes = 0 ;
	int cour = n ;
	while (cour != 0)
	{
		cour = pred[cour] ;
		myData->solutionNbRoutes ++ ;
	}

	myData->solutionTemplateDist  = vector <double> (myData->solutionNbRoutes) ;
	myData->solutionTemplateTime  = vector <double> (myData->solutionNbRoutes) ;
	myData->solutionTemplateTrips = vector <int> (myData->solutionNbRoutes) ;

	cour = n ;
	for (int i = myData->solutionNbRoutes-1 ; i >= 0 ; i--)
	{
		int p = pred[cour] ;
		myData->solutionTemplateDist[i]  = A[p] + B[cour] ;
		myData->solutionTemplateTime[i]  = (At[p] + Bt[cour]) ;
		myData->solutionTemplateTrips[i] = predK[cour] ;
		cour = p ;
		myData->solution[i] = cour+1 ;
	}

	myData->solutionCost = potential[n] ;

	return 0 ;
}
