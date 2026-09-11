
//--------------------------------------------------------
//LAYERED-K SPLIT FOR THE PERIODIC-TEMPLATE VRP (PT-VRP)
//Extends the LIBRARY OF SPLIT ALGORITHM FOR VEHICLE ROUTING PROBLEMS (Thibaut VIDAL, 2015)
//See Split_Layered_PTVRP.h for why fixing the trip count restores the deque's validity.
//--------------------------------------------------------

#include "Split_Layered_PTVRP.h"

int Split_Layered_PTVRP::solve()
{
	const int n = myData->nbNodes ;
	const double Q = myData->vehCapacity ;
	const double speed = myData->speed ;
	const double horizon = myData->horizon ;

	potential = vector <double> (n+1, 1.e30) ;
	pred = vector <int> (n+1, -1) ;
	predK = vector <int> (n+1, 0) ;
	sumDistance = vector <double> (n+1, 0) ;
	sumLoad = vector <double> (n+1, 0) ;

	for (int i = 1 ; i <= n ; i++)
	{
		sumLoad[i] = sumLoad[i-1] + myData->cli[i].demand ;
		sumDistance[i] = sumDistance[i-1] + myData->cli[i-1].dnext ;
	}
	potential[0] = 0 ;

	// d(i,j) = A[i] + B[j] : the start term keeps the depot-out leg minus the inner prefix it will not
	// travel, the end term keeps the inner prefix up to j plus the depot-in leg.
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

	// The windows below slide with two pointers, which assumes At non-increasing and Bt non-decreasing.
	// Both follow from the triangle inequality; count where that fails instead of refusing to run.
	monotonicityViolations = 0 ;
	for (int i = 0 ; i + 1 < n ; i++)
		if (At[i+1] > At[i] + 1.e-9) monotonicityViolations ++ ;
	for (int j = 1 ; j < n ; j++)
		if (Bt[j+1] < Bt[j] - 1.e-9) monotonicityViolations ++ ;

	layersFullTour = trips(0, n) ;

	// firstLoadLE[k] : first start whose segment to the current j needs at most k trips.
	// firstTimeLE[k] : first start whose segment to the current j fits the horizon at k trips.
	// Every pointer is monotone in j, so each index enters each layer's deque at most once.
	// The containers start small and grow on demand : allocating ceil(total demand / Q) layers up front
	// would reserve thousands of deques that the horizon bound below never lets the loop reach.
	vector <int> firstLoadLE (2, 0) ;
	vector <int> firstTimeLE (2, 0) ;
	vector < deque<int> > queues (2) ;
	vector <int> nextAdd (2, 0) ;

	maxLayerUsed = 0 ;
	layersAllocated = 0 ;

	// Frontier of feasibility : the first start whose template to j fits the horizon at all. Both
	// tau(i,j) and m(i,j) grow as the template is extended backwards, so their product does too and the
	// feasible starts form a suffix whose left end only moves right as j grows -- the same monotonicity
	// the Bellman solver uses to stop its inner loop.
	int firstFeasible = 0 ;

	for (int j = 1 ; j <= n ; j++)
	{
		while (firstFeasible < j
		       && (At[firstFeasible] + Bt[j]) * (double) trips(firstFeasible, j) > horizon + 1.e-9)
			firstFeasible ++ ;
		if (firstFeasible >= j)
			continue ; // not even the singleton {j} fits the horizon

		// No feasible start needs more trips than the earliest feasible one, so this caps the layer
		// loop far below ceil(total demand / Q) whenever the horizon binds before the load does.
		int columnK = trips(firstFeasible, j) ;
		int kHi = (PTVRP_MAX_K > 0 && PTVRP_MAX_K < columnK) ? PTVRP_MAX_K : columnK ;

		if (kHi + 1 > (int) queues.size())
		{
			int grown = kHi + 2 ;
			firstLoadLE.resize(grown + 1, 0) ;
			firstTimeLE.resize(grown + 1, 0) ;
			queues.resize(grown) ;
			nextAdd.resize(grown, 0) ;
		}
		if (kHi > layersAllocated) layersAllocated = kHi ;

		for (int k = 1 ; k <= kHi ; k++)
		{
			// starts needing exactly k trips : [firstLoadLE[k], firstLoadLE[k-1]), with j closing k = 1
			while (firstLoadLE[k] < j && trips(firstLoadLE[k], j) > k)
				firstLoadLE[k] ++ ;

			int loadLower = firstLoadLE[k] ;
			int loadUpper = (k == 1) ? j : firstLoadLE[k-1] ;
			if (loadLower >= loadUpper)
				continue ;

			// horizon at k trips : tau(i,j)*k <= T_H  <=>  At[i] <= T_H/k - Bt[j], a suffix in i
			double timeBound = horizon / (double) k - Bt[j] ;
			while (firstTimeLE[k] < j && At[firstTimeLE[k]] > timeBound + 1.e-9)
				firstTimeLE[k] ++ ;

			int lower = (loadLower > firstTimeLE[k]) ? loadLower : firstTimeLE[k] ;
			int upper = (loadUpper < j) ? loadUpper : j ;
			if (lower >= upper)
				continue ;

			deque<int> & dq = queues[k] ;

			while (!dq.empty() && dq.front() < lower)
				dq.pop_front() ;

			if (nextAdd[k] < lower)
				nextAdd[k] = lower ;

			while (nextAdd[k] < upper)
			{
				int i = nextAdd[k] ;
				nextAdd[k] ++ ;
				if (potential[i] > 1.e29)
					continue ;
				double ki = key(i, k) ;
				// on ties keep the newer predecessor, matching the Bellman solver's preference
				while (!dq.empty() && key(dq.back(), k) >= ki)
					dq.pop_back() ;
				dq.push_back(i) ;
			}

			while (!dq.empty() && dq.front() < lower)
				dq.pop_front() ;

			if (dq.empty())
				continue ;

			int front = dq.front() ;
			double cand = key(front, k) + (double) k * B[j] ;
			if (cand < potential[j])
			{
				potential[j] = cand ;
				pred[j] = front ;
				predK[j] = k ;
				if (k > maxLayerUsed) maxLayerUsed = k ;
			}
		}
	}

	// THE CORE OF THE SPLIT ALGORITHM IS FINISHED HERE,
	// NOW JUST SWEEPING THE ROUTE in O(n) TO REPORT THE SOLUTION (IN THE GOOD DIRECTION)

	if (potential[n] > 1.e29)
	{
		cout << "ERROR : no Split solution has been propagated until the last node" << endl ;
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

	maxLayerOnPath = 0 ;
	cour = n ;
	for (int i = myData->solutionNbRoutes-1 ; i >= 0 ; i--)
	{
		int endNode = cour ;
		int k = predK[cour] ;
		cour = pred[cour] ;
		myData->solution[i] = cour+1 ;
		myData->solutionTemplateDist[i]  = A[cour] + B[endNode] ;
		myData->solutionTemplateTime[i]  = At[cour] + Bt[endNode] ;
		myData->solutionTemplateTrips[i] = k ;
		if (k > maxLayerOnPath) maxLayerOnPath = k ;
	}

	myData->solutionCost = potential[n] ;

	cout << endl ;
	cout << "K FULL TOUR          : " << layersFullTour << "   (ceil(total demand / Q), the naive layer count)" << endl ;
	cout << "LAYERS ALLOCATED (K) : " << layersAllocated
		 << "   (after the horizon bound" << (PTVRP_MAX_K > 0 ? " and MAX_K cap" : "") << ")" << endl ;
	cout << "MAX LAYER USED       : " << maxLayerUsed << "   (largest k that improved any label)" << endl ;
	cout << "MAX LAYER ON PATH    : " << maxLayerOnPath << "   (largest m(sigma) in the solution)" << endl ;
	if (monotonicityViolations > 0)
		cout << "WARNING : " << monotonicityViolations
			 << " positions break the monotonicity the sliding windows assume (rounded distances)" << endl ;

	return 0 ;
}
