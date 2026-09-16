//--------------------------------------------------------
//LAYERED-K SPLIT FOR THE PT-VRP, SOUND BOUND AND SOUND EVICTION
//Extends the LIBRARY OF SPLIT ALGORITHM FOR VEHICLE ROUTING PROBLEMS (Thibaut VIDAL, 2015)
//See Split_Layered_PTVRP_safe.h : path-out pruning, plus eviction on BOTH cost and feasibility.
//--------------------------------------------------------

#include "Split_Layered_PTVRP_safe.h"

int Split_Layered_PTVRP_safe::solve()
{
	const int n = myData->nbNodes ;
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

	A  = vector <double> (n, 0) ;
	At = vector <double> (n, 0) ;
	B  = vector <double> (n+1, 0) ;
	Bt = vector <double> (n+1, 0) ;
	Bout_t = vector <double> (n+1, 0) ;
	for (int i = 0 ; i < n ; i++)
	{
		A[i]  = myData->cli[i+1].dreturn - sumDistance[i+1] ;
		At[i] = A[i] / speed - PTVRP_SERVICE_TIME * (double) i ;
	}
	for (int j = 1 ; j <= n ; j++)
	{
		B[j]  = sumDistance[j] + myData->cli[j].dreturn ;
		Bt[j] = B[j] / speed + PTVRP_SERVICE_TIME * (double) j ;
		// path out : the same prefix WITHOUT the leg home. Non-decreasing in j, unconditionally.
		Bout_t[j] = sumDistance[j] / speed + PTVRP_SERVICE_TIME * (double) j ;
	}

	// Reported for comparison only; this solver's pointers do not rely on it.
	monotonicityViolations = 0 ;
	for (int i = 0 ; i + 1 < n ; i++)
		if (At[i+1] > At[i] + 1.e-9) monotonicityViolations ++ ;
	for (int j = 1 ; j < n ; j++)
		if (Bt[j+1] < Bt[j] - 1.e-9) monotonicityViolations ++ ;

	layersFullTour = trips(0, n) ;

	vector <int> firstLoadLE (2, 0) ;
	vector <int> firstTimeLE (2, 0) ;
	vector < deque<int> > queues (2) ;
	vector <int> nextAdd (2, 0) ;
	// a layer stays key-sorted until the guard blocks its first pop
	vector <char> sortedByKey (2, 1) ;

	// The frontier Split_Layered_PTVRP would use : full route time, not sound. Replayed for counting only.
	int frontierFullRoute = 0 ;

	// This solver's frontier : path out. A start dropped here can never fit again, whatever the rounding.
	int frontierPathOut = 0 ;

	if (myData->trace)
		cout << endl << "=== LAYERED, SOUND BOUND + SOUND EVICTION : evict only when cheaper AND at least as feasible ===" << endl
			 << "    pathout(i,j) = A[i] + sumDistance[j] never shrinks, so dropping a start on it is permanent" << endl ;

	for (int j = 1 ; j <= n ; j++)
	{
		// Sound : both factors are non-decreasing in j, so once this fires for a start it fires for ever.
		while (frontierPathOut < j
		       && (At[frontierPathOut] + Bout_t[j]) * (double) trips(frontierPathOut, j) > horizon + 1.e-9)
			frontierPathOut ++ ;

		// What the full-route frontier would have done. Never restricts the DP; counted only.
		while (frontierFullRoute < j
		       && (At[frontierFullRoute] + Bt[j]) * (double) trips(frontierFullRoute, j) > horizon + 1.e-9)
			frontierFullRoute ++ ;

		if (frontierPathOut >= j)
			continue ; // not even the singleton {j} can fit, on a bound that ignores the leg home

		int kHi = trips(frontierPathOut, j) ;
		if (kHi > layersAllocated) layersAllocated = kHi ;

		if (myData->trace)
			cout << endl << "  +-- p[" << j << "] : path-out frontier at " << frontierPathOut
				 << ", layers 1.." << kHi
				 << "   (full-route frontier would stand at " << frontierFullRoute << ")" << endl ;

		if (kHi + 1 > (int) queues.size())
		{
			int grown = kHi + 2 ;
			firstLoadLE.resize(grown + 1, 0) ;
			firstTimeLE.resize(grown + 1, 0) ;
			sortedByKey.resize(grown, 1) ;
			queues.resize(grown) ;
			nextAdd.resize(grown, 0) ;
		}

		for (int k = 1 ; k <= kHi ; k++)
		{
			layerIterations ++ ;

			while (firstLoadLE[k] < j && trips(firstLoadLE[k], j) > k)
				firstLoadLE[k] ++ ;

			int loadLower = firstLoadLE[k] ;
			int loadUpper = (k == 1) ? j : firstLoadLE[k-1] ;
			if (loadLower >= loadUpper)
				continue ;

			// Sound per-layer suffix : At[i] > T_H/k - Bout_t[j] means i cannot fit at k trips now, and
			// Bout_t only grows while trips only grows, so it cannot fit at k or above ever again.
			double timeBound = horizon / (double) k - Bout_t[j] ;
			while (firstTimeLE[k] < j && At[firstTimeLE[k]] > timeBound + 1.e-9)
				firstTimeLE[k] ++ ;

			int lower = loadLower ;
			if (firstTimeLE[k] > lower) lower = firstTimeLE[k] ;
			if (frontierPathOut > lower) lower = frontierPathOut ;
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
				// Evict only on JOINT dominance : the newcomer must be at least as cheap AND at least
				// as feasible. At[new] <= At[old] means "wherever old fits, new fits", at every j.
				while (!dq.empty() && key(dq.back(), k) >= ki)
				{
					if (At[i] > At[dq.back()] + 1.e-9)
					{
						// cheaper but harder to fit : keep the incumbent, and the layer is no longer
						// sorted by key, so its query has to scan.
						blockedPops ++ ;
						sortedByKey[k] = 0 ;
						break ;
					}
					dq.pop_back() ;
				}
				dq.push_back(i) ;
			}

			// Feasibility itself is tested exactly, on the true route time. Nothing is popped on time.
			// While the layer is still key-sorted the first feasible entry is the cheapest one, which is
			// the original O(1) query. Once the guard has blocked a pop that ordering is gone, so the
			// whole deque is scanned for the cheapest feasible entry instead.
			int front = -1 ;
			double bestKey = 1.e30 ;
			if (sortedByKey[k])
			{
				for (size_t q = 0 ; q < dq.size() ; q++)
				{
					if ((At[dq[q]] + Bt[j]) * (double) k <= horizon + 1.e-9)
					{
						front = dq[q] ; bestKey = key(front, k) ;
						break ;
					}
					infeasibleSkips ++ ;
				}
			}
			else
			{
				unsortedQueries ++ ;
				for (size_t q = 0 ; q < dq.size() ; q++)
				{
					if ((At[dq[q]] + Bt[j]) * (double) k > horizon + 1.e-9) { infeasibleSkips ++ ; continue ; }
					double kk = key(dq[q], k) ;
					if (kk < bestKey) { bestKey = kk ; front = dq[q] ; }
				}
			}

			if (front < 0)
				continue ;

			double cand = bestKey + (double) k * B[j] ;
			bool behind = (front < frontierFullRoute) ;
			if (behind)
			{
				revivedStarts ++ ;
				if (cand < potential[j]) revivedImproving ++ ;
			}

			if (myData->trace)
			{
				cout << "  |  layer k=" << k << " : starts [" << lower << "," << upper << ")  deque [ " ;
				for (size_t q = 0 ; q < dq.size() ; q++) cout << dq[q] << " " ;
				cout << "]  first feasible=" << front
					 << "   cand = p[" << front << "] + " << k << "*d(" << front << "," << j << ")"
					 << " = " << cand
					 << (behind ? "   [behind the full-route frontier]" : "")
					 << ((cand < potential[j]) ? "   <- best so far" : "") << endl ;
			}

			if (cand < potential[j])
			{
				potential[j] = cand ;
				pred[j] = front ;
				predK[j] = k ;
				if (k > maxLayerUsed) maxLayerUsed = k ;
			}
		}
	}

	cout << endl ;
	cout << "K FULL TOUR          : " << layersFullTour << "   (ceil(total demand / Q), the naive layer count)" << endl ;
	cout << "LAYERS ALLOCATED (K) : " << layersAllocated << "   (after the path-out horizon bound)" << endl ;
	cout << "MAX LAYER USED       : " << maxLayerUsed << "   (largest k that improved any label)" << endl ;
	cout << "LAYER ITERATIONS     : " << layerIterations << "   (the work actually done; effective K = "
		 << (double) layerIterations / (double) n << " per column)" << endl ;
	cout << "INFEASIBLE SKIPS : " << infeasibleSkips << endl ;
	cout << "UNSAFE POPS : " << unsafePops << endl ;
	cout << "BLOCKED POPS : " << blockedPops << endl ;
	cout << "UNSORTED QUERIES : " << unsortedQueries << endl ;
	cout << "REVIVED STARTS : " << revivedStarts << endl ;
	cout << "REVIVED IMPROVING : " << revivedImproving << endl ;
	if (monotonicityViolations > 0)
		cout << "WARNING : " << monotonicityViolations
			 << " positions break the monotonicity the full-route pointers assume (this solver does not use them)" << endl ;

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

	cout << "MAX LAYER ON PATH    : " << maxLayerOnPath << "   (largest m(sigma) in the solution)" << endl ;

	return 0 ;
}
