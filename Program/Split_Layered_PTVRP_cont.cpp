//--------------------------------------------------------
//LAYERED-K SPLIT FOR THE PT-VRP, NO HORIZON PRUNING
//Extends the LIBRARY OF SPLIT ALGORITHM FOR VEHICLE ROUTING PROBLEMS (Thibaut VIDAL, 2015)
//See Split_Layered_PTVRP_cont.h : the layers and their deques are kept, the two one-way time pointers are not.
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

	monotonicityViolations = 0 ;
	for (int i = 0 ; i + 1 < n ; i++)
		if (At[i+1] > At[i] + 1.e-9) monotonicityViolations ++ ;
	for (int j = 1 ; j < n ; j++)
		if (Bt[j+1] < Bt[j] - 1.e-9) monotonicityViolations ++ ;

	layersFullTour = trips(0, n) ;

	// Load windows and deques exactly as in Split_Layered_PTVRP. Containers grow on demand.
	vector <int> firstLoadLE (2, 0) ;
	vector < deque<int> > queues (2) ;
	vector <int> nextAdd (2, 0) ;

	// Split_Layered_PTVRP's two time pointers, replayed for counting only. They never restrict the DP.
	int shadowFeasible = 0 ;
	vector <int> shadowTimeLE (2, 0) ;

	maxLayerUsed = 0 ;
	layerIterations = 0 ;
	infeasibleSkips = 0 ;
	unsafePops = 0 ;
	revivedStarts = 0 ;
	revivedImproving = 0 ;

	if (myData->trace)
		cout << endl << "=== LAYERED, NO HORIZON PRUNING : one deque per trip count, infeasible starts stepped over, never dropped ===" << endl
			 << "    evicted only when the load window leaves them behind, or a newer start has a key at least as small" << endl ;

	for (int j = 1 ; j <= n ; j++)
	{
		while (shadowFeasible < j
		       && (At[shadowFeasible] + Bt[j]) * (double) trips(shadowFeasible, j) > horizon + 1.e-9)
			shadowFeasible ++ ;
		int shadowKHi = (shadowFeasible < j) ? trips(shadowFeasible, j) : 0 ;

		// No frontier, so no horizon bound : the deepest layer is the one the whole prefix needs.
		int kHi = trips(0, j) ;

		if (myData->trace)
			cout << endl << "  +-- p[" << j << "] : layers 1.." << kHi
				 << "   (pruned solver : feasible starts begin at " << shadowFeasible << ", layers 1.." << shadowKHi << ")" << endl ;

		if (kHi + 1 > (int) queues.size())
		{
			int grown = kHi + 2 ;
			firstLoadLE.resize(grown + 1, 0) ;
			shadowTimeLE.resize(grown + 1, 0) ;
			queues.resize(grown) ;
			nextAdd.resize(grown, 0) ;
		}

		for (int k = 1 ; k <= kHi ; k++)
		{
			layerIterations ++ ;

			// starts needing exactly k trips : [firstLoadLE[k], firstLoadLE[k-1]), with j closing k = 1
			while (firstLoadLE[k] < j && trips(firstLoadLE[k], j) > k)
				firstLoadLE[k] ++ ;

			int lower = firstLoadLE[k] ;
			int upper = (k == 1) ? j : firstLoadLE[k-1] ;
			if (lower >= upper)
				continue ;

			// Where the pruned solver's window for this layer would begin; j when it would not visit the layer.
			int shadowLower = j ;
			if (k <= shadowKHi)
			{
				double timeBound = horizon / (double) k - Bt[j] ;
				while (shadowTimeLE[k] < j && At[shadowTimeLE[k]] > timeBound + 1.e-9)
					shadowTimeLE[k] ++ ;
				shadowLower = shadowTimeLE[k] ;
			}

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
				// on ties keep the newer predecessor, matching Split_Layered_PTVRP
				while (!dq.empty() && key(dq.back(), k) >= ki)
				{
					// the evicted start fits the horizon more easily than its replacement : not provably safe
					if (At[i] > At[dq.back()] + 1.e-9)
						unsafePops ++ ;
					dq.pop_back() ;
				}
				dq.push_back(i) ;
			}

			// First start from the front that fits the horizon at k trips. Nothing is popped on time.
			int front = -1 ;
			for (size_t q = 0 ; q < dq.size() ; q++)
			{
				if ((At[dq[q]] + Bt[j]) * (double) k <= horizon + 1.e-9)
				{
					front = dq[q] ;
					break ;
				}
				infeasibleSkips ++ ;
			}

			if (front < 0)
				continue ;

			double cand = key(front, k) + (double) k * B[j] ;
			bool behind = (front < shadowLower) ;
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
					 << (behind ? "   [behind the frontier]" : "")
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

	// Diagnostics go out before the feasibility check so an infeasible instance still reports them.
	cout << endl ;
	cout << "K FULL TOUR          : " << layersFullTour << "   (ceil(total demand / Q), the naive layer count)" << endl ;
	cout << "LAYERS ALLOCATED (K) : " << layersFullTour << "   (no horizon bound : every layer the prefix needs)" << endl ;
	cout << "MAX LAYER USED       : " << maxLayerUsed << "   (largest k that improved any label)" << endl ;
	cout << "LAYER ITERATIONS     : " << layerIterations << "   (the work actually done; effective K = "
		 << (double) layerIterations / (double) n << " per column)" << endl ;
	cout << "INFEASIBLE SKIPS : " << infeasibleSkips << endl ;
	cout << "UNSAFE POPS : " << unsafePops << endl ;
	// A revived start that improves a label is necessary, not sufficient, for the pruning to change the
	// answer : a later layer may improve the same label further. Compare SOLUTION COST against PTVRP_LAYERED.
	cout << "REVIVED STARTS : " << revivedStarts << endl ;
	cout << "REVIVED IMPROVING : " << revivedImproving << endl ;
	if (monotonicityViolations > 0)
		cout << "WARNING : " << monotonicityViolations
			 << " positions break the monotonicity the pruned solver's pointers assume (rounded distances)" << endl ;

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
