//--------------------------------------------------------
//LAYERED-K SPLIT FOR THE PT-VRP, SOUND HORIZON BOUND
//Extends the LIBRARY OF SPLIT ALGORITHM FOR VEHICLE ROUTING PROBLEMS (Thibaut VIDAL, 2015)
//See Split_Layered_PTVRP_cont_fix.h : prune on the path out, test feasibility on the full route.
//--------------------------------------------------------

#include "Split_Layered_PTVRP_cont_fix.h"

int Split_Layered_PTVRP_cont_fix::solve()
{
	const int n = myData->nbNodes ;
	const double speed = myData->speed ;
	const double horizon = myData->horizon ;

	potential = vector <double> (n+1, 1.e30) ;
	pred = vector <int> (n+1, -1) ;
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

	vector <int> firstLoadLE (2, 0) ;
	vector <int> firstTimeLE (2, 0) ;
	vector < deque<int> > queues (2) ;
	vector <int> nextAdd (2, 0) ;

	// This solver's frontier : path out. A start dropped here can never fit again, whatever the rounding.
	int frontierPathOut = 0 ;

	for (int j = 1 ; j <= n ; j++)
	{
		// Sound : both factors are non-decreasing in j, so once this fires for a start it fires for ever.
		while (frontierPathOut < j
		       && (At[frontierPathOut] + Bout_t[j]) * (double) trips(frontierPathOut, j) > horizon + 1.e-9)
			frontierPathOut ++ ;

		if (frontierPathOut >= j)
			continue ; // not even the singleton {j} can fit, on a bound that ignores the leg home

		int kHi = trips(frontierPathOut, j) ;

		if (kHi + 1 > (int) queues.size())
		{
			int grown = kHi + 2 ;
			firstLoadLE.resize(grown + 1, 0) ;
			firstTimeLE.resize(grown + 1, 0) ;
			queues.resize(grown) ;
			nextAdd.resize(grown, 0) ;
		}

		for (int k = 1 ; k <= kHi ; k++)
		{
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
				while (!dq.empty() && key(dq.back(), k) >= ki)
					dq.pop_back() ;
				dq.push_back(i) ;
			}

			// Feasibility itself is tested exactly, on the true route time. Nothing is popped on time.
			int front = -1 ;
			for (size_t q = 0 ; q < dq.size() ; q++)
			{
				if ((At[dq[q]] + Bt[j]) * (double) k <= horizon + 1.e-9)
				{
					front = dq[q] ;
					break ;
				}
			}

			if (front < 0)
				continue ;

			double cand = key(front, k) + (double) k * B[j] ;
			if (cand < potential[j])
			{
				potential[j] = cand ;
				pred[j] = front ;
			}
		}
	}

	extractSolution(potential, pred) ;
	return 0 ;
}
