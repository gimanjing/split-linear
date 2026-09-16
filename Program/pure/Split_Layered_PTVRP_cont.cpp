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

	// Load windows and deques exactly as in Split_Layered_PTVRP. Containers grow on demand.
	vector <int> firstLoadLE (2, 0) ;
	vector < deque<int> > queues (2) ;
	vector <int> nextAdd (2, 0) ;

	for (int j = 1 ; j <= n ; j++)
	{
		// No frontier, so no horizon bound : the deepest layer is the one the whole prefix needs.
		int kHi = trips(0, j) ;

		if (kHi + 1 > (int) queues.size())
		{
			int grown = kHi + 2 ;
			firstLoadLE.resize(grown + 1, 0) ;
			queues.resize(grown) ;
			nextAdd.resize(grown, 0) ;
		}

		for (int k = 1 ; k <= kHi ; k++)
		{
			// starts needing exactly k trips : [firstLoadLE[k], firstLoadLE[k-1]), with j closing k = 1
			while (firstLoadLE[k] < j && trips(firstLoadLE[k], j) > k)
				firstLoadLE[k] ++ ;

			int lower = firstLoadLE[k] ;
			int upper = (k == 1) ? j : firstLoadLE[k-1] ;
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
				// on ties keep the newer predecessor, matching Split_Layered_PTVRP
				while (!dq.empty() && key(dq.back(), k) >= ki)
					dq.pop_back() ;
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
