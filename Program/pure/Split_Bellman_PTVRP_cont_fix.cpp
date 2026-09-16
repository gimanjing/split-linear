//--------------------------------------------------------
//SPLIT ALGORITHM FOR THE PT-VRP, SOUND EARLY STOP
//Extends the LIBRARY OF SPLIT ALGORITHM FOR VEHICLE ROUTING PROBLEMS (Thibaut VIDAL, 2015)
//See Split_Bellman_PTVRP_cont_fix.h : stop on the path out, test feasibility on the full route.
//--------------------------------------------------------

#include "Split_Bellman_PTVRP_cont_fix.h"

int Split_Bellman_PTVRP_cont_fix::solve()
{
	double load, dist, time ;

	potential = vector <double> (myData->nbNodes+1, 1.e30) ;
	pred = vector <int> (myData->nbNodes+1, -1) ;
	potential[0] = 0 ;

	for (int i = 0 ; i < myData->nbNodes ; i++)
	{
		load = 0 ; dist = 0 ; time = 0 ;
		for (int j = i+1 ; j <= myData->nbNodes ; j++)
		{
			load += myData->cli[j].demand ;
			if (j == i+1)
			{
				dist += myData->cli[j].dreturn ;
				time += myData->cli[j].dreturn / myData->speed ;
			}
			else
			{
				dist += myData->cli[j-1].dnext ;
				time += myData->cli[j-1].dnext / myData->speed ;
			}
			time += PTVRP_SERVICE_TIME ;

			// "time" here is the PATH OUT : depot -> v_{i+1} -> ... -> v_j, with no leg home. It only
			// ever grows, because extending the template appends an arc and the leg home is not in it.
			double pathOut = time ;

			double d_ij = dist + myData->cli[j].dreturn ;
			double tau_ij = time + myData->cli[j].dreturn / myData->speed ;

			int m = (int) ceil(load / myData->vehCapacity - 1.e-9) ;
			if (m < 1) m = 1 ;

			// SOUND STOP. pathOut and m are both non-decreasing in j and tau >= pathOut, so once this
			// fires no longer template from i can fit, whatever the rounding does to the leg home.
			if (pathOut * m > myData->horizon + 1.e-9)
				break ;

			// Exact feasibility test on the full route. Failing it skips the arc, it does not end the scan.
			if (tau_ij * m > myData->horizon + 1.e-9)
				continue ;

			double cost = d_ij * m ;
			if (potential[i] + cost < potential[j])
			{
				potential[j] = potential[i] + cost ;
				pred[j] = i ;
			}
		}
	}

	extractSolution(potential, pred) ;
	return 0 ;
}
