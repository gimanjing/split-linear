
//--------------------------------------------------------
//SPLIT ALGORITHM FOR THE PERIODIC-TEMPLATE VRP (PT-VRP)
//Extends the LIBRARY OF SPLIT ALGORITHM FOR VEHICLE ROUTING PROBLEMS (Thibaut VIDAL, 2015)
//--------------------------------------------------------

#include "Split_Bellman_PTVRP.h"

int Split_Bellman_PTVRP::solve()
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

			double d_ij = dist + myData->cli[j].dreturn ;
			double tau_ij = time + myData->cli[j].dreturn / myData->speed ;

			int m = (int) ceil(load / myData->vehCapacity - 1.e-9) ;
			if (m < 1) m = 1 ; // a template must be executed at least once even at zero/negligible demand

			if (tau_ij * m > myData->horizon + 1.e-9)
				break ; // early stop : assumes the triangle inequality on travel times

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
