
//--------------------------------------------------------
//SPLIT ALGORITHM FOR THE PERIODIC-TEMPLATE VRP (PT-VRP)
//Extends the LIBRARY OF SPLIT ALGORITHM FOR VEHICLE ROUTING PROBLEMS (Thibaut VIDAL, 2015)
//--------------------------------------------------------

#include "Split_Bellman_PTVRP.h"

// The Split algorithm here is again a shortest path in a graph with n+1 nodes, but the arc (i,j) cost is
// d(i,j) * m(i,j) instead of the plain d(i,j) of the classical CVRP Split, and the arc exists only if
// tau(i,j) * m(i,j) <= horizon, where m(i,j) = ceil((Q[j]-Q[i]) / vehCapacity) is the number of trips the
// template [i+1..j] requires over the horizon.
//
// This non-linear (step) multiplier is what breaks the O(n) Split_Linear dominance argument : Vidal's
// Property 2 requires c(i1,j) - c(i2,j) to be a constant independent of j for any two predecessors
// i1 < i2, which holds for a purely additive distance cost but not for d(i,j)*m(i,j), since m(i1,j) and
// m(i2,j) step up at different values of j (see the docx proof this solver was validated against). So
// this file deliberately stays with the classical Bellman-style O(n*B) DP -- letting it enumerate every
// candidate predecessor rather than trying to discard "dominated" ones, which is not a safe simplification
// for this cost structure.
//
// NOTE ON THE EARLY-STOP ASSUMPTION : the inner loop below stops extending j for a given i as soon as
// tau(i,j)*m(i,j) becomes infeasible. This is only safe because m(i,j) is monotone non-decreasing in j
// (loads are non-negative, so this always holds) AND tau(i,j) is monotone non-decreasing in j, which in
// turn requires the travel times to satisfy the triangle inequality (T_{j,0} <= T_{j,j+1} + T_{j+1,0}) --
// true for ordinary Euclidean/road travel times, which is what this codebase's instances use. If that
// assumption does not hold for your data, replace the "break" below with "continue" (dropping the
// complexity from O(n*B) back to O(n^2), but staying correct for arbitrary travel times).
int Split_Bellman_PTVRP::solve()
{
	double load, dist, time ;

	// Initialization of the structures
	potential = vector <double> (myData->nbNodes+1) ;
	pred = vector <int> (myData->nbNodes+1) ;
	predD = vector <double> (myData->nbNodes+1) ;
	predT = vector <double> (myData->nbNodes+1) ;
	predM = vector <int> (myData->nbNodes+1) ;

	for (int i = 0 ; i < myData->nbNodes+1 ; i++)
	{
		potential[i] = 1.e30 ;
		pred[i] = -1 ;
	}
	potential[0] = 0 ;

	// Split algorithm here
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
			time += myData->cli[j].service ;

			double d_ij = dist + myData->cli[j].dreturn ;
			double tau_ij = time + myData->cli[j].dreturn / myData->speed ;

			int m = (int) ceil(load / myData->vehCapacity - 1.e-9) ;
			if (m < 1) m = 1 ; // a template must be executed at least once even at zero/negligible demand

			if (tau_ij * m > myData->horizon + 1.e-9)
				break ; // infeasible from here on -- see the early-stop note above

			double cost = d_ij * m ;
			if (potential[i] + cost < potential[j])
			{
				potential[j] = potential[i] + cost ;
				pred[j] = i ;
				predD[j] = d_ij ;
				predT[j] = tau_ij ;
				predM[j] = m ;
			}
		}
	}

	// THE CORE OF THE SPLIT ALGORITHM IS FINISHED HERE,
	// NOW JUST SWEEPING THE ROUTE in O(n) TO REPORT THE SOLUTION (IN THE GOOD DIRECTION)

	if (potential[myData->nbNodes] > 1.e29)
	{
		cout << "ERROR : no Split solution has been propagated until the last node "
			 << "(every vendor beyond this point is infeasible within the horizon)" << endl ;
		throw string ("ERROR : no Split solution has been propagated until the last node");
	}

	// Counting the number of templates using the pred structure (linear complexity)
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

	// Filling myData->solution and the per-template reporting arrays, in order (linear complexity)
	cour = myData->nbNodes ;
	for (int i = myData->solutionNbRoutes-1 ; i >= 0 ; i--)
	{
		myData->solutionTemplateDist[i]  = predD[cour] ;
		myData->solutionTemplateTime[i]  = predT[cour] ;
		myData->solutionTemplateTrips[i] = predM[cour] ;
		cour = pred[cour] ;
		myData->solution[i] = cour+1 ;
	}

	myData->solutionCost = potential[myData->nbNodes] ;

	return 0 ;
}
