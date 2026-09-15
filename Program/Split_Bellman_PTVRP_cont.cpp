
//--------------------------------------------------------
//SPLIT ALGORITHM FOR THE PERIODIC-TEMPLATE VRP (PT-VRP), NO EARLY STOP
//Extends the LIBRARY OF SPLIT ALGORITHM FOR VEHICLE ROUTING PROBLEMS (Thibaut VIDAL, 2015)
//--------------------------------------------------------

#include "Split_Bellman_PTVRP_cont.h"

// Same DP as Split_Bellman_PTVRP, arc for arc. The only change is the horizon test : an infeasible
// template is skipped with "continue" rather than ending the scan with "break", so every pair (i,j) is
// evaluated and no triangle-inequality assumption is made. See Split_Bellman_PTVRP_cont.h.
int Split_Bellman_PTVRP_cont::solve()
{
	double load, dist, time ;

	// Initialization of the structures
	potential = vector <double> (myData->nbNodes+1) ;
	pred = vector <int> (myData->nbNodes+1) ;
	predD = vector <double> (myData->nbNodes+1) ;
	predT = vector <double> (myData->nbNodes+1) ;
	predM = vector <int> (myData->nbNodes+1) ;
	revivedArcs = 0 ;
	revivedImproving = 0 ;

	for (int i = 0 ; i < myData->nbNodes+1 ; i++)
	{
		potential[i] = 1.e30 ;
		pred[i] = -1 ;
	}
	potential[0] = 0 ;

	// trace lines are collected per endpoint so the printout reads in DP order
	vector < vector<string> > traceByJ (myData->nbNodes+1) ;

	// Split algorithm here
	for (int i = 0 ; i < myData->nbNodes ; i++)
	{
		load = 0 ; dist = 0 ; time = 0 ;
		bool seenInfeasible = false ; // where Split_Bellman_PTVRP would already have stopped
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
			{
				seenInfeasible = true ;
				continue ; // no early stop : a longer template may fit again if travel times break the triangle inequality
			}

			double cost = d_ij * m ;
			bool improves = (potential[i] + cost < potential[j]) ;
			if (seenInfeasible)
			{
				revivedArcs ++ ;
				if (improves) revivedImproving ++ ;
			}

			if (myData->trace)
			{
				ostringstream os ;
				os << "  |  start=" << i << " : template(" << i+1 << ".." << j << ")"
				   << "  d=" << d_ij << "  q=" << load << "  m=" << m << "  cost=" << d_ij << "*" << m
				   << "=" << cost << "   ->  p[" << i << "]=" << potential[i] << " + " << cost
				   << " = " << potential[i] + cost
				   << (seenInfeasible ? "   [revived]" : "")
				   << (improves ? "   <- best so far" : "") ;
				traceByJ[j].push_back(os.str()) ;
			}
			if (improves)
			{
				potential[j] = potential[i] + cost ;
				pred[j] = i ;
				predD[j] = d_ij ;
				predT[j] = tau_ij ;
				predM[j] = m ;
			}
		}
	}

	if (myData->trace)
	{
		cout << endl << "=== BELLMAN (NO EARLY STOP) : every pair (i,j), feasible or not ===" << endl ;
		for (int j = 1 ; j <= myData->nbNodes ; j++)
		{
			cout << endl << "  +-- p[" << j << "] : cheapest way to serve vendors 1.." << j << endl ;
			for (size_t t = 0 ; t < traceByJ[j].size() ; t++)
				cout << traceByJ[j][t] << endl ;
			cout << "  +-- p[" << j << "] = " << potential[j] << "  (predecessor " << pred[j] << ")" << endl ;
		}
	}

	// A revived arc that improves a label is necessary, not sufficient, for the early stop to change the
	// answer : a later arc may improve the same label further. Compare SOLUTION COST against PTVRP to decide.
	cout << "REVIVED ARCS : " << revivedArcs << endl ;
	cout << "REVIVED IMPROVING : " << revivedImproving << endl ;

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
