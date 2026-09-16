//--------------------------------------------------------
//SPLIT ALGORITHM FOR THE PT-VRP, SOUND EARLY STOP
//Extends the LIBRARY OF SPLIT ALGORITHM FOR VEHICLE ROUTING PROBLEMS (Thibaut VIDAL, 2015)
//See Split_Bellman_PTVRP_cont_fix.h : stop on the path out, test feasibility on the full route.
//--------------------------------------------------------

#include "Split_Bellman_PTVRP_cont_fix.h"

int Split_Bellman_PTVRP_cont_fix::solve()
{
	double load, dist, time ;

	potential = vector <double> (myData->nbNodes+1) ;
	pred = vector <int> (myData->nbNodes+1) ;
	predD = vector <double> (myData->nbNodes+1) ;
	predT = vector <double> (myData->nbNodes+1) ;
	predM = vector <int> (myData->nbNodes+1) ;
	revivedArcs = 0 ;
	revivedImproving = 0 ;
	arcsScanned = 0 ;
	arcsUnsoundStop = 0 ;

	for (int i = 0 ; i < myData->nbNodes+1 ; i++)
	{
		potential[i] = 1.e30 ;
		pred[i] = -1 ;
	}
	potential[0] = 0 ;

	vector < vector<string> > traceByJ (myData->nbNodes+1) ;

	for (int i = 0 ; i < myData->nbNodes ; i++)
	{
		load = 0 ; dist = 0 ; time = 0 ;
		bool seenInfeasible = false ;  // where Split_Bellman_PTVRP would already have stopped
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

			arcsScanned ++ ;
			if (!seenInfeasible) arcsUnsoundStop ++ ;

			// SOUND STOP. pathOut and m are both non-decreasing in j and tau >= pathOut, so once this
			// fires no longer template from i can fit, whatever the rounding does to the leg home.
			if (pathOut * m > myData->horizon + 1.e-9)
				break ;

			// Exact feasibility test on the full route. Failing it skips the arc, it does not end the scan.
			if (tau_ij * m > myData->horizon + 1.e-9)
			{
				seenInfeasible = true ;
				continue ;
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
		cout << endl << "=== BELLMAN (SOUND STOP) : stop when the path out alone exceeds the horizon ===" << endl ;
		for (int j = 1 ; j <= myData->nbNodes ; j++)
		{
			cout << endl << "  +-- p[" << j << "] : cheapest way to serve vendors 1.." << j << endl ;
			for (size_t t = 0 ; t < traceByJ[j].size() ; t++)
				cout << traceByJ[j][t] << endl ;
			cout << "  +-- p[" << j << "] = " << potential[j] << "  (predecessor " << pred[j] << ")" << endl ;
		}
	}

	cout << "REVIVED ARCS : " << revivedArcs << endl ;
	cout << "REVIVED IMPROVING : " << revivedImproving << endl ;
	cout << "ARCS SCANNED : " << arcsScanned << "   (effective B = "
		 << (double) arcsScanned / (double) myData->nbNodes << " per start)" << endl ;
	cout << "ARCS UNSOUND STOP : " << arcsUnsoundStop << "   (where Split_Bellman_PTVRP would have ended each scan)" << endl ;

	if (potential[myData->nbNodes] > 1.e29)
	{
		cout << "ERROR : no Split solution has been propagated until the last node "
			 << "(every vendor beyond this point is infeasible within the horizon)" << endl ;
		throw string ("ERROR : no Split solution has been propagated until the last node");
	}

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
