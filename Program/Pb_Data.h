#ifndef DATA_H
#define DATA_H

#include <stdlib.h>
#include <stdio.h> 
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream> 
#include <time.h>
#include <math.h>
#include <algorithm>

using namespace std ;

// Solver types:
// BELLMAN --> Classic O(nB) approach -- B is the average size of a route
// BELLMAN_SOFT --> Classic O(nB') approach with possible penalized capacity excess and unlimited fleet -- B' is the average size of a route (need a limit on capacity excess otherwise B' = n)
// BELLMAN_BOUNDED --> Classic O(nBK) approach, with limited fleet
// LINEAR --> Efficient O(n) approach for Split
// LINEAR_SOFT --> Efficient O(n) approach, with possible penalized capacity excess
// LINEAR_BOUNDED --> Efficient O(n) approach, with bounded fleet
// BELLMAN_PTVRP --> Periodic-Template VRP: a template (route) is executed ceil(q(sigma)/Q) times over a
//                   horizon T_H, so both cost and duration are multiplied by that trip count instead of
//                   being bounded by a single-trip capacity check. This multiplier is a non-linear (step)
//                   function of the segment's cumulative demand, which breaks the constant-offset dominance
//                   property the O(n) linear Split relies on (see Split_Bellman_PTVRP.cpp) -- only a
//                   Bellman-style O(nB) DP is used for this solver type.
// LINEAR_PTVRP --> deliberately UNSOUND control: Vidal's O(n) deque applied to the PT-VRP cost anyway.
//                  The dominance test is left exactly as it is in Split_Linear, so it compares
//                  predecessors by their fixed cost while ignoring the trip multiplier that actually
//                  scales that cost. It exists to measure what the shortcut costs, not to be used.
enum SolverType {BELLMAN, BELLMAN_SOFT, BELLMAN_BOUNDED, LINEAR, LINEAR_SOFT, LINEAR_BOUNDED, BELLMAN_PTVRP, LINEAR_PTVRP};

// Service time incurred at a vendor on each visit, in the same time units as the horizon.
// PT-VRP defines tau(sigma) as the travel time of one trip plus the service time of every vendor
// visited, but none of the instance sets carry per-vendor service data, and the value is expected to
// be the same across instances, so it is fixed here for every vendor rather than read from the file.
// Change this single line to model a non-zero service time.
// Scale note: on the Instances 1/2/3 sets (MAX_ROUTE = 86400, distances doubling as travel times)
// any value below roughly 3600 leaves the optimum unchanged, and values above roughly 40000 make the
// instances infeasible -- so the horizon only reacts to service times of that order.
const double PTVRP_SERVICE_TIME = 0.0 ;

struct Client
{
	int index ;
	double demand ;
	double dreturn ;
	double dnext ;
};

class Pb_Data
{
public:

/* PROBLEM DATA */

// address of the problem instance
string pathToInstance ;

/* METHOD PARAMETERS */
// type of solver used
SolverType solverType ;

// number of nodes
int nbNodes ;

// vehicle capacity
double vehCapacity ;

// max number of vehicle (for problems with limited fleet)
int nbVehicles ;

// penalty coefficient for one unit of load excess (for problems with soft capacity constraint)
double penaltyLoad ;

// PT-VRP only : speed used to derive travel time from distance (time = distance / speed)
double speed ;

// PT-VRP only : time horizon T_H that the duration of a template (over all its executions) must respect
double horizon ;

// vector of clients, i.e., locations to be visited in the TSP
vector < Client > cli ;

// Starting time of the optimization
clock_t time_StartComput ;

// End time of the optimization
clock_t time_EndComput ;

/* SOLUTION STRUCTURE */

// list of indices of clients where a route starts, -1 if not filled
vector < int > solution ;

// cost of the solution
double solutionCost ;

// number of routes in the solution
int solutionNbRoutes ;

/* PT-VRP ONLY : per-template reporting, filled by Split_Bellman_PTVRP, indexed like solution[] */

// d(sigma) : one-trip distance of each template
vector < double > solutionTemplateDist ;

// tau(sigma) : one-trip duration (travel + service) of each template
vector < double > solutionTemplateTime ;

// m(sigma) = ceil(q(sigma)/Q) : number of executions of each template
vector < int > solutionTemplateTrips ;

/* METHODS TO TEST AND EXPORT THE FINAL SOLUTION */

// Method to display a solution
void printSolution()
{ 
	cout << endl ;
	cout << "------------------------------------" << endl ;
	cout << "SOLUTION COST : " << std::setprecision(12) << solutionCost << endl ;
	cout << "NB ROUTES : " << solutionNbRoutes << endl ;
	cout << "SOLUTION : [ " ;
	for (int i=0 ; i < solutionNbRoutes ; i++)
		cout << solution[i] << " " ; 
	cout << "]" << endl ;
	cout << "------------------------------------" << endl ;
	cout << endl ;
}

// Method to test a solution, verify the solution cost 
void checkSolution()
{ 
	double costTotal = 0 ;
	int begin, end ;
	double load, distance ;

	if (solution[0] != 1)
		cout << " ERROR : First route should start with customer 0" << endl ;

	for (int i=0 ; i < solutionNbRoutes ; i++)
	{
		begin = solution[i] ;
		if (i < solutionNbRoutes-1)
			end = solution[i+1]-1 ;
		else
			end = nbNodes ;

		load = 0 ;
		for (int j = begin ; j <= end ; j++)
			load += cli[j].demand ;

		distance = cli[begin].dreturn + cli[end].dreturn ;
		for (int j = begin ; j < end ; j++)
			distance += cli[j].dnext ;

		if (!(solverType == BELLMAN_SOFT || solverType == LINEAR_SOFT) && load > vehCapacity + 0.0001)
		{
			cout << "ERROR : One route is exceeding the capacity limit" << endl ;
			throw string("ERROR : One route is exceeding the capacity limit");
		}

		costTotal += distance + penaltyLoad * max<double>(load - vehCapacity, 0);
	}

	//cout << "Cost reported by the Split algorithm : " << solutionCost << endl ;
	//cout << "Cost evaluated by the solution checker : " << costTotal << endl ;

	if (costTotal > solutionCost + 0.0001 || costTotal < solutionCost - 0.0001)
		cout << "ERROR : Solution checker does not find the same solution cost" << endl ;
}

// Method to display a PT-VRP solution (templates, trip counts, per-template distance/duration)
void printSolutionPTVRP()
{
	cout << endl ;
	cout << "------------------------------------" << endl ;
	cout << "SOLUTION COST : " << std::setprecision(12) << solutionCost << endl ;
	cout << "NB TEMPLATES : " << solutionNbRoutes << endl ;
	for (int i = 0 ; i < solutionNbRoutes ; i++)
	{
		int begin = solution[i] ;
		int end = (i < solutionNbRoutes-1) ? solution[i+1]-1 : nbNodes ;
		cout << "TEMPLATE " << i << " : vendors [" << begin << ".." << end << "]"
			 << "  d(sigma)=" << solutionTemplateDist[i]
			 << "  tau(sigma)=" << solutionTemplateTime[i]
			 << "  m(sigma)=" << solutionTemplateTrips[i]
			 << "  cost=" << solutionTemplateDist[i] * solutionTemplateTrips[i] << endl ;
	}
	cout << "------------------------------------" << endl ;
	cout << endl ;
}

// Method to test a PT-VRP solution : recomputes d(sigma), tau(sigma), m(sigma) per template from
// scratch and checks the total cost and every template's duration feasibility against horizon.
// strict=false reports violations instead of throwing, for the unsound LINEAR_PTVRP control whose
// whole purpose is to be measured when it returns something infeasible. Returns the violation count.
int checkSolutionPTVRP(bool strict = true)
{
	double costTotal = 0 ;
	int violations = 0 ;
	for (int i = 0 ; i < solutionNbRoutes ; i++)
	{
		int begin = solution[i] ;
		int end = (i < solutionNbRoutes-1) ? solution[i+1]-1 : nbNodes ;

		double load = 0, dist = 0, time = 0 ;
		for (int j = begin ; j <= end ; j++)
			load += cli[j].demand ;
		dist = cli[begin].dreturn + cli[end].dreturn ;
		time = cli[begin].dreturn / speed + cli[end].dreturn / speed ;
		for (int j = begin ; j < end ; j++)
		{
			dist += cli[j].dnext ;
			time += cli[j].dnext / speed ;
		}
		time += PTVRP_SERVICE_TIME * (end - begin + 1) ;

		int m = (int) ceil(load / vehCapacity - 1.e-9) ;
		if (m < 1) m = 1 ;

		if (time * m > horizon + 0.0001)
		{
			violations ++ ;
			cout << "ERROR : template " << i << " violates the horizon constraint ("
				 << time * m << " > " << horizon << ")" << endl ;
			if (strict)
				throw string("ERROR : template violates the horizon constraint");
		}

		costTotal += dist * m ;
	}

	if (costTotal > solutionCost + 0.0001 || costTotal < solutionCost - 0.0001)
	{
		violations ++ ;
		cout << "ERROR : Solution checker does not find the same solution cost" << endl ;
	}

	return violations ;
}

// Constructor
Pb_Data(string pathToInstance, SolverType solverType, int nbVeh, double penaltyLoad);

~Pb_Data(void);
};

#endif

