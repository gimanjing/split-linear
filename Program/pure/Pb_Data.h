#ifndef DATA_H
#define DATA_H

#include <stdlib.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <algorithm>

using namespace std ;

// PT-VRP solvers only. Recording-free copies of the solvers in Program/ : same DP, same answers, no trace,
// no counters, no per-template reporting. See Program/Pb_Data.h for what each solver is.
enum SolverType {BELLMAN_PTVRP, BELLMAN_PTVRP_CONT, BELLMAN_PTVRP_CONT_FIX, LINEAR_PTVRP,
                 LAYERED_PTVRP, LAYERED_PTVRP_CONT, LAYERED_PTVRP_CONT_FIX, LAYERED_PTVRP_SAFE};

// Service time incurred at a vendor on each visit, in the same time units as the horizon.
// Must stay equal to the value in Program/Pb_Data.h for the two builds to solve the same problem.
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

// type of solver used
SolverType solverType ;

// number of nodes
int nbNodes ;

// vehicle capacity
double vehCapacity ;

// speed used to derive travel time from distance (time = distance / speed)
double speed ;

// time horizon T_H that the duration of a template (over all its executions) must respect
double horizon ;

// vector of clients, i.e., locations to be visited in the TSP
vector < Client > cli ;

/* SOLUTION STRUCTURE */

// list of indices of clients where a template starts, -1 if not filled
vector < int > solution ;

// cost of the solution
double solutionCost ;

// number of templates in the solution
int solutionNbRoutes ;

// Constructor
Pb_Data(string pathToInstance, SolverType solverType);

~Pb_Data(void);
};

#endif
