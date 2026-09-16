
//--------------------------------------------------------
//LIBRARY OF SPLIT ALGORITHM FOR VEHICLE ROUTING PROBLEMS
//Author : Thibaut VIDAL
//Date   : August 15th, 2015.
//E-mail : vidalt@inf.puc-rio.br
//
//This code is distributed for research purposes.
//All rights reserved.
//--------------------------------------------------------

// PT-VRP solvers without recording. The only output is the result and the runtime of solve() :
//   SOLUTION COST : <cost>        or   NO SOLUTION
//   NB TEMPLATES  : <count>
//   SOLVE TIME    : <seconds>     wall-clock around solve() only : excludes process start and parsing

#include "commandline.h"
#include "Split_Bellman_PTVRP.h"
#include "Split_Bellman_PTVRP_cont.h"
#include "Split_Bellman_PTVRP_cont_fix.h"
#include "Split_Linear_PTVRP.h"
#include "Split_Layered_PTVRP.h"
#include "Split_Layered_PTVRP_cont.h"
#include "Split_Layered_PTVRP_cont_fix.h"
#include "Split_Layered_PTVRP_safe.h"
#include <chrono>
#include <iostream>

int main (int argc, char *argv[])
{
	try
	{
		commandline c(argc, argv);
		Pb_Data myData(c.get_path_to_instance(), c.get_solver_type()) ;

		Split * mySolver ;
		switch (myData.solverType)
		{
			case BELLMAN_PTVRP :          mySolver = new Split_Bellman_PTVRP(&myData) ; break ;
			case BELLMAN_PTVRP_CONT :     mySolver = new Split_Bellman_PTVRP_cont(&myData) ; break ;
			case BELLMAN_PTVRP_CONT_FIX : mySolver = new Split_Bellman_PTVRP_cont_fix(&myData) ; break ;
			case LINEAR_PTVRP :           mySolver = new Split_Linear_PTVRP(&myData) ; break ;
			case LAYERED_PTVRP :          mySolver = new Split_Layered_PTVRP(&myData) ; break ;
			case LAYERED_PTVRP_CONT :     mySolver = new Split_Layered_PTVRP_cont(&myData) ; break ;
			case LAYERED_PTVRP_CONT_FIX : mySolver = new Split_Layered_PTVRP_cont_fix(&myData) ; break ;
			case LAYERED_PTVRP_SAFE :     mySolver = new Split_Layered_PTVRP_safe(&myData) ; break ;
			default : throw string ("ERROR : no solver with this name") ;
		}

		// The solver throws when no partition reaches the last vendor : an infeasible instance, not an error.
		bool solved = true ;
		chrono::steady_clock::time_point start = chrono::steady_clock::now() ;
		try
		{
			mySolver->solve() ;
		}
		catch (const string &)
		{
			solved = false ;
		}
		chrono::steady_clock::time_point end = chrono::steady_clock::now() ;
		double seconds = chrono::duration <double> (end - start).count() ;

		if (solved)
		{
			cout << "SOLUTION COST : " << setprecision(12) << myData.solutionCost << endl ;
			cout << "NB TEMPLATES : " << myData.solutionNbRoutes << endl ;
		}
		else
			cout << "NO SOLUTION" << endl ;
		cout << "SOLVE TIME : " << fixed << setprecision(9) << seconds << endl ;

		delete mySolver ;
		return 0 ;
	}
	catch (const string & e)
	{
		cerr << e << endl ;
		return 1 ;
	}
}
