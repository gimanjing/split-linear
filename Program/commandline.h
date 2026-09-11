#ifndef COMMANDLINE_H
#define COMMANDLINE_H

#include <iostream>
#include <cstdlib>
#include <string>
#include "Pb_Data.h"
using namespace std;

class commandline
{
    private:

        // is the commandline valid ? 
        bool command_ok;

        // path of the TSP instance (simple TSPlib instance format)
        string instance_name;

		// type de solver
		SolverType solverType ;

		// nbVehicles, only necessary for problems with bounded fleet
		int nbVeh ;

		// unit penalty per amount off capacity excess, only necessary for problems with soft capacity constraints
		double penaltyLoad ;

		// uniform service time applied to every vendor (PT-VRP). Negative means "not specified on the
		// commandline", in which case the instance file's own service column is used, or 0 if it has none.
		double serviceTime ;

		// set the name of the instance
        void set_instance_name(string to_parse);

		// set the solver type
		int set_solver_type(string to_parse);

		// display the name of the problem
        void display_problem_name(string to_parse);

    public:

        // constructor
        commandline(int argc, char* argv[]);

        // destructor
        ~commandline();

		// gets the path to the instance
        string get_path_to_instance();

		// gets the solver type
		SolverType get_solver_type();

		// get the number of vehicles
		int get_nbVeh();

		// get the load penalty
		double get_penaltyLoad();

		// get the uniform service time (negative if not specified)
		double get_serviceTime();

		// is the commandline valid ?
        bool is_valid();
};

#endif

