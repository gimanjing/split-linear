#ifndef COMMANDLINE_H
#define COMMANDLINE_H

#include <iostream>
#include <cstdlib>
#include <string>
#include "Pb_Data.h"
using namespace std;

// Usage : ./split path_to_instance -solver solver_type
class commandline
{
    private:

        // is the commandline valid ?
        bool command_ok;

        // path of the instance
        string instance_name;

		// type of solver
		SolverType solverType ;

		// set the solver type, returns -1 when the name is not a PT-VRP solver
		int set_solver_type(string to_parse);

    public:

        // constructor
        commandline(int argc, char* argv[]);

        // destructor
        ~commandline();

		// gets the path to the instance
        string get_path_to_instance();

		// gets the solver type
		SolverType get_solver_type();

		// is the commandline valid ?
        bool is_valid();
};

#endif
