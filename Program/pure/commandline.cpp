#include "commandline.h"

int commandline::set_solver_type(string to_parse)
{
	if (to_parse == "PTVRP")
		solverType = BELLMAN_PTVRP ;
	else if (to_parse == "PTVRP_CONT")
		solverType = BELLMAN_PTVRP_CONT ;
	else if (to_parse == "PTVRP_CONT_FIX")
		solverType = BELLMAN_PTVRP_CONT_FIX ;
	else if (to_parse == "PTVRP_LINEAR")
		solverType = LINEAR_PTVRP ;
	else if (to_parse == "PTVRP_LAYERED")
		solverType = LAYERED_PTVRP ;
	else if (to_parse == "PTVRP_LAYERED_CONT")
		solverType = LAYERED_PTVRP_CONT ;
	else if (to_parse == "PTVRP_LAYERED_CONT_FIX")
		solverType = LAYERED_PTVRP_CONT_FIX ;
	else if (to_parse == "PTVRP_LAYERED_SAFE")
		solverType = LAYERED_PTVRP_SAFE ;
	else
		return -1; // problem

	return 0 ; // OK
}

commandline::commandline(int argc, char* argv[])
{
	command_ok = false ;

	if (argc != 4 || string(argv[2]) != "-solver")
		throw string ("ERROR : invalid command line. USAGE : ./split path_to_instance -solver solver_type") ;

	instance_name = string(argv[1]) ;

	if (set_solver_type(string(argv[3])) != 0)
		throw string ("ERROR : Unrecognized solver type : " + string(argv[3])) ;

	command_ok = true ;
}

commandline::~commandline(){}

string commandline::get_path_to_instance()
{
    return instance_name;
}

SolverType commandline::get_solver_type()
{
	return solverType ;
}

bool commandline::is_valid()
{
    return command_ok;
}
