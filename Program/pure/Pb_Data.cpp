
//--------------------------------------------------------
//LIBRARY OF SPLIT ALGORITHM FOR VEHICLE ROUTING PROBLEMS
//Author : Thibaut VIDAL
//Date   : August 15th, 2015.
//E-mail : vidalt@inf.puc-rio.br
//
//This code is distributed for research purposes.
//All rights reserved.
//--------------------------------------------------------

#include "Pb_Data.h"

Pb_Data::Pb_Data(string pathToInstance, SolverType solverType) : pathToInstance(pathToInstance), solverType(solverType),
	solutionCost(0), solutionNbRoutes(0)
{
	ifstream fichier ;
	string uselessStr ;

	/* parsing the TSPlib instance and deriving node and depot informations */
	fichier.open(pathToInstance.c_str());
	if (fichier.is_open())
	{
		// Token-driven header scan, tolerant of the header variants found across the instance sets.
		// Unrecognized keys are skipped rather than assumed away by position.
		vector <string> tok ;
		while (fichier >> uselessStr)
			tok.push_back(uselessStr) ;

		nbNodes = -1 ;
		vehCapacity = -1 ;
		speed = 1.0 ;      // absent SPEED : distances double as travel times
		horizon = 1.e30 ;  // absent MAX_ROUTE/HORIZON : no duration limit

		size_t pos = 0 ;
		while (pos < tok.size() && tok[pos] != "GIANT_TOUR_SECTION")
		{
			string key = tok[pos] ;
			if (key == "DIMENSION" || key == "CAPACITY" || key == "MAX_ROUTE" ||
			    key == "HORIZON"   || key == "SPEED")
			{
				size_t v = pos + 1 ;
				if (v < tok.size() && tok[v] == ":") v++ ; // the ':' separator is optional
				if (v < tok.size())
				{
					double val = atof(tok[v].c_str()) ;
					if      (key == "DIMENSION") nbNodes = (int) val ;
					else if (key == "CAPACITY")  vehCapacity = val ;
					else if (key == "SPEED")     speed = val ;
					else                         horizon = val ; // MAX_ROUTE or HORIZON
					pos = v + 1 ;
					continue ;
				}
			}
			pos++ ;
		}

		if (pos >= tok.size())
			throw string ("ERROR when reading instance, no GIANT_TOUR_SECTION found");
		pos++ ; // step over GIANT_TOUR_SECTION

		if (nbNodes <= 0 || vehCapacity <= 0)
			throw string ("ERROR when reading instance, missing DIMENSION or CAPACITY");

		// node 0 in the vector of Clients is a sentinel standing for the depot, to match the auxiliary graph
		cli = vector <Client> (nbNodes+1) ;
		cli[0].demand = 0 ;
		cli[0].dreturn = 0 ;
		cli[0].dnext = 0 ;
		cli[0].index = 0 ;
		for (int i = 1 ; i <= nbNodes ; i++)
		{
			if (pos + 2 >= tok.size())
				throw string ("ERROR when reading instance, truncated GIANT_TOUR_SECTION");
			cli[i].index   = atoi(tok[pos++].c_str()) ;
			cli[i].demand  = atof(tok[pos++].c_str()) ;
			cli[i].dreturn = atof(tok[pos++].c_str()) ;
			if (i < nbNodes)
				cli[i].dnext = atof(tok[pos++].c_str()) ;
			else
				cli[i].dnext = -1 ;
		}

		if (pos >= tok.size() || tok[pos] != "EOF")
			throw string ("ERROR when reading instance, not finding EOF when it should be");

		solution = vector <int> (nbNodes, -1) ;

		fichier.close();
	}
	else
		throw string("ERROR : Impossible to find instance file : " + pathToInstance);
}

Pb_Data::~Pb_Data(void)
{}
