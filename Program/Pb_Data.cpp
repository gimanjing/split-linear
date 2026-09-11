
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

Pb_Data::Pb_Data(string pathToInstance, SolverType solverType, int nbVeh, double penaltyLoad) : pathToInstance(pathToInstance), solverType(solverType), nbVehicles(nbVeh), penaltyLoad(penaltyLoad)
{
	// For now it's hard coded, but should be a variable of the problem or a commandline input
	ifstream fichier ;
	string uselessStr ;

	// to generate different demands for each instance
	srand((unsigned int)time(NULL)); 

	/* parsing the TSPlib instance and deriving node and depot informations */
	fichier.open(pathToInstance.c_str());
	if (fichier.is_open())
	{
		// Token-driven header scan, tolerant of the header variants found across the instance sets :
		// the original Vidal files carry only DIMENSION/CAPACITY, while the Instances 1/2/3 sets also
		// carry MAX_ROUTE (the PT-VRP time horizon T_H) followed by a blank line. Unrecognized keys are
		// skipped rather than assumed away by position.
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
		{
			cout << "ERROR when reading instance, no GIANT_TOUR_SECTION found" << endl ;
			throw string ("ERROR when reading instance, no GIANT_TOUR_SECTION found");
		}
		pos++ ; // step over GIANT_TOUR_SECTION

		if (nbNodes <= 0 || vehCapacity <= 0)
		{
			cout << "ERROR when reading instance, missing DIMENSION or CAPACITY" << endl ;
			throw string ("ERROR when reading instance, missing DIMENSION or CAPACITY");
		}

		// creating the data structure for clients
		// to match the index of the auxiliary graph, the node 0 in the vector of Clients will be a sentinel (which can somehow stand for the depot)
		cli = vector <Client> (nbNodes+1) ;
		cli[0].demand = 0 ;
		cli[0].dreturn = 0 ;
		cli[0].dnext = 0 ;
		cli[0].index = 0 ;
		for (int i = 1 ; i <= nbNodes ; i++)
		{
			if (pos + 2 >= tok.size())
			{
				cout << "ERROR when reading instance, truncated GIANT_TOUR_SECTION" << endl ;
				throw string ("ERROR when reading instance, truncated GIANT_TOUR_SECTION");
			}
			cli[i].index   = atoi(tok[pos++].c_str()) ;
			cli[i].demand  = atof(tok[pos++].c_str()) ;
			cli[i].dreturn = atof(tok[pos++].c_str()) ;
			if (i < nbNodes)
				cli[i].dnext = atof(tok[pos++].c_str()) ;
			else
				cli[i].dnext = -1 ;
		}

		// little debugging test
		if (pos >= tok.size() || tok[pos] != "EOF")
		{
			cout << "ERROR when reading instance, not finding EOF when it should be" << endl ;
			throw string ("ERROR when reading instance, not finding EOF when it should be");
		}

		// if there are too many vehicles, simply reducing it to a more reasonable value
		if (nbVehicles > nbNodes) 
			nbVehicles = nbNodes ;

		// creating the solution structure
		solution = vector <int> (nbNodes) ;
		for (int i=0 ; i < nbNodes ; i++)
			solution[i] = -1 ;

		fichier.close();
	}
	else 
	{
		cout << "ERROR : Impossible to find instance file : " << pathToInstance << endl ;
		throw string("ERROR : Impossible to find instance file : " + pathToInstance);
	}
}

Pb_Data::~Pb_Data(void)
{}