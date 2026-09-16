#ifndef SPLIT_H
#define SPLIT_H

#include "Pb_Data.h"
#include "Trivial_Deque.h"
#include <iostream>
using namespace std;

class Split {

protected:

	// Parameters of the problem
	Pb_Data * myData ;

	// Shared by every solver once its DP is finished : throws when no partition reaches the last vendor,
	// otherwise walks pred back from n to fill myData->solution in order, and records the cost.
	void extractSolution(const vector <double> & potential, const vector <int> & pred)
	{
		const int n = myData->nbNodes ;
		if (potential[n] > 1.e29)
			throw string ("ERROR : no Split solution has been propagated until the last node");

		myData->solutionNbRoutes = 0 ;
		for (int cour = n ; cour != 0 ; cour = pred[cour])
			myData->solutionNbRoutes ++ ;

		int cour = n ;
		for (int i = myData->solutionNbRoutes-1 ; i >= 0 ; i--)
		{
			cour = pred[cour] ;
			myData->solution[i] = cour+1 ;
		}

		myData->solutionCost = potential[n] ;
	}

public:

	// Constructor common to all Split solvers
	Split (Pb_Data * myData) : myData(myData) {}

	// Solution methods, implemented in each solver independently
	virtual int solve (void) = 0 ;

	// Virtual destructor (needs to stay virtual otherwise inherited destructors will not be called)
	virtual ~Split() {} ;

};

#endif
