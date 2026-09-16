#ifndef SPLIT_BELLMAN_PTVRP_CONT_FIX_H
#define SPLIT_BELLMAN_PTVRP_CONT_FIX_H

#include "Split.h"

// Bellman-style Split for the PT-VRP with a SOUND early stop : O(n*B) with B bounded by the horizon.
//
// Split_Bellman_PTVRP stops the scan at the first j with tau(i,j)*m > T_H. That assumes tau(i,j) is
// non-decreasing in j, which needs the triangle inequality on travel times. Extending a template removes
// the leg home from v_j and adds two legs, and on rounded instances the removal can outweigh the addition,
// so a longer template can be shorter and fit again after a shorter one did not (revival.md).
//
// Split_Bellman_PTVRP_cont answered that by never stopping : every pair (i,j) is evaluated, O(n^2).
//
// This solver stops, and is still exact, by testing the PATH OUT rather than the full route :
//
//     path out = depot -> v_{i+1} -> ... -> v_j          the route WITHOUT the leg home
//
// which is the loop's running "time" before the return leg is added. Two facts make it a valid stopping
// rule with no assumption about the instance at all :
//   1. the path out only grows -- extending a template appends an arc and removes nothing, because the leg
//      home is exactly what has been excluded ;
//   2. tau(i,j) = path out + leg home >= path out, and m(i,j) only grows.
// So  pathout(i,j)*m(i,j) > T_H  proves every longer template from i is infeasible, permanently. Revival
// lives entirely in the leg home, which this bound ignores, so it cannot escape it.
//
// The feasibility test that decides whether an arc may be USED is unchanged and exact : tau(i,j)*m <= T_H.
// An arc that fails it is skipped, not stopped on. Pruning is conservative, feasibility is exact.
class Split_Bellman_PTVRP_cont_fix: public Split
{

private:

	vector < double > potential ;
	vector < int > pred ;

	vector < double > predD ;
	vector < double > predT ;
	vector < int > predM ;

public:

	// Arcs that are feasible although some (i,j') with j' < j was not : what the unsound early stop misses.
	long revivedArcs ;
	long revivedImproving ;

	// Arcs actually evaluated, and where Split_Bellman_PTVRP's unsound stop would have ended each scan.
	// The gap is the price of the sound bound.
	long arcsScanned ;
	long arcsUnsoundStop ;

	Split_Bellman_PTVRP_cont_fix(Pb_Data * myData) : Split(myData),
		revivedArcs(0), revivedImproving(0), arcsScanned(0), arcsUnsoundStop(0) {}

	int solve();
};

#endif
