#include "walknetcontroller.h"

lpzrobots::walknetcontroller::walknetcontroller(void) {
	// Make 6 walknetSeparateLeg objects and put them in separateLegs array
	// Give them a legnumber of some kind
}

lpzrobots::walknetcontroller::~walknetcontroller(void) {
}

void lpzrobots::walknetcontroller::stepWalknet(const sensor* sensor,
		double* angleVector) {
	// Iterate through all the 6 walknetSeparateleg object (the step function)
	// Collect all the "via" angles in the angleVector array
}
