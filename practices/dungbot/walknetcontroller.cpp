#include "walknetcontroller.h"

lpzrobots::walknetcontroller::walknetcontroller(void) {
	walknetSeparateLeg leg1();
	walknetSeparateLeg leg2();
	walknetSeparateLeg leg3();
	walknetSeparateLeg leg4();
	walknetSeparateLeg leg5();
	walknetSeparateLeg leg6();

	separateLegs[0] = leg1;
	separateLegs[1] = leg2;
	separateLegs[2] = leg3;
	separateLegs[3] = leg4;
	separateLegs[4] = leg5;
	separateLegs[5] = leg6;
}

lpzrobots::walknetcontroller::~walknetcontroller(void) {
	delete separateLegs[0];
	delete separateLegs[1];
	delete separateLegs[2];
	delete separateLegs[3];
	delete separateLegs[4];
	delete separateLegs[5];
}

void lpzrobots::walknetcontroller::stepWalknet(const sensor* sensor,
		double* angleVector) {

	for(int i = 0; i < 7; i++){
		// should we extract only the neighboring leg sensors here?
		angleVector[i] = separateLegs[i].stepWalknetSeprateLeg(sensor); //Should the angleVector in fact be a 6 by 3 matrix?
																		// I have made it like that here! If so change it in the other functions
	}
}
