#include "walknetSeparateLeg.h"

walknetSeparateLeg::walknetSeparateLeg() {
}

walknetSeparateLeg::walknetSeparateLeg(int newlegNum) {
	legNum = newlegNum;
}

double* walknetSeparateLeg::stepWalknetSeprateLeg(const sensor* sensor) {

	//selectorNet() -> stanceNet() || swingNet() -> tragetoryGenerator();
	double *viaAngle = new double[3];
	viaAngle[0] = 0.65;
	viaAngle[1] = 0.75;
	viaAngle[2] = 0.85;
	return viaAngle;
}

walknetSeparateLeg::~walknetSeparateLeg(void) {
}

double walknetSeparateLeg::selectorNet(const sensor* sensor) {
	return 0.0;
}

double walknetSeparateLeg::stanceNet(const sensor* sensor) {
	return 0.0;
}

double walknetSeparateLeg::swingNet(const sensor* sensor) {
	const double HEIGHT = 1;
	const double MID_COXA_POS = (PEP[0]-AEP[0])/2;
	double middlePos[3] = {0,0,0};

	if(initSwing){  // initialize things here
		initSwing = false;
	}
	else if(true){ 	// is there ground contact
		// lift the leg
	}
	else if(!atPosition(middlePos)){  // Arrived at middle-point
		// go to this point
	}
	else if(!atPosition(AEP)){	// Arrived at AEP
		// go to this point
	}
	else if(true){	// is there ground contact
		// lower the leg
	}

	return 0.0;
}

bool walknetSeparateLeg::atPosition(double targetPos[]) {

	// Make function that checks if we are at a position (with some deadband)
	double coxaError = targetPos[0] ;//- sensor[0];
	double femurError = targetPos[1] ;//- sensor[1];
	double tibiaError = targetPos[2] ;//- sensor[2];
	double deadBand = 0.2;

	if(abs(coxaError) < deadBand && abs(femurError) < deadBand && abs(tibiaError) < deadBand){
		return true;
	} else {
		return false;
	}
}
