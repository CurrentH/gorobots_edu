#include "walknetSeparateLeg.h"

walknetSeparateLeg::walknetSeparateLeg() {
}

walknetSeparateLeg::walknetSeparateLeg(int newlegNum) {
	legNum = newlegNum;
}

double* walknetSeparateLeg::stepWalknetSeprateLeg(const sensor* sensor) {

	//selectorNet() -> stanceNet() || swingNet() -> tragetoryGenerator();

	double viaAngle[3] = {1, 1, 1};
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
	return 0.0;
}
