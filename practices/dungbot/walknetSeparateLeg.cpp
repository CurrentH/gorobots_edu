#include "walknetSeparateLeg.h"

walknetSeparateLeg::walknetSeparateLeg() {
}

walknetSeparateLeg::walknetSeparateLeg(int newlegNum) {
	legNum = newlegNum;
}

double* walknetSeparateLeg::stepWalknetSeprateLeg(const sensor* sensor) {

	//selectorNet() -> stanceNet() || swingNet() -> tragetoryGenerator();
	double* arrayPointer;
	double viaAngle[3] = {0.9, 0.9, 0.9};

	arrayPointer = viaAngle;
	return arrayPointer;
}

walknetSeparateLeg::~walknetSeparateLeg(void) {
}

double walknetSeparateLeg::selectorNet(const sensor* sensor)
{
	double* tmp;

	tmp = extractSensor( sensor, legNum );
	GCunit = tmp[3];	//	Check if there is Ground Contact
	PEPunit = checkPEP();

	//RSunit = RSunit + PEPunit - GCunit;
	//PSunit = PSunit - PEPunit + GCunit;

	if( RSunit == true )
	{
		swingNet( sensor );
	}
	else if( PSunit == true )
	{
		stanceNet( sensor );
	}

	return 0.0;
}

double walknetSeparateLeg::stanceNet(const sensor* sensor) {
	return 0.0;
}

double walknetSeparateLeg::swingNet(const sensor* sensor) {
	//const double HEIGHT = 1;
	//const double MID_COXA_POS = (PEP[0]-AEP[0])/2;
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

bool walknetSeparateLeg::checkPEP()
{
	bool check = false;
	double treshold;
	double distance;
	//double PEP[3] = {0};
	//double pos[3] = {0};

	//distance = sqrt( (PEP[0]-pos[0])^2 + (PEP[1]-pos[1])^2 + (PEP[2]-pos[2])^2 );

	if( abs( distance ) < treshold )
	{
		check = true;
	}

	return check;
}

double* walknetSeparateLeg::extractSensor( const sensor* sensor, int leg )
{
	double extractedSensors[4] = {0};

	for( int i = 0; i < 3; i++ )
	{
		extractedSensors[ i ] = sensor[ leg + i*6 ];
	}

	for( int i = 0; i < 5; i++ )
	{
		if( sensor[25 + 5*leg + i] == true ){
			extractedSensors[4] = 1;
		}
	}

	return extractedSensors;
}

bool walknetSeparateLeg::atPosition( double targetPos[] )
{
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
