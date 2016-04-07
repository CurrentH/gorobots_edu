#include "walknetSeparateLeg.h"

using namespace std;

walknetSeparateLeg::walknetSeparateLeg(int newlegNum) {
	legNum = newlegNum;
	PEP.resize( 6 , 0 );
	AEP.resize( 6 , 0 );
	localSensorArray.resize( 4, 0 );
}

std::vector<double> walknetSeparateLeg::stepWalknetSeprateLeg(const sensor* sensor) {
	localSensorArray = extractSensor(sensor, legNum);
	//selectorNet() -> stanceNet() || swingNet() -> tragetoryGenerator();
	std::vector<double> viaAngle(3,0);

	viaAngle[0] = 0.65;
	viaAngle[1] = 0.75;
	viaAngle[2] = 0.85;

	return viaAngle;
}

walknetSeparateLeg::~walknetSeparateLeg(void) {
}

double walknetSeparateLeg::selectorNet(const sensor* sensor)
{
	std::vector<double> tmp = extractSensor( sensor, legNum );

	GCunit = tmp[3];				//	Check if there is Ground Contact
	PEPunit = atPosition( PEP );	//	Check if the leg is at the PEP.

	RSunit = RSunit + PEPunit - GCunit;
	PSunit = PSunit - PEPunit + GCunit;

	if( RSunit == true )
	{
		return swingNet( sensor );
	}
	else if( PSunit == true )
	{
		return stanceNet( sensor );
	}

	return 0.0;
}

double walknetSeparateLeg::stanceNet(const sensor* sensor) {
	initSwing = true;
	return 0.0;
}

//TODO Beware, contact with tibia instead of tarsus
double walknetSeparateLeg::swingNet(const sensor* sensor) {
	//const double HEIGHT = 1;
	//const double MID_COXA_POS = (PEP[0]-AEP[0])/2;
	std::vector<double> middlePos = {0,0,0};

	if(initSwing){  // initialize things here
		initSwing = false;
		stage3 = true;
		stage4 = true;
	}
	else if(localSensorArray[4] == 1){ 	// If there is ground contact
		// lift the leg
		// Height manipulation

	}
	else if(!atPosition(middlePos) && stage3){  // Arrived at middle-point
		// go to this point
	}
	else if(!atPosition(AEP) && stage4){	// Arrived at AEP
		stage3 = false;
		// go to this point
	}
	else if(localSensorArray[4] == 0){	// If there is ground contact
		stage4 = false;
		// lower the leg
	}

	return 0.0;
}

std::vector<double> walknetSeparateLeg::extractSensor( const sensor* sensor, int leg )
{
	std::vector<double> extractedSensors(4,0);
	extractedSensors[4] = 0;

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

bool walknetSeparateLeg::atPosition( std::vector<double> targetPos )
{
	double coxaError = targetPos[0] - localSensorArray[0];
	double femurError = targetPos[1] - localSensorArray[1];
	double tibiaError = targetPos[2] - localSensorArray[2];
	double deadBand = 0.2;

	if(abs(coxaError) < deadBand && abs(femurError) < deadBand && abs(tibiaError) < deadBand){
		return true;
	} else {
		return false;
	}
}
