#include "walknetSeparateLeg.h"

using namespace std;

walknetSeparateLeg::walknetSeparateLeg(int newlegNum) {
	legNum = newlegNum;
	PEP.resize( 6 , 0 );
	AEP.resize( 6 , 0 );
	localSensorArray.assign( 4, 0 );

	PEP[0] = -0.1;
	PEP[1] = 0.0;
	PEP[2] = -0.9;

	AEP[0] = 0.1;
	AEP[1] = 0.0;
	AEP[2] = -0.9;
}

void walknetSeparateLeg::stepWalknetSeprateLeg(const sensor* sensor, std::vector<double> &viaAngle) {
	 extractSensor(sensor, legNum, localSensorArray);
	//selectorNet() -> stanceNet() || swingNet() -> tragetoryGenerator();
	if(legNum == 1 || true){
		swingNet(sensor, viaAngle);
	} else if(legNum == 2 || legNum == 5 ){
	viaAngle[0] = -1.0;
	viaAngle[1] = 1.2;
	viaAngle[2] = -1.8;
	} else{
	viaAngle[0] = 1.0;
	viaAngle[1] = 1.2;
	viaAngle[2] = -1.8;
	}
}

walknetSeparateLeg::~walknetSeparateLeg(void) {
}

std::vector<double> walknetSeparateLeg::selectorNet(const sensor* sensor)
{
	std::vector<double> tmp(4,0);
	extractSensor( sensor, legNum, tmp );

	GCunit = tmp[3];				//	Check if there is Ground Contact
	PEPunit = atPosition( PEP );	//	Check if the leg is at the PEP.

	RSunit = RSunit + PEPunit - GCunit;
	PSunit = PSunit - PEPunit + GCunit;

	if( RSunit == true )
	{
		//return swingNet( sensor );
	}
	else if( PSunit == true )
	{
		return stanceNet( sensor );
	}

	std::vector<double>nullVector(0);
	return nullVector;
}

std::vector<double> walknetSeparateLeg::stanceNet(const sensor* sensor) {
	initSwing = true;
	std::vector<double> middlePos(3,0);
	return middlePos;
}

//TODO Beware, contact with tibia instead of tarsus
void walknetSeparateLeg::swingNet(const sensor* sensor, std::vector<double> &swingNetAngle) {

	//const double HEIGHT = 1;
	//const double MID_COXA_POS = (PEP[0]-AEP[0])/2;

	std::vector<double> middlePos(3,0);
	middlePos[0] = 0.0;
	middlePos[1] = 1.0;
	middlePos[2] = -0.5;

	if(!atPosition(PEP) && stage2){
		swingNetAngle=PEP;

	} else if(!atPosition(middlePos) && stage3){
		swingNetAngle=middlePos;
		stage2 = false;

	} else if(!atPosition(AEP)){
		swingNetAngle=AEP;
		stage3 = false;

	} else{
		stage2 = true;
		stage3 = true;
	}


	/*
	if(initSwing){  // initialize things here
		initSwing = false;
		stage2 = true;
		stage3 = true;
		stage4 = true;

		swingNetAngle[0] = PEP[0];
		swingNetAngle[1] = PEP[1]; // Femur angle
		swingNetAngle[2] = PEP[2]; // Tibia angle
	}
	else if(false){ 	// If there is ground contact localSensorArray[4] == 1 && stage3
		// lift the leg
		// Height manipulation
		swingNetAngle[1] += 0.01;
		swingNetAngle[2] += 0.005;
		cout << "Lift" << endl;

	}
	else if(!atPosition(middlePos) && stage3){  // Arrived at middle-point
		swingNetAngle = middlePos;
		cout << "middlePos" << endl;
	}
	else if(!atPosition(AEP) && stage4){	// Arrived at AEP
		stage3 = false;	// do not go to middle point again.
		swingNetAngle = AEP;
		cout << "AEPpos" << endl;
	}
	else if(true){	// If there is ground contact localSensorArray[4] == 0
		stage2 = false; // do not lift again.
		stage4 = false;	// do not go to AEP again
		// lower the leg
		swingNetAngle[1] -= 0.1;

		stage3 = true;
	}*/

}

void walknetSeparateLeg::extractSensor( const sensor* sensor, int leg, std::vector<double> & extractedSensors )
{

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
