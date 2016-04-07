#include "walknetSeparateLeg.h"

using namespace std;

walknetSeparateLeg::walknetSeparateLeg(int newlegNum) {
	legNum = newlegNum;
	PEP.assign( 3 , 0 );
	AEP.assign( 3 , 0 );
	localSensorArray.assign( 4, 0 );
	coordinationRules.assign( 3, 0);
}

void walknetSeparateLeg::stepWalknetSeprateLeg(const sensor* sensor, std::vector<double> &viaAngle) {
	 extractSensor(sensor, legNum, localSensorArray);
	//selectorNet() -> stanceNet() || swingNet() -> tragetoryGenerator();

	viaAngle[0] = 0.1;
	viaAngle[1] = 0.1;
	viaAngle[2] = 0.1;
}

walknetSeparateLeg::~walknetSeparateLeg(void) {
}

void walknetSeparateLeg::selectorNet(const sensor* sensor)
{
	GCunit = localSensorArray[3];	//	Check if there is Ground Contact
	PEPunit = atPosition( PEP );	//	Check if the leg is at the PEP.

	RSunit = RSunit + PEPunit - GCunit;	//	Do the logic that tells the leg if it should move.
	PSunit = PSunit - PEPunit + GCunit;

/*
	RSunit:
	true + true - true = 1;		Have just walked	Is at PEP		Touches ground			Do: SwingNet
	true + true - false = 2;	Have just walked	Is at PEP		Does not touch ground	Do: SwingNet
	true + false - true = 0;	Have just walked	Is not at PEP	Touches ground			Do: Nothing
	true + false - false = 1;	Have just walked	Is not at PEP	Does not touch ground	Do: SwingNet
	false + true - true = 0;	Was just standing	Is at PEP		Touches ground			Do: SwingNet
	false + true - false = 1;	Was just standing	Is at PEP		Does not touch ground	Do: Nothing
	false + false - true = -1;	Was just standing	Is not at PEP	Touches ground			Do: Nothing
	false + false - false = 0;	Was just standing	Is not at PEP	Does not touch ground	Do: Nothing

	PSunit:
	true - true + true = 1;		Was just standing	Is not at PEP	Touches ground			Do: StanceNet
	true - true + false = 0;	Was just standing	Is not at PEP	Does not touch ground	Do: Nothing
	true - false + true = 2;	Was just standing	Is at PEP		Touches ground			Do: StanceNet
	true - false + false = 1;	Was just standing	Is at PEP		Does not touch ground	Do: StanceNet
	false - true + true = 0;	Have just walked	Is not at PEP	Touches ground			Do: Nothing
	false - true + false = -1;	Have just walked	Is not at PEP	Does not touch ground	Do: Nothing
	false - false + true = 1;	Have just walked	Is at PEP		Touches ground			Do: StanceNet
	false - false + false = 0;	Have just walked	Is not at PEP	Does not touch ground	Do: Nothing
*/

	if( RSunit == true )
	{
		swingNet( sensor );
	}
	else if( PSunit == true )
	{
		stanceNet( sensor );
	}
}

std::vector<double> walknetSeparateLeg::stanceNet(const sensor* sensor) {
	initSwing = true;
	std::vector<double> middlePos(3,0);
	return middlePos;
}

//TODO Beware, contact with tibia instead of tarsus
std::vector<double> walknetSeparateLeg::swingNet(const sensor* sensor) {

	//const double HEIGHT = 1;
	//const double MID_COXA_POS = (PEP[0]-AEP[0])/2;

	std::vector<double> middlePos(3,0);
	std::vector<double> swingNetAngle(3,0);

	if(initSwing){  // initialize things here
		initSwing = false;
		stage2 = true;
		stage3 = true;
		stage4 = true;

		swingNetAngle[1] = 0.5; // Femur angle
		swingNetAngle[2] = 0.5; // Tibia angle
	}
	else if(localSensorArray[4] == 1 && stage3){ 	// If there is ground contact
		// lift the leg
		// Height manipulation
		swingNetAngle[1] += 0.1;

	}
	else if(!atPosition(middlePos) && stage3){  // Arrived at middle-point
		swingNetAngle = middlePos;
	}
	else if(!atPosition(AEP) && stage4){	// Arrived at AEP
		stage3 = false;	// do not go to middle point again.
		swingNetAngle = AEP;
	}
	else if(localSensorArray[4] == 0){	// If there is ground contact
		stage2 = false; // do not lift again.
		stage4 = false;	// do not go to AEP again

		// lower the leg
		swingNetAngle[1] -= 0.1;

	}

	return swingNetAngle;
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

bool walknetSeparateLeg::getPhase( void )
{
	return phase;
}
bool walknetSeparateLeg::getGroundContact( void )
{
	return localSensorArray[3];
}
void walknetSeparateLeg::setRule( int index, bool flag )
{
	coordinationRules[index] = flag;
}

