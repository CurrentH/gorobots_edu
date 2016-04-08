#include "walknetSeparateLeg.h"

using namespace std;

walknetSeparateLeg::walknetSeparateLeg( int newlegNum ){
	legNum = newlegNum;
	PEP.assign( 3 , 0 );
	AEP.assign( 3 , 0 );
	localSensorArray.assign( 4, 0 );
	coordinationRules.assign( 3, 0);

	switch (newlegNum)
	{
	case 0: case 3: PEP[0] = -1.0; PEP[1] = 0.4; PEP[2] = -0.7;
	    break;
	case 1: case 4: PEP[0] = -1.0; PEP[1] = -0.2; PEP[2] = -0.4;
	    break;
	case 2: case 5: PEP[0] = -0.65; PEP[1] = -0.2; PEP[2] = -0.3;
	    break;
	default: cout << "LEG UNKNOWN";
	    break;
	}

	switch (newlegNum)
	{
	case 0: case 3: AEP[0] = 0.2; AEP[1] = 0.2; AEP[2] = -0.7;
	    break;
	case 1: case 4: AEP[0] = 0.5; AEP[1] = 0.5; AEP[2] = -0.6;
	    break;
	case 2: case 5: AEP[0] = 0.5; AEP[1] = 0.85; AEP[2] = -0.9;
	    break;
	default: cout << "LEG UNKNOWN";
	    break;
	}

}

void walknetSeparateLeg::stepWalknetSeprateLeg( const sensor* sensor, std::vector<double> &viaAngle ) {
	 extractSensor(sensor, legNum, localSensorArray);

	 //selectorNet( sensor, viaAngle );

	if(legNum == 1 || legNum == 4 || true){
		swingNet(sensor, viaAngle);
		//viaAngle = PEP;
	} else if(legNum == 2 || legNum == 5 ){
	viaAngle[0] = -1.0;
	viaAngle[1] = 1.0;
	viaAngle[2] = -1.0;
	} else{
	viaAngle[0] = 1.0;
	viaAngle[1] = 1.0;
	viaAngle[2] = -1.0;
	}

}

walknetSeparateLeg::~walknetSeparateLeg(void) {
}

void walknetSeparateLeg::selectorNet( const sensor* sensor, std::vector<double> &viaAngle )
{
	GCunit = getGroundContact();	//	Check if there is Ground Contact
	PEPunit = atPosition( PEP );	//	Check if the leg is at the PEP.

	RSunit = RSunit + PEPunit - GCunit;	//	Do the logic that tells the leg if it should move.
	PSunit = PSunit - PEPunit + GCunit;


	if( legNum == 2 )
	{
		std::cout << RSunit << "+"<< PEPunit << "-" << GCunit << std::endl;
		std::cout << PSunit << "-"<< PEPunit << "+" << GCunit << std::endl;
	}
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

	if( RSunit == true || coordinationRules[1] == true || coordinationRules[2] == true )
	{
		std::cout << "TEST1" << std::endl;
		phase = true;
		swingNet( sensor, viaAngle );
	}
	else if( PSunit == true || coordinationRules[0] == true )
	{
		std::cout << "\t TEST1" << std::endl;
		phase = false;
		stanceNet( sensor, viaAngle );
	}
}

void walknetSeparateLeg::stanceNet(const sensor* sensor, std::vector<double> &swingNetAngle) {
	initSwing = true;
	std::vector<double> middlePos(3,0);
}

//TODO Beware, contact with tibia instead of tarsus
void walknetSeparateLeg::swingNet(const sensor* sensor, std::vector<double> &swingNetAngle) {

	//const double HEIGHT = 1;
	const double MID_COXA_POS = (PEP[0]-AEP[0])/2;

	std::vector<double> middlePos(3,0);
	middlePos[0] = MID_COXA_POS;
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
			extractedSensors[3] = 1.0;
		}
	}
	getGroundContact();
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
std::vector<double> walknetSeparateLeg::getAEP( void )
{
	return AEP;
}
std::vector<double> walknetSeparateLeg::getPEP( void )
{
	return PEP;
}


