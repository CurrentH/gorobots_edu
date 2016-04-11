#include "walknetSeparateLeg.h"

using namespace std;

walknetSeparateLeg::walknetSeparateLeg( int newlegNum ){
	legNum = newlegNum;
	PEP.assign( 3 , 0 );
	MID.assign( 3 , 0 );
	AEP.assign( 3 , 0 );
	localSensorArray.assign( 4, 0 );
	coordinationRules.assign( 3, 0);

	switch (newlegNum)
	{
	case 0: case 3: PEP[0] = -0.5; PEP[1] = 0.9; PEP[2] = -0.4; // ok (but needs a bit visual tweak)
	    break;
	case 1: case 4: PEP[0] = -1.0; PEP[1] = 0.15; PEP[2] = -0.55;
	    break;
	case 2: case 5: PEP[0] = -0.75; PEP[1] = 0.0; PEP[2] = -0.55;
	    break;
	default: cout << "LEG UNKNOWN";
	    break;
	}

	switch (newlegNum)
	{
	case 0: case 3: MID[0] =  0.3; MID[1] = 1.0; MID[2] = -0.5; // ok (but needs a bit visual tweak)
	    break;
	case 1: case 4: MID[0] = -0.5; MID[1] = 0.8; MID[2] = -0.6;
	    break;
	case 2: case 5: MID[0] = -0.5; MID[1] = 1.0; MID[2] = -0.7;
	    break;
	default: cout << "LEG UNKNOWN";
	    break;
	}

	switch (newlegNum)
	{
	case 0: case 3: AEP[0] =  0.4; AEP[1] = 0.0; AEP[2] = -0.8; // ok (but needs a bit visual tweak)
	    break;
	case 1: case 4: AEP[0] = 0.1; AEP[1] = 0.6; AEP[2] = -0.55;
	    break;
	case 2: case 5: AEP[0] = -0.2; AEP[1] = 1.0; AEP[2] = -0.7;
	    break;
	default: cout << "LEG UNKNOWN";
	    break;
	}

}

void walknetSeparateLeg::stepWalknetSeprateLeg( const sensor* sensor, std::vector<double> &viaAngle )
{
	 extractSensor(sensor, legNum, localSensorArray);
	 selectorNet( sensor, viaAngle );

	 //swingNet(sensor, viaAngle);
/*
	if(legNum == 3 || legNum == 0)
	{
		//swingNet(sensor, viaAngle);
		viaAngle = PEP;
	}
	else if(legNum == 2 || legNum == 5 )
	{
		viaAngle[0] = -1.0;
		viaAngle[1] = 1.0;
		viaAngle[2] = -1.0;
	}
	else
	{
		viaAngle[0] = 1.0;
		viaAngle[1] = 1.0;
		viaAngle[2] = -1.0;
	}
*/
}

walknetSeparateLeg::~walknetSeparateLeg(void) {
}

void walknetSeparateLeg::selectorNet( const sensor* sensor, std::vector<double> &viaAngle )
{
	GCunit = getGroundContact();	//	Check if there is Ground Contact
	PEPunit = atPosition( PEP , 0.01);	//	Check if the leg is at the PEP.

	RSunit = RSunit + PEPunit - GCunit;	//	Do the logic that tells the leg if it should move.
	PSunit = PSunit - PEPunit + GCunit;

	if( (RSunit == true || coordinationRules[1] == true || coordinationRules[2] == true) && coordinationRules[0] == false )
	{
		phase = true;
		swingNet( sensor, viaAngle );
	}
	else if( PSunit == true || coordinationRules[0] == true )
	{
		phase = false;
		stanceNet( sensor, viaAngle );
	}

}

void walknetSeparateLeg::stanceNet(const sensor* sensor, std::vector<double> &swingNetAngle) {
	const double pos_deadband = 0.01;

	/*
	 * Stance net should maybe also have a mid point
	 * The height is changing when coxa is rotating
	 * Resulting in the legs being lower than PEP and APE
	 * At the mid points between AEP and PEP. This should maybe
	 * be done mathematically right?
	 */

	if(initStance){
		initStance = false;
		stageAEP = true;
	}
	if(!atPosition(AEP, pos_deadband) && stageAEP){ // It should be in its AEP (from swing)
		swingNetAngle=AEP;

	} else if(!atPosition(PEP, pos_deadband)){
		swingNetAngle=PEP;
		stageAEP = false;

	} else{
		initSwing = true;
		//stageAEP = true; // Use this to repeat stanceNet
	}
}

//TODO Beware, contact with tibia instead of tarsus
void walknetSeparateLeg::swingNet(const sensor* sensor, std::vector<double> &swingNetAngle) {

	const double pos_deadband = 0.01;

	if(initSwing){
		initSwing = false;
		stagePEP = true;
		stageMID = true;
	}
	if(!atPosition(PEP, pos_deadband) && stagePEP){ // It should be in its PEP (from stance)
		swingNetAngle=PEP;

	} else if(!atPosition(MID, pos_deadband) && stageMID){
		swingNetAngle=MID;
		stagePEP = false;

	} else if(!atPosition(AEP, pos_deadband)){
		swingNetAngle=AEP;
		stageMID = false;

	} else{
		//stagePEP = true; // Use this to repeat swingNet
		//stageAEP = true; // Use this to repeat stanceNet
		initStance = true;
	}

}

void walknetSeparateLeg::extractSensor( const sensor* sensor, int leg, std::vector<double> & extractedSensors )
{
	for( int i = 0; i < 3; i++ )
	{
		extractedSensors[ i ] = sensor[ leg + i*6 ];
	}

	for( int i = 0; i < 7; i++ )
	{
		if( sensor[25 + 6*leg + i] == true ){
			extractedSensors[3] = 1.0;
		}
	}
	getGroundContact();
}

bool walknetSeparateLeg::atPosition( std::vector<double> targetPos, double deadband )
{
	double coxaError = targetPos[0] - localSensorArray[0];
	double femurError = targetPos[1] - localSensorArray[1];
	double tibiaError = targetPos[2] - localSensorArray[2];

	if(abs(coxaError) < deadband && abs(femurError) < deadband && abs(tibiaError) < deadband){
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
