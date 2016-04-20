#include "walknetSeparateLeg.h"

using namespace std;

walknetSeparateLeg::walknetSeparateLeg( int newlegNum ){
	legNum = newlegNum;
	PEP.assign( 3 , 0 );
	MID.assign( 3 , 0 );
	AEP.assign( 3 , 0 );
	localSensorArray.assign( 4, 0 );
	coordinationRules.assign( 3, 0);

	swingState = SWING_DONE;
	stanceState = STANCE_DONE;
	swingState2 = SWING2_DONE;
	stanceState2 = STANCE2_DONE;

	switch (newlegNum)
	{
		case 0: case 3: PEP[0] = -0.5; PEP[1] = 0.9; PEP[2] = -0.4; // ok (but needs a bit visual tweak)
			break;
		case 1: case 4: PEP[0] = -0.9; PEP[1] = 0.15; PEP[2] = -0.55;
			break;
		case 2: case 5: PEP[0] = -0.75; PEP[1] = 0.0; PEP[2] = -0.55;
			break;
		default: cout << "LEG UNKNOWN";
			break;
	}

	switch( newlegNum )
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

	switch( newlegNum )
	{
		case 0: case 3: AEP[0] =  0.4; AEP[1] = 0.0; AEP[2] = -0.6; // ok (but needs a bit visual tweak)
			break;
		case 1: case 4: AEP[0] = 0.1; AEP[1] = 0.6; AEP[2] = -0.55;
			break;
		case 2: case 5: AEP[0] = -0.2; AEP[1] = 0.9; AEP[2] = -0.7;
			break;
		default: cout << "LEG UNKNOWN";
			break;
	}

}

void walknetSeparateLeg::stepWalknetSeprateLeg( const sensor* sensor, std::vector<double> &viaAngle )
{
	 extractSensor(sensor, legNum, localSensorArray);
	 selectorNet( sensor, viaAngle );
	 //swingNet2(sensor,viaAngle);
	 //stanceNet2(sensor,viaAngle);
}

walknetSeparateLeg::~walknetSeparateLeg(void) {
}

void walknetSeparateLeg::selectorNet( const sensor* sensor, std::vector<double> &viaAngle )
{
	GCunit = getGroundContact();	//	Check if there is Ground Contact
	PEPunit = atPosition( PEP , 0.1);	//	Check if the leg is at the PEP.

	RSunit = RSunit + PEPunit - GCunit;	//	Do the logic that tells the leg if it should move.
	PSunit = PSunit - PEPunit + GCunit + ( coordinationRules[0] && ( coordinationRules[1] || coordinationRules[2]));

	if( RSunit > 1 ){ RSunit = 1; }else if( RSunit < 0 ){ RSunit = 0; }
	if( PSunit > 1 ){ PSunit = 1; }else if( PSunit < 0 ){ PSunit = 0; }

	if( RSunit ){
		startSwing = true; startStance = false; phase = true;
		stanceNet2( sensor, viaAngle );
		swingNet2( sensor, viaAngle );
	}else if( PSunit == true ){
		startSwing = false; startStance = true; phase = false;
		swingNet2( sensor, viaAngle );
		stanceNet2( sensor, viaAngle );
	}

/*
	if( coordinationRules[0] == true && ( RSunit == true || coordinationRules[1] == true || coordinationRules[2] == true ) ){
		startSwing = true; startStance = false; phase = true;
		stanceNet2( sensor, viaAngle );
		swingNet2( sensor, viaAngle );
	}else if( PSunit == true && coordinationRules[0] == false ){
		startSwing = false; startStance = true; phase = false;
		swingNet2( sensor, viaAngle );
		stanceNet2( sensor, viaAngle );
	}
*/
/*
	if( ( RSunit == true || coordinationRules[1] == true || coordinationRules[2] == true) && coordinationRules[0] == true ){
		startSwing = true; startStance = false; phase = true;
		stanceNet2( sensor, viaAngle );
		swingNet2( sensor, viaAngle );
	}else if( PSunit == true || coordinationRules[0] == false ){
		startSwing = false; startStance = true; phase = false;
		swingNet2( sensor, viaAngle );
		stanceNet2( sensor, viaAngle );
	}
*/
}

void walknetSeparateLeg::stanceNet(const sensor* sensor, std::vector<double> &swingNetAngle) {

	const double pos_deadband = 0.01;

	switch(stanceState)
	{
		case SWING_TO_PEP:

			if(!localSensorArray[3]){
				//cout << "GET_GC" << endl;
				stanceState = GET_GC;
			}else if( !atAngle(PEP[0], 0, 0.01) ) {
				//cout << "TO PEP" << endl;
				swingNetAngle[0] = PEP[0];
				swingNetAngle[1] = localSensorArray[1]; // Leave CF and TF as they are
				swingNetAngle[2] = localSensorArray[2];
			}else{
				startStance = false;
				stanceState = STANCE_DONE;
			}
			break;

		case GET_GC:
			//cout << "GET_GC" << endl;
			if(!localSensorArray[3]){

				if(localSensorArray[1] >= -0.9){
					swingNetAngle[0] = localSensorArray[0];
					swingNetAngle[1] = localSensorArray[1] - 0.13; // TODO Faster or slower?
					swingNetAngle[2] = localSensorArray[2];
				} else {
					swingNetAngle[0] = localSensorArray[0];
					swingNetAngle[1] = localSensorArray[1];
					swingNetAngle[2] = localSensorArray[2] - 0.13; // TODO Faster or slower?
				}

			} else {
				stanceState = SWING_TO_PEP;
			}
			break;

		case STANCE_DONE:
			if(startStance)
			{
				stanceState = SWING_TO_PEP;
			}
			break;
		default: cout << "stanceState Error!" << endl;
			break;
	}

}

void walknetSeparateLeg::swingNet(const sensor* sensor, std::vector<double> &swingNetAngle) {

	const double MID_POINT 				= ( AEP[0] + PEP[0] ) / 2;
	const double SWING_HEIGHT_cf	  	= MID[1];
	const double SWING_HEIGHT_ft		= MID[2];
	const double STANCE_HEIGHT_cf	  	= AEP[1];
	const double STANCE_HEIGHT_ft		= AEP[2];
	const double pos_deadband 			= 0.01;

	// Default is set in constructor to be SET_SWING_HEIGHT
	switch(swingState)
	{
		case SET_SWING_HEIGHT:
			// Change to swing height
			swingNetAngle[0] = localSensorArray[0]; // Leave CT as it is
			swingNetAngle[1] = SWING_HEIGHT_cf;
			swingNetAngle[2] = SWING_HEIGHT_ft;

			// Swing height reached
			if( atAngle( SWING_HEIGHT_cf, 1, 0.01 ) && atAngle( SWING_HEIGHT_ft, 2, 0.01 ) )
			{
				swingState = SWING_COXA;
			}
			break;

		case SWING_COXA:
			// Change coxa to AEP position
			swingNetAngle[0] = AEP[0];
			swingNetAngle[1] = localSensorArray[1]; // Leave CF and TF as they are
			swingNetAngle[2] = localSensorArray[2];

			// Lift leg at ground contact
			if(localSensorArray[3]){
				swingState = RAISE_HEIGHT;
			}
			else if( atAngle(AEP[0], 0, 0.01) )  // Coxa AEP reached
			{
				swingState = SET_STANCE_HEIGHT;
			}

			break;

		case SET_STANCE_HEIGHT:

			// Change to swing height
			if(!STANCE_REACHED){
				swingNetAngle[0] = localSensorArray[0];
				swingNetAngle[1] = STANCE_HEIGHT_cf;
				swingNetAngle[2] = STANCE_HEIGHT_ft;
			} else {
				swingNetAngle[0] = localSensorArray[0];
				swingNetAngle[1] = localSensorArray[1];
				swingNetAngle[2] = localSensorArray[2];
			}

			// Swing height reached
			if( (atAngle( STANCE_HEIGHT_cf, 1, 0.01 ) && atAngle( STANCE_HEIGHT_ft, 2, 0.01 )) || STANCE_REACHED )
			{
				STANCE_REACHED = true;

				if(localSensorArray[3]){
					STANCE_REACHED = false;
					startSwing = false;
					swingState = SWING_DONE;
				} else {
					swingState = LOWER_HEIGHT;
				}
			}
			break;
		case LOWER_HEIGHT:
			if(!localSensorArray[3]){

				if(localSensorArray[1] >= -0.9){
					swingNetAngle[0] = localSensorArray[0];
					swingNetAngle[1] = localSensorArray[1] - 0.03; // TODO Faster or slower?
					swingNetAngle[2] = localSensorArray[2];
				} else {
					swingNetAngle[0] = localSensorArray[0];
					swingNetAngle[1] = localSensorArray[1];
					swingNetAngle[2] = localSensorArray[2] - 0.03; // TODO Faster or slower?
				}

			} else {
				swingState = SET_STANCE_HEIGHT;
			}
			break;
		case RAISE_HEIGHT:
			// Raise leg if ground contact
			if(localSensorArray[3]){
				if(localSensorArray[1] <= 0.9){
					swingNetAngle[0] = localSensorArray[0];
					swingNetAngle[1] = localSensorArray[1] + 0.03; // TODO Faster or slower?
					swingNetAngle[2] = localSensorArray[2];
				} else {
					swingNetAngle[0] = localSensorArray[0];
					swingNetAngle[1] = localSensorArray[1];
					swingNetAngle[2] = localSensorArray[2] + 0.03; // TODO Faster or slower?
				}
			} else {
				swingState = SWING_COXA;
			}
			break;
		case SWING_DONE:
			if(startSwing)
			{
				swingState = SET_SWING_HEIGHT;
			}
			break;
		default: cout << "swingState Error!" << endl;
			break;
	}

}

void walknetSeparateLeg::extractSensor( const sensor* sensor, int leg, std::vector<double> & extractedSensors )
{
	//	Set the three first places for the angles for that specific leg.
	for( int i = 0; i < 3; i++ )
	{
		extractedSensors[ i ] = sensor[ leg + i*6 ];
	}

	//	Set index 3, for the contact sensor
	extractedSensors[3] = 0.0;
	for( int i = 0; i < 6; i++ )
	{
		if( sensor[25 + 6*leg + i] == true ){
			extractedSensors[3] = 1.0;
		}
	}
	getGroundContact();
}

bool walknetSeparateLeg::atAngle( double targetPos, int legPartNum, double deadband )
{
	double error = targetPos - localSensorArray[legPartNum];

	if(abs(error) < deadband ){
		return true;
	} else {
		return false;
	}
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
	//TRUE = swing, FALSE = stance
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

void walknetSeparateLeg::stanceNet2(const sensor* sensor, std::vector<double> &viaAngle){
	if(startStance == false){
		stanceState2 = STANCE2_DONE;
	}

	switch(stanceState2)
			{

				case TO_PEP_STANCE:
					if( !atPosition(PEP,0.01) )
					{
						viaAngle[0] = PEP[0];
						viaAngle[1] = PEP[1];
						viaAngle[2] = PEP[2];
					} else {
						startStance = false;
						stanceState2 = STANCE2_DONE;
					}
					break;

				case STANCE2_DONE:
					if(startStance){
						stanceState2 = TO_PEP_STANCE;
					}
					break;

				default: cout << "swingState Error!" << endl;
					break;
			}


}
void walknetSeparateLeg::swingNet2(const sensor* sensor, std::vector<double> &viaAngle){

	const double MID_COXA_POS = (AEP[0] + PEP[0]) / 2;


	if(startSwing == false){
		swingState2 = SWING2_DONE;
	}

	switch(swingState2)
		{
			case LIFT:
				if( !atAngle(MID[1], 1, 0.01) && !atAngle(MID[2], 2, 0.01))
				{
					viaAngle[0] = PEP[0];
					viaAngle[1] = MID[1];
					viaAngle[2] = MID[2];
				} else {
					swingState2 = TO_AEP_SWING;
				}
				break;

			case TO_AEP_SWING:
				if( !atAngle(AEP[0], 0, 0.01) )
				{
					viaAngle[0] = AEP[0];
					viaAngle[1] = MID[1];
					viaAngle[2] = MID[2];
				} else {
					swingState2 = LOWER;
				}
				break;

			case LOWER:
				if( !atAngle(AEP[1], 1, 0.01) && !atAngle(AEP[2], 2, 0.01) )
				{
					viaAngle[0] = AEP[0];
					viaAngle[1] = AEP[1];
					viaAngle[2] = AEP[2];
				} else {
					if(!localSensorArray[3]){
						viaAngle[1] = localSensorArray[1] - 0.08;
					} else {
						startSwing = false;
						swingState2 = SWING2_DONE;
					}
				}
				break;

			case SWING2_DONE:
				if(startSwing){
					swingState2 = LIFT;
				}
				break;

			default: cout << "swingState Error!" << endl;
				break;
		}
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

