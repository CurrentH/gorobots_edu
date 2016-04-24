#include "walknetSeparateLeg.h"

using namespace std;

walknetSeparateLeg::walknetSeparateLeg( int newlegNum ){
	legNum = newlegNum;
	PEP.assign( 3 , 0 );
	MID.assign( 3 , 0 );
	AEP.assign( 3 , 0 );
	maxAEP.assign( 3 , 0 );
	localSensorArray.assign( 4, 0 );
	coordinationRules.assign( 3, 0);

	swingState2 = SWING2_DONE;
	stanceState2 = STANCE2_DONE;

	if(true){ // use the simple robot AEP, PEP and MID
		PEP[0] = -0.4; PEP[1] = -0.6; PEP[2] = 0;
		AEP[0] = 0.6; AEP[1] = 0.0; AEP[2] = 0.0;
		MID[0] = 0.0; MID[1] = 0.0; MID[2] = 0.0;
	} else {
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
							maxAEP = AEP;
				break;
			case 1: case 4: AEP[0] = 0.1; AEP[1] = 0.6; AEP[2] = -0.55;
							maxAEP = AEP;
				break;
			case 2: case 5: AEP[0] = -0.2; AEP[1] = 0.9; AEP[2] = -0.7;
							maxAEP = AEP;
				break;
			default: cout << "LEG UNKNOWN";
				break;
		}
	}
}

walknetSeparateLeg::~walknetSeparateLeg(void) {
}

void walknetSeparateLeg::stepWalknetSeprateLeg( const sensor* sensor, std::vector<double> &viaAngle )
{
	 extractSensor(sensor, legNum, localSensorArray);
	 selectorNet( sensor, viaAngle );

	 //swingNetSimple(sensor,viaAngle);
	 //stanceNetSimple(sensor,viaAngle);
}

void walknetSeparateLeg::selectorNet( const sensor* sensor, std::vector<double> &viaAngle )
{
	GCunit = getGroundContact();			//	Check if there is Ground Contact
	PEPunit = atAngle( PEP[0] , 0, 0.01);	//	Check if the leg is at the PEP.

	if(legNum == 5) std::cout << "PS: " << PSunit << "=" << PSunit << "-" << PEPunit << "+" << GCunit << "+" << coordinationRules[0];
	RSunit = RSunit + PEPunit - GCunit;	//	Logic for entering swingNet
	PSunit = PSunit - PEPunit + GCunit + coordinationRules[0]; //  Logic for entering stanceNet
	if(legNum == 5) std::cout << "  == " << PSunit;

	//PSunit = PSunit + (coordinationRules[0]) || coordinationRules[1] || coordinationRules[2]);

	if( RSunit > 1 ){ RSunit = 1; } else if( RSunit < 0 ){ RSunit = 0; }
	if( PSunit > 1 ){ PSunit = 1; } else if( PSunit < 0 ){ PSunit = 0; }
	if(legNum == 5) std::cout << "  == " << PSunit << endl;

	if( swingState2 == START_SWING ){
		startSwing = true; startStance = false; phase = true;
	}
	else if( stanceState2 == START_STANCE ){
		startSwing = false; startStance = true; phase = false;
	}

	if( RSunit || startSwing){
		//stanceNet1( sensor, viaAngle );
		swingNetSimple( sensor, viaAngle );
	}else if( PSunit || startStance ){
		//swingNet3( sensor, viaAngle );
		stanceNetSimple( sensor, viaAngle );
	}
}


void walknetSeparateLeg::stanceNetSimple(const sensor* sensor, std::vector<double>& viaAngle) {
	if(startStance == false){
		stanceState2 = STANCE2_DONE;
	}

	switch(stanceState2)
	{
		case START_STANCE:
			stanceState2 = TO_PEP_STANCE;
			break;

		case TO_PEP_STANCE:
			if( !atAngle(PEP[0], 0, 0.01) )
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
			swingState2 = START_SWING;
			/*
			if(startStance){
				stanceState2 = TO_PEP_STANCE;
			}
			*/
			break;

		default: cout << "stanceState Error!" << endl;
			break;
	}
}

void walknetSeparateLeg::swingNetSimple(const sensor* sensor, std::vector<double>& viaAngle) {
	const double MID_COXA_POS = 0;


		if(startSwing == false){
			swingState2 = SWING2_DONE;
		}

		switch(swingState2)
			{
				case LIFT:
					if( !atAngle(MID[1], 1, 0.01) )
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

						if(!localSensorArray[3]){
							viaAngle[1] = localSensorArray[1] - 0.2;
						} else {
							startSwing = false;
							swingState2 = SWING2_DONE;
						}

					break;

				case SWING2_DONE:
					/*
					if(startSwing){
						swingState2 = LIFT;
					}
					*/
					break;

				default: cout << "swingState Error!" << endl;
					break;
			}
}

void walknetSeparateLeg::stanceNet1(const sensor* sensor, std::vector<double> &viaAngle){
	if(startStance == false){
		stanceState2 = STANCE2_DONE;
	}

	switch(stanceState2)
	{
		case START_STANCE:
 			stanceState2 = TO_PEP_STANCE;
 			break;

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
			swingState2 = START_SWING;
			/*
			if(startStance){
				stanceState2 = TO_PEP_STANCE;
			}
			*/
			break;
		default: cout << "swingState Error!" << endl;
			break;
	}
}

void walknetSeparateLeg::swingNet1(const sensor* sensor, std::vector<double> &viaAngle){

	const double MID_COXA_POS = (AEP[0] + PEP[0]) / 2;

	switch(swingState2)
	{
		case TO_MID_SWING:

			if( !atPosition(MID,0.01) )
			{
				viaAngle[0] = MID[0];
				viaAngle[1] = MID[1];
				viaAngle[2] = MID[2];
			} else {
				swingState2 = TO_AEP_SWING;
			}
			break;

		case TO_AEP_SWING:
			if( !atPosition(AEP,0.01) )
			{
				viaAngle[0] = AEP[0];
				viaAngle[1] = AEP[1];
				viaAngle[2] = AEP[2];
			} else {
				startSwing = false;
				swingState2 = SWING2_DONE;
			}
			break;

		case SWING2_DONE:
			if(startSwing){
				swingState2 = TO_MID_SWING;
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
			case START_SWING:
				swingState2 = LIFT;
				break;

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
				/*
				if(startSwing){
					swingState2 = LIFT;
				}
 				*/
			default: cout << "swingState Error!" << endl;
				break;
		}
}

void walknetSeparateLeg::swingNet3(const sensor* sensor, std::vector<double> &viaAngle){

	const double MID_COXA_POS = (AEP[0] + PEP[0]) / 2;


	if(startSwing == false){
		swingState2 = SWING2_DONE;
		AEP = maxAEP;
	}

	switch(swingState2)
	{
		case LIFT:
			swingState2 = TO_AEP_SWING;
			break;

		case TO_AEP_SWING:
			if( !atAngle(AEP[0], 0, 0.01) )
			{
				viaAngle[0] = AEP[0];
				viaAngle[1] = trajectory(5,1);
				viaAngle[2] = trajectory(-1,2);
			} else {
				swingState2 = FINAL_SWING_POS;
			}
			break;

		case FINAL_SWING_POS:
			if( (!atAngle(AEP[1], 1, 0.01) && !atAngle(AEP[2], 2, 0.01)) )
			{
				viaAngle[0] = AEP[0];
				viaAngle[1] = AEP[1];
				viaAngle[2] = AEP[2];
			} else{
				swingState2 = LOWER;
			}
			break;

		case LOWER:
			if(!localSensorArray[3]){
				viaAngle[1] = localSensorArray[1] - 0.1;
				viaAngle[1] = localSensorArray[2] + 0.06;
			} else {
				startSwing = false;
				swingState2 = SWING2_DONE;
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

void walknetSeparateLeg::swingNet4(const sensor* sensor, std::vector<double> &viaAngle){

	const double MID_COXA_POS = (AEP[0] + PEP[0]) / 2;
	double coxaspeed = 0, femurUp = 0, femurDown = 0, tibiaUp = 0, tibiaDown = 0;

	// Speeds for the different joints: 1 = MAX_SPEED
	// It is possible to set the following more precise
	switch( legNum )
	{
		case 0: case 3: //FRONTLEGS
			coxaspeed = 0.14;
			femurUp = 0.20; femurDown = 0.32;
			tibiaUp = 0.05; tibiaDown = 0.03;
			break;
		case 1: case 4: //MIDDLELEGS
			coxaspeed = 0.15;
			femurUp = 0.30; femurDown = 0.1;
			tibiaUp = 0.30; tibiaDown = 0.12;
			break;
		case 2: case 5: //HINDLEGS
			coxaspeed = 0.1;
			femurUp = 1.0; femurDown = 0.02;
			tibiaUp = 1.0; tibiaDown = 0.08;
			break;
		default: cout << "LEG UNKNOWN";
			break;
	}


	if(startSwing == false){
		swingState2 = SWING2_DONE;
		AEP = maxAEP;
	}

	switch(swingState2)
	{
		case LIFT:
			swingState2 = TO_AEP_SWING;
			break;

		case TO_AEP_SWING:
			if( !atAngle(AEP[0], 0, 0.01) )
			{

				if ( localSensorArray[0] <= MID_COXA_POS ) {
					viaAngle[0] = localSensorArray[0] + coxaspeed;
					viaAngle[1] = localSensorArray[1] + femurUp;
					viaAngle[2] = localSensorArray[2] - tibiaUp;

				} else {
					viaAngle[0] = localSensorArray[0] + coxaspeed;
					if(!localSensorArray[3]){
						viaAngle[1] = localSensorArray[1] - femurDown;
						viaAngle[2] = localSensorArray[2] + tibiaDown;
					} else {
						viaAngle[1] = localSensorArray[1];
						viaAngle[2] = localSensorArray[2];
					}
				}

			} else {
				swingState2 = LOWER;
			}
			break;

		case LOWER:
			if( (!atAngle(AEP[1], 1, 0.01) && !atAngle(AEP[2], 2, 0.01)) && !localSensorArray[3] )
			{
				viaAngle[0] = AEP[0];
				viaAngle[1] = AEP[1];
				viaAngle[2] = AEP[2];
			} else {
				if(!localSensorArray[3] && false){
					viaAngle[1] = localSensorArray[1] - 0.1;
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
	getGroundContact();
}


double walknetSeparateLeg::trajectory(double height, int legPart)
{
	double x1 = PEP[0];
	double y1 = PEP[legPart];

	double x2 = (PEP[0] + AEP[0])/2;
	double y2 = height;

	double x3 = AEP[0];
	double y3 = AEP[legPart];

	double A1 = -x1*x1+x2*x2;
	double B1 = -x1+x2;
	double D1 = -y1+y2;
	double A2 = -x2*x2+x3*x3;
	double B2 = -x2+x3;
	double D2 = -y2+y3;
	double BM = -(B2/B1);
	double A3 = BM * A1 + A2;
	double D3 = BM * D1 + D2;

	double a = D3/A3;
	double b = (D1-A1*a)/(B1);
	double c = y1-a*x1*x1-b*x1;

	return a*localSensorArray[0]*localSensorArray[0]+b*localSensorArray[0]+c;
}

void walknetSeparateLeg::extractSensor( const sensor* sensor, int leg, std::vector<double> & extractedSensors )
{
        //      Set the three first places for the angles for that specific leg.
        for( int i = 0; i < 3; i++ )
        {
                extractedSensors[ i ] = sensor[ leg + i*6 ];
        }

        //      Set index 3, for the contact sensor
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

void walknetSeparateLeg::getAEP( std::vector<double> & tmpAEP )
{
	tmpAEP = AEP;
}

std::vector<double> walknetSeparateLeg::getPEP( void )
{
	return PEP;
}

void walknetSeparateLeg::setAEP( std::vector<double> & newAEP )
{
	if( newAEP[0] < maxAEP[0] && newAEP[1] < maxAEP[1] && newAEP[2] < maxAEP[2] )
	{
		AEP = newAEP;
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

