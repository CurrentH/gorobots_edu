/*
 * rollnetSeparateLeg.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: mat
 */

#include "rollnetSeparateLeg.h"

using namespace std;

rollnetSeparateLeg::rollnetSeparateLeg( int newlegNum ){
	legNum = newlegNum;
}

rollnetSeparateLeg::~rollnetSeparateLeg(void) {
}

void rollnetSeparateLeg::stepRollnetSeprateLeg( const sensor* sensor, std::vector<double> &viaAngle, std::vector<double> &jointVel )
{
	 extractSensor(sensor, legNum, localSensorArray);
	 // selectorNet( sensor, viaAngle, jointVel );
}

void rollnetSeparateLeg::extractSensor( const sensor* sensor, int leg, std::vector<double> & extractedSensors )
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

bool rollnetSeparateLeg::atAngle( double targetPos, int legPartNum, double deadband )
{
	double error = targetPos - localSensorArray[legPartNum];

	if(abs(error) < deadband ){
		return true;
	} else {
		return false;
	}
}

bool rollnetSeparateLeg::atPosition( std::vector<double> targetPos, double deadband )
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

bool rollnetSeparateLeg::getGroundContact( void )
{
	return localSensorArray[3];
}



