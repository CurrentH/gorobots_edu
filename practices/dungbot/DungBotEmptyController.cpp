/*****************************************************************************
*   "THE BEER-WARE LICENSE" (Revision 43):
*   This software was written by Theis Str�m-Hansen <thstroemhansen@gmail.com>
*   and Mathias Thor <mathias.thor@gmail.com>
*   As long as you retain this notice you can do whatever you want with it.
*   If we meet some day, and you think this stuff is worth it, you can buy me
*   a beer in return.
*
*   Should this software ever become self-aware, remember: I am your master
*****************************************************************************/

#include <assert.h>
#include <iostream>

#include "DungBotEmptyController.h"

using namespace std;
using namespace matrix;

// The constructor implements the AbstractController interface. A trial number can be
// passed to the constuctor in order to automate variations in the controller setup
// over different trials.
DungBotEmptyController::DungBotEmptyController( const std::string& name )
: AbstractController( name, "1.0" )
{
	initialised = false;

	ticks_since_init = 0.0;

	speedSetpoint = 4.0;
	phaseSetpoint = 4.0;

	nSensors = 0;
	nMotors = 0;
}

DungBotEmptyController::~DungBotEmptyController()
{
}

void DungBotEmptyController::stepNoLearning( const sensor* sensor, int sensorNumber, motor* motor, int motorNumber )
{
	assert( motorNumber >= DungBotMotorSensor::DUNGBOT_MOTOR_MAX );
	assert( sensorNumber >= DungBotMotorSensor::DUNGBOT_SENSOR_MAX );
}

void DungBotEmptyController::step( const sensor* sensor, int sensorNumber, motor* motor, int motorNumber )
{
	//	Update internal time
	ticks_since_init++;

	for( int i = 0; i < DungBotMotorSensor::DUNGBOT_MOTOR_MAX; i++ )
	{

		//motor[i] = 0;
		motor[i] = sin( 0.001 * ticks_since_init );

		if( int(ticks_since_init)%50 == 0 )
		{
			std::cout << "------------------------------------------------------------------" << std::endl;

			std::cout << "Coxa:  " << motor[0] << " " << motor[1] << " " << motor[2] << " "
									<< motor[3] << " " << motor[4] << " " << motor[5] << std::endl;
			std::cout << "Femur: " << motor[6] << " " << motor[7] << " " << motor[8] << " "
									<< motor[9] << " " << motor[10] << " " << motor[11] << std::endl;
			std::cout << "Tibia: " << motor[12] << " " << motor[13] << " " << motor[14] << " "
									<< motor[15] << " " << motor[16] << " " << motor[17] << std::endl;
			std::cout << "Coxa:  " << sensor[0] << " " << sensor[1] << " " << sensor[2] << " "
									<< sensor[3] << " " << sensor[4] << " " << sensor[5] << std::endl;
			std::cout << "Femur: " << sensor[6] << " " << sensor[7] << " " << sensor[8] << " "
									<< sensor[9] << " " << sensor[10] << " " << sensor[11] << std::endl;
			std::cout << "Tibia: " << sensor[12] << " " << sensor[13] << " " << sensor[14] << " "
									<< sensor[15] << " " << sensor[16] << " " << sensor[17] << std::endl;

			std::cout << "------------------------------------------------------------------" << std::endl;
		}



	}

}

void DungBotEmptyController::init( int sensorNumber, int motorNumber, RandGen* randGen )
{
	assert( motorNumber >= DungBotMotorSensor::DUNGBOT_MOTOR_MAX );
	assert( sensorNumber >= DungBotMotorSensor::DUNGBOT_SENSOR_MAX );

	nSensors = sensorNumber;
	nMotors = motorNumber;
	initialised = true;
}

int DungBotEmptyController::getSensorNumber() const
{
	return nSensors;
}

int DungBotEmptyController::getMotorNumber() const
{
	return nMotors;
}

bool DungBotEmptyController::store( FILE* f ) const
{
	Configurable::print(f,0);
	return true;
}

bool DungBotEmptyController::restore( FILE* f )
{
	Configurable::parse(f);
	return true;
}


