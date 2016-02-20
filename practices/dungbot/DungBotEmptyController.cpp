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

	ticks_since_init = 0;

	speedSetpoint = 4.0;
	phaseSetpoint = 4.0;
}

DungBotEmptyController::~DungBotEmptyController()
{
}

void DungBotEmptyController::stepNoLearning( const sensor* sensor, int sensorNumber, motor* motor, int motorNumber )
{
	assert( motorNumber >= DungBotMotorSensor::DUNGBOT_MOTOR_MAX );
	assert( sensorNumber >= DungBotMotorSensor::DUNGBOT_SENSOR_MAX );

	//TODO:	Find out how to properly find the data from the sensors, look at the lines below:


	std::cout << motor[DungBotMotorSensor::TR0_m];

	/*
	double leftFrontPosition = sensors[SIdx("left front motor")];
	double leftRearPosition = sensors[SIdx("left rear motor")];
	double rightFrontPosition = sensors[SIdx("right front motor")];
	double rightRearPosition = sensors[SIdx("right rear motor")];
	*/
}

void DungBotEmptyController::step( const sensor* sensors, int sensorNumber, motor* motors, int motorNumber )
{
	stepNoLearning( sensors, sensorNumber, motors, motorNumber );
	// Update internal time
	ticks_since_init++;
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


