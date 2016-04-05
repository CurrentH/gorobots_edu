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

#include "DungBotEmptyController.h"
using namespace std;
using namespace matrix;

// The constructor implements the AbstractController interface. A trial number can be
// passed to the constuctor in order to automate variations in the controller setup
// over different trials.
DungBotEmptyController::DungBotEmptyController( const std::string& name )
: AbstractController( name, "1.0" )
{
	initialised=false;

	ticks_since_init = 0;

	speedSetpoint = 4.0;
	phaseSetpoint = 4.0;
}

DungBotEmptyController::~DungBotEmptyController()
{
}

void DungBotEmptyController::stepNoLearning( const sensor* sensors, int number_sensors, motor* motors, int number_motors )
{
}

void DungBotEmptyController::step( const sensor* sensors, int sensornumber, motor* motors, int motornumber )
{
	// Update internal time
	ticks_since_init++;
}

void DungBotEmptyController::init( int sensornumber, int motornumber, RandGen* randGen )
{
	nSensors = sensornumber;
	nMotors  = motornumber;
	initialised=true;
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


