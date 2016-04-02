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

#include "DungBotEmptyController.h"
#include "walknetcontroller.h"

//using namespace matrix;

// The constructor implements the AbstractController interface. A trial number can be
// passed to the constructor in order to automate variations in the controller setup
// over different trials.
DungBotEmptyController::DungBotEmptyController( const std::string& name  )
: AbstractController( name, "1.0" )
{
	initialised = false;

	ticks_since_init = 0.0;

	nSensors = 0;
	nMotors = 0;
}

DungBotEmptyController::~DungBotEmptyController()
{
	if( writeOutput )
	{
		outputFile.close();
	}
}

void DungBotEmptyController::stepNoLearning( const sensor* sensor, int sensorNumber, motor* motor, int motorNumber )
{
	assert( motorNumber >= DungBotMotorSensor::DUNGBOT_MOTOR_MAX );
	assert( sensorNumber >= DungBotMotorSensor::DUNGBOT_SENSOR_MAX );

	//	Update internal time
	ticks_since_init++;

	if( int(ticks_since_init) < 1000 ) // Start after 1000 ticks in init position. Please drop before times run out
	{
		start(motor, 0.1);
		if( int(ticks_since_init)%100==0 )
			std::cout << (1000-ticks_since_init)/100 << std::endl;
		return;
	}

	stand( angleVector );

	walknet->stepWalknet( sensor );

	moveRobot( motor, angleVector );

	if( int( ticks_since_init )%200 == 0 )
	{
		outputData( sensor, motor );
	}
}

void DungBotEmptyController::step( const sensor* sensor, int sensorNumber, motor* motor, int motorNumber )
{
	stepNoLearning(sensor, sensorNumber, motor, motorNumber);
}

void DungBotEmptyController::init( int sensorNumber, int motorNumber, RandGen* randGen )
{
	assert( motorNumber >= DungBotMotorSensor::DUNGBOT_MOTOR_MAX );
	assert( sensorNumber >= DungBotMotorSensor::DUNGBOT_SENSOR_MAX );

	nSensors = sensorNumber;
	nMotors = motorNumber;

	if( writeOutput )
	{
		outputFile.open( "output.csv", std::ios::app );
	}

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

void DungBotEmptyController::stand( double* forceVector )
{
	double coxa_pos[3] 	= {0.0, -0.2, -0.5}; // Front, Middle, Rear
	double femur_pos[3]	= {0.2, 0.0, 0.5};
	double tibia_pos[3]	= {-0.6, -0.9, -0.4};

	for( int i = 0; i < DungBotMotorSensor::DUNGBOT_MOTOR_MAX; i++ )
	{
		if( i >= 0 && i < 6)		// COXA
		{
			forceVector[i] = coxa_pos[i%3];
		}
		if( i >= 6 && i < 12 ) 		// FEMUR
		{
			forceVector[i] = femur_pos[i%3];
		}
		if( i >= 12 && i < 18 ) 	// TIBIA
		{
			forceVector[i] = tibia_pos[i%3];
			//forceVector[i] = 10 + sin( 0.01 * ticks_since_init ); // Test with Sine
		}
	}
}

void DungBotEmptyController::start( motor* motor, double vel ) {

	for( int i = 0; i < DungBotMotorSensor::DUNGBOT_MOTOR_MAX; i++ )
	{
		if( i >= 0 && i < 6)		// COXA
		{
			motor[i] = -vel;
		}
		if( i >= 6 && i < 12 ) 		// FEMUR
		{
			motor[i] = vel;
		}
		if( i >= 12 && i < 18 ) 	// TIBIA
		{
			motor[i] = -vel;
		}
	}
}

void DungBotEmptyController::moveRobot( motor* motor, double* forceVector ) {

	for( int i = 0; i < DungBotMotorSensor::DUNGBOT_MOTOR_MAX; i++ )
	{
		motor[i] = forceVector[i];
	}
}

void DungBotEmptyController::outputData( const sensor* sensor, motor* motor )
{
	std::vector<double> sensorOutput;
	std::vector<double> motorInput;

	std::cout << "------------------------------------------------------------------" << std::endl;
	std::cout << "Ticks: " << ticks_since_init << std::endl;
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

	if( writeOutput )
	{
		collectData( motorInput, sensorOutput );
	}
}

void DungBotEmptyController::collectData( std::vector<double> motorInput, std::vector<double> sensorOutput )
{
	if( outputFile.is_open() && writeOutput)
	{
		outputFile << ticks_since_init;
		for( unsigned int i = 0; i < motorInput.size(); i++ )
		{
			outputFile << "," << motorInput[i] << "," << sensorOutput[i];
		}
		outputFile << std::endl;
	}
	else
	{
		std::cout << "DungBot controller: File not open" << std::endl;
	}
}
