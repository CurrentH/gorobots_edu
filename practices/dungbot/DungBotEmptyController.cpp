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

using namespace matrix;

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
	if(writeOutput){
	outputFile.close();
	}
}

void DungBotEmptyController::stepNoLearning( const sensor* sensor, int sensorNumber, motor* motor, int motorNumber )
{
	assert( motorNumber >= DungBotMotorSensor::DUNGBOT_MOTOR_MAX );
	assert( sensorNumber >= DungBotMotorSensor::DUNGBOT_SENSOR_MAX );

	//	Update internal time
	ticks_since_init++;
	double output_temp = sin( 0.001 * ticks_since_init );

	//stand(motor);
	output_temp += 10;

	motor[12] = output_temp;

	for( int i = 0; i < DungBotMotorSensor::DUNGBOT_MOTOR_MAX; i++ )
	{
		if( i >= 0 && i < 6)		// COXA
		{
			//	motor[i] = out	put_temp;
		}
		if( i >= 6 && i < 12 ) 		// FEMUR
		{
			//motor[i] = output_temp;
		}
		if( i >= 12 && i < 18 ) 	// TIBIA
		{
			//motor[i] = output_temp;
		}
	}

	//startPos(motor);

	if( int( ticks_since_init )%200 == 0 )
	{
		outputData( sensor, motor );
	}
}

void DungBotEmptyController::step( const sensor* sensor, int sensorNumber, motor* motor, int motorNumber )
{
	stepNoLearning(sensor, sensorNumber, motor, motorNumber);
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
	// Output
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

void DungBotEmptyController::stand( motor* motor )
{

}














/*//	Coxa
	state[0][0] = state[3][0] =  0.7; // Front - Movement towards head?
	state[0][1] = state[3][1] = -0.7; // Front - Movement away from head?

	state[1][0] = state[4][0] =  0.7; // Middle - Movement towards head?
	state[1][1] = state[4][1] = -0.7; // Middle - Movement away from head?

	state[2][0] = state[5][0] =  0.7; // Rear - Movement towards head?
	state[2][1] = state[5][1] = -0.7; // Rear - Movement away from head?

	//	Femur
	state[6][0] = state[9][0] =  0.7;	// Front - Upward
	state[6][1] = state[9][1] = -0.7;	// Front - Downward

	state[7][0] = state[10][0] =  0.7;	// Middle - Upward
	state[7][1] = state[10][1] = -0.7;	// Middle - Downward

	state[8][0] = state[11][0] =  0.7;	// Rear - Upward
	state[8][1] = state[11][1] = -0.7;	// Rear - Downward

	//	Tibia
	state[12][0] = state[15][0] = 0.7;	// Front - Upward
	state[12][1] = state[15][1] = -0.7; // Front - Downward

	state[13][0] = state[16][0] = 0.7;	// Front - Upward
	state[13][1] = state[16][1] = -0.7; // Front - Downward

	state[14][0] = state[17][0] = 0.7;	// Front - Upward
	state[15][1] = state[17][1] = -0.7; // Front - Downward*/

/*for( int i = 0; i < DungBotMotorSensor::DUNGBOT_MOTOR_MAX; i++ )
	{
		if( i >= 0 && i < 6)
		{
			if( int(ticks_since_init)%2000 == 0 )
			{
				if( i%2 == 0 )
				{
					motor[i] = -movement;//state[i][0];
				}
				else
				{
					motor[i] = -movement;//state[i][0];
				}

			}
			else if( int(ticks_since_init)%1000 == 0 )
			{true
				if( i%2 == 0 )
				{
					motor[i] = -movement;//state[i][1];
				}
				else
				{
					motor[i] = -movement;//state[i][1];
				}
			}
		}
		if( i >= 6 && i < 12 ) //FEMUR
		{
			if( int(ticks_since_init)%2000 == 0 )
			{
				motor[i] = movement;//state[i][1];
			}
			else if( int(ticks_since_init)%1000 == 0 )
			{
				motor[i] = movement;//state[i][1];
			}
		}
		if( i >= 12 && i < 18 )
		{
			if( int(ticks_since_init)%2000 == 0 )
			{
				motor[i] = movement;//state[i][1];
			}
			else if( int(ticks_since_init)%1000 == 0 )
			{
				motor[i] = movement;//state[i][1];
			}
		}


		if( writeOutput )
		{
			motorInput.push_back( motor[i] );
			sensorOutput.push_back( sensor[i] );
		}
	}*/
