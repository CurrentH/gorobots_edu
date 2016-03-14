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
DungBotEmptyController::DungBotEmptyController( const std::string& name )
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

double DungBotEmptyController::PID( double targetPosition, double actualPosition, int motorNumber, double deltaT )
{
	//TODO MAKE ALL THE MOTOR TO servoVel
	//TODO MAKE A DEADBAND FOR WHEN ACCEPT A POSITION AND TURN OF THE MOTOR

	double Kp, Ki, Kd; 												//TODO find values for these (MATLAB or MATHEMATICA)
	double error[17], integral[17], derivative[17], errorlast[17];  //TODO these needs to be saved somewhere else, so that they keep their value.
	double maxOutput = 100; 										//TODO find a value for this (it should just be maxVel?)

	errorlast[motorNumber] = error[motorNumber];
	error[motorNumber] = targetPosition - actualPosition; //Target position - actual position (Is the sensor still position?)
	double output = (Kp * error[motorNumber]) + (Ki * integral[motorNumber]) + (Kd * derivative[motorNumber]);


	// First to if's is for saturation
	if (output > maxOutput)
		output = maxOutput;
	else if (output < -maxOutput)
		output = -maxOutput;
	else
	     integral[motorNumber] += (error[motorNumber] * deltaT);	// Else we just calculate the integral

	derivative[motorNumber] = (error[motorNumber] - errorlast[motorNumber])/deltaT;

	return output;
}

void DungBotEmptyController::stepNoLearning( const sensor* sensor, int sensorNumber, motor* motor, int motorNumber )
{
	assert( motorNumber >= DungBotMotorSensor::DUNGBOT_MOTOR_MAX );
	assert( sensorNumber >= DungBotMotorSensor::DUNGBOT_SENSOR_MAX );

	//	Update internal time
		ticks_since_init++;
		std::vector<double> sensorOutput;
		std::vector<double> motorInput;

		for( int i = 0; i < DungBotMotorSensor::DUNGBOT_MOTOR_MAX; i++ )
		{

			motor[i] = -0.5;
			//motor[i] = sin( 0.01 * ticks_since_init );

			if(writeOutput){
			motorInput.push_back( motor[i] );
			sensorOutput.push_back( sensor[i] );
			}
		}

		if( int(ticks_since_init)%200 == 0 )
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

			if(writeOutput)
			collectData( motorInput, sensorOutput );
		}

}

void DungBotEmptyController::step( const sensor* sensor, int sensorNumber, motor* motor, int motorNumber )
{
	stepNoLearning(sensor, sensorNumber, motor, motorNumber);
}

void DungBotEmptyController::collectData( std::vector<double> motorInput, std::vector<double> sensorOutput )
{
	// Output
	if( outputFile.is_open() && writeOutput)
	{
		outputFile << ticks_since_init;
		for( int i = 0; i < motorInput.size(); i++ )
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

	if(writeOutput){
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
