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
using namespace std;

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

	angleVector.assign( 6 , vector<double>( 3 , 0 ) );

	walknet = new walknetcontroller();
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

	if( int(ticks_since_init) < 600 ) // Start after 1000 ticks in init position. Please drop before times run out
	{
		stand( angleVector );
		moveRobot( motor, angleVector );
		if( int(ticks_since_init)%100==0 )
			std::cout << (600-ticks_since_init)/100 << std::endl;
		return;
	}

	// ----------------------------------
	//start(motor, 1.0);
	//stand( angleVector );
	//walknet->stepWalknet( sensor, angleVector );
	walknet->stepWalknet( sensor, angleVector );
	moveRobot( motor, angleVector );
	// ----------------------------------

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

void DungBotEmptyController::stand( std::vector<std::vector<double>> &angleVector )
{
	double coxa_pos[3] 	= {0.0, -0.2, -0.5}; // Front, Middle, Rear
	double femur_pos[3]	= {0.2, 0.05, 0.5};
	double tibia_pos[3]	= {0.6, 0.9, 0.4};

	for( int i = 0; i < 6; i++ )
	{
		angleVector[i][0] = coxa_pos[i%3];
		angleVector[i][1] = femur_pos[i%3];
		angleVector[i][2] = -tibia_pos[i%3];
	}
}

void DungBotEmptyController::start( motor* motor, double vel ) {

	for( int i = 0; i < DungBotMotorSensor::DUNGBOT_MOTOR_MAX; i++ )
	{
		if( i >= 0 && i < 6)		// COXA
		{
			motor[i] = -vel;//*sin(ticks_since_init*0.001);
		}
		if( i >= 6 && i < 12 ) 		// FEMUR
		{
			motor[i] = vel;//*sin(ticks_since_init*0.001);
		}
		if( i >= 12 && i < 18 ) 	// TIBIA
		{
			motor[i] = -vel;//*sin(ticks_since_init*0.001);
		}
	}
}

void DungBotEmptyController::moveRobot( motor* motor, std::vector<std::vector<double>> angleVector ) {

	for( int i = 0; i < 3; i++ )
	{
		for(int j = 0; j < 6; j ++)
		{
			motor[i*6+j] = angleVector[j][i];
		}
	}

}

void DungBotEmptyController::outputData( const sensor* sensor, motor* motor )
{
	// TODO: Make function that write (!) at the end if there is a miss match between motor[] and sensor[]

	std::vector<double> sensorOutput;
	std::vector<double> motorInput;
	std::vector<bool> legPhase;
	walknet->getPhase( legPhase );

    cout.setf(ios::fixed, ios::floatfield);
    cout.precision(2);
/*
	std::cout << "------------------------------------------------------------------" << std::endl;
	std::cout << "     \t   \tCurrent\t     \t\t    \tDesired\t    \t\tPhase\t    " << std::endl;
	std::cout << "     \tCoxa\tFemur\tTibia\t\tCoxa\tFemur\tTibia" << std::endl;
	std::cout << "Leg0:\t"<< sensor[0] <<"\t"<< sensor[6] <<"\t"<< sensor[12] <<"     \t"<< motor[0] <<"\t"<< motor[6] <<"\t"<< motor[12] <<"    \t" << legPhase[0] << std::endl;
	std::cout << "Leg1:\t"<< sensor[1] <<"\t"<< sensor[7] <<"\t"<< sensor[13] <<"     \t"<< motor[1] <<"\t"<< motor[7] <<"\t"<< motor[13] <<"    \t" << legPhase[1]<< std::endl;
	std::cout << "Leg2:\t"<< sensor[2] <<"\t"<< sensor[8] <<"\t"<< sensor[14] <<"     \t"<< motor[2] <<"\t"<< motor[8] <<"\t"<< motor[14] <<"    \t" << legPhase[2]<< std::endl;
	std::cout << "Leg3:\t"<< sensor[3] <<"\t"<< sensor[9] <<"\t"<< sensor[15] <<"     \t"<< motor[3] <<"\t"<< motor[9] <<"\t"<< motor[15] <<"    \t" << legPhase[3]<< std::endl;
	std::cout << "Leg4:\t"<< sensor[4] <<"\t"<< sensor[10] <<"\t"<< sensor[16] <<"     \t"<< motor[4] <<"\t"<< motor[10] <<"\t"<< motor[16] <<"    \t" << legPhase[4]<< std::endl;
	std::cout << "Leg5:\t"<< sensor[5] <<"\t"<< sensor[11] <<"\t"<< sensor[17] <<"     \t"<< motor[5] <<"\t"<< motor[11] <<"\t"<< motor[17] <<"    \t" << legPhase[5] << std::endl;
	std::cout << "\nTicks: " << ticks_since_init << std::endl;
	std::cout << "Contact: " << sensor[25] << ", "<< sensor[31] << ", "<< sensor[37] << ", "<<sensor[43] << ", " << sensor[49] << ", " << sensor[55] << std::endl;
	std::cout << "------------------------------------------------------------------" << std::endl;
*/

//	Print sensor values for the stumps.
    /*
    std::cout << sensor[DungBotMotorSensor::L0_s0] << " "<< sensor[DungBotMotorSensor::L1_s0] << " "<< sensor[DungBotMotorSensor::L2_s0] << " "
    		<< sensor[DungBotMotorSensor::R0_s0] << " "<< sensor[DungBotMotorSensor::R1_s0] << " "<< sensor[DungBotMotorSensor::R2_s0] << std::endl;
*/
//	Print the phase of each leg

    for( int i = 0; i < 6; i++ )
    {
    	std::cout << legPhase[i] << " ";
    }
    std::cout << std::endl;

	if( writeOutput )
	{
		collectData( sensor, motor );
	}
}

void DungBotEmptyController::collectData( const sensor* sensor, motor* motor )
{
	if( outputFile.is_open() && writeOutput)
	{
		//	Print all motor and sensor values here.
		/*
		outputFile << ticks_since_init;
		for( unsigned int i = 0; i < motorInput.size(); i++ )
		{
			outputFile << "," << motor[i] << "," << sensor[i];
		}
		outputFile << std::endl;
		*/

		//	Print the contact sensors for the stump.
		outputFile << ticks_since_init << "," << sensor[DungBotMotorSensor::L0_s0] << "," << sensor[DungBotMotorSensor::L1_s0] << "," << sensor[DungBotMotorSensor::L2_s0] << ","
		    		<< sensor[DungBotMotorSensor::R0_s0] << "," << sensor[DungBotMotorSensor::R1_s0] << "," << sensor[DungBotMotorSensor::R2_s0];
		outputFile << std::endl;
	}
	else
	{
		std::cout << "DungBot controller: File not open" << std::endl;
	}
}
