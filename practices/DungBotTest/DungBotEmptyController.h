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

#ifndef ODE_ROBOTS_ROBOTS_DUNGBOTCONTROLLER_H_
#define ODE_ROBOTS_ROBOTS_DUNGBOTCONTROLLER_H_

#include "DungBotSensorMotorDefinition.h"

#include <selforg/abstractcontroller.h>
#include <ode_robots/joint.h>
#include <ode_robots/contactsensor.h>
#include <assert.h>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <cmath>

class DungBotEmptyController : public AbstractController
{
	public:
		DungBotEmptyController( const std::string& name );
		virtual ~DungBotEmptyController();

		virtual void init( int sensornumber, int motornumber, RandGen* randGen = 0 )  override;
		virtual int getSensorNumber( void ) const override;
		virtual int getMotorNumber( void ) const override;

		virtual void step( const sensor*, int, motor*, int ) override;
		virtual void stepNoLearning( const sensor* , int, motor*, int ) override;

		virtual bool store( FILE* f ) const override;
		virtual bool restore( FILE* f ) override;

	protected:
		double nSensors;
		double nMotors;
		bool initialised;
		double ticks_since_init;
		std::ofstream outputFile;

	private:
		void outputData( const sensor*, motor* );
		void collectData( std::vector<double>, std::vector<double> );
		void start( motor* motor, double );

		void stand(double* arr );
		void moveRobot( motor* motor, double* arr);

		bool writeOutput = false;
		double angleVector[17] = {0};

		double state[DungBotMotorSensor::DUNGBOT_MOTOR_MAX][2];
};


#endif /* ODE_ROBOTS_ROBOTS_DUNGBOTCONTROLLER_H_ */
