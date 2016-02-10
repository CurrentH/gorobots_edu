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

#ifndef ODE_ROBOTS_ROBOTS_DUNGBOTPHASECONTROLLER_H_
#define ODE_ROBOTS_ROBOTS_DUNGBOTPHASECONTROLLER_H_

#include <selforg/abstractcontroller.h>
#include <ode_robots/joint.h>
#include <ode_robots/contactsensor.h>
#include <iostream>
#include <fstream>
#include <map>

class DungBotEmptyController : public AbstractController {
public:
	DungBotEmptyController( const std::string& name );
	virtual ~DungBotEmptyController();

	virtual void init( int sensornumber, int motornumber, RandGen* randGen = 0 )  override;
	virtual int getSensorNumber( void ) const override;
	virtual int getMotorNumber( void ) const override;

	virtual void step( const sensor* sensors, int sensornumber, motor* motors, int motornumber ) override;
	virtual void stepNoLearning( const sensor* , int number_sensors, motor* , int number_motors ) override;

	virtual bool store( FILE* f ) const override;
	virtual bool restore( FILE* f ) override;

protected:
	double nSensors;
	double nMotors;
	bool initialised;
	long ticks_since_init;
	double speedSetpoint;
	double phaseSetpoint;
};


#endif /* ODE_ROBOTS_ROBOTS_DUNGBOTPHASECONTROLLER_H_ */
