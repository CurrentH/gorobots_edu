/*
 * rollnetSeparateLeg.h
 *
 *  Created on: Aug 3, 2016
 *      Author: mat
 */

#ifndef GOROBOTS_EDU_PRACTICES_DUNGBOT_ROLLNETSEPARATELEG_H_
#define GOROBOTS_EDU_PRACTICES_DUNGBOT_ROLLNETSEPARATELEG_H_

// #include <ode/ode.h>
#include <ode-dbl/ode.h>
#include <selforg/abstractcontroller.h>

// Extra includes
#include <vector>
#include <iostream>
#include <string>
#include <math.h>
#include <time.h>


class rollnetSeparateLeg
{
	public:
	//	Public attributes

	public:
	//	Public methods
	rollnetSeparateLeg( );
	rollnetSeparateLeg( int legNum );
	virtual ~rollnetSeparateLeg( void );
	void stepRollnetSeprateLeg( const sensor* sensor, std::vector<double> &, std::vector<double> &  );
	//	Used by the walknet to make the control laws for the legs.
	void extractSensor( const sensor* sensor, int leg, std::vector<double> & );
	bool atPosition( std::vector<double> , double);
	bool atAngle( double targetPos, int legPartNum, double deadband );
	bool getGroundContact();

	protected:
	//	Protected attributes

	protected:
	//	Protected methods,.

	private:
	//	Private attributes
	int legNum;
	std::vector<double> localSensorArray;

	private:
	//	Private methods

};




#endif /* GOROBOTS_EDU_PRACTICES_DUNGBOT_ROLLNETSEPARATELEG_H_ */
