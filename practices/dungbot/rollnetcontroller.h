/*
 * rollnetcontroller.h
 *
 *  Created on: Aug 3, 2016
 *      Author: mat
 */

#ifndef GOROBOTS_EDU_PRACTICES_DUNGBOT_ROLLNETCONTROLLER_H_
#define GOROBOTS_EDU_PRACTICES_DUNGBOT_ROLLNETCONTROLLER_H_

// #include <ode/ode.h>
#include <ode-dbl/ode.h>
#include <selforg/abstractcontroller.h>

// Extra includes
#include <vector>
#include <iostream>
#include <string>
#include "rollnetSeparateLeg.h"

class rollnetcontroller
{
	public:
	//	Public attributes

	public:
	//	Public methods
	rollnetcontroller( void );
	virtual ~rollnetcontroller( void );

	void stepRollnet( const sensor* sensor, std::vector<std::vector<double>> &, std::vector<std::vector<double>> & );

	protected:
	//	Protected attributes

	protected:
	//	Protected methods

	private:
	//	Private attributes
	std::vector<rollnetSeparateLeg> separateLegs;

	private:
	//	Private methods

};

#endif /* GOROBOTS_EDU_PRACTICES_DUNGBOT_ROLLNETCONTROLLER_H_ */
