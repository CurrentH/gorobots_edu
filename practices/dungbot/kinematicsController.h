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

#ifndef __ODE_ROBOTS_ROBOTS_DUNGBOTKINEMATICSCONTROLLER_H_
#define __ODE_ROBOTS_ROBOTS_DUNGBOTKINEMATICSCONTROLLER_H_

// #include <ode/ode.h>
#include <ode-dbl/ode.h>
#include <selforg/abstractcontroller.h>

// Extra includes
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>

class kinematicsController
{
	public:
	//	Public attributes

	public:
	//	Public methods
	kinematicsController( void );
	virtual ~kinematicsController( void );

	void stepKinematicsController( const sensor* sensor, std::vector<std::vector<double>> & );

	protected:
	//	Protected attributes

	protected:
	//	Protected methods

	private:
	//	Private attributes

	//	std::vector<std::vector<std::vector<double>>> positionList;
	//	First vector is the legs.
	//	Second vector is the positions.
	//	Third vector is the joint positions.

	std::vector<std::vector<double>> positionListL0;
	std::vector<std::vector<double>> positionListL1;
	std::vector<std::vector<double>> positionListL2;
	std::vector<std::vector<double>> positionListR0;
	std::vector<std::vector<double>> positionListR1;
	std::vector<std::vector<double>> positionListR2;

	std::vector<int> targetPositionPointer;

	int counter = 0;

	private:
	//	Private methods
	void loadPositionVectors( void );
	void legPositionControl( const sensor*, std::vector<std::vector<double>> &, int );
	bool legAtPosition( std::vector<double>, double, int );




};

#endif


