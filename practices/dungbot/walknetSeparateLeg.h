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

#ifndef __ODE_ROBOTS_ROBOTS_DUNGBOTWALKNETSEPARATELEG_H_
#define __ODE_ROBOTS_ROBOTS_DUNGBOTWALKNETSEPARATELEG_H_

// #include <ode/ode.h>
#include <ode-dbl/ode.h>
#include <selforg/abstractcontroller.h>

// Extra includes
#include <vector>
#include <iostream>
#include <string>
#include <math.h>


class walknetSeparateLeg
{
	public:
	//	Public attributes

	public:
	//	Public methods
	walknetSeparateLeg( );
	walknetSeparateLeg( int legNum );
	virtual ~walknetSeparateLeg( void );
	double* stepWalknetSeprateLeg( const sensor* sensor );

	protected:
	//	Protected attributes

	protected:
	//	Protected methods

	private:
	//	Private attributes
	int legNum;
	double PEP[3] = {0};
	double AEP[3] = {0};
	bool initSwing = true;
	bool initStance = true;

	private:
	//	Private methods
	double selectorNet( const sensor* sensor );
	double stanceNet( const sensor* sensor );
	double swingNet( const sensor* sensor );
	bool atPosition( double targetPos[] );

};

#endif
