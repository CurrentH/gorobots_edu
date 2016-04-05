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
	double* getAEP( void );
	double* getPEP( void );

	protected:
	//	Protected attributes

	protected:
	//	Protected methods

	private:
	//	Private attributes
	int legNum;

	bool RSunit = false;	//	Return Stroke unit (swing movement)
	bool PSunit = false;	//	Power Stroke unit (stance movement)
	bool GCunit = false;	//	Ground Contact
	bool PEPunit = false;	//	Boolean value if the leg is in the PEP position.

	private:
	//	Private methods
	double selectorNet( const sensor* sensor );
	double stanceNet( const sensor* sensor );
	double swingNet( const sensor* sensor );
	double* extractSensor( const sensor* sensor, int leg );
	bool checkPEP();
	void setAEP( double );
	void setPEP( double );

};

#endif
