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
	void stepWalknetSeprateLeg( const sensor* sensor, std::vector<double> &  );
	//	Used by the walknet to make the control laws for the legs.
	void extractSensor( const sensor* sensor, int leg, std::vector<double> & );
	void setAEP( double );
	void setPEP( double );
	void setRule( int, bool );
	std::vector<double> getAEP( void );
	std::vector<double> getPEP( void );
	bool atPosition( std::vector<double> );
	bool getPhase();
	bool getGroundContact();

	protected:
	//	Protected attributes

	protected:
	//	Protected methods,.

	private:
	//	Private attributes
	int legNum;
	bool phase;
	std::vector<double> PEP;
	std::vector<double> AEP;
	std::vector<double> localSensorArray;
	std::vector<bool> coordinationRules;

	bool initStance = true;
	bool initSwing = true;
	bool stage2 = true;
	bool stage3 = true;
	bool stage4 = true;

	bool RSunit = false;	//	Return Stroke unit (swing movement)
	bool PSunit = false;	//	Power Stroke unit (stance movement)
	bool GCunit = false;	//	Ground Contact
	bool PEPunit = false;	//	Boolean value if the leg is in the PEP position.

	private:
	//	Private methods
	void selectorNet( const sensor* sensor, std::vector<double> & );
	void stanceNet( const sensor* sensor, std::vector<double> & );
	void swingNet( const sensor* sensor, std::vector<double> & );

};

#endif
