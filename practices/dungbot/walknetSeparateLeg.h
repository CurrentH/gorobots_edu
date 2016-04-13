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
	bool startSwing = true;
	bool startStance = true;

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
	bool atPosition( std::vector<double> , double);
	bool atAngle( double targetPos, int legPartNum, double deadband );
	bool getPhase();
	bool getGroundContact();

	protected:
	//	Protected attributes

	protected:
	//	Protected methods,.

	private:
	//	Private attributes
	int legNum;
	bool phase = false;
	std::vector<double> PEP;
	std::vector<double> MID;
	std::vector<double> AEP;
	std::vector<double> localSensorArray;
	std::vector<bool> coordinationRules;

	enum swingState { SET_SWING_HEIGHT, SWING_COXA, RAISE_HEIGHT,
					  SET_STANCE_HEIGHT, LOWER_HEIGHT, SWING_DONE };
	int swingState; // TODO make get and set

	enum stanceState { SWING_TO_PEP, GET_GC, STANCE_DONE };
	int stanceState; // TODO make get and set

	bool STANCE_REACHED	= false;
	bool SWING_REACHED	= false;

	bool RSunit 	= false;	//	Return Stroke unit (swing movement)
	bool PSunit		= false;	//	Power Stroke unit (stance movement)
	bool GCunit 	= false;	//	Ground Contact
	bool PEPunit 	= false;	//	Boolean value if the leg is in the PEP position.

	private:
	//	Private methods
	void selectorNet( const sensor* sensor, std::vector<double> & );
	void stanceNet( const sensor* sensor, std::vector<double> & );
	void swingNet( const sensor* sensor, std::vector<double> & );

};

#endif
