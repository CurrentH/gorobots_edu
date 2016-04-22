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
	bool startSwing = false;
	bool startStance = false;

	int swingState2; // TODO make get and set
	int stanceState2; // TODO make get and set
	enum swingState2 { TO_PEP_SWING, LIFT, LOWER, TO_MID_SWING, TO_AEP_SWING, SWING2_DONE, FINAL_SWING_POS };
	enum stanceState2 { TO_PEP_STANCE, TO_MID_STANCE, TO_AEP_STANCE, STANCE2_DONE };

	public:
	//	Public methods
	walknetSeparateLeg( );
	walknetSeparateLeg( int legNum );
	virtual ~walknetSeparateLeg( void );
	void stepWalknetSeprateLeg( const sensor* sensor, std::vector<double> &  );
	//	Used by the walknet to make the control laws for the legs.
	void extractSensor( const sensor* sensor, int leg, std::vector<double> & );
	void setAEP( std::vector<double> & );
	void setPEP( double );
	void setRule( int, bool );
	void getAEP( std::vector<double> & );
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
	std::vector<double> maxAEP;
	std::vector<double> localSensorArray;
	std::vector<bool> coordinationRules;

	int swingState; // TODO make get and set
	int stanceState; // TODO make get and set
	enum swingState { SET_SWING_HEIGHT, SWING_COXA, RAISE_HEIGHT, SET_STANCE_HEIGHT, LOWER_HEIGHT, SWING_DONE };
	enum stanceState { SWING_TO_PEP, GET_GC, STANCE_DONE };

	bool STANCE_REACHED	= false;
	bool SWING_REACHED	= false;

	bool RSunit 	= false;	//	Return Stroke unit (swing movement)
	bool PSunit		= false;	//	Power Stroke unit (stance movement)
	bool GCunit 	= false;	//	Ground Contact
	bool PEPunit 	= false;	//	Boolean value if the leg is in the PEP position.

	private:
	//	Private methods
	void selectorNet( const sensor* sensor, std::vector<double> & );
	void stanceNet1( const sensor* sensor, std::vector<double> & );
	void swingNet1( const sensor* sensor, std::vector<double> & );
	void swingNet2( const sensor* sensor, std::vector<double> & );
	void swingNet3( const sensor* sensor, std::vector<double> & );
	void swingNet4( const sensor* sensor, std::vector<double> & );
	double trajectory( double, int );


};

#endif
