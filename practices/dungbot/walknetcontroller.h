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

#ifndef __ODE_ROBOTS_ROBOTS_DUNGBOTWALKNETCONTROLLER_H_
#define __ODE_ROBOTS_ROBOTS_DUNGBOTWALKNETCONTROLLER_H_

// #include <ode/ode.h>
#include <ode-dbl/ode.h>

// Extra includes
#include <vector>
#include <iostream>
#include <string>

namespace lpzrobots
{
	class walknetcontroller
	{
		public:
		//	Public attributes

		public:
		//	Public methods
		walknetcontroller( void );
		virtual ~walknetcontroller( void );

		virtual void start( void );
		//For each step, call this to calculate and receive the new motor inputs.
		std::vector<double> stepWalknet( sensor* sensor );

		protected:
		//	Protected attributes

		protected:
		//	Protected methods

		private:
		//	Private attributes
		std::vector< walknetSeparateLeg > separateLegs;

		private:
		//	Private methods




	};
}

#endif
