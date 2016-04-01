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

// Extra includes
#include <vector>
#include <iostream>
#include <string>

namespace lpzrobots
{
	class walknetSeparateLeg
	{
		public:
		//	Public attributes

		public:
		//	Public methods
		double stepWalknetSeprateLeg( const sensor* sensor ); //returns double array[3]
		walknetSeparateLeg( void );
		virtual ~walknetSeparateLeg( void );

		protected:
		//	Protected attributes

		protected:
		//	Protected methods

		private:
		//	Private attributes
		void selectorNet();
		void stanceNet();
		void swingNet();

		private:
		//	Private methods



	};
}

#endif
