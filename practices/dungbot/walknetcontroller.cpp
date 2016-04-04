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

#include "walknetcontroller.h"

walknetcontroller::walknetcontroller( void )
{
	walknetSeparateLeg leg1;
	walknetSeparateLeg leg2;
	walknetSeparateLeg leg3;
	walknetSeparateLeg leg4;
	walknetSeparateLeg leg5;
	walknetSeparateLeg leg6;

	separateLegs[0] = leg1;
	separateLegs[1] = leg2;
	separateLegs[2] = leg3;
	separateLegs[3] = leg4;
	separateLegs[4] = leg5;
	separateLegs[5] = leg6;
}

walknetcontroller::~walknetcontroller( void )
{
	for( int i = 0; i <= 6; i++ )
	{
		separateLegs[i].~walknetSeparateLeg();
	}
}

void walknetcontroller::stepWalknet( const sensor* sensor, double angleVector[][3]  )
{
	double* tmp;

	for( int i = 0; i <= 5; i++ )
	{
		tmp = separateLegs[i].stepWalknetSeprateLeg( sensor );

		for( int j = 0; j < 3; j++ )
		{
			angleVector[i][j] = 1.0;
		}
	}

}

