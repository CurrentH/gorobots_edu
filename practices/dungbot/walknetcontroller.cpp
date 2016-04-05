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
	walknetSeparateLeg legFL(0);
	walknetSeparateLeg legML(1);
	walknetSeparateLeg legRL(2);
	walknetSeparateLeg legFR(3);
	walknetSeparateLeg legMR(4);
	walknetSeparateLeg legRR(5);

	separateLegs[0] = legFL;
	separateLegs[1] = legML;
	separateLegs[2] = legRL;
	separateLegs[3] = legFR;
	separateLegs[4] = legMR;
	separateLegs[5] = legRR;
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
	double *tmpArr;
	for( int i = 0; i < 6; i++ )
	{
		tmpArr = separateLegs[i].stepWalknetSeprateLeg( sensor );
		//std::cout << tmpArr[1] << std::endl;
		for( int j = 0; j < 3; j++ )
		{
			//std::cout << tmpArr[j] << std::endl;
			angleVector[i][j] = tmpArr[j];
			//std::cout << angleVector[i][j] << std::endl;
		}
	}

}

