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

	separateLegs.push_back(legFL);
	separateLegs.push_back(legML);
	separateLegs.push_back(legRL);
	separateLegs.push_back(legFR);
	separateLegs.push_back(legMR);
	separateLegs.push_back(legRR);
}

walknetcontroller::~walknetcontroller( void )
{
	for( int i = 0; i <= 6; i++ )
	{
		separateLegs[i].~walknetSeparateLeg();
	}
}

std::vector<std::vector<double>> walknetcontroller::stepWalknet( const sensor* sensor, std::vector<std::vector<double>> angleVector  )
{

	std::vector<double> tmpArr;

	for( int i = 0; i < 6; i++ )
	{
		tmpArr = separateLegs[i].stepWalknetSeprateLeg( sensor );

		for( int j = 0; j < 3; j++ )
		{
			angleVector[i][j] = tmpArr[j];
		}
	}

	return angleVector;

}

