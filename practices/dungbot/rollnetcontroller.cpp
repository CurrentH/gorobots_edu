/*
 * rollnetcontroller.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: mat
 */
#include "rollnetcontroller.h"

rollnetcontroller::rollnetcontroller( void )
{
	rollnetSeparateLeg legFL(0);
	rollnetSeparateLeg legML(1);
	rollnetSeparateLeg legRL(2);
	rollnetSeparateLeg legFR(3);
	rollnetSeparateLeg legMR(4);
	rollnetSeparateLeg legRR(5);

	separateLegs.push_back(legFL);
	separateLegs.push_back(legML);
	separateLegs.push_back(legRL);
	separateLegs.push_back(legFR);
	separateLegs.push_back(legMR);
	separateLegs.push_back(legRR);
}

rollnetcontroller::~rollnetcontroller( void )
{
	for( int i = 0; i <= 6; i++ )
	{
		separateLegs[i].~rollnetSeparateLeg();
	}
}


void rollnetcontroller::stepRollnet( const sensor* sensor, std::vector<std::vector<double>> &angleVector, std::vector<std::vector<double>> &velocityVector  )
{
	for( int i = 0; i < 6; i++ )
	{
		separateLegs[i].stepRollnetSeprateLeg( sensor, angleVector[i], velocityVector[i] );
	}
}
