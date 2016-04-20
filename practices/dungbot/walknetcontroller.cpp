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

void walknetcontroller::stepWalknetTripod( const sensor* sensor, std::vector<std::vector<double>> &angleVector )
{
	bool flag = true;

	for( int i = 0; i < 6; i++ )
	{
		if( separateLegs[i].startStance == true || separateLegs[i].startSwing == true )
		{
			flag = false;
		}


		std::cout << (separateLegs[i].startStance == true || separateLegs[i].startSwing == true) << " ";
	}
	if( switchFlag )
	{
		std::cout << " F: " << switchFlag << " SW ST SW ST SW ST " << std::endl;
	}
	else
	{
		std::cout << " F: " << switchFlag << " ST SW ST SW ST SW" << std::endl;
	}

	if( flag )
	{
		std::cout << "SwitchFlag changes from: " << switchFlag << " to: " << !switchFlag << std::endl;
		switchFlag = !switchFlag;

		if( switchFlag )
		{
			separateLegs[0].startSwing = true;
			separateLegs[2].startSwing = true;
			separateLegs[4].startSwing = true;
			separateLegs[1].startStance = true;
			separateLegs[3].startStance = true;
			separateLegs[5].startStance = true;
		}
		else
		{
			separateLegs[0].startStance = true;
			separateLegs[2].startStance = true;
			separateLegs[4].startStance = true;
			separateLegs[1].startSwing = true;
			separateLegs[3].startSwing = true;
			separateLegs[5].startSwing = true;
		}

	}

	for( int i = 0; i < 6; i++ )
	{
		separateLegs[i].stepWalknetSeprateLeg( sensor, angleVector[i] );
	}

}

void walknetcontroller::stepWalknet( const sensor* sensor, std::vector<std::vector<double>> &angleVector  )
{
	coordinatingInfluences();
	for( int i = 0; i < 6; i++ )
	{
		separateLegs[i].stepWalknetSeprateLeg( sensor, angleVector[i] );
		//std::cout << separateLegs[i].startSwing << " ";
	}
	//std::cout << std::endl;
}

void walknetcontroller::getPhase( std::vector<bool> &phaseVector )
{
	phaseVector.clear();
	for( int i = 0; i < 6; i++ )
	{
		phaseVector.push_back( separateLegs[i].getPhase() );
	}
}

void walknetcontroller::coordinateRule1( void ){
	for( int i = 0; i < 6; i++ )
	{
		switch(i){
			case 0:
			case 1:
			case 3:
			case 4:
				if( separateLegs[i+1].getPhase() == false ){
					separateLegs[i].setRule(0,true);
				}else{
					separateLegs[i].setRule(0,false);
				}
				break;
			case 2:
			case 5:
				break;
			default:
				std::cout << "Problem in walknet controller rule 1 for leg:" << i << std::endl;
				break;
		}
	}
}
void walknetcontroller::coordinateRule2( void ){
	for( int i = 0; i < 6; i++ )
	{
		switch(i){
			case 0:
			case 1:
				if( separateLegs[i+1].getPhase() == false || separateLegs[i+3].getPhase() == false ){
					separateLegs[i].setRule(1,true);
				}else{
					separateLegs[i].setRule(1,false);
				}
				break;
			case 2:
				if( separateLegs[i+3].getPhase() == false ){
					separateLegs[i].setRule(1,true);
				}else{
					separateLegs[i].setRule(1,false);
				}
				break;
			case 3:
			case 4:
				if( separateLegs[i+1].getPhase() == false || separateLegs[i-3].getPhase() == false ){
					separateLegs[i].setRule(1,true);
				}else{
					separateLegs[i].setRule(1,false);
				}
				break;
			case 5:
				if( separateLegs[i-3].getPhase() == false ){
					separateLegs[i].setRule(1,true);
				}else{
					separateLegs[i].setRule(1,false);
				}
				break;
			default:
				std::cout << "Problem in walknet controller rule 2 for leg:" << i << std::endl;
				break;
		}
	}
}
void walknetcontroller::coordinateRule3( void ){
	for( int i = 0; i < 6; i++ )
	{
		switch(i){
			case 0:
				if( separateLegs[i+3].getPhase() == false ){
					separateLegs[i].setRule(2,true);
				}else{
					separateLegs[i].setRule(2,false);
				}
				break;
			case 1:
			case 2:
				if( separateLegs[i-1].getPhase() == false || separateLegs[i+3].getPhase() == false ){
					separateLegs[i].setRule(2,true);
				}else{
					separateLegs[i].setRule(2,false);
				}
				break;
			case 3:
				if( separateLegs[i-3].getPhase() == false ){
					separateLegs[i].setRule(2,true);
				}else{
					separateLegs[i].setRule(2,false);
				}
				break;
			case 4:
			case 5:
				if( separateLegs[i-1].getPhase() == false || separateLegs[i-3].getPhase() == false ){
					separateLegs[i].setRule(2,true);
				}else{
					separateLegs[i].setRule(2,false);
				}
				break;
			default:
				std::cout << "Problem in walknet controller rule 3 for leg:" << i << std::endl;
				break;
		}
	}
}

void walknetcontroller::coordinatingInfluences( void )
{
	/**
	 * 		Do the coordination influences here.
	 * 		Figure out what the legs need to do, and send the
	 * 		signals to the separate legs.
	 */

	coordinateRule1();
	coordinateRule2();
	coordinateRule3();

}
