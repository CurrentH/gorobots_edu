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

void walknetcontroller::stepWalknet( const sensor* sensor, std::vector<std::vector<double>> &angleVector  )
{
	for( int i = 0; i < 6; i++ )
	{
		separateLegs[i].stepWalknetSeprateLeg( sensor, angleVector[i] );
	}
}
void walknetcontroller::coordinatingInfluences( void )
{
	/**
	 * 		Do the coordination influences here.
	 * 		Figure out what the legs need to do, and send the
	 * 		signals to the separate legs.
	 */
	for( int i = 0; i < 6; i++ )
	{
		switch (i) {
			case 0://	Front left
				/*Rule1*/ if( separateLegs[i+1].getPhase() == true )
							{ separateLegs[i].setRule(1, true); } else
							{ separateLegs[i].setRule(1, false); }
				/*Rule2*/ if( separateLegs[i+1].atPosition( separateLegs[i+1].getAEP() ) &&
								separateLegs[i+1].getGroundContact() )
							{ separateLegs[i].setRule(2, true); } else
							{ separateLegs[i].setRule(2, false); }
				break;
			case 1://	Middle left
				/*Rule1*/ if( separateLegs[i+1].getPhase() == true )
							{ separateLegs[i].setRule(1, true); } else
							{ separateLegs[i].setRule(1, false); }
				/*Rule2*/ if( separateLegs[i+1].atPosition( separateLegs[i+1].getAEP() ) &&
								separateLegs[i+1].getGroundContact() )
							{ separateLegs[i].setRule(2, true); } else
							{ separateLegs[i].setRule(2, false); }
				/*Rule3*/ if( separateLegs[i-1].atPosition( separateLegs[i+1].getPEP() ) )
							{ separateLegs[i+3].setRule(3, true); } else
							{ separateLegs[i+3].setRule(3, false); }
				break;
			case 2://	Rear left
				/*Rule3*/ if( separateLegs[i-1].atPosition( separateLegs[i+1].getPEP() ) )
							{ separateLegs[i+3].setRule(3, true); } else
							{ separateLegs[i+3].setRule(3, false); }
				break;
			case 3://	Front Right
				/*Rule1*/ if( separateLegs[i+1].getPhase() == true )
							{ separateLegs[i].setRule(1, true); } else
							{ separateLegs[i].setRule(1, false); }
				/*Rule2*/ if( separateLegs[i+1].atPosition( separateLegs[i+1].getAEP() ) &&
								separateLegs[i+1].getGroundContact() )
							{ separateLegs[i].setRule(2, true); } else
							{ separateLegs[i].setRule(2, false); }
				break;
			case 4://	Middle Right
				/*Rule1*/ if( separateLegs[i+1].getPhase() == true )
							{ separateLegs[i].setRule(1, true); } else
							{ separateLegs[i].setRule(1, false); }
				/*Rule2*/ if( separateLegs[i+1].atPosition( separateLegs[i+1].getAEP() ) &&
								separateLegs[i+1].getGroundContact() )
							{ separateLegs[i].setRule(2, true); } else
							{ separateLegs[i].setRule(2, false); }
				/*Rule3*/ if( separateLegs[i-1].atPosition( separateLegs[i+1].getPEP() ) )
							{ separateLegs[i].setRule(3, true); } else
							{ separateLegs[i].setRule(3, false); }
				break;
			case 5://	Rear Right
				/*Rule3*/ if( separateLegs[i-1].atPosition( separateLegs[i+1].getPEP() ) )
							{ separateLegs[i].setRule(3, true); } else
							{ separateLegs[i].setRule(3, false); }
				break;
			default:
				break;
		}
	}

}
