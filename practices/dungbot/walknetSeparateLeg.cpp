#include "walknetSeparateLeg.h"

walknetSeparateLeg::walknetSeparateLeg() {
}

walknetSeparateLeg::walknetSeparateLeg(int newlegNum) {
	legNum = newlegNum;
}

double* walknetSeparateLeg::stepWalknetSeprateLeg(const sensor* sensor) {

	//selectorNet() -> stanceNet() || swingNet() -> tragetoryGenerator();

	double viaAngle[3] = {1, 1, 1};
	return viaAngle;
}

walknetSeparateLeg::~walknetSeparateLeg(void) {
}

double walknetSeparateLeg::selectorNet(const sensor* sensor)
{
	double tmp[4] = extractSensor( sensor, legNum );
	GCunit = tmp[3];	//	Check if there is Ground Contact
	PEPunit = checkPEP();

	RSunit = RSunit + PEPunit - GCunit;
	PSunit = PSunit - PEPunit + GCunit;

	if( RSunit == true )
	{
		swingNet( sensor );
	}
	else if( PSunit == true )
	{
		stanceNet( sensor );
	}

	return 0.0;
}

double walknetSeparateLeg::stanceNet(const sensor* sensor) {
	return 0.0;
}

double walknetSeparateLeg::swingNet(const sensor* sensor) {
	return 0.0;
}

bool walknetSeparateLeg::checkPEP()
{
	bool check = false;
	double treshold;
	double distance;
	double PEP[3] = {0};
	double pos[3] = {0};

	distance = sqrt( (PEP[0]-pos[0])^2 + (PEP[1]-pos[1])^2 + (PEP[2]-pos[2])^2 );

	if( abs( distance ) < treshold )
	{
		check = true;
	}

	return check;
}

double* walknetSeparateLeg::extractSensor( const sensor* sensor, int leg )
{
	double extractedSensors[4] = {0};

	for( int i = 0; i < 3; i++ )
	{
		extractedSensors[ i ] = sensor[ leg + i*6 ];
	}

	for( int i = 0; i < 5; i++ )
	{
		if( sensor[25 + 5*leg + i] == true ){
			extractedSensors[4] = 1;
		}
	}

	return extractedSensors;
}
