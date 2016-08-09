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

#include "kinematicsController.h"

kinematicsController::kinematicsController(void) {
	std::cout << "CREATING KINEMATICS CONTROLLER" << std::endl;

	targetPositionPointer.assign( 6, 1 );

	std::cout << "LOADING POSITION VECTORS" << std::endl;

	loadPositionVectors();

	std::cout << "KinematicsController Constructor DONE" << std::endl;
}

kinematicsController::~kinematicsController(void) {
}

void kinematicsController::stepKinematicsController( const sensor* sensor, std::vector<std::vector<double>> &angleVector ) {
	/**
	 * 	For each step, we need to take a look at the position each leg is in now.
	 * 	And if it is close enough, send it to the next "location".
	 *
	 *	The cycle is fixed.
	 *
	 * 	The "atPosition" method in the bottom should be used.
	 *
	 *	We look at the sensor values for each leg. Decide if it is
	 *	close enough to the position, that we can send the leg
	 *	to the next position.
	 */

	std::cout << "*********************" << std::endl;

	std::cout << "COUNTER IS: " << counter << std::endl;

	for( int i = 0; i < 6; i++ ){
		legPositionControl( sensor, angleVector, 1 );
		break; //Remove
	}
/*
	for( int i = 0; i<6; i++ ){
		std::cout << targetPositionPointer[i] << " ";
	}
	std::cout << std::endl;
*/
	counter++;
	std::cout << "$$$$$$$$$$$$$$$$$$$$$" << std::endl;

}

void kinematicsController::legPositionControl( const sensor* sensor, std::vector<std::vector<double>> &angleVector, int legNum ) {
	/**
	 * 	We want to set the "arm" at some (x,y,z) coordinate.
	 * 	(0,0,0) is the place where the coxa and femur is connected.
	 *
	 * 	Will be 2D for a starter
	 *
	 * 	We want to give it a position. We know the current position.
	 * 	We then want to find the difference, from the current POS
	 * 	to the target POS.
	 *
	 * 	Then we want to use the Jacobian matrix, to get to the
	 * 	target POS.
	 */

	//	Define the stepsize //TODO: Maybe in .h, or a proper define.
	double stepsize = 0.01;
	double femurLength = 0.13994910502;
	double tibiaLength = 0.20226137589;

	//	Write the target position. (load from file)
	std::vector<double> targetPos;
	targetPos.push_back( 0.27 ); //x
	targetPos.push_back( 0.17 ); //y

	//	Do forward kinematics to find the current position.
	std::vector<double> currentPos;
	currentPos.push_back( femurLength*cos( sensor[7] ) + tibiaLength*cos( sensor[7] + sensor[13] ) );
	currentPos.push_back( femurLength*sin( sensor[7] ) + tibiaLength*sin( sensor[7] + sensor[13] ) );

	//	Find the error between the target- and current position.
	std::vector<double> errorPos;
	errorPos.push_back( targetPos[0]-currentPos[0] );
	errorPos.push_back( targetPos[1]-currentPos[1] );

	//	Find the distance from the start.
	double dist_start = sqrt( pow(errorPos[0],2) + pow(errorPos[1],2) );

	//	Step we want to take
	std::vector<double> deltaStep;
	deltaStep.push_back( errorPos[0]*stepsize );
	deltaStep.push_back( errorPos[1]*stepsize );

	//	Position we want to hit
	std::vector<double> stepPosition;
	stepPosition.push_back( currentPos[0] + deltaStep[0] );
	stepPosition.push_back( currentPos[1] + deltaStep[1] );

	//	Now that we have the position we want to hit, we try
	//	to rotate the joints, to the positions.
	std::cout << std::endl;
	std::cout << "*******" << std::endl;
	std::cout << "cx: " << currentPos[0] << " cy: " << currentPos[1] << std::endl;
	std::cout << "dx: " << stepPosition[0] << " dy: " << stepPosition[1] << std::endl;
	std::cout << "tx: " << targetPos[0] << " ty: " << targetPos[1] << std::endl;
	std::cout << "*******" << std::endl;
	std::cout << std::endl;

	//	joint 1-
	std::vector<double> cal1;
	cal1.push_back( femurLength*cos( sensor[7]-stepsize ) +tibiaLength*cos( sensor[7]-stepsize + sensor[13] ) );
	cal1.push_back( femurLength*sin( sensor[7]-stepsize ) +tibiaLength*sin( sensor[7]-stepsize + sensor[13] ) );
	//	joint 1+
	std::vector<double> cal2;
	cal2.push_back( femurLength*cos( sensor[7]+stepsize ) +tibiaLength*cos( sensor[7]+stepsize + sensor[13] ) );
	cal2.push_back( femurLength*sin( sensor[7]+stepsize ) +tibiaLength*sin( sensor[7]+stepsize + sensor[13] ) );
	//	joint 2-
	std::vector<double> cal3;
	cal3.push_back( femurLength*cos( sensor[7] ) +tibiaLength*cos( sensor[7] + sensor[13]-stepsize ) );
	cal3.push_back( femurLength*sin( sensor[7] ) +tibiaLength*sin( sensor[7] + sensor[13]-stepsize ) );
	//	joint 2+
	std::vector<double> cal4;
	cal4.push_back( femurLength*cos( sensor[7] ) +tibiaLength*cos( sensor[7] + sensor[13]+stepsize ) );
	cal4.push_back( femurLength*sin( sensor[7] ) +tibiaLength*sin( sensor[7] + sensor[13]+stepsize ) );

	//	Now we need to calculate the distance to the point, with the different joint moves.
	std::vector<double> lenghtTest;
	lenghtTest.push_back( sqrt( pow(stepPosition[0] - cal1[0],2) + pow(stepPosition[1] - cal1[1],2)) );
	lenghtTest.push_back( sqrt( pow(stepPosition[0] - cal2[0],2) + pow(stepPosition[1] - cal2[1],2)) );
	lenghtTest.push_back( sqrt( pow(stepPosition[0] - cal3[0],2) + pow(stepPosition[1] - cal3[1],2)) );
	lenghtTest.push_back( sqrt( pow(stepPosition[0] - cal4[0],2) + pow(stepPosition[1] - cal4[1],2)) );

	if( (lenghtTest[0] < lenghtTest[1]) && (lenghtTest[0] < lenghtTest[2]) && (lenghtTest[0] < lenghtTest[3]) ){
		std::cout << "CASE 0" << std::endl;
		angleVector[legNum][1] = sensor[7]-stepsize;
		angleVector[legNum][2] = sensor[13];
	}
	else if( (lenghtTest[1] < lenghtTest[0]) && (lenghtTest[1] < lenghtTest[2]) && (lenghtTest[1] < lenghtTest[3]) ){
		std::cout << "CASE 1" << std::endl;
		angleVector[legNum][1] = sensor[7]+stepsize;
		angleVector[legNum][2] = sensor[13];
	}
	else if( (lenghtTest[2] < lenghtTest[0]) && (lenghtTest[2] < lenghtTest[1]) && (lenghtTest[2] < lenghtTest[3]) ){
		std::cout << "CASE 2" << std::endl;
		angleVector[legNum][1] = sensor[7];
		angleVector[legNum][2] = sensor[13]-stepsize;
	}
	else if( (lenghtTest[3] < lenghtTest[0]) && (lenghtTest[3] < lenghtTest[1]) && (lenghtTest[3] < lenghtTest[2]) ){
		std::cout << "CASE 3" << std::endl;
		angleVector[legNum][1] = sensor[7];
		angleVector[legNum][2] = sensor[13]+stepsize;
	}
	else{
		std::cout << "DEFAULT CASE" << std::endl;
	}

	std::vector<double> nextPos;
	nextPos.push_back( femurLength*cos( angleVector[legNum][1] ) +tibiaLength*cos( angleVector[legNum][1] + angleVector[legNum][2] ) );
	nextPos.push_back( femurLength*cos( angleVector[legNum][1] ) +tibiaLength*cos( angleVector[legNum][1] + angleVector[legNum][2] ) );

	double dist_fin = sqrt( pow(targetPos[0] - nextPos[0],2) + pow(targetPos[1] - nextPos[1],2) );

	std::cout << dist_fin << "-" << dist_start << "=" << dist_fin-dist_start << std::endl;


	/*
	std::vector<double> temp_vec;

	for( int i = 0; i < 3; i++ ){
		temp_vec.push_back( sensor[legNum+6*i] );
	}

	if( legAtPosition( temp_vec, 0.001, legNum ) )
	{
		targetPositionPointer[legNum] = targetPositionPointer[legNum]+1;
		std::cout << "TRUE TRUE TRUE" << std::endl;
		std::cout << "leg: " << legNum << " state: " << targetPositionPointer[legNum] % positionListL0.size() << std::endl;
	}
	else{
		std::cout << "FALSE FALSE FALSE" << std::endl;
		std::cout << "leg: " << legNum << " state: " << targetPositionPointer[legNum] % positionListL0.size() << std::endl;
	}

	switch(legNum){
		case 0:
			for( int i = 0; i < 3; i++ ){
				angleVector[legNum][i] = positionListL0[(targetPositionPointer[legNum]) % positionListL0.size()][i];
			}
			break;
		case 1:
			for( int i = 0; i < 3; i++ ){
				angleVector[legNum][i] = positionListL1[(targetPositionPointer[legNum]) % positionListL1.size()][i];
			}
			break;
		case 2:
			for( int i = 0; i < 3; i++ ){
				angleVector[legNum][i] = positionListL2[(targetPositionPointer[legNum]) % positionListL2.size()][i];
			}
			break;
		case 3:
			for( int i = 0; i < 3; i++ ){
				angleVector[legNum][i] = positionListR0[(targetPositionPointer[legNum]) % positionListR0.size()][i];
			}
			break;
		case 4:
			for( int i = 0; i < 3; i++ ){
				angleVector[legNum][i] = positionListR1[(targetPositionPointer[legNum]) % positionListR1.size()][i];
			}
			break;
		case 5:
			for( int i = 0; i < 3; i++ ){
				angleVector[legNum][i] = positionListR2[(targetPositionPointer[legNum]) % positionListR2.size()][i];
			}
			break;
		default:
			std::cout << "error 2" << std::endl;
			break;
	}
	*/
}

bool kinematicsController::legAtPosition( std::vector<double> currentPos, double deadband, int legNum )
{
	std::vector<double> targetPos;

	switch(legNum){
		case 0:
			for( int i = 0; i < 3; i++ ){
				targetPos.push_back( positionListL0[(targetPositionPointer[legNum]) % positionListL0.size()][i] );
			}
			break;
		case 1:
			for( int i = 0; i < 3; i++ ){
				targetPos.push_back( positionListL1[(targetPositionPointer[legNum]) % positionListL1.size()][i] );
			}
			break;
		case 2:
			for( int i = 0; i < 3; i++ ){
				targetPos.push_back( positionListL2[(targetPositionPointer[legNum]) % positionListL2.size()][i] );
			}
			break;
		case 3:
			for( int i = 0; i < 3; i++ ){
				targetPos.push_back( positionListR0[(targetPositionPointer[legNum]) % positionListR0.size()][i] );
			}
			break;
		case 4:
			for( int i = 0; i < 3; i++ ){
				targetPos.push_back( positionListR1[(targetPositionPointer[legNum]) % positionListR1.size()][i] );
			}
			break;
		case 5:
			for( int i = 0; i < 3; i++ ){
				targetPos.push_back( positionListR2[(targetPositionPointer[legNum]) % positionListR2.size()][i] );
			}
			break;
		default:
			break;
	}

	double coxaError = targetPos[0] - currentPos[0];
	double femurError = targetPos[1] - currentPos[1];
	double tibiaError = targetPos[2] - currentPos[2];
/*
	if( legNum == 0 ){
		std::cout << "TEST TEST" << std::endl;
		std::cout << coxaError << " " << femurError << " " << tibiaError << std::endl;
		std::cout << std::abs(coxaError) << " " << std::abs(femurError) << " " << std::abs(tibiaError) << std::endl;
	}
*/

	if(std::abs(coxaError) < deadband && std::abs(femurError) < deadband && std::abs(tibiaError) < deadband){
		return true;
	} else {
		return false;
	}
}



void kinematicsController::loadPositionVectors(void) {
	/**
	 * Input is row upon rows.
	 * Three data values, for each leg.
	 * These are all stored in a 2D vector.
	 * The 2D vector is 3 for each row,
	 * and the height depends on the resolution
	 * of the data given.
	 */

	std::ifstream myfile ("position.data", std::ios::in);

	/**
	 * 	Make a check on the input strings, that it is empty
	 * 	or that there is a '-' to end.
	 */

	int m = 0;
	while(myfile)
	{
		std::string s;
		if( !getline( myfile, s ) ){
			break;
		}

		std::istringstream ss( s );
		std::vector<double> record;

		while( ss ){
			std::string s;
			if (!getline( ss, s, ',' )){
				break;
			}
			record.push_back( std::stod( s ) );
		}

		for( int i=0; i<record.size(); i=i+3 )
		{
			std::cout << record[i] << " I= " << i << " m= " << m << std::endl;

			//	INPUT TO THE VECTOR
			std::vector<double> temp_vec;
			temp_vec.push_back(record[i]);

			switch (i) {
				case 0: case 1: case 2:			//	L0
					temp_vec.push_back(record[i]);
					temp_vec.push_back(record[i+1]);
					temp_vec.push_back(record[i+2]);

					positionListL0.push_back( temp_vec );
					break;

				case 3: case 4: case 5:			//	L1
					temp_vec.push_back(record[i]);
					temp_vec.push_back(record[i+1]);
					temp_vec.push_back(record[i+2]);

					positionListL1.push_back( temp_vec );
					break;

				case 6: case 7: case 8:			//	L2
					temp_vec.push_back(record[i]);
					temp_vec.push_back(record[i+1]);
					temp_vec.push_back(record[i+2]);

					positionListL2.push_back( temp_vec );
					break;
				case 9: case 10: case 11:		//	R0
					temp_vec.push_back(record[i]);
					temp_vec.push_back(record[i+1]);
					temp_vec.push_back(record[i+2]);

					positionListR0.push_back( temp_vec );
					break;
				case 12: case 13: case 14:		//	R1
					temp_vec.push_back(record[i]);
					temp_vec.push_back(record[i+1]);
					temp_vec.push_back(record[i+2]);

					positionListR1.push_back( temp_vec );
					break;
				case 15: case 16: case 17:		//	R2
					temp_vec.push_back(record[i]);
					temp_vec.push_back(record[i+1]);
					temp_vec.push_back(record[i+2]);

					positionListR2.push_back( temp_vec );
					break;

				default:
					std::cout << "default" << std::endl;
					break;
			}
		}
		std::cout << std::endl;

		m++;	//	Increment m for the rows for the vectors.
	}
}
