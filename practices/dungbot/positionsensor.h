/*
 * positionsensor.h
 *
 *  Created on: Apr 26, 2016
 *      Author: theis
 */

#ifndef POSITIONSENSOR_H_
#define POSITIONSENSOR_H_

#include "sensor.h"

namespace lpzrobots{

	/*
	 *	Class used for sensing the position compared to
	 *	position (0,0,0).
	 */
	class positionsensor : public Sensor {
		public:
			positionsensor();
			virtual ~positionsensor();

	};

}

#endif /* POSITIONSENSOR_H_ */
