#include <ode-dbl/ode.h>
#include <iostream>

#include "controller_PID.h"
using namespace std;

namespace lpzrobots {

controller_PID::controller_PID(OneAxisJoint* joint, double maxPower,
		double _min, double _max, double KP, double KI, double KD)
	: KP(KP), KI(KI), KD(KD), maxPower(maxPower), joint(joint), min(_min), max(_max){
    proportional   = 0;
    integrator	   = 0;
    derivative     = 0;
    position       = 0;
    lastposition   = 0;
    error          = 0;
    lasttime       = -1;
    lasterror      = 0;
    force          = 0;
}

controller_PID::controller_PID(){
};

double controller_PID::step(double target_position) {
	target_position = clip(target_position, -1.0, 1.0);
	if(target_position > 0){
		target_position *= max;
	}else{
		target_position *= -min;
	}

	double current_time = joint->odeHandle.getTime();
	double current_position = joint->getPosition1();

	if((lasttime != -1 && current_time - lasttime > 0) && error > 0.005)
	{
		lastposition = position;
		position = current_position;
		double stepsize=current_time-lasttime;

		error = target_position - position;
		derivative = (error - lasterror) / stepsize;
		integrator += stepsize * error;


		force = (error*KP) + (derivative*KD) + (integrator*KI);

		if( force > 1 )
			force = 1;
		else if( force < -1 )
			force = -1;

	} else force=0;


	lasttime=current_time;
	lasterror = error;
	//cout << "Kp " << KP << endl;
	//cout << "Ki " << KI << endl;
	//cout << "Kd " << KD << endl;
	force = 0.003;
	return force;
}

controller_PID::~controller_PID(){};
}
