#ifndef __controller_PID_H
#define __controller_PID_H

namespace lpzrobots {
class controller_PID
  {
    //*********************attributes***************
  public:

    double position;
    double lastposition;

    double error;
    double lasterror;
    double proportional;
    double derivative;
    double integrator;

    double KP;
    double KI;
    double KD;

    double force;
    double lasttime;

    double maxPower;
    OneAxisJoint* joint;
    double min;
    double max;

    //*********************methods******************
  public :
    controller_PID ( OneAxisJoint* joint, double maxPower, double _min,
    				 double _max, double KP , double KI , double KD );

    controller_PID ();
    ~controller_PID();

    double step(double target_position);


  };

}

#endif
