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

#ifndef __DUNGBOT_H
#define __DUNGBOT_H

// #include <ode/ode.h>
#include <ode-dbl/ode.h>

// include primitives (box, spheres, cylinders ...)
#include <ode_robots/primitive.h>

// include sensors
#include <ode_robots/contactsensor.h>

// include joints
#include <ode_robots/joint.h>
#include <ode_robots/oneaxisservo.h>
#include <ode_robots/constantmotor.h>
#include <string>

// Extra includes
#include <vector>
#include <selforg/inspectable.h>
#include <ode_robots/oderobot.h>

/**
 * forward declarations
 */
namespace lpzrobots {
  class HingeJoint;
  class IRSensor;
  class Joint;
  class OneAxisServo;
  class Primitive;
  class RaySensorBank;
  class SliderJoint;
  class SpeedSensor;
  class Spring;
  class TwoAxisServo;
  // Added sound sensors (2) class
  class SoundSensor;
}





namespace lpzrobots
{

typedef struct
{
	double massFront;
	double massRear;
	std::vector<double> frontDimension;
	std::vector<double> rearDimension;

	//std::vector<double> shoulderLength;
	//std::vector<double> coxaLength;
	//std::vector<double> femurLength;
	//std::vector<double> tibiaLength;

	std::vector<osg::Matrixd> shoulderRotation;

	// TEMP LEG
	double shoulderLength;
	double shoulderRadius;
	double coxaLength;
	double coxaRadius;
	double femurLength;
	double femurRadius;
	double tebiaLength;
	double tebiaRadius;
	double footRange;
	double footRadius;
	double legdistHindMiddle;
	double legdistFrontMiddle;
	double fLegTrunkAngleV;
	double fLegTrunkAngleH;
	double fLegRotAngle;
	double mLegTrunkAngleV;
	double mLegTrunkAngleH;
	double mLegRotAngle;
	double rLegTrunkAngleV;
	double rLegTrunkAngleH;
	double rLegRotAngle;

}DungBotConf;


	class DungBot : public OdeRobot, public Inspectable
	{
		DungBotConf conf;
		public:
			static DungBotConf getDefaultConf()
			{
				DungBotConf conf;
				//	Dependent parameters
				conf.massFront = 1;
				conf.massRear = 1;
				conf.frontDimension = { 0.6, 0.65, 0.25 };
				conf.rearDimension = { 1.0, 0.75, 0.25 };

				conf.shoulderRotation = { osg::Matrix::rotate( ( ( M_PI )/2 ), 1, 0, 0 ), osg::Matrix::rotate( ( ( M_PI )/2 ), -1, 0, 0 ) };

				// ------------ Leg dimensions --------
				conf.shoulderLength = 0.5;
				conf.shoulderRadius = 0.02;
				conf.coxaLength = 0.5;
				conf.coxaRadius = 0.02;
				conf.femurLength = 0.5;
				conf.femurRadius = 0.03;
				conf.tebiaLength = 0.5;
				conf.tebiaRadius = 0.02;
				conf.footRange = 0.075;
				conf.footRadius = 0.02;
			    // ------------- Front legs -------------
			    conf.fLegTrunkAngleV = 0.0;	// => forward/backward
			    conf.fLegTrunkAngleH = 0.0;	// => upward/downward
			    conf.fLegRotAngle = 0.0;	// => till
			    // ------------- Middle legs ----------------
			    conf.mLegTrunkAngleV = 0.0;	// => forward/backward
			    conf.mLegTrunkAngleH = 0.0;	// => upward/downward
			    conf.mLegRotAngle = 0.0;	// => till
			    // ------------- Rear legs ------------------
			    conf.rLegTrunkAngleV = 0.0;	// => forward/backward
			    conf.rLegTrunkAngleH = 0.0;	// => upward/downward
			    conf.rLegRotAngle = 0.0;	// => till

				return conf;
			}

			enum LegPos {
				L0, L1, L2, R0, R1, R2, LEG_POS_MAX
			};
			enum LegJointType
			{
				//  Thoroca-Coxal joint for forward (+) and backward (-) movements.
				TC,
				//  Coxa-Trochanteral joint for elevation (+) and depression (-) of the leg
				CTR,
				//  Femur-Tibia joints for extension (+) and flexion (-) of the tibia
				FTI,
				//  Maximum value, used for iteration
				LEG_JOINT_TYPE_MAX
			};

			DungBot( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
						const DungBotConf &conf = getDefaultConf(),
						const std::string& name = "DungBot" );

			virtual void placeIntern( const osg::Matrix& pose ) override;
			virtual void create( const osg::Matrix& pose );
			virtual void doInternalStuff( GlobalData& globalData );
			virtual void update( void );
			virtual void sense( GlobalData& globalData );
			virtual ~DungBot();

		protected:
		      struct Leg {
		          Leg();
		          HingeJoint * tcJoint;
		          HingeJoint * ctJoint;	//Called ctrJoint in AmosII
				  HingeJoint * ftJoint;	//Called ftiJoing in AmosII
		        /*Slider*/Joint * footJoint;
		          OneAxisServo * tcServo;
		          OneAxisServo * ctrServo;
		          OneAxisServo * ftiServo;
		          Spring * footSpring;
		          Primitive * shoulder;
		          Primitive * coxa;
		          Primitive * femur; 	//Called second in AmosII
		          Primitive * tibia;
		          Primitive * foot;
		      };

		private:
			lpzrobots::Primitive* makeBody( const osg::Matrix&, const double , const std::vector<double> );
			lpzrobots::Primitive* makeLegPart( const osg::Matrix&, const double , const double, const double );
			lpzrobots::Primitive* makeFoot( const osg::Matrix& );
			void makeAllLegs( const osg::Matrix& pose, Primitive*, Primitive* );
			void makeBodyHingeJoint( Primitive*, Primitive*, const Pos, Axis, const double );
			void makeLegHingeJoint( Primitive*, Primitive*, const Pos, Axis, const double );

			lpzrobots::Primitive* makeLegSphereJoint( const osg::Matrix&, const double, const double );

			// for legs
			typedef std::map<LegPos, Leg> LegMap;
			LegMap legs;

		protected:
			Position startPosition;
			Position position;
	};
} //End namespace lpzrobot



#endif
