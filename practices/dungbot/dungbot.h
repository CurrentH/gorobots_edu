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

//	include motor and sensor definitions
#include "DungBotSensorMotorDefinition.h"

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
#include <iostream>
#include <selforg/inspectable.h>
#include <ode_robots/oderobot.h>

/**
 * forward declarations
 */
// TODO: Any reason these is here, and not inside the namespace below?
namespace lpzrobots
{
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

		// Legs
		double coxaLength;
		double coxaRadius;
		double coxaMass;
		double femurLength;
		double femurRadius;
		double femurMass;
		double tibiaLength;
		double tibiaRadius;
		double tibiaMass;
		double footRange;
		double footRadius;
		double footMass;
		double footSpringPreload;
		double tarusMass;
		bool   makeFoot;

	}DungBotConf;

	class DungBot : public OdeRobot, public Inspectable
	{
		DungBotConf conf;
		public:
			static DungBotConf getDefaultConf()
			{
				DungBotConf conf;

				/*		MATHIAS THOR's MASS CALCULATION (c) :D
				 * ---------------------------------------------------------------------
				 * 	The average density of the human body [ρ=kg/m³]: 985
				 * 	Mass of a capsule [m]: 	m = ρV = ρ π radius²((4/3)radius+height)
				 * 	Mass of a box [m]: 		m = ρV = ρ height width length
				 * 	eg. conf.coxaMass = 985*3.14*conf.coxaRadius*conf.coxaRadius*((4/3)*conf.coxaRadius+conf.coxaLength);
				 * ---------------------------------------------------------------------
				 */

				//	----------- Body dimensions -------
				conf.frontDimension = { 0.4, 0.45, 0.2 };
				conf.massFront 	= 1.75;
				conf.rearDimension 	= { 0.8, 0.55, 0.2 };
				conf.massRear 	= 2;
				//-------------------------------------

				// ------------ Leg dimensions --------
				conf.coxaLength = 0.3;
				conf.coxaRadius = 0.02;		// COXA
				conf.coxaMass = 0.25;

				conf.femurLength = 0.3;
				conf.femurRadius = 0.02;	// FEMUR
				conf.femurMass = 0.25;

				conf.tibiaLength = 0.3;
				conf.tibiaRadius = 0.02;	// TEBIA
				conf.tibiaMass = 0.25;

				conf.footRange = 0.05;
				conf.footRadius = 0.015;		// FOOT
				conf.footMass = 0.09;
			    conf.footSpringPreload = 0.0;

			    conf.tarusMass = 0.008;
			    //-------------------------------------
			    conf.makeFoot = true;		// If true the foot is made

				return conf;
			}

			enum LegPos
			{
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
			typedef DungBotMotorSensor::DungBotMotorNames MotorName;

			DungBot( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
						const DungBotConf &conf = getDefaultConf(),
						const std::string& name = "DungBot" );

			virtual void placeIntern( const osg::Matrix& pose ) override;
			virtual void create( const osg::Matrix& pose );
			virtual void doInternalStuff( GlobalData& globalData );
			virtual void update( void );
			virtual void sense( GlobalData& globalData );
			virtual ~DungBot();


			virtual void setMotorsIntern( const double* motors, int motornumber );
			virtual int getSensorsIntern( sensor* sensors, int sensornumber );

			virtual int getSensorNumberIntern( void );
			virtual int getMotorNumberIntern( void );

			static MotorName getMotorName( LegPos leg, LegJointType joint );

		protected:
			void nameMotor( const int motorNo, const char* name );
			void nameSensor( const int sensorNo, const char* name );

			struct Leg
			{
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

			Position startPosition;
			Position position;

		private:
			lpzrobots::Primitive* makeBody( const osg::Matrix&, const double , const std::vector<double> );
			lpzrobots::Primitive* makeLegPart( const osg::Matrix&, const double , const double, const double );
			lpzrobots::Primitive* makeFoot( const osg::Matrix& );
			lpzrobots::Primitive* makeLegSphereJoint( const osg::Matrix&, const double, const double );
			void makeAllLegs( const osg::Matrix& pose, Primitive*, Primitive* );
			void makeBodyHingeJoint( Primitive*, Primitive*, const Pos, Axis, const double );
			void makeLegHingeJoint( Primitive*, Primitive*, const Pos, Axis, const double );

			bool created;

			//	For legs
			typedef std::map< LegPos, Leg > LegMap;
			typedef std::map< MotorName, OneAxisServo* > MotorMap;
			typedef std::map< std::pair< LegPos, int >, ContactSensor* > TarsusContactMap;
			LegMap legs;

			//	For servos
			OneAxisServo * backboneServo;
			MotorMap servos;

			//	For tarsus contact
			TarsusContactMap tarsusContactSensors;
	};
} //End namespace lpzrobot

#endif
