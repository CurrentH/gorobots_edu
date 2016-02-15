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

#include <ode_robots/oderobot.h>
#include <ode_robots/primitive.h>
#include <ode_robots/joint.h>
#include <selforg/inspectable.h>

#include <ode_robots/contactsensor.h>

#include <string>
#include <vector>

typedef struct
{
	double massFront;
	double massRear;
	std::vector<double> frontDimension;
	std::vector<double> rearDimension;

}DungBotConf;

namespace lpzrobots
{
	class DungBot : public OdeRobot, public Inspectable
	{
		struct Leg
		{
			Leg();
			HingeJoint * tcJoint;
			HingeJoint * ctrJoint;
			HingeJoint * ftiJoint;
			/*Slider*/Joint * footJoint;
			//OneAxisServo * tcServo;
			//OneAxisServo * ctrServo;
			//OneAxisServo * ftiServo;
			//Spring * footSpring;
			Primitive * shoulder;
			Primitive * coxa;
			Primitive * second;
			Primitive * tibia;
			Primitive * foot;
		};

		DungBotConf conf;

		public:
			static DungBotConf getDefaultConf()
			{
				DungBotConf conf;
				//double scale = 5;

				//	Dependent parameters
				conf.massFront = 1;
				conf.massRear = 1;
				conf.frontDimension = {0.6, 0.5, 0.25};
				conf.rearDimension = {1.0, 0.75, 0.25};

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

			DungBot( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
						const DungBotConf &conf = getDefaultConf(),
						const std::string& name = "DungBot" );

			virtual void placeIntern( const osg::Matrix& pose ) override;
			virtual void create( const osg::Matrix& pose );
			virtual void doInternalStuff( GlobalData& globalData );
			virtual void update( void );
			virtual void sense( GlobalData& globalData );
			virtual ~DungBot();

		private:
			lpzrobots::Primitive* makeBody( const osg::Matrix&, const double , const std::vector<double> );
			lpzrobots::Primitive* makeLegPart( const osg::Matrix&, const double , const double, const double );
			lpzrobots::Primitive* makeFoot( const osg::Matrix& );

			void makeBodyHingeJoint( Primitive*, Primitive*, const Pos, Axis, const double );
			void makeLegHingeJoint( Primitive*, Primitive*, const Pos, Axis, const double );

		protected:
			Position startPosition;
			Position position;
	};
} //End namespace lpzrobot



#endif
