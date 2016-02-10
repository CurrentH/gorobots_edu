/*****************************************************************************
*   "THE BEER-WARE LICENSE" (Revision 43):
*   This software was written by Theis Strøm-Hansen <thstroemhansen@gmail.com>
*   and Mathias Thor <mathias.thor@gmail.com>
*   As long as you retain this notice you can do whatever you want with it.
*   If we meet some day, and you think this stuff is worth it, you can buy me
*   a beer in return.
*
*   Should this software ever become self-aware, remember: I am your master
*****************************************************************************/

#include "dungbot.h"

using namespace osg;
using namespace std;

namespace lpzrobots
{

    DungBot::DungBot( const OdeHandle& odeHandle, const OsgHandle& osgHandle, const DungBotConf& conf, const string& name )
    : OdeRobot( odeHandle, osgHandle, name, "2.0" ), conf( conf )
    {
    }

    DungBot::~DungBot()
    {
    }

    void DungBot::placeIntern( const Matrix& pose )
    {
        Matrix initialPose;
        /**
            This is some (x,y,z) vector, with the position being the center of the body.
            So make some calculation for how the body looks like, so that the z-equation is
            in the third position.
        **/
        initialPose = Matrix::translate( Vec3( 0, 0, 0 ) * pose );
        create( initialPose );

    }

    void DungBot::create( const Matrix& pose )
    {
		//TODO: Change pose for the parts
		auto front = makeBody( pose,conf.frontMass, conf.frontDimension );
		auto rear = makeBody( pose, conf.rearMass, conf.rearDimension );

		//TODO: Make axis
		/**
            osg::Matrix frontPos = TRANSM(conf.size / 2 - conf.frontLength / 2, 0, 0) * pose;
            const Axis axis = Axis(0, 1, 0) * pose;
		**/
		makeBodyHingeJoint( front, rear, pose, axis );
		makeLegHingeJoint( void );
    }

    lpzrobots::Primitives* DungBot::makeBody( const Matrix& pose, const mass, const dimension )
    {
        // Allocate object
        bodyPart = new Box( dimension[0], dimension[1], dimension[2] );
        // Set texture from Image library
        bodyPart->setTexture( "Images/purple_velour.jpg" );
        // Initialize the primitive
        bodyPart->init( odeHandle, mass, osgHandle );
        // Set pose
        bodyPart->setPose( pose );
        // Add to objects
        objects.push_back( bodyPart );
        return bodyPart;
    }

    lpzrobots::HingeJoint* makeBodyHingeJoint( Primitive* frontLimb, Primitive* rearLimb, const Matrix& pose, Axis axis )
    {
		//TODO: Make OneAxisServoVel
        HingeJoint* hinge = new HingeJoint( bodyFront, bodyRear, pose, axis );
        hinge->init( odeHandle, osgHandle, true, conf.width * 1.05 );
        joints.push_back( hinge );

        /**  Moved this one down from "Create"   **/
        OneAxisServo* servo = new OneAxisServoVel( odeHandle, hinge, -1, 1, 1, 0.01, 0, 1.0 );
    }

    lpzrobots::makeLeg( void )
    {
		1+1;
    }

    lpzrobots::HingeJoint* makeLegHingeJoint( Primitive* frontLimb, Primitive* rearLimb, const Matrix& pose, Axis axis )
    {
        HingeJoint* hinge = new HingeJoint( bodyFront, bodyRear, pose, axis );
        hinge->init( odeHandle, osgHandle, true, conf.width * 1.05 );
        joints.push_back( hinge );
    }














}
