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
    	// TODO: Should we add a delete func like in amos?
    }

    void DungBot::placeIntern( const Matrix& pose )
    {
        Matrix initialPose;
        /**
            This is some (x,y,z) vector, with the position being the center of the body.
            So make some calculation for how the body looks like, so that the z-equation is
            in the third position.
        **/

        initialPose = osg::Matrix::translate( Vec3( 0, 0, 1 ) * pose );
        create( initialPose );

    }
    void DungBot::doInternalStuff( GlobalData& globalData )
    {

    }

    void DungBot::update( void )
    {

    }

    void DungBot::sense( GlobalData& globalData )
    {

    }


    void DungBot::create( const Matrix& pose )
    {
		//TODO: Change pose for the parts
    	osg::Matrix frontPos = osg::Matrix::translate((conf.frontDimensionX / 2), 0, 0) * pose;
		auto front = makeBody( frontPos, conf.massFront, conf.frontDimensionX,conf.frontDimensionY,conf.frontDimensionZ );

		osg::Matrix rearPos = osg::Matrix::translate((-conf.rearDimensionX / 2), 0, 0) * pose;
		auto rear = makeBody( rearPos, conf.massRear, conf.rearDimensionX,conf.rearDimensionY,conf.rearDimensionZ );

		//TODO: Make axis
		const Pos nullpos(0,0,0);

		makeBodyHingeJoint( front, rear, nullpos*osg::Matrix::translate(-conf.frontDimensionX / 2, 0, 0) * frontPos, Axis(0,1,0)*frontPos, conf.rearDimensionY );
		//makeLegHingeJoint( void );
    }

    lpzrobots::Primitive* DungBot::makeBody( const Matrix& pose, const double mass, const double X, const double Y, const double Z )
    {
        // Allocate object
        auto bodyPart = new Box( X,Y,Z );
        // Set texture from Image library
        bodyPart->setTexture( "Images/wall.jpg" );
        // Initialize the primitive
        bodyPart->init( odeHandle, mass, osgHandle );
        // Set pose
        bodyPart->setPose( pose );
        // Add to objects
        objects.push_back( bodyPart );
        return bodyPart;
    }

    void DungBot::makeBodyHingeJoint( Primitive* frontLimb, Primitive* rearLimb, const Pos position, Axis axis, const double Y )
    {
		HingeJoint* hinge = new HingeJoint(frontLimb, rearLimb, position, axis);
		hinge->init( odeHandle, osgHandle, true, Y * 1.05 );
		joints.push_back( hinge );

		/**  Moved this one down from "Create"   **/
		//OneAxisServo* servo = new OneAxisServoVel( odeHandle, hinge, -1, 1, 1, 0.01, 0, 1.0 );
    }

    void DungBot::makeLegHingeJoint( Primitive* frontLimb, Primitive* rearLimb, const Pos position, Axis axis, const double Y )
    {
        HingeJoint* hinge = new HingeJoint( frontLimb, rearLimb, position, axis );
        hinge->init( odeHandle, osgHandle, true, Y * 1.05 );
        joints.push_back( hinge );
    }

}
