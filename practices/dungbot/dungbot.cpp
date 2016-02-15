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
        /************************************
         * BODY PARTS
         ***********************************/
    	//TODO: Find away to give different textures to the different body parts.

		//First we find the pose for the center of the body-part to be made, then we run the functions that creates it.
    	osg::Matrix frontPos = osg::Matrix::translate( ( conf.frontDimension[0] / 2 ), 0, 0) * pose;
		auto front = makeBody( frontPos, conf.massFront, conf.frontDimension );

		osg::Matrix rearPos = osg::Matrix::translate( ( -conf.rearDimension[0] / 2 ), 0, 0) * pose;
		auto rear = makeBody( rearPos, conf.massRear, conf.rearDimension );

		//Origin position
		const Pos nullpos(0,0,0);

		//Place the joint between the two body-parts
		makeBodyHingeJoint( front, rear, nullpos*osg::Matrix::translate( -conf.frontDimension[0] / 2, 0, 0 ) * frontPos, Axis( 0, 1, 0 ) * frontPos, conf.rearDimension[1] );

	    /************************************
	     * LEGS
	     ***********************************/
		//TODO: Need to be coded
		/* Ideen er at vi kalder makeLegPart med de tre forskellige leg-parts (coxa, femur, tibia).
		 * Så skal vi kalde en legjoint funktion der kan oprette de tre led (TC, CT, FT)
		 * SE --> --> --> http://www.manoonpong.com/paper/2015/SWARM_2015_DungBeetleRobot.pdf
		 * */

		/**
		 * 	Leg 1
		 */
				//TODO: Make outisde function for this, that recursive makes the leg parts.
		osg::Matrix shoulderPos1 = osg::Matrix::translate( (conf.rearDimension[0]/4), (conf.rearDimension[1] / 2), -(conf.rearDimension[2] / 2) ) * rearPos;
		auto shoulder1 = makeLegSphereJoint( conf.shoulderRotation[0]*shoulderPos1, 1, 0.05 );
		makeLegHingeJoint( rear, shoulder1, nullpos*shoulderPos1, Axis( 0, 1, 0 ) * shoulderPos1, 0.1 );

		osg::Matrix tcPos1 = shoulderPos1 * osg::Matrix::translate( 0, 0 , -0.05 );
		auto TC1 = makeLegPart( tcPos1, 1, 0.05, 0.1 );
		makeLegHingeJoint( shoulder1, TC1, nullpos*shoulderPos1, Axis( 1, 0, 0 )* shoulderPos1, 0.1 );
				//TODO: For the next two hinges, find the correct height.
		osg::Matrix ctPos1 = tcPos1 * osg::Matrix::translate( 0, 0, -0.1);
		auto CT1 = makeLegPart( ctPos1, 1, 0.05, 0.1 );
		makeLegHingeJoint( TC1, CT1, nullpos*ctPos1, Axis( 1, 0, 0 )* ctPos1, 0.1 );

		osg::Matrix ftPos1 = ctPos1 * osg::Matrix::translate( 0, 0, -0.1);
		auto FT1 = makeLegPart( ftPos1, 1, 0.05, 0.1 );
		makeLegHingeJoint( CT1, FT1, nullpos*ftPos1, Axis( 1, 0, 0 )* ftPos1, 0.1 );
		/**
		 * 	Leg 1 done
		 */

		osg::Matrix shoulderPos2 = osg::Matrix::translate( -(conf.rearDimension[0]/4), (conf.rearDimension[1] / 2), -(conf.rearDimension[2] / 2) ) * rearPos;
		auto shoulder2 = makeLegSphereJoint( conf.shoulderRotation[1]*shoulderPos2, 1, 0.05 );
		osg::Matrix shoulderPos3 = osg::Matrix::translate( (conf.rearDimension[0]/4), -(conf.rearDimension[1] / 2), -(conf.rearDimension[2] / 2) ) * rearPos;
		auto shoulder3 = makeLegSphereJoint( conf.shoulderRotation[0]*shoulderPos3, 1, 0.05 );
		osg::Matrix shoulderPos4 = osg::Matrix::translate( -(conf.rearDimension[0]/4), -(conf.rearDimension[1] / 2), -(conf.rearDimension[2] / 2) ) * rearPos;
		auto shoulder4 = makeLegSphereJoint( conf.shoulderRotation[1]*shoulderPos4, 1, 0.05 );
		osg::Matrix shoulderPos5 = osg::Matrix::translate( 0, (conf.frontDimension[1] / 2), -(conf.frontDimension[2] / 2) ) * frontPos;
		auto shoulder5 = makeLegSphereJoint( conf.shoulderRotation[0]*shoulderPos5, 1, 0.05 );
		osg::Matrix shoulderPos6 = osg::Matrix::translate( 0, -(conf.frontDimension[1] / 2), -(conf.frontDimension[2] / 2) ) * frontPos;
		auto shoulder6 = makeLegSphereJoint( conf.shoulderRotation[1]*shoulderPos6, 1, 0.05 );



    }

    lpzrobots::Primitive* DungBot::makeBody( const Matrix& pose, const double mass, const std::vector<double> dimension )
    {
        // Allocate object
        auto bodyPart = new Box( dimension[0], dimension[1], dimension[2] );
        // Set texture from Image library
        bodyPart->setTexture( "Images/chess.rgb");
        // Initialize the primitive
        bodyPart->init( odeHandle, mass, osgHandle );
        // Set pose
        bodyPart->setPose( pose );
        // Add to objects
        objects.push_back( bodyPart );

        return bodyPart;
    }

    lpzrobots::Primitive* DungBot::makeLegPart( const osg::Matrix& pose, const double mass, const double legRadius, const double legHeight)
    {
    	// Allocate object
    	//lpzrobots::Primitive* leg = new Capsule( legRadius, legHeight );
    	lpzrobots::Primitive* leg = new Cylinder( legRadius, legHeight );
    	// Set texture from Image library
    	leg->setTexture( "Images/red_velour.rgb" );
    	// Initialize the primitive
    	leg->init( odeHandle, mass, osgHandle );
    	// Set pose
    	leg->setPose( pose );
    	// Add to objects
    	objects.push_back( leg );

    	return leg;
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

    lpzrobots::Primitive* DungBot::makeLegSphereJoint( const osg::Matrix& pose, const double mass, const double sphereRadius )
    {
    	// Allocate object
    	lpzrobots::Primitive* sphereJoint = new Sphere( sphereRadius );
    	// Set texture from Image library
    	sphereJoint->setTexture( "Images/sandyground.rgb" );
    	// Initialize the primitive
    	sphereJoint->init( odeHandle, mass, osgHandle );
    	// Set pose
    	sphereJoint->setPose( pose );
    	// Add to objects
    	objects.push_back( sphereJoint );

    	return sphereJoint;
    }


}
