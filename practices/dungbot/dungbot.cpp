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
	DungBot::Leg::Leg() {
	  tcJoint = 0;
	  ctJoint = 0;
	  ftJoint = 0;
	  footJoint = 0;
	  //tcServo = 0;
	  //ctrServo = 0;
	  //ftiServo = 0;
	  //footSpring = 0;
	  shoulder = 0;
	  coxa = 0;
	  femur = 0;
	  tibia = 0;
	  foot = 0;
	}

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

        initialPose = osg::Matrix::translate( Vec3( 0, 0, 0.5 ) * pose );
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

		// representation of the origin
		const Pos nullpos(0,0,0);

		//Place the joint between the two body-parts
		makeBodyHingeJoint( front, rear, nullpos*osg::Matrix::translate( -conf.frontDimension[0] / 2, 0, 0 ) * frontPos, Axis( 0, 1, 0 ) * frontPos, conf.rearDimension[1] );

	    /************************************
	     * LEGS
	     ***********************************/
		/* SE --> http://www.manoonpong.com/paper/2015/SWARM_2015_DungBeetleRobot.pdf */

		/* New leg making, based on dungbeetle.cpp */
		const double coxaLength  = conf.coxaLength;
		const double coxaRadius  = conf.coxaRadius;
		const double femurLength = conf.femurLength;
		const double femurRadius = conf.femurRadius;
		const double tebiaLength = conf.tebiaLength; //- 2 * conf.tebiaRadius - conf.footRange;
		const double tebiaRadius = conf.tebiaRadius;
		const double footLength  = 2 * conf.tebiaRadius + conf.footRange - conf.footRadius;
		const double footRadius  = conf.footRadius;
		double xPosition=0, yPosition=0, zPosition=0;

		std::map<LegPos, osg::Matrix> legtrunkconnections;


		// The purpose of this for-loop is to get all the leg-trunk-connections
		for (int i = 0; i < LEG_POS_MAX; i++) // Run through all of the legs
		{
			LegPos leg = LegPos(i);	// Give a value to leg (0-6), then if (leg == 'something') is true = 1

			// Make the right legs have a negative sign
			const double lr = (leg == L0 || leg == L1 || leg == L2) - (leg == R0 || leg == R1 || leg == R2);
			// Save hind legs
			const double lr2= leg==L1 || leg==R1 || leg==L2 || leg==R2;

			// create 3d-coordinates for the leg-trunk connection:
			switch (i) {
			  case L0:
			  case R0:
				  xPosition = conf.frontDimension[0]/2;
				  yPosition = lr * conf.frontDimension[1]/2;
				  zPosition = -conf.frontDimension[2]/2;
				  break;
			  case L1:
			  case R1:
				  xPosition = -conf.rearDimension[0]/3;
				  yPosition = lr * conf.rearDimension[1]/2;
				  zPosition = -conf.rearDimension[2]/2;
				  break;
			  case L2:
			  case R2:
				  xPosition = (-2*conf.rearDimension[0])/3;
				  yPosition = lr * conf.rearDimension[1]/2;
				  zPosition = -conf.rearDimension[2]/2;
				  break;
			  default:
				  xPosition = 0;
			  }
			Pos pos = Pos(xPosition,yPosition,zPosition);

		    // Now the rotation. lr2 rotates the four hind legs (PI/2)/2 in Y...
			legtrunkconnections[leg] = osg::Matrix::rotate(M_PI/2 , lr, 0, 0) * osg::Matrix::translate(pos) * pose;
		}

		// Some rotations.. Examine further
		/*legtrunkconnections[R2] = osg::Matrix::rotate(M_PI/2, 0, 0, 1) * osg::Matrix::rotate(M_PI/2, 1, 0, 0)
			* osg::Matrix::rotate(M_PI/2, 1, 1, 0) * legtrunkconnections[R2];
		legtrunkconnections[L2] = osg::Matrix::rotate(M_PI/2, 0, 0, -1) * osg::Matrix::rotate(M_PI/2, -1, 0, 0)
			* osg::Matrix::rotate(M_PI/2, 0, 1, 0) * legtrunkconnections[L2];
		legtrunkconnections[R1] = osg::Matrix::rotate(M_PI/2, 0, 0, 1) * osg::Matrix::rotate(M_PI/2, 1, 0, 0)
			* osg::Matrix::rotate(M_PI/2, 0, 1, 0) * legtrunkconnections[R1];
		legtrunkconnections[L1] = osg::Matrix::rotate(M_PI/2, 0, 0, -1) * osg::Matrix::rotate(M_PI/2, -1, 0, 0)
			* osg::Matrix::rotate(M_PI/2, 0, 1, 0) * legtrunkconnections[L1];
		legtrunkconnections[R0] = osg::Matrix::rotate(M_PI/2, 0, 0, 1) * osg::Matrix::rotate(M_PI/2+100, 1, 0, 0)
			  * osg::Matrix::rotate(M_PI/2, 0, 1, 0) * legtrunkconnections[R0];
		legtrunkconnections[L0] = osg::Matrix::rotate(M_PI/2, 0, 0, -1) * osg::Matrix::rotate(M_PI/2+100, -1, 0, 0)
			  * osg::Matrix::rotate(M_PI/2, 0, 1, 0) * legtrunkconnections[L0];*/

	    // create the legs
	    for (int i = 0; i < LEG_POS_MAX; i++)
	    {
			LegPos leg = LegPos(i);

			// +1 for R1,R2,R3, -1 for L1,L2,tebiaRadius
			const double pmrl = (leg == R0 || leg == R1 || leg == R2) - (leg == L0 || leg == L1 || leg == L2);

			//first Coxa position
			osg::Matrix c1 = legtrunkconnections[leg];

			// Coxa placement
			osg::Matrix CoaxCenter = osg::Matrix::translate(0, 0, -coxaLength / 2) * c1; //Position of Center of Mass
			Primitive* coxaThorax = new Capsule(coxaRadius, coxaLength);
			coxaThorax->setTexture("coxa.jpg");
			coxaThorax->init(odeHandle, 1, osgHandle); // TODO: 1 should be coxa mass
			coxaThorax->setPose(CoaxCenter);
			legs[leg].coxa = coxaThorax;
			objects.push_back(coxaThorax);

			// Femur placement
			osg::Matrix c2 = osg::Matrix::translate(0, 0, -coxaLength/2 ) * CoaxCenter;//coxaLength/2
			osg::Matrix femurcenter = osg::Matrix::translate(0, 0, -femurLength / 2) * c2;
			Primitive* femurThorax = new Capsule(femurRadius, femurLength);
			femurThorax->setTexture("femur.jpg");
			femurThorax->init(odeHandle, 1, osgHandle); // TODO: 1 should be mass
			femurThorax->setPose(femurcenter);
			legs[leg].femur = femurThorax;
			objects.push_back(femurThorax);

			// Tibia placement
			Primitive* tebia = new Capsule(tebiaRadius, tebiaLength);
			osg::Matrix c3 = osg::Matrix::translate(0, 0, -femurLength / 2) * femurcenter;
			osg::Matrix tibiaCenter = osg::Matrix::translate(0, 0, -tebiaLength / 2) * c3;
			tebia->setTexture("tibia.jpg");
			tebia->init(odeHandle, 1, osgHandle); // TODO: 1 should be tebia mass
			tebia->setPose(tibiaCenter);
			legs[leg].tibia = tebia;
			objects.push_back(tebia);

			// calculate anchor and axis of the first joint
			const osg::Vec3 anchor1 = nullpos * c1;
			const Axis axis1 = Axis(0,0,1) * c1;

			// Proceed along the leg (and the respective z-axis) for second limb
			const osg::Vec3 anchor2 = nullpos * c2;
			const Axis axis2 = Axis(pmrl,0,0) * c2;

			//and third
			const osg::Vec3 anchor3 = nullpos * c3;
			const Axis axis3 = Axis(pmrl,0,0) * c3;

			// hingeJoint to first limb
			HingeJoint* j = new HingeJoint((leg == L0 || leg == R0) ? front : rear, coxaThorax, anchor1, -axis1); // Only L0 and R0 should be attached to front
			j->init(odeHandle, osgHandle.changeColor("joint"), true, coxaRadius * 2.1);
			joints.push_back(j);

			// create the joint from first to second limb (coxa to second)
			HingeJoint* k = new HingeJoint(coxaThorax, femurThorax, anchor2, -axis2);
			k->init(odeHandle, osgHandle.changeColor("joint"), true, coxaRadius * 2.1);
			legs[leg].ctJoint = k;
			joints.push_back(k);


			// springy knee joint
			HingeJoint* l = new HingeJoint(femurThorax, tebia, anchor3, -axis3);
			l->init(odeHandle, osgHandle.changeColor("joint"), true, tebiaRadius * 2.1);
			legs[leg].ftJoint = l;
			joints.push_back(l);

	        /*// Spring foot at the end // true = use foot
			if (false)
			{
				osg::Matrix c4 = osg::Matrix::translate(0, 0, -tebiaLength / 2 - 2 * conf.tebiaRadius - conf.footRange + conf.footRadius) * tibiaCenter;
				osg::Matrix m4 = osg::Matrix::translate(0, 0, 0.0) * c4; //0.0 should be footspringpreload

				const osg::Vec3 anchor4 = nullpos * m4;
				const Axis axis4 = Axis(0, 0, -1) * c4;

				OdeHandle my_odeHandle = odeHandle;
				if (true)
				{		// True = rubber feet
					const Substance FootSubstance(3.0, 0.0, 500.0, 0.1);
					my_odeHandle.substance = FootSubstance;
				}

				Primitive* foot;
				foot = new Capsule(footRadius, footLength);
				foot->setTexture("Images/fur2.jpg");
				foot->init(my_odeHandle, 1, osgHandle); // TODO 1 should be food mass
				foot->setPose(m4);
				legs[leg].foot = foot;
				objects.push_back(foot);

				SliderJoint* m = new SliderJoint(tebia, foot, anchor4, axis4);
				m->init(odeHandle, osgHandle.changeColor("joint"), true, tebiaRadius, true);
				legs[leg].footJoint = m;
				joints.push_back(m);

				odeHandle.addIgnoredPair(femurThorax, foot);
			}*/
	    }
    }

    lpzrobots::Primitive* DungBot::makeBody( const Matrix& pose, const double mass, const std::vector<double> dimension )
    {
        // Allocate object
        auto bodyPart = new Box( dimension[0], dimension[1], dimension[2] );
        // Set texture from Image library
        bodyPart->setTexture( "body.jpg");
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
