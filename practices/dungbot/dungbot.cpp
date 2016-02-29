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
	DungBot::Leg::Leg()
	{
		tcJoint = 0;
		ctJoint = 0;
		ftJoint = 0;
		footJoint = 0;
		tcServo = 0;
		ctrServo = 0;
		ftiServo = 0;
		footSpring = 0;
		shoulder = 0;
		coxa = 0;
		femur = 0;
		tibia = 0;
		foot = 0;
	}

    DungBot::DungBot( const OdeHandle& odeHandle, const OsgHandle& osgHandle, const DungBotConf& conf, const string& name )
    : OdeRobot( odeHandle, osgHandle, name, "2.0" ), conf( conf )
    {
    	created = false;

    	//	Name the sensors
    	nameSensor(DungBotMotorSensor::TR0_as, "*TR0 angle sensor");
		nameSensor(DungBotMotorSensor::TR1_as, "*TR1 angle sensor");
		nameSensor(DungBotMotorSensor::TR2_as, "*TR2 angle sensor");
		nameSensor(DungBotMotorSensor::TL0_as, "*TL0 angle sensor");
		nameSensor(DungBotMotorSensor::TL1_as, "*TL1 angle sensor");
		nameSensor(DungBotMotorSensor::TL2_as, "*TL2 angle sensor");
		nameSensor(DungBotMotorSensor::CR0_as, "*CR0 angle sensor");
		nameSensor(DungBotMotorSensor::CR1_as, "*CR1 angle sensor");
		nameSensor(DungBotMotorSensor::CR2_as, "*CR2 angle sensor");
		nameSensor(DungBotMotorSensor::CL0_as, "*CL0 angle sensor");
		nameSensor(DungBotMotorSensor::CL1_as, "*CL1 angle sensor");
		nameSensor(DungBotMotorSensor::CL2_as, "*CL2 angle sensor");
		nameSensor(DungBotMotorSensor::FR0_as, "*FR0 angle sensor");
		nameSensor(DungBotMotorSensor::FR1_as, "*FR1 angle sensor");
		nameSensor(DungBotMotorSensor::FR2_as, "*FR2 angle sensor");
		nameSensor(DungBotMotorSensor::FL0_as, "*FL0 angle sensor");
		nameSensor(DungBotMotorSensor::FL1_as, "*FL1 angle sensor");
		nameSensor(DungBotMotorSensor::FL2_as, "*FL2 angle sensor");

        //	Name the motors
        nameMotor(DungBotMotorSensor::TR0_m, "TR0 motor");
        nameMotor(DungBotMotorSensor::TR1_m, "TR1 motor");
        nameMotor(DungBotMotorSensor::TR2_m, "TR2 motor");
        nameMotor(DungBotMotorSensor::TL0_m, "TL0 motor");
        nameMotor(DungBotMotorSensor::TL1_m, "TL1 motor");
        nameMotor(DungBotMotorSensor::TL2_m, "TL2 motor");
        nameMotor(DungBotMotorSensor::CR0_m, "CR0 motor");
        nameMotor(DungBotMotorSensor::CR1_m, "CR1 motor");
        nameMotor(DungBotMotorSensor::CR2_m, "CR2 motor");
        nameMotor(DungBotMotorSensor::CL0_m, "CL0 motor");
        nameMotor(DungBotMotorSensor::CL1_m, "CL1 motor");
        nameMotor(DungBotMotorSensor::CL2_m, "CL2 motor");
        nameMotor(DungBotMotorSensor::FR0_m, "FR0 motor");
        nameMotor(DungBotMotorSensor::FR1_m, "FR1 motor");
        nameMotor(DungBotMotorSensor::FR2_m, "FR2 motor");
        nameMotor(DungBotMotorSensor::FL0_m, "FL0 motor");
        nameMotor(DungBotMotorSensor::FL1_m, "FL1 motor");
        nameMotor(DungBotMotorSensor::FL2_m, "FL2 motor");
        nameMotor(DungBotMotorSensor::BJ_m, "BJ motor");
    }

    DungBot::~DungBot()
    {
    	// TODO: Should we add a delete func like in amos?
    }

    void DungBot::placeIntern( const Matrix& pose )
    {
        /**
            This is some (x,y,z) vector, with the position being the center of the body.
            So make some calculation for how the body looks like, so that the z-equation is
            in the third position.
        **/

        osg::Matrix initialPose = pose * osg::Matrix::translate(0, 0, conf.rearDimension[2]+conf.coxaRadius*1.2);
        create( initialPose );
    }

    void DungBot::doInternalStuff( GlobalData& globalData )
    {
    	OdeRobot::doInternalStuff(globalData);
    	    // update statistics
    	position = getPosition();
    }

    void DungBot::update( void )
    {
    	OdeRobot::update();
    	assert( created );

    	for( int i = 0; i < LEG_POS_MAX; i++ )
		{
			for( int j = 1; j < 6; j++ ) //TODO: Find out if j should be = 0 instead of 1.
			{
				if( tarsusContactSensors[ std::make_pair( LegPos(i), j ) ] )
				{
					tarsusContactSensors[ std::make_pair( LegPos(i), j ) ]->update();
				}
			}
		}
    }

    void DungBot::sense( GlobalData& globalData )
    {
    	OdeRobot::sense( globalData );

    	for( int i = 0; i < LEG_POS_MAX; i++ )
		{
			for( int j = 1; j < 6; j++ )
			{
				if( tarsusContactSensors[ std::make_pair( LegPos(i), j ) ] )
				{
					tarsusContactSensors[ std::make_pair( LegPos(i), j ) ]->sense( globalData );
				}
			}
		}
    }

    void DungBot::create( const Matrix& pose )
    {
    	assert( !created );

    	odeHandle.createNewSimpleSpace(parentspace, false);

    	std::cout << "FRONT MASS:\t" << conf.massFront << std::endl;
    	std::cout << "REAR MASS:\t" << conf.massRear << std::endl;
    	std::cout << "COXA MASS:\t" << conf.coxaMass[0] << " " << conf.coxaMass[1] << " " << conf.coxaMass[2] << " " << std::endl;
    	std::cout << "FEMUR MASS:\t" << conf.femurMass[0] << " " << conf.femurMass[1] << " " << conf.femurMass[2] << " " << std::endl;
    	std::cout << "TIBIA MASS:\t" << conf.tibiaMass[0] << " " << conf.tibiaMass[1] << " " << conf.tibiaMass[2] << " " << std::endl;
    	std::cout << "FOOT MASS:\t" << conf.footMass << std::endl;
    	std::cout << "TARUS MASS:\t" << conf.tarusMass << std::endl;

    	/************************************
         * BODY PARTS
         ***********************************/
		//	First we find the pose for the center of the body-part to be made, then we run the functions that creates it.
    	osg::Matrix rearPos = osg::Matrix::translate( ( conf.rearDimension[0] / 2 ), 0, 0) * pose;
    	auto rear = makeBody( rearPos, conf.massRear, conf.rearDimension );

    	osg::Matrix frontPos = osg::Matrix::translate( ( -conf.frontDimension[0] / 2 ), 0, 0) * pose;
		auto front = makeBody( frontPos, conf.massFront, conf.frontDimension );

		osg::Matrix headPos = osg::Matrix::translate( ( -conf.frontDimension[0] ), 0, 0) * pose;
		auto head = makeHead( headPos, conf.massHead, conf.headDimension );

		//	Representation of the origin
		const Pos nullpos(0,0,0);

		//	Place the joint between the two body-parts
		makeBodyHingeJoint( front, rear, nullpos*osg::Matrix::translate( conf.frontDimension[0] / 2, 0, 0 ) * frontPos, Axis( 0, 1, 0 ) * frontPos, conf.rearDimension[1] );
		makeHeadHingeJoint( front, head, nullpos*osg::Matrix::translate( -conf.frontDimension[0] / 2, 0, 0 ) * frontPos, Axis( 0, 1, 0 ) * headPos, conf.headDimension[1] );

	    /************************************
	     * Make all the legs
	     ***********************************/
		makeAllLegs( pose , rear, front );

		/************************************
		 * 	Set all the parameters
		 ***********************************/
		setParam("dummy", 0); // apply all parameters.

		created = true;
    }

    void DungBot::makeAllLegs( const Matrix& pose, Primitive* rear, Primitive* front)
    {
    	// representation of the origin
    	const Pos nullpos(0,0,0);
		double xPosition=0, yPosition=0, zPosition=0;
		std::map<LegPos, osg::Matrix> legTrunkConnections;

		// The purpose of this for-loop is to get all the leg-trunk-connections
		for (int i = 0; i < LEG_POS_MAX; i++) // Run through all of the legs
		{
			LegPos leg = LegPos(i);	// Give a value to leg (0-6), then if (leg == 'something') is true = 1

			// Make the right legs have a negative sign
			const double lr = (leg == L0 || leg == L1 || leg == L2) - (leg == R0 || leg == R1 || leg == R2);
			// Hind legs
			//const double lr2 = leg==L1 || leg==R1 || leg==L2 || leg==R2;

			// create 3d-coordinates for the leg-trunk connection:
			switch (i)
			{
				case L0:
				case R0:
					xPosition = -conf.frontDimension[0]/2+(sin(45)/sin(90)*conf.coxaLength[0]);
					yPosition = lr * conf.frontDimension[1]*( 0.25 );
					zPosition = -(conf.frontDimension[2]/2+1.2*conf.coxaRadius);
					break;
				case L1:
				case R1:
					xPosition = conf.rearDimension[0]*0.07;
					yPosition = lr * conf.rearDimension[1]*0.265;
					zPosition = -(conf.rearDimension[2]/2+1.2*conf.coxaRadius);
					break;
				case L2:
				case R2:
					//xPosition = conf.rearDimension[0]*0.3;
					xPosition = conf.rearDimension[0]*0.35;
					yPosition = lr * conf.rearDimension[1]*0.412;
					zPosition = -(conf.rearDimension[2]/2+1.2*conf.coxaRadius);
					break;
				default:
					xPosition = 0;
			}

			Pos pos = Pos( xPosition, yPosition, zPosition );

			legTrunkConnections[leg] = osg::Matrix::translate(pos) * pose;
		}

		std::vector<Primitive*> tarsusParts;

	    // create the legs
	    for (int i = 0; i < LEG_POS_MAX; i++)
	    {
			LegPos leg = LegPos(i);

			//	+1 for R0,R1,R2, -1 for L0,L1,L2
	        const double backLeg = ( leg == R1 || leg == R2 ) - ( leg == L1 || leg == L2 );
	        const double backLegInverse = ( leg == R1 || leg == R2 ) + ( leg == L1 || leg == L2 );
	        const double frontLeg = ( leg == R0 ) - ( leg == L0 );
	        const double frontLegInverse = ( leg == R0 ) + ( leg == L0 );
	        const double fb = (leg == L0 || leg == R0) - (leg == L1 || leg == L2 || leg == R1 || leg == R2);
	        const double lr = (leg == L0 || leg == L1 || leg == L2) - (leg == R0 || leg == R1 || leg == R2);
	        const double backLegOnly = ( leg == L1 || leg == L2 || leg == R1 || leg == R2 );
	        const double frontLegOnly = ( leg == R0 || leg == L0 );


			// Coxa placement
	        osg::Matrix c1 = osg::Matrix::rotate( -M_PI/2, lr, 0 , 0 ) *
	        					osg::Matrix::rotate( M_PI/4, 0, 0, backLegOnly*lr ) *
	        					osg::Matrix::rotate( -M_PI/4, 0, 0, frontLegOnly*lr ) *
	        					legTrunkConnections[leg];
			osg::Matrix coxaCenter = osg::Matrix::translate(0,0,-conf.coxaLength[i%3]/2)*c1; //Position of Center of Mass
			Primitive* coxaThorax = new Capsule( conf.coxaRadius, conf.coxaLength[i%3] );
			coxaThorax->setTexture( "coxa1.jpg");
			coxaThorax->init( odeHandle, conf.coxaMass[i%3], osgHandle );
			coxaThorax->setPose( coxaCenter );
			legs[leg].coxa = coxaThorax;
			objects.push_back( coxaThorax );

			// Femur placement
			osg::Matrix c2 = osg::Matrix::rotate( M_PI/2, lr, 0 , 0 ) *	//Anti
								osg::Matrix::rotate( -M_PI/4, 0, 0, backLegOnly*lr ) * //Anti
								osg::Matrix::rotate( M_PI/4, 0, 0, frontLegOnly*lr ) * //Anti
								osg::Matrix::rotate( M_PI/4, 0, fb, 0 ) *
								//osg::Matrix::rotate( M_PI/7, 0, fb, 0 ) * //Visual
								osg::Matrix::translate( 0, 0, -conf.coxaLength[i%3]/2 ) *
								coxaCenter;
			osg::Matrix femurcenter = osg::Matrix::translate( 0, 0, -conf.femurLength[i%3] / 2 ) * c2;
			Primitive* femurThorax = new Capsule( conf.femurRadius, conf.femurLength[i%3]  );
			femurThorax->setTexture( "femur.jpg" );
			femurThorax->init( odeHandle, conf.femurMass[i%3], osgHandle );
			femurThorax->setPose( femurcenter );
			legs[leg].femur = femurThorax;
			objects.push_back( femurThorax );

			// Tibia placement
			osg::Matrix c3 = //osg::Matrix::rotate( M_PI/6, 0, -1, 0 ) * //Visual
								//osg::Matrix::rotate( -M_PI/8, backLeg, 0, 0 ) * //Visual
								//osg::Matrix::rotate( M_PI/8, frontLeg, 0, 0 ) * //Visual
								osg::Matrix::translate( 0, 0, -conf.femurLength[i%3] / 2 ) *
								femurcenter;
			osg::Matrix tibiaCenter = osg::Matrix::translate( 0, 0, -conf.tibiaLength[i%3] / 2 ) * c3;
			Primitive* tibia = new Capsule( conf.tibiaRadius, conf.tibiaLength[i%3] );
			tibia->setTexture( "tebia.jpg" );
			tibia->init( odeHandle, conf.tibiaMass[i%3], osgHandle );
			tibia->setPose( tibiaCenter );
			legs[leg].tibia = tibia;
			objects.push_back( tibia );

			// calculate anchor and axis of the first joint
			const osg::Vec3 anchor1 = nullpos * c1;
	        Axis axis1 = Axis( 0, 0, backLeg+frontLeg )*c1;
	        switch (i)
	        {
case 0: axis1=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis1;	//front left
break;
case 1: axis1=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis1;	//middle left
break;
case 2: axis1=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis1; //rear left
break;
case 3: axis1=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis1;	//front right
break;
case 4: axis1=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis1;	// middle right
break;
case 5: axis1=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis1;	// rear right
break;
default: axis1=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis1;
break;
	        }

			// Proceed along the leg (and the respective z-axis) for second limb
			const osg::Vec3 anchor2 = nullpos * c2;
			Axis axis2 = Axis( backLeg+frontLeg, 0, 0 );
			switch (i)
			{
case 0: axis2=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis2;	//front left
break;
case 1: axis2=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis2;
break;
case 2: axis2=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis2;
break;
case 3: axis2=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis2;	//front right
break;
case 4:	axis2=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis2;
break;
case 5: axis2=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis2;
break;
default: axis2=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis2;
break;
			}

			//and third
			const osg::Vec3 anchor3 = nullpos * c3;
	        Axis axis3 = Axis( backLeg+frontLeg, 0, 0 );
	        switch (i)
	        {
case 0: axis3=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis3;//front left
break;
case 1: axis3=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis3;//middle left
break;
case 2: axis3=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis3; //rear left
break;
case 3: axis3=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis3; //front right
break;
case 4: axis3=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis3;  //middle right
break;
case 5: axis3=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis3; //rear right
break;
default:axis3=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis3;
break;
	        }

	        //	Torso coxa hinge joint.
	        if( conf.testCoxa || conf.testNo )
	        {
	        	HingeJoint* j = new HingeJoint( (leg == L0 || leg == R0) ? front : rear, coxaThorax, anchor1, -axis1 ); // Only L0 and R0 should be attached to front
				j->init( odeHandle, osgHandle.changeColor("joint"), true, conf.coxaRadius * 3.1 );
				legs[leg].tcJoint = j;
				joints.push_back( j );
				//OneAxisServo * coxaMotor = new OneAxisServoVel( odeHandle, j, -1.0, 1.0, 1.0, 0.01, 20.0, 1.0 );
				OneAxisServo * coxaMotor = new OneAxisServo( j, -1.0, 1.0, 1.0, 0.2, 2, 10.0, 1.3, true );
				legs[leg].tcServo = coxaMotor;
				servos[ getMotorName( leg, TC ) ] = coxaMotor;
	        }
	        else
	        {
	        	FixedJoint* j = new FixedJoint( (leg == L0 || leg == R0) ? front : rear, coxaThorax, anchor1 ); // Only L0 and R0 should be attached to front
	        	j->init( odeHandle, osgHandle.changeColor("joint"), true, conf.coxaRadius * 3.1 );
				joints.push_back( j );
	        }

	        // Coxa femur hinge joint.
	        if( conf.testFemur || conf.testNo )
	        {
	        	HingeJoint* k = new HingeJoint( coxaThorax, femurThorax, anchor2, -axis2 );
				k->init( odeHandle, osgHandle.changeColor("joint"), true, conf.coxaRadius * 3.1 );
				legs[leg].ctJoint = k;
				joints.push_back( k );
				//OneAxisServo * femurMotor = new OneAxisServoVel( odeHandle, k, -1.0, 1.0, 1.0, 0.01, 20.0, 1.0 );
				OneAxisServo * femurMotor = new OneAxisServo( k, -1.0, 1.0, 1.0, 0.2, 2, 10.0, 1.3, true );
				legs[leg].ctrServo = femurMotor;
				servos[ getMotorName( leg, CTR ) ] = femurMotor;
	        }
	        else
	        {
	        	FixedJoint* k = new FixedJoint( coxaThorax, femurThorax, anchor2 );
				k->init( odeHandle, osgHandle.changeColor("joint"), true, conf.coxaRadius * 3.1 );
				joints.push_back( k );
	        }

	        // Femur tibia hinge joint.
	        if( conf.testTibia || conf.testNo )
	        {
	        	HingeJoint* l = new HingeJoint( femurThorax, tibia, anchor3, -axis3 );
				l->init( odeHandle, osgHandle.changeColor("joint"), true, conf.tibiaRadius * 3.1 );
				legs[leg].ftJoint = l;
				joints.push_back( l );
				//OneAxisServo * tibiaMotor = new OneAxisServoCentered( l, -1.0, 1.0, 1.0, 0.01, 20.0, 1.0, 1.3 );
				OneAxisServo * tibiaMotor = new OneAxisServo( l, -1.0, 1.0, 1.0, 0.2, 2, 10.0, 1.3, true );
				legs[leg].ftiServo = tibiaMotor;
				servos[ getMotorName( leg, FTI ) ] = tibiaMotor;
	        }
	        else
	        {
	        	FixedJoint* l = new FixedJoint( femurThorax, tibia, anchor3 );
	        	l->init( odeHandle, osgHandle.changeColor("joint"), true, conf.tibiaRadius * 3.1 );
	        	joints.push_back( l );
	        }

			// Foot
			if( conf.makeFoot ) // toggle foot
			{
				osg::Matrix c4 = osg::Matrix::translate( 0, 0, -conf.tibiaLength[i%3] / 2 - conf.footRange ) * tibiaCenter;
				osg::Matrix footCenter = osg::Matrix::translate( 0, 0, -conf.footSpringPreload ) * c4;

				const osg::Vec3 anchor4 = nullpos * footCenter;
				const Axis axis4 = Axis(0, 0, -1) * c4;

				OdeHandle my_odeHandle = odeHandle;

				// set the foot to use rubberFeet
				const Substance FootSubstance( 3.0, 0.0, 500.0, 0.1 );
				my_odeHandle.substance = FootSubstance;

				Primitive* foot;
				foot = new Capsule( conf.footRadius, conf.footRange );
				foot->setTexture( "foot.jpg" );
				foot->init(my_odeHandle, conf.footMass, osgHandle);
				foot->setPose(footCenter);
				legs[leg].foot = foot;
				objects.push_back(foot);

				//SliderJoint* m = new SliderJoint( tibia, foot, anchor4, axis4 );
				HingeJoint* m = new HingeJoint( tibia, foot, anchor4, axis4 );	//TODO SHOULD BE A SliderJoint and not HingeJoint
				m->init( odeHandle, osgHandle.changeColor( "joint" ), true, conf.tibiaRadius, true );
				legs[leg].footJoint = m;
				joints.push_back( m );

				/** parameters are set later */
				Spring* spring = new Spring( m, -1, 1, 1, 0.05, 1, 0, 1 );
				legs[leg].footSpring = spring;
				passiveServos.push_back( spring );
				odeHandle.addIgnoredPair( femurThorax, foot );

				// New: tarsus
				Primitive *tarsus;
				double angle = M_PI/12;

				double radius = conf.footRadius/2; // was conf.footRadius/3
				double length = conf.footRange;
				double mass = conf.tarusMass/10;
				tarsus = new Capsule( radius,length );
				tarsus->setTexture("tarsus.jpg");
				tarsus->init(odeHandle, mass, osgHandle);

				osg::Matrix m6;
				osg::Matrix m5 = 	osg::Matrix::rotate(-angle,i%2==0 ? -1 : 1,0,0) *
									osg::Matrix::translate(0,0,-length/2) *
									footCenter;

				double angleTarsus=0;

				//	Rotate manually tarsus here
				switch (i)
				{	//TODO:	See if the rotations of the Tarsus is correct
					case 0: angleTarsus=0;	//front left
					break;
					case 1: angleTarsus=(M_PI/180)*300+M_PI+(M_PI/180)*20;	//middle left
					break;
					case 2: angleTarsus=0;	//rear left ok
					break;
					case 3: angleTarsus=(M_PI/180)*20;	//front right
					break;
					case 4: angleTarsus=(M_PI/180)*140;	// middle right
					break;
					case 5: angleTarsus=-(M_PI/180)*290;	// rear right ok
					break;
					default: angleTarsus=0;
					break;
				}

				if( i < 2 )
				{
					m6 = osg::Matrix::rotate(i%2==0 ? angle : -angle,0,i%2==0 ? -1 : 1,0) * m5;
				}
				else if( i > 3 )
				{
					m6 = osg::Matrix::rotate(i%2==0 ? -angle : angle,0,i%2==0 ? -1 : 1,0) * m5;
				}
				else
				{
					m6 = m5;
				}
				m6 = osg::Matrix::rotate(0,1,0,0) *osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(angleTarsus,0,0,1)* osg::Matrix::translate(0,0,-length/2) * m6 ;

				std::cout << "leg number     " << i << std::endl;

				tarsus->setPose(m6);
				tarsusParts.push_back(tarsus);
				objects.push_back(tarsus);

				FixedJoint* q = new FixedJoint(foot, tarsus);
				q->init(odeHandle, osgHandle, false);
				joints.push_back(q);

				Primitive *section = tarsus;

				for( int j = 1; j < 6; j++ )
				{
					 double lengthS = length/1.9;
					 double radiusS = radius/1.5;
					 section = new Capsule( radiusS, lengthS );
					 section->setTexture( "tarsus.jpg" );
					 section->init( odeHandle, mass, osgHandle );

					 osg::Matrix m7;

					 if( i < 2 )
					 {
						 m7 =	osg::Matrix::translate(0,0,-lengthS/2) *
								osg::Matrix::rotate(i%2==0 ? angle : -angle,0,i%2==0 ? -1 : 1,0) *
								osg::Matrix::rotate(i%2==0 ? angle : -angle,1,0,0) *
								osg::Matrix::translate(0,0,-length/2) *
								m6;
					 }
					 else if(i > 3)
					 {
						 m7 =	osg::Matrix::translate(0,0,-lengthS/2) *
								osg::Matrix::rotate(i%2==0 ? -angle : angle,0,i%2==0 ? -1 : 1,0) *
								osg::Matrix::rotate(i%2==0 ? angle : -angle,1,0,0) *
								osg::Matrix::translate(0,0,-length/2) *
								m6;
					 }
					 else
					 {
						 m7 =	osg::Matrix::translate(0,0,-lengthS/2) *
								osg::Matrix::rotate(i%2==0 ? angle : -angle,1,0,0) *
								osg::Matrix::translate(0,0,-length/2) *
								m6;
					 }
					 section->setPose(m7);
					 objects.push_back(section);
					 tarsusParts.push_back(section);

				 if( j == 1 )
				 {
					 HingeJoint* k = new HingeJoint(tarsusParts[j-1], tarsusParts[j], Pos(0,0,length/3) * m7, Axis(i%2==0 ? -1 : 1,0,0) * m7);
					 k->init(odeHandle, osgHandle, true, lengthS/16 * 2.1);

					 // servo used as a spring
					 auto servo = std::make_shared<OneAxisServoVel>(odeHandle,k, -1, 1, 1, 0.05); // parameters are set later
					 joints.push_back(k);
					 auto spring = std::make_shared<ConstantMotor>(servo, 0.0);
					 tarsussprings.push_back(servo);
					 addMotor(spring);
				 }
				 else if( j == 2 )
				 {
					 HingeJoint* k = new HingeJoint(tarsusParts[j-1], tarsusParts[j], Pos(0,0,length/3) * m7, Axis(i%2==0 ? -1 : 1,0,0) * m7);
					 k->init(odeHandle, osgHandle, true, lengthS/16 * 2.1);

					 // servo used as a spring
					 auto servo = std::make_shared<OneAxisServoVel>(odeHandle,k, -1, 1, 1, 0.05); // parameters are set later
					 joints.push_back(k);
					 auto spring = std::make_shared<ConstantMotor>(servo, 0.0);
					 tarsussprings.push_back(servo);
					 addMotor(spring);
				 }
				 else
				 {
					 HingeJoint* k = new HingeJoint(tarsusParts[j-1], tarsusParts[j], Pos(0,0,length/3) * m7, Axis(i%2==0 ? -1 : 1,0,0) * m7);
					 k->init(odeHandle, osgHandle, true, lengthS/16 * 2.1);

					 // servo used as a spring
					 auto servo = std::make_shared<OneAxisServoVel>( odeHandle, k, -1, 1, 1, 0.01 ); // parameters are set later
					 joints.push_back( k );
					 auto spring = std::make_shared<ConstantMotor>( servo, 0.0 );
					 tarsussprings.push_back( servo );
					 addMotor( spring );
				 }

				 m6 = m7;

				 tarsusContactSensors[ std::make_pair( LegPos(i), j) ] = new ContactSensor(true, 65, 1.5 * radiusS, false, true, Color(1,9,3));
				 tarsusContactSensors[ std::make_pair( LegPos(i), j) ]->setInitData(odeHandle, osgHandle, osg::Matrix::translate(0, 0, -(0.5) * lengthS));
				 tarsusContactSensors[ std::make_pair( LegPos(i), j) ]->init(tarsusParts.at(j));
				}
			}
			tarsusParts.clear();
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
    	if( conf.testBody || conf.testNo )
    	{
    		HingeJoint* hinge = new HingeJoint( frontLimb, rearLimb, position, axis );
			hinge->init( odeHandle, osgHandle, true, Y * 1.05 );
			joints.push_back( hinge );
			//OneAxisServo * bodyMotor = new OneAxisServoVel( odeHandle, hinge, -1.0, 1.0,1.0, 0.01, 20.0, 1.0 );
			OneAxisServo * bodyMotor = new OneAxisServo( hinge, -1.0, 1.0, 1.0, 0.2, 2, 10.0, 1.3, true );
			servos[DungBotMotorSensor::BJ_m] = bodyMotor;
			backboneServo = bodyMotor;
    	}
    	else
    	{
    		FixedJoint* hinge = new FixedJoint( frontLimb, rearLimb, position );
    		hinge->init( odeHandle, osgHandle, false, Y * 1.05 );
			joints.push_back( hinge );
    	}
    }

    void DungBot::makeHeadHingeJoint( Primitive* frontLimb, Primitive* rearLimb, const Pos position, Axis axis, const double Y )
	{
    	if( conf.testHead || conf.testNo )
		{
			HingeJoint* hinge = new HingeJoint( frontLimb, rearLimb, position, axis );
			hinge->init( odeHandle, osgHandle, true, Y * 1.05 );
			joints.push_back( hinge );
			//OneAxisServo * headMotor = new OneAxisServoVel( odeHandle, hinge, -1.0, 1.0,1.0, 0.01, 20.0, 1.0 );
			OneAxisServo * headMotor = new OneAxisServo( hinge, -1.0, 1.0, 1.0, 0.2, 2, 10.0, 1.3, true );
			servos[DungBotMotorSensor::HJ_m] = headMotor;
			headServo = headMotor;
		}
		else
		{
			FixedJoint* hinge = new FixedJoint( frontLimb, rearLimb, position );
			hinge->init( odeHandle, osgHandle, false, Y * 1.05 );
			joints.push_back( hinge );
		}
	}

    void DungBot::makeFixedJoint(Primitive* frontLimb, Primitive* rearLimb, const Pos position, const double Y)
    {
		FixedJoint* fixed = new FixedJoint( frontLimb, frontLimb, position);
		fixed->init( odeHandle, osgHandle, false, Y * 1.00 );
		joints.push_back( fixed );
    }

    void DungBot::makeHingeJoint( Primitive* frontLimb, Primitive* rearLimb, const Pos position, Axis axis, const double Y )
    {
    	FixedJoint* hinge = new FixedJoint( frontLimb, rearLimb, position );
        hinge->init( odeHandle, osgHandle, false, Y * 1.05 );
        joints.push_back( hinge );
    }

    lpzrobots::Primitive* DungBot::makeHead( const osg::Matrix& pose, const double mass, const std::vector<double> dimension )
    {
    	// Allocate object
    	lpzrobots::Primitive* head = new Cylinder( dimension[2], dimension[0] );
    	// Set texture from Image library
    	head->setTexture( "body.jpg" );
    	// Initialize the primitive
    	head->init( odeHandle, mass, osgHandle );
    	// Set pose
    	head->setPose( pose );
    	// Add to objects
    	objects.push_back( head );

    	return head;
    }

	void DungBot::nameMotor( const int motorNumber, const char* name )
	{
		addInspectableDescription( "y[" + std::itos( motorNumber ) + "]", name );
	}
	void DungBot::nameSensor( const int sensorNumber, const char* name )
	{
		addInspectableDescription( "x[" + std::itos( sensorNumber ) + "]", name );
	}

	void DungBot::setMotorsIntern( const double* motors, int motorNumber )
	{
		assert( created );
		assert( motorNumber >= getMotorNumberIntern() );
		for( MotorMap::iterator it = servos.begin(); it != servos.end(); it++ )
		{
			MotorName const name = it->first;
			OneAxisServo * const servo = it->second;
			//We multiple with -1 to map to real hexapod
			if( servo )
			{
				servo->set( -motors[ name ] );
			}
		}
	}

	int DungBot::getSensorsIntern( double* sensors, int sensorNumber )
	{
		assert( created );
		assert( sensorNumber >= getSensorNumberIntern() );

		//	Angle sensors
		//	We multiple with -1 to map to real hexapod
		sensors[DungBotMotorSensor::TR0_as] = servos[DungBotMotorSensor::TR0_m] ? -servos[DungBotMotorSensor::TR0_m]->get() : 0;
		sensors[DungBotMotorSensor::TR1_as] = servos[DungBotMotorSensor::TR1_m] ? -servos[DungBotMotorSensor::TR1_m]->get() : 0;
		sensors[DungBotMotorSensor::TR2_as] = servos[DungBotMotorSensor::TR2_m] ? -servos[DungBotMotorSensor::TR2_m]->get() : 0;
		sensors[DungBotMotorSensor::TL0_as] = servos[DungBotMotorSensor::TL0_m] ? -servos[DungBotMotorSensor::TL0_m]->get() : 0;
		sensors[DungBotMotorSensor::TL1_as] = servos[DungBotMotorSensor::TL1_m] ? -servos[DungBotMotorSensor::TL1_m]->get() : 0;
		sensors[DungBotMotorSensor::TL2_as] = servos[DungBotMotorSensor::TL2_m] ? -servos[DungBotMotorSensor::TL2_m]->get() : 0;
		sensors[DungBotMotorSensor::CR0_as] = servos[DungBotMotorSensor::CR0_m] ? -servos[DungBotMotorSensor::CR0_m]->get() : 0;
		sensors[DungBotMotorSensor::CR1_as] = servos[DungBotMotorSensor::CR1_m] ? -servos[DungBotMotorSensor::CR1_m]->get() : 0;
		sensors[DungBotMotorSensor::CR2_as] = servos[DungBotMotorSensor::CR2_m] ? -servos[DungBotMotorSensor::CR2_m]->get() : 0;
		sensors[DungBotMotorSensor::CL0_as] = servos[DungBotMotorSensor::CL0_m] ? -servos[DungBotMotorSensor::CL0_m]->get() : 0;
		sensors[DungBotMotorSensor::CL1_as] = servos[DungBotMotorSensor::CL1_m] ? -servos[DungBotMotorSensor::CL1_m]->get() : 0;
		sensors[DungBotMotorSensor::CL2_as] = servos[DungBotMotorSensor::CL2_m] ? -servos[DungBotMotorSensor::CL2_m]->get() : 0;
		sensors[DungBotMotorSensor::FR0_as] = servos[DungBotMotorSensor::FR0_m] ? -servos[DungBotMotorSensor::FR0_m]->get() : 0;
		sensors[DungBotMotorSensor::FR1_as] = servos[DungBotMotorSensor::FR1_m] ? -servos[DungBotMotorSensor::FR1_m]->get() : 0;
		sensors[DungBotMotorSensor::FR2_as] = servos[DungBotMotorSensor::FR2_m] ? -servos[DungBotMotorSensor::FR2_m]->get() : 0;
		sensors[DungBotMotorSensor::FL0_as] = servos[DungBotMotorSensor::FL0_m] ? -servos[DungBotMotorSensor::FL0_m]->get() : 0;
		sensors[DungBotMotorSensor::FL1_as] = servos[DungBotMotorSensor::FL1_m] ? -servos[DungBotMotorSensor::FL1_m]->get() : 0;
		sensors[DungBotMotorSensor::FL2_as] = servos[DungBotMotorSensor::FL2_m] ? -servos[DungBotMotorSensor::FL2_m]->get() : 0;
		sensors[DungBotMotorSensor::BJ_as] = servos[DungBotMotorSensor::BJ_m] ? -servos[DungBotMotorSensor::BJ_m]->get() : 0;

		sensors[DungBotMotorSensor::L0_s1] = tarsusContactSensors[std::make_pair(L0,1)]->get();
		sensors[DungBotMotorSensor::L0_s2] = tarsusContactSensors[std::make_pair(L0,2)]->get();
		sensors[DungBotMotorSensor::L0_s3] = tarsusContactSensors[std::make_pair(L0,3)]->get();
		sensors[DungBotMotorSensor::L0_s4] = tarsusContactSensors[std::make_pair(L0,4)]->get();
		sensors[DungBotMotorSensor::L0_s5] = tarsusContactSensors[std::make_pair(L0,5)]->get();

		sensors[DungBotMotorSensor::R0_s1] = tarsusContactSensors[std::make_pair(R0,1)]->get();
		sensors[DungBotMotorSensor::R0_s2] = tarsusContactSensors[std::make_pair(R0,2)]->get();
		sensors[DungBotMotorSensor::R0_s3] = tarsusContactSensors[std::make_pair(R0,3)]->get();
		sensors[DungBotMotorSensor::R0_s4] = tarsusContactSensors[std::make_pair(R0,4)]->get();
		sensors[DungBotMotorSensor::R0_s5] = tarsusContactSensors[std::make_pair(R0,5)]->get();

		sensors[DungBotMotorSensor::L1_s1] = tarsusContactSensors[std::make_pair(L1,1)]->get();
		sensors[DungBotMotorSensor::L1_s2] = tarsusContactSensors[std::make_pair(L1,2)]->get();
		sensors[DungBotMotorSensor::L1_s3] = tarsusContactSensors[std::make_pair(L1,3)]->get();
		sensors[DungBotMotorSensor::L1_s4] = tarsusContactSensors[std::make_pair(L1,4)]->get();
		sensors[DungBotMotorSensor::L1_s5] = tarsusContactSensors[std::make_pair(L1,5)]->get();

		sensors[DungBotMotorSensor::R1_s1] = tarsusContactSensors[std::make_pair(R1,1)]->get();
		sensors[DungBotMotorSensor::R1_s2] = tarsusContactSensors[std::make_pair(R1,2)]->get();
		sensors[DungBotMotorSensor::R1_s3] = tarsusContactSensors[std::make_pair(R1,3)]->get();
		sensors[DungBotMotorSensor::R1_s4] = tarsusContactSensors[std::make_pair(R1,4)]->get();
		sensors[DungBotMotorSensor::R1_s5] = tarsusContactSensors[std::make_pair(R1,5)]->get();

		sensors[DungBotMotorSensor::L2_s1] = tarsusContactSensors[std::make_pair(L2,1)]->get();
		sensors[DungBotMotorSensor::L2_s2] = tarsusContactSensors[std::make_pair(L2,2)]->get();
		sensors[DungBotMotorSensor::L2_s3] = tarsusContactSensors[std::make_pair(L2,3)]->get();
		sensors[DungBotMotorSensor::L2_s4] = tarsusContactSensors[std::make_pair(L2,4)]->get();
		sensors[DungBotMotorSensor::L2_s5] = tarsusContactSensors[std::make_pair(L2,5)]->get();

		sensors[DungBotMotorSensor::R2_s1] = tarsusContactSensors[std::make_pair(R2,1)]->get();
		sensors[DungBotMotorSensor::R2_s2] = tarsusContactSensors[std::make_pair(R2,2)]->get();
		sensors[DungBotMotorSensor::R2_s3] = tarsusContactSensors[std::make_pair(R2,3)]->get();
		sensors[DungBotMotorSensor::R2_s4] = tarsusContactSensors[std::make_pair(R2,4)]->get();
		sensors[DungBotMotorSensor::R2_s5] = tarsusContactSensors[std::make_pair(R2,5)]->get();

		return DungBotMotorSensor::DUNGBOT_SENSOR_MAX;
	}

	int DungBot::getMotorNumberIntern( void )
	{
		return DungBotMotorSensor::DUNGBOT_MOTOR_MAX;
	}

	int DungBot::getSensorNumberIntern( void )
	{
		return DungBotMotorSensor::DUNGBOT_SENSOR_MAX;
	}

	DungBot::MotorName DungBot::getMotorName( LegPos leg, LegJointType joint )
	{
		if (leg == L0 && joint == TC)	return DungBotMotorSensor::TL0_m;
		if (leg == L0 && joint == CTR)	return DungBotMotorSensor::CL0_m;
		if (leg == L0 && joint == FTI)	return DungBotMotorSensor::FL0_m;
		if (leg == L1 && joint == TC)	return DungBotMotorSensor::TL1_m;
		if (leg == L1 && joint == CTR)	return DungBotMotorSensor::CL1_m;
		if (leg == L1 && joint == FTI)	return DungBotMotorSensor::FL1_m;
		if (leg == L2 && joint == TC)	return DungBotMotorSensor::TL2_m;
		if (leg == L2 && joint == CTR)	return DungBotMotorSensor::CL2_m;
		if (leg == L2 && joint == FTI)	return DungBotMotorSensor::FL2_m;
		if (leg == R0 && joint == TC)	return DungBotMotorSensor::TR0_m;
		if (leg == R0 && joint == CTR)	return DungBotMotorSensor::CR0_m;
		if (leg == R0 && joint == FTI)	return DungBotMotorSensor::FR0_m;
		if (leg == R1 && joint == TC)	return DungBotMotorSensor::TR1_m;
		if (leg == R1 && joint == CTR)	return DungBotMotorSensor::CR1_m;
		if (leg == R1 && joint == FTI)	return DungBotMotorSensor::FR1_m;
		if (leg == R2 && joint == TC)	return DungBotMotorSensor::TR2_m;
		if (leg == R2 && joint == CTR)	return DungBotMotorSensor::CR2_m;
		if (leg == R2 && joint == FTI)	return DungBotMotorSensor::FR2_m;
		return DungBotMotorSensor::DUNGBOT_MOTOR_MAX;
	}

	bool DungBot::setParam( const paramkey& key, paramval val )
	{
	    bool rv = Configurable::setParam(key, val);

	    //	We set all parameters here
	    for( LegMap::iterator it = legs.begin(); it != legs.end(); it++ )
	    {
			OneAxisServo * tc = it->second.tcServo;
			if( tc )
			{
				tc->setPower(conf.coxaPower);
				tc->setDamping(conf.coxaDamping);
				tc->setMaxVel(conf.coxaMaxVel);
				if (it->first == L2 || it->first == R2) tc->setMinMax(conf.rCoxaJointLimitF, conf.rCoxaJointLimitB);
				if (it->first == L1 || it->first == R1) tc->setMinMax(conf.mCoxaJointLimitF, conf.mCoxaJointLimitB);
				if (it->first == L0 || it->first == R0) tc->setMinMax(conf.fCoxaJointLimitF, conf.fCoxaJointLimitB);
			}

			OneAxisServo * ctr = it->second.ctrServo;
			if (ctr)
			{
				ctr->setPower( conf.femurPower );
				ctr->setDamping( conf.femurDamping );
				ctr->setMaxVel( conf.femurMaxVel );
				//	Min is up, up is negative
				if (it->first == L2 || it->first == R2) ctr->setMinMax(conf.rFemurJointLimitU, conf.rFemurJointLimitD);
				if (it->first == L1 || it->first == R1) ctr->setMinMax(conf.rFemurJointLimitU, conf.mFemurJointLimitD);
				if (it->first == L0 || it->first == R0) ctr->setMinMax(conf.rFemurJointLimitU, conf.fFemurJointLimitD);
			}

			OneAxisServo * fti = it->second.ftiServo;
			if ( fti )
			{
				fti->setPower(conf.tibiaPower);
				fti->setDamping(conf.tibiaDamping);
				fti->setMaxVel(conf.tibiaMaxVel);
				//	Min is up, up is negative
				if (it->first == L2 || it->first == R2) fti->setMinMax(conf.rTibiaJointLimitU, conf.rTibiaJointLimitD);
				if (it->first == L1 || it->first == R1) fti->setMinMax(conf.mTibiaJointLimitU, conf.mTibiaJointLimitD);
				if (it->first == L0 || it->first == R0) fti->setMinMax(conf.fTibiaJointLimitU, conf.fTibiaJointLimitD);
			}

			Spring * const footspring = it->second.footSpring;
			if( footspring )
			{
				footspring->setPower(conf.footPower);
				footspring->setDamping(conf.footDamping);
				footspring->setMaxVel(conf.footMaxVel);
				//	Min is up, Max is down.
				footspring->setMinMax(conf.footSpringLimitD, conf.footSpringLimitU);
			}
		}

		if (backboneServo)
		{
			backboneServo->setPower(conf.backPower);
			backboneServo->setDamping(conf.backDamping);
			backboneServo->setMaxVel(conf.backMaxVel);
			backboneServo->setMinMax(conf.backJointLimitU, conf.backJointLimitD);
		}

		return rv;
	}

	DungBotConf DungBot::getDefaultConf( void )
	{
		DungBotConf conf;

		/**
		 * 	Test of the legs
		 */
		conf.testNo = false;	//	If true, then all hinges exist.
		conf.testHead = false;	//	If true, then Head hinges is made else fixed joints.
		conf.testBody = false;	//	If true, then Body hinges is made else fixed joints.
		conf.testCoxa = true;	//	If true, then Coxa hinges is made else fixed joints.
		conf.testFemur = true;	//	If true, then Femur hinges is made else fixed joints.
		conf.testTibia = true;	//	If true, then Tibia hinges is made else fixed joints.

		/**
		 * MATHIAS THOR's MASS CALCULATION (c) :D
		 * ---------------------------------------------------------------------
		 * 	The average density of the human body [ρ=kg/m³]: 985
		 * 	Mass of a capsule [m]: 	m = ρV = ρ π radius²((4/3)radius+height)
		 * 	Mass of a box [m]: 		m = ρV = ρ height width length
		 * 	eg. conf.coxaMass = 985*3.14*conf.coxaRadius*conf.coxaRadius*((4/3)*conf.coxaRadius+conf.coxaLength);
		 * ---------------------------------------------------------------------
		 */

		//	----------- Body dimensions -------
		conf.rearDimension 	= { 0.65, 0.65, 0.2 };	// Length and Width should be equal
		conf.frontDimension = { conf.rearDimension[0]*0.5, conf.rearDimension[1]*0.83, conf.rearDimension[2]*1 };
		conf.headDimension 	= { conf.frontDimension[0]*0.57, conf.frontDimension[0]*0.57, conf.rearDimension[2]*1 };

		//TODO:Set these after trip to Kiel
		conf.massFront = 0.01;
		conf.massRear = 0.01;
		conf.massHead = 0.01;

		// ------------ Leg dimensions --------
		conf.coxaLength = { conf.rearDimension[0]*0.177, conf.rearDimension[0]*0.3, conf.rearDimension[0]*0.45 };
		conf.coxaRadius = 0.02;
		conf.coxaMass = {
				9.85*M_PI*conf.coxaRadius*conf.coxaRadius*( (4/3)*conf.coxaRadius + conf.coxaLength[0] ),
				9.85*M_PI*conf.coxaRadius*conf.coxaRadius*( (4/3)*conf.coxaRadius + conf.coxaLength[1] ),
				9.85*M_PI*conf.coxaRadius*conf.coxaRadius*( (4/3)*conf.coxaRadius + conf.coxaLength[2] )};

		conf.femurLength = { conf.rearDimension[0]*0.50, conf.rearDimension[0]*0.55, conf.rearDimension[0]*0.55 };
		conf.femurRadius = 0.02;
		conf.femurMass = {
				9.85*M_PI*conf.femurRadius*conf.femurRadius*( (4/3)*conf.femurRadius + conf.femurLength[0] ),
				9.85*M_PI*conf.femurRadius*conf.femurRadius*( (4/3)*conf.femurRadius + conf.femurLength[1] ),
				9.85*M_PI*conf.femurRadius*conf.femurRadius*( (4/3)*conf.femurRadius + conf.femurLength[2] )};

		conf.tibiaLength = { conf.rearDimension[0]*0.679, conf.rearDimension[0]*0.50, conf.rearDimension[0]*0.70 };
		conf.tibiaRadius = 0.02;
		conf.tibiaMass = {
				9.85*M_PI*conf.tibiaRadius*conf.tibiaRadius*( (4/3)*conf.tibiaRadius + conf.tibiaLength[0] ),
				9.85*M_PI*conf.tibiaRadius*conf.tibiaRadius*( (4/3)*conf.tibiaRadius + conf.tibiaLength[1] ),
				9.85*M_PI*conf.tibiaRadius*conf.tibiaRadius*( (4/3)*conf.tibiaRadius + conf.tibiaLength[2] )};

		conf.footRange = 0.05;
		conf.footRadius = 0.02;
		conf.footMass = 0.09;
		conf.footSpringPreload = 0.0;

		conf.tarusMass = 0.008;

		/**
		 *	Joint Limits
		 *	Setting the Max, and Min values of each joint.
		 */
		conf.backJointLimitD = M_PI / 180 * 180.0;
		conf.backJointLimitU =	-M_PI / 180 * 180.0;

		//	TC JOINT
		conf.fCoxaJointLimitF = -M_PI / 180.0 * 40.0;	// 70 deg; forward (-) MAX --> normal walking range 60 deg MAX
		conf.fCoxaJointLimitB =  M_PI / 180.0 * 40.0;	//-70 deg; backward (+) MIN --> normal walking range -10 deg MIN
	    conf.mCoxaJointLimitF = -M_PI / 180.0 * 40.0;	// 60 deg; forward (-) MAX --> normal walking range 30 deg MAX
	    conf.mCoxaJointLimitB =  M_PI / 180.0 * 40.0;	// 60 deg; backward (+) MIN --> normal walking range -40 deg MIN
	    conf.rCoxaJointLimitF = -M_PI / 180.0 * 40.0;	// 70 deg; forward (-) MAX --> normal walking range 60 deg MAX
	    conf.rCoxaJointLimitB =  M_PI / 180.0 * 40.0;	// 70 deg; backward (+) MIN --> normal walking range -10 deg MIN
	    //	CT JOINT
	    conf.fFemurJointLimitD =  M_PI / 180.0 * 20.0;
	    conf.fFemurJointLimitU = -M_PI / 180.0 * 20.0;
	    conf.mFemurJointLimitD =  M_PI / 180.0 * 20.0;
	    conf.mFemurJointLimitU = -M_PI / 180.0 * 20.0;
	    conf.rFemurJointLimitD =  M_PI / 180.0 * 20.0;
	    conf.rFemurJointLimitU = -M_PI / 180.0 * 20.0;
	    //	FT JOINT
	    conf.fTibiaJointLimitD =  M_PI / 180.0 * 20.0;
	    conf.fTibiaJointLimitU = -M_PI / 180.0 * 20.0;
	    conf.mTibiaJointLimitD =  M_PI / 180.0 * 20.0;
	    conf.mTibiaJointLimitU = -M_PI / 180.0 * 20.0;
	    conf.rTibiaJointLimitD =  M_PI / 180.0 * 20.0;
	    conf.rTibiaJointLimitU = -M_PI / 180.0 * 20.0;

		/**
		 * 	Power of the motors, and joint stiffness
		 */

		conf.footSpringPreload = 8.0 / 43.0 * conf.size;
		// negative is downwards (spring extends)
		conf.footSpringLimitD = conf.footSpringPreload;
		conf.footSpringLimitU = conf.footSpringPreload + conf.footRange;

		const double backPower_scale = 30.0;
		const double coxaPower_scale = 10.0;
		const double springstiffness = 35.0;

		conf.backPower = backPower_scale * (1.962 / (0.035 * 2.2)) * conf.coxaLength[0] * conf.massRear;
		conf.coxaPower = coxaPower_scale * (1.962 / (0.035 * 2.2)) * conf.coxaLength[0] * conf.massRear;
		conf.femurPower = conf.coxaPower;
		conf.tibiaPower = conf.coxaPower;
		conf.footPower = ( springstiffness * 0.08 / 2.2 ) * conf.massRear / conf.footSpringPreload;

		std::cout << "Back power: "<< conf.backPower << std::endl;
		std::cout << "Coxa power: "<< conf.coxaPower << std::endl;
		std::cout << "Femur power: "<< conf.femurPower << std::endl;
		std::cout << "Tibia power: "<< conf.backPower << std::endl;
		std::cout << "Foot power: "<< conf.tibiaPower << std::endl;

		conf.backDamping = 0.0;
		conf.coxaDamping = 0.01;
		conf.femurDamping = 0.01;
		conf.tibiaDamping = 0.01;
		conf.footDamping = 0.05;

		conf.backMaxVel = 1.7 * 1.961 * M_PI;
		conf.coxaMaxVel = 1.7 * 1.961 * M_PI;
		conf.femurMaxVel = 1.7 * 1.961 * M_PI;
		conf.tibiaMaxVel = 1.7 * 1.961 * M_PI;
		conf.footMaxVel = 1.7 * 1.961 * M_PI;

		conf.makeFoot = true;		// If true the foot is made

		return conf;
	}
}
