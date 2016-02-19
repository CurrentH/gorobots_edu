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

        //for (ServoList::iterator it = passiveServos.begin(); it != passiveServos.end(); it++) {
        //  (*it)->set(0.0);
        //}
    }

    void DungBot::update( void )
    {
    	OdeRobot::update();
    }

    void DungBot::sense( GlobalData& globalData )
    {
    }

    void DungBot::create( const Matrix& pose )
    {
    	odeHandle.createNewSimpleSpace(parentspace, false);

    	std::cout << "FRONT MASS:\t" << conf.massFront << std::endl;
    	std::cout << "REAR MASS:\t" << conf.massRear << std::endl;
    	std::cout << "COXA MASS:\t" << conf.coxaMass << std::endl;
    	std::cout << "FEMUR MASS:\t" << conf.coxaMass << std::endl;
    	std::cout << "TIBIA MASS:\t" << conf.coxaMass << std::endl;
    	std::cout << "FOOT MASS:\t" << conf.footMass << std::endl;
    	std::cout << "TARUS MASS:\t" << conf.tarusMass << std::endl;

    	/************************************
         * BODY PARTS
         ***********************************/

		//First we find the pose for the center of the body-part to be made, then we run the functions that creates it.
    	osg::Matrix rearPos = osg::Matrix::translate( ( -conf.rearDimension[0] / 2 ), 0, 0) * pose;
    	auto rear = makeBody( rearPos, conf.massRear, conf.rearDimension );

    	osg::Matrix frontPos = osg::Matrix::translate( ( conf.frontDimension[0] / 2 ), 0, 0) * pose;
		auto front = makeBody( frontPos, conf.massFront, conf.frontDimension );


		// representation of the origin
		const Pos nullpos(0,0,0);

		//Place the joint between the two body-parts
		makeBodyHingeJoint( front, rear, nullpos*osg::Matrix::translate( -conf.frontDimension[0] / 2, 0, 0 ) * frontPos, Axis( 0, 1, 0 ) * frontPos, conf.rearDimension[1] );

	    /************************************
	     * LEGS
	     ***********************************/
		/* SE --> http://www.manoonpong.com/paper/2015/SWARM_2015_DungBeetleRobot.pdf */
		makeAllLegs( pose , rear, front );

    }

    void DungBot::makeAllLegs( const Matrix& pose, Primitive* front, Primitive* rear)
    {
    	// representation of the origin
    	const Pos nullpos(0,0,0);
		double xPosition=0, yPosition=0, zPosition=0;
		std::map<LegPos, osg::Matrix> legtrunkconnections;


		// The purpose of this for-loop is to get all the leg-trunk-connections
		for (int i = 0; i < LEG_POS_MAX; i++) // Run through all of the legs
		{
			LegPos leg = LegPos(i);	// Give a value to leg (0-6), then if (leg == 'something') is true = 1

			// Make the right legs have a negative sign
			const double lr = (leg == L0 || leg == L1 || leg == L2) - (leg == R0 || leg == R1 || leg == R2);
			// Hind legs
			const double lr2= leg==L1 || leg==R1 || leg==L2 || leg==R2;

			// create 3d-coordinates for the leg-trunk connection:
			switch (i) {
			  case L0:
			  case R0:
				  xPosition = conf.frontDimension[0]/2;
				  yPosition = lr * conf.frontDimension[1]/3;
				  zPosition = -(conf.frontDimension[2]/2+0.8*conf.coxaRadius);
				  break;
			  case L1:
			  case R1:
				  xPosition = -conf.rearDimension[0]/3;
				  yPosition = lr * conf.rearDimension[1]/3;
				  zPosition = -(conf.rearDimension[2]/2+0.8*conf.coxaRadius);
				  break;
			  case L2:
			  case R2:
				  xPosition = (-2*conf.rearDimension[0])/3;
				  yPosition = lr * conf.rearDimension[1]/3;
				  zPosition = -(conf.rearDimension[2]/2+0.8*conf.coxaRadius);
				  break;
			  default:
				  xPosition = 0;
			  }
			Pos pos = Pos(xPosition,yPosition,zPosition);

		    // Now the rotation. lr2 rotates the four hind legs (PI/2)/2 in Y...
			legtrunkconnections[leg] = osg::Matrix::rotate(M_PI/2 , lr, lr2/2, 0) * osg::Matrix::translate(pos) * pose;
		}


		std::vector<Primitive*> tarsusParts;

	    // create the legs
	    for (int i = 0; i < LEG_POS_MAX; i++)
	    {
			LegPos leg = LegPos(i);

			// +1 for R1,R2,R3, -1 for L1,L2,conf.tebiaRadius
			//const double pmrl = (leg == R0 || leg == R1 || leg == R2) - (leg == L0 || leg == L1 || leg == L2);

	        const double backLeg = (leg == R1 || leg == R2) - (leg == L1 || leg == L2);
	        const double backLegInverse = (leg == R1 || leg == R2) + (leg == L1 || leg == L2);

	        double frontL = (leg == R0) - (leg == L0);
	        double frontInverse = (leg == R0) + (leg == L0);

			//first Coxa position
			osg::Matrix c1 = legtrunkconnections[leg];

			// Coxa placement
			osg::Matrix coaxCenter = osg::Matrix::translate(0, 0, -conf.coxaLength / 2) * c1; //Position of Center of Mass
			Primitive* coxaThorax = new Capsule(conf.coxaRadius, conf.coxaLength);
			coxaThorax->setTexture("coxa.jpg");
			coxaThorax->init(odeHandle, conf.coxaMass, osgHandle);
			coxaThorax->setPose(coaxCenter);
			legs[leg].coxa = coxaThorax;
			objects.push_back(coxaThorax);

			// Femur placement
			osg::Matrix c2 = osg::Matrix::translate(0, 0, -conf.coxaLength/2 ) * coaxCenter;
			osg::Matrix femurcenter = osg::Matrix::translate(0, 0, -conf.femurLength / 2) * c2;
			Primitive* femurThorax = new Capsule(conf.femurRadius, conf.femurLength);
			femurThorax->setTexture("femur.jpg");
			femurThorax->init(odeHandle, conf.femurMass, osgHandle);
			femurThorax->setPose(femurcenter);
			legs[leg].femur = femurThorax;
			objects.push_back(femurThorax);

			// Tibia placement
			osg::Matrix c3 = osg::Matrix::translate(0, 0, -conf.femurLength / 2) * femurcenter;
			osg::Matrix tibiaCenter = osg::Matrix::translate(0, 0, -conf.tibiaLength / 2) * c3;
			Primitive* tibia = new Capsule(conf.tibiaRadius, conf.tibiaLength);
			tibia->setTexture("tebia.jpg");
			tibia->init(odeHandle, conf.tibiaMass, osgHandle);
			tibia->setPose(tibiaCenter);
			legs[leg].tibia = tibia;
			objects.push_back(tibia);

			// calculate anchor and axis of the first joint
			const osg::Vec3 anchor1 = nullpos * c1;
	        Axis axis1 = Axis(0,0,backLeg+frontL) * c1;
	        switch (i)
	        {
	        case 0: axis1=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis1;//front left
	        break;
	        case 1: axis1=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis1;//middle left
	        break;
	        case 2: axis1=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis1; //rear left ok
	        break;
	        case 3: axis1=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis1; //front right
	        break;
	        case 4: axis1=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis1;  // middle right
	        break;
	        case 5: axis1=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis1;  // rear right ok
	        break;
	        default: axis1=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis1;
	        break;
	        }

			// Proceed along the leg (and the respective z-axis) for second limb
			const osg::Vec3 anchor2 = nullpos * c2;
			Axis axis2 = Axis(backLeg+frontL,backLegInverse,backLeg) * c2;
			switch (i)
			{
			case 0: axis2=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis2;//front left
			break;
			case 1:axis2=osg::Matrix::rotate(M_PI/180,1,0,0)*osg::Matrix::rotate(M_PI/180*-100,0,1,0)*osg::Matrix::rotate(M_PI/180*30+M_PI/2,0,0,1)*axis2; //midle left ok
			break;
			case 2: axis2=osg::Matrix::rotate(M_PI/180,1,0,0)*osg::Matrix::rotate(M_PI/180*-80,0,1,0)*osg::Matrix::rotate(M_PI/180*30+M_PI/2,0,0,1)*axis2; //rear left ok
			break;
			case 3: axis2=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis2; //front right
			break;
			case 4: axis2=osg::Matrix::rotate(M_PI/180*180,1,0,0)*osg::Matrix::rotate(-95*(M_PI/180-100)+M_PI,0,1,0)*osg::Matrix::rotate(-(M_PI/180*30+M_PI/2),0,0,1)*axis2;  // middle right
			break;
			case 5: axis2=osg::Matrix::rotate(M_PI/180,1,0,0)*osg::Matrix::rotate((M_PI/180*-80),0,1,0)*osg::Matrix::rotate(-(M_PI/180*30+M_PI/2),0,0,1)*axis2;  // rear right ok
			break;
			default: axis2=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis2;
			break;
			}

			//and third
			const osg::Vec3 anchor3 = nullpos * c3;
	        Axis axis3 = Axis(backLeg+frontL,backLegInverse-frontInverse,-frontL) * c3;
	        switch (i)
	        {
	        case 0: axis3=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis3;//front left
	        break;
	        case 1: axis3=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(M_PI/180*-50,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis3;//middle left
	        break;
	        case 2: axis3=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(M_PI/180*-30,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis3; //rear left ok
	        break;
	        case 3: axis3=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis3; //front right
	        break;
	        case 4: axis3=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(M_PI/180*-50,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis3;  // middle right
	        break;
	        case 5: axis3=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(M_PI/180*-30,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis3; // rear right ok
	        break;
	        default:axis3=osg::Matrix::rotate(0,1,0,0)*osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(0,0,0,1)*axis3;
	        break;
	        }

			// hingeJoint to first limb
			HingeJoint* j = new HingeJoint((leg == L0 || leg == R0) ? rear : front, coxaThorax, anchor1, -axis1); // Only L0 and R0 should be attached to front
			j->init(odeHandle, osgHandle.changeColor("joint"), true, conf.coxaRadius * 2.1);
			joints.push_back(j);
	        // create motor, overwrite the jointLimit argument with 1.0
	        // because it is less obscure and setMinMax makes mistakes
	        // otherwise. Parameters are set later
			OneAxisServo * servo1 = new OneAxisServoVel(odeHandle, j, -1, 1, 1, 0.01, 0, 1.0); //TODO VIGTIGT SKAL VIRKE ASAP Noget med noget moter funk der skal med
			legs[leg].tcServo = servo1;

			// create the joint from first to second limb (coxa to femur)
			HingeJoint* k = new HingeJoint(coxaThorax, femurThorax, anchor2, -axis2);
			k->init(odeHandle, osgHandle.changeColor("joint"), true, conf.coxaRadius * 2.1);
			legs[leg].ctJoint = k;
			joints.push_back(k);
	        // create motor, overwrite the jointLimit argument with 1.0
	        // because it is less obscure and setMinMax makes mistakes
	        // otherwise. Parameters are set later
			OneAxisServo * servo2 = new OneAxisServoVel(odeHandle, k, -1, 1, 1, 0.01, 0, 1.0); //TODO VIGTIGT SKAL VIRKE ASAP Noget med noget moter funk der skal med
			legs[leg].ctrServo = servo2;

			// springy knee joint
			HingeJoint* l = new HingeJoint(femurThorax, tibia, anchor3, -axis3);
			l->init(odeHandle, osgHandle.changeColor("joint"), true, conf.tibiaRadius * 2.1);
			legs[leg].ftJoint = l;
			joints.push_back(l);
	        // create motor, overwrite the jointLimit argument with 1.0
	        // because it is less obscure and setMinMax makes mistakes
	        // otherwise. Parameters are set later
			OneAxisServo * servo3 = new OneAxisServoVel(odeHandle, l, -1, 1, 1, 0.01, 0, 1.0); //TODO VIGTIGT SKAL VIRKE ASAP Noget med noget moter funk der skal med
			legs[leg].ftiServo = servo3;

			// Foot
			if(conf.makeFoot) // toggle foot
			{
				osg::Matrix c4 = osg::Matrix::translate(0, 0, -conf.tibiaLength / 2 - conf.footRange) * tibiaCenter;
				osg::Matrix footCenter = osg::Matrix::translate(0, 0, -conf.footSpringPreload) * c4;

				const osg::Vec3 anchor4 = nullpos * footCenter;
				const Axis axis4 = Axis(0, 0, -1) * c4;

				OdeHandle my_odeHandle = odeHandle;

				// set the foot to use rubberFeet
				const Substance FootSubstance(3.0, 0.0, 500.0, 0.1);
				my_odeHandle.substance = FootSubstance;


				Primitive* foot;
				foot = new Capsule(conf.footRadius, conf.footRange);
				foot->setTexture("foot.jpg");
				foot->init(my_odeHandle, conf.footMass, osgHandle);
				foot->setPose(footCenter);
				legs[leg].foot = foot;
				objects.push_back(foot);

				HingeJoint* m = new HingeJoint(tibia, foot, anchor4, axis4);						//TODO SHOULD BE A SliderJoint and not HingeJoint
				m->init(odeHandle, osgHandle.changeColor("joint"), true, conf.tibiaRadius, true);
				legs[leg].footJoint = m;
				joints.push_back(m);

				/** parameters are set later */
				//Spring* spring = new Spring(m, -1, 1, 1);
				//legs[leg].footSpring = spring;
				//passiveServos.push_back(spring);
				odeHandle.addIgnoredPair(femurThorax, foot);

				// New: tarsus
				Primitive *tarsus;
				double angle = M_PI/12;

				double radius = conf.footRadius/3;
				double length = conf.footRange;
				double mass = conf.tarusMass/10;
				tarsus = new Capsule(radius,length);
				tarsus->setTexture("tarsus.jpg");
				tarsus->init(odeHandle, mass, osgHandle);

				osg::Matrix m6;
				osg::Matrix m5 = 	osg::Matrix::rotate(-angle,i%2==0 ? -1 : 1,0,0) *
									osg::Matrix::translate(0,0,-length/2) *
									footCenter;

				double angleTarsus=0;

				//rotate manually tarsus here
				switch (i)
				{
				case 0: angleTarsus=0;//front left
				break;
				case 1: angleTarsus=(M_PI/180)*300+M_PI+(M_PI/180)*20; //middle left
				break;
				case 2: angleTarsus=0; //rear left ok
				break;
				case 3: angleTarsus=(M_PI/180)*20; //front right
				break;
				case 4: angleTarsus=(M_PI/180)*140;  // middle right
				break;
				case 5: angleTarsus=-(M_PI/180)*290;  // rear right ok
				break;
				default: angleTarsus=0;
				break;
				}

				if(i < 2){
				 m6 = osg::Matrix::rotate(i%2==0 ? angle : -angle,0,i%2==0 ? -1 : 1,0) * m5;
				}else if( i > 3){
				 m6 = osg::Matrix::rotate(i%2==0 ? -angle : angle,0,i%2==0 ? -1 : 1,0) * m5;
				}else{
				 m6 = m5;
				}
				m6 = osg::Matrix::rotate(0,1,0,0) *osg::Matrix::rotate(0,0,1,0)*osg::Matrix::rotate(angleTarsus,0,0,1)* osg::Matrix::translate(0,0,-length/2) * m6 ;

				std::cout << "leg number     " << i << std::endl;

				tarsus->setPose(m6);
				tarsusParts.push_back(tarsus);
				objects.push_back(tarsus);

				FixedJoint* q = new FixedJoint(foot, tarsus);
				q->init(odeHandle, osgHandle, false);
				joints.push_back(k);



				Primitive *section = tarsus;

				for(int j = 1; j < 6; j++){

				 double lengthS = length/1.9;
				 double radiusS = radius/1.5;
				 section = new Capsule(radiusS,lengthS);
				 section->setTexture("tarsus.jpg");
				 section->init(odeHandle, mass, osgHandle);

				 osg::Matrix m7;

				 if(i < 2){
					 m7 =                  osg::Matrix::translate(0,0,-lengthS/2) *
							 osg::Matrix::rotate(i%2==0 ? angle : -angle,0,i%2==0 ? -1 : 1,0) *
							 osg::Matrix::rotate(i%2==0 ? angle : -angle,1,0,0) *
							 osg::Matrix::translate(0,0,-length/2) *
							 m6;
				 }else if(i > 3){
					 m7 =                       osg::Matrix::translate(0,0,-lengthS/2) *
							 osg::Matrix::rotate(i%2==0 ? -angle : angle,0,i%2==0 ? -1 : 1,0) *
							 osg::Matrix::rotate(i%2==0 ? angle : -angle,1,0,0) *
							 osg::Matrix::translate(0,0,-length/2) *
							 m6;

				 }else{
					 m7 =                             osg::Matrix::translate(0,0,-lengthS/2) *
							 osg::Matrix::rotate(i%2==0 ? angle : -angle,1,0,0) *
							 osg::Matrix::translate(0,0,-length/2) *
							 m6;
				 }

				 section->setPose(m7);
				 objects.push_back(section);
				 tarsusParts.push_back(section);


				 if(j==1)
				 {
					 HingeJoint* k = new HingeJoint(tarsusParts[j-1], tarsusParts[j], Pos(0,0,length/3) * m7, Axis(i%2==0 ? -1 : 1,0,0) * m7);
					 k->init(odeHandle, osgHandle, true, lengthS/16 * 2.1);
					 // servo used as a spring
					 //auto servo = std::make_shared<OneAxisServoVel>(odeHandle,k, -1, 1, 1, 0.05); // parameters are set later
					 joints.push_back(k);
					 //auto spring = std::make_shared<ConstantMotor>(servo, 0.0);
					 //tarsussprings.push_back(servo);
					 //addMotor(spring);

				 }

				 if(j==2)
				 {
					 HingeJoint* k = new HingeJoint(tarsusParts[j-1], tarsusParts[j], Pos(0,0,length/3) * m7, Axis(i%2==0 ? -1 : 1,0,0) * m7);
					 k->init(odeHandle, osgHandle, true, lengthS/16 * 2.1);
					 // servo used as a spring
					 //auto servo = std::make_shared<OneAxisServoVel>(odeHandle,k, -1, 1, 1, 0.05); // parameters are set later
					 joints.push_back(k);
					 //auto spring = std::make_shared<ConstantMotor>(servo, 0.0);
					 //tarsussprings.push_back(servo);
					 //addMotor(spring);

				 }



				 else{
					 HingeJoint* k = new HingeJoint(tarsusParts[j-1], tarsusParts[j], Pos(0,0,length/3) * m7, Axis(i%2==0 ? -1 : 1,0,0) * m7);
					 k->init(odeHandle, osgHandle, true, lengthS/16 * 2.1);
					 // servo used as a spring
					 //auto servo = std::make_shared<OneAxisServoVel>(odeHandle,k, -1, 1, 1, 0.01); // parameters are set later
					 joints.push_back(k);
					 //auto spring = std::make_shared<ConstantMotor>(servo, 0.0);
					 //tarsussprings.push_back(servo);
					 //addMotor(spring);
				 }

				 m6 = m7;

				 //tarsusContactSensors[std::make_pair(LegPos(i),j)] = new ContactSensor(true, 65/*koh changed 100*/, 1.5 * radiusS, false, true, Color(1,9,3));
				 //tarsusContactSensors[std::make_pair(LegPos(i),j)]->setInitData(odeHandle, osgHandle, osg::Matrix::translate(0, 0, -(0.5) * lengthS));
				 //tarsusContactSensors[std::make_pair(LegPos(i),j)]->init(tarsusParts.at(j));
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
