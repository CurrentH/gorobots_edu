namespace lpzrobots
{
	/**
	 * forward declarations
	 */
	class HingeJoint;
	class IRSensor;
	class Joint;
	class OneAxisServo;
	class Primitive;
	class RaySensorBank;
	class SliderJoint;
	class Spring;
	class TwoAxisServo;

	typedef struct
	{
		//	Body
		double massFront;
		double massRear;
		double massHead;
		double size;
		std::vector<double> headDimension;
		std::vector<double> frontDimension;
		std::vector<double> rearDimension;

		//	Legs
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
		double footSpringLimitD;
		double footSpringLimitU;
		double tarusMass;
		bool   makeFoot;

		//	Motor settings
		double backPower;
		double coxaPower;
		double femurPower;
		double tibiaPower;
		double footPower;
		double backDamping;
		double coxaDamping;
		double femurDamping;
		double tibiaDamping;
		double footDamping;
		double backMaxVel;
		double coxaMaxVel;
		double femurMaxVel;
		double tibiaMaxVel;
		double footMaxVel;

		//	Joint limits
		double backJointLimitD;
		double backJointLimitU;
		double fCoxaJointLimitF;
		double fCoxaJointLimitB;
		double mCoxaJointLimitF;
		double mCoxaJointLimitB;
		double rCoxaJointLimitF;
		double rCoxaJointLimitB;
		double fFemurJointLimitD;
		double fFemurJointLimitU;
		double mFemurJointLimitD;
		double mFemurJointLimitU;
		double rFemurJointLimitD;
		double rFemurJointLimitU;
		double fTibiaJointLimitD;
		double fTibiaJointLimitU;
		double mTibiaJointLimitD;
		double mTibiaJointLimitU;
		double rTibiaJointLimitD;
		double rTibiaJointLimitU;
	}DungBotConf;

}
