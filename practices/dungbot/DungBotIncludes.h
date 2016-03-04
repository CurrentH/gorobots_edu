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
		//	Used for tests
		bool testTarsusSensor;
		bool testNo;
		bool testHead;
		bool testBody;
		bool testCoxa;
		bool testFemur;
		bool testTibia;

		//	Body
		double massFront;
		double massRear;
		double massHead;
		double size;
		std::vector<double> headDimension;
		std::vector<double> frontDimension;
		std::vector<double> rearDimension;

		//	Legs
		std::vector<double> coxaLength;
		std::vector<double> coxaRadius;
		std::vector<double> coxaMass;
		std::vector<double> femurLength;
		std::vector<double> femurRadius;
		std::vector<double> femurMass;
		std::vector<double> tibiaLength;
		std::vector<double> tibiaRadius;
		std::vector<double> tibiaMass;
		double tarsusMass;
		std::vector<double> tarsusLength;
		std::vector<double> tarsusRadius;

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
