#ifndef DUNGBOTSENSORMOTORDEFINITION_H_
#define DUNGBOTSENSORMOTORDEFINITION_H_

namespace DungBotMotorSensor
{
	enum DungBotSensorNames
	{
		// Angle sensors (for actoric-sensor board (new board))
		TR0_as = 0, //Thoracic joint of right front leg
		TR1_as = 1, //Thoracic joint of right middle leg
		TR2_as = 2, //Thoracic joint of right hind leg

		TL0_as = 3, //Thoracic joint of left front leg
		TL1_as = 4, //Thoracic joint of left middle leg
		TL2_as = 5, //Thoracic joint of left hind leg

		CR0_as = 6, //Coxa joint of right front leg
		CR1_as = 7, //Coxa joint of right middle leg
		CR2_as = 8, //Coxa joint of right hind leg

		CL0_as = 9,  //Coxa joint of left front leg
		CL1_as = 10, //Coxa joint of left middle leg
		CL2_as = 11, //Coxa joint of left hind leg

		FR0_as = 12, //Fibula joint of right front leg
		FR1_as = 13, //Fibula joint of right middle leg
		FR2_as = 14, //Fibula joint of right hind leg

		FL0_as = 15, //Fibula joint of left front leg
		FL1_as = 16, //Fibula joint of left middle leg
		FL2_as = 17, //Fibula joint of left hind leg

		BJ_as = 18, //Backbone joint angle
		HJ_as = 19, //Head joint angle

		R0_s0 = 25,
		R0_s1 = 26,
		R0_s2 = 27,
		R0_s3 = 28,
		R0_s4 = 29,
		R0_s5 = 30,

		L0_s0 = 31,
		L0_s1 = 32,
		L0_s2 = 33,
		L0_s3 = 34,
		L0_s4 = 35,
		L0_s5 = 36,

		R1_s0 = 37,
		R1_s1 = 38,//middle right
		R1_s2 = 39,
		R1_s3 = 40,
		R1_s4 = 41,
		R1_s5 = 42,

		L1_s0 = 43,
		L1_s1 = 44,//middle left
		L1_s2 = 45,
		L1_s3 = 46,
		L1_s4 = 47,
		L1_s5 = 48,

		R2_s0 = 49,
		R2_s1 = 50,
		R2_s2 = 51,
		R2_s3 = 52,
		R2_s4 = 53,
		R2_s5 = 54,

		L2_s0 = 55,
		L2_s1 = 56,
		L2_s2 = 57,
		L2_s3 = 58,
		L2_s4 = 59,
		L2_s5 = 60,

		//	Contact sensors on the tarsus stump.







		DUNGBOT_SENSOR_MAX = 61,
	};

	enum DungBotMotorNames
	{
		TR0_m = 0,	// Upward (+), Downward (-)
		TR1_m = 1,
		TR2_m = 2,
		TL0_m = 3,
		TL1_m = 4,
		TL2_m = 5,

		CR0_m = 6,	// Upward (+), Downward (-)
		CR1_m = 7,
		CR2_m = 8,
		CL0_m = 9,
		CL1_m = 10,
		CL2_m = 11,

		FR0_m = 12,	// Upward (+), Downward (-)
		FR1_m = 13,
		FR2_m = 14,
		FL0_m = 15,
		FL1_m = 16,
		FL2_m = 17,

		BJ_m = 18,  // Upward (+), Downward (-)
		HJ_m = 19,
		//Changing according to the maximum motor number
		DUNGBOT_MOTOR_MAX = 19,
	};
}//	Ending namespace DungBot
#endif
