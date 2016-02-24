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

		CL0_as = 9,  //Coxa joint of left hind leg
		CL1_as = 10, //Coxa joint of left hind leg
		CL2_as = 11, //Coxa joint of left hind leg

		FR0_as = 12, //Fibula joint of right front leg
		FR1_as = 13, //Fibula joint of right middle leg
		FR2_as = 14, //Fibula joint of right hind leg

		FL0_as = 15, //Fibula joint of left front leg
		FL1_as = 16, //Fibula joint of left middle leg
		FL2_as = 17, //Fibula joint of left hind leg

		BJ_as = 18, //Backbone joint angle
		HJ_as = 19, //Head joint angle

		R0_s1 = 25,
		R0_s2 = 26,
		R0_s3 = 27,
		R0_s4 = 28,
		R0_s5 = 29,

		L0_s1 = 30,
		L0_s2 = 31,
		L0_s3 = 32,
		L0_s4 = 33,
		L0_s5 = 34,

		R1_s1 = 35,//middle right
		R1_s2 = 36,
		R1_s3 = 37,
		R1_s4 = 38,
		R1_s5 = 39,

		L1_s1 = 40,//middle left
		L1_s2 = 41,
		L1_s3 = 42,
		L1_s4 = 43,
		L1_s5 = 44,

		R2_s1 = 45,
		R2_s2 = 46,
		R2_s3 = 47,
		R2_s4 = 48,
		R2_s5 = 49,

		L2_s1 = 50,
		L2_s2 = 51,
		L2_s3 = 52,
		L2_s4 = 53,
		L2_s5 = 54,

		DUNGBOT_SENSOR_MAX = 55,
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
