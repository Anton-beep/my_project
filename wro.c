#pragma config(Sensor, S1,     sen1,           sensorEV3_Color, modeEV3Color_RGB_Raw)
#pragma config(Sensor, S2,     sen2,           sensorEV3_Color, modeEV3Color_RGB_Raw)
#pragma config(Sensor, S3,     sen3,           sensorEV3_Color, modeEV3Color_RGB_Raw)
#pragma config(Motor,  motorA,          motA,          tmotorEV3_Large, openLoop)
#pragma config(Motor,  motorB,          motB,          tmotorEV3_Large, openLoop, encoder)
#pragma config(Motor,  motorC,          motC,          tmotorEV3_Large, openLoop, encoder)
#pragma config(Motor,  motorD,          motD,          tmotorEV3_Large, openLoop, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "include/PID_c_motors.c"
#include "include/motion.c"
#include "include/sen_for_colors.c"
#include "include/motion_with_sensors.c"
#include "include/PID_c_manip.c"
#include "logic/main_logic.c"
#include "logic/define_func.c"

task main (){
	// define everything
	motor[motB] = 0;
	motor[motC] = 0;
	motor[motD] = 0;
	//int r, g, b;
	//getColorRawRGB(sen1, r, g, b);
	//getColorRawRGB(sen2, r, g, b);
	//getColorRawRGB(sen3, r, g, b);
	//sleep(200);

	setSoundVolume(5);//	RETURN VOLUME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	startTask(checkBat);
	clearTimer(T3);
	defStructures();
	setMotorBrakeMode(motB, motorCoast);
	setMotorBrakeMode(motC, motorCoast);
	//setMotorBrakeMode(motD, motorCoast);
	startTask(PIDEngineMot, 7);
	//startTask(checkMotorProblems, 7);
	//startTask(keepBMoving);
	//startTask(keepCMoving);

	defineForLogic();

	define_main();

#if RUN_ONLY_TEST_FUNCTION == 1
	testFunc();
#else
	mainLogic();
#endif

	stopBC();
	displayExecTime();
	stopAllTasks();
}
