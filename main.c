#pragma config(Sensor, S1,     sen1,           sensorEV3_Color, modeEV3Color_RGB_Raw)
#pragma config(Sensor, S2,     sen2,           sensorEV3_Color, modeEV3Color_RGB_Raw)
#pragma config(Sensor, S3,     sen3,           sensorEV3_Color, modeEV3Color_RGB_Raw)
#pragma config(Motor,  motorA,          motA,          tmotorEV3_Large, openLoop, encoder)
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

task main (){
	// define everything
	setSoundVolume(0);
	checkBat();
	clearTimer(T3);
	defStructures();
	setMotorBrakeMode(motB, motorCoast);
	setMotorBrakeMode(motC, motorCoast);
	setMotorBrakeMode(motD, motorCoast);
	startTask(PIDEngineMot);
	//startTask(keepBMoving);
	//startTask(keepCMoving);

	defineForLogic();
	mainLogic();
	//testFunc();

	stopBC();
	displayExecTime();
	stopAllTasks();
}
