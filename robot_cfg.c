#include "include/PID_c.c"

// ALL DISTANCES IN CM

float WHEEL_DIAMETER = 5.6;
float BASE_LEN = 0;

short MOTORS_MAX_POWER = 94;
short MOTORS_MIN_POWER = 10;

// MOTORS PID CONTROLLER

PIDSettings MOT_PID_SETTINGS;

void defStructures()
{
    MOT_PID_SETTINGS.Kp = 1;
    MOT_PID_SETTINGS.Ki = 0;
    MOT_PID_SETTINGS.Kd = 0;
    MOT_PID_SETTINGS.prevErr = 0;
    MOT_PID_SETTINGS.integral = 0;
    MOT_PID_SETTINGS.errNow = 0;
    MOT_PID_SETTINGS.dt = 0.01;
    MOT_PID_SETTINGS.pauseAction = false;
}