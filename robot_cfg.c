#include "include/PID_c.c"
#include "structures.c"

// ALL DISTANCES IN CM

float WHEEL_DIAMETER = 5.6;
float BASE_LEN = 18.5;

short MOTORS_MAX_POWER = 100;
short MOTORS_MIN_POWER = 0;

// MOTORS PID CONTROLLER

PIDSettings MOT_PID_SETTINGS;

PIDSettings KEEP_MOVING_B_SETTINGS;
PIDSettings KEEP_MOVING_C_SETTINGS;

SenSettings SEN1_CALIBRATION;
SenSettings SEN2_CALIBRATION;
SenSettings SEN3_CALIBRATION;

PIDSettings DEFAULT_LINE_PID;

PIDSettings MANIP_A_PID_SETTINGS;
PIDSettings MANIP_D_PID_SETTINGS;
PIDSettings *MANIP_A_PID_PTR;
PIDSettings *MANIP_D_PID_PTR;

void defStructures()
{
    MOT_PID_SETTINGS.Kp = 10;
    MOT_PID_SETTINGS.Ki = 3;
    MOT_PID_SETTINGS.Kd = 0;
    MOT_PID_SETTINGS.prevErr = 0;
    MOT_PID_SETTINGS.integral = 0;
    MOT_PID_SETTINGS.errNow = 0;
    MOT_PID_SETTINGS.dt = 0.001;
    MOT_PID_SETTINGS.pauseAction = false;

    // ----------------------------------

    KEEP_MOVING_B_SETTINGS.Kp = 0.1;
    KEEP_MOVING_B_SETTINGS.Ki = 0;
    KEEP_MOVING_B_SETTINGS.Kd = 0;
    KEEP_MOVING_B_SETTINGS.prevErr = 0;
    KEEP_MOVING_B_SETTINGS.integral = 0;
    KEEP_MOVING_B_SETTINGS.errNow = 0;
    KEEP_MOVING_B_SETTINGS.dt = 0.01;
    KEEP_MOVING_B_SETTINGS.pauseAction = false;

    // ----------------------------------

    KEEP_MOVING_C_SETTINGS.Kp = 0.1;
    KEEP_MOVING_C_SETTINGS.Ki = 0;
    KEEP_MOVING_C_SETTINGS.Kd = 0;
    KEEP_MOVING_C_SETTINGS.prevErr = 0;
    KEEP_MOVING_C_SETTINGS.integral = 0;
    KEEP_MOVING_C_SETTINGS.errNow = 0;
    KEEP_MOVING_C_SETTINGS.dt = 0.01;
    KEEP_MOVING_C_SETTINGS.pauseAction = false;

    // ----------------------------------

    SEN1_CALIBRATION.minR = 0;
    SEN1_CALIBRATION.minG = 0;
    SEN1_CALIBRATION.minB = 0;

    SEN1_CALIBRATION.maxR = 400;
    SEN1_CALIBRATION.maxG = 400;
    SEN1_CALIBRATION.maxB = 400;

    // ----------------------------------

    SEN2_CALIBRATION.minR = 0;
    SEN2_CALIBRATION.minG = 0;
    SEN2_CALIBRATION.minB = 0;

    SEN2_CALIBRATION.maxR = 400;
    SEN2_CALIBRATION.maxG = 400;
    SEN2_CALIBRATION.maxB = 400;

    // ----------------------------------

    SEN3_CALIBRATION.minR = 0;
    SEN3_CALIBRATION.minG = 0;
    SEN3_CALIBRATION.minB = 0;

    SEN3_CALIBRATION.maxR = 400;
    SEN3_CALIBRATION.maxG = 400;
    SEN3_CALIBRATION.maxB = 400;

    // ----------------------------------

    DEFAULT_LINE_PID.Kp = 0.01;
    DEFAULT_LINE_PID.Ki = 0.01;
    DEFAULT_LINE_PID.Kd = 0;
    DEFAULT_LINE_PID.prevErr = 0;
    DEFAULT_LINE_PID.integral = 0;
    DEFAULT_LINE_PID.errNow = 0;
    DEFAULT_LINE_PID.dt = 0.001;
    DEFAULT_LINE_PID.pauseAction = false;

    // ----------------------------------

    MANIP_A_PID_SETTINGS.Kp = 1;
    MANIP_A_PID_SETTINGS.Ki = 0;
    MANIP_A_PID_SETTINGS.Kd = 0;
    MANIP_A_PID_SETTINGS.prevErr = 0;
    MANIP_A_PID_SETTINGS.integral = 0;
    MANIP_A_PID_SETTINGS.errNow = 0;
    MANIP_A_PID_SETTINGS.dt = 0.1;
    MANIP_A_PID_SETTINGS.pauseAction = false;
    MANIP_A_PID_PTR = &MANIP_A_PID_SETTINGS;

    // ----------------------------------

    MANIP_D_PID_SETTINGS.Kp = 0.3;
    MANIP_D_PID_SETTINGS.Ki = 0.1;
    MANIP_D_PID_SETTINGS.Kd = 0.01;
    MANIP_D_PID_SETTINGS.prevErr = 0;
    MANIP_D_PID_SETTINGS.integral = 0;
    MANIP_D_PID_SETTINGS.errNow = 0;
    MANIP_D_PID_SETTINGS.dt = 0.1;
    MANIP_D_PID_SETTINGS.pauseAction = false;
    MANIP_D_PID_PTR = &MANIP_D_PID_SETTINGS;
}
