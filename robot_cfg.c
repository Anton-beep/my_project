#include "include/PID_c.c"
#include "structures.c"

// ALL DISTANCES IN CM

const float WHEEL_DIAMETER = 5.6;
const float BASE_LEN = 17.85;

const short MOTORS_MAX_POWER = 100;
const short MOTORS_MIN_POWER = 0;

const float KOEF_ENC_B = 1;
const float KOEF_ENC_C = 1;

const float PREDICT_KOEF = 0.2346;

// MOTORS PID CONTROLLER

PIDSettings MOT_PID_SETTINGS;

PIDSettings KEEP_MOVING_B_SETTINGS;
PIDSettings KEEP_MOVING_C_SETTINGS;

SenSettings SEN1_CALIBRATION;
SenSettings SEN2_CALIBRATION;
SenSettings SEN3_CALIBRATION;
SenSettings SEN3_CALIBRATION_CUBES;
SenSettings SEN3_CALIBRATION_1CUBE;

PIDSettings DEFAULT_LINE_PID_SUPRA;
PIDSettings DEFAULT_LINE_PID_FAST;
PIDSettings DEFAULT_LINE_PID_MEDIUM;
PIDSettings DEFAULT_LINE_PID_SLOW;

PIDSettings MANIP_A_PID_SETTINGS;
PIDSettings MANIP_D_PID_SETTINGS;
PIDSettings *MANIP_A_PID_PTR;
PIDSettings *MANIP_D_PID_PTR;

void defStructures()
{
    MOT_PID_SETTINGS.Kp = 4;
    MOT_PID_SETTINGS.Ki = 2;
    MOT_PID_SETTINGS.Kd = 0.002;
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

    SEN1_CALIBRATION.minR = 38;
    SEN1_CALIBRATION.minG = 47;
    SEN1_CALIBRATION.minB = 20;

    SEN1_CALIBRATION.maxR = 340;
    SEN1_CALIBRATION.maxG = 364;
    SEN1_CALIBRATION.maxB = 199;

    // ----------------------------------

    SEN2_CALIBRATION.minR = 40;
    SEN2_CALIBRATION.minG = 49;
    SEN2_CALIBRATION.minB = 33;

    SEN2_CALIBRATION.maxR = 318;
    SEN2_CALIBRATION.maxG = 410;
    SEN2_CALIBRATION.maxB = 313;

    // ----------------------------------

    SEN3_CALIBRATION.minR = 0;
    SEN3_CALIBRATION.minG = 0;
    SEN3_CALIBRATION.minB = 0;

    SEN3_CALIBRATION.maxR = 32;
    SEN3_CALIBRATION.maxG = 28;
    SEN3_CALIBRATION.maxB = 34;

    // ----------------------------------

    SEN3_CALIBRATION_1CUBE.minR = 0;
    SEN3_CALIBRATION_1CUBE.minG = 0;
    SEN3_CALIBRATION_1CUBE.minB = 0;

    SEN3_CALIBRATION_1CUBE.maxR = 62;
    SEN3_CALIBRATION_1CUBE.maxG = 49;
    SEN3_CALIBRATION_1CUBE.maxB = 68;

    // ----------------------------------

    SEN3_CALIBRATION_CUBES.minR = 0;
    SEN3_CALIBRATION_CUBES.minG = 0;
    SEN3_CALIBRATION_CUBES.minB = 0;

    SEN3_CALIBRATION_CUBES.maxR = 430;
    SEN3_CALIBRATION_CUBES.maxG = 350;
    SEN3_CALIBRATION_CUBES.maxB = 346;

    // ----------------------------------

    // power 85 - 100
    DEFAULT_LINE_PID_SUPRA.Kp = 0.29;
    DEFAULT_LINE_PID_SUPRA.Ki = 0.045;
    DEFAULT_LINE_PID_SUPRA.Kd = 0.00154;
    DEFAULT_LINE_PID_SUPRA.prevErr = 0;
    DEFAULT_LINE_PID_SUPRA.integral = 0;
    DEFAULT_LINE_PID_SUPRA.errNow = 0;
    DEFAULT_LINE_PID_SUPRA.dt = 0.001;
    DEFAULT_LINE_PID_SUPRA.pauseAction = false;

    // power 60 - 85
    DEFAULT_LINE_PID_FAST.Kp = 0.27;
    DEFAULT_LINE_PID_FAST.Ki = 0.042;
    DEFAULT_LINE_PID_FAST.Kd = 0.00151;
    DEFAULT_LINE_PID_FAST.prevErr = 0;
    DEFAULT_LINE_PID_FAST.integral = 0;
    DEFAULT_LINE_PID_FAST.errNow = 0;
    DEFAULT_LINE_PID_FAST.dt = 0.001;
    DEFAULT_LINE_PID_FAST.pauseAction = false;

    // ----------------------------------

    // power 30 - 60
    DEFAULT_LINE_PID_MEDIUM.Kp = 0.25;
    DEFAULT_LINE_PID_MEDIUM.Ki = 0.041;
    DEFAULT_LINE_PID_MEDIUM.Kd = 0.00149;
    DEFAULT_LINE_PID_MEDIUM.prevErr = 0;
    DEFAULT_LINE_PID_MEDIUM.integral = 0;
    DEFAULT_LINE_PID_MEDIUM.errNow = 0;
    DEFAULT_LINE_PID_MEDIUM.dt = 0.001;
    DEFAULT_LINE_PID_MEDIUM.pauseAction = false;

    // ----------------------------------

    // power 0 - 30
    DEFAULT_LINE_PID_SLOW.Kp = 0.24;
    DEFAULT_LINE_PID_SLOW.Ki = 0.04;
    DEFAULT_LINE_PID_SLOW.Kd = 0.00145;
    DEFAULT_LINE_PID_SLOW.prevErr = 0;
    DEFAULT_LINE_PID_SLOW.integral = 0;
    DEFAULT_LINE_PID_SLOW.errNow = 0;
    DEFAULT_LINE_PID_SLOW.dt = 0.001;
    DEFAULT_LINE_PID_SLOW.pauseAction = false;

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
    MANIP_D_PID_SETTINGS.Ki = 0.3;
    MANIP_D_PID_SETTINGS.Kd = 0;
    MANIP_D_PID_SETTINGS.prevErr = 0;
    MANIP_D_PID_SETTINGS.integral = 0;
    MANIP_D_PID_SETTINGS.errNow = 0;
    MANIP_D_PID_SETTINGS.dt = 0.001;
    MANIP_D_PID_SETTINGS.pauseAction = false;
    MANIP_D_PID_PTR = &MANIP_D_PID_SETTINGS;
}
