#include "PID_c.c"
#include "tools.c"
#include "../robot_cfg.c"

// calculation of an error
// it calculates using RATIO_MOT vars

short ERR_MODE;
/*
0 - powers are zeros
1 - different signs
2 - same signs
3 - one is zero
*/

float getPIDErrMot(float ratioMotB, float ratioMotC, float startDegMotB, float startDegMotC)
{
    switch (ERR_MODE)
    {
    case 0:
        return (nMotorEncoder[motC] - startDegMotC) - (nMotorEncoder[motB] - startDegMotB);
        break;

    case 1:
        return (ratioMotB * (nMotorEncoder[motC] - startDegMotC) - (nMotorEncoder[motB] - startDegMotB) * ratioMotC) / (ratioMotC - ratioMotB);
        break;

    case 2:
        return (ratioMotB * (nMotorEncoder[motC] - startDegMotC) - (nMotorEncoder[motB] - startDegMotB) * ratioMotC) / (ratioMotC + ratioMotB);
        break;

    default:
        return 0;
        break;
    }
}

// DO NOT CHANGE VALUES, ONLY THROUGH FUNCTION
short POWER_MOT_B = 0;
short POWER_MOT_C = 0;
float RATIO_MOT_B = 0;
float RATIO_MOT_C = 0;
int START_DEG_MOT_B = 0;
int START_DEG_MOT_C = 0;

short out_PID;

task PIDEngineMot()
{
    while (true)
    {
        MOT_PID_SETTINGS.errNow = getPIDErrMot(RATIO_MOT_B, RATIO_MOT_C, START_DEG_MOT_B, START_DEG_MOT_C);
        out_PID = PIDFunction(&MOT_PID_SETTINGS);
        switch (ERR_MODE)
        {
        case 1:
            motor[motB] = POWER_MOT_B + out_PID;
            motor[motC] = POWER_MOT_C + out_PID;
            break;

        default:
            motor[motB] = POWER_MOT_B + out_PID;
            motor[motC] = POWER_MOT_C - out_PID;
            break;
        }
        sleep(MOT_PID_SETTINGS.dt * 1000);
    }
}

void setErrModeAndStartDeg(float powerMotB, float powerMotC)
{
    if (powerMotB == 0 && powerMotC == 0)
    {
        ERR_MODE = 0;
    }
    else if (difSignsFloat(powerMotB, powerMotC))
    {
        // dif signs
        ERR_MODE = 1;
    }
    else if (!(difSignsFloat(powerMotB, powerMotC)))
    {
        // same signs
        ERR_MODE = 2;
    }
    else if (powerMotB == 0 || powerMotC == 0)
    {
        ERR_MODE = 3;
    }

    START_DEG_MOT_B = nMotorEncoder[motB];
    START_DEG_MOT_C = nMotorEncoder[motC];
}

void setNewMotBCPowersAndRatio(float powerMotB, float powerMotC)
{
    RATIO_MOT_B = powerMotB;
    RATIO_MOT_C = powerMotC;
    POWER_MOT_B = powerMotB;
    POWER_MOT_C = powerMotC;

    // PIDReset(&)
    setErrModeAndStartDeg(powerMotB, powerMotC);
}

void setNewMotBCOnlyPowers(short powerMotB, short powerMotC)
{
    POWER_MOT_B = powerMotB;
    POWER_MOT_C = powerMotC;

    setErrModeAndStartDeg(powerMotB, powerMotC);
}

void setNewMotBCOnlyRatio(float powerMotB, float powerMotC)
{
    RATIO_MOT_B = powerMotB;
    RATIO_MOT_C = powerMotC;

    setErrModeAndStartDeg(powerMotB, powerMotC);
}

void pausePIDMot()
{
    MOT_PID_SETTINGS.pauseAction = true;
}

void resumePIDMot()
{
    MOT_PID_SETTINGS.pauseAction = false;
}
