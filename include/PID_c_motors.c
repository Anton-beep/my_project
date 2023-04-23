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
        // return (nMotorEncoder[motC] - startDegMotC) - (nMotorEncoder[motB] - startDegMotB);
        return 0;
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

short outPID;

task PIDEngineMot()
{
    while (true)
    {
        if (!(MOT_PID_SETTINGS.pauseAction))
        {
            MOT_PID_SETTINGS.errNow = getPIDErrMot(RATIO_MOT_B, RATIO_MOT_C, START_DEG_MOT_B, START_DEG_MOT_C);
            switch (ERR_MODE)
            {
            case 1:
                outPID = PIDFunction(&MOT_PID_SETTINGS);
                motor[motB] = POWER_MOT_B + outPID;
                motor[motC] = POWER_MOT_C + outPID;
                break;

            case 3:
                if (POWER_MOT_B != 0)
                {
                    motor[motB] = POWER_MOT_B;
                }
                if (POWER_MOT_C != 0)
                {
                    motor[motC] = POWER_MOT_C;
                }
                break;

            default:
                outPID = PIDFunction(&MOT_PID_SETTINGS);
                motor[motB] = POWER_MOT_B + outPID;
                motor[motC] = POWER_MOT_C - outPID;
                break;
            }
            sleep(MOT_PID_SETTINGS.dt * 1000);
        }
    }
}

void setStartDegMotBCAndReset(int startB, int startC)
{
    START_DEG_MOT_B = startB;
    START_DEG_MOT_C = startC;
    PIDReset(&MOT_PID_SETTINGS);
}

void setErrMode(float powerMotB, float powerMotC)
{
    if (powerMotB == 0 && powerMotC == 0)
    {
        ERR_MODE = 0;
    }
    else if (difSignsFloat(powerMotB, powerMotC))
    {
        // dif signs
        if (ERR_MODE != 1)
        {
            setStartDegMotBCAndReset(nMotorEncoder[motB], nMotorEncoder[motC]);
        }
        ERR_MODE = 1;
    }
    else if (!(difSignsFloat(powerMotB, powerMotC)))
    {
        // same signs
        if (ERR_MODE != 2)
        {
            setStartDegMotBCAndReset(nMotorEncoder[motB], nMotorEncoder[motC]);
        }
        ERR_MODE = 2;
    }
    else if (powerMotB == 0 || powerMotC == 0)
    {
        ERR_MODE = 3;
    }
}

void setNewMotBCPowersAndRatio(float powerMotB, float powerMotC)
{
    MOT_PID_SETTINGS.pauseAction = true;

    RATIO_MOT_B = powerMotB;
    RATIO_MOT_C = powerMotC;
    POWER_MOT_B = powerMotB;
    POWER_MOT_C = powerMotC;

    // PIDReset(&MOT_PID_SETTINGS);
    setErrMode(powerMotB, powerMotC);
    if (powerMotB != 0 || powerMotC != 0)
    {
        MOT_PID_SETTINGS.pauseAction = false;
    }
    else if (powerMotB == 0 && powerMotC != 0)
    {
        motor[motB] = 0;
    }
    else if (powerMotB != 0 && powerMotC == 0)
    {
        motor[motC] = 0;
    }
    else
    {
        motor[motB] = 0;
        motor[motC] = 0;
    }
}

void setNewMotBCOnlyPowers(short powerMotB, short powerMotC)
{
    MOT_PID_SETTINGS.pauseAction = true;
    POWER_MOT_B = powerMotB;
    POWER_MOT_C = powerMotC;

    setErrMode(powerMotB, powerMotC);
    if (powerMotB != 0 || powerMotC != 0)
    {
        MOT_PID_SETTINGS.pauseAction = false;
    }
    else if (powerMotB == 0 && powerMotC != 0)
    {
        motor[motB] = 0;
    }
    else if (powerMotB != 0 && powerMotC == 0)
    {
        motor[motC] = 0;
    }
    else
    {
        motor[motB] = 0;
        motor[motC] = 0;
    }
}

void setNewMotBCOnlyRatio(float powerMotB, float powerMotC)
{
    MOT_PID_SETTINGS.pauseAction = true;
    RATIO_MOT_B = powerMotB;
    RATIO_MOT_C = powerMotC;

    setErrMode(powerMotB, powerMotC);
    if (powerMotB != 0 || powerMotC != 0)
    {
        MOT_PID_SETTINGS.pauseAction = false;
    }
    else if (powerMotB == 0 && powerMotC != 0)
    {
        motor[motB] = 0;
    }
    else if (powerMotB != 0 && powerMotC == 0)
    {
        motor[motC] = 0;
    }
    else
    {
        motor[motB] = 0;
        motor[motC] = 0;
    }
}

void pausePIDMot()
{
    MOT_PID_SETTINGS.pauseAction = true;
}

void resumePIDMot()
{
    MOT_PID_SETTINGS.pauseAction = false;
}

bool KEEP_MOVING_B_WORKING = false;
bool KEEP_MOVING_C_WORKING = false;
bool PAUSE_KEEP_MOVING_B = false;
bool PAUSE_KEEP_MOVING_C = false;

task keepBMoving()
{
    float startTimeKeepMovingB = 0;
    while (true)
    {
        if (!(PAUSE_KEEP_MOVING_B))
        {
            if (getMotorRPM(motB) == 0 && POWER_MOT_B != 0)
            {
                if (!(KEEP_MOVING_B_WORKING))
                {
                    clearTimer(T1);
                    KEEP_MOVING_B_WORKING = true;
                }
                KEEP_MOVING_B_SETTINGS.errNow = time1[T1];
                if (POWER_MOT_B > 0)
                {
                    motor[motB] += PIDFunction(&KEEP_MOVING_B_SETTINGS);
                }
                else
                {
                    motor[motB] -= PIDFunction(&KEEP_MOVING_B_SETTINGS);
                }
                sleep(KEEP_MOVING_B_SETTINGS.dt * 1000);
            }
            else
            {
                KEEP_MOVING_B_WORKING = false;
            }
        }
    }
}

task keepCMoving()
{
    float startTimeKeepMovingC = 0;
    while (true)
    {
        if (!(PAUSE_KEEP_MOVING_C))
        {
            if (getMotorRPM(motC) == 0 && POWER_MOT_C != 0)
            {
                if (!(KEEP_MOVING_C_WORKING))
                {
                    clearTimer(T2);
                    KEEP_MOVING_C_WORKING = true;
                }
                KEEP_MOVING_C_SETTINGS.errNow = time1[T2];
                if (POWER_MOT_C > 0)
                {
                    motor[motC] += PIDFunction(&KEEP_MOVING_C_SETTINGS);
                }
                else
                {
                    motor[motC] -= PIDFunction(&KEEP_MOVING_C_SETTINGS);
                }
                sleep(KEEP_MOVING_C_SETTINGS.dt * 1000);
            }
            else
            {
                KEEP_MOVING_C_WORKING = false;
            }
        }
    }
}