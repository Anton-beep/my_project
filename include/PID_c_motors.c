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

/*
Процесс калибровки:
Взять мотор, прокрутить (НА НИЗКОЙ СКОРОСТИ, ЧТОБЫ ИЗБЕЖАТЬ ТРОТЛИНГА) на много градусов (лучше кртаных 360, так нагляднее)
Менять коэфициент до тех пор, пока отклонение будет не заметно

По хорошему нужно придумать механическое устройство
*/

float getEncoderB()
{
    return nMotorEncoder[motB] * KOEF_ENC_B;
}

float getEncoderC()
{
    return nMotorEncoder[motC] * KOEF_ENC_C;
}

float getPIDErrMot(float ratioMotB, float ratioMotC, float startDegMotB, float startDegMotC)
{
    switch (ERR_MODE)
    {
    case 0:
        // return (nMotorEncoder[motC] - startDegMotC) - (nMotorEncoder[motB] - startDegMotB);
        return 0;
        break;

    case 1:
        return (ratioMotB * (getEncoderC() - startDegMotC) - (getEncoderB() - startDegMotB) * ratioMotC) / (ratioMotC - ratioMotB);
        break;

    case 2:
        return (ratioMotB * (getEncoderC() - startDegMotC) - (getEncoderB() - startDegMotB) * ratioMotC) / (ratioMotC + ratioMotB);
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
            setStartDegMotBCAndReset(getEncoderB(), getEncoderC());
        }
        ERR_MODE = 1;
    }
    else if (!(difSignsFloat(powerMotB, powerMotC)))
    {
        // same signs
        if (ERR_MODE != 2)
        {
            setStartDegMotBCAndReset(getEncoderB(), getEncoderC());
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

float TEST_CALIBRATION_POWB;
float TEST_CALIBRATION_POWC;
int TEST_CALIBRATION_MEASURE_GAP;
float TEST_CALIBRATION_LAST_DEGB;
float TEST_CALIBRATION_LAST_DEGC;
float TEST_CALIBRATION_MEASUREDB;
float TEST_CALIBRATION_MEASUREDC;

task task_testMotCalibration()
{
    TEST_CALIBRATION_LAST_DEGB = 0;
    TEST_CALIBRATION_LAST_DEGC = 0;
    while (true)
    {
        sleep(TEST_CALIBRATION_MEASURE_GAP);
        float nowDegB = getEncoderB();
        float nowDegC = getEncoderC();
        TEST_CALIBRATION_MEASUREDB = nowDegB - TEST_CALIBRATION_LAST_DEGB;
        TEST_CALIBRATION_MEASUREDC = nowDegC - TEST_CALIBRATION_LAST_DEGC;
    }
}

void testMotCalibration(short powB, short powC, int measureGap)
{
    TEST_CALIBRATION_POWB = powB;
    TEST_CALIBRATION_POWC = powC;
    TEST_CALIBRATION_MEASURE_GAP = measureGap;
    resetMotorEncoder(motB);
    resetMotorEncoder(motC);
    setNewMotBCPowersAndRatio(TEST_CALIBRATION_POWB, TEST_CALIBRATION_POWC);
    startTask(task_testMotCalibration);
    sleep(200);
    while (getButtonPress(buttonEnter) == false)
    {
        float nowDegB = getEncoderB();
        float nowDegC = getEncoderC();
        TEST_CALIBRATION_LAST_DEGB = nowDegB;
        TEST_CALIBRATION_LAST_DEGC = nowDegC;
        eraseDisplay();
        displayCenteredTextLine(0, "Aim: %f", TEST_CALIBRATION_POWB / TEST_CALIBRATION_POWC);
        if (TEST_CALIBRATION_MEASUREDC == 0)
        {
            displayCenteredTextLine(2, "Measured: wheel blocked!");
        }
        else
        {
            displayCenteredTextLine(2, "Measured: %f", TEST_CALIBRATION_MEASUREDB / TEST_CALIBRATION_MEASUREDC);
        }
        displayCenteredTextLine(4, "All measured: %f", (float)nowDegB / nowDegC);
        if (TEST_CALIBRATION_MEASUREDC == 0)
        {
            displayCenteredTextLine(6, "dif: wheel blocked!");
        }
        else
        {
            displayCenteredTextLine(6, "dif: %f", fabs(TEST_CALIBRATION_POWB / TEST_CALIBRATION_POWC - TEST_CALIBRATION_MEASUREDB / TEST_CALIBRATION_MEASUREDC));
        }
        displayCenteredTextLine(8, "All dif: %f", fabs(TEST_CALIBRATION_POWB / TEST_CALIBRATION_POWC - (float)nowDegB / nowDegC));
        displayCenteredTextLine(10, "realPowB: %d  realPowC: %d", motor[motB], motor[motC]);
        sleep(150);
    }
    stopTask(task_testMotCalibration);
}

void testMotorCalibrationDebStream(short powB, short powC)
{
    setNewMotBCPowersAndRatio(powB, powC);
    float nowDegB, nowDegC, aim = (float)   powB / powC;
    while(getButtonPress(buttonEnter) == false)
    {
        nowDegB = getEncoderB();
        nowDegC = getEncoderC();
        // degB, degC, aim
        writeDebugStreamLine("%d; %d; %f; %d; %d", nowDegB, nowDegC, aim, motor[motB], motor[motC]);
        sleep(5);
    } 
}