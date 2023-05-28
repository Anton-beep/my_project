#include "structures.c"
#include "PID_c.c"
#include "../robot_cfg.c"

void stopA()
{
    motor[motA] = 0;
    setMotorBrakeMode(motA, motorBrake);
}

void stopD()
{
    motor[motD] = 0;
    setMotorBrakeMode(motD, motorBrake);
}

void setPowerAdjustBatteryManipA(short pow, float batVoltage)
{
    if (pow == 0)
    {
        motor[motA] = 0;
    }
    else
    {
        float val = pow * batVoltage / getBatteryVoltage();
        if (val < 0)
        {
            motor[motA] = min(-1, val);
        }
        else
        {
            motor[motA] = max(1, val);
        }
    }
}

void setPowerAdjustBatteryManipD(short pow, float batVoltage)
{
    if (pow == 0)
    {
        motor[motD] = 0;
    }
    else
    {
        float val = pow * batVoltage / getBatteryVoltage();
        if (val < 0)
        {
            motor[motD] = min(-1, val);
        }
        else
        {
            motor[motD] = max(1, val);
        }
    }
}

void testAdjust()
{
    setPowerAdjustBatteryManipA(80, 7.6);
    while (true)
    {
        writeDebugStreamLine("%f; %d", getBatteryVoltage(), getSignedRPM(motA));
        sleep(1);
    }
}

bool MANIP_A_READY = true;
bool MANIP_D_READY = true;

void waitForManipA()
{
    while (!MANIP_A_READY)
    {
        sleep(1);
    }
}

void waitForManipD()
{
    while (!MANIP_D_READY)
    {
        sleep(1);
    }
}

short TIME_MANIP_A_START_POW;
short TIME_MANIP_A_AFTER_POW;
float TIME_MANIP_A_VOLTAGE;
int TIME_MANIP_A_TIME;

task moveTimeManipA()
{
    MANIP_A_READY = false;
    setPowerAdjustBatteryManipA(TIME_MANIP_A_START_POW, TIME_MANIP_A_VOLTAGE);
    sleep(TIME_MANIP_A_TIME);
    setPowerAdjustBatteryManipA(TIME_MANIP_A_AFTER_POW, TIME_MANIP_A_VOLTAGE);
    MANIP_A_READY = true;
}

short TIME_MANIP_D_START_POW;
short TIME_MANIP_D_AFTER_POW;
float TIME_MANIP_D_VOLTAGE;
int TIME_MANIP_D_TIME;

task moveTimeManipD()
{
    MANIP_D_READY = false;
    setPowerAdjustBatteryManipD(TIME_MANIP_D_START_POW, TIME_MANIP_D_VOLTAGE);
    sleep(TIME_MANIP_D_TIME);
    setPowerAdjustBatteryManipD(TIME_MANIP_D_AFTER_POW, TIME_MANIP_D_VOLTAGE);
    MANIP_D_READY = true;
}

int DEG_MANIP_A_DEG;
short DEG_MANIP_A_START_POW;
short DEG_MANIP_A_AFTER_POW;
float DEG_MANIP_A_VOLTAGE;

task moveDegManipA()
{
    MANIP_A_READY = false;
    setPowerAdjustBatteryManipA(DEG_MANIP_A_START_POW, DEG_MANIP_A_VOLTAGE);
    int endDeg;
    if (DEG_MANIP_A_START_POW >= 0)
    {
        endDeg = getEncoderA() + DEG_MANIP_A_DEG;
        while (getEncoderA() < endDeg)
        {
            sleep(1);
        }
    }
    else
    {
        endDeg = getEncoderA() - DEG_MANIP_A_DEG;
        while (getEncoderA() > endDeg)
        {
            sleep(1);
        }
    }
    setPowerAdjustBatteryManipA(DEG_MANIP_A_AFTER_POW, DEG_MANIP_A_VOLTAGE);
    MANIP_A_READY = true;
}

int DEG_MANIP_D_DEG;
short DEG_MANIP_D_START_POW;
short DEG_MANIP_D_AFTER_POW;
float DEG_MANIP_D_VOLTAGE;

task moveDegManipD()
{
    MANIP_D_READY = false;
    setPowerAdjustBatteryManipD(DEG_MANIP_D_START_POW, DEG_MANIP_D_VOLTAGE);
    int endDeg;
    if (DEG_MANIP_D_START_POW >= 0)
    {
        endDeg = getEncoderD() + DEG_MANIP_D_DEG;
        while (getEncoderD() < endDeg)
        {
            sleep(1);
        }
    }
    else
    {
        endDeg = getEncoderD() - DEG_MANIP_D_DEG;
        while (getEncoderD() > endDeg)
        {
            sleep(1);
        }
    }
    setPowerAdjustBatteryManipD(DEG_MANIP_D_AFTER_POW, DEG_MANIP_D_VOLTAGE);
    MANIP_D_READY = true;
}

bool *setTimeManipA(int time, short powStart, short powAfter, float voltage = 7.9)
{
    stopTask(moveTimeManipA);
    stopTask(moveDegManipA);
    TIME_MANIP_A_START_POW = powStart;
    TIME_MANIP_A_AFTER_POW = powAfter;
    TIME_MANIP_A_VOLTAGE = voltage;
    TIME_MANIP_A_TIME = time;
    startTask(moveTimeManipA);
    return &MANIP_A_READY;
}

bool *setTimeManipD(int time, short powStart, short powAfter, float voltage = 7.9)
{
    stopTask(moveTimeManipD);
    stopTask(moveDegManipD);
    TIME_MANIP_D_START_POW = powStart;
    TIME_MANIP_D_AFTER_POW = powAfter;
    TIME_MANIP_D_VOLTAGE = voltage;
    TIME_MANIP_D_TIME = time;
    startTask(moveTimeManipD);
    return &MANIP_D_READY;
}

bool *setDegManipA(int deg, short powStart, short powAfter, float voltage = 8)
{
    stopTask(moveDegManipA);
    stopTask(moveTimeManipA);
    DEG_MANIP_A_DEG = deg;
    DEG_MANIP_A_START_POW = powStart;
    DEG_MANIP_A_AFTER_POW = powAfter;
    DEG_MANIP_A_VOLTAGE = voltage;
    startTask(moveDegManipA);
    return &MANIP_A_READY;
}

bool *setDegManipD(int deg, short powStart, short powAfter, float voltage = 8)
{
    stopTask(moveDegManipD);
    stopTask(moveTimeManipD);
    DEG_MANIP_D_DEG = deg;
    DEG_MANIP_D_START_POW = powStart;
    DEG_MANIP_D_AFTER_POW = powAfter;
    DEG_MANIP_D_VOLTAGE = voltage;
    startTask(moveDegManipD);
    return &MANIP_D_READY;
}

void runMot(short mot)
{
    int pow = 0;
    bool pressedUp = false;
    bool pressedDown = false;
    while (getButtonPress(buttonEnter) == false)
    {
        if (getButtonPress(buttonLeft))
            motor[mot] = -1 * pow;
        else if (getButtonPress(buttonRight))
            motor[mot] = pow;
        else
            motor[mot] = 0;

        if (getButtonPress(buttonUp))
            pressedUp = true;
        else if (getButtonPress(buttonDown))
            pressedDown = true;

        if (getButtonPress(buttonUp) == false && pressedUp == true)
        {
            pressedUp = false;
            pow++;
        }
        else if (getButtonPress(buttonDown) == false && pressedDown == true)
        {
            pressedDown = false;
            pow--;
        }
        eraseDisplay();
        displayTextLine(1, "Pow: %d", pow);
        sleep(100);
    }
}