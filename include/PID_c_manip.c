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
    motor[motA] = pow * batVoltage / getBatteryVoltage();
}

void setPowerAdjustBatteryManipD(short pow, float batVoltage)
{
    motor[motD] = pow * batVoltage / getBatteryVoltage();
}

void testAdjust()
{
    setPowerAdjustBatteryManipD(80, 7.6);
    while (getButtonPress(buttonEnter) == false)
    {
        eraseDisplay();
        displayCenteredTextLine(5, "%f", getBatteryVoltage());
        displayCenteredTextLine(10, "%d", getSignedRPM(motD));
        sleep(100);
    }
}

void moveTimeManipA(int time, short powStart, short powAfter, float voltage)
{
    setPowerAdjustBatteryManipA(powStart, voltage);
    sleep(time);
    setPowerAdjustBatteryManipA(powAfter, voltage);
}

void moveTimeManipD(int time, short powStart, short powAfter, float voltage)
{
    setPowerAdjustBatteryManipD(powStart, voltage);
    sleep(time);
    setPowerAdjustBatteryManipD(powAfter, voltage);
}

void moveDegManipA(int deg, short powStart, short powAfter, float voltage)
{
    setPowerAdjustBatteryManipA(powStart, voltage);
    int endDeg;
    if (powStart >= 0)
    {
        endDeg = nMotorEncoder[motA] + deg;
        while (nMotorEncoder[motA] < endDeg)
        {
            sleep(1);
        }
    }
    else
    {
        endDeg = nMotorEncoder[motA] - deg;
        while (nMotorEncoder[motA] > endDeg)
        {
            sleep(1);
        }
    }
    setPowerAdjustBatteryManipA(powAfter, voltage);
}

void moveDegManipD(int deg, short powStart, short powAfter, float voltage)
{
    setPowerAdjustBatteryManipD(powStart, voltage);
    int endDeg;
    if (powStart >= 0)
    {
        endDeg = nMotorEncoder[motD] + deg;
        while (nMotorEncoder[motD] < endDeg)
        {
            sleep(1);
        }
    }
    else
    {
        endDeg = nMotorEncoder[motD] - deg;
        while (nMotorEncoder[motD] > endDeg)
        {
            sleep(1);
        }
    }
}