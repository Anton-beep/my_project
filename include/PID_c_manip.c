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

void setPowerAdjustBatteryManipD(short pow, float batVoltage)
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

void testAdjust()
{
    setPowerAdjustBatteryManipA(80, 7.6);
    while (true)
    {
        writeDebugStreamLine("%f; %d", getBatteryVoltage(), getSignedRPM(motA));
        sleep(1);
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

void runMot(short mot){
    int pow = 0;
    bool pressedUp = false;
    bool pressedDown = false;
    while (getButtonPress(buttonEnter) == false){
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
        
        if (getButtonPress(buttonUp) == false && pressedUp == true){
            pressedUp = false;
            pow++;
        }
        else if (getButtonPress(buttonDown) == false && pressedDown == true){
            pressedDown = false;
            pow--;
        }
        eraseDisplay();
        displayTextLine(1, "Pow: %d", pow);
        sleep(100);
    }
}