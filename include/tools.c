#include "../robot_cfg.c"
#include "structures.c"

char *floatToString(float val)
{
    char str[10];
    snprintf(str, 10, "%f", val);
    return str;
}

bool difSignsFloat(float val1, float val2)
{
    return (val1 < 0) ? (val2 >= 0) : (val2 < 0);
}

float cmToDeg(float cm)
{
    return 360 * cm / (PI * WHEEL_DIAMETER);
}

float min(float a, float b)
{
    return a < b ? a : b;
}

float max(float a, float b)
{
    return a > b ? a : b;
}

float circleDegToCm(float cd)
{
    return BASE_LEN * PI * cd / 360;
}

float bigCircleDegToCm(float cd)
{
    return 2 * BASE_LEN * PI * cd / 360;
}

float getFractionalPart(float val)
{
    return val - (int)val;
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void checkBat()
{
    if (getBatteryVoltage() < 7.8)
    {
        playSound(soundBeepBeep);
    }
    eraseDisplay();
    displayCenteredBigTextLine(5, "voltage");
    displayCenteredBigTextLine(10, "%f", getBatteryVoltage());
}

void displayExecTime()
{
    eraseDisplay();
    displayCenteredBigTextLine(5, "seconds");
    displayCenteredBigTextLine(10, "%f", (float) time1[T3] / 1000);
    flushButtonMessages();
    waitForButtonPress();
}