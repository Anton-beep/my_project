#include "tools.c"
#include "PID_c_motors.c"

void moveBC(float dist, short powB, short powC)
{
    int startDegB = nMotorEncoder[motB];
    if (powB > 0)
    {
        int endDegB = startDegB + cmToDeg(dist);
        setNewMotBCPowers(powB, powC);
        while (nMotorEncoder[motB] < endDegB)
            ;
    }
    else
    {
        int endDegB = startDegB - cmToDeg(dist);
        setNewMotBCPowers(powB, powC);
        while (nMotorEncoder[motB] > endDegB)
            ;
    }
}

void checkStopAccelAndChange(short *powB, short *powC, short *newPowB, short *newPowC)
{
    *newPowB = min(MOTORS_MAX_POWER, max(MOTORS_MIN_POWER, *newPowB));
    *newPowC = min(MOTORS_MAX_POWER, max(MOTORS_MIN_POWER, *newPowC));
}

void moveBCCustomAccelMainB(float dist, short powB, short powC, float accelB, float accelC)
{
    int startDegB = nMotorEncoder[motB];
    int startDegC = nMotorEncoder[motC];

    short newPowB;
    short newPowC;
    clearTimer(T1);
    if (powB >= 0)
    {
        int endDegB = startDegB + cmToDeg(dist);
        setNewMotBCPowers(powB, powC);
        while (nMotorEncoder[motB] < endDegB)
        {
            if (time1[T1] % 100 == 0)
            {
                newPowB = round(sqrt(fabs(nMotorEncoder[motB] - startDegB) * 2 * accelB + pow(powB, 2)));
                if (powC >= 0)
                {
                    newPowC = round(sqrt(fabs(nMotorEncoder[motC] - startDegC) * 2 * accelC + pow(powC, 2)));
                }
                else
                {
                    newPowC = -1 * round(sqrt(fabs(nMotorEncoder[motC] - startDegC) * 2 * accelC + pow(powC, 2)));
                }
                checkStopAccelAndChange(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
                setNewMotBCPowers(newPowB, newPowC);
            }
        }
    }
    else
    {
        int endDegB = startDegB - cmToDeg(dist);
        setNewMotBCPowers(powB, powC);
        while (nMotorEncoder[motB] > endDegB)
        {
            if (time1[T1] % 100 == 0)
            {
                newPowB = -1 * round(sqrt(fabs(nMotorEncoder[motB] - startDegB) * 2 * accelB + pow(powB, 2)));
                if (powC >= 0)
                {
                    newPowC = round(sqrt(fabs(nMotorEncoder[motC] - startDegC) * 2 * accelC + pow(powC, 2)));
                }
                else
                {
                    newPowC = -1 * round(sqrt(fabs(nMotorEncoder[motC] - startDegC) * 2 * accelC + pow(powC, 2)));
                }
                checkStopAccelAndChange(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
                setNewMotBCPowers(newPowB, newPowC);
                sleep(100);
            }
        }
    }
}

void moveBCCustomAccelMainC(float dist, short powB, short powC, float accelB, float accelC)
{
    int startDegB = nMotorEncoder[motB];
    int startDegC = nMotorEncoder[motC];

    short newPowB;
    short newPowC;
    clearTimer(T1);
    if (powC >= 0)
    {
        int endDegC = startDegC + cmToDeg(dist);
        setNewMotBCPowers(powB, powC);
        while (nMotorEncoder[motC] < endDegC)
        {
            if (time1[T1] % 100 == 0)
            {
                newPowC = round(sqrt(fabs(nMotorEncoder[motC] - startDegC) * 2 * accelC + pow(powC, 2)));
                if (powB >= 0)
                {
                    newPowB = round(sqrt(fabs(nMotorEncoder[motB] - startDegB) * 2 * accelB + pow(powB, 2)));
                }
                else
                {
                    newPowB = -1 * round(sqrt(fabs(nMotorEncoder[motB] - startDegB) * 2 * accelB + pow(powB, 2)));
                }
                checkStopAccelAndChange(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
                setNewMotBCPowers(newPowB, newPowC);
                sleep(100);
            }
        }
    }
    else
    {
        int endDegC = startDegC - cmToDeg(dist);
        setNewMotBCPowers(powB, powC);
        while (nMotorEncoder[motC] > endDegC)
        {
            if (time1[T1] % 100 == 0)
            {
                newPowC = -1 * round(sqrt(fabs(nMotorEncoder[motC] - startDegC) * 2 * accelC + pow(powC, 2)));
                if (powB > 0)
                {
                    newPowB = round(sqrt(fabs(nMotorEncoder[motB] - startDegB) * 2 * accelB + pow(powB, 2)));
                }
                else
                {
                    newPowB = -1 * round(sqrt(fabs(nMotorEncoder[motB] - startDegB) * 2 * accelC + pow(powB, 2)));
                }
                checkStopAccelAndChange(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
                setNewMotBCPowers(newPowB, newPowC);
                sleep(100);
            }
        }
    }
}

void moveBCAccelPartMainB(float dist, short startPowB, short startPowC, short endPowB, short endPowC)
{
    float accelB = (pow(endPowB, 2) - pow(startPowB, 2)) / (cmToDeg(dist) * 2);
    float accelC = (pow(endPowC, 2) - pow(startPowC, 2)) / (cmToDeg(dist) * 2);
    moveBCCustomAccelMainB(dist, startPowB, startPowC, accelB, accelC);
}

void moveBCAccelPartMainC(float dist, short startPowB, short startPowC, short endPowB, short endPowC)
{
    float accelB = (pow(endPowB, 2) - pow(startPowB, 2)) / (cmToDeg(dist) * 2);
    float accelC = (pow(endPowC, 2) - pow(startPowC, 2)) / (cmToDeg(dist) * 2);
    moveBCCustomAccelMainC(dist, startPowB, startPowC, accelB, accelC);
}

void moveBCEqualAccelPart(float dist, short startPow, short endPow)
{
    moveBCAccelPartMainB(dist, startPow, startPow, endPow, endPow);
}

void moveBAccel(float dist, short startPow, float accel)
{
    moveBCCustomAccelMainB(dist, startPow, 0, accel, 0);
}

void moveCAccel(float dist, short startPow, float accel)
{
    moveBCCustomAccelMainC(dist, 0, startPow, 0, accel);
}

void moveBCEqualAccels (float dist, short startPow, short accel)
{
    moveBCCustomAccelMainB(dist, startPow, startPow, accel, accel);
}

void moveBPart(float dist, short startPow, short endPow)
{
    moveBCAccelPartMainB(dist, startPow, 0, endPow, 0);
}

void moveCPart(float dist, short startPow, short endPow)
{
    moveBCAccelPartMainC(dist, 0, startPow, 0, endPow);
}

void moveBC3Parts(float dist1, float dist2, float dist3, short startPowB, short startPowC, short maxPowB, short maxPowC, short endPowB, short endPowC)
{
    moveBCAccelPartMainB(dist1, startPowB, startPowC, maxPowB, maxPowC);
    moveBC(dist2, maxPowB, maxPowC);
    moveBCAccelPartMainB(dist3, maxPowB, maxPowC, endPowB, endPowC);
}

void moveBC3PartsEqual(float dist1, float dist2, float dist3, short startPow, short maxPow, short endPow)
{
    moveBC3Parts(dist1, dist2, dist3, startPow, startPow, maxPow, maxPow, endPow, endPow);
}

void moveBC3Accels(float dist1, float dist2, float dist3, short startPowB, short startPowC, float accelB1, float accelC1, float accelB2, float accelC2)
{
    moveBCCustomAccelMainB(dist1, startPowB, startPowC, accelB1, accelC1);
    moveBC(dist2, POWER_MOT_B, POWER_MOT_B);
    moveBCCustomAccelMainB(dist3, POWER_MOT_B, POWER_MOT_C, accelB2, accelC2);
}

void moveBC3Def(float dist, short startPow, short endPow)
{
    moveBCEqualAccels(dist * DEF_RAT_1PART, startPow, DEF_ACCEL_INC);
    moveBC(dist * DEF_RAT_2PART, POWER_MOT_B, POWER_MOT_C);
    moveBCEqualAccelPart(dist * DEF_RAT_3PART, POWER_MOT_B, endPow);
}