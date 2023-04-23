#include "tools.c"
#include "PID_c_motors.c"

// TO CHANGE DIRECTION YOU MAY USE ONLY POWER, NOT DISTANCE!!!!!!!!!!!!!!!!

void stopBC()
{
    setNewMotBCPowersAndRatio(0, 0);
    setMotorBrakeMode(motB, motorBrake);
    setMotorBrakeMode(motC, motorBrake);
}

void stopB()
{
    setNewMotBCPowersAndRatio(0, POWER_MOT_C);
    setMotorBrakeMode(motB, motorBrake);
}

void stopC()
{
    setNewMotBCPowersAndRatio(POWER_MOT_B, 0);
    setMotorBrakeMode(motB, motorBrake);
}

void coastBC()
{
    setNewMotBCPowersAndRatio(0, 0);
    setMotorBrakeMode(motB, motorCoast);
    setMotorBrakeMode(motC, motorCoast);
}

void moveBC(float dist, float powB, float powC)
{
    setStartDegMotBCAndReset(nMotorEncoder[motB], nMotorEncoder[motC]);
    int startDegB = nMotorEncoder[motB];
    float endDegB;
    if (powB > 0)
    {
        endDegB = startDegB + dist;
        setNewMotBCPowersAndRatio(powB, powC);
        while (nMotorEncoder[motB] < endDegB)
            ;
    }
    else
    {
        endDegB = startDegB - dist;
        setNewMotBCPowersAndRatio(powB, powC);
        while (nMotorEncoder[motB] > endDegB)
            ;
    }
}

void moveBCMainC(float dist, float powB, float powC)
{
    setStartDegMotBCAndReset(nMotorEncoder[motB], nMotorEncoder[motC]);
    int startDegC = nMotorEncoder[motC];
    float endDegC;
    if (powC > 0)
    {
        endDegC = startDegC + dist;
        setNewMotBCPowersAndRatio(powB, powC);
        while (nMotorEncoder[motC] < endDegC)
            ;
    }
    else
    {
        endDegC = startDegC - dist;
        setNewMotBCPowersAndRatio(powB, powC);
        while (nMotorEncoder[motC] > endDegC)
            ;
    }
}

void moveB(float dist, float pow)
{
    setStartDegMotBCAndReset(nMotorEncoder[motB], nMotorEncoder[motC]);
    int startDegB = nMotorEncoder[motB];
    float endDegB;
    if (pow > 0)
    {
        endDegB = startDegB + dist;
        setNewMotBCPowersAndRatio(pow, 0);
        while (nMotorEncoder[motB] < endDegB)
            ;
    }
    else
    {
        endDegB = startDegB - dist;
        setNewMotBCPowersAndRatio(pow, 0);
        while (nMotorEncoder[motB] > endDegB)
            ;
    }
}

void moveC(float dist, float pow)
{
    setStartDegMotBCAndReset(nMotorEncoder[motB], nMotorEncoder[motC]);
    int startDegC = nMotorEncoder[motC];
    float endDegC;
    if (pow > 0)
    {
        endDegC = startDegC + dist;
        setNewMotBCPowersAndRatio(0, pow);
        while (nMotorEncoder[motC] < endDegC)
            ;
    }
    else
    {
        endDegC = startDegC - dist;
        setNewMotBCPowersAndRatio(0, pow);
        while (nMotorEncoder[motC] > endDegC)
            ;
    }
}

/*
Will not across the border MOTORS_MIN_POWER and MOTORS_MAX_POWER in any situation
Return true, when nothing to change more, false in other cases
*/
bool applyNewAccels(short *powB, short *powC, float *newPowB, float *newPowC)
{
    int bufNewPowB = fabs(*newPowB);
    int bufNewPowC = fabs(*newPowC);
    int bufPowB = fabs(*powB);
    int bufPowC = fabs(*powC);

    bool newBInBorder = bufNewPowB >= MOTORS_MIN_POWER && bufNewPowB <= MOTORS_MAX_POWER;
    bool newCInBorder = bufNewPowC >= MOTORS_MIN_POWER && bufNewPowC <= MOTORS_MAX_POWER;
    bool oldBInBorder = bufPowB >= MOTORS_MIN_POWER && bufPowB <= MOTORS_MAX_POWER;
    bool oldCInBorder = bufPowC >= MOTORS_MIN_POWER && bufPowC <= MOTORS_MAX_POWER;
    bool bExitingLim = !(oldBInBorder) && ((bufPowB < MOTORS_MIN_POWER && bufNewPowB > bufPowB) || (bufPowB > MOTORS_MAX_POWER && bufNewPowB < bufPowB));
    bool cExitingLim = !(oldCInBorder) && ((bufPowC < MOTORS_MIN_POWER && bufNewPowC > bufPowC) || (bufPowC > MOTORS_MAX_POWER && bufNewPowC < bufPowC));

    bool applyB = false;
    bool applyC = false;
    if (bExitingLim || newBInBorder)
    {
        applyB = true;
    }
    if (cExitingLim || newCInBorder)
    {
        applyC = true;
    }

    writeDebugStreamLine("%d %d", POWER_MOT_B, POWER_MOT_C);
    if (applyB && applyC)
    {
        setNewMotBCPowersAndRatio(*newPowB, *newPowC);
    }
    else if (applyB)
    {
        setNewMotBCPowersAndRatio(*newPowB, POWER_MOT_C);
    }
    else if (applyC)
    {
        setNewMotBCPowersAndRatio(POWER_MOT_B, *newPowC);
    }
    else
    {
        return (!(newBInBorder) && !(bExitingLim)) || (!(newCInBorder) && !(cExitingLim));
    }
    return false;
}

void moveBCCustomAccelMainB(float dist, float powB, float powC, float accelB, float accelC)
{
    setStartDegMotBCAndReset(nMotorEncoder[motB], nMotorEncoder[motC]);
    if (accelB == 0 && accelC == 0)
    {
        moveBC(dist, powB, powC);
    }
    else
    {
        int startDegB = nMotorEncoder[motB], startDegC = nMotorEncoder[motC];

        float newPowB, newPowC, endDegB;
        float startTime = nPgmTime;
        bool flagAccel = false;
        if (powB >= 0)
        {
            endDegB = startDegB + dist;
            // if (powC > 0)
            // {
            //     setNewMotBCPowersAndRatio(80, 80);
            // }
            // else
            // {
            //     setNewMotBCPowersAndRatio(80, -80);
            // }
            while (nMotorEncoder[motB] < endDegB)
            {
                if (!(flagAccel))
                {
                    newPowB = round(sqrt(fabs(nMotorEncoder[motB] - startDegB) * 2 * accelB + pow(powB, 2)));
                    newPowC = newPowB * accelC / accelB;
                    flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
                    startTime = nPgmTime;
                }
            }
        }
        else
        {
            endDegB = startDegB - dist;
            // if (powC > 0)
            // {
            //     setNewMotBCPowersAndRatio(-80, 80);
            // }
            // else
            // {
            //     setNewMotBCPowersAndRatio(-80, -80);
            // }
            while (nMotorEncoder[motB] > endDegB)
            {
                if (!(flagAccel))
                {
                    newPowB = -1 * round(sqrt(fabs(nMotorEncoder[motB] - startDegB) * 2 * accelB * -1 + pow(powB, 2)));
                    newPowC = newPowB * accelC / accelB;
                    flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
                    startTime = nPgmTime;
                }
            }
        }
    }
}

void moveBCCustomAccelMainC(float dist, float powB, float powC, float accelB, float accelC)
{
    setStartDegMotBCAndReset(nMotorEncoder[motB], nMotorEncoder[motC]);
    if (accelB == 0 && accelC == 0)
    {
        moveBC(dist, powB, powC);
    }
    else
    {
        int startDegB = nMotorEncoder[motB], startDegC = nMotorEncoder[motC];

        float newPowB, newPowC, endDegC;
        float startTime = nPgmTime;
        bool flagAccel = false;
        if (powC >= 0)
        {
            endDegC = startDegC + dist;
            // if (powB > 0)
            // {
            //     setNewMotBCPowersAndRatio(80, 80);
            // }
            // else
            // {
            //     setNewMotBCPowersAndRatio(80, -80);
            // }
            while (nMotorEncoder[motC] < endDegC)
            {
                if (!(flagAccel))
                {
                    newPowC = round(sqrt(fabs(nMotorEncoder[motC] - startDegC) * 2 * accelC + pow(powC, 2)));
                    newPowB = newPowC * accelB / accelC;
                    flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
                    startTime = nPgmTime;
                }
            }
        }
        else
        {
            endDegC = startDegC - dist;
            // if (powB > 0)
            // {
            //     setNewMotBCPowersAndRatio(-80, 80);
            // }
            // else
            // {
            //     setNewMotBCPowersAndRatio(-80, -80);
            // }
            while (nMotorEncoder[motC] > endDegC)
            {
                if (!(flagAccel))
                {
                    newPowC = -1 * round(sqrt(fabs(nMotorEncoder[motC] - startDegC) * 2 * accelC * -1 + pow(powC, 2)));
                    newPowB = newPowC * accelB / accelC;
                    flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
                    startTime = nPgmTime;
                }
            }
        }
    }
}

void moveBCAccelPartMainB(float dist, float startPowB, float startPowC, float endPowB, float endPowC)
{
    float accelB, accelC;
    if (endPowB > startPowB)
    {
        accelB = fabs((pow(endPowB, 2) - pow(startPowB, 2)) / (dist * 2));
    }
    else
    {
        accelB = -1 * fabs((pow(endPowB, 2) - pow(startPowB, 2)) / (dist * 2));
    }
    accelC = startPowC * accelB / startPowB;
    moveBCCustomAccelMainB(dist, startPowB, startPowC, accelB, accelC);
}

void moveBCAccelPartMainC(float dist, float startPowB, float startPowC, float endPowB, float endPowC)
{
    float accelB, accelC;
    if (endPowC > startPowC)
    {
        accelC = fabs((pow(endPowC, 2) - pow(startPowC, 2)) / (dist * 2));
    }
    else
    {
        accelC = -1 * fabs((pow(endPowC, 2) - pow(startPowC, 2)) / (dist * 2));
    }
    accelB = startPowB * accelC / startPowC;
    moveBCCustomAccelMainC(dist, startPowB, startPowC, accelB, accelC);
}

void moveBCAceelPartMainBKeepRatio(float dist, float startPowB, float startPowC, float endPowB)
{
    moveBCAccelPartMainB(dist, startPowB, startPowC, endPowB, startPowC * endPowB / startPowC);
}

// moving forward or curves
// -------------------------------------------------------------------------------------------------------------------------
void moveBCEqualAccelPart(float dist, float startPow, float endPow)
{
    moveBCAceelPartMainBKeepRatio(dist, startPow * -1, startPow, endPow * -1);
}

void moveBPart(float dist, float startPow, float endPow)
{
    moveBCAccelPartMainB(dist, startPow, 0, endPow, 0);
}

void moveCPart(float dist, float startPow, float endPow)
{
    moveBCAccelPartMainC(dist, 0, startPow, 0, endPow);
}

void moveB3Parts(float dist1, float dist2, float dist3, float startPow, float maxPow, float endPow)
{
    moveBPart(dist1, startPow, maxPow);
    moveB(dist2, maxPow);
    moveBPart(dist3, maxPow, endPow);
}

void moveC3Parts(float dist1, float dist2, float dist3, float startPow, float maxPow, float endPow)
{
    moveCPart(dist1, startPow, maxPow);
    moveC(dist2, maxPow);
    moveCPart(dist3, maxPow, endPow);
}

void moveBC3Parts(float dist1, float dist2, float dist3, float startPowB, float startPowC, float maxPowB, float maxPowC, float endPowB, float endPowC)
{
    moveBCAccelPartMainB(dist1, startPowB, startPowC, maxPowB, maxPowC);
    moveBC(dist2, maxPowB, maxPowC);
    moveBCAccelPartMainB(dist3, maxPowB, maxPowC, endPowB, endPowC);
}

// turns one wheel or tank
// -------------------------------------------------------------------------------------------------------------------------

void turnBDegr(float circleDeg1, float circleDeg2, float circleDeg3, float startPow, float maxPow, float endPow)
{
    float dist1 = bigCircleDegToCm(circleDeg1);
    float dist2 = bigCircleDegToCm(circleDeg2);
    float dist3 = bigCircleDegToCm(circleDeg3);
    float deg1 = cmToDeg(dist1);
    float deg2 = cmToDeg(dist2);
    float deg3 = cmToDeg(dist3);
    moveB3Parts(deg1, deg2, deg3, startPow, maxPow, endPow);
}

void turnCDegr(float circleDeg1, float circleDeg2, float circleDeg3, float startPow, float maxPow, float endPow)
{
    float dist1 = bigCircleDegToCm(circleDeg1);
    float dist2 = bigCircleDegToCm(circleDeg2);
    float dist3 = bigCircleDegToCm(circleDeg3);
    float deg1 = cmToDeg(dist1);
    float deg2 = cmToDeg(dist2);
    float deg3 = cmToDeg(dist3);
    moveC3Parts(deg1, deg2, deg3, startPow, maxPow, endPow);
}

// NS - no sensors
// clock turn -> +
void tankTurnNS3Parts(float circleDeg1, float circleDeg2, float circleDeg3, float startPowB, float startPowC, float maxPowB, float maxPowC, float endPowB, float endPowC)
{
    float dist1 = circleDegToCm(circleDeg1);
    float dist2 = circleDegToCm(circleDeg2);
    float dist3 = circleDegToCm(circleDeg3);
    float deg1 = cmToDeg(dist1);
    float deg2 = cmToDeg(dist2);
    float deg3 = cmToDeg(dist3);
    moveBC3Parts(deg1, deg2, deg3, startPowB, startPowC, maxPowB, maxPowC, endPowB, endPowC);
}
