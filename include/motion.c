#include "tools.c"
#include "PID_c_motors.c"

float DEG_DEFECT_B;
float DEG_DEFECT_C;

void moveBC(float dist, float powB, float powC)
{
    int startDegB = nMotorEncoder[motB];
    float endDegB;
    if (powB > 0)
    {
        endDegB = startDegB + cmToDeg(dist);
        setNewMotBCPowersAndRatio(powB, powC);
        while (nMotorEncoder[motB] < endDegB)
            ;
    }
    else
    {
        endDegB = startDegB - cmToDeg(dist);
        setNewMotBCPowersAndRatio(powB, powC);
        while (nMotorEncoder[motB] > endDegB)
            ;
    }
    DEG_DEFECT_B += getFractionalPart(endDegB);
    tryRepairDefect(powB, powC);
}

void moveB(float dist, float pow)
{
    int startDegB = nMotorEncoder[motB];
    float endDegB;
    if (pow > 0)
    {
        endDegB = startDegB + cmToDeg(dist);
        setNewMotBCPowersAndRatio(pow, 0);
        while (nMotorEncoder[motB] < endDegB)
            ;
    }
    else
    {
        endDegB = startDegB - cmToDeg(dist);
        setNewMotBCPowersAndRatio(pow, 0);
        while (nMotorEncoder[motB] > endDegB)
            ;
    }
    DEG_DEFECT_B += getFractionalPart(endDegB);
    tryRepairDefect(pow, pow);
}

void moveC(float dist, float pow)
{
    int startDegC = nMotorEncoder[motC];
    float endDegC;
    if (pow > 0)
    {
        endDegC = startDegC + cmToDeg(dist);
        setNewMotBCPowersAndRatio(pow, 0);
        while (nMotorEncoder[motC] < endDegC)
            ;
    }
    else
    {
        endDegC = startDegC - cmToDeg(dist);
        setNewMotBCPowersAndRatio(pow, 0);
        while (nMotorEncoder[motC] > endDegC)
            ;
    }
    DEG_DEFECT_C += getFractionalPart(endDegC);
    tryRepairDefect(pow, pow);
}

void tryRepairDefect(float powB, float powC)
{
    short repB = (short)DEG_DEFECT_B;
    short repC = (short)DEG_DEFECT_C;

    if (repB < 0){
        powB = powB * -1;
        repB = repB * -1;
    }
    if (repC < 0){
        powC = powC * -1;
        repC = repC * -1;
    } 

    if (repB == repC)
    {
        moveBC(repB, powB, powC);
    }
    else
    {
        if (repB != 0)
        {
            moveB(repB, powB, powC);
        }
        if (repC != 0)
        {
            moveC(repC, powB, powC);
        }
    }
}
/*
Will not across the border MOTORS_MIN_POWER and MOTORS_MAX_POWER in any situation
Return true, when nothing to change more, false in other cases
*/
bool applyNewAccels(float *powB, float *powC, float *newPowB, float *newPowC)
{

    bool newBInBorder = *newPowB >= MOTORS_MIN_POWER && *newPowB <= MOTORS_MAX_POWER;
    bool newCInBorder = *newPowC >= MOTORS_MIN_POWER && *newPowC <= MOTORS_MAX_POWER;
    bool oldBInBorder = *powB >= MOTORS_MIN_POWER && *powB <= MOTORS_MAX_POWER;
    bool oldCInBorder = *powC >= MOTORS_MIN_POWER && *powC <= MOTORS_MAX_POWER;
    bool bExitingLim = !(oldBInBorder) && ((*powB<MOTORS_MIN_POWER && * newPowB> * powB) || (*powB > MOTORS_MAX_POWER && *newPowB < *powB));
    bool cExitingLim = !(oldCInBorder) && ((*powC<MOTORS_MIN_POWER && * newPowC> * powC) || (*powC > MOTORS_MAX_POWER && *newPowC < *powC));

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
    int startDegB = nMotorEncoder[motB];
    int startDegC = nMotorEncoder[motC];

    float newPowB;
    float newPowC;
    clearTimer(T1);
    bool flagAccel = false;
    float endDegB;
    if (powB >= 0)
    {
        endDegB = startDegB + cmToDeg(dist);
        setNewMotBCPowersAndRatio(powB, powC);
        while (nMotorEncoder[motB] < endDegB)
        {
            if (!(flagAccel) && time1[T1] % 100 == 0)
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
                flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
            }
        }
    }
    else
    {
        endDegB = startDegB - cmToDeg(dist);
        setNewMotBCPowersAndRatio(powB, powC);
        while (nMotorEncoder[motB] > endDegB)
        {
            if (!(flagAccel) && time1[T1] % 100 == 0)
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
                flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
            }
        }
    }
    DEG_DEFECT_B += getFractionalPart(endDegB);
    tryRepairDefect(POWER_MOT_B, POWER_MOT_C);
}

void moveBCCustomAccelMainC(float dist, float powB, float powC, float accelB, float accelC)
{
    int startDegB = nMotorEncoder[motB];
    int startDegC = nMotorEncoder[motC];

    float newPowB;
    float newPowC;
    clearTimer(T1);
    bool flagAccel = false;
    float endDegC;
    if (powC >= 0)
    {
        endDegC = startDegC + cmToDeg(dist);
        setNewMotBCPowersAndRatio(powB, powC);
        while (nMotorEncoder[motC] < endDegC)
        {
            if (!(flagAccel) && time1[T1] % 100 == 0)
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
                flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
            }
        }
    }
    else
    {
        endDegC = startDegC - cmToDeg(dist);
        setNewMotBCPowersAndRatio(powB, powC);
        while (nMotorEncoder[motC] > endDegC)
        {
            if (!(flagAccel) && time1[T1] % 100 == 0)
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
                flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
            }
        }
    }
    DEG_DEFECT_C += getFractionalPart(endDegC);
    tryRepairDefect(POWER_MOT_B, POWER_MOT_C);
}

void moveBCAccelPartMainB(float dist, float startPowB, float startPowC, float endPowB, float endPowC)
{
    float accelB = (pow(endPowB, 2) - pow(startPowB, 2)) / (cmToDeg(dist) * 2);
    float accelC = (pow(endPowC, 2) - pow(startPowC, 2)) / (cmToDeg(dist) * 2);
    moveBCCustomAccelMainB(dist, startPowB, startPowC, accelB, accelC);
}

void moveBCAccelPartMainC(float dist, float startPowB, float startPowC, float endPowB, float endPowC)
{
    float accelB = (pow(endPowB, 2) - pow(startPowB, 2)) / (cmToDeg(dist) * 2);
    float accelC = (pow(endPowC, 2) - pow(startPowC, 2)) / (cmToDeg(dist) * 2);
    moveBCCustomAccelMainC(dist, startPowB, startPowC, accelB, accelC);
}

// moving forward or curves
// -------------------------------------------------------------------------------------------------------------------------
void moveBCEqualAccelPart(float dist, float startPow, float endPow)
{
    moveBCAccelPartMainB(dist, startPow, startPow, endPow, endPow);
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
    float dist1 = circleDegToCm(circleDeg1);
    float dist2 = circleDegToCm(circleDeg2);
    float dist3 = circleDegToCm(circleDeg3);
    moveB3Parts(dist1, dist2, dist3, startPow, maxPow, endPow);
}

void turnCDegr(float circleDeg1, float circleDeg2, float circleDeg3, float startPow, float maxPow, float endPow)
{
    float dist1 = circleDegToCm(circleDeg1);
    float dist2 = circleDegToCm(circleDeg2);
    float dist3 = circleDegToCm(circleDeg3);
    moveC3Parts(dist1, dist2, dist3, startPow, maxPow, endPow);
}

// NS - no sensors
// clock turn -> +
void tankTurnNS3Parts(float circleDeg1, float circleDeg2, float circleDeg3, float startPow, float maxPow, float endPow)
{
    float dist1 = circleDegToCm(circleDeg1);
    float dist2 = circleDegToCm(circleDeg2);
    float dist3 = circleDegToCm(circleDeg3);
    moveBC3Parts(dist1, dist2, dist3, startPow, startPow * -1, maxPow, maxPow * -1, endPow, endPow * -1);
}