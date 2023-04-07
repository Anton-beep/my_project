#include "tools.c"
#include "PID_c_motors.c"

void moveBC(float dist, float powB, float powC)
{
    int startDegB = nMotorEncoder[motB];
    if (powB > 0)
    {
        int endDegB = startDegB + cmToDeg(dist);
        setNewMotBCPowersAndRatio(powB, powC);
        while (nMotorEncoder[motB] < endDegB)
            ;
    }
    else
    {
        int endDegB = startDegB - cmToDeg(dist);
        setNewMotBCPowersAndRatio(powB, powC);
        while (nMotorEncoder[motB] > endDegB)
            ;
    }
}

void moveB(float dist, float pow)
{
    int startDegB = nMotorEncoder[motB];
    if (pow > 0)
    {
        int endDegB = startDegB + cmToDeg(dist);
        setNewMotBCPowersAndRatio(pow, 0);
        while (nMotorEncoder[motB] < endDegB)
            ;
    }
    else
    {
        int endDegB = startDegB - cmToDeg(dist);
        setNewMotBCPowersAndRatio(pow, 0);
        while (nMotorEncoder[motB] > endDegB)
            ;
    }
}

void moveC(float dist, float pow)
{
    int startDegC = nMotorEncoder[motC];
    if (pow > 0)
    {
        int endDegC = startDegC  + cmToDeg(dist);
        setNewMotBCPowersAndRatio(pow, 0);
        while (nMotorEncoder[motC] < endDegC)
            ;
    }
    else
    {
        int endDegC = startDegC - cmToDeg(dist);
        setNewMotBCPowersAndRatio(pow, 0);
        while (nMotorEncoder[motC] > endDegC)
            ;
    }
}

/*
Will not across the border MOTORS_MIN_POWER and MOTORS_MAX_POWER in any situation
Return true, when nothing to change more, false in other cases
*/
bool applyNewAccels(short *powB, short *powC, short *newPowB, short *newPowC)
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

void moveBCCustomAccelMainB(float dist, short powB, short powC, float accelB, float accelC)
{
    int startDegB = nMotorEncoder[motB];
    int startDegC = nMotorEncoder[motC];

    short newPowB;
    short newPowC;
    clearTimer(T1);
    bool flagAccel = false;
    if (powB >= 0)
    {
        int endDegB = startDegB + cmToDeg(dist);
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
        int endDegB = startDegB - cmToDeg(dist);
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
}

void moveBCCustomAccelMainC(float dist, short powB, short powC, float accelB, float accelC)
{
    int startDegB = nMotorEncoder[motB];
    int startDegC = nMotorEncoder[motC];

    short newPowB;
    short newPowC;
    clearTimer(T1);
    bool flagAccel = false;
    if (powC >= 0)
    {
        int endDegC = startDegC + cmToDeg(dist);
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
        int endDegC = startDegC - cmToDeg(dist);
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

// moving forward or curves
// -------------------------------------------------------------------------------------------------------------------------
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

void moveBCEqualAccels(float dist, short startPow, short accel)
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

// turns one wheel or tank
// -------------------------------------------------------------------------------------------------------------------------

void turnBDegr(float circleDeg1, float circleDeg2, float circleDeg3, float startPow, float maxPow, float endPow)
{
    moveB3Parts(circleDegToCm(circleDeg1), circleDegToCm(circleDeg2), circleDegToCm(circleDeg3), startPow, maxPow, endPow);
}

void turnCDegr(float circleDeg1, float circleDeg2, float circleDeg3, float startPow, float maxPow, float endPow)
{
    moveC3Parts(circleDegToCm(circleDeg1), circleDegToCm(circleDeg2), circleDegToCm(circleDeg3), startPow, maxPow, endPow);
}

// NS - no sensors
// clock turn -> +
void tankTurnNS3Parts(float circleDeg1, float circleDeg2, float circleDeg3, float startPow, float maxPow, float endPow){
    float spb, spc, mpb, mpc, epb, epc;
    if (circleDeg1 >= 0){
        spb = startPow;
        spc = startPow * -1;
    }
    else{
        spb = startPow * -1;
        spc = startPow;
    }
    if (circleDeg2 >= 0){
        mpb = maxPow;
        mpc = maxPow * -1;
    }
    else{
        mpb = maxPow * -1;
        mpc = maxPow;
    }
    if (circleDeg3 >= 0){
        epb = endPow;
        epc = endPow * -1;
    }
    else{
        epb = endPow * -1;
        epc = endPow;
    }
    moveBC3Parts(circleDegToCm(circleDeg1), circleDegToCm(circleDeg2), circleDegToCm(circleDeg3), spb, spc, mpb, mpc, epb, epc);
}