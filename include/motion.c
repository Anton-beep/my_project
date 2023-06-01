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
    setMotorBrakeMode(motC, motorBrake);
}

void coastBC()
{
    setNewMotBCPowersAndRatio(0, 0);
    setMotorBrakeMode(motB, motorCoast);
    setMotorBrakeMode(motC, motorCoast);
}

void moveBC(float dist, float powB, float powC)
{
    setStartDegMotBCAndReset(getEncoderB(), getEncoderC());
    int startDegB = getEncoderB();
    float endDegB;
    if (powB > 0)
    {
        endDegB = startDegB + dist;
        setNewMotBCPowersAndRatio(powB, powC);
        while (getEncoderB() < endDegB){sleep(1);}
            ;
    }
    else
    {
        endDegB = startDegB - dist;
        setNewMotBCPowersAndRatio(powB, powC);
        while (getEncoderB() > endDegB){sleep(1);}
            ;
    }
}

void moveBCMainC(float dist, float powB, float powC)
{
    setStartDegMotBCAndReset(getEncoderB(), getEncoderC());
    int startDegC = getEncoderC();
    float endDegC;
    if (powC > 0)
    {
        endDegC = startDegC + dist;
        setNewMotBCPowersAndRatio(powB, powC);
        while (getEncoderC() < endDegC){sleep(1);}
            ;
    }
    else
    {
        endDegC = startDegC - dist;
        setNewMotBCPowersAndRatio(powB, powC);
        while (getEncoderC() > endDegC){sleep(1);}
            ;
    }
}

void moveB(float dist, float pow)
{
    setStartDegMotBCAndReset(getEncoderB(), getEncoderC());
    int startDegB = getEncoderB();
    float endDegB;
    if (pow > 0)
    {
        endDegB = startDegB + dist;
        setNewMotBCPowersAndRatio(pow, 0);
        while (getEncoderB() < endDegB){sleep(1);}
            ;
    }
    else
    {
        endDegB = startDegB - dist;
        setNewMotBCPowersAndRatio(pow, 0);
        while (getEncoderB() > endDegB){sleep(1);}
            ;
    }
}

void moveC(float dist, float pow)
{
    setStartDegMotBCAndReset(getEncoderB(), getEncoderC());
    int startDegC = getEncoderC();
    float endDegC;
    if (pow > 0)
    {
        endDegC = startDegC + dist;
        setNewMotBCPowersAndRatio(0, pow);
        while (getEncoderC() < endDegC){sleep(1);}
            ;
    }
    else
    {
        endDegC = startDegC - dist;
        setNewMotBCPowersAndRatio(0, pow);
        while (getEncoderC() > endDegC){sleep(1);}
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
    setStartDegMotBCAndReset(getEncoderB(), getEncoderC());
    if (accelB == 0 && accelC == 0)
    {
        moveBC(dist, powB, powC);
    }
    else
    {
        int startDegB = getEncoderB(), startDegC = getEncoderC();

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
            while (getEncoderB() < endDegB)
            {
                if (!(flagAccel))
                {
                    newPowB = round(sqrt(fabs(getEncoderB() - startDegB) * 2 * accelB + pow(powB, 2)));
                    newPowC = newPowB * accelC / accelB;
                    flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
                    startTime = nPgmTime;
                }
                sleep(1);
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
            while (getEncoderB() > endDegB)
            {
                if (!(flagAccel))
                {
                    newPowB = -1 * round(sqrt(fabs(getEncoderB() - startDegB) * 2 * accelB * -1 + pow(powB, 2)));
                    newPowC = newPowB * accelC / accelB;
                    flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
                    startTime = nPgmTime;
                }
                sleep(1);
            }
        }
    }
}

void moveBCCustomAccelMainC(float dist, float powB, float powC, float accelB, float accelC)
{
    setStartDegMotBCAndReset(getEncoderB(), getEncoderC());
    if (accelB == 0 && accelC == 0)
    {
        moveBC(dist, powB, powC);
    }
    else
    {
        int startDegB = getEncoderB(), startDegC = getEncoderC();

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
            while (getEncoderC() < endDegC)
            {
                if (!(flagAccel))
                {
                    newPowC = round(sqrt(fabs(getEncoderC() - startDegC) * 2 * accelC + pow(powC, 2)));
                    newPowB = newPowC * accelB / accelC;
                    flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
                    startTime = nPgmTime;
                }
                sleep(1);
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
            while (getEncoderC() > endDegC)
            {
                if (!(flagAccel))
                {
                    newPowC = -1 * round(sqrt(fabs(getEncoderC() - startDegC) * 2 * accelC * -1 + pow(powC, 2)));
                    newPowB = newPowC * accelB / accelC;
                    flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
                    startTime = nPgmTime;
                }
                sleep(1);
            }
        }
    }
}

void moveBCAccelPartMainB(float dist, float startPowB, float startPowC, float endPowB, float endPowC = 0)
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

void moveBC3PartsMainC(float dist1, float dist2, float dist3, float startPowB, float startPowC, float maxPowB, float maxPowC, float endPowB, float endPowC)
{
    moveBCAccelPartMainC(dist1, startPowB, startPowC, maxPowB, maxPowC);
    moveBCMainC(dist2, maxPowB, maxPowC);
    moveBCAccelPartMainC(dist3, maxPowB, maxPowC, endPowB, endPowC);
}

// Adjust movement depending on SAFE_START_ACCEL and SAFE_END_ACCEL
void moveBCSmartAccel(int dist, float startPowB, float startPowC, float endPowB, float endPowC, float startAccel = SAFE_START_ACCEL, float endAccel = SAFE_END_ACCEL)
{
    // TODO: fix problem with endPowB and endPowC when they are 0
    if (endPowB == 0){
        if (startPowB > 0){
            endPowB = 0.01;
        }
        else {
            endPowB = -0.01;
        }
    }

    if (endPowC == 0){
        if (startPowC > 0){
            endPowC = 0.01;
        }
        else {
            endPowC = -0.01;
        }
    }

    int incDeg, decDeg, fromStartDecDeg;
    float maxPowerB, maxPowerC;
    incDeg = round((pow(MOTORS_MAX_POWER, 2) - pow(max(fabs(startPowB), fabs(startPowC)), 2)) / (2 * startAccel));
    decDeg = round((pow(MOTORS_MAX_POWER, 2) - pow(max(fabs(endPowB), fabs(endPowC)), 2)) / (2 * endAccel));
    fromStartDecDeg = round((pow(max(fabs(startPowB), fabs(startPowC)), 2) - pow(max(fabs(endPowB), fabs(endPowC)), 2)) / (2 * endAccel));

    if (fromStartDecDeg >= dist){
        playSound(soundException);

        // eraseDisplay();
        // displayCenteredTextLine(0, "fromStartDeg: %d", fromStartDecDeg);

        moveBCAccelPartMainB(dist, startPowB, startPowC, endPowB, endPowC);
    }
    else if (dist > incDeg + decDeg)
    {
        if (startPowB > 0)
        {
            maxPowerB = MOTORS_MAX_POWER;
        }
        else
        {
            maxPowerB = -1 * MOTORS_MAX_POWER;
        }
        if (startPowC > 0)
        {
            maxPowerC = MOTORS_MAX_POWER;
        }
        else
        {
            maxPowerC = -1 * MOTORS_MAX_POWER;
        }

        // eraseDisplay();
        // displayCenteredTextLine(0, "incDeg: %d", incDeg);
        // displayCenteredTextLine(1, "max dist: %d", dist - incDeg - decDeg);
        // displayCenteredTextLine(2, "decDeg: %d", decDeg);
        // displayCenteredTextLine(3, "maxPowerB: %d", maxPowerB);
        // displayCenteredTextLine(4, "maxPowerC: %d", maxPowerC);
        // displayCenteredTextLine(5, "startPowB: %d", startPowB);
        // displayCenteredTextLine(6, "startPowC: %d", startPowC);
        // displayCenteredTextLine(7, "endPowB: %d", endPowB);
        // displayCenteredTextLine(8, "endPowC: %d", endPowC);

        moveBC3Parts(incDeg, dist - incDeg - decDeg, decDeg, startPowB, startPowC, maxPowerB, maxPowerC, endPowB, endPowC);
    }
    else
    {
        float maxStartPow = max(fabs(startPowB), fabs(startPowC));
        float maxEndPow = max(fabs(endPowB), fabs(endPowC));
        decDeg = round((dist * 2 * startAccel - pow(maxEndPow, 2) + pow(maxStartPow, 2)) / (2 * startAccel + 2 * endAccel));
        incDeg = dist - decDeg;
        float startAccelB, startAccelC, endAccelB, endAccelC;
        if (fabs(startPowB) > fabs(startPowC))
        {
            if (startPowB > 0)
            {
                startAccelB = startAccel;
                endAccelB = -1 * endAccel;
            }
            else
            {
                startAccelB = -1 * startAccel;
                endAccelB = endAccel;
            }
            if (startPowC > 0)
            {
                startAccelC = startAccel * fabs(startPowC) / fabs(startPowB);
                endAccelC = -1 * endAccel * fabs(endPowC) / fabs(endPowB);
            }
            else
            {
                startAccelC = -1 * startAccel * fabs(startPowC) / fabs(startPowB);
                endAccelC = endAccel * fabs(endPowC) / fabs(endPowB);
            }
        }
        else
        {
            if (startPowC > 0)
            {
                startAccelC = startAccel;
                endAccelC = -1 * endAccel;
            }
            else
            {
                startAccelC = -1 * startAccel;
                endAccelC = endAccel;
            }
            if (startPowB > 0)
            {
                startAccelB = startAccel * fabs(startPowB) / fabs(startPowC);
                endAccelB = -1 * endAccel * fabs(endPowB) / fabs(endPowC);
            }
            else
            {
                startAccelB = -1 * startAccel * fabs(startPowB) / fabs(startPowC);
                endAccelB = endAccel * fabs(endPowB) / fabs(endPowC);
            }
        }

        float maxPowerB, maxPowerC;
        if (startPowB > 0)
        {
            maxPowerB = sqrt(pow(startPowB, 2) + 2 * fabs(startAccelB) * incDeg);
        }
        else
        {
            maxPowerB = -1 * sqrt(pow(startPowB, 2) + 2 * fabs(startAccelB) * incDeg);
        }

        if (startPowC > 0)
        {
            maxPowerC = sqrt(pow(startPowC, 2) + 2 * fabs(startAccelC) * incDeg);
        }
        else
        {
            maxPowerC = -1 * sqrt(pow(startPowC, 2) + 2 * fabs(startAccelC) * incDeg);
        }

        // eraseDisplay();
        // displayCenteredTextLine(0, "incDeg: %d", incDeg);
        // displayCenteredTextLine(1, "decDeg: %d", decDeg);
        // displayCenteredTextLine(2, "startAccelB: %f", startAccelB);
        // displayCenteredTextLine(3, "startAccelC: %f", startAccelC);
        // displayCenteredTextLine(4, "endAccelB: %f", endAccelB);
        // displayCenteredTextLine(5, "endAccelC: %f", endAccelC);
        // displayCenteredTextLine(6, "maxPowerB: %f", maxPowerB);
        // displayCenteredTextLine(7, "maxPowerC: %f", maxPowerC);

        moveBCCustomAccelMainB(incDeg, startPowB, startPowC, startAccelB, startAccelC);
        moveBCCustomAccelMainB(decDeg, maxPowerB, maxPowerC, endAccelB, endAccelC);
    }
}

void moveBCSmartAccelSamePowers(int dist, float powB, float powC, float startAccel = SAFE_START_ACCEL, float endAccel = SAFE_END_ACCEL){
    moveBCSmartAccel(dist, powB, powC, powB, powC, startAccel, endAccel);
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

void turnBDegrSmartAccel(float circleDeg, float startPow, float endPow, float startAccel = SAFE_START_ACCEL, float endAccel = SAFE_END_ACCEL)
{
    float dist = bigCircleDegToCm(circleDeg);
    float deg = cmToDeg(dist);
    moveBCSmartAccel(deg, startPow, 0, endPow, 0, startAccel, endAccel);
}

void turnCDegrSmartAccel(float circleDeg, float startPow, float endPow, float startAccel = SAFE_START_ACCEL, float endAccel = SAFE_END_ACCEL)
{
    float dist = bigCircleDegToCm(circleDeg);
    float deg = cmToDeg(dist);
    moveBCSmartAccel(deg, 0, startPow, 0, endPow, startAccel, endAccel);
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

// + means clockwise
void tankTurnNSSmartAccel(float circleDeg, float startPow, float endPow, float startAccel = SAFE_START_ACCEL, float endAccel = SAFE_END_ACCEL)
{
    float dist = circleDegToCm(circleDeg);
    float deg = cmToDeg(dist);
    moveBCSmartAccel(deg, startPow * -1, startPow * -1, endPow * -1, endPow * -1, startAccel, endAccel);
}
