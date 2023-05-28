#include "PID_c_sensors.c"
#include "motion.c"

// Line forward
// TODO: smart accels
void line2SenDist(PIDSettings *PIDSet, float dist, float pow)
{
    int startDegB = getEncoderB();
    float endDegB;

    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (getEncoderB() > endDegB)
    {
        PID2SensWork(PIDSet, pow);
        sleep(1);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen1InDist(PIDSettings *PIDSet, float dist, float pow, float target)
{
    int startDegB = getEncoderB();
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (getEncoderB() > endDegB)
    {
        PIDSen1InWork(PIDSet, target, pow);
        sleep(1);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen2InDist(PIDSettings *PIDSet, float dist, float pow, float target)
{
    int startDegB = getEncoderB();
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (getEncoderB() > endDegB)
    {
        PIDSen2InWork(PIDSet, target, pow);
        sleep(1);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen1OutDist(PIDSettings *PIDSet, float dist, float pow, float target)
{
    int startDegB = getEncoderB();
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (getEncoderB() > endDegB)
    {
        PIDSen1InWork(PIDSet, target, pow);
        sleep(1);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen2OutDist(PIDSettings *PIDSet, float dist, float pow, float target)
{
    int startDegB = getEncoderB();
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (getEncoderB() > endDegB)
    {
        PIDSen2InWork(PIDSet, target, pow);
        sleep(1);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

bool applyNewAccelsSen(short *powB, short *powC, float *newPowB, float *newPowC)
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
    if (!(applyB) && !(applyC))
    {
        return (!(newBInBorder) && !(bExitingLim)) || (!(newCInBorder) && !(cExitingLim));
    }
    return false;
}

void line2SenCustomAccel(PIDSettings *PIDSet, float dist, float power, float accel)
{
    int startDegB = getEncoderB();

    float newPow, newPowB;
    bool flagAccel = false;
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (getEncoderB() > endDegB)
    {
        if (!(flagAccel))
        {
            newPow = round(sqrt(fabs(getEncoderB() - startDegB) * 2 * accel + pow(power, 2)));
            newPowB = newPow * -1;
            flagAccel = applyNewAccelsSen(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPow);
        }
        PID2SensWork(PIDSet, newPow);
        sleep(1);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

PIDSettings *PIDSetFromPower(int power)
{
    if ((DEFAULT_LINE_PID_SUPRA_MAX_POWER > power) && (DEFAULT_LINE_PID_SUPRA_MIN_POWER) < power)
    {
        return &DEFAULT_LINE_PID_SUPRA;
    }
    else if ((DEFAULT_LINE_PID_FAST_MAX_POWER > power) && (DEFAULT_LINE_PID_FAST_MIN_POWER) < power)
    {
        return &DEFAULT_LINE_PID_FAST;
    }
    else if ((DEFAULT_LINE_PID_MEDIUM_MAX_POWER > power) && (DEFAULT_LINE_PID_MEDIUM_MIN_POWER) < power)
    {
        return &DEFAULT_LINE_PID_MEDIUM;
    }
    else
    {
        return &DEFAULT_LINE_PID_SLOW;
    }
}

void line2SenCustomAccelAutoSet(float dist, float power, float accel)
{
    int startDegB = getEncoderB();

    float newPow, newPowB;
    bool flagAccel = false;
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (getEncoderB() > endDegB)
    {
        if (!(flagAccel))
        {
            newPow = round(sqrt(fabs(getEncoderB() - startDegB) * 2 * accel + pow(power, 2)));
            newPowB = newPow * -1;
            flagAccel = applyNewAccelsSen(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPow);
        }
        PID2SensWork(PIDSetFromPower(newPow), newPow);
        sleep(1);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen1InCustomAccel(PIDSettings *PIDSet, float dist, float power, float accel, float target)
{
    int startDegB = getEncoderB();

    float newPow, newPowB;
    bool flagAccel = false;
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (getEncoderB() > endDegB)
    {
        if (!(flagAccel))
        {
            newPow = round(sqrt(fabs(getEncoderB() - startDegB) * 2 * accel + pow(power, 2)));
            newPowB = newPow * -1;
            flagAccel = applyNewAccelsSen(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPow);
        }
        PIDSen1InWork(PIDSet, target, newPow);
        sleep(1);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen2InCustomAccel(PIDSettings *PIDSet, float dist, float power, float accel, float target, float updateTime = 100)
{
    int startDegB = getEncoderB();

    float newPow, newPowB;
    bool flagAccel = false;
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (getEncoderB() > endDegB)
    {
        if (!(flagAccel))
        {
            newPow = round(sqrt(fabs(getEncoderB() - startDegB) * 2 * accel + pow(power, 2)));
            newPowB = -1 * newPow;
            flagAccel = applyNewAccelsSen(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPow);
        }
        PIDSen2InWork(PIDSet, target, newPow);
        sleep(1);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen1OutCustomAccel(PIDSettings *PIDSet, float dist, float power, float accel, float target, float updateTime = 100)
{
    int startDegB = getEncoderB();

    float newPow, newPowB;
    bool flagAccel = false;
    float endDegB;

    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (getEncoderB() > endDegB)
    {
        if (!(flagAccel))
        {
            newPow = round(sqrt(fabs(getEncoderB() - startDegB) * 2 * accel + pow(power, 2)));
            newPowB = -1 * newPow;
            flagAccel = applyNewAccelsSen(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPow);
        }
        PIDSen1OutWork(PIDSet, target, newPow);
        sleep(1);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen2OutCustomAccel(PIDSettings *PIDSet, float dist, float power, float accel, float target, float updateTime = 100)
{
    int startDegB = getEncoderB();

    float newPow, newPowB;
    bool flagAccel = false;
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (getEncoderB() > endDegB)
    {
        if (!(flagAccel))
        {
            newPow = round(sqrt(fabs(getEncoderB() - startDegB) * 2 * accel + pow(power, 2)));
            newPowB = -1 * newPow;
            flagAccel = applyNewAccelsSen(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPow);
        }
        PIDSen2OutWork(PIDSet, target, newPow);
        sleep(1);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void line2SenAccelPart(PIDSettings *PIDSet, float dist, float startPow, float endPow)
{
    float accel;
    if (endPow > startPow)
    {
        accel = fabs((pow(endPow, 2) - pow(startPow, 2)) / (dist * 2));
    }
    else
    {
        accel = -1 * fabs((pow(endPow, 2) - pow(startPow, 2)) / (dist * 2));
    }
    line2SenCustomAccel(PIDSet, dist, startPow, accel);
}

void line2SenAccelPartAutoSet(float dist, float startPow, float endPow)
{
    float accel;
    if (endPow > startPow)
    {
        accel = fabs((pow(endPow, 2) - pow(startPow, 2)) / (dist * 2));
    }
    else
    {
        accel = -1 * fabs((pow(endPow, 2) - pow(startPow, 2)) / (dist * 2));
    }
    line2SenCustomAccelAutoSet(dist, startPow, accel);
}

void lineSen1InAccelPart(PIDSettings *PIDSet, float dist, float startPow, float endPow, float target)
{
    float accel;
    if (endPow > startPow)
    {
        accel = fabs((pow(endPow, 2) - pow(startPow, 2)) / (dist * 2));
    }
    else
    {
        accel = -1 * fabs((pow(endPow, 2) - pow(startPow, 2)) / (dist * 2));
    }
    lineSen1InCustomAccel(PIDSet, dist, startPow, accel, target);
}

void lineSen1OutAccelPart(PIDSettings *PIDSet, float dist, float startPow, float endPow, float target)
{
    float accel;
    if (endPow > startPow)
    {
        accel = fabs((pow(endPow, 2) - pow(startPow, 2)) / (dist * 2));
    }
    else
    {
        accel = -1 * fabs((pow(endPow, 2) - pow(startPow, 2)) / (dist * 2));
    }
    lineSen1OutCustomAccel(PIDSet, dist, startPow, accel, target);
}

void lineSen2InAccelPart(PIDSettings *PIDSet, float dist, float startPow, float endPow, float target)
{
    float accel;
    if (endPow > startPow)
    {
        accel = fabs((pow(endPow, 2) - pow(startPow, 2)) / (dist * 2));
    }
    else
    {
        accel = -1 * fabs((pow(endPow, 2) - pow(startPow, 2)) / (dist * 2));
    }
    lineSen2InCustomAccel(PIDSet, dist, startPow, accel, target);
}

void lineSen2OutAccelPart(PIDSettings *PIDSet, float dist, float startPow, float endPow, float target)
{
    float accel;
    if (endPow > startPow)
    {
        accel = fabs((pow(endPow, 2) - pow(startPow, 2)) / (dist * 2));
    }
    else
    {
        accel = -1 * fabs((pow(endPow, 2) - pow(startPow, 2)) / (dist * 2));
    }
    lineSen2OutCustomAccel(PIDSet, dist, startPow, accel, target);
}

void line2Sen3Parts(PIDSettings *PIDSet1, PIDSettings *PIDSet2, PIDSettings *PIDSet3, float dist1, float dist2, float dist3, float startPow, float maxPow, float endPow)
{
    line2SenAccelPart(PIDSet1, dist1, startPow, maxPow);
    line2SenDist(PIDSet2, dist2, maxPow);
    line2SenAccelPart(PIDSet3, dist3, maxPow, endPow);
}

void line2Sen3PartsAutoSet(float dist1, float dist2, float dist3, float startPow, float maxPow, float endPow)
{
    line2SenAccelPartAutoSet(dist1, startPow, maxPow);
    line2SenDist(PIDSetFromPower(maxPow), dist2, maxPow);
    line2SenAccelPartAutoSet(dist3, maxPow, endPow);
}

// Adjust movement depending on SAFE_START_ACCEL and SAFE_END_ACCEL
// problem: moveBCSmartAccel(50, -20, 20, -30, 30);
void line2SenSmartAccel(int dist, float startPow, float endPow, float startAccel = SAFE_START_ACCEL, float endAccel = SAFE_END_ACCEL)
{
    // TODO: fix problem with endPow when it is 0
    if (endPow == 0)
    {
        if (startPow > 0)
        {
            endPow = -0.01;
        }
        else
        {
            endPow = 0.01;
        }
    }

    int incDeg, decDeg, fromStartDecDeg;
    float maxPower;
    incDeg = round((pow(MOTORS_MAX_POWER, 2) - pow(startPow, 2)) / (2 * startAccel));
    decDeg = round((pow(MOTORS_MAX_POWER, 2) - pow(endPow, 2)) / (2 * endAccel));
    fromStartDecDeg = round((pow(startPow, 2) - pow(endPow, 2)) / (2 * endAccel));

    if (fromStartDecDeg >= dist)
    {
        playSound(soundException);

        line2SenAccelPartAutoSet(dist, startPow, endPow);
    }
    else if (dist > incDeg + decDeg)
    {
        if (startPow > 0)
        {
            maxPower = MOTORS_MAX_POWER;
        }
        else
        {
            maxPower = -1 * MOTORS_MAX_POWER;
        }

        line2Sen3PartsAutoSet(incDeg, dist - incDeg - decDeg, decDeg, startPow, maxPower, endPow);
    }
    else
    {
        decDeg = round((dist * 2 * startAccel - pow(endPow, 2) + pow(startPow, 2)) / (2 * startAccel + 2 * endAccel));
        incDeg = dist - decDeg;
        float maxPower = sqrt(pow(startPow, 2) + 2 * fabs(startAccel) * incDeg);

        line2SenCustomAccelAutoSet(incDeg, startPow, startAccel);
        line2SenCustomAccelAutoSet(decDeg, maxPower, endAccel * -1);
    }
}

void line2SenCrawl(PIDSettings *PIDSet, float pow, float blackLineSumRGBSen1, float blackLineSumRGBSen2)
{
    MOT_PID_SETTINGS.pauseAction = true;
    while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGBSen1 || readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGBSen2)
    {
        PID2SensWork(PIDSet, pow);
        sleep(1);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

// turns
void tankTurnSenCrawl(float powB1Part, float powC1Part, float powB2Part, float powC2Part, float blackLineSumRGB1, float blackLineSumRGB2)
{
    if (powB1Part < 0)
    {
        setNewMotBCPowersAndRatio(powB1Part, powC1Part);
        while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGB1)
        {
            sleep(1);
        };
        setNewMotBCPowersAndRatio(powB2Part, powC2Part);
        while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGB2)
        {
            sleep(1);
        };
    }
    else
    {
        setNewMotBCPowersAndRatio(powB1Part, powC1Part);
        while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGB1)
        {
            sleep(1);
        };
        setNewMotBCPowersAndRatio(powB2Part, powC2Part);
        while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGB2)
        {
            sleep(1);
        };
    }
}

void turnBSenCrawl(float powB1Part, float powB2Part, float blackLineSumRGB1, float blackLineSumRGB2)
{
    if (powB1Part < 0)
    {
        setNewMotBCPowersAndRatio(powB1Part, 0);
        while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGB1)
        {
            sleep(1);
        };
        setNewMotBCPowersAndRatio(powB2Part, 0);
        while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGB2)
        {
            sleep(1);
        };
    }
    else
    {
        setNewMotBCPowersAndRatio(powB1Part, 0);
        while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGB1)
        {
            sleep(1);
        };
        setNewMotBCPowersAndRatio(powB2Part, 0);
        while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGB2)
        {
            sleep(1);
        };
    }
}

void turnCSenCrawl(float powC1Part, float powC2Part, float blackLineSumRGB1, float blackLineSumRGB2)
{
    if (powC1Part > 0)
    {
        setNewMotBCPowersAndRatio(0, powC1Part);
        while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGB1)
        {
            sleep(1);
        };
        setNewMotBCPowersAndRatio(0, powC2Part);
        while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGB2)
        {
            sleep(1);
        };
    }
    else
    {
        setNewMotBCPowersAndRatio(0, powC1Part);
        while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGB1)
        {
            sleep(1);
        };
        setNewMotBCPowersAndRatio(0, powC2Part);
        while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGB2)
        {
            sleep(1);
        };
    }
}

void correctWithLine(float powB1, float powC1, float powB2, float powC2, float blackLineRGB1Sen1, float blackLineRGB1Sen2, float blackLineRGB2Sen1, float blackLineRGB2Sen2)
{
    bool flagSen1First = false, flagSen2First = false;
    setNewMotBCPowersAndRatio(powB1, powC1);
    while (!(flagSen1First) && !(flagSen2First))
    {
        if (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) <= blackLineRGB1Sen1)
        {
            flagSen1First = true;
        }
        else if (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) <= blackLineRGB1Sen2)
        {
            flagSen2First = true;
        }
        sleep(1);
    }
    if (flagSen1First)
    {
        setNewMotBCPowersAndRatio(0, powC2);
        while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineRGB2Sen2)
        {
            sleep(1);
        };
    }
    else if (flagSen2First)
    {
        setNewMotBCPowersAndRatio(powB2, 0);
        while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineRGB2Sen1)
        {
            sleep(1);
        };
    }
}

void waitFor2Sen(float blackLineSumRGBSen1 = 400, float blackLineSumRGBSen2 = 400)
{
    while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGBSen1 || readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGBSen2)
    {
        sleep(1);
    };
}

void waitForSen1(float blackLineSumRGB)
{
    while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGB)
    {
        sleep(1);
    };
}

void waitForSen2(float blackLineSumRGB)
{
    while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGB)
    {
        sleep(1);
    };
}