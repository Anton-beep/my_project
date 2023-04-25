#include "PID_c_sensors.c"
#include "motion.c"

// Line forward
void line2SenDist(PIDSettings *PIDSet, float dist, float pow)
{
    int startDegB = nMotorEncoder[motB];
    float endDegB;

    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (nMotorEncoder[motB] > endDegB)
    {
        PID2SensWork(PIDSet, pow);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen1InDist(PIDSettings *PIDSet, float dist, float pow, float target)
{
    int startDegB = nMotorEncoder[motB];
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (nMotorEncoder[motB] > endDegB)
    {
        PIDSen1InWork(PIDSet, target, pow);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen2InDist(PIDSettings *PIDSet, float dist, float pow, float target)
{
    int startDegB = nMotorEncoder[motB];
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (nMotorEncoder[motB] > endDegB)
    {
        PIDSen2InWork(PIDSet, target, pow);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen1OutDist(PIDSettings *PIDSet, float dist, float pow, float target)
{
    int startDegB = nMotorEncoder[motB];
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (nMotorEncoder[motB] > endDegB)
    {
        PIDSen1InWork(PIDSet, target, pow);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen2OutDist(PIDSettings *PIDSet, float dist, float pow, float target)
{
    int startDegB = nMotorEncoder[motB];
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (nMotorEncoder[motB] > endDegB)
    {
        PIDSen2InWork(PIDSet, target, pow);
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
    int startDegB = nMotorEncoder[motB];
    int madeDist;

    float newPow, newPowB;
    float startTime = nPgmTime;
    bool flagAccel = false;
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (nMotorEncoder[motB] > endDegB)
    {
        if (!(flagAccel))
        {
            newPow = round(sqrt(fabs(nMotorEncoder[motB] - startDegB) * 2 * accel + pow(power, 2)));
            newPowB = newPow * -1;
            flagAccel = applyNewAccelsSen(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPow);
            startTime = nPgmTime;
        }
        PID2SensWork(PIDSet, newPow);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen1InCustomAccel(PIDSettings *PIDSet, float dist, float power, float accel, float target, float updateTime = 0)
{
    int startDegB = nMotorEncoder[motB];
    int madeDist;

    float newPow, newPowB;
    float startTime = nPgmTime;
    bool flagAccel = false;
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (nMotorEncoder[motB] > endDegB)
    {
        if (!(flagAccel) && (nPgmTime - startTime) > updateTime)
        {
            newPow = round(sqrt(fabs(nMotorEncoder[motB] - startDegB) * 2 * accel + pow(power, 2)));
            newPowB = newPow * -1;
            flagAccel = applyNewAccelsSen(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPow);
            startTime = nPgmTime;
        }
        PIDSen1InWork(PIDSet, target, newPow);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen2InCustomAccel(PIDSettings *PIDSet, float dist, float power, float accel, float target, float updateTime = 100)
{
    int startDegB = nMotorEncoder[motB];
    int madeDist;

    float newPow, newPowB;
    float startTime = nPgmTime;
    bool flagAccel = false;
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (nMotorEncoder[motB] > endDegB)
    {
        if (!(flagAccel) && (nPgmTime - startTime) > updateTime)
        {
            newPow = round(sqrt(fabs(nMotorEncoder[motB] - startDegB) * 2 * accel + pow(power, 2)));
            newPowB = -1 * newPow;
            flagAccel = applyNewAccelsSen(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPow);
            startTime = nPgmTime;
        }
        PIDSen2InWork(PIDSet, target, newPow);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen1OutCustomAccel(PIDSettings *PIDSet, float dist, float power, float accel, float target, float updateTime = 100)
{
    int startDegB = nMotorEncoder[motB];
    int madeDist;

    float newPow, newPowB;
    float startTime = nPgmTime;
    bool flagAccel = false;
    float endDegB;

    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (nMotorEncoder[motB] > endDegB)
    {
        if (!(flagAccel) && (nPgmTime - startTime) > updateTime)
        {
            newPow = round(sqrt(fabs(nMotorEncoder[motB] - startDegB) * 2 * accel + pow(power, 2)));
            newPowB = -1 * newPow;
            flagAccel = applyNewAccelsSen(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPow);
            startTime = nPgmTime;
        }
        PIDSen1OutWork(PIDSet, target, newPow);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void lineSen2OutCustomAccel(PIDSettings *PIDSet, float dist, float power, float accel, float target, float updateTime = 100)
{
    int startDegB = nMotorEncoder[motB];
    int madeDist;

    float newPow, newPowB;
    float startTime = nPgmTime;
    bool flagAccel = false;
    float endDegB;
    endDegB = startDegB - dist;
    MOT_PID_SETTINGS.pauseAction = true;
    while (nMotorEncoder[motB] > endDegB)
    {
        if (!(flagAccel) && (nPgmTime - startTime) > updateTime)
        {
            newPow = round(sqrt(fabs(nMotorEncoder[motB] - startDegB) * 2 * accel + pow(power, 2)));
            newPowB = -1 * newPow;
            flagAccel = applyNewAccelsSen(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPow);
            startTime = nPgmTime;
        }
        PIDSen2OutWork(PIDSet, target, newPow);
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

void line2Sen3PartsOneSet(PIDSettings *PIDSet, float dist1, float dist2, float dist3, float startPow, float maxPow, float endPow)
{
    line2Sen3Parts(PIDSet, PIDSet, PIDSet, dist1, dist2, dist3, startPow, maxPow, endPow);
}

void line2SenCrawl(PIDSettings *PIDSet, float pow, float blackLineSumRGBSen1, float blackLineSumRGBSen2)
{
    MOT_PID_SETTINGS.pauseAction = true;
    while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGBSen1 || readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGBSen2)
    {
        PID2SensWork(PIDSet, pow);
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
            ;
        setNewMotBCPowersAndRatio(powB2Part, powC2Part);
        while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGB2)
            ;
    }
    else
    {
        setNewMotBCPowersAndRatio(powB1Part, powC1Part);
        while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGB1)
            ;
        setNewMotBCPowersAndRatio(powB2Part, powC2Part);
        while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGB2)
            ;
    }
}

void turnBSenCrawl(float powB1Part, float powB2Part, float blackLineSumRGB1, float blackLineSumRGB2)
{
    if (powB1Part < 0)
    {
        setNewMotBCPowersAndRatio(powB1Part, 0);
        while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGB1)
            ;
        setNewMotBCPowersAndRatio(powB2Part, 0);
        while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGB2)
            ;
    }
    else
    {
        setNewMotBCPowersAndRatio(powB1Part, 0);
        while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGB1)
            ;
        setNewMotBCPowersAndRatio(powB2Part, 0);
        while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGB2)
            ;
    }
}

void turnCSenCrawl(float powC1Part, float powC2Part, float blackLineSumRGB1, float blackLineSumRGB2)
{
    if (powC1Part > 0)
    {
        setNewMotBCPowersAndRatio(0, powC1Part);
        while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGB1)
            ;
        setNewMotBCPowersAndRatio(0, powC2Part);
        while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGB2)
            ;
    }
    else
    {
        setNewMotBCPowersAndRatio(0, powC1Part);
        while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGB1)
            ;
        setNewMotBCPowersAndRatio(0, powC2Part);
        while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGB2)
            ;
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
    }
    if (flagSen1First)
    {
        setNewMotBCPowersAndRatio(0, powC2);
        while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineRGB2Sen2)
            ;
    }
    else if (flagSen2First)
    {
        setNewMotBCPowersAndRatio(powB2, 0);
        while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineRGB2Sen1)
            ;
    }
}

void waitForLine(float blackLineSumRGBSen1, float blackLineSumRGBSen2)
{
    while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGBSen1 || readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGBSen2)
        ;
}

void waitForSen1(float blackLineSumRGB)
{
    while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) > blackLineSumRGB)
        ;
}

void waitForSen2(float blackLineSumRGB)
{
    while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) > blackLineSumRGB)
        ;
}