#include "PID_c_sensors.c"
#include "motion.c"

// Line forward
void line2SenDist(PIDSettings *PIDSet, float dist, float pow)
{
    int startDegB = nMotorEncoder[motB];
    float endDegB;
    if (pow > 0)
    {
        endDegB = startDegB + cmToDeg(dist);
        while (nMotorEncoder[motB] < endDegB)
        {
            PID2SensWork(PIDSet, pow);
        }
    }
    else
    {
        endDegB = startDegB - cmToDeg(dist);
        while (nMotorEncoder[motB] > endDegB)
        {
            PID2SensWork(PIDSet, pow);
        }
    }
}

void lineSen1InDist(PIDSettings *PIDSet, float dist, float pow, float target)
{
    int startDegB = nMotorEncoder[motB];
    float endDegB;
    if (pow > 0)
    {
        endDegB = startDegB + cmToDeg(dist);
        while (nMotorEncoder[motB] < endDegB)
        {
            PIDSen1InWork(PIDSet, target, pow);
        }
    }
    else
    {
        endDegB = startDegB - cmToDeg(dist);
        while (nMotorEncoder[motB] > endDegB)
        {
            PIDSen1InWork(PIDSet, target, pow);
        }
    }
}

void lineSen2InDist(PIDSettings *PIDSet, float dist, float pow, float target)
{
    int startDegB = nMotorEncoder[motB];
    float endDegB;
    if (pow > 0)
    {
        endDegB = startDegB + cmToDeg(dist);
        while (nMotorEncoder[motB] < endDegB)
        {
            PIDSen2InWork(PIDSet, target, pow);
        }
    }
    else
    {
        endDegB = startDegB - cmToDeg(dist);
        while (nMotorEncoder[motB] > endDegB)
        {
            PIDSen2InWork(PIDSet, target, pow);
        }
    }
}

void lineSen1OutDist(PIDSettings *PIDSet, float dist, float pow, float target)
{
    int startDegB = nMotorEncoder[motB];
    float endDegB;
    if (pow > 0)
    {
        endDegB = startDegB + cmToDeg(dist);
        while (nMotorEncoder[motB] < endDegB)
        {
            PIDSen1InWork(PIDSet, target, pow);
        }
    }
    else
    {
        endDegB = startDegB - cmToDeg(dist);
        while (nMotorEncoder[motB] > endDegB)
        {
            PIDSen1InWork(PIDSet, target, pow);
        }
    }
}

void lineSen2OutDist(PIDSettings *PIDSet, float dist, float pow, float target)
{
    int startDegB = nMotorEncoder[motB];
    float endDegB;
    if (pow > 0)
    {
        endDegB = startDegB + cmToDeg(dist);
        while (nMotorEncoder[motB] < endDegB)
        {
            PIDSen2InWork(PIDSet, target, pow);
        }
    }
    else
    {
        endDegB = startDegB - cmToDeg(dist);
        while (nMotorEncoder[motB] > endDegB)
        {
            PIDSen2InWork(PIDSet, target, pow);
        }
    }
}

void line2SenCustomAccel(PIDSettings *PIDSet, float dist, float power, float accel, float updateTime = 100)
{
    tryRepairDefect(POWER_MOT_B, POWER_MOT_C);
    int startDegB = nMotorEncoder[motB];
    int madeDist;

    float newPowB;
    float newPowC;
    float startTime = nPgmTime;
    bool flagAccel = false;
    float endDegB;
    if (power >= 0)
    {
        endDegB = startDegB + cmToDeg(dist);
        while (nMotorEncoder[motB] < endDegB)
        {
            if (!(flagAccel) && (nPgmTime - startTime) % 100 == 0)
            {
                madeDist = fabs(nMotorEncoder[motB] - startDegB);
                newPowB = round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                if (power >= 0)
                {
                    newPowC = round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                else
                {
                    newPowC = -1 * round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
            }
            PID2SensWork(PIDSet, power);
        }
    }
    else
    {
        endDegB = startDegB - cmToDeg(dist);
        while (nMotorEncoder[motB] > endDegB)
        {
            if (!(flagAccel) && (nPgmTime - startTime) % 100 == 0)
            {
                madeDist = fabs(nMotorEncoder[motB] - startDegB);
                newPowB = -1 * round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                if (power >= 0)
                {
                    newPowC = round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                else
                {
                    newPowC = -1 * round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
            }
            PID2SensWork(PIDSet, power);
        }
    }
}

void lineSen1InCustomAccel(PIDSettings *PIDSet, float dist, float power, float accel, float target, float updateTime = 100)
{
    tryRepairDefect(POWER_MOT_B, POWER_MOT_C);
    int startDegB = nMotorEncoder[motB];
    int madeDist;

    float newPowB;
    float newPowC;
    float startTime = nPgmTime;
    bool flagAccel = false;
    float endDegB;
    if (power >= 0)
    {
        endDegB = startDegB + cmToDeg(dist);
        while (nMotorEncoder[motB] < endDegB)
        {
            if (!(flagAccel) && (nPgmTime - startTime) % 100 == 0)
            {
                madeDist = fabs(nMotorEncoder[motB] - startDegB);
                newPowB = round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                if (power >= 0)
                {
                    newPowC = round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                else
                {
                    newPowC = -1 * round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
            }
            PIDSen1InWork(PIDSet, target, power);
        }
    }
    else
    {
        endDegB = startDegB - cmToDeg(dist);
        while (nMotorEncoder[motB] > endDegB)
        {
            if (!(flagAccel) && (nPgmTime - startTime) % 100 == 0)
            {
                madeDist = fabs(nMotorEncoder[motB] - startDegB);
                newPowB = -1 * round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                if (power >= 0)
                {
                    newPowC = round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                else
                {
                    newPowC = -1 * round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
            }
            PIDSen1InWork(PIDSet, target, power);
        }
    }
}

void lineSen2InCustomAccel(PIDSettings *PIDSet, float dist, float power, float accel, float target, float updateTime = 100)
{
    tryRepairDefect(POWER_MOT_B, POWER_MOT_C);
    int startDegB = nMotorEncoder[motB];
    int madeDist;

    float newPowB;
    float newPowC;
    float startTime = nPgmTime;
    bool flagAccel = false;
    float endDegB;
    if (power >= 0)
    {
        endDegB = startDegB + cmToDeg(dist);
        while (nMotorEncoder[motB] < endDegB)
        {
            if (!(flagAccel) && (nPgmTime - startTime) % 100 == 0)
            {
                madeDist = fabs(nMotorEncoder[motB] - startDegB);
                newPowB = round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                if (power >= 0)
                {
                    newPowC = round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                else
                {
                    newPowC = -1 * round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
            }
            PIDSen2InWork(PIDSet, target, power);
        }
    }
    else
    {
        endDegB = startDegB - cmToDeg(dist);
        while (nMotorEncoder[motB] > endDegB)
        {
            if (!(flagAccel) && (nPgmTime - startTime) % 100 == 0)
            {
                madeDist = fabs(nMotorEncoder[motB] - startDegB);
                newPowB = -1 * round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                if (power >= 0)
                {
                    newPowC = round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                else
                {
                    newPowC = -1 * round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
            }
            PIDSen2InWork(PIDSet, target, power);
        }
    }
}

void lineSen1OutCustomAccel(PIDSettings *PIDSet, float dist, float power, float accel, float target, float updateTime = 100)
{
    tryRepairDefect(POWER_MOT_B, POWER_MOT_C);
    int startDegB = nMotorEncoder[motB];
    int madeDist;

    float newPowB;
    float newPowC;
    float startTime = nPgmTime;
    bool flagAccel = false;
    float endDegB;
    if (power >= 0)
    {
        endDegB = startDegB + cmToDeg(dist);
        while (nMotorEncoder[motB] < endDegB)
        {
            if (!(flagAccel) && (nPgmTime - startTime) % 100 == 0)
            {
                madeDist = fabs(nMotorEncoder[motB] - startDegB);
                newPowB = round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                if (power >= 0)
                {
                    newPowC = round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                else
                {
                    newPowC = -1 * round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
            }
            PIDSen1OutWork(PIDSet, target, power);
        }
    }
    else
    {
        endDegB = startDegB - cmToDeg(dist);
        while (nMotorEncoder[motB] > endDegB)
        {
            if (!(flagAccel) && (nPgmTime - startTime) % 100 == 0)
            {
                madeDist = fabs(nMotorEncoder[motB] - startDegB);
                newPowB = -1 * round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                if (power >= 0)
                {
                    newPowC = round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                else
                {
                    newPowC = -1 * round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
            }
            PIDSen1OutWork(PIDSet, target, power);
        }
    }
}

void lineSen2OutCustomAccel(PIDSettings *PIDSet, float dist, float power, float accel, float target, float updateTime = 100)
{
    tryRepairDefect(POWER_MOT_B, POWER_MOT_C);
    int startDegB = nMotorEncoder[motB];
    int madeDist;

    float newPowB;
    float newPowC;
    float startTime = nPgmTime;
    bool flagAccel = false;
    float endDegB;
    if (power >= 0)
    {
        endDegB = startDegB + cmToDeg(dist);
        while (nMotorEncoder[motB] < endDegB)
        {
            if (!(flagAccel) && (nPgmTime - startTime) % 100 == 0)
            {
                madeDist = fabs(nMotorEncoder[motB] - startDegB);
                newPowB = round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                if (power >= 0)
                {
                    newPowC = round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                else
                {
                    newPowC = -1 * round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
            }
            PIDSen2OutWork(PIDSet, target, power);
        }
    }
    else
    {
        endDegB = startDegB - cmToDeg(dist);
        while (nMotorEncoder[motB] > endDegB)
        {
            if (!(flagAccel) && (nPgmTime - startTime) % 100 == 0)
            {
                madeDist = fabs(nMotorEncoder[motB] - startDegB);
                newPowB = -1 * round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                if (power >= 0)
                {
                    newPowC = round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                else
                {
                    newPowC = -1 * round(sqrt(madeDist * 2 * accel + pow(power, 2)));
                }
                flagAccel = applyNewAccels(&POWER_MOT_B, &POWER_MOT_C, &newPowB, &newPowC);
            }
            PIDSen2OutWork(PIDSet, target, power);
        }
    }
}

void line2SenAccelPart(PIDSettings *PIDSet, float dist, float startPow, float endPow)
{
    float accel = (pow(endPow, 2) - pow(startPow, 2)) / (cmToDeg(dist) * 2);
    line2SenCustomAccel(PIDSet, dist, startPow, accel);
}

void lineSen1InAccelPart(PIDSettings *PIDSet, float dist, float startPow, float endPow, float target)
{
    float accel = (pow(endPow, 2) - pow(startPow, 2)) / (cmToDeg(dist) * 2);
    lineSen1InCustomAccel(PIDSet, dist, startPow, accel, target);
}

void lineSen1OutAccelPart(PIDSettings *PIDSet, float dist, float startPow, float endPow, float target)
{
    float accel = (pow(endPow, 2) - pow(startPow, 2)) / (cmToDeg(dist) * 2);
    lineSen1OutCustomAccel(PIDSet, dist, startPow, accel, target);
}

void lineSen2InAccelPart(PIDSettings *PIDSet, float dist, float startPow, float endPow, float target)
{
    float accel = (pow(endPow, 2) - pow(startPow, 2)) / (cmToDeg(dist) * 2);
    lineSen2InCustomAccel(PIDSet, dist, startPow, accel, target);
}

void lineSen2OutAccelPart(PIDSettings *PIDSet, float dist, float startPow, float endPow, float target)
{
    float accel = (pow(endPow, 2) - pow(startPow, 2)) / (cmToDeg(dist) * 2);
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
    while (readCalibratedSenSumRGB(sen1, SEN1_CALIBRATION) > blackLineSumRGBSen1 && readCalibratedSenSumRGB(sen2, SEN2_CALIBRATION) > blackLineSumRGBSen2)
    {
        PID2SensWork(PIDSet, pow);
    }
}

// turns
void tankTurnSenCrawl(float pow1, float pow2, float blackLineSumRGB1, float blackLineSumRGB2)
{
    if (pow1 > 0)
    {
        setNewMotBCPowersAndRatio(pow1, pow1 * -1);
        while (readCalibratedSenSumRGB(sen2, SEN2_CALIBRATION) > blackLineSumRGB1)
            ;
        setNewMotBCPowersAndRatio(pow2, pow2 * -1);
        while (readCalibratedSenSumRGB(sen1, SEN1_CALIBRATION) > blackLineSumRGB2)
            ;
    }
    else
    {
        setNewMotBCPowersAndRatio(pow1, pow1 * -1);
        while (readCalibratedSenSumRGB(sen1, SEN1_CALIBRATION) > blackLineSumRGB1)
            ;
        setNewMotBCPowersAndRatio(pow2, pow2 * -1);
        while (readCalibratedSenSumRGB(sen2, SEN2_CALIBRATION) > blackLineSumRGB2)
            ;
    }
}

void turnBSenCrawl(float pow1, float pow2, float blackLineSumRGB1, float blackLineSumRGB2)
{
    if (pow1 > 0)
    {
        setNewMotBCPowersAndRatio(pow1, 0);
        while (readCalibratedSenSumRGB(sen2, SEN2_CALIBRATION) > blackLineSumRGB1)
            ;
        setNewMotBCPowersAndRatio(pow2, 0);
        while (readCalibratedSenSumRGB(sen1, SEN1_CALIBRATION) > blackLineSumRGB2)
            ;
    }
    else
    {
        setNewMotBCPowersAndRatio(pow1, 0);
        while (readCalibratedSenSumRGB(sen1, SEN1_CALIBRATION) > blackLineSumRGB1)
            ;
        setNewMotBCPowersAndRatio(pow2, 0);
        while (readCalibratedSenSumRGB(sen2, SEN2_CALIBRATION) > blackLineSumRGB2)
            ;
    }
}

// CLOCKWISE!!!!!!!!!!!!!!!!!!
void turnCSenCraw(float pow1, float pow2, float blackLineSumRGB1, float blackLineSumRGB2)
{
    if (pow1 < 0)
    {
        setNewMotBCPowersAndRatio(0, pow1 * -1);
        while (readCalibratedSenSumRGB(sen2, SEN2_CALIBRATION) > blackLineSumRGB1)
            ;
        setNewMotBCPowersAndRatio(0, pow2 * -1);
        while (readCalibratedSenSumRGB(sen1, SEN1_CALIBRATION) > blackLineSumRGB2)
            ;
    }
    else
    {
        setNewMotBCPowersAndRatio(0, pow1 * -1);
        while (readCalibratedSenSumRGB(sen1, SEN1_CALIBRATION) > blackLineSumRGB1)
            ;
        setNewMotBCPowersAndRatio(0, pow2 * -1);
        while (readCalibratedSenSumRGB(sen2, SEN2_CALIBRATION) > blackLineSumRGB2)
            ;
    }
}
