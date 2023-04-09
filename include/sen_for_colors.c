#include "sen_calibration.c"
#include "tools.c"

float distHSVVals(int ptr1, int ptr2)
{
    SenHSVVals *vals1 = (SenHSVVals *)ptr1;
    SenHSVVals *vals2 = (SenHSVVals *)ptr2;
    return sqrt(pow(vals1->H - vals2->H, 2) + pow(vals1->S - vals2->S, 2) + pow(vals1->V - vals2->V, 2));
}

float distRGBVals(int ptr1, int ptr2)
{
    SenRGBVals *vals1 = (SenRGBVals *)ptr1;
    SenRGBVals *vals2 = (SenRGBVals *)ptr2;
    return sqrt(pow(vals1->R - vals2->R, 2) + pow(vals1->G - vals2->G, 2) + pow(vals1->B - vals2->B, 2));
}

// vals is array with pointers to SenHSVVals
// I recommend not to touch this, because operations with pointers like int are not "safe" :)
SenHSVVals *getNearestFromValsHSV(SenHSVVals *inVal, int *samples, int len)
{
    int minVal = samples[0];
    float minDist = distHSVVals((int)inVal, samples[0]);
    float buf;
    for (int i = 1; i < len; i++)
    {
        buf = distHSVVals((int)inVal, samples[i]);
        if (buf < minDist)
        {
            minDist = buf;
            minVal = samples[i];
        }
    }
    return (SenHSVVals *)minVal;
}

SenHSVVals *getNearestFromValsRGB(SenRGBVals *inVal, int *samples, int len)
{
    int minVal = samples[0];
    float minDist = distRGBVals((int)inVal, samples[0]);
    float buf;
    for (int i = 1; i < len; i++)
    {
        buf = distRGBVals((int)inVal, samples[i]);
        if (buf < minDist)
        {
            minDist = buf;
            minVal = samples[i];
        }
    }
    return (SenRGBVals *)minVal;
}

int mode(int *a, int n)
{
    int maxValue = 0, maxCount = 0, i, j;

    for (i = 0; i < n; ++i)
    {
        int count = 0;

        for (j = 0; j < n; ++j)
        {
            if (a[j] == a[i])
                ++count;
        }

        if (count > maxCount)
        {
            maxCount = count;
            maxValue = a[i];
        }
    }

    return maxValue;
}

const int WIN_ARR_LEN = 10;
int WIN_ARR[WIN_ARR_LEN];

SenHSVVals *readWindowHSV(short sen, SenSettings *senSet, int *samples, int lenArr)
{
    SenHSVVals buf;
    for (int i = 0; i < WIN_ARR_LEN; i++)
    {
        readCalibratedSenHSV(sen, senSet, &buf);
        WIN_ARR[i] = (int)getNearestFromValsHSV(&buf, samples, lenArr);
    }
    SenHSVVals *res = (SenHSVVals *)mode(WIN_ARR, WIN_ARR_LEN);
    return res;
}

SenRGBVals *readWindowRGB(short sen, SenSettings *senSet, int *samples, int lenArr)
{
    SenRGBVals buf;
    for (int i = 0; i < WIN_ARR_LEN; i++)
    {
        readCalibratedSenRGB(sen, senSet, &buf);
        // displayTextLine(4 + i, "%d %d %d", buf.R, buf.G, buf.B);
        WIN_ARR[i] = (int)getNearestFromValsRGB(&buf, samples, lenArr);
    }
    SenRGBVals *res = (SenRGBVals *)mode(WIN_ARR, WIN_ARR_LEN);
    return res;
}

const short READ_ROW_MAX_OBJECTS = 20;
int READ_ROW_OF_OBJECTS_RES[READ_ROW_MAX_OBJECTS];
int READ_ROW_OF_OBJECTS_IN_POINTERS[READ_ROW_MAX_OBJECTS];
short READ_ROW_OF_OBJECTS_LEN_IN_POINTERS = 0;
short READ_ROW_OF_OBJECTS_SIZE_OF_WINDOW = 5;
short READ_ROW_OF_OBJECTS_RES_LEN = 0;
short READ_ROW_OF_OBJECTS_SEN = sen3;
SenSettings *READ_ROW_OF_OBJECTS_SEN_CALIBRATION_PTR = &SEN3_CALIBRATION;

// you can moving and start this task to "scan" row with color sensor
// task changes index of the res array when last el and now el are different

task readRowOfObjectsHSV()
{
    short nowInd = 0;
    while (nowInd < READ_ROW_MAX_OBJECTS)
    {
        SenHSVVals *nowPtr = readWindowHSV(READ_ROW_OF_OBJECTS_SEN, READ_ROW_OF_OBJECTS_SEN_CALIBRATION_PTR, READ_ROW_OF_OBJECTS_IN_POINTERS, READ_ROW_OF_OBJECTS_LEN_IN_POINTERS);
        if (nowInd == 0)
        {
            READ_ROW_OF_OBJECTS_RES[nowInd] = nowPtr;
            nowInd++;
        }
        else
        {
            if (READ_ROW_OF_OBJECTS_RES[nowInd - 1] != nowPtr)
            {
                READ_ROW_OF_OBJECTS_RES[nowInd] = nowPtr;
                nowInd++;
            }
        }
    }
}

task readRowOfObjectsRGB()
{
    short nowInd = 0;
    while (nowInd < READ_ROW_MAX_OBJECTS)
    {
        SenRGBVals *nowPtr = readWindowRGB(READ_ROW_OF_OBJECTS_SEN, READ_ROW_OF_OBJECTS_SEN_CALIBRATION_PTR, READ_ROW_OF_OBJECTS_IN_POINTERS, READ_ROW_OF_OBJECTS_LEN_IN_POINTERS);
        if (nowInd == 0)
        {
            READ_ROW_OF_OBJECTS_RES[nowInd] = nowPtr;
            nowInd++;
        }
        else
        {
            if (READ_ROW_OF_OBJECTS_RES[nowInd - 1] != nowPtr)
            {
                READ_ROW_OF_OBJECTS_RES[nowInd] = nowPtr;
                nowInd++;
            }
        }
    }
}

void debReadAndShowHSV(short sen, SenSettings *senSet)
{
    SenHSVVals buf;
    while (true)
    {
        readCalibratedSenHSV(sen, senSet, &buf);
        eraseDisplay();
        displayCenteredBigTextLine(2, "H: %d", buf.H);
        displayCenteredBigTextLine(4, "S: %d", buf.S);
        displayCenteredBigTextLine(6, "V: %d", buf.V);
    }
}

void debReadAndShowRGB(short sen, SenSettings *senSet)
{
    SenRGBVals buf;
    while (true)
    {
        readCalibratedSenRGB(sen, senSet, &buf);
        eraseDisplay();
        displayCenteredBigTextLine(2, "R: %d", buf.R);
        displayCenteredBigTextLine(4, "G: %d", buf.G);
        displayCenteredBigTextLine(6, "B: %d", buf.B);
    }
}
