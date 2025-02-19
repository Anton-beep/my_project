#include "sen_calibration.c"
#include "tools.c"

/*
TODO:
- добавить разные возможности для считывания, например фиксировать цвет при прохождении мотором 
определенного количества градусов или значение на другом датчике
- тест нейронки
- разработка алгоритма для фиксации цвета только с помощью калибровки на сорев. Нужно, чтобы
без каких-либо проблем можно было считывать цвета на любом расстоянии уже на соревнованиях,
при этом без какой-либо дополнительной калибровки цветов, только основная калибровка всех датчиков
(макимум и минимум)
- можно продолжить поиск лучших алгоритмов фильтрации/обработки
*/

float distHSVVals(int ptr1, int ptr2)
{
    SenHSVVals *vals1 = (SenHSVVals *)ptr1;
    SenHSVVals *vals2 = (SenHSVVals *)ptr2;

    return sqrt(pow(vals1->H - vals2->H, 2) + pow(vals1->S - vals2->S, 2));
    // return sqrt(pow(vals1->H - vals2->H, 2) + pow(vals1->S - vals2->S, 2) + pow(vals1->V - vals2->V, 2));
}

float distRGBVals(int ptr1, int ptr2)
{
    SenRGBVals *vals1 = (SenRGBVals *)ptr1;
    SenRGBVals *vals2 = (SenRGBVals *)ptr2;
    return sqrt(pow(vals1->R - vals2->R, 2) + pow(vals1->G - vals2->G, 2) + pow(vals1->B - vals2->B, 2));
}

// vals is array with pointers to SenHSVVals
// I recommend not to touch this, because operations with pointers like int are not "safe" :)
int getNearestFromValsHSV(int inVal, int *samples, int len)
{
    int minVal = samples[0];
    float minDist = distHSVVals(inVal, samples[0]);
    float buf;
    for (int i = 1; i < len; i++)
    {
        buf = distHSVVals(inVal, samples[i]);
        if (buf < minDist)
        {
            minDist = buf;
            minVal = samples[i];
        }
    }
    return minVal;
}

int getNearestFromValsRGB(int inVal, int *samples, int len)
{

    int minVal = samples[0];
    float minDist = distRGBVals(inVal, samples[0]);
    float buf;
    for (int i = 1; i < len; i++)
    {
        buf = distRGBVals(inVal, samples[i]);
        if (buf < minDist)
        {
            minDist = buf;
            minVal = samples[i];
        }
    }
    return minVal;
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

int mean(int *a, int n)
{
    int sum = 0;
    for (int i = 0; i < n; i++)
    {
        sum += a[i];
    }
    return sum / n;
}

const int WIN_ARR_LEN = 6;
int WIN_ARR[WIN_ARR_LEN];

int readWindowHSV(short sen, SenSettings *senSet, SenHSVVals *nothingPtr, int nothingMaxVal, int *samples, int lenArr)
{
    SenHSVVals buf;
    for (int i = 0; i < WIN_ARR_LEN; i++)
    {
        readCalibratedSenHSV(sen, senSet, &buf);
        if (buf.V < nothingMaxVal)
        {
            WIN_ARR[i] = (int)nothingPtr;
        }
        else
        {
            WIN_ARR[i] = (int)getNearestFromValsHSV((int)&buf, samples, lenArr);
        }
    }
    int res = mode(WIN_ARR, WIN_ARR_LEN);
    return res;
}

int readWindowRGB(short sen, SenSettings *senSet, int *samples, int lenArr)
{
    SenRGBVals buf;
    for (int i = 0; i < WIN_ARR_LEN; i++)
    {
        readCalibratedSenRGB(sen, senSet, &buf);
        // displayTextLine(4 + i, "%d %d %d", buf.R, buf.G, buf.B);
        WIN_ARR[i] = getNearestFromValsRGB((int)&buf, samples, lenArr);
    }

    int res = mode(WIN_ARR, WIN_ARR_LEN);
    return res;
}

const short READ_ROW_MAX_OBJECTS = 20;
int READ_ROW_OF_OBJECTS_RES[READ_ROW_MAX_OBJECTS];
int READ_ROW_OF_OBJECTS_IN_POINTERS[READ_ROW_MAX_OBJECTS];
short READ_ROW_OF_OBJECTS_LEN_IN_POINTERS = 0;
short READ_ROW_OF_OBJECTS_RES_LEN = 0;
short READ_ROW_OF_OBJECTS_SEN = sen3;
SenHSVVals *NOTHING_PTR;
int NOTHING_MAX_VAL;
SenSettings *READ_ROW_OF_OBJECTS_SEN_CALIBRATION_PTR;
bool *READ_ROW_OF_OBJECTS_PTR_WORK_FLAG;

//TODO: улучшить фунции ниже: добавить функционал считывания через определенное количество градусов или при каких-либо других условиях
// you can moving and start this task to "scan" row with color sensor
// task changes index of the res array when last el and now el are different
// do not start it using startTask
task readRowOfObjectsHSV()
{
    short nowInd = 0;
    while (nowInd < READ_ROW_MAX_OBJECTS)
    {
        int nowPtr = readWindowHSV(READ_ROW_OF_OBJECTS_SEN, READ_ROW_OF_OBJECTS_SEN_CALIBRATION_PTR, NOTHING_PTR, NOTHING_MAX_VAL, READ_ROW_OF_OBJECTS_IN_POINTERS, READ_ROW_OF_OBJECTS_LEN_IN_POINTERS);
        if (nowInd == 0)
        {
            READ_ROW_OF_OBJECTS_RES[nowInd] = nowPtr;
            nowInd++;
            READ_ROW_OF_OBJECTS_RES_LEN = 1;
        }
        else
        {
            if (READ_ROW_OF_OBJECTS_RES[nowInd - 1] != nowPtr)
            {
                READ_ROW_OF_OBJECTS_RES[nowInd] = nowPtr;
                nowInd++;
                READ_ROW_OF_OBJECTS_RES_LEN++;
            }
        }
        sleep(1);
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
            READ_ROW_OF_OBJECTS_RES[nowInd] = (int)nowPtr;
            nowInd++;
            READ_ROW_OF_OBJECTS_RES_LEN = 1;
        }
        else
        {
            if (READ_ROW_OF_OBJECTS_RES[nowInd - 1] != (int)nowPtr)
            {
                READ_ROW_OF_OBJECTS_RES[nowInd] = (int)nowPtr;
                nowInd++;
                READ_ROW_OF_OBJECTS_RES_LEN++;
            }
        }
        sleep(1);
    }
}

int *setReadRowOfObjects(int *inPtrsArr, int lenInPtrsArr, short sen, SenSettings *senSet)
{
    for (int i = 0; i < READ_ROW_OF_OBJECTS_RES_LEN; i++)
    {
        READ_ROW_OF_OBJECTS_RES[i] = NULL;
    }
    for (int i = 0; i < lenInPtrsArr; i++)
    {
        READ_ROW_OF_OBJECTS_IN_POINTERS[i] = inPtrsArr[i];
    }
    READ_ROW_OF_OBJECTS_LEN_IN_POINTERS = lenInPtrsArr;
    READ_ROW_OF_OBJECTS_SEN = sen;
    READ_ROW_OF_OBJECTS_SEN_CALIBRATION_PTR = senSet;

    return READ_ROW_OF_OBJECTS_RES;
}

int *startReadRowOfObjectsHSV(SenHSVVals *nothingPtr, int nothingMaxVal, int *inPtrsArr, int lenInPtrsArr, short sen, SenSettings *senSet)
{
    int *buf = setReadRowOfObjects(inPtrsArr, lenInPtrsArr, sen, senSet);
    NOTHING_PTR = nothingPtr;
    NOTHING_MAX_VAL = nothingMaxVal;
    startTask(readRowOfObjectsHSV);
    return buf;
}

int *startReadRowOfObjectsRGB(int *inPtrsArr, int lenInPtrsArr, short sen, SenSettings *senSet)
{
    int *buf = setReadRowOfObjects(inPtrsArr, lenInPtrsArr, sen, senSet);
    startTask(readRowOfObjectsRGB);
    return buf;
}
