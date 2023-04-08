#include "sen_calibration.c"

float distHSVVals(SenHSVVals *vals1, SenHSVVals *vals2)
{
    return sqrt(pow(vals1->H - vals2->H, 2) + pow(vals1->S - vals2->S, 2) + pow(vals1->V - vals2->V, 2));
}

SenHSVVals *getNearestFromVals(SenHSVVals *inVal, int *vals, int len)
{
    SenHSVVals *minVals = vals[0];
    float minDist = distHSVVals(inVal, (SenHSVVals *)vals[0]);
    float buf;
    for (int i = 0; i < len; i++)
    {
        buf = distHSVVals(inVal, (SenHSVVals *)vals[i]) if (buf < minDist)
        {
            minDist = buf;
            minVals = (SenHSVVals *)vals[i];
        }
    }
}
