#include "tools.c"
#include "structures.c"

void readCalibratedSenRGB(short sen, SenSettings *senParams, SenRGBVals *outParams)
{
    int r, g, b;
    getColorRawRGB(sen, r, g, b);
    outParams->R = map(r, senParams->minR, senParams->maxR, 0, 255);
    outParams->G = map(g, senParams->minG, senParams->maxG, 0, 255);
    outParams->B = map(b, senParams->minB, senParams->maxB, 0, 255);
}

float readCalibratedSenSumRGB(short sen, SenSettings *senParams){
    int r, g, b;
    getColorRawRGB(sen, r, g, b);
    return r + g + b;
}

void RGBtoHSV(SenRGBVals *inRGB, SenHSVVals *outHSV)
{
    float fCMax = max(max(inRGB->R, inRGB->G), inRGB->B);
    float fCMin = min(min(inRGB->R, inRGB->G), inRGB->B);
    float fDelta = fCMax - fCMin;

    if (fDelta > 0)
    {
        if (fCMax == inRGB->R)
        {
            outHSV->H = 60 * (((inRGB->G - inRGB->B) / fDelta) % 6);
        }
        else if (fCMax == inRGB->G)
        {
            outHSV->H = 60 * (((inRGB->B - inRGB->R) / fDelta) + 2);
        }
        else if (fCMax == inRGB->B)
        {
            outHSV->H = 60 * (((inRGB->R - inRGB->G) / fDelta) + 4);
        }

        if (fCMax > 0)
        {
            outHSV->S = fDelta / fCMax;
        }
        else
        {
            outHSV->S = 0;
        }

        outHSV->V = fCMax;
    }
    else
    {
        outHSV->H = 0;
        outHSV->S = 0;
        outHSV->V = fCMax;
    }

    if (outHSV->H < 0)
    {
        outHSV->H = 360 + outHSV->H;
    }
}

void readCalibratedSenHSV(short sen, SenSettings *senParams, SenHSVVals *outParams)
{
    SenRGBVals buf;
    readCalibratedSenRGB(sen, SenSettings, buf);
    RGBtoHSV(buf, outParams);
}
