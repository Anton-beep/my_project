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

float readCalibratedSenSumRGB(short sen, SenSettings *senParams)
{
    int r, g, b;
    getColorRawRGB(sen, r, g, b);
    return r + g + b;
}

void RGBtoHSV(SenRGBVals *inRGB, SenHSVVals *outHSV)
{
    // R, G, B values are divided by 255
    // to change the range from 0..255 to 0..1:
    float r = inRGB->R, g = inRGB->G, b = inRGB->B;
    r /= 255.0;
    g /= 255.0;
    b /= 255.0;
    float buf = max(r, g);
    float cmax = max(buf, b); // maximum of r, g, b
    buf = min(r, g);
    float cmin = min(buf, b); // minimum of r, g, b
    float diff = cmax - cmin; // diff of cmax and cmin.
    if (cmax == cmin)
        outHSV->H = 0;
    else if (cmax == r)
        outHSV->H = (60 * ((g - b) / diff) + 360) % 360;
    else if (cmax == g)
        outHSV->H = (60 * ((b - r) / diff) + 120) % 360;
    else if (cmax == b)
        outHSV->H = (60 * ((r - g) / diff) + 240) % 360;
    // if cmax equal zero
    if (cmax == 0)
        outHSV->S = 0;
    else
        outHSV->S = (diff / cmax) * 100;
    // compute v
    outHSV->V = cmax * 100;
}

void readCalibratedSenHSV(short sen, SenSettings *senParams, SenHSVVals *outParams)
{
    SenRGBVals buf;
    readCalibratedSenRGB(sen, senParams, &buf);
    RGBtoHSV(&buf, outParams);
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

void displaySenRGB()
{
    int sens[4] = {S1, S2, S3, S4};
    int r, g, b;
    while (true)
    {
        eraseDisplay();
        for (int i = 0; i < 4; i++)
        {
            if (SensorType[sens[i]] != 0)
            {
                getColorRawRGB(sens[i], r, g, b);
                displayTextLine(5 + i * 2, "sen%d->R:%d\tG:%d\tB:%d", i + 1, r, g, b);
            }
        }
        sleep(100);
    }
}
