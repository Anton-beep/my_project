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
    SenRGBVals buf;
    readCalibratedSenRGB(sen, senParams, &buf);
    return buf.R + buf.G + buf.B;
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

void displayMeanCalibratedRGB(short sen, SenSettings *senSet)
{
    SenRGBVals buf;
    float sumR = 0, sumG = 0, sumB = 0, i = 1;
    while (true)
    {
        readCalibratedSenRGB(sen, senSet, &buf);
        sumR += buf.R;
        sumG += buf.G;
        sumB += buf.B;
        eraseDisplay();
        displayCenteredBigTextLine(2, "R: %f", sumR / i);
        displayCenteredBigTextLine(4, "G: %f", sumG / i);
        displayCenteredBigTextLine(6, "B: %f", sumB / i);
        displayCenteredBigTextLine(8, "Sum: %f", sumR / i + sumG / i + sumB / i);
        i++;
    }
}

void displayMeanCalibratedHSV(short sen, SenSettings *senSet)
{
    SenHSVVals buf;
    float sumH = 0, sumS = 0, sumV = 0, i = 1;
    while (true)
    {
        readCalibratedSenHSV(sen, senSet, &buf);
        sumH += buf.H;
        sumS += buf.S;
        sumV += buf.V;
        eraseDisplay();
        displayCenteredBigTextLine(2, "H: %f", sumH / i);
        displayCenteredBigTextLine(4, "S: %f", sumS / i);
        displayCenteredBigTextLine(6, "V: %f", sumV / i);
        i++;
    }
}


void displayCalibrationValues()
{
    eraseDisplay();
    displayCenteredTextLine(2, "Put on max and press any button");
    flushButtonMessages();
    waitForButtonPress();
    SenRGBVals maxSen1;
    SenRGBVals maxSen2;
    SenRGBVals maxSen3;
    maxSen1.R = 0;
    maxSen1.G = 0;
    maxSen1.B = 0;
    maxSen2.R = 0;
    maxSen2.G = 0;
    maxSen2.B = 0;
    maxSen3.R = 0;
    maxSen3.G = 0;
    maxSen3.B = 0;
    int r, g, b;
    sleep(200);
    flushButtonMessages();
    while (getButtonPress(buttonEnter) == false)
    {
        getColorRawRGB(sen1, r, g, b);
        maxSen1.R = max(maxSen1.R, r);
        maxSen1.G = max(maxSen1.G, g);
        maxSen1.B = max(maxSen1.B, b);
        getColorRawRGB(sen2, r, g, b);
        maxSen2.R = max(maxSen2.R, r);
        maxSen2.G = max(maxSen2.G, g);
        maxSen2.B = max(maxSen2.B, b);
        getColorRawRGB(sen3, r, g, b);
        maxSen3.R = max(maxSen3.R, r);
        maxSen3.G = max(maxSen3.G, g);
        maxSen3.B = max(maxSen3.B, b);
        eraseDisplay();
        displayCenteredTextLine(2, "sen1: R:%d\tG:%d\tB:%d", maxSen1.R, maxSen1.G, maxSen1.B);
        displayCenteredTextLine(4, "sen2: R:%d\tG:%d\tB:%d", maxSen2.R, maxSen2.G, maxSen2.B);
        displayCenteredTextLine(6, "sen3: R:%d\tG:%d\tB:%d", maxSen3.R, maxSen3.G, maxSen3.B);
        displayCenteredTextLine(8, "Press center button to stop");
        sleep(100);
    }
    eraseDisplay();
    displayCenteredTextLine(2, "Put on min and press any button");
    flushButtonMessages();
    waitForButtonPress();
    SenRGBVals minSen1;
    SenRGBVals minSen2;
    SenRGBVals minSen3;
    minSen1.R = 1000;
    minSen1.G = 1000;
    minSen1.B = 1000;
    minSen2.R = 1000;
    minSen2.G = 1000;
    minSen2.B = 1000;
    minSen3.R = 1000;
    minSen3.G = 1000;
    minSen3.B = 1000;
    r = 1000;
    g = 1000;
    b = 1000;
    sleep(200);
    flushButtonMessages();
    while (getButtonPress(buttonEnter) == false)
    {
        getColorRawRGB(sen1, r, g, b);
        minSen1.R = min(minSen1.R, r);
        minSen1.G = min(minSen1.G, g);
        minSen1.B = min(minSen1.B, b);
        getColorRawRGB(sen2, r, g, b);
        minSen2.R = min(minSen2.R, r);
        minSen2.G = min(minSen2.G, g);
        minSen2.B = min(minSen2.B, b);
        getColorRawRGB(sen3, r, g, b);
        minSen3.R = min(minSen3.R, r);
        minSen3.G = min(minSen3.G, g);
        minSen3.B = min(minSen3.B, b);
        eraseDisplay();
        displayCenteredTextLine(2, "sen1: R:%d\tG:%d\tB:%d", minSen1.R, minSen1.G, minSen1.B);
        displayCenteredTextLine(4, "sen2: R:%d\tG:%d\tB:%d", minSen2.R, minSen2.G, minSen2.B);
        displayCenteredTextLine(6, "sen3: R:%d\tG:%d\tB:%d", minSen3.R, minSen3.G, minSen3.B);
        displayCenteredTextLine(8, "Press center button to stop");
        sleep(100);
    }
}