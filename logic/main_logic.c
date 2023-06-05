#include "include/PID_c_motors.c"
#include "include/motion.c"
#include "include/sen_for_colors.c"
#include "include/motion_with_sensors.c"
#include "include/PID_c_manip.c"
#include "logic_vars.c"
#include "define_vals.c"

void startReadPullShip()
{
    // set manip

    sleep(600);
    moveBCAccelPartMainB(150, -20, 20, -35, 35);
    int *firstScanOut = startReadRowOfObjectsHSV(&NOTHING_FIRST_SCAN, 13, IN_PTRS_FIRST_SCAN, 2, sen3, &SEN3_CALIBRATION);
    line2SenAccelPart(&DEFAULT_LINE_PID_MEDIUM, 230, 35, 30);
    setNewMotBCPowersAndRatio(-30, 30);
    waitForSen2(395);
    moveBCAccelPartMainB(163, -30, 30, -50, 50);
    moveBCAccelPartMainB(193, -50, 50, -15, 15);
    stopBC();
    stopTask(readRowOfObjectsHSV);
    sleep(100);
    moveBCAccelPartMainB(215, 60, -60, -20, 20);
    tankTurnNS3Parts(30, 30, 30, -20, -20, -70, -70, -17, -17);

    resetMotorEncoder(motD);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 70, 175, 130, 20, 60, 30);
    setDegManipD(-43, 2, -30);
    moveBCMainC(1600, -21, 79);
    moveBC(150, -14, 14);
#if SLEEP_ON_FIRST_TAKE_BIG_SHIP == 1
    sleep(5000);
#endif
    stopBC();
    setTimeManipD(500, -79, -70);
    sleep(600);

    if (firstScanOut[1] == &GREEN_CUBE_FIRST_SCAN)
    {
        FIRST_CUBE_TO_TAKE = 0;
    }
    else if (firstScanOut[1] == &BLUE_CUBE_FIRST_SCAN)
    {
        FIRST_CUBE_TO_TAKE = 1;
    }

    if (firstScanOut[3] == &GREEN_CUBE_FIRST_SCAN)
    {
        SECOND_CUBE_TO_TAKE = 0;
    }
    else if (firstScanOut[3] == &BLUE_CUBE_FIRST_SCAN)
    {
        SECOND_CUBE_TO_TAKE = 1;
    }
    eraseDisplay();
    for (int i = 0; i < 10; i++)
    {
        if (firstScanOut[i] == &GREEN_CUBE_FIRST_SCAN)
        {
            displayCenteredTextLine(i, "GREEN");
        }
        else if (firstScanOut[i] == &BLUE_CUBE_FIRST_SCAN)
        {
            displayCenteredTextLine(i, "BLUE");
        }
        else
        {
            displayCenteredTextLine(i, "NONE");
        }
    }

#if WRITE_DATA_IN_FILE == 1
    stopBC();
    fileWriteChar(DEBUG_FILE_HND, FIRST_CUBE_TO_TAKE);
    fileWriteChar(DEBUG_FILE_HND, SECOND_CUBE_TO_TAKE);
#endif

    moveBCAccelPartMainB(190, 23, -23, 70, -70);
    moveBCMainC(200, 70, -100);
    moveBC(200, 100, -70);
    moveBCAccelPartMainB(250, 100, -100, 21, -21);
    waitFor2Sen(150, 150);

    moveBCMainC(500, 16, 60);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 100, 40, 180, 23, 100, 21);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 50, 50);

    moveBC3PartsMainC(134, 200, 156, 10, 60, 15, 90, 10, 60);
}

void conCrane()
{
    line2Sen3Parts(&DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_FAST, 189, 1941, 250, 23, 100, 25);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 25, 50, 50);

    moveBC3Parts(64, 360, 66, -60, -10, -90, -15, -60, -10);

    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 306, 170, 100, 25, 70, 20);
    moveBC3Parts(56, 490, 83, 19, -19, 80, -80, 20, -20);

    tankTurnNS3Parts(18, 49.5, 22.5, -20, -20, -60, -60, -17, -17);
    stopBC();
    sleep(250);

    line2Sen3Parts(&DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_FAST, 100, 2100, 270, 35, 95, 30);
}

void waitForCubes()
{
    SenRGBVals buf1;
    SenRGBVals buf2;
    readCalibratedSenRGB(sen1, &SEN1_CALIBRATION, &buf1);
    readCalibratedSenRGB(sen2, &SEN2_CALIBRATION, &buf2);
    while ((buf1.B - buf1.R < 19 || buf1.B - buf1.G < 19) || (buf2.B - buf2.R < 19 || buf2.B - buf2.G < 19))
    {
        readCalibratedSenRGB(sen1, &SEN1_CALIBRATION, &buf1);
        readCalibratedSenRGB(sen2, &SEN2_CALIBRATION, &buf2);
    }
}

void goToCubes()
{
    setTimeManipD(250, -50, -40);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 80, 80);
    tankTurnNS3Parts(36, 63, 75, -30, -30, -55, -55, -21, -21);
    tankTurnSenCrawl(-21, -21, -20, -20, 150, 400);

    line2SenDist(&DEFAULT_LINE_PID_SLOW, 100, 20);
    stopBC();

    setTimeManipD(250, 100, 2);
    sleep(250);

    moveBC3Parts(65, 97, 64, 20, -20, 35, -35, 17, 17);
    tankTurnNS3Parts(18, 49.5, 22.5, 19, 19, 65, 65, 17, 17);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 380, 300, 17, 98, 9);
    setNewMotBCPowersAndRatio(-9, 9);
    waitForCubes();
    moveBC(11, 18, -18);
}

void readCubes()
{
    int cubesScan[4];
    moveBC3PartsMainC(27, 128, 34, -6, -22, -12, -44, -6, -22);
    stopBC();

    cubesScan[0] = readWindowHSV(sen3, &SEN3_CALIBRATION_1CUBE, &NOTHING_CUBES, 13, IN_PTRS_1CUBE, 2);

// POS TO CALIBRATE 1 AND 2 CUBES
#if DEB_CALIBRATE_CUBES_1PART == 1
    stopBC();
    displayCalibrationValues();
    displayMeanCalibratedHSV(sen3, &SEN3_CALIBRATION_1CUBE);
#endif

    moveBC(30, 13, -13);
    moveBC3PartsMainC(36, 74, 45, -6, -13, -18, -39, -6, -13);
    stopBC();

    cubesScan[1] = readWindowHSV(sen3, &SEN3_CALIBRATION_1CUBE, &NOTHING_CUBES, 13, IN_PTRS_1CUBE, 2);

    moveBC(123, -55, 7);
    stopBC();
#if DEB_CALIBRATE_CUBES_2PART == 1
    // POS TO CALIBRATE 3 AND 4 CUBES
    stopBC();
    displayCalibrationValues();
    displayMeanCalibratedHSV(sen3, &SEN3_CALIBRATION_CUBES);
#endif
    cubesScan[2] = readWindowHSV(sen3, &SEN3_CALIBRATION_CUBES, &NOTHING_CUBES, 13, IN_PTRS_CUBES, 2);

    moveBC(136, -18, 16);
    stopBC();

    cubesScan[3] = readWindowHSV(sen3, &SEN3_CALIBRATION_CUBES, &NOTHING_CUBES, 13, IN_PTRS_CUBES, 2);

    for (int i = 0; i < 4; i++)
    {
        if (cubesScan[i] == &GREEN_CUBE_CUBES || cubesScan[i] == &ANOTHER_GREEN_CUBE_CUBES)
        {
            CUBES_COLORS[i] = 0;
        }
        else if (cubesScan[i] == &BLUE_CUBE_CUBES || cubesScan[i] == &ANOTHER_BLUE_CUBE_CUBES)
        {
            CUBES_COLORS[i] = 1;
        }
        else
        {
            CUBES_COLORS[i] = -2;
        }
    }
    eraseDisplay();
    for (int i = 0; i < 4; i++)
    {
        if (cubesScan[i] == &GREEN_CUBE_CUBES || cubesScan[i] == &ANOTHER_GREEN_CUBE_CUBES)
        {
            displayCenteredTextLine(i * 2, "GREEN");
        }
        else if (cubesScan[i] == &BLUE_CUBE_CUBES || cubesScan[i] == &ANOTHER_BLUE_CUBE_CUBES)
        {
            displayCenteredTextLine(i * 2, "BLUE");
        }
        else if (cubesScan[i] == &NOTHING_CUBES)
        {
            displayCenteredTextLine(i * 2, "NOTHING");
        }
        else
        {
            displayCenteredTextLine(i * 2, "ERROR");
        }
    }

#if WRITE_DATA_IN_FILE == 1
    for (int i = 0; i < 4; i++)
    {
        fileWriteChar(DEBUG_FILE_HND, CUBES_COLORS[i]);
    }
    fileClose(DEBUG_FILE_HND);
#endif

    moveBC(130, 20, -20);
    tankTurnNS3Parts(30, 30, 30, -20, -20, -50, -50, -20, -20);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 125, 5, 125, 20, 55, 17);

    tankTurnNS3Parts(36, 99, 45, 30, 30, 90, 90, 17, 17);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 25, 130, 125, 20, 55, 9);
    setNewMotBCPowersAndRatio(-9, 9);
    waitForCubes();
    moveBC(69, 18, -18);
}

// left to right
void takeCube1()
{
    moveBC(229, -24, -60);
    moveBC(557, 60, 24);
#if SLEEP_ON_CUBES == 1
    stopBC();
    sleep(5000);
#endif
    stopBC();
    setDegManipD(-70, 1, -20);
    moveBC3Parts(40, 115, 50, -20, 20, -40, 40, -13, 13);
    moveBC(40, -13, 13);
    stopBC();
    setTimeManipD(400, -65, -50);
    sleep(450);

    moveBC3Parts(20, 105, 25, 20, -20, 40, -40, 15, -15);
    moveBC3Parts(189, 274, 477, 70, 13, 98, 18.2, 70, 13);
}

void takeCube2()
{
    moveBC(208, -26, -44);
    moveBC(337, 44, 26);
#if SLEEP_ON_CUBES == 1
    stopBC();
    sleep(5000);
#endif
    stopBC();
    setDegManipD(-70, 1, -25);
    moveBC3Parts(40, 45, 50, -20, 20, -40, 40, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    setTimeManipD(400, -65, -50);
    sleep(450);

    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBC3Parts(189, 55, 477, 70, 45, 98, 63, 70, 45);
}

void takeCube3()
{
    moveBC(318, 44, 26);
    moveBC(190, -26, -44);
#if SLEEP_ON_CUBES == 1
    stopBC();
    sleep(5000);
#endif
    stopBC();
    setDegManipD(-70, 1, -25);
    moveBC3Parts(40, 45, 50, -20, 20, -40, 40, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    setTimeManipD(400, -65, -50);
    sleep(450);

    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBC3PartsMainC(189, 39, 477, -45, -70, -63, -98, -45, -70);
}

void takeCube4()
{
    moveBC(535, 60, 24);
    moveBC(217, -24, -60);
#if SLEEP_ON_CUBES == 1
    stopBC();
    sleep(5000);
#endif
    stopBC();
    setDegManipD(-70, 1, -25);
    moveBC3Parts(40, 86, 50, -20, 20, -40, 40, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    setTimeManipD(400, -65, -50);
    sleep(450);

    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBC3PartsMainC(189, 280, 477, -13, -70, -18.2, -98, -13, -70)
}

void putCubeOnShip()
{
    setDegManipD(142, 1, -12, 0);
    sleep(450);
    // setTimeManipD(150, 52, 0);
    // sleep(200);
    moveBCAccelPartMainB(35, 20, -20, 60, -60);
    setTimeManipD(500, -70, -5);
    moveBCAccelPartMainB(195, 60, -60, 19, -19);
}

void putCubeOnSmallShip()
{
    setDegManipD(79, 1, 26);
    sleep(450);
    moveBC(30, -20, 20);
    setTimeManipD(500, 100, 2);
    // setTimeManipD(150, 52, 0);
    // sleep(200);
    moveBCAccelPartMainB(100, 20, -20, 60, -60);
    setTimeManipD(500, -30, -5);
    moveBCAccelPartMainB(190, 60, -60, 19, -19);
    stopBC();
}

void putCubes()
{
    short secCube;
    if (CUBES_COLORS[0] == FIRST_CUBE_TO_TAKE)
    {
        secCube = SECOND_CUBE_TO_TAKE;
        takeCube1();
        CUBES_COLORS[0] = -1;
    }
    else if (CUBES_COLORS[0] == SECOND_CUBE_TO_TAKE)
    {
        secCube = FIRST_CUBE_TO_TAKE;
        takeCube1();
        CUBES_COLORS[0] = -1;
    }
    else if (CUBES_COLORS[3] == FIRST_CUBE_TO_TAKE)
    {
        secCube = SECOND_CUBE_TO_TAKE;
        takeCube4();
        CUBES_COLORS[3] = -1;
    }
    else if (CUBES_COLORS[3] == SECOND_CUBE_TO_TAKE)
    {
        secCube = FIRST_CUBE_TO_TAKE;
        takeCube4();
        CUBES_COLORS[3] = -1;
    }
    else if (CUBES_COLORS[1] == FIRST_CUBE_TO_TAKE)
    {
        secCube = SECOND_CUBE_TO_TAKE;
        takeCube2();
        CUBES_COLORS[1] = -1;
    }
    else if (CUBES_COLORS[1] == SECOND_CUBE_TO_TAKE)
    {
        secCube = FIRST_CUBE_TO_TAKE;
        takeCube2();
        CUBES_COLORS[1] = -1;
    }
    else if (CUBES_COLORS[2] == FIRST_CUBE_TO_TAKE)
    {
        secCube = SECOND_CUBE_TO_TAKE;
        takeCube3();
        CUBES_COLORS[2] = -1;
    }
    else if (CUBES_COLORS[2] == SECOND_CUBE_TO_TAKE)
    {
        secCube = FIRST_CUBE_TO_TAKE;
        takeCube3();
        CUBES_COLORS[2] = -1;
    }
    else
    {
        secCube = SECOND_CUBE_TO_TAKE;
        takeCube1();
        CUBES_COLORS[0] = -1;
    }

    stopBC();

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 60, 233, 237, 20, 70, 18);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 50, 50);

    moveBC(28, -16, 16);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 13, 13);

    moveBCMainC(310, 30, 67);
    moveBC(310, -67, -30);
    moveBC(155, -20, 20);
    stopBC();
    putCubeOnShip();
    moveBC(40, 20, -20);
    stopBC();
    setTimeManipD(250, 100, 2);
    tankTurnNS3Parts(30, 30, 30, 20, 20, 60, 60, 20, 20);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 170, 351, 17, 90, 9);
    setNewMotBCPowersAndRatio(-9, 9);
    waitForCubes();
    moveBC(69, 18, -18);

    if (secCube == CUBES_COLORS[0])
    {
        takeCube1();
        CUBES_COLORS[0] = -1;
    }
    else if (secCube == CUBES_COLORS[3])
    {
        takeCube4();
        CUBES_COLORS[3] = -1;
    }
    else if (secCube == CUBES_COLORS[1])
    {
        takeCube2();
        CUBES_COLORS[1] = -1;
    }
    else
    {
        takeCube3();
        CUBES_COLORS[2] = -1;
    }

    stopBC();

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 60, 253, 237, 20, 80, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 50, 50);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 115, 0, 50, 17, 32, 16);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 15, 15);

    moveBCMainC(310, 30, 67);
    moveBC(310, -67, -30);
    moveBC(145, -20, 20);
    stopBC();
    putCubeOnShip();

    stopBC();
    setDegManipD(200, 1, 20, 0);
    moveBC(300, -40, -30);
    moveBCMainC(300, 30, 40);
}

void moveBigShipToWhite()
{
    setTimeManipD(550, 0, -60);
    line2SenDist(&DEFAULT_LINE_PID_SLOW, 200, 17);
    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 70, 502, 88, 17, 95, 24);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 24, 150, 150);

    moveBC3Parts(43, 119, 56, -25, 25, -40, 40, -23, 23);
    stopC();
    moveB(40, -23);
    stopB();
    setTimeManipD(350, 100, 2);
    sleep(300);
    moveB(40, 23);

    moveBC3Parts(20, 45, 25, 20, -20, 30, -30, 17, -17);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 60, 60, 17, 17);
}

void take2WhiteCube()
{
    moveBC(260, 80, 10);
    setDegManipD(-60, 1, -20);
    moveBCMainC(258, -10, -80);
    moveBC3Parts(40, 50, 80, -17, 17, -40, 40, -17, 17);
    moveBC(100, -17, 17);
    stopBC();
    setTimeManipD(400, -58, -50);
    sleep(400);

    moveBC(100, 20, -20);
    moveBC3Parts(74, 204, 92, -20, -40, -45, -90, -10, -20);
}

void whiteCubeOnBig()
{
    setTimeManipD(350, 100, 2);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 390, 301, 17, 98, 15);
    setNewMotBCPowersAndRatio(-13, 13);
    resetMotorEncoder(motD);
    waitFor2Sen(560, 560);
    stopBC();
    sleep(200);

    take2WhiteCube();
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SLOW, 80, 380, 100, 20, 80, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 50, 50);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 58, 191, 73, 20, 50, 17);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 15, 15);

    moveBCMainC(310, 30, 67);
    moveBC(315, -67, -30);
    moveBC(10, -20, 20);
    stopBC();
    putCubeOnShip();

    moveBC(460, -35, -60);

    stopBC();
}

void takeAnyLeftCube()
{
    if (CUBES_COLORS[0] != -1)
    {
        takeCube1();
        CUBES_COLORS[0] = -1;
    }
    else if (CUBES_COLORS[3] != -1)
    {
        takeCube4();
        CUBES_COLORS[3] = -1;
    }
    else if (CUBES_COLORS[1] != -1)
    {
        takeCube2();
        CUBES_COLORS[1] = -1;
    }
    else
    {
        takeCube3();
        CUBES_COLORS[2] = -1;
    }
}

void goToSmallShip()
{
    setTimeManipD(350, 100, 2);

    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 60, 605, 75, 25, 75, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 16, 150, 150);

    moveBC3Parts(30, 50, 38, -20, 20, -40, 40, -20, 20);
    stopBC();
    setDegManipD(-52, 1, -30);
    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 60, 60, 17, 17);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SLOW, 100, 205, 125, 20, 80, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 16, 200, 200);
    stopBC();

    setTimeManipD(350, -50, -40);
    sleep(500);

    moveBC3Parts(60, 365, 150, 20, -20, 60, -60, 17, -17);
    setNewMotBCPowersAndRatio(17, -17);
    waitFor2Sen(150, 150);
    moveBC3Parts(30, 30, 38, -20, 20, -40, 40, -20, 20);
    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 45, 45, 17, 17);

    line2SenDist(&DEFAULT_LINE_PID_MEDIUM, 245, 25);
    stopBC();

    setTimeManipD(250, 100, 10);
    sleep(400);
}

void putCubesOnSmallShip()
{
    moveBC3Parts(35, 130, 44, 20, -20, 50, -50, 17, 17);

    tankTurnNS3Parts(18, 49.5, 22.5, 19, 19, 90, 90, 17, 17);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 360, 301, 17, 98, 15);
    setNewMotBCPowersAndRatio(-9, 9);
    waitForCubes();
    moveBC(69, 18, -18);

    takeAnyLeftCube();

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SLOW, 30, 410, 37, 20, 70, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 50, 50);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SLOW, 110, 0, 48, 17, 33, 16);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 15, 15);

    moveBCMainC(310, 30, 67);
    moveBC(315, -67, -30);
    moveBC(190, -20, 20);
    stopBC();

    putCubeOnShip();
    moveBC(60, 20, -20);

    setTimeManipD(250, 100, 2);
    tankTurnNS3Parts(30, 30, 30, 20, 20, 60, 60, 20, 20);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 350, 301, 17, 90, 13);
    setNewMotBCPowersAndRatio(-9, 9);
    waitForCubes();
    moveBC(69, 18, -18);

    takeAnyLeftCube();

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SLOW, 30, 410, 37, 20, 60, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 50, 50);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 185, 0, 95, 17, 39, 16);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 15, 15);

    moveBCMainC(310, 30, 67);
    moveBC(315, -67, -30);
    moveBC(165, -20, 20);
    stopBC();
    sleep(200);

    putCubeOnShip();
    moveBC(160, 20, -20);
    stopBC();

    setTimeManipD(250, 100, 2);
    sleep(500);
    setDegManipD(-46, 1, -30);
    moveBCMainC(240, 5, 70);
    moveBC(240, -70, -5);
    line2SenDist(&DEFAULT_LINE_PID_SLOW, 250, 18);
    setTimeManipD(400, -70, -50);
    line2SenDist(&DEFAULT_LINE_PID_SLOW, 70, 18);
}

void shipsAndFinish()
{
    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 100, 220, 100, 25, 75, 21);
    line2SenCrawl(&DEFAULT_LINE_PID_MEDIUM, 21, 150, 150);
    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 100, 1100, 300, 40, 100, 37);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 37, 150, 150);

    moveBC3Parts(20, 0, 65, -37, 37, -45, 45, -25, 25);
    moveBC3Parts(50, 0, 35, 25, -25, 40, -40, 25, -25);

    stopB();

    turnCDegr(18, 49.5, 22.5, -25, -70, -25);
    moveBC3Parts(30, 152, 38, -20, 20, -60, 60, -17, 17);
    stopBC();
    setTimeManipD(300, 100, 20);
    sleep(300);
    moveBC3Parts(30, 152, 38, 20, -20, 60, -60, 17, -17);
    turnCDegr(18, 49.5, 22.5, 20, 90, 17);

    moveBC3Parts(30, 70, 38, -20, 20, -40, 40, -20, 20);
    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 60, 60, 17, 17);

    line2SenAccelPart(&DEFAULT_LINE_PID_MEDIUM, 100, 20, 80);
    line2SenDist(&DEFAULT_LINE_PID_MEDIUM, 375, 80);
    moveBCAccelPartMainB(360, -80, 80, -13, 13);
    stopBC();
}

void mainLogic()
{
    startReadPullShip();
    conCrane();
    goToCubes();
    readCubes();
    putCubes();
    moveBigShipToWhite();
    whiteCubeOnBig();
    goToSmallShip();
    putCubesOnSmallShip();
    shipsAndFinish();
}

void testFunc()
{
    displayMeanCalibratedHSV(sen3, &SEN3_CALIBRATION);

    // setTimeManipD(400, -65, -50);
    // sleep(3000);
    // putCubeOnShip();

    // tankTurnNS3Parts(90, 360 * 2, 90, -20, -20, -50, -50, -17, -17);

    // displayMeanCalibratedHSV(sen3, &SEN3_CALIBRATION);

    // line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 50, 50);

    // moveBC3PartsMainC(334, 0, 156, 10, 60, 15, 90, 10, 60);
    // stopBC();

    // line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 40, 50, 50);
    //  debReadAndShowRGB(sen1, &SEN1_CALIBRATION);
    //   displayMeanCalibratedRGB(sen2, &SEN2_CALIBRATION);

    // displayCalibrationValues();
    //     bool *flag = setDegManipD(&MANIP_D_PID_SETTINGS, 150, 2, 10);
    //     while (*flag)
    //         ;

    // debReadAndShowHSV(sen3, &SEN3_CALIBRATION);

    // setTimeManipD(150, 100, 2);
    // sleep(1000);
    // resetMotorEncoder(motD);
    // setTimeManipD(500, -50, -20);
    // sleep(5000);
    // setDegManipD(&MANIP_D_PID_SETTINGS, 160, 2, 0);
    // sleep(300);
    // moveBC(100, 20, -20);
    // setTimeManipD(500, -30, -10);
    // moveBC(100, 20, -20);
    // stopBC();

    // takeCube1();
    //  setTimeManipD(3000, -50, -50);
    //  sleep(5000);
    //  setTimeManipD(3000, 70, 70);

    // line2SenCrawl(&DEFAULT_LINE_PID_SUPRA, 85, 200, 200);
    // stopBC();

    //  displayCalibrationValues();
}