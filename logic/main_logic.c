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
    startTimeD(350, 100, 2);
    moveBCAccelPartMainB(100, -20, 20, -40, 40);
    int *firstScanOut = startReadRowOfObjectsHSV(&NOTHING_FIRST_SCAN, IN_PTRS_FIRST_SCAN, 2, sen3, &SEN3_CALIBRATION);
    line2SenAccelPart(&DEFAULT_LINE_PID_MEDIUM, 185, 40, 30);
    setNewMotBCPowersAndRatio(-30, 30);
    waitForSen2(395);
    moveBCAccelPartMainB(160, -30, 30, -60, 60);
    moveBCAccelPartMainB(190, -60, 60, -15, 15);
    resetMotorEncoder(motD);
    stopBC();
    stopTask(readRowOfObjectsHSV);
    sleep(100);
    moveBCAccelPartMainB(215, 60, -60, -20, 20);
    tankTurnNS3Parts(30, 30, 30, -20, -20, -60, -60, -17, -17);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 70, 175, 130, 20, 60, 30);
    stratManipD(&MANIP_D_PID_SETTINGS, -57, 2, -30);
    moveBCMainC(1592, -21, 79);
    stopTask(manipMoveD);
    moveBC(160, -13, 13);
    moveBC(5, 17, -17);
    stopBC();
#if SLEEP_ON_FIRST_TAKE_BIG_SHIP == 1
    sleep(5000);
#endif
    startTimeD(500, -65, -70);
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
    moveBCAccelPartMainB(170, 80, -80, 21, -21);
    waitForLine(150, 150);

    moveBCMainC(500, 16, 60);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 100, 10, 180, 23, 90, 21);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 50, 50);

    moveBC3PartsMainC(234, 100, 156, 10, 60, 15, 90, 10, 60);
}

void conCrane()
{
    line2Sen3Parts(&DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_FAST, 189, 1714, 477, 23, 100, 21);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 50, 50);

    moveBC3Parts(334, 0, 156, -60, -10, -90, -15, -60, -10);

    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 106, 260, 133, 20, 70, 20);
    moveBC3Parts(106, 360, 133, 19, -19, 80, -80, 20, -20);

    tankTurnNS3Parts(18, 49.5, 22.5, -20, -20, -70, -70, -17, -17);

    line2Sen3Parts(&DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_FAST, 100, 2100, 270, 35, 95, 30);
}

void waitForCubes()
{
    SenRGBVals buf1;
    SenRGBVals buf2;
    readCalibratedSenRGB(sen1, &SEN1_CALIBRATION, &buf1);
    readCalibratedSenRGB(sen2, &SEN2_CALIBRATION, &buf2);
    while ((buf1.B - buf1.R < 20 || buf1.B - buf1.G < 20) || (buf2.B - buf2.R < 20 || buf2.B - buf2.G < 20))
    {
        readCalibratedSenRGB(sen1, &SEN1_CALIBRATION, &buf1);
        readCalibratedSenRGB(sen2, &SEN2_CALIBRATION, &buf2);
    }
}

void goToCubes()
{
    startTimeD(250, -50, -40);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 80, 80);
    tankTurnNS3Parts(36, 69, 75, -30, -30, -70, -70, -17, -17);

    line2SenDist(&DEFAULT_LINE_PID_SLOW, 100, 20);
    stopBC();

    startTimeD(250, 100, 2);
    sleep(200);

    moveBC3Parts(65, 100, 64, 20, -20, 35, -35, 17, 17);
    tankTurnNS3Parts(18, 49.5, 22.5, 19, 19, 70, 70, 17, 17);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 380, 300, 17, 98, 8);
    setNewMotBCPowersAndRatio(-8, 8);
    waitForCubes();
    moveBC(11, 18, -18);
}

void readCubes()
{
    int cubesScan[4];
    moveBC3PartsMainC(27, 78, 34, -6, -22, -12, -44, -6, -22);
    stopBC();
    sleep(400);
    cubesScan[0] = readWindowHSV(sen3, &SEN3_CALIBRATION_1CUBE, &NOTHING_CUBES, IN_PTRS_1CUBE, 2);

// POS TO CALIBRATE 1 AND 2 CUBES
#if DEB_CALIBRATE_CUBES_1PART == 1
    stopBC();
    displayCalibrationValues();
    displayMeanCalibratedHSV(sen3, &SEN3_CALIBRATION_1CUBE);
#endif

    moveBC(30, 13, -13);
    moveBC3PartsMainC(36, 101, 45, -5, -13, -15, -39, -5, -13);
    stopBC();
    sleep(5000);
    cubesScan[1] = readWindowHSV(sen3, &SEN3_CALIBRATION_1CUBE, &NOTHING_CUBES, IN_PTRS_1CUBE, 2);

    moveBC(120, -45, 7);
    stopBC();
#if DEB_CALIBRATE_CUBES_2PART == 1
    // POS TO CALIBRATE 3 AND 4 CUBES
    stopBC();
    displayCalibrationValues();
    displayMeanCalibratedHSV(sen3, &SEN3_CALIBRATION_CUBES);
#endif
    cubesScan[2] = readWindowHSV(sen3, &SEN3_CALIBRATION_CUBES, &NOTHING_CUBES, IN_PTRS_CUBES, 2);

    moveBC3Parts(27, 65, 34, -10, 4, -50, 20, -10, 4);
    stopBC();
#if SLEEP_ON_READ_CUBE4 == 1
    sleep(5000);
#endif

    cubesScan[3] = readWindowHSV(sen3, &SEN3_CALIBRATION_CUBES, &NOTHING_CUBES, IN_PTRS_CUBES, 2);

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

    moveC(29, 30);
    moveBC(120, 20, -20);
    moveBC3Parts(238, 0, 112, -30, -27, -60, -54, -30, -27);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 125, 55, 125, 20, 55, 17);

    tankTurnNS3Parts(36, 99, 45, 30, 30, 90, 90, 17, 17);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 25, 180, 125, 20, 55, 9);
    setNewMotBCPowersAndRatio(-9, 9);
    waitForCubes();
    stopBC();
    sleep(300);
    moveBC(69, 18, -18);
    stopBC();
    sleep(300);
}

// left to right
void takeCube1()
{
    moveBCMainC(530, -18, -54);
    moveBC(515, 54, 18);
#if SLEEP_ON_CUBES == 1
    stopBC();
    sleep(5000);
#endif
    stopBC();
    stratManipD(&MANIP_D_PID_SETTINGS, -86, 1, -20);
    sleep(200);
    moveBC3Parts(40, 115, 50, -20, 20, -40, 40, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    stopTask(manipMoveD);
    startTimeD(400, -55, -50);
    sleep(450);

    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBC3Parts(189, 274, 477, 70, 13, 98, 18.2, 70, 13);
}

void takeCube2()
{
    moveBCMainC(268, -11, -44);
    moveBC(266, 44, 11);
#if SLEEP_ON_CUBES == 1
    stopBC();
    sleep(5000);
#endif
    stopBC();
    stratManipD(&MANIP_D_PID_SETTINGS, -86, 1, -20);
    sleep(200);
    moveBC3Parts(40, 116, 50, -20, 20, -40, 40, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    stopTask(manipMoveD);
    startTimeD(400, -55, -50);
    sleep(450);

    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBC3Parts(189, 55, 477, 70, 45, 98, 63, 70, 45);
}

void takeCube3()
{
    moveBC(230, 44, 11);
    moveBCMainC(230, -11, -44);
#if SLEEP_ON_CUBES == 1
    stopBC();
    sleep(5000);
#endif
    stopBC();
    stratManipD(&MANIP_D_PID_SETTINGS, -83, 1, -20);
    sleep(200);
    moveBC3Parts(40, 97, 50, -20, 20, -40, 40, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    stopTask(manipMoveD);
    startTimeD(400, -55, -50);
    sleep(450);

    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBC3PartsMainC(189, 39, 477, -45, -70, -63, -98, -45, -70);
}

void takeCube4()
{
    moveBC(495, 54, 18);
    moveBCMainC(502, -18, -54);
#if SLEEP_ON_CUBES == 1
    stopBC();
    sleep(5000);
#endif
    stopBC();
    stratManipD(&MANIP_D_PID_SETTINGS, -81, 1, -20);
    sleep(200);
    moveBC3Parts(40, 120, 50, -20, 20, -40, 40, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    stopTask(manipMoveD);
    startTimeD(400, -55, -50);
    sleep(450);

    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBC3PartsMainC(189, 256, 477, -13, -70, -18.2, -98, -13, -70)
}

void putCubeOnShip()
{
    stratManipD(&MANIP_D_PID_SETTINGS, 80, 1, 30);
    sleep(400);
    // startTimeD(150, 52, 0);
    // sleep(200);
    moveBCAccelPartMainB(100, 20, -20, 60, -60);
    startTimeD(500, -30, -5);
    moveBCAccelPartMainB(100, 60, -60, 19, -19);
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

    moveBC(25, -16, 16);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 13, 13);

    moveBCMainC(310, 30, 67);
    moveBC(337, -67, -30);
    moveBC(140, -20, 20);
    stopBC();
    putCubeOnShip();
    startTimeD(350, 60, 2);
    moveBC3Parts(88, 242, 110, 60, 20, 90, 30, 60, 20);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 310, 351, 17, 90, 9);
    setNewMotBCPowersAndRatio(-9, 9);
    waitForCubes();
    stopBC();
    sleep(300);
    moveBC(69, 18, -18);
    stopBC();
    sleep(300);

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

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 112, 0, 53, 17, 32, 16);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 15, 15);

    moveBCMainC(310, 30, 67);
    moveBC(330, -67, -30);
    moveBC(140, -20, 20);
    stopBC();
    putCubeOnShip();

    stopBC();
    sleep(500);
    stratManipD(&MANIP_D_PID_SETTINGS, 172, 1, 0, 0);
    sleep(200);
    moveBC(270, -70, -10);
    moveBCMainC(270, 10, 70);
}

void moveBigShipToWhite()
{
    startTimeD(350, 0, -60);
    line2SenDist(&DEFAULT_LINE_PID_SLOW, 150, 17);
    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 70, 322, 88, 17, 75, 15);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 15, 150, 150);

    moveBC3Parts(43, 119, 56, -15, 15, -40, 40, -19, 19);
    stopC();
    moveB(45, -22);
    stopB();
    startTimeD(350, 100, 2);
    sleep(550);
    moveB(45, 22);

    moveBC3Parts(20, 45, 25, 18, -18, 30, -30, 14, -14);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 60, 60, 17, 17);
}

void take2WhiteCube()
{
    moveBC(260, 80, 10);
    stratManipD(&MANIP_D_PID_SETTINGS, -70, 1, -20);
    moveBCMainC(258, -10, -80);
    moveBC3Parts(40, 50, 80, -17, 17, -40, 40, -17, 17);
    moveBC(100, -17, 17);
    stopBC();
    stopTask(manipMoveD);
    startTimeD(400, -58, -50);
    sleep(400);

    moveBC(100, 20, -20);
    moveBC3Parts(74, 204, 92, -20, -40, -45, -90, -10, -20);
}

void whiteCubeOnBig()
{
    startTimeD(350, 100, 2);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 410, 301, 17, 98, 15);
    setNewMotBCPowersAndRatio(-13, 13);
    resetMotorEncoder(motD);
    waitForLine(560, 560);
    stopBC();
    sleep(200);

    take2WhiteCube();
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SLOW, 80, 380, 100, 20, 80, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 50, 50);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 58, 191, 73, 20, 50, 17);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 15, 15);

    moveBCMainC(310, 30, 67);
    moveBC(330, -67, -30);
    moveBC(30, -20, 20);
    stopBC();
    putCubeOnShip();

    moveBC(470, -40, -60);
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
    startTimeD(350, 100, 2);

    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 60, 605, 75, 25, 75, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 16, 150, 150);

    moveBC3Parts(30, 50, 38, -20, 20, -40, 40, -20, 20);
    stopBC();
    stratManipD(&MANIP_D_PID_SETTINGS, -72, 1, -30);
    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 60, 60, 17, 17);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SLOW, 100, 205, 125, 20, 80, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 16, 200, 200);
    stopBC();

    startTimeD(350, -50, -40);
    sleep(600);

    moveBC3Parts(60, 365, 150, 20, -20, 70, -70, 17, -17);
    setNewMotBCPowersAndRatio(17, -17);
    waitForLine(150, 150);
    moveBC3Parts(30, 30, 38, -20, 20, -40, 40, -20, 20);
    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 17, 17);

    line2SenDist(&DEFAULT_LINE_PID_SLOW, 230, 25);
    stopBC();

    startTimeD(250, 100, 2);
    sleep(200);
}

void putCubesOnSmallShip()
{
    moveBC3Parts(35, 125, 44, 20, -20, 50, -50, 17, 17);

    tankTurnNS3Parts(18, 49.5, 22.5, 19, 19, 90, 90, 17, 17);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 390, 301, 17, 98, 15);
    setNewMotBCPowersAndRatio(-13, 13);
    waitForCubes();
    moveBC(69, 18, -18);

    takeAnyLeftCube();

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SLOW, 30, 410, 37, 20, 70, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 50, 50);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SLOW, 121, 0, 61, 17, 33, 16);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 15, 15);

    moveBCMainC(310, 30, 67);
    moveBC(337, -67, -30);
    moveBC(150, -20, 20);
    stopBC();

    putCubeOnShip();

    startTimeD(350, 100, 2);
    moveBC3Parts(88, 242, 110, 60, 20, 90, 30, 60, 20);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 410, 301, 17, 90, 13);
    setNewMotBCPowersAndRatio(-13, 13);
    waitForCubes();
    moveBC(55, 18, -18);

    takeAnyLeftCube();

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SLOW, 30, 410, 37, 20, 60, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 50, 50);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 180, 0, 90, 17, 39, 16);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 15, 15);

    moveBCMainC(310, 30, 67);
    moveBC(337, -67, -30);
    moveBC(150, -20, 20);
    stopBC();

    putCubeOnShip();
    moveBC(60, 20, -20);

    startTimeD(250, 100, 2);
    moveBCMainC(240, 5, 70);
    stratManipD(&MANIP_D_PID_SETTINGS, -79, 1, -30);
    moveBC(240, -70, -5);
    line2SenDist(&DEFAULT_LINE_PID_SLOW, 90, 20);
    setNewMotBCPowersAndRatio(-20, 20);
    startTimeD(400, -70, -50);
    sleep(400);
}

void shipsAndFinish()
{
    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 100, 350, 100, 25, 75, 21);
    line2SenCrawl(&DEFAULT_LINE_PID_MEDIUM, 21, 150, 150);
    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 100, 1100, 300, 40, 100, 30);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 30, 150, 150);

    moveBC3Parts(50, 0, 35, -20, 20, -40, 40, -15, 15);
    moveBC3Parts(50, 0, 35, 20, -20, 40, -40, 15, -15);

    stopB();

    turnCDegr(18, 49.5, 22.5, -23, -55, -23);
    moveBC3Parts(30, 152, 38, -20, 20, -60, 60, -17, 17);
    stopBC();
    startTimeD(300, 100, 20);
    sleep(300);
    moveBC3Parts(30, 152, 38, 20, -20, 60, -60, 17, -17);
    turnCDegr(18, 49.5, 22.5, 20, 90, 17);

    moveBC3Parts(30, 70, 38, -20, 20, -40, 40, -20, 20);
    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 60, 60, 17, 17);

    line2SenAccelPart(&DEFAULT_LINE_PID_MEDIUM, 100, 20, 80);
    line2SenDist(&DEFAULT_LINE_PID_MEDIUM, 375, 80);
    moveBCAccelPartMainB(380, -80, 80, -13, 13);
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

    // line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 50, 50);

    // moveBC3PartsMainC(334, 0, 156, 10, 60, 15, 90, 10, 60);
    // stopBC();

    // line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 50, 50);
    //  debReadAndShowRGB(sen1, &SEN1_CALIBRATION);
    //   displayMeanCalibratedRGB(sen2, &SEN2_CALIBRATION);

    // displayCalibrationValues();
    //     bool *flag = stratManipD(&MANIP_D_PID_SETTINGS, 150, 2, 10);
    //     while (*flag)
    //         ;

    // debReadAndShowHSV(sen3, &SEN3_CALIBRATION);

    // startTimeD(150, 100, 2);
    // sleep(1000);
    // resetMotorEncoder(motD);
    // startTimeD(500, -50, -20);
    // sleep(5000);
    // stratManipD(&MANIP_D_PID_SETTINGS, 160, 2, 0);
    // sleep(300);
    // moveBC(100, 20, -20);
    // startTimeD(500, -30, -10);
    // moveBC(100, 20, -20);
    // stopBC();

    // takeCube1();
    //  startTimeD(3000, -50, -50);
    //  sleep(5000);
    //  startTimeD(3000, 70, 70);

    // line2SenCrawl(&DEFAULT_LINE_PID_SUPRA, 85, 200, 200);
    // stopBC();

    //  displayCalibrationValues();
}