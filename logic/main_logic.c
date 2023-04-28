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
    moveBCAccelPartMainB(150, -20, 20, -70, 70);
    int *firstScanOut = startReadRowOfObjectsRGB(IN_PTRS_FIRST_SCAN, 3, sen3, &SEN3_CALIBRATION);
    line2SenAccelPart(&DEFAULT_LINE_PID_MEDIUM, 180, 70, 30);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 30, 9999, 395);
    moveBCAccelPartMainB(160, -30, 30, -70, 70);
    moveBCAccelPartMainB(190, -70, 70, -15, 15);
    resetMotorEncoder(motD);
    stopBC();
    stopTask(readRowOfObjectsRGB);
    moveBCAccelPartMainB(215, 60, -60, -20, 20);
    tankTurnNS3Parts(30, 30, 30, -20, -20, -60, -60, -17, -17);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 70, 150, 130, 20, 60, 30);
    stratManipD(&MANIP_D_PID_SETTINGS, -64, 2, -30);
    moveBCMainC(1580, -22, 80);
    stopTask(manipMoveD);
    moveBC(100, -15, 15);
    startTimeD(500, -60, -67);
    moveBC(50, -14, 14);
    stopBC();
    sleep(300);

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

#if WRITE_DATA_IN_FILE == 1
    stopBC();
    fileWriteChar(DEBUG_FILE_HND, FIRST_CUBE_TO_TAKE);
    fileWriteChar(DEBUG_FILE_HND, SECOND_CUBE_TO_TAKE);
#endif

    moveBC3Parts(190, 294, 377, 23, -23, 100, -100, 21, -21);
    waitForLine(150, 150);

    moveBCMainC(500, 16, 60);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 100, 0, 190, 23, 90, 21);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 50, 50);

    moveBC3PartsMainC(334, 0, 156, 10, 60, 15, 90, 10, 60);
}

void conCrane()
{
    line2Sen3Parts(&DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_FAST, 189, 1714, 477, 23, 100, 21);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 50, 50);

    moveBC3Parts(334, 0, 156, -60, -10, -90, -15, -60, -10);

    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 106, 291, 133, 20, 70, 20);
    moveBC3Parts(106, 360, 133, 19, -19, 80, -80, 20, -20);

    tankTurnNS3Parts(18, 49.5, 22.5, -20, -20, -60, -60, -17, -17);

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

    moveBC3Parts(65, 105, 64, 20, -20, 35, -35, 17, 17);
    tankTurnNS3Parts(18, 49.5, 22.5, 19, 19, 90, 90, 17, 17);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 380, 300, 17, 98, 8);
    setNewMotBCPowersAndRatio(-8, 8);
    waitForCubes();
    moveBC(11, 18, -18);
}

void readCubes()
{
    int cubesScan[4];
    moveBC3PartsMainC(27, 76, 34, -6, -22, -12, -44, -6, -22);
    cubesScan[0] = readWindowRGB(sen3, &SEN3_CALIBRATION_1CUBE, IN_PTRS_1CUBE, 3);

    // POS TO CALIBRATE 1 AND 2 CUBES
    //
    // stopBC();
    // displayCalibrationValues();
    // displayMeanCalibratedRGB(sen3, &SEN3_CALIBRATION_1CUBE);

    moveBC(30, 13, -13);
    moveBC3PartsMainC(36, 99, 45, -5, -15, -15, -45, -5, -15);
    cubesScan[1] = readWindowRGB(sen3, &SEN3_CALIBRATION_1CUBE, IN_PTRS_1CUBE, 3);

    moveBC(130, -45, 7);
    cubesScan[2] = readWindowRGB(sen3, &SEN3_CALIBRATION_CUBES, IN_PTRS_CUBES, 3);

    // POS TO CALIBRATE 3 AND 4 CUBES
    //
    // stopBC();
    // displayCalibrationValues();
    // displayMeanCalibratedRGB(sen3, &SEN3_CALIBRATION_CUBES);

    moveBC3Parts(27, 45, 34, -9, 6, -45, 30, -9, 6);
    stopBC();

    cubesScan[3] = readWindowRGB(sen3, &SEN3_CALIBRATION_CUBES, IN_PTRS_CUBES, 3);

    for (int i = 0, j = 0; cubesScan[i] != NULL; i++)
    {
        if (cubesScan[i] == &GREEN_CUBE_CUBES || cubesScan[i] == &ANOTHER_GREEN_CUBE_CUBES)
        {
            CUBES_COLORS[j] = 0;
            j++;
        }
        else if (cubesScan[i] == &BLUE_CUBE_CUBES || cubesScan[i] == &ANOTHER_BLUE_CUBE_CUBES)
        {
            CUBES_COLORS[j] = 1;
            j++;
        }
    }
    eraseDisplay();
    for (int i = 0; cubesScan[i] != NULL; i++)
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
    moveBC(135, 20, -20);
    moveBC(350, -60, -54);

    stopBC();

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 125, 55, 125, 20, 55, 17);

    tankTurnNS3Parts(36, 99, 45, 30, 30, 90, 90, 17, 17);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 25, 180, 125, 20, 55, 8);
    setNewMotBCPowersAndRatio(-8, 8);
    waitForCubes();
    stopBC();
    sleep(300);
    moveBC(39, 18, -18);
    stopBC();
    sleep(300);
}

// left to right
void takeCube1()
{
    moveBC(30, 20, -20);
    moveBC3PartsMainC(100, 300, 125, -15, -45, -18, -54, -15, -45);
    moveBC3Parts(100, 290, 125, 45, 15, 54, 18, 45, 15);
    stratManipD(&MANIP_D_PID_SETTINGS, -86, 1, -20);
    moveBC3Parts(40, 136, 50, -20, 20, -40, 40, -15, 15);
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
    moveBC(30, 20, -20);
    moveBC3PartsMainC(54, 148, 68, -10, -40, -11, -44, -10, -40);
    moveBC3Parts(54, 137, 68, 40, 10, 44, 11, 40, 10);
    stratManipD(&MANIP_D_PID_SETTINGS, -86, 1, -20);
    moveBC3Parts(40, 116, 50, -20, 20, -40, 40, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    stopTask(manipMoveD);
    startTimeD(400, -55, -50);
    sleep(450);

    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBC3Parts(189, 19, 477, 70, 45, 98, 63, 70, 45);
}

void takeCube3()
{
    moveBC(30, 20, -20);
    moveBC3Parts(54, 102, 68, 40, 10, 44, 11, 40, 10);
    moveBC3PartsMainC(54, 105, 68, -10, -40, -11, -44, -10, -40);
    stratManipD(&MANIP_D_PID_SETTINGS, -88, 1, -20);
    moveBC3Parts(40, 116, 50, -20, 20, -40, 40, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    stopTask(manipMoveD);
    startTimeD(400, -55, -50);
    sleep(450);

    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBC3PartsMainC(189, 19, 477, -45, -70, -63, -98, -45, -70);
}

void takeCube4()
{
    moveBC(30, 20, -20);
    moveBC3Parts(100, 255, 125, 45, 15, 72, 24, 45, 15);
    moveBC3PartsMainC(100, 255, 125, -15, -45, -24, -72, -15, -45);
    stratManipD(&MANIP_D_PID_SETTINGS, -81, 1, -20);
    moveBC3Parts(40, 136, 50, -20, 20, -40, 40, -15, 15);
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
    stratManipD(&MANIP_D_PID_SETTINGS, 80, 1, 20);
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
    if (FIRST_CUBE_TO_TAKE == CUBES_COLORS[0])
    {
        takeCube1();
        CUBES_COLORS[0] = -1;
    }
    else if (FIRST_CUBE_TO_TAKE == CUBES_COLORS[3])
    {
        takeCube4();
        CUBES_COLORS[3] = -1;
    }
    else if (FIRST_CUBE_TO_TAKE == CUBES_COLORS[1])
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

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 60, 253, 237, 20, 70, 18);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 50, 50);

    moveBC(45, -16, 16);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 13, 13);

    moveBCMainC(310, 30, 67);
    moveBC(337, -67, -30);
    moveBC(140, -20, 20);
    stopBC();
    putCubeOnShip();
    startTimeD(350, 60, 2);
    moveBC3Parts(88, 242, 110, 60, 20, 90, 30, 60, 20);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 310, 351, 17, 90, 8);
    setNewMotBCPowersAndRatio(-8, 8);
    waitForCubes();
    stopBC();
    sleep(300);
    moveBC(39, 18, -18);
    stopBC();
    sleep(300);


    if (SECOND_CUBE_TO_TAKE == CUBES_COLORS[0])
    {
        takeCube1();
        CUBES_COLORS[0] = -1;
    }
    else if (SECOND_CUBE_TO_TAKE == CUBES_COLORS[3])
    {
        takeCube4();
        CUBES_COLORS[3] = -1;
    }
    else if (SECOND_CUBE_TO_TAKE == CUBES_COLORS[1])
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

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 112, 2, 53, 17, 32, 16);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 15, 15);

    moveBCMainC(310, 30, 67);
    moveBC(330, -67, -30);
    moveBC(140, -20, 20);
    stopBC();
    putCubeOnShip();

    stopBC();
    startTimeD(190, 50, 0);
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
    moveB(50, -22);
    stopB();
    startTimeD(350, 100, 2);
    sleep(350);
    moveB(50, 22);

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
    startTimeD(400, -60, -50);
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

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 58, 159, 73, 20, 50, 17);

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
    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 60, 60, 17, 17);

    stratManipD(&MANIP_D_PID_SETTINGS, -64, 2, -30);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SLOW, 100, 205, 125, 20, 80, 19);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 200, 200);
    stopBC();

    startTimeD(250, -50, -40);
    sleep(400);

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
    moveBC(39, 18, -18);

    takeAnyLeftCube();

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SLOW, 30, 410, 37, 20, 70, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 50, 50);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SLOW, 121, 0, 57, 17, 33, 16);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 15, 15);

    moveBCMainC(310, 30, 67);
    moveBC(330, -67, -30);
    moveBC(150, -20, 20);
    stopBC();

    putCubeOnShip();

    startTimeD(350, 60, 2);
    moveBC3Parts(88, 242, 110, 60, 20, 90, 30, 60, 20);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 410, 301, 17, 90, 13);
    setNewMotBCPowersAndRatio(-13, 13);
    waitForCubes();
    moveBC(30, 18, -18);

    takeAnyLeftCube();

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SLOW, 30, 410, 37, 20, 60, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 50, 50);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 190, 0, 90, 17, 39, 16);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 15, 15);

    moveBCMainC(310, 30, 67);
    moveBC(330, -67, -30);
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
    startTimeD(400, -60, -50);
    sleep(200);
}

void shipsAndFinish()
{
    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 100, 410, 100, 25, 75, 21);
    line2SenCrawl(&DEFAULT_LINE_PID_MEDIUM, 21, 50, 50);
    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 100, 1100, 300, 40, 100, 30);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 30, 50, 50);

    moveBC3Parts(50, 0, 75, -20, 20, -60, 60, -15, 15);
    moveBC3Parts(50, 0, 75, 20, -20, 60, -60, 15, -15);

    stopB();

    turnCDegr(18, 49.5, 22.5, -20, -50, -17);
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
    moveBCAccelPartMainB(390, -80, 80, -13, 13);
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
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 50, 50);

    moveBC3PartsMainC(334, 0, 156, 10, 60, 15, 90, 10, 60);
    stopBC();
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