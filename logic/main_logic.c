#include "include/PID_c_motors.c"
#include "include/motion.c"
#include "include/sen_for_colors.c"
#include "include/motion_with_sensors.c"
#include "include/PID_c_manip.c"
#include "logic_vars.c"

void testFunc()
{
    displayMeanCalibratedRGB(sen1, &SEN1_CALIBRATION);

    // displayCalibrationValues();
    //   bool *flag = stratManipD(&MANIP_D_PID_SETTINGS, 150, 2, 10);
    //   while (*flag)
    //       ;

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

    // line2SenCrawl(&DEFAULT_LINE_PID_SUPRA, 95, 200, 200);
    // stopBC();

    //  displayCalibrationValues();
}

void startReadPullShip()
{
    // set manip
    startTimeD(350, 100, 2);
    moveBCAccelPartMainB(200, -20, 20, -70, 70);
    int *firstScanOut = startReadRowOfObjectsRGB(IN_PTRS_FIRST_SCAN, 3, sen3, &SEN3_CALIBRATION);
    line2SenAccelPart(&DEFAULT_LINE_PID_MEDIUM, 170, 70, 30);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 30, 9999, 300);
    moveBCAccelPartMainB(160, -30, 30, -70, 70);
    moveBCAccelPartMainB(190, -70, 70, -15, 15);
    resetMotorEncoder(motD);
    stopBC();
    stopTask(readRowOfObjectsRGB);
    moveBCAccelPartMainB(240, 60, -60, -20, 20);
    tankTurnNS3Parts(15, 25, 30, -20, -20, -60, -60, -17, -17);
    tankTurnSenCrawl(-30, -30, -10, -10, 190, 190);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 70, 150, 130, 20, 60, 30);
    stratManipD(&MANIP_D_PID_SETTINGS, -70, 2, -30);
    moveBCMainC(1555, -22, 80);
    moveBC(150, -12, 12);
    stopTask(manipMoveD);
    stopBC();
    startTimeD(500, -50, -85);
    sleep(700);

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
    // eraseDisplay();
    // for (int i = 0; firstScanOut[i] != NULL; i++)
    // {
    //     if (firstScanOut[i] == &GREEN_CUBE_FIRST_SCAN)
    //     {
    //         displayCenteredTextLine(i * 2, "GREEN");
    //     }
    //     else if (firstScanOut[i] == &BLUE_CUBE_FIRST_SCAN)
    //     {
    //         displayCenteredTextLine(i * 2, "BLUE");
    //     }
    //     else if (firstScanOut[i] == &NOTHING_FIRST_SCAN)
    //     {
    //         displayCenteredTextLine(i * 2, "NOTHING");
    //     }
    // }
    // flushButtonMessages();
    // waitForButtonPress();

    moveBC3Parts(160, 500, 200, 20, -20, 90, -90, 30, -30);
    waitForLine(150, 150);

    moveBCAccelPartMainC(100, 10, 30, 15, 45);
    moveBCMainC(200, 23, 69);
    moveBCAccelPartMainC(150, 23, 69, 20, 60);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 30, 120, 50, 20, 80, 20);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 50, 50);

    moveBCAccelPartMainC(100, 20, 60, 23, 69);
    moveBCMainC(200, 23, 69);
    moveBCAccelPartMainC(150, 23, 69, 20, 60);
}

void conCrane()
{
    line2Sen3Parts(&DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_FAST, 100, 2000, 200, 40, 95, 30);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 50, 50);

    // moveBCAccelPartMainB(100, -60, -20, -90, -30);
    // moveBC(200, -90, -30);
    // moveBCAccelPartMainB(150, -90, -30, -60, -20);

    moveBC3Parts(30, 50, 38, -20, 20, -40, 40, -20, 20);
    tankTurnNS3Parts(18, 49.5, 22.5, -20, -20, -60, -60, -17, -17);

    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_MEDIUM, 106, 291, 133, 20, 60, 15);
    moveBC3Parts(106, 275, 133, 15, -15, 60, -60, 20, -20);

    tankTurnNS3Parts(18, 49.5, 22.5, -20, -20, -60, -60, -17, -17);

    line2Sen3Parts(&DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_FAST, 100, 2050, 200, 40, 95, 30);
}

void goToCubes()
{
    startTimeD(250, -50, -40);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 80, 80);
    tankTurnNS3Parts(36, 69, 75, -30, -30, -50, -50, -17, -17);

    line2SenDist(&DEFAULT_LINE_PID_SLOW, 100, 20);
    stopBC();

    startTimeD(250, 100, 2);
    sleep(200);

    moveBC3Parts(35, 154, 44, 20, -20, 50, -50, 17, 17);
    tankTurnNS3Parts(18, 49.5, 22.5, 19, 19, 90, 90, 17, 17);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 410, 301, 17, 98, 15);
    setNewMotBCPowersAndRatio(-13, 13);
    waitForLine(560, 560);
}

void readCubes()
{
    int cubesScan[4];
    moveBC3PartsMainC(27, 76, 34, -6, -22, -12, -44, -6, -22);
    cubesScan[0] = readWindowRGB(sen3, &SEN3_CALIBRATION_1CUBE, IN_PTRS_1CUBE, 3);

    // stopBC();
    // displayCalibrationValues();
    // displayMeanCalibratedRGB(sen3, &SEN3_CALIBRATION_1CUBE);

    moveBC(30, 13, -13);
    moveBC3PartsMainC(36, 99, 45, -5, -15, -15, -45, -5, -15);
    cubesScan[1] = readWindowRGB(sen3, &SEN3_CALIBRATION_1CUBE, IN_PTRS_1CUBE, 3);

    moveBC(130, -45, 7);
    cubesScan[2] = readWindowRGB(sen3, &SEN3_CALIBRATION_CUBES, IN_PTRS_CUBES, 3);

    // stopBC();
    // displayCalibrationValues();
    // displayMeanCalibratedRGB(sen3, &SEN3_CALIBRATION_CUBES);

    moveBC3Parts(27, 30, 34, -9, 4, -45, 20, -9, 4);
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

    // flushButtonMessages();
    // waitForButtonPress();

    moveC(62, 30);
    moveBC(135, 20, -20);
    moveBC(300, -60, -55);
    stratManipD(&MANIP_D_PID_SETTINGS, -86, 1, -20);

    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SLOW, 106, 291, 133, 20, 60, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 150, 150);

    tankTurnNS3Parts(36, 99, 45, 30, 30, 90, 90, 17, 17);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 74, 203, 93, 20, 40, 17);
}

// left to right
void takeCube1()
{
    moveBC3PartsMainC(100, 300, 125, -15, -45, -30, -90, -15, -45);
    moveBC3Parts(100, 290, 125, 45, 15, 90, 30, 45, 15);
    stratManipD(&MANIP_D_PID_SETTINGS, -86, 1, -20);
    moveBC3Parts(40, 106, 50, -20, 20, -40, 40, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    stopTask(manipMoveD);
    startTimeD(400, -60, -50);
    sleep(400);

    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBC(900, 70, 13);

    // tankTurnNS3Parts(8.756, 26, 10.945, 20, 20, 25, 25, 7, 7);
    // moveBC3Parts(67, 175, 83, -20, 20, -30, 30, -7, 7);
    // tankTurnNS3Parts(8.756, 24.08, 10.945, -20, -20, -25, -25, -7, -7);
    // moveBC(160, -18, 18);
    // stopBC();
    // stopTask(manipMoveD);
    // startTimeD(400, -60, -50);
    // sleep(400);

    // moveBC(160, 18, -18);
    // tankTurnNS3Parts(8.756, 24.08, 10.945, 20, 20, 25, 25, 7, 7);
    // moveBC3Parts(67, 175, 83, 20, -20, 50, -50, 7, -7);
    // tankTurnNS3Parts(8.756, 26, 10.945, -20, -20, -35, -35, -7, -7);
    // stopBC();
}

void takeCube2()
{
    moveBC3PartsMainC(54, 148, 68, -10, -40, -15, -60, -10, -40);
    moveBC3Parts(54, 137, 68, 40, 10, 60, 15, 40, 10);
    stratManipD(&MANIP_D_PID_SETTINGS, -86, 1, -20);
    moveBC3Parts(40, 86, 50, -20, 20, -40, 40, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    stopTask(manipMoveD);
    startTimeD(400, -60, -50);
    sleep(400);

    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBC(650, 70, 45);

    // tankTurnNS3Parts(4, 11, 5.28, 20, 20, 25, 25, 7, 7);
    // moveBC3Parts(49, 124, 62, -20, 20, -30, 30, -7, 7);
    // tankTurnNS3Parts(4, 5, 11, -20, -20, -25, -25, -7, -7);
    // moveBC(160, -18, 18);
    // stopBC();
    // stopTask(manipMoveD);
    // startTimeD(400, -60, -50);
    // sleep(400);

    // moveBC(160, 18, -18);
    // tankTurnNS3Parts(4, 5, 11, 20, 20, 35, 35, 7, 7);
    // moveBC3Parts(49, 124, 62, 20, -20, 50, -50, 7, -7);
    // tankTurnNS3Parts(4, 11, 5.28, -20, -20, -25, -25, -7, -7);
    // stopBC();
}

void takeCube3()
{
    moveBC3Parts(54, 138, 68, 40, 10, 60, 15, 40, 10);
    moveBC3PartsMainC(54, 146, 68, -10, -40, -15, -60, -10, -40);
    stratManipD(&MANIP_D_PID_SETTINGS, -86, 1, -20);
    moveBC3Parts(40, 86, 50, -20, 20, -40, 40, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    stopTask(manipMoveD);
    startTimeD(400, -60, -50);
    sleep(400);

    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBCMainC(665, -45, -70);
}

void takeCube4()
{
    moveBC3Parts(100, 285, 125, 45, 15, 90, 30, 45, 15);
    moveBC3PartsMainC(100, 285, 125, -15, -45, -30, -90, -15, -45);
    stratManipD(&MANIP_D_PID_SETTINGS, -81, 1, -20);
    moveBC3Parts(40, 95, 50, -20, 20, -40, 40, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    stopTask(manipMoveD);
    startTimeD(400, -60, -50);
    sleep(400);

    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBCMainC(920, -13, -70);

    // tankTurnNS3Parts(8.756, 24.08, 10.945, -20, -20, -25, -25, -5, -5);
    // moveBC3Parts(67, 225, 83, -20, 20, -30, 30, -5, 5);
    // tankTurnNS3Parts(8.756, 26, 10.945, 20, 20, 25, 25, 5, 5);
    // moveBC(140, -18, 18);
    // stopBC();
    // stopTask(manipMoveD);
    // startTimeD(400, -60, -50);
    // sleep(400);

    // moveBC(140, 18, -18);
    // tankTurnNS3Parts(8.756, 24.08, 10.945, -20, -20, -25, -25, -5, -5);
    // moveBC3Parts(67, 225, 83, 20, -20, 50, -50, 5, -5);
    // tankTurnNS3Parts(8.756, 26, 10.945, 20, 20, 35, 35, 5, 5);
    // stopBC();
}

void putCubeOnShip()
{
    startTimeD(147, 65, 0);
    sleep(200);
    moveBC(100, 20, -20);
    startTimeD(500, -30, -10);
    moveBC(100, 20, -20);
    stopBC();
}

void putCubes()
{
    if (FIRST_CUBE_TO_TAKE == CUBES_COLORS[3])
    {
        takeCube1();
        CUBES_COLORS[3] = -1;
    }
    else if (FIRST_CUBE_TO_TAKE == CUBES_COLORS[2])
    {
        takeCube2();
        CUBES_COLORS[2] = -1;
    }
    else if (FIRST_CUBE_TO_TAKE == CUBES_COLORS[1])
    {
        takeCube3();
        CUBES_COLORS[1] = -1;
    }
    else if (FIRST_CUBE_TO_TAKE == CUBES_COLORS[0])
    {
        takeCube4();
        CUBES_COLORS[0] = -1;
    }

    stopBC();

    tankTurnNS3Parts(36, 99, 45, -20, -20, -90, -90, -17, -17);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 30, 83, 37, 20, 60, 18);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 50, 50);

    moveBC(30, -16, 16);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 13, 13);

    moveBCMainC(310, 30, 67);
    moveBC(330, -67, -30);
    moveBC(140, -20, 20);
    stopBC();
    putCubeOnShip();
    startTimeD(350, 60, 2);
    moveBC3Parts(88, 242, 110, 60, 20, 90, 30, 60, 20);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 270, 301, 17, 90, 8);
    setNewMotBCPowersAndRatio(-8, 8);
    waitForLine(515, 515);
    stopBC();
    sleep(200);

    moveBC3Parts(80, 199, 100, 17, -17, 30, -30, 15, -15);
    stratManipD(&MANIP_D_PID_SETTINGS, -71, 1, -20);

    if (SECOND_CUBE_TO_TAKE == CUBES_COLORS[3])
    {
        takeCube1();
        CUBES_COLORS[3] = -1;
    }
    else if (SECOND_CUBE_TO_TAKE == CUBES_COLORS[2])
    {
        takeCube2();
        CUBES_COLORS[2] = -1;
    }
    else if (SECOND_CUBE_TO_TAKE == CUBES_COLORS[1])
    {
        takeCube3();
        CUBES_COLORS[1] = -1;
    }
    else if (SECOND_CUBE_TO_TAKE == CUBES_COLORS[0])
    {
        takeCube4();
        CUBES_COLORS[0] = -1;
    }

    stopBC();

    tankTurnNS3Parts(36, 99, 45, -20, -20, -90, -90, -17, -17);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 30, 83, 37, 20, 60, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 50, 50);

    line2SenDist(&DEFAULT_LINE_PID_SLOW, 160, 18);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 15, 15);

    moveBCMainC(310, 30, 67);
    moveBC(330, -67, -30);
    moveBC(140, -20, 20);
    stopBC();
    putCubeOnShip();

    stratManipD(&MANIP_D_PID_SETTINGS, 120, 2, 30);
    moveBC(270, -70, -10);
    moveBCMainC(270, 10, 70);
}

void moveBigShipToWhite()
{
    startTimeD(350, 0, -60);
    line2SenDist(&DEFAULT_LINE_PID_SLOW, 150, 17);
    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 70, 242, 88, 17, 75, 15);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 15, 150, 150);

    moveBC3Parts(43, 119, 56, -25, 25, -50, 50, -19, 19);

    stopBC();
    startTimeD(350, 100, 2);
    sleep(350);

    moveBC3Parts(20, 55, 25, 18, -18, 30, -30, 14, -14);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 60, 60, 17, 17);
}

void take2WhiteCube()
{
    moveBC(260, 80, 10);
    stratManipD(&MANIP_D_PID_SETTINGS, -70, 1, -20);
    moveBCMainC(255, -10, -80);
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
    // возврат на линию после корбаля, скорее всего одной дугой и разворот на 180 и наезд на линию
}

void takeAnyLeftCube()
{
    if (CUBES_COLORS[3] != -1)
    {
        takeCube1();
        CUBES_COLORS[3] = -1;
    }
    else if (CUBES_COLORS[2] != -1)
    {
        takeCube2();
        CUBES_COLORS[2] = -1;
    }
    else if (CUBES_COLORS[1] != -1)
    {
        takeCube3();
        CUBES_COLORS[1] = -1;
    }
    else if (CUBES_COLORS[0] != -1)
    {
        takeCube4();
        CUBES_COLORS[0] = -1;
    }
}

void goToSmallShip()
{
    startTimeD(350, 100, 2);

    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 60, 165, 75, 25, 45, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 16, 150, 150);

    moveBC3Parts(30, 50, 38, -20, 20, -40, 40, -20, 20);
    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 60, 60, 17, 17);

    stratManipD(&MANIP_D_PID_SETTINGS, -64, 2, -30);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SLOW, 100, 205, 125, 20, 80, 19);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 200, 200);
    stopBC();

    startTimeD(250, -50, -40);
    sleep(400);

    moveBC3Parts(60, 365, 150, 20, -20, 60, -60, 17, -17);
    waitForLine(150, 150);
    moveBC3Parts(30, 30, 38, -20, 20, -40, 40, -20, 20);
    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 17, 17);

    line2SenDist(&DEFAULT_LINE_PID_SLOW, 230, 20);
    stopBC();

    startTimeD(250, 100, 2);
    sleep(200);
}

void putCubesOnSmallShip()
{
    moveBC3Parts(35, 125, 44, 20, -20, 50, -50, 17, 17);

    tankTurnNS3Parts(18, 49.5, 22.5, 19, 19, 90, 90, 17, 17);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 410, 301, 17, 98, 15);
    setNewMotBCPowersAndRatio(-13, 13);
    waitForLine(500, 500);
    stopBC();
    sleep(200);

    moveBC3Parts(80, 199, 100, 17, -17, 30, -30, 15, -15);
    stratManipD(&MANIP_D_PID_SETTINGS, -76, 1, -20);

    takeAnyLeftCube();

    stopBC();

    tankTurnNS3Parts(36, 99, 45, -20, -20, -90, -90, -17, -17);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 30, 83, 37, 20, 60, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 50, 50);

    line2SenDist(&DEFAULT_LINE_PID_SLOW, 165, 18);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 15, 15);

    moveBCMainC(310, 30, 67);
    moveBC(330, -67, -30);
    moveBC(150, -20, 20);
    stopBC();

    putCubeOnShip();

    startTimeD(350, 60, 2);
    moveBC3Parts(88, 242, 110, 60, 20, 90, 30, 60, 20);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_SLOW, 67, 270, 301, 17, 90, 8);
    setNewMotBCPowersAndRatio(-8, 8);
    waitForLine(515, 515);
    stopBC();
    sleep(200);

    moveBC3Parts(80, 199, 100, 17, -17, 30, -30, 15, -15);
    stratManipD(&MANIP_D_PID_SETTINGS, -74, 1, -20);

    takeAnyLeftCube();

    tankTurnNS3Parts(36, 99, 45, -20, -20, -90, -90, -17, -17);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 30, 83, 37, 20, 60, 17);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 50, 50);

    line2SenDist(&DEFAULT_LINE_PID_SLOW, 270, 18);

    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 50, 50, 15, 15);

    moveBCMainC(310, 30, 67);
    moveBC(330, -67, -30);
    moveBC(150, -20, 20);
    stopBC();

    putCubeOnShip();

    startTimeD(250, 100, 2);
    moveBCMainC(190, 5, 70);
    stratManipD(&MANIP_D_PID_SETTINGS, -80, 2, -30);
    moveBC(190, -70, -5);
    line2SenDist(&DEFAULT_LINE_PID_SLOW, 70, 20);
    stopBC();
    startTimeD(400, -60, -50);
    sleep(200);
    sleep(5000);
}

void shipsAndFinish()
{
    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 252, 693, 315, 35, 95, 35);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 30, 50, 50);

    moveBC3Parts(60, 5, 75, -20, 20, -60, 60, -15, 15);
    moveBC3Parts(60, 5, 75, 20, -20, 60, -60, 15, -15);

    stopB();

    turnCDegr(18, 49.5, 22.5, -20, -50, -17);
    moveBC3Parts(30, 152, 38, -20, 20, -60, 60, -17, 17);
    stopBC();
    startTimeD(300, 100, 20);
    sleep(300);
    moveBC3Parts(30, 152, 38, 20, -20, 60, -60, 17, -17);
    turnCDegr(18, 49.5, 22.5, 20, 50, 17);

    moveBC3Parts(30, 50, 38, -20, 20, -40, 40, -20, 20);
    tankTurnNS3Parts(18, 49.5, 22.5, 20, 20, 60, 60, 17, 17);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 70, 192, 88, 20, 60, 7);
    line2SenAccelPart(&DEFAULT_LINE_PID_MEDIUM, 100, 20, 60);
    line2SenDist(&DEFAULT_LINE_PID_MEDIUM, 275, 60);
    moveBCAccelPartMainB(110, -60, 60, -13, 13);
    stopBC();
}

void mainLogic()
{
    // manip work
    // startTimeD(800, -30, -19);
    // sleep(5000);
    // startTimeD(500, 20, 40);
    // sleep(1000);
    // startTimeD(2000, -30, -20);
    // sleep(5000);

    // startReadPullShip();
    // conCrane();
    goToCubes();
    readCubes();
    putCubes();
    // moveBigShipToWhite();
    // whiteCubeOnBig();
    // goToSmallShip();
    // putCubesOnSmallShip();
    // shipsAndFinish();
}