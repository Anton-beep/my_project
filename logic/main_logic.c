#include "include/PID_c_motors.c"
#include "include/motion.c"
#include "include/sen_for_colors.c"
#include "include/motion_with_sensors.c"
#include "include/PID_c_manip.c"

void testFunc()
{
    takeCube1();
    // startTimeD(3000, -50, -50);
    // sleep(5000);
    // startTimeD(3000, 70, 70);

    // line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 9999, 200);
    // stopBC();

    // displayMeanCalibratedRGB(sen3, &SEN3_CALIBRATION_CUBES);
    //  displayCalibrationValues();
}

// ONLY int *______________________________________________________________
int IN_PTRS_FIRST_SCAN[3];
int IN_PTRS_CUBES[3];

// 0 - green, 1 - blue, -1 - no cube
int CUBES_COLORS[4] = {0, 1, 0, 1};

// ONLY int________________________________________________________________
int DEFAULT_BLACK_LINE = 5;

// 0 - green, 1 - blue
int FIRST_CUBE_TO_TAKE = 0;
int SECOND_CUBE_TO_TAKE = 1;

// ONLY SenRGBVals_________________________________________________________
SenRGBVals GREEN_CUBE_FIRST_SCAN;
SenRGBVals BLUE_CUBE_FIRST_SCAN;
SenRGBVals NOTHING_FIRST_SCAN;
SenRGBVals GREEN_CUBE_CUBES;
SenRGBVals BLUE_CUBE_CUBES;
SenRGBVals NOTHING_CUBES;

void defineForLogic()
{
    // ONLY SenRGBVals________________________________________________________
    GREEN_CUBE_FIRST_SCAN.R = 10;
    GREEN_CUBE_FIRST_SCAN.G = 97;
    GREEN_CUBE_FIRST_SCAN.B = 25;

    BLUE_CUBE_FIRST_SCAN.R = 30;
    BLUE_CUBE_FIRST_SCAN.G = 72;
    BLUE_CUBE_FIRST_SCAN.B = 110;

    NOTHING_FIRST_SCAN.R = 0;
    NOTHING_FIRST_SCAN.G = 0;
    NOTHING_FIRST_SCAN.B = 0;

    GREEN_CUBE_CUBES.R = 22;
    GREEN_CUBE_CUBES.G = 96;
    GREEN_CUBE_CUBES.B = 32;

    BLUE_CUBE_CUBES.R = 21;
    BLUE_CUBE_CUBES.G = 59;
    BLUE_CUBE_CUBES.B = 134;

    NOTHING_CUBES.R = 0;
    NOTHING_CUBES.G = 0;
    NOTHING_CUBES.B = 0;

    // ONLY int *_____________________________________________________________

    IN_PTRS_FIRST_SCAN[0] = (int)&GREEN_CUBE_FIRST_SCAN;
    IN_PTRS_FIRST_SCAN[1] = (int)&BLUE_CUBE_FIRST_SCAN;
    IN_PTRS_FIRST_SCAN[2] = (int)&NOTHING_FIRST_SCAN;

    IN_PTRS_CUBES[0] = (int)&GREEN_CUBE_CUBES;
    IN_PTRS_CUBES[1] = (int)&BLUE_CUBE_CUBES;
    IN_PTRS_CUBES[2] = (int)&NOTHING_CUBES;
}

void startReadPullShip()
{
    // set manip
    startTimeD(150, 100, 2);
    moveBCAccelPartMainB(200, -20, 20, -70, 70);
    int *firstScanOut = startReadRowOfObjectsRGB(IN_PTRS_FIRST_SCAN, 3, sen3, &SEN3_CALIBRATION);
    line2SenAccelPart(&DEFAULT_LINE_PID_MEDIUM, 170, 70, 30);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 30, 9999, 300);
    moveBCAccelPartMainB(160, -30, 30, -70, 70);
    moveBCAccelPartMainB(190, -70, 70, -15, 15);
    stopBC();
    stopTask(readRowOfObjectsRGB);
    moveBCAccelPartMainB(210, 60, -60, -20, 20);
    tankTurnNS3Parts(15, 25, 30, -20, -20, -60, -60, -30, -30);
    tankTurnSenCrawl(-30, -30, -10, -10, 190, 190);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 70, 100, 130, 20, 60, 30);
    stratManipD(&MANIP_D_PID_SETTINGS, -65, 2);
    moveBCAccelPartMainC(1590, -23, 92, -20, 80);
    moveBC(60, -15, 15);
    stopBC();
    startTimeD(250, -50, -40);
    sleep(400);

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
    //  for (int i = 0; firstScanOut[i] != NULL; i++)
    //  {
    //      if (firstScanOut[i] == &GREEN_CUBE_FIRST_SCAN)
    //      {
    //          displayCenteredTextLine(i * 2, "GREEN");
    //      }
    //      else if (firstScanOut[i] == &BLUE_CUBE_FIRST_SCAN)
    //      {
    //          displayCenteredTextLine(i * 2, "BLUE");
    //      }
    //      else if (firstScanOut[i] == &NOTHING_FIRST_SCAN)
    //      {
    //          displayCenteredTextLine(i * 2, "NOTHING");
    //      }
    //  }
    //  flushButtonMessages();
    //  waitForButtonPress();

    moveBC3Parts(160, 500, 200, 20, -20, 90, -90, 30, -30);
    waitForLine(150, 150);

    moveBCAccelPartMainC(100, 20, 60, 30, 90);
    moveBCMainC(200, 30, 90);
    moveBCAccelPartMainC(150, 30, 90, 20, 60);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 30, 120, 50, 20, 80, 20);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 50, 50);

    moveBCAccelPartMainC(100, 20, 60, 30, 90);
    moveBCMainC(200, 30, 90);
    moveBCAccelPartMainC(150, 30, 90, 20, 60);
}

void conCrane()
{
    line2Sen3Parts(&DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_FAST, 100, 2000, 200, 40, 95, 30);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 50, 50);

    // moveBCAccelPartMainB(100, -60, -20, -90, -30);
    // moveBC(200, -90, -30);
    // moveBCAccelPartMainB(150, -90, -30, -60, -20);

    moveBC3Parts(30, 50, 38, -20, 20, -40, 40, -20, 20);
    tankTurnNS3Parts(15, 41.25, 18.75, -20, -20, -60, -60, -30, -30);
    tankTurnSenCrawl(-30, -30, -20, -20, 190, 190);

    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_MEDIUM, 106, 291, 133, 20, 60, 15);
    moveBC3Parts(106, 275, 133, 15, -15, 60, -60, 20, -20);

    tankTurnNS3Parts(15, 41, 18.75, -20, -20, -60, -60, -30, -30);
    tankTurnSenCrawl(-30, -30, -20, -20, 190, 190);

    line2Sen3Parts(&DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_SUPRA, &DEFAULT_LINE_PID_FAST, 100, 2050, 200, 40, 95, 30);
}

void goToCubes()
{
    // не трогать блять
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 15, 50, 50);

    moveBCAccelPartMainB(118, -15, 15, -9, 9);
    tankTurnNS3Parts(15, 41, 18.75, 10, 10, 50, 50, 20, 20);
    tankTurnSenCrawl(20, 20, 10, 10, 190, 190);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 20, 180, 65, 10, 20, 0);
    stopBC();

    startTimeD(250, 100, 2);
    sleep(100);

    moveBC3Parts(16, 44, 20, 20, -20, 25, -25, 9, -9);
    tankTurnNS3Parts(33, 90, 42, 20, 20, 50, 50, 30, 30);
    tankTurnSenCrawl(30, 30, 10, 10, 190, 190);

    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 17, 50, 50);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 60, 523, 75, 17, 30, 10);
    stopC();
    turnBDegr(18, 49.5, 22.5, -10, -20, -17);
    stopBC();
    sleep(100);
}

void readCubes()
{
    int *cubesScan = startReadRowOfObjectsRGB(IN_PTRS_CUBES, 3, sen3, &SEN3_CALIBRATION_CUBES);
    moveBC3Parts(112, 338, 140, 12, -12, 13, -13, 12, -12);
    stopBC();
    stopTask(readRowOfObjectsRGB);

    for (int i = 0, j = 0; cubesScan[i] != NULL; i++)
    {
        if (cubesScan[i] == &GREEN_CUBE_CUBES)
        {
            CUBES_COLORS[j] = 0;
            j++;
        }
        else if (cubesScan[i] == &BLUE_CUBE_CUBES)
        {
            CUBES_COLORS[j] = 1;
            j++;
        }
    }
    // eraseDisplay();
    // for (int i = 0; cubesScan[i] != NULL; i++)
    // {
    //     if (cubesScan[i] == &GREEN_CUBE_CUBES)
    //     {
    //         displayCenteredTextLine(i * 2, "GREEN");
    //     }
    //     else if (cubesScan[i] == &BLUE_CUBE_CUBES)
    //     {
    //         displayCenteredTextLine(i * 2, "BLUE");
    //     }
    //     else if (cubesScan[i] == &NOTHING_CUBES)
    //     {
    //         displayCenteredTextLine(i * 2, "NOTHING");
    //     }
    //     else
    //     {
    //         displayCenteredTextLine(i * 2, "ERROR");
    //     }
    // }

    // flushButtonMessages();
    // waitForButtonPress();
}

// left to right
void takeCube1()
{
    tankTurnNS3Parts(8.756, 24.08, 10.945, 20, 20, 25, 25, 7, 7);
    moveBC3Parts(67, 175, 83, -20, 20, -30, 30, -7, 7);
    tankTurnNS3Parts(8.756, 24.08, 10.945, -20, -20, -25, -25, -7, -7);
    moveBC(160, -18, 18);
    stopBC();
    startTimeD(400, -60, -40);
    sleep(300);

    moveBC(160, 18, -18);
    tankTurnNS3Parts(8.756, 24.08, 10.945, 20, 20, 25, 25, 7, 7);
    moveBC3Parts(67, 175, 83, 20, -20, 50, -50, 7, -7);
    tankTurnNS3Parts(8.756, 24.08, 10.945, -20, -20, -35, -35, -7, -7);
    stopBC();
}

void takeCube2()
{
    tankTurnNS3Parts(4, 11, 5.28, 20, 20, 25, 25, 7, 7);
    moveBC3Parts(49, 124, 62, -20, 20, -30, 30, -7, 7);
    tankTurnNS3Parts(4, 11, 11, -20, -20, -25, -25, -7, -7);
    moveBC(160, -18, 18);
    stopBC();
    startTimeD(400, -60, -40);
    sleep(300);

    moveBC(160, 18, -18);
    tankTurnNS3Parts(4, 11, 11, 20, 20, 35, 35, 7, 7);
    moveBC3Parts(49, 124, 62, 20, -20, 50, -50, 7, -7);
    tankTurnNS3Parts(4, 11, 5.28, -20, -20, -25, -25, -7, -7);
    stopBC();
}

void takeCube3()
{
    tankTurnNS3Parts(4, 11, 5.28, -20, -20, -25, -25, -7, -7);
    moveBC3Parts(49, 124, 62, -20, 20, -30, 30, -7, 7);
    tankTurnNS3Parts(4, 10, 4, 20, 20, 25, 25, 7, 7);
    stopBC();
    sleep(5000);
    moveBC(160, -18, 18);
    stopBC();
    startTimeD(400, -60, -40);
    sleep(300);

    moveBC(160, 18, -18);
    tankTurnNS3Parts(4, 10, 4, -20, -20, -35, -35, -7, -7);
    moveBC3Parts(49, 124, 62, 20, -20, 50, -50, 7, -7);
    tankTurnNS3Parts(4, 11, 5.28, 20, 20, 25, 25, 7, 7);
    stopBC();
}

void takeCube4()
{
    tankTurnNS3Parts(8.756, 24.08, 10.945, -20, -20, -25, -25, -5, -5);
    moveBC3Parts(67, 225, 83, -20, 20, -30, 30, -5, 5);
    tankTurnNS3Parts(8.756, 26, 10.945, 20, 20, 25, 25, 5, 5);
    moveBC(140, -18, 18);
    stopBC();
    startTimeD(400, -60, -40);
    sleep(300);

    moveBC(140, 18, -18);
    tankTurnNS3Parts(8.756, 24.08, 10.945, -20, -20, -25, -25, -5, -5);
    moveBC3Parts(67, 225, 83, 20, -20, 50, -50, 5, -5);
    tankTurnNS3Parts(8.756, 26, 10.945, 20, 20, 35, 35, 5, 5);
    stopBC();
}

void putCubes()
{
    tankTurnNS3Parts(8, 22, 10, -20, -20, -50, -50, -5, -5);
    moveBC3Parts(128, 265, 160, -20, 20, -50, 50, -5, 5);
    tankTurnNS3Parts(26, 71.5, 32.5, 20, 20, 50, 50, 5, 5);
    stopBC();
    stratManipD(&MANIP_D_PID_SETTINGS, -98, 2);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 20, 55, 25, 20, 30, 5);
    stopBC();

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

    tankTurnNS3Parts(33, 90.75, 41.25, -20, -20, -70, -70, -20, -20);
    tankTurnSenCrawl(-15, -15, -10, -10);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 17, 50, 50);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 20, 55, 25, 20, 35, 5);
    stopC();
    turnBDegr(6, 16.5, 7.5, -20, -40, -10);
    stopB();

    sleep(5000);

    startTimeD(300, 100, 2);
    sleep(300);
    turnBDegr(6, 16.5, 7.5, 20, 40, 10);

    moveBC3Parts(16, 44, 20, 20, -20, 30, -30, 8, -8);
    tankTurnNS3Parts(33, 90.75, 41.25, -20, -20, -70, -70, -20, -20);
    tankTurnSenCrawl(-15, -15, -10, -10);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 17, 50, 50);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 60, 165, 75, 20, 45, 7);
    stopBC();

    sleep(5000);

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

    tankTurnNS3Parts(33, 90.75, 41.25, -20, -20, -70, -70, -20, -20);
    tankTurnSenCrawl(-15, -15, -10, -10);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 17, 50, 50);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 20, 85, 25, 20, 35, 5);
    stopBC();

    sleep(5000);

    startTimeD(300, 100, 20);
    sleep(300);

    moveBC3Parts(16, 44, 20, 20, -20, 30, -30, 8, -8);
    stratManipD(&MANIP_D_PID_SETTINGS, -65, 2);
    moveBC3Parts(16, 50, 20, -20, 20, -30, 30, -8, 8);
    stopBC();
    startTimeD(250, -50, -40);
    sleep(400);
    moveBC3Parts(30, 82, 38, 20, -20, 35, -35, 10, -10);
    stopBC();

    sleep(5000);
}

void whiteCubesOnBigShip()
{
    tankTurnNS3Parts(13, 35.75, 16.25, -20, -20, -50, -50, -20, -20);
    tankTurnSenCrawl(-17, -17, -15, -15);

    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 188, 517, 235, 20, 85, 15);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 15, 50, 50);

    moveBC3Parts(30, 50, 38, -20, 20, -40, 40, -20, 20);
    tankTurnNS3Parts(14, 38.5, 17.5, -20, -20, -60, -60, -20, -20);
    tankTurnSenCrawl(-20, -20, -15, -15);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 20, 55, 25, 20, 40, 15);
    startTimeD(300, 100, 20);
    sleep(300);

    moveBC3Parts(16, 44, 20, 20, -20, 25, -25, 9, -9);
    tankTurnNS3Parts(33, 90, 42, 20, 20, 50, 50, 30, 30);
    tankTurnSenCrawl(30, 30, 10, 10, 190, 190);
    stopBC();
    sleep(5000);

    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 17, 50, 50);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 60, 523, 75, 17, 30, 10);

    stopBC();
    sleep(5000);

    takeCube3();
    tankTurnNS3Parts(33, 90.75, 41.25, -20, -20, -70, -70, -20, -20);
    tankTurnSenCrawl(-15, -15, -10, -10);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 17, 50, 50);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 12, 33, 15, 20, 30, 5);
    stopB();
    turnCDegr(6, 16.5, 7.5, 20, 40, 10);
    stopC();

    sleep(5000);
    startTimeD(300, 100, 2);
    sleep(300);
    turnCDegr(6, 16.5, 7.5, -20, -40, -10);
    moveBC3Parts(16, 44, 20, 20, -20, 30, -30, 8, -8);
    stratManipD(&MANIP_D_PID_SETTINGS, -65, 2);
    moveBC3Parts(16, 50, 20, -20, 20, -30, 30, -8, 8);
    stopBC();
    startTimeD(250, -50, -40);
    sleep(400);

    moveBC3Parts(30, 82, 38, 20, -20, 35, -35, 10, -10);
    stopBC();
    tankTurnNS3Parts(13, 35.75, 16.25, -20, -20, -50, -50, -20, -20);
    tankTurnSenCrawl(-17, -17, -15, -15);

    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 20, 55, 25, 25, 50, 15);
    stopBC();
    startTimeD(300, 100, 2);
    sleep(300);

    moveBC3Parts(16, 44, 20, 20, -20, 32, -32, 8, -8);
    tankTurnNS3Parts(33, 90.75, 41.25, -20, -20, -70, -70, -20, -20);
    tankTurnSenCrawl(-20, -20, -15, -15);
    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 188, 517, 235, 20, 85, 15);
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
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 15, 50, 50);

    moveBC3Parts(30, 50, 38, -20, 20, -40, 40, -20, 20);
    tankTurnNS3Parts(14, 38.5, 17.5, 20, 20, 60, 60, 20, 20);
    tankTurnSenCrawl(20, 20, 15, 15);

    stratManipD(&MANIP_D_PID_SETTINGS, -70, 2);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 60, 165, 75, 20, 60, 15);
    stopBC();

    sleep(5000);

    startTimeD(250, -50, -40);
    sleep(400);

    moveBC3Parts(30, 82, 38, 20, -20, 35, -35, 10, -10);
    stopBC();
    startTimeD(300, 100, 2);
    sleep(300);
}

void putCubesOnSmallShip()
{
    moveBC3Parts(16, 44, 20, 20, -20, 32, -32, 8, -8);
    tankTurnNS3Parts(33, 90.75, 41.25, -20, -20, -70, -70, -20, -20);
    tankTurnSenCrawl(-20, -20, -15, -15);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 15, 50, 50);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 60, 165, 75, 20, 45, 7);
    stopBC();

    sleep(5000);

    takeAnyLeftCube();

    tankTurnNS3Parts(33, 90.75, 41.25, -20, -20, -70, -70, -20, -20);
    tankTurnSenCrawl(-15, -15, -10, -10);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 17, 50, 50);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 20, 55, 25, 20, 35, 5);
    stopC();
    turnBDegr(6, 16.5, 7.5, -20, -40, -10);
    stopB();

    sleep(5000);

    startTimeD(300, 100, 2);
    sleep(300);
    turnBDegr(6, 16.5, 7.5, 20, 40, 10);

    moveBC3Parts(16, 44, 20, 20, -20, 30, -30, 8, -8);
    tankTurnNS3Parts(33, 90.75, 41.25, -20, -20, -70, -70, -20, -20);
    tankTurnSenCrawl(-15, -15, -10, -10);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 17, 50, 50);

    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 60, 165, 75, 20, 45, 7);
    stopBC();

    sleep(5000);

    takeAnyLeftCube();

    tankTurnNS3Parts(33, 90.75, 41.25, -20, -20, -70, -70, -20, -20);
    tankTurnSenCrawl(-15, -15, -10, -10);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 17, 50, 50);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 20, 55, 25, 20, 35, 5);
    stopC();
    turnCDegr(6, 16.5, 7.5, 20, 40, 10);
    stopB();

    sleep(5000);

    startTimeD(300, 100, 2);
    sleep(300);
    turnCDegr(6, 16.5, 7.5, -20, -40, -10);

    moveBC3Parts(16, 44, 20, 20, -20, 30, -30, 8, -8);
    stratManipD(&MANIP_D_PID_SETTINGS, -65, 2);
    moveBC3Parts(16, 50, 20, -20, 20, -30, 30, -8, 8);

    stopBC();
    startTimeD(250, -50, -40);
    sleep(400);
    moveBC3Parts(30, 82, 38, 20, -20, 35, -35, 10, -10);
    stopBC();
}

void shipsAndFinish()
{
    tankTurnNS3Parts(13, 35.75, 16.25, -20, -20, -50, -50, -20, -20);
    tankTurnSenCrawl(-17, -17, -15, -15);

    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 188, 517, 235, 20, 85, 20);
    line2SenDist(&DEFAULT_LINE_PID_MEDIUM, 100, 20);
    line2Sen3Parts(&DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_FAST, &DEFAULT_LINE_PID_MEDIUM, 252, 693, 315, 35, 95, 35);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 30, 50, 50);

    moveBCAccelPartMainB(300, -30, 30, -20, 20);
    startTimeD(300, 100, 20);
    sleep(300);
    moveBC3Parts(60, 165, 75, 20, -20, 60, -60, 15, -15);
    tankTurnNS3Parts(13, 35.75, 16.25, 20, 20, 60, 60, 20, 20);
    tankTurnSenCrawl(20, 20, 15, 15, 150, 150);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 70, 192, 88, 20, 60, 7);
    line2SenAccelPart(&DEFAULT_LINE_PID_SLOW, 100, 20, 60);
    line2SenDist(&DEFAULT_LINE_PID_MEDIUM, 275, 60);
    moveBCAccelPartMainB(125, -60, 60, -13, 13);
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
}