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
    setTimeManipD(350, 100, 1);

    sleep(600);
    moveBCAccelPartMainB(150, -20, 20, -35, 35);
    int *firstScanOut = startReadRowOfObjectsHSV(&NOTHING_FIRST_SCAN, 13, IN_PTRS_FIRST_SCAN, 2, sen3, &SEN3_CALIBRATION);
    line2SenAccelPart(&DEFAULT_LINE_PID_MEDIUM, 230, 35, 30);
    setNewMotBCPowersAndRatio(-30, 30);
    waitForSen2(395);
    moveBCAccelPartMainB(163, -30, 30, -50, 50);
    moveBCAccelPartMainB(193, -50, 50, -15, 15);
    stopBC();
    resetMotorEncoder(motD);
    stopTask(readRowOfObjectsHSV);
    sleep(100);
    moveBCSmartAccelExtreme(215, 22, -22, 22, -22);
    tankTurnNSSmartAccelExtreme(90, 22, 22);

    line2SenSmartAccelExtreme(940, 30, 20);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 20, 300, 300);
    moveBCSmartAccelExtreme(DEG_AFTER_LINE_FOR_TURN, -20, 20, -19, 19);
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
    tankTurnNSSmartAccel(90, -20, -20);

    line2SenSmartAccel(760, 20, 9, EXTREME_START_ACCEL);
    setNewMotBCPowersAndRatio(-9, 9);
    waitForCubes();
    moveBC(11, 18, -18);

    resetMotorEncoder(motD);
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

    // for (int i = 0; i < 4; i++)
    // {
    //     if (cubesScan[i] == &GREEN_CUBE_CUBES || cubesScan[i] == &ANOTHER_GREEN_CUBE_CUBES)
    //     {
    //         CUBES_COLORS[i] = 0;
    //     }
    //     else if (cubesScan[i] == &BLUE_CUBE_CUBES || cubesScan[i] == &ANOTHER_BLUE_CUBE_CUBES)
    //     {
    //         CUBES_COLORS[i] = 1;
    //     }
    //     else
    //     {
    //         CUBES_COLORS[i] = -2;
    //     }
    // }
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
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 25, 110, 125, 20, 55, 9);
    setNewMotBCPowersAndRatio(-9, 9);
    waitForCubes();
    moveBC(69, 18, -18);
}

void setDelivery()
{
    if (CUBES_COLORS[1] == FIRST_CUBE_TO_TAKE)
    {
        CUBES_DELIVERY[1] = 1;
    }
    else if (CUBES_COLORS[2] == FIRST_CUBE_TO_TAKE)
    {
        CUBES_DELIVERY[2] = 1;
    }
    else if (CUBES_COLORS[0] == FIRST_CUBE_TO_TAKE)
    {
        CUBES_DELIVERY[0] = 1;
    }
    else if (CUBES_COLORS[3] == FIRST_CUBE_TO_TAKE)
    {
        CUBES_DELIVERY[3] = 1;
    }

    if (CUBES_COLORS[1] == SECOND_CUBE_TO_TAKE && CUBES_DELIVERY[1] == -1)
    {
        CUBES_DELIVERY[1] = 1;
    }
    else if (CUBES_COLORS[2] == SECOND_CUBE_TO_TAKE && CUBES_DELIVERY[2] == -1)
    {
        CUBES_DELIVERY[2] = 1;
    }
    else if (CUBES_COLORS[0] == SECOND_CUBE_TO_TAKE && CUBES_DELIVERY[0] == -1)
    {
        CUBES_DELIVERY[0] = 1;
    }
    else if (CUBES_COLORS[3] == SECOND_CUBE_TO_TAKE && CUBES_DELIVERY[3] == -1)
    {
        CUBES_DELIVERY[3] = 1;
    }

    for (int i = 0; i < 4; i++)
    {
        if (CUBES_DELIVERY[i] == -1)
        {
            CUBES_DELIVERY[i] = 0;
        }
    }
}

// left to right
void takeCube1()
{
    moveBC(229, -24, -60);
    moveBC(583, 60, 24);
#if SLEEP_ON_CUBES == 1
    stopBC();
    sleep(5000);
#endif
    stopBC();
    setDegManipD(103, -50, 0);
    sleep(100);
    waitForManipD();
    moveBCSmartAccel(205, -20, 20, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    setTimeManipD(450, -65, -50);
    sleep(450);
}

void goToLineFrom1Cube()
{
    moveBC3Parts(20, 105, 25, 20, -20, 40, -40, 15, -15);
    moveBC3Parts(189, 274, 477, 70, 13, 98, 18.2, 70, 13);
}

void goToBigShipFrom1Cube()
{
    moveBCSmartAccel(120, 20, -20, -20, 20);
    tankTurnNSSmartAccel(95, -20, -20);
    moveBCSmartAccel(20, -20, 20, -20, 20);
    stopC();
    moveB(10, -40);
}

void takeCube2()
{
    moveBC(208, -26, -44);
    moveBC(343, 44, 26);
#if SLEEP_ON_CUBES == 1
    stopBC();
    sleep(5000);
#endif
    stopBC();
    setDegManipD(103, -50, 0);
    sleep(100);
    waitForManipD();
    moveBC3Parts(40, 45, 50, -20, 20, -40, 40, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    setTimeManipD(450, -65, -50);
    sleep(450);
}

void goToLineFrom2Cube()
{
    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBC3Parts(189, 55, 477, 70, 45, 98, 63, 70, 45);
}

void goToBigShipFrom2Cube()
{
    moveBCSmartAccel(120, 20, -20, -20, 20);
    tankTurnNSSmartAccel(95, -20, -20);
    moveBCSmartAccel(210, -20, 20, -20, 20);
    stopC();
    moveB(80, -40);
}

void takeCube3()
{
    moveBC(318, 44, 26);
    moveBC(184, -26, -44);
#if SLEEP_ON_CUBES == 1
    stopBC();
    sleep(5000);
#endif
    stopBC();
    setDegManipD(103, -50, 0);
    sleep(100);
    waitForManipD();
    moveBC3Parts(40, 45, 50, -20, 20, -40, 40, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    setTimeManipD(450, -65, -50);
    sleep(450);
}

void goToLineFrom3Cube()
{
    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBC3PartsMainC(189, 39, 477, -45, -70, -63, -98, -45, -70);
}

void goToBigShipFrom3Cube()
{
    moveBCSmartAccel(120, 20, -20, -20, 20);
    tankTurnNSSmartAccel(95, -20, -20);
    moveBCSmartAccel(370, -20, 20, -20, 20);
    stopC();
    moveB(80, -40);
}

void takeCube4()
{
    moveBC(535, 60, 24);
    moveBC(215, -24, -60);
#if SLEEP_ON_CUBES == 1
    stopBC();
    sleep(5000);
#endif
    stopBC();
    setDegManipD(103, -50, 0);
    sleep(100);
    waitForManipD();
    moveBCSmartAccel(210, -20, 20, -15, 15);
    moveBC(40, -15, 15);
    stopBC();
    setTimeManipD(450, -65, -50);
    sleep(450);
}

void goToLineFrom4Cube()
{
    moveBC3Parts(20, 55, 25, 20, -20, 40, -40, 15, -15);
    moveBC3PartsMainC(189, 280, 477, -13, -70, -18.2, -98, -13, -70)
}

void goToBigShipFrom4Cube()
{
    moveBCSmartAccel(120, 20, -20, -20, 20);
    tankTurnNSSmartAccel(95, -20, -20);
    moveBCSmartAccel(530, -20, 20, -20, 20);
    stopC();
    moveB(80, -40);
}

void moveOnLineToBigShip()
{
    SenRGBVals senData;
    bool flagWorking = true;
    setNewMotBCPowersAndRatio(-40, 0);
    readCalibratedSenRGB(sen2, &SEN2_CALIBRATION, &senData);
    eraseDisplay();
    while (flagWorking)
    {
        if ((senData.R + senData.G + senData.B) != 0)
        {
            if (senData.B / (senData.R + senData.G + senData.B) < 0.43)
            {
                flagWorking = false;
            }
        }
        readCalibratedSenRGB(sen2, &SEN2_CALIBRATION, &senData);
        displayCenteredTextLine(0, "%d", senData.B / (senData.R + senData.G + senData.B));
        sleep(1);
    }
    stopBC();
    lineSen2InDistOnlyBlue(&GREY_WHITEBLUE_LINE_PID, 250, 40, 105);
    MOT_PID_SETTINGS.pauseAction = true;
    while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) < 260)
    {
        PIDSen2InWorkOnlyBlue(&GREY_WHITEBLUE_LINE_PID, 105, 40);
        sleep(1);
    }
    MOT_PID_SETTINGS.pauseAction = false;
}

void putCubeOnShip()
{
    setDegManipD(100, 15, 0, 8);
    sleep(250);
    waitForManipD();
}

void put1CubeBigShipAndBack()
{
    moveOnLineToBigShip();

    moveBCSmartAccel(50, 10, 30, 10, 30);
    moveBCSmartAccel(150, -30, -10, -30, -10);
    stopBC();
    putCubeOnShip();

    moveBCSmartAccelExtreme(150, 30, -30, 25, -25);
    tankTurnNSSmartAccelExtreme(20, 25, 25);
    setTimeManipD(600, 100, 20, 8);
    moveBCSmartAccelExtreme(690, 30, -30, 19, -19);
    tankTurnNSSmartAccelExtreme(83, 20, 20);
}

void put2CubeBigShip()
{
    moveOnLineToBigShip();

    moveBCSmartAccelExtreme(351, 40, 30, 40, 30);
    stopBC();
    sleep(200);
    moveBCSmartAccelExtreme(130, -20, 20, -20, 20);
    moveBCSmartAccelExtreme(500, -30, -5, -30, -5);
    stopBC();
    putCubeOnShip();

    moveBCSmartAccel(90, 20, -20, 20, -20);
    moveB(250, 50);
    stopB();
    moveC(265, -50);
    stopC();
    setTimeManipD(300, 100, 20, 8);
    sleep(200);
    waitForManipD();
    setDegManipD(110, -40, 0, 8);
    moveBCSmartAccel(320, -20, 20, -17, 17);
    stopBC();
    sleep(200);
    setTimeManipD(500, -50, -50, 8);

    // толкать маленький корабль до белых кубиков
}

void put1CubeSmallShipAndBack()
{
    // кубик
    line2SenSmartAccelExtreme(450, 25, 50);
    line2SenCrawl(&DEFAULT_LINE_PID_MEDIUM, 50, 400, 400);
    moveBCSmartAccel(DEG_AFTER_LINE_FOR_MOVE, -50, 50, -50, 50);

    line2SenSmartAccel(370, 50, 20);
    moveBCSmartAccel(65, 10, 40, 10, 40);
    moveBCSmartAccel(260, -40, -10, -40, -10);
    stopBC();

    putCubeOnShip();
    moveBCSmartAccelExtreme(90, 25, -25, 25, -25);
    moveBCSmartAccelExtreme(735, 30, 20, 30, 20);
    setTimeManipD(700, 100, 20, 8);
    line2SenSmartAccelExtreme(210, 25, 60);
    line2SenCrawl(&DEFAULT_LINE_PID_MEDIUM, 60, 400, 400);
    moveBCSmartAccelExtreme(DEG_AFTER_LINE_FOR_MOVE, -60, 60, -60, 60);
}

void put2CubeSmallShipAndBack()
{
    line2SenSmartAccelExtreme(515, 25, 65);
    line2SenCrawl(&DEFAULT_LINE_PID_FAST, 65, 400, 400);
    moveBCSmartAccel(DEG_AFTER_LINE_FOR_MOVE, -65, 65, -60, 60);

    line2SenSmartAccel(340, 60, 20);
    moveBCSmartAccel(226, -40, -10, -40, -10);
    moveBCSmartAccel(60, 10, 40, 10, 40);
    stopBC();

    putCubeOnShip();
    moveBCSmartAccel(280, 20, -20, 20, -20);
    stopBC();
    setTimeManipD(300, 100, 20, 8);
    sleep(200);
    waitForManipD();
    setDegManipD(110, -40, 0, 8);
    moveBCSmartAccel(60, 10, 40, 10, 40);
    moveBCSmartAccel(225, -40, -10, -40, -10);
    line2SenDist(&DEFAULT_LINE_PID_SLOW, 140, 22);
    moveBC(40, -22, 22);
    stopBC();
    sleep(200);
    setTimeManipD(500, -50, -50, 8);
    waitForManipD();

    moveBCSmartAccel(530, 21, -21, 51, -51);
    setNewMotBCPowersAndRatio(51, -51);
    waitFor2Sen(400, 400);
    moveBCSmartAccel(210, 51, -51, 21, -21);
    stopBC();
    setTimeManipD(700, 100, 20, 8);
    sleep(100);
    moveBCSmartAccelExtreme(100, 20, -20, 20, -20);
    tankTurnNSSmartAccel(180, 22, 20);
    setNewMotBCPowersAndRatio(-20, -20);
    waitForSen1(300);
}

void putCubes()
{
    // два кубика на маленький корабль, после второго кубика "подтянуть" кораблик
    setDelivery();

    eraseDisplay();
    for (int i = 0; i < 4; i++)
    {
        displayCenteredTextLine(i, "%d", CUBES_DELIVERY[i]);
    }

    if (CUBES_DELIVERY[0] == 0)
    {
        takeCube1();
        CUBES_DELIVERY[0] = -1;
        goToLineFrom1Cube();
    }
    else if (CUBES_DELIVERY[3] == 0)
    {
        takeCube4();
        CUBES_DELIVERY[3] = -1;
        goToLineFrom4Cube();
    }
    else if (CUBES_DELIVERY[1] == 0)
    {
        takeCube2();
        CUBES_DELIVERY[1] = -1;
        goToLineFrom2Cube();
    }
    else
    {
        takeCube3();
        CUBES_DELIVERY[2] = -1;
        goToLineFrom3Cube();
    }

    put1CubeSmallShipAndBack();

    stopBC();

    line2SenSmartAccel(775, 60, 9);
    setNewMotBCPowersAndRatio(-9, 9);
    waitForCubes();
    moveBC(69, 18, -18);

    if (CUBES_DELIVERY[0] == 0)
    {
        takeCube1();
        CUBES_DELIVERY[0] = -1;
        goToLineFrom1Cube();
    }
    else if (CUBES_DELIVERY[3] == 0)
    {
        takeCube4();
        CUBES_DELIVERY[3] = -1;
        goToLineFrom4Cube();
    }
    else if (CUBES_DELIVERY[1] == 0)
    {
        takeCube2();
        CUBES_DELIVERY[1] = -1;
        goToLineFrom2Cube();
    }
    else
    {
        takeCube3();
        CUBES_DELIVERY[2] = -1;
        goToLineFrom3Cube();
    }

    put2CubeSmallShipAndBack();

    line2SenSmartAccel(310, 21, 9);
    setNewMotBCPowersAndRatio(-9, 9);
    waitForCubes();
    moveBC(69, 18, -18);

    if (CUBES_DELIVERY[0] == 1)
    {
        takeCube1();
        CUBES_DELIVERY[0] = -1;
        goToBigShipFrom1Cube();
    }
    else if (CUBES_DELIVERY[3] == 1)
    {
        takeCube4();
        CUBES_DELIVERY[3] = -1;
        goToBigShipFrom4Cube();
    }
    else if (CUBES_DELIVERY[1] == 1)
    {
        takeCube2();
        CUBES_DELIVERY[1] = -1;
        gotoBigShipFrom2Cube();
    }
    else
    {
        takeCube3();
        CUBES_DELIVERY[2] = -1;
        gotoBigShipFrom3Cube();
    }

    put1CubeBigShipAndBack();

    line2SenSmartAccel(210, 21, 9);
    setNewMotBCPowersAndRatio(-9, 9);
    waitForCubes();
    moveBC(69, 18, -18);

    if (CUBES_DELIVERY[0] == 1)
    {
        takeCube1();
        CUBES_DELIVERY[0] = -1;
        goToBigShipFrom1Cube();
    }
    else if (CUBES_DELIVERY[3] == 1)
    {
        takeCube4();
        CUBES_DELIVERY[3] = -1;
        goToBigShipFrom4Cube();
    }
    else if (CUBES_DELIVERY[1] == 1)
    {
        takeCube2();
        CUBES_DELIVERY[1] = -1;
        gotoBigShipFrom2Cube();
    }
    else
    {
        takeCube3();
        CUBES_DELIVERY[2] = -1;
        gotoBigShipFrom3Cube();
    }

    put2CubeBigShip();
}

void moveBigAndSmallShipToWhite()
{
    setTimeManipD(500, -60, -50, 8);
    waitForManipD();

    tankTurnNSSmartAccelExtreme(100, -24, -24);
    moveBCSmartAccelExtreme(450, -25, 25, -50, 50);
    setNewMotBCPowersAndRatio(-50, 50);
    waitFor2Sen(100, 100);
    moveBCSmartAccelExtreme(125, -50, 50, -20, 20);
    tankTurnNSSmartAccelExtreme(90, -24, -24);
    setNewMotBCPowersAndRatio(24, 24);
    waitForSen2(200);
    line2SenSmartAccelExtreme(270, 25, 30);
    stopBC();
    sleep(5000);
    line2SenDist(&DEFAULT_LINE_PID_MEDIUM, 100, 30);
    line2SenSmartAccelExtreme(1530, 25, 24);
    line2SenCrawl(&DEFAULT_LINE_PID_MEDIUM, 24, 400, 400);

    moveBCSmartAccel(218, -24, 24, -24, 24);
    stopBC();
    setTimeManipD(350, 100, 2);
    sleep(300);
    waitForManipD();

    moveBCSmartAccel(90, 20, -20, 20, -20);
    tankTurnNSSmartAccel(90, -21, -21);
}

void take2WhiteCube()
{
    moveBC(260, 80, 10);
    moveBCMainC(258, -10, -80);
    stopBC();
    setDegManipD(102, -50, 0);
    sleep(300);
    waitForManipD();
    moveBC3Parts(40, 50, 80, -17, 17, -40, 40, -17, 17);
    moveBC(100, -17, 17);
    stopBC();
    setTimeManipD(450, -65, -50);
    sleep(450);

    moveBC(100, 20, -20);
    moveBC3Parts(74, 204, 92, -20, -40, -45, -90, -10, -20);
}

void whiteCubeOnBig()
{
    setTimeManipD(350, 100, 2);
    line2SenSmartAccelExtreme(758, 17, 15);
    setNewMotBCPowersAndRatio(-13, 13);
    resetMotorEncoder(motD);
    waitFor2Sen(560, 560);
    stopBC();
    sleep(200);

    take2WhiteCube();
    line2SenSmartAccelExtreme(560, 20, 18);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 18, 400, 400);
    stopBC();
    sleep(200);
    moveBCSmartAccel(71, 20, -20, 20, -20);

    tankTurnNSSmartAccel(98, -21, -21);

    moveBCSmartAccel(150, -20, 20, -20, 20);
    stopBC();
    putCubeOnShip();
}

void pushShips()
{
    moveBCSmartAccelExtreme(100, 20, -20, 20, -20);
    stopC();
    moveB(400, 80);
    stopB();
    moveC(400, -80);
    stopC();
    setTimeManipD(350, 100, 2);
    waitForManipD();
    setDegManipD(100, -40, 0, 8);
    line2SenSmartAccelExtreme(420, 20, 22);
    line2SenDist(&DEFAULT_LINE_PID_MEDIUM, 50, 22);
    stopBC();
    setTimeManipD(300, -50, -50, 8);
    sleep(300);
    waitForManipD();
    line2SenSmartAccelExtreme(1400, 24, 24);
    line2SenCrawl(&DEFAULT_LINE_PID_MEDIUM, 24, 400, 400);
}

void conCraneAndFinish()
{
    moveC(330, -50);
    setStartDegMotBCAndReset(getEncoderB(), getEncoderC());
    setNewMotBCPowersAndRatio(-35, 35);
    while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) < 700)
    {
        sleep(1);
    }
    sleep(100);
    waitForSen2(400);
    sleep(100);
    while (readCalibratedSenSumRGB(sen2, &SEN2_CALIBRATION) < 700)
    {
        sleep(1);
    }
    waitForSen2(400);
    sleep(100);
    setStartDegMotBCAndReset(getEncoderB(), getEncoderC());
    setNewMotBCPowersAndRatio(-50, 0);
    stopC();
    while (readCalibratedSenSumRGB(sen1, &SEN1_CALIBRATION) < 700)
    {
        sleep(1);
    }
    waitForSen1(500);
    line2SenDist(&DEFAULT_LINE_PID_SLOW, 200, 30);
    setStartDegMotBCAndReset(getEncoderB(), getEncoderC());
    setNewMotBCPowersAndRatio(-80, 80);
    sleep(500);
    stopBC();
    moveBC(10, 20, -20);
    setTimeManipD(350, 100, 2);

    moveBCSmartAccelExtreme(150, 23, -23, 23, -23);
    tankTurnNSSmartAccelExtreme(180, -24, -24);
    stopBC();
    line2SenSmartAccelExtreme(290, 26, 70);
    setNewMotBCPowersAndRatio(-70, 70);
    waitForSen1(400);
    moveBCSmartAccelExtreme(200, -70, 70, -80, 80);

    line2SenSmartAccelExtreme(500, 80, 70);
    moveBCSmartAccelExtreme(220, -70, 70, -30, 30);
    stopBC();
    setSoundVolume(100);
    playSound(soundException);
}

void mainLogic()
{
    startReadPullShip();
    goToCubes();
    readCubes();
    putCubes();
    moveBigAndSmallShipToWhite();
    whiteCubeOnBig();
    pushShips();
    conCraneAndFinish();
}

void testFunc()
{
    conCraneAndFinish();

    // lineSen2InDistOnlyBlue(&GREY_WHITEBLUE_LINE_PID, 2000000, 40, 105);
    // setTimeManipD(1000, 100, 20, 8);
    // goToCubes();
    // readCubes();
    // putCubes();

    // takeCube3();
    // goToBigShipFrom4Cube();
    // put2CubeBigShip();

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