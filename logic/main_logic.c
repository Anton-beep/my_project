#include "include/PID_c_motors.c"
#include "include/motion.c"
#include "include/sen_for_colors.c"
#include "include/motion_with_sensors.c"
#include "include/PID_c_manip.c"
#include "logic_vars.c"
#include "define_vals.c"

task setManipsStart()
{
    setTimeManipA(300, 100, 1);
    setTimeManipD(300, -100, -1);
    waitForManipA();
    waitForManipD();
    resetMotorEncoder(motA);
    resetMotorEncoder(motD);
    MANIPS_READY = true;
}

void setCompactManips()
{
    setTimeManipA(300, -100, -1);
    setTimeManipD(300, 100, 1);
    waitForManipA();
    waitForManipD();
}

void startPushShipAndRead()
{
    moveBCSmartAccel(210, -18, 18, -50, 50);
    int *indicatorsScan = startReadRowOfObjectsHSV(&NOTHING_INDICATORS, 20, IN_PTRS_INDICATORS, 2, sen3, &SEN3_CALIBRATION_INDICATORS);
    line2SenSmartAccel(230, 50, 30);
    setNewMotBCPowersAndRatio(-30, 30);
    waitForSen2(395);
    moveBCSmartAccel(270, -30, 30, -27, 27);
    setNewMotBCPowersAndRatio(-27, 27);
    sleep(200);
    stopBC();
    stopTask(readRowOfObjectsHSV);

    if (indicatorsScan[1] == &GREEN_CUBE_INDICATORS)
    {
        CUBES_FOR_BIG_SHIP[0] = 3;
    }
    else if (indicatorsScan[1] == &BLUE_CUBE_INDICATORS)
    {
        CUBES_FOR_BIG_SHIP[1] = 4;
    }

    if (indicatorsScan[3] == &GREEN_CUBE_INDICATORS)
    {
        CUBES_FOR_BIG_SHIP[0] = 3;
    }
    else if (indicatorsScan[3] == &BLUE_CUBE_INDICATORS)
    {
        CUBES_FOR_BIG_SHIP[0] = 4;
    }
    eraseDisplay();
    for (int i = 0; i < 10; i++)
    {
        if (indicatorsScan[i] == &GREEN_CUBE_INDICATORS)
        {
            displayCenteredTextLine(i, "GREEN");
        }
        else if (indicatorsScan[i] == &BLUE_CUBE_INDICATORS)
        {
            displayCenteredTextLine(i, "BLUE");
        }
        else
        {
            displayCenteredTextLine(i, "NONE");
        }
    }

#if WRITE_DATA_IN_FILE == 1
    fileWriteChar(DEBUG_FILE_HND, FIRST_CUBE_TO_TAKE);
    fileWriteChar(DEBUG_FILE_HND, SECOND_CUBE_TO_TAKE);
#endif

    moveBCSmartAccel(170, 30, -30, 17, -17);
    tankTurnNSSmartAccel(90, 30, 17);
    stopBC();
}

void goToCubes()
{
    setCompactManips();

    line2SenSmartAccel(850, 30, 23);
    setNewMotBCPowersAndRatio(-23, 23);
    waitFor2Sen();
    moveBCSmartAccel(DEG_AFTER_LINE_FOR_TANK, -23, 23, -21, 21);

    tankTurnNSSmartAccel(90, -30, -20);

    moveBCSmartAccel(300, 30, -30, 20, -20);
    line2SenSmartAccel(211, 25, 25);
    setNewMotBCPowersAndRatio(-25, 25);
    waitFor2Sen();
    moveBCCustomAccelMainB(DEG_AFTER_LINE_FOR_LINE_MOVE, -25, 25, SAFE_START_ACCEL * -1, SAFE_START_ACCEL);
    line2SenSmartAccel(657, POWER_MOT_C, 20);
    stopBC();
}

void writeValToCubes(int val, int ind)
{
    if (val == &GREEN_CUBE_CUBES)
    {
        CUBES_COLORS[ind] = 3;
    }
    else if (val == &BLUE_CUBE_CUBES)
    {
        CUBES_COLORS[ind] = 4;
    }
    else
        (val == &NOTHING_CUBES)
        {
            CUBES_COLORS[ind] = 1;
        }
}

void readCubes()
{
    moveBC(152, -16, -50);
    stopBC();
    sleep(100);
    moveBC(295, 25, -14.5);
    stopBC();
    sleep(100);
    writeValToCubes(readWindowHSV(sen3, &SEN3_CALIBRATION_CUBES, &NOTHING_CUBES, 15, IN_PTRS_CUBES, 2), 0);

    // read cube1

    moveBC(132, -20, 24);
    writeValToCubes(readWindowHSV(sen3, &SEN3_CALIBRATION_CUBES, &NOTHING_CUBES, 15, IN_PTRS_CUBES, 2), 1);

    // read cube2

    moveBC(136, -30, 30);
    writeValToCubes(readWindowHSV(sen3, &SEN3_CALIBRATION_CUBES, &NOTHING_CUBES, 15, IN_PTRS_CUBES, 2), 2);

    // read cube3

    moveBC(125, -30, 30);
    writeValToCubes(readWindowHSV(sen3, &SEN3_CALIBRATION_CUBES, &NOTHING_CUBES, 15, IN_PTRS_CUBES, 2), 3);

    // read cube4

    stopBC();

    for (int i = 0; i < 4; i++)
    {
        if (CUBES_COLORS[i] == 2)
        {
            displayCenteredTextLine(i, "GREEN");
        }
        else if (CUBES_COLORS[i] == 3)
        {
            displayCenteredTextLine(i, "BLUE");
        }
        else if (CUBES_COLORS[i] == 1)
        {
            displayCenteredTextLine(i, "NONE");
        }
        else
        {
            displayCenteredTextLine(i, "ERROR");
        }
#if WRITE_DATA_IN_FILE == 1
        fileWriteChar(DEBUG_FILE_HND, CUBES_COLORS[i]);
#endif
    }

    moveBC(210, -20, 20);
    tankTurnNSSmartAccel(30.5, -20, -20);
    moveBCSmartAccelSamePowers(255, 30, -30);
    moveB(315, 40);
    stopBC();
}

task setManipAForTake()
{
    setTimeManipA(600, 100, 1);
    waitForManipA();
    setDegManipA(86, -40, 0);
    waitForManipA();
    MANIP_A_READY_TO_TAKE = true;
}

task setManipDForTake()
{
    setTimeManipD(600, -100, 1);
    waitForManipD();
    setDegManipD(86, 40, 0);
    waitForManipD();
    MANIP_D_READY_TO_TAKE = true;
}

task takeLeftCubeFromPos()
{
    setDegManipA(40, -32, -70, 8.09);
    waitForManipA();
    setDegManipA(50, -70, -2, 8);
    waitForManipA();
    LEFT_MANIP_CUBE = CUBES_COLORS[0];
    LEFT_CUBE_READY = true;
}

task takeRightCubeFromPos()
{
    setDegManipD(40, 32, 70, 8.09);
    waitForManipD();
    setDegManipD(58, 70, 5, 8);
    waitForManipD();
    LEFT_MANIP_CUBE = CUBES_COLORS[3];
    RIGHT_CUBE_READY = true;
}

void takeFromPosCubes()
{
    startTask(takeLeftCubeFromPos, 8);
    startTask(takeRightCubeFromPos, 8);
    sleep(300);
    while (!LEFT_CUBE_READY || !RIGHT_CUBE_READY)
    {
        sleep(1);
    }
    moveBC(33, -22, 22);
    coastBC();
    setDegManipA(31, -60, -5);
    setTimeManipD(83, 70, 5);
    waitForManipA();
    waitForManipD();
    moveBC(100, -20, 20);
    stopBC();
    setTimeManipA(200, -60, -25);
    setTimeManipD(200, 60, 25);
    waitForManipA();
    waitForManipD();
    sleep(500);
}

void takeCubes()
{
    startTask(setManipAForTake, 8);
    startTask(setManipDForTake, 8);
    sleep(300);
    line2SenSmartAccel(322, 30, 16);
    coastBC();
    while (!MANIP_A_READY_TO_TAKE || !MANIP_D_READY_TO_TAKE)
    {
        sleep(1);
    }
    takeFromPosCubes();
}

task releaseLeftCubeOnShip()
{
    MANIP_A_RELEASED = false;
    setDegManipA(40, 40, 0);
    waitForManipA();
    sleep(300);
    setTimeManipA(200, -50, -20);
    LEFT_MANIP_CUBE = 1;
    MANIP_A_RELEASED = true;
}

task releaseRightCubeOnShip()
{
    MANIP_D_RELEASED = false;
    setDegManipD(40, -40, 0);
    waitForManipD();
    sleep(300);
    setTimeManipD(200, 50, 20);
    LEFT_MANIP_CUBE = 1;
    MANIP_D_RELEASED = true;
}

void releaseLeftOnSmallShip()
{
    moveBC(200, -40, -10);
    moveBC(50, 10, 40);

    stopBC();
    sleep(5000);

    moveBC(100, -20, 20);

    stopBC();
    sleep(5000);

    stopBC();
    startTask(releaseLeftCubeOnShip, 8);
    LEFT_MANIP_CUBE = 1;
    sleep(300);
    while (!MANIP_A_RELEASED)
    {
        sleep(1);
    }

    sleep(5000);

    moveBC(100, 20, -20);
    moveBC(50, -10, -40);
    moveBC(200, 40, 10);
    stopBC();

    sleep(5000);
}

void releaseRightOnSmallShip()
{
    moveBC(50, 10, 40);
    moveBC(200, -40, -10);

    stopBC();
    sleep(5000);

    moveBC(100, -20, 20);

    stopBC();
    sleep(5000);

    stopBC();
    startTask(releaseRightCubeOnShip, 8);
    RIGHT_MANIP_CUBE = 1;
    sleep(300);
    while (!MANIP_D_RELEASED)
    {
        sleep(1);
    }

    sleep(5000);

    moveBC(100, 20, -20);
    moveBC(200, 40, 10);
    moveBC(50, -10, -40);
    stopBC();

    sleep(5000);
}

void releaseOverlappedLeftOnSmallShip()
{
    moveBC(400, -40, -10);
    moveBC(100, 10, 40);

    stopBC();
    sleep(5000);

    moveBC(30, -20, 20);

    stopBC();
    sleep(5000);

    stopBC();
    startTask(releaseLeftCubeOnShip, 8);
    LEFT_MANIP_CUBE = 1;
    sleep(300);
    while (!MANIP_A_RELEASED)
    {
        sleep(1);
    }

    sleep(5000);

    moveBC(30, 20, -20);
    moveBC(100, -10, -40);
    moveBC(400, 40, 10);
    stopBC();

    sleep(5000);
}

void releaseOverlappedRightOnSmallShip()
{
    moveBC(100, 10, 40);
    moveBC(400, -40, -10);

    stopBC();
    sleep(5000);

    moveBC(30, -20, 20);
    stopBC();

    stopBC();
    sleep(5000);

    startTask(releaseRightCubeOnShip, 8);
    RIGHT_MANIP_CUBE = 1;
    sleep(300);
    while (!MANIP_D_RELEASED)
    {
        sleep(1);
    }

    sleep(5000);

    moveBC(30, 20, -20);
    moveBC(400, 40, 10);
    moveBC(100, -10, -40);
    stopBC();

    sleep(5000);
}

task manipLeftCubeFromStorage()
{
    LEFT_CUBE_READY = false;
    setDegManipA(40, -32, -70, 8.09);
    waitForManipA();
    setTimeManipA(300, -80, -5);
    waitForManipA();
    LEFT_MANIP_CUBE = CUBES_COLORS[1];
    LEFT_CUBE_READY = true;
}

void takeLeftFromStorage()
{
    setTimeManipA(300, 100, 1);
    waitForManipA();
    moveBC(200, 20, -20);

    stopBC();
    sleep(5000);

    moveBC(50, 10, 50);
    moveBC(250, -50, -10);
    stopBC();

    stopBC();
    sleep(5000);

    startTask(manipLeftCubeFromStorage, 8);
    sleep(400);
    while (!LEFT_CUBE_READY)
    {
        sleep(1);
    }

    sleep(5000);

    moveBC(500, 50, 10);
    moveBC(100, -10, -50);
    stopBC();
}

void takeRightFromStorage()
{
    setTimeManipD(300, -100, -1);
    waitForManipD();
    moveBC(200, 20, -20);

    stopBC();
    sleep(5000);

    moveBC(250, -50, -10);
    moveBC(50, 10, 50);
    stopBC();

    stopBC();
    sleep(5000);

    startTask(manipRightCubeFromStorage, 8);
    sleep(400);
    while (!RIGHT_CUBE_READY)
    {
        sleep(1);
    }

    sleep(5000);

    moveBC(100, -10, -50);
    moveBC(500, 50, 10);
    stopBC();

    sleep(5000);
}

void take2CubesFromStorage()
{
    takeLeftFromStorage();
    takeRightFromStorage();
}

void fillInManips()
{
    if (LEFT_MANIP_CUBE == 1 && RIGHT_MANIP_CUBE == 1)
    {
        take2CubesFromStorage();
    }
    else if (LEFT_MANIP_CUBE == 1)
    {
        takeLeftFromStorage();
    }
    else if (RIGHT_MANIP_CUBE == 1)
    {
        takeRightFromStorage();
    }
}

void firstTimeBigShipAndRelease(bool releaseLeft, bool releaseRight)
{
    moveBCSmartAccel(300, -21, 21, -21, 21);
    moveBC(50, -20, 20);
    stopBC();

    sleep(5000);

    if (releaseLeft && releaseRight)
    {
        startTask(releaseLeftCubeOnShip);
        startTask(releaseRightCubeOnShip);
        sleep(300);
        while (!MANIP_A_RELEASED || !MANIP_D_RELEASED)
        {
            sleep(1);
        }

        sleep(5000);
    }
    else if (releaseLeft)
    {
        startTask(releaseLeftCubeOnShip);
        sleep(300);
        while (!MANIP_A_RELEASED)
        {
            sleep(1);
        }

        sleep(5000);
    }
    else if (releaseRight)
    {
        startTask(releaseRightCubeOnShip);
        sleep(300);
        while (!MANIP_D_RELEASED)
        {
            sleep(1);
        }

        sleep(5000);
    }
}

void smallShipAndRelease(bool releaseLeft, bool releaseRight)
{
    line2SenSmartAccel(250, 21, 21);
    stopBC();

    sleep(5000);

    // put cubes on small ship
    if (releaseLeft && releaseRight)
    {
        releaseLeftOnSmallShip();

        sleep(5000);

        releaseRightOnSmallShip();
    }
    else if (releaseLeft)
    {
        releaseLeftOnSmallShip();
    }
    else if (releaseRight)
    {
        releaseRightOnSmallShip();
    }
}

void fromSmallToBigShipAndTake()
{
    tankTurnNSSmartAccel(180, 21, -21);

    line2SenSmartAccel(300, 21, 21);
    setNewMotBCPowersAndRatio(-21, 21);
    waitFor2Sen();
    moveBCSmartAccel(DEG_AFTER_LINE_FOR_LINE_MOVE, -21, 21, -21, 21);

    line2SenSmartAccel(350, 21, 21);
    stopBC();

    setTimeManipA(300, 100, 1);
    setTimeManipD(300, -100, -1);

    tankTurnNSSmartAccel(90, 21, 21);
    stopBC();

    sleep(5000);

    waitForManipA();
    waitForManipD();
    moveBCSmartAccel(300, 21, -21, 21, -21);
    moveBC(50, 20, 20);

    setTimeManipA(300, -100, -30);
    setTimeManipD(300, 100, 30);
    waitForManipA();
    waitForManipD();
}

void workWithCubes()
{
    moveBCSmartAccel(300, 30, -30);
    stopBC();

    sleep(5000);

    /* In total 6 possible variants, depending on postions of cubes and which cubes robot should put on big ship
    fast : put two cubes on the big or small ship instantly, changing both together after first ship, small or big ship could be first
    normal : put one cube on the big ship, change, put two on the small ship, change, put the last one on the big ship
    overlap: put one cube on the big ship, change, put one on the small ship, change, put one on the small ship (but on a different position), put one on the big ship (but on a different position)
    */

    // "fast" variants
    if ((FIRST_CUBE_TO_TAKE == CUBES_COLORS[1] && SECOND_CUBE_TO_TAKE == CUBES_COLORS[2]) || (FIRST_CUBE_TO_TAKE == CUBES_COLORS[2] && SECOND_CUBE_TO_TAKE == CUBES_COLORS[1]))
    {
        // cubes on the big ship are in the storage, therefore the small ship is the first
        tankTurnNSSmartAccel(180, 21, -21);
        line2SenSmartAccel(300, 21, 21);
        setNewMotBCPowersAndRatio(-21, 21);
        waitFor2Sen();
        smallShipAndRelease(true, true);

        // change
        tankTurnNSSmartAccel(180, 21, -21);
        line2SenSmartAccel(300, 21, 21);
        stopBC();
        fillInManips();

        // go to big ship, put both cubes
        line2SenSmartAccel(100, 21, 21);
        setNewMotBCPowersAndRatio(-21, 21);
        waitFor2Sen();
        moveBCSmartAccel(DEG_AFTER_LINE_FOR_LINE_MOVE, -21, 21, -21, 21);
        line2SenSmartAccel(400, 21, 21);

        // должен остановиться на линии на большой корабль

        // go to big ship, put both cubes
        firstTimeBigShipAndRelease(true, true);

        // take big ship
        moveBCSmartAccel(200, 21, -21, 21, -21);

        stopBC();
        sleep(5000);

        setTimeManipA(300, 100, 1);
        setTimeManipD(300, -100, -1);
        tankTurnNSSmartAccel(180, 21, -21);
        waitForManipA();
        waitForManipD();

        moveBC(70, 20, -20);
        coastBC();

        setTimeManipA(300, -100, -30);
        setTimeManipD(300, 100, 30);
        waitForManipA();
        waitForManipD();
    }
    else if ((FIRST_CUBE_TO_TAKE == CUBES_COLORS[0] && SECOND_CUBE_TO_TAKE == CUBES_COLORS[3]) || (FIRST_CUBE_TO_TAKE == CUBES_COLORS[3] && SECOND_CUBE_TO_TAKE == CUBES_COLORS[0]))
    {
        // cubes on the big ship are in the manips, therefore the big ship is the first
        firstTimeBigShipAndRelease(true, true);
        moveBCSmartAccel(350, 21, -21, 21, -21);
        tankTurnNSSmartAccel(90, -21, 21);
        stopBC();

        // change
        fillInManips();

        // go to small ship, put both cubes
        line2SenSmartAccel(300, 21, 21);
        setNewMotBCPowersAndRatio(-21, 21);
        waitFor2Sen();
        moveBCSmartAccel(DEG_AFTER_LINE_FOR_LINE_MOVE, -21, 21, -21, 21);

        smallShipAndRelease(true, true);

        // go to big ship, take it

        fromSmallToBigShipAndTake();
    }
    // "normal" variants
    else if ((FIRST_CUBE_TO_TAKE == CUBES_COLORS[0] && SECOND_CUBE_TO_TAKE == CUBES_COLORS[2]) || (FIRST_CUBE_TO_TAKE == CUBES_COLORS[2] && SECOND_CUBE_TO_TAKE == CUBES_COLORS[0]))
    {
        // to big ship, release left
        firstTimeBigShipAndRelease(true, false);
        moveBCSmartAccel(350, 21, -21, 21, -21);
        tankTurnNSSmartAccel(90, -21, 21);
        stopBC();

        // change
        fillInManips();

        // to small ship, release both
        line2SenSmartAccel(300, 21, 21);
        setNewMotBCPowersAndRatio(-21, 21);
        waitFor2Sen();
        moveBCSmartAccel(DEG_AFTER_LINE_FOR_LINE_MOVE, -21, 21, -21, 21);

        smallShipAndRelease(true, true);

        // go to big ship, take it

        fromSmallToBigShipAndTake();
    }
    else if ((FIRST_CUBE_TO_TAKE == CUBES_COLORS[1] && SECOND_CUBE_TO_TAKE == CUBES_COLORS[3]) || (FIRST_CUBE_TO_TAKE == CUBES_COLORS[3] && SECOND_CUBE_TO_TAKE == CUBES_COLORS[1]))
    {
        // to big ship, release right
        firstTimeBigShipAndRelease(false, true);
        moveBCSmartAccel(350, 21, -21, 21, -21);
        tankTurnNSSmartAccel(90, -21, 21);
        stopBC();

        // change
        fillInManips();

        // to small ship, release both
        line2SenSmartAccel(300, 21, 21);
        setNewMotBCPowersAndRatio(-21, 21);
        waitFor2Sen();
        moveBCSmartAccel(DEG_AFTER_LINE_FOR_LINE_MOVE, -21, 21, -21, 21);

        smallShipAndRelease(true, true);

        // go to big ship, take it

        fromSmallToBigShipAndTake();
    }
    // "overlap" variants
    else if ((FIRST_CUBE_TO_TAKE == CUBES_COLORS[2] && SECOND_CUBE_TO_TAKE == CUBES_COLORS[3]) || (FIRST_CUBE_TO_TAKE == CUBES_COLORS[3] && SECOND_CUBE_TO_TAKE == CUBES_COLORS[2]))
    {
        // to big ship, release left
        firstTimeBigShipAndRelease(true, false);
        moveBCSmartAccel(350, 21, -21, 21, -21);
        tankTurnNSSmartAccel(90, -21, 21);
        stopBC();

        // change
        fillInManips();

        // to small ship, release left
        line2SenSmartAccel(300, 21, 21);
        setNewMotBCPowersAndRatio(-21, 21);
        waitFor2Sen();
        moveBCSmartAccel(DEG_AFTER_LINE_FOR_LINE_MOVE, -21, 21, -21, 21);

        smallShipAndRelease(true, false);

        // change
        moveBC(100, 21, -21);
        stopBC();
        fillInManips();

        // release overlaped left
        line2SenSmartAccel(150, 21, 21);
        releaseOverlappedLeftOnSmallShip();

        // go to big ship, take it

        fromSmallToBigShipAndTake();
    }
    else if ((FIRST_CUBE_TO_TAKE == CUBES_COLORS[0] && SECOND_CUBE_TO_TAKE == CUBES_COLORS[1]) || (FIRST_CUBE_TO_TAKE == CUBES_COLORS[1] && SECOND_CUBE_TO_TAKE == CUBES_COLORS[0]))
    {
        // to big ship, release right
        firstTimeBigShipAndRelease(false, true);
        moveBCSmartAccel(350, 21, -21, 21, -21);
        tankTurnNSSmartAccel(90, -21, 21);
        stopBC();

        // change
        fillInManips();

        // to small ship, release right
        line2SenSmartAccel(300, 21, 21);
        setNewMotBCPowersAndRatio(-21, 21);
        waitFor2Sen();
        moveBCSmartAccel(DEG_AFTER_LINE_FOR_LINE_MOVE, -21, 21, -21, 21);

        smallShipAndRelease(false, true);

        // change
        moveBC(100, 21, -21);
        stopBC();
        fillInManips();

        // release overlaped right
        line2SenSmartAccel(150, 21, 21);
        releaseOverlappedRightOnSmallShip();

        // go to big ship, take it

        fromSmallToBigShipAndTake();
    }
}

void mainLogic()
{
    startTask(setManipsStart);
    startPushShipAndRead();
    goToCubes();
    readCubes();
    takeCubes();
}

void testFunc()
{
    startTask(setManipsStart);
    while (!MANIPS_READY)
    {
        sleep(1);
    }
    sleep(5000);
    setDegManipD(50, 10, 0);
    waitForManipD();
    stopD();
    setDegManipA(50, -10, 0);
    waitForManipA();
    stopA();
    // testMotorCalibrationDebStream(100, 100);

    // startTimeD(400, -65, -50);
    // sleep(3000);
    // putCubeOnShip();

    // tankTurnNS3Parts(90, 360 * 2, 90, -20, -20, -50, -50, -17, -17);

    // displayMeanCalibratedHSV(sen3, &SEN3_CALIBRATION);

    // moveBC3PartsMainC(334, 0, 156, 10, 60, 15, 90, 10, 60);
    // stopBC();

    // line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 40, 50, 50);
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