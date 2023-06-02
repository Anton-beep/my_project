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
    setTimeManipA(700, -100, -10);
    setTimeManipD(700, 100, 10);
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
        FIRST_CUBE_TO_TAKE = 3;
    }
    else if (indicatorsScan[1] == &BLUE_CUBE_INDICATORS)
    {
        FIRST_CUBE_TO_TAKE = 4;
    }

    if (indicatorsScan[3] == &GREEN_CUBE_INDICATORS)
    {
        SECOND_CUBE_TO_TAKE = 3;
    }
    else if (indicatorsScan[3] == &BLUE_CUBE_INDICATORS)
    {
        SECOND_CUBE_TO_TAKE = 4;
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
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 23, 400, 400);
    moveBCSmartAccel(DEG_AFTER_LINE_FOR_TANK, -23, 23, -21, 21);

    tankTurnNSSmartAccel(90, -30, -20);

    moveBCSmartAccel(300, 30, -30, 20, -20);
    line2SenSmartAccel(170, 25, 25);
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 25, 400, 400);
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
    // writeValToCubes(readWindowHSV(sen3, &SEN3_CALIBRATION_CUBES, &NOTHING_CUBES, 5, IN_PTRS_CUBES, 2), 0);

    // read cube1

    moveBC(132, -20, 24);
    // writeValToCubes(readWindowHSV(sen3, &SEN3_CALIBRATION_CUBES, &NOTHING_CUBES, 5, IN_PTRS_CUBES, 2), 1);

    // read cube2

    moveBC(136, -30, 30);
    // writeValToCubes(readWindowHSV(sen3, &SEN3_CALIBRATION_CUBES, &NOTHING_CUBES, 5, IN_PTRS_CUBES, 2), 2);

    // read cube3

    moveBC(125, -30, 30);
    // writeValToCubes(readWindowHSV(sen3, &SEN3_CALIBRATION_CUBES, &NOTHING_CUBES, 5, IN_PTRS_CUBES, 2), 3);

    // read cube4

    stopBC();

    for (int i = 0; i < 4; i++)
    {
        if (CUBES_COLORS[i] == 3)
        {
            displayCenteredTextLine(i, "GREEN");
        }
        else if (CUBES_COLORS[i] == 4)
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
    moveBCSmartAccelSamePowers(235, 30, -30);
    moveB(315, 65);
    stopBC();
}

task takeLeftCubeFromPos()
{
    setDegManipA(90, -30, -60, 8.09);
    waitForManipA();
    setDegManipA(100, -60, -1, 8);
    waitForManipA();
    LEFT_MANIP_CUBE = CUBES_COLORS[0];
    LEFT_CUBE_READY = true;
}

task takeRightCubeFromPos()
{
    setDegManipD(90, 30, 60, 8.09);
    waitForManipD();
    setDegManipD(100, 60, 1, 8);
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
    moveBC(91, -21, 21);
    coastBC();
    setTimeManipA(800, -40, -55);
    setTimeManipD(800, 40, 55);
    waitForManipA();
    waitForManipD();
}

// START ONLY WHEN ROBOT IS GOING FORWARD
task correctCubesInStorage()
{
    setDegManipA(50, 40, 0);
    setDegManipD(50, -40, 0);
    waitForManipA();
    waitForManipD();
    sleep(500);
    setTimeManipA(300, -50, -35);
    setTimeManipD(300, 50, 35);
}

void takeCubes()
{
    setTimeManipA(400, 100, 1);
    setTimeManipD(400, -100, -1);
    sleep(300);
    line2SenSmartAccel(340, 30, 14);
    coastBC();
    takeFromPosCubes();
}


// TODO: версия без кубиков в отсеках 
task releaseLeftCubeOnShip()
{
    MANIP_A_RELEASED = false;
    setTimeManipA(800, 28, 0);
    sleep(300);
    waitForManipA();
    sleep(300);
    setDegManipA(10, -20, 0);
    waitForManipA();
    moveBC(50, 20, -20);
    stopBC();
    setDegManipA(220, -80, 0);
    sleep(300);
    waitForManipA();
    moveBC(50, -20, 20);
    stopBC()
    setTimeManipA(400, -50, -20);
    waitForManipA();
    LEFT_MANIP_CUBE = 1;
    MANIP_A_RELEASED = true;
}

task releaseRightCubeOnShip()
{
    MANIP_D_RELEASED = false;
    setTimeManipD(800, -28, 0);
    sleep(300);
    waitForManipD();
    sleep(300);
    setDegManipD(10, 20, 0);
    waitForManipD();
    moveBC(50, 21, -21);
    stopBC();
    setDegManipD(220, 80, 0);
    sleep(300);
    waitForManipD();
    moveBC(50, -21, 21);
    stopBC();
    setTimeManipD(400, 50, 40);
    waitForManipD();
    LEFT_MANIP_CUBE = 1;
    MANIP_D_RELEASED = true;
}

void releaseLeftOnSmallShip()
{
    moveBC(372, -40, -10);
    moveBC(93, 10, 40);

    moveBC(40, -20, 20);

    stopBC();
    startTask(releaseLeftCubeOnShip, 8);
    sleep(300);
    while (!MANIP_A_RELEASED)
    {
        sleep(1);
    }

    moveBC(40, 20, -20);

    moveBC(93, -10, -40);
    moveBC(372, 40, 10);
    stopBC();
}

void releaseRightOnSmallShip()
{
    moveBC(110, 10, 40);
    moveBC(430, -40, -10);

    moveBC(45, -20, 20);

    stopBC();
    startTask(releaseRightCubeOnShip, 8);
    sleep(300);
    while (!MANIP_D_RELEASED)
    {
        sleep(1);
    }

    moveBC(40, 20, -20);

    moveBC(430, 40, 10);
    moveBC(110, -10, -40);
    stopBC();
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

void releaseLeftOnBigShip()
{
    moveBC(252, -40, -10);
    moveBC(65, 10, 40);

    moveBC(49, -20, 20);

    stopBC();
    startTask(releaseLeftCubeOnShip, 8);
    sleep(300);
    while (!MANIP_A_RELEASED)
    {
        sleep(1);
    }

    moveBC(49, 20, -20);

    moveBC(65, -10, -40);
    moveBC(252, 40, 10);
    stopBC();
}

void releaseRightOnBigShip()
{
    moveBC(65, 10, 40);
    moveBC(227, -40, -10);

    moveBC(72, -20, 20);

    stopBC();
    startTask(releaseRightCubeOnShip, 8);
    sleep(300);
    while (!MANIP_D_RELEASED)
    {
        sleep(1);
    }

    moveBC(72, 20, -20);

    moveBC(227, 40, 10);
    moveBC(65, -10, -40);
    stopBC();
}

void releaseOverlappedLeftOnBigShip()
{
    moveBC(400, -40, -10);
    moveBC(100, 10, 40);

    moveBC(40, -20, 20);

    stopBC();
    startTask(releaseLeftCubeOnShip, 8);
    sleep(300);
    while (!MANIP_A_RELEASED)
    {
        sleep(1);
    }

    moveBC(40, 20, -20);

    moveBC(100, -10, -40);
    moveBC(400, 40, 10);
    stopBC();
}

void releaseOverlappedRightOnBigShip()
{
    moveBC(100, 10, 40);
    moveBC(400, -40, -10);

    moveBC(45, -20, 20);

    stopBC();
    startTask(releaseRightCubeOnShip, 8);
    sleep(300);
    while (!MANIP_D_RELEASED)
    {
        sleep(1);
    }

    moveBC(40, 20, -20);

    moveBC(400, 40, 10);
    moveBC(100, -10, -40);
    stopBC();
}

void manipLeftCubeFromStorage(int newCube = CUBES_COLORS[1])
{
    setDegManipA(150, -35, -35, 8.09);
    sleep(300);
    waitForManipA();
    setTimeManipA(1000, -35, -5);
    LEFT_MANIP_CUBE = newCube;
}

void manipRightCubeFromStorage(int newCube = CUBES_COLORS[2])
{
    setDegManipD(150, 35, 35, 8.09);
    sleep(300);
    waitForManipD();
    setTimeManipD(1000, 35, 5);
    RIGHT_MANIP_CUBE = newCube;
}

void takeLeftFromStorage()
{
    setTimeManipA(500, 60, 1);
    waitForManipA();

    moveB(250, 40);
    stopB();
    moveC(250, -40);
    stopC();
    sleep(200);
    moveBC(110, -20, 20);
    stopBC();

    manipLeftCubeFromStorage();

    moveC(250, 40);
    stopC();
    moveB(250, -40);
    stopB();
}

void takeRightFromStorage()
{
    setTimeManipD(500, -60, -1);
    waitForManipD();

    moveC(250, -40);
    stopC();
    moveB(250, 40);
    stopB();
    sleep(200);
    moveBC(110, -20, 20);
    stopBC();

    manipRightCubeFromStorage();

    moveB(250, -40);
    stopB();
    moveC(250, 40);
    stopC();
}

void take2CubesFromStorage()
{
    takeLeftFromStorage();
    line2SenSmartAccel(70, 20, 19);
    stopBC();
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
    lineSen2InDist(&DEFAULT_LINE_PID_SLOW, 220, 21, 361);
    setNewMotBCPowersAndRatio(-23, 23);
    waitForSen1(200);
    moveBC(200, -25, 25);

    if (releaseLeft && releaseRight)
    {
        releaseLeftOnBigShip();
        releaseRightOnBigShip();
    }
    else if (releaseLeft)
    {
        releaseLeftOnBigShip();
    }
    else if (releaseRight)
    {
        releaseRightOnBigShip();
    }
}

void smallShipAndRelease(bool releaseLeft, bool releaseRight)
{
    line2SenSmartAccel(300, 21, 21);
    stopBC();

    // put cubes on small ship
    if (releaseLeft && releaseRight)
    {
        releaseLeftOnSmallShip();
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
    moveBC(267, 21, -21);
    stopBC();

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
        startTask(correctCubesInStorage);
        line2SenSmartAccel(300, 21, 21);
        setNewMotBCPowersAndRatio(-21, 21);
        waitFor2Sen();
        smallShipAndRelease(true, true);

        // change
        tankTurnNSSmartAccel(180, 21, -21);
        stopBC();
        startTask(correctCubesInStorage);
        line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 25, 400, 400);
        moveBCSmartAccel(DEG_AFTER_LINE_FOR_LINE_MOVE, -21, 21, -21, 21);
        line2SenSmartAccel(130, 21, 21);
        stopBC();
        sleep(5000);
        waitForManipA();
        waitForManipD();
        fillInManips();
        stopBC();
        line2SenSmartAccel(100, 21, 21);
        sleep(5000);

        // go to big ship, put both cubes

        // должен остановиться на линии на большой корабль

        // go to big ship, put both cubes
        tankTurnNSSmartAccel(90, -21, -21);
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
        tankTurnNSSmartAccel(90, -21, -21);
        stopBC();
        startTask(correctCubesInStorage);
        firstTimeBigShipAndRelease(true, true);
        moveBCSmartAccel(495, 22, -22, 22, -22);
        tankTurnNSSmartAccel(90, -23, -23);
        stopBC();

        // change
        fillInManips();

        stopBC();
        // go to small ship, put both cubes
        line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 25, 400, 400);

        smallShipAndRelease(true, true);

        // go to big ship, take it

        fromSmallToBigShipAndTake();
    }
    // "normal" variants
    else if ((FIRST_CUBE_TO_TAKE == CUBES_COLORS[0] && SECOND_CUBE_TO_TAKE == CUBES_COLORS[2]) || (FIRST_CUBE_TO_TAKE == CUBES_COLORS[2] && SECOND_CUBE_TO_TAKE == CUBES_COLORS[0]))
    {
        // to big ship, release left
        startTask(correctCubesInStorage);
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
        startTask(correctCubesInStorage);
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
        startTask(correctCubesInStorage);
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
        startTask(correctCubesInStorage);
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

void bigShipToWhiteCubes()
{
    // from big ship to small ship
    moveBC(100, -21, 21);
    stopC();
    turnBDegrSmartAccel(90, -21, -21);

    stopBC();
    sleep(5000);

    moveBCSmartAccel(500, -21, 21, -21, 21);
    stopB();
    turnCDegrSmartAccel(90, 21, 21);
    stopBC();

    sleep(5000);

    // from small ship to white cubes

    moveBC(100, -21, 21);
    moveBCSmartAccel(100, -10, 50, -15, 75);
    moveBCSmartAccel(500, -75, 15, -10, 50);
}

void mainLogic()
{
    // startTask(setManipsStart);
    // startPushShipAndRead();
    goToCubes();
    readCubes();
    takeCubes();
    workWithCubes();
}

void testFunc()
{
    // setCompactManips();
    // workWithCubes();

    lineSen2InDist(&DEFAULT_LINE_PID_SLOW, 200, 21, 361);

    // to debug small ship
    // setCompactManips();
    // line2SenSmartAccel(300, 21, 21);
    // setNewMotBCPowersAndRatio(-21, 21);
    // waitFor2Sen();
    // smallShipAndRelease(true, true);

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