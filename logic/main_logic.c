#include "include/PID_c_motors.c"
#include "include/motion.c"
#include "include/sen_for_colors.c"
#include "include/motion_with_sensors.c"
#include "include/PID_c_manip.c"
#include "logic_vars.c"
#include "define_vals.c"

task setManips()
{
    setTimeManipA(300, 100, 1);
    setTimeManipD(300, -100, -1);
    sleep(400);
    resetMotorEncoder(motA);
    resetMotorEncoder(motD);
    MANIPS_READY = true;
}

void takeCubes()
{
    line2SenCrawl(&DEFAULT_LINE_PID_SLOW, 25, 200, 200);
    setDegManipA(77, -10, 0, 8);
    setDegManipD(80, 10, 0, 8);
    moveBC(50, -20, 20);
    line2Sen3Parts(&DEFAULT_LINE_PID_SLOW, &DEFAULT_LINE_PID_MEDIUM, &DEFAULT_LINE_PID_SLOW, 200, 0, 538, 30, 80, 3);
    stopBC();
}

task takeLeftCubeFromPos()
{
    setDegManipA(40, -20, -70, 8);
    setDegManipA(60, -70, -10, 8);
    LEFT_CUBE_READY = true;
}

task takeRightCubeFromPos()
{
    setDegManipD(40, 20, 70, 8);
    setDegManipD(60, 70, 10, 8);
    RIGHT_CUBE_READY = true;
}

void takeFromPosCubes()
{
    startTask(takeLeftCubeFromPos);
    startTask(takeRightCubeFromPos);
    while (!LEFT_CUBE_READY || !RIGHT_CUBE_READY)
    {
        sleep(1);
    }
    moveBC(125, -25, 25);
    stopBC();
    setTimeManipA(500, -90, -10);
    setTimeManipD(500, 90, 10);
    sleep(500)
}

void mainLogic()
{
    sleep(5000);
    startTask(setManips);
    sleep(500);
    takeCubes();
    takeFromPosCubes();
    moveBC(400, 50, -50);
}

void testFunc()
{
    testRPM(motA);

    // testMotorCalibrationDebStream(100, 100);

    // startTimeD(400, -65, -50);
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