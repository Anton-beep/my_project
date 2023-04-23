#include "structures.c"
#include "PID_c.c"
#include "../robot_cfg.c"

void stopA()
{
    motor[motA] = 0;
    setMotorBrakeMode(motA, motorBrake);
}

void stopD()
{
    motor[motD] = 0;
    setMotorBrakeMode(motD, motorBrake);
}

int MANIP_A_DEG_TO_MOVE;
int MANIP_D_DEG_TO_MOVE;
short MANIP_A_END_POWER_MOVE;
short MANIP_D_END_POWER_MOVE;
short MANIP_A_ALLOWED_ERR = 2;
short MANIP_D_ALLOWED_ERR = 2;
bool MANIP_A_WORKING = false;
bool MANIP_D_WORKING = false;

task manipMoveA()
{
    MANIP_A_WORKING = true;
    PIDReset(MANIP_A_PID_PTR);
    int endDegA = nMotorEncoder[motA] + MANIP_A_DEG_TO_MOVE;
    while (fabs(nMotorEncoder[motA] - endDegA) > MANIP_A_ALLOWED_ERR)
    {
        MANIP_A_PID_PTR->errNow = endDegA - nMotorEncoder[motA];
        short outPID = PIDFunction(MANIP_A_PID_PTR);
        motor[motA] = outPID;
        sleep(MANIP_A_PID_PTR->dt * 1000);
    }
    motor[motA] = MANIP_A_END_POWER_MOVE;
    MANIP_A_WORKING = false;
}

task manipMoveD()
{
    MANIP_D_WORKING = true;
    PIDReset(MANIP_D_PID_PTR);
    int endDegD = nMotorEncoder[motD] + MANIP_D_DEG_TO_MOVE;
    while (fabs(nMotorEncoder[motD] - endDegD) > MANIP_D_ALLOWED_ERR)
    {
        MANIP_D_PID_PTR->errNow = endDegD - nMotorEncoder[motD];
        short outPID = PIDFunction(MANIP_D_PID_PTR);
        motor[motD] = outPID;
        sleep(MANIP_D_PID_PTR->dt * 1000);
    }
    motor[motD] = MANIP_D_END_POWER_MOVE;
    MANIP_D_WORKING = false;
}

// returns bool pointer to track when manipalutor stops working
bool *stratManipA(PIDSettings *PIDSetPtr, int deg, short allowedErr, short endPow = 0)
{
    MANIP_A_PID_PTR = PIDSetPtr;
    MANIP_A_DEG_TO_MOVE = deg;
    MANIP_A_ALLOWED_ERR = allowedErr;
    MANIP_A_END_POWER_MOVE = endPow;
    startTask(manipMoveA);
    return &MANIP_A_WORKING;
}

// returns bool pointer to track when manipalutor stops working
bool *stratManipD(PIDSettings *PIDSetPtr, int deg, short allowedErr, short endPow = 0)
{
    MANIP_D_PID_PTR = PIDSetPtr;
    MANIP_D_DEG_TO_MOVE = deg;
    MANIP_D_ALLOWED_ERR = allowedErr;
    MANIP_D_END_POWER_MOVE = endPow;
    startTask(manipMoveD);
    return &MANIP_D_WORKING;
}

unsigned long MSECONDS_A = 0;
unsigned long MSECONDS_D = 0;
short POWER_TIME_A = 0;
short POWER_TIME_D = 0;
short END_POWER_TIME_A = 0;
short END_POWER_TIME_D = 0;

task manipTimeA()
{
    motor[motA] = POWER_TIME_A;
    sleep(MSECONDS_A);
    motor[motA] = END_POWER_TIME_A;
}

task manipTimeD()
{
    motor[motD] = POWER_TIME_D;
    sleep(MSECONDS_D);
    motor[motD] = END_POWER_TIME_D;
}

void startTimeA(int time, int powStart, int powEnd)
{
    MSECONDS_A = time;
    POWER_TIME_A = powStart;
    END_POWER_TIME_A = powEnd;
    startTask(manipTimeA);
}

void startTimeD(int time, int powStart, int powEnd)
{
    MSECONDS_D = time;
    POWER_TIME_D = powStart;
    END_POWER_TIME_D = powEnd;
    startTask(manipTimeD);
}