#include "structures.c"
#include "PID_c.c"
#include "../robot_cfg.c"

int MANIP_A_DEG_TO_MOVE;
int MANIP_D_DEG_TO_MOVE;
short MANIP_A_ALLOWED_ERR = 2;
short MANIP_D_ALLOWED_ERR = 2;
bool MANIP_A_WORKING = false;
bool MANIP_D_WORKING = false;

task manipMoveA()
{
    MANIP_A_WORKING = true;
    PIDReset(MANIP_A_PID_PTR);
    int endDegA;
    if (MANIP_A_DEG_TO_MOVE > 0)
    {
        endDegA = nMotorEncoder[motA] + MANIP_A_DEG_TO_MOVE;
    }
    else
    {
        endDegA = nMotorEncoder[motA] - MANIP_A_DEG_TO_MOVE;
    }
    while (fabs(nMotorEncoder[motA] - endDegA) > MANIP_A_ALLOWED_ERR)
    {
        MANIP_A_PID_PTR->errNow = endDegA - nMotorEncoder[motA];
        short outPID = PIDFunction(MANIP_A_PID_PTR);
        motor[motA] = outPID;
        sleep(MANIP_A_PID_PTR->dt * 1000);
    }
    motor[motA] = 0;
    setMotorBrakeMode(motA, motorBrake);
    MANIP_A_WORKING = false;
}

task manipMoveD()
{
    MANIP_D_WORKING = true;
    PIDReset(MANIP_D_PID_PTR);
    int endDegD;
    if (MANIP_D_DEG_TO_MOVE > 0)
    {
        endDegD = nMotorEncoder[motD] + MANIP_D_DEG_TO_MOVE;
    }
    else
    {
        endDegD = nMotorEncoder[motD] - MANIP_D_DEG_TO_MOVE;
    }
    while (fabs(nMotorEncoder[motD] - endDegD) > MANIP_D_ALLOWED_ERR)
    {
        MANIP_D_PID_PTR->errNow = endDegD - nMotorEncoder[motD];
        short outPID = PIDFunction(MANIP_D_PID_PTR);
        motor[motD] = outPID;
        sleep(MANIP_D_PID_PTR->dt * 1000);
    }
    motor[motD] = 0;
    setMotorBrakeMode(motD, motorBrake);
    MANIP_D_WORKING = false;
}

// returns bool pointer to track when manipalutor stops working
bool *stratManipA(PIDSettings *PIDSetPtr, int deg, short allowedErr)
{
    MANIP_A_PID_PTR = PIDSetPtr;
    MANIP_A_DEG_TO_MOVE = deg;
    MANIP_A_ALLOWED_ERR = allowedErr;
    startTask(manipMoveA);
    return &MANIP_A_WORKING;
}

// returns bool pointer to track when manipalutor stops working
bool *stratManipD(PIDSettings *PIDSetPtr, int deg, short allowedErr)
{
    MANIP_D_PID_PTR = PIDSetPtr;
    MANIP_D_DEG_TO_MOVE = deg;
    MANIP_D_ALLOWED_ERR = allowedErr;
    startTask(manipMoveD);
    return &MANIP_D_WORKING;
}