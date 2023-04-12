#include "PID_c.c"
#include "tools.c"
#include "../robot_cfg.c"
#include "sen_calibration.c"
#include "PID_c_motors.c"

void PID2SensWork(PIDSettings *PIDSet, float pow)
{
    PIDSet->errNow = readCalibratedSenSumRGB(sen1, SEN1_CALIBRATION) - readCalibratedSenSumRGB(sen2, SEN2_CALIBRATION);
    float err = PIDFunction(PIDSet);
    setNewMotBCPowersAndRatio((pow + err) * -1, pow - err); // could be wrong
    writeDebugStreamLine("%d %d", (pow + err) * -1, pow - err);
}

void PIDSen1InWork(PIDSettings *PIDSet, float target, float pow)
{
    PIDSet->errNow = readCalibratedSenSumRGB(sen1, SEN1_CALIBRATION) - target;
    float err = PIDFunction(PIDSet);
    setNewMotBCPowersAndRatio((pow + err) * -1, pow - err); // could be wrong
}

void PIDSen2InWork(PIDSettings *PIDSet, float target, float pow)
{
    PIDSet->errNow = readCalibratedSenSumRGB(sen2, SEN1_CALIBRATION) - target;
    float err = PIDFunction(PIDSet);
    setNewMotBCPowersAndRatio((pow + err) * -1, pow - err); // could be wrong
}

void PIDSen1OutWork(PIDSettings *PIDSet, float target, float pow)
{
    PIDSet->errNow = readCalibratedSenSumRGB(sen1, SEN1_CALIBRATION) - target;
    float err = PIDFunction(PIDSet);
    setNewMotBCPowersAndRatio((pow + err) * -1, pow - err); // could be wrong
}

void PIDSen2OutWork(PIDSettings *PIDSet, float target, float pow)
{
    PIDSet->errNow = readCalibratedSenSumRGB(sen2, SEN1_CALIBRATION) - target;
    float err = PIDFunction(PIDSet);
    setNewMotBCPowersAndRatio((pow + err) * -1, pow - err); // could be wrong
}

// TODO: check 4 functions for one sensors: sen1 line is in, sen1 line is out, sen2 line is in, sen2 line is out