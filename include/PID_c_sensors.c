#include "PID_c.c"
#include "tools.c"
#include "../robot_cfg.c"
#include "sen_calibration.c"
#include "PID_c_motors.c"

void PID2SensWork(PIDSettings *PIDSet, float pow)
{
    PIDSet->errNow = readCalibratedSenSumRGB(sen1, SEN1_CALIBRATION) - readCalibratedSenSumRGB(sen2, SEN2_CALIBRATION);
    float err = PIDFunction(PIDSet);
    motor[motB] = min(MIN_SEN_POWER, (pow + err) * -1);
    motor[motC] = max(-1 * MIN_SEN_POWER, pow - err);
}

void PIDSen1InWork(PIDSettings *PIDSet, float target, float pow)
{
    PIDSet->errNow = readCalibratedSenSumRGB(sen1, SEN1_CALIBRATION) - target;
    float err = PIDFunction(PIDSet);
    motor[motB] = min(MIN_SEN_POWER, (pow + err) * -1);
    motor[motC] = max(-1 * MIN_SEN_POWER, pow - err);
}

void PIDSen2InWork(PIDSettings *PIDSet, float target, float pow)
{
    PIDSet->errNow = readCalibratedSenSumRGB(sen2, SEN1_CALIBRATION) - target;
    float err = PIDFunction(PIDSet);
    motor[motB] = min(MIN_SEN_POWER, (pow + err) * -1);
    motor[motC] = max(-1 * MIN_SEN_POWER, pow - err);
}

SenRGBVals DATA_FOR_PID_SEN2_IN_WORK_ONLY_BLUE;

void PIDSen2InWorkOnlyBlue(PIDSettings *PIDSet, float target, float pow)
{
    readCalibratedSenRGB(sen2, &SEN2_CALIBRATION, &DATA_FOR_PID_SEN2_IN_WORK_ONLY_BLUE);

    PIDSet->errNow = DATA_FOR_PID_SEN2_IN_WORK_ONLY_BLUE.B - target;
    float err = PIDFunction(PIDSet);
    motor[motB] = min(MIN_SEN_POWER, (pow + err) * -1);
    motor[motC] = max(-1 * MIN_SEN_POWER, pow - err);
}

void PIDSen1OutWork(PIDSettings *PIDSet, float target, float pow)
{
    PIDSet->errNow = readCalibratedSenSumRGB(sen1, SEN1_CALIBRATION) - target;
    float err = PIDFunction(PIDSet);
    motor[motB] = min(MIN_SEN_POWER, (pow + err) * -1);
    motor[motC] = max(-1 * MIN_SEN_POWER, pow - err);
}

void PIDSen2OutWork(PIDSettings *PIDSet, float target, float pow)
{
    PIDSet->errNow = readCalibratedSenSumRGB(sen2, SEN1_CALIBRATION) - target;
    float err = PIDFunction(PIDSet);
    motor[motB] = min(MIN_SEN_POWER, (pow + err) * -1);
    motor[motC] = max(-1 * MIN_SEN_POWER, pow - err);
}