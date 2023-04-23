#include "structures.c"

float PIDFunction (PIDSettings *params){
    if (params->pauseAction == true){
        return 0;
    }

    params->integral += params->errNow * params->dt;
    float derivative = (params->errNow - params->prevErr) / params->dt;
    params->prevErr = params->errNow;
    return params->Kp * params->errNow + params->Ki * params->integral + params->Kd * derivative;
}

void PIDReset (PIDSettings *params){
    params->prevErr = 0;
    params->integral = 0;
    params->errNow = 0;
}