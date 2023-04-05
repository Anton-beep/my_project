typedef struct PIDSettings 
{
    float Kp;
    float Ki;
    float Kd;
    float prevErr;
    float integral;
    float errNow;
    float dt;   // IN SECONDS!!!!!!!!!
    bool pauseAction;
} PIDSettings;


float PIDFunction (PIDSettings *params){
    if (params->pauseAction){
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