#include "structures.c"

/*
TODO:
- корректировка коэффицентов, в зависимости от установленной мощности моторов. На данный момент есть
проблема, что для разных мощностей моторов приходится создавать разные настройки коэффицентов регулятора, 
надо изучить как избежать такой проблемы и сделать коэффиценты едиными для всех мощностей, если это возомжно
- поэкспериминтировать с интегральной составляющей: в теории, если сделать ограниченную интегральную составляющую,
то это может помочь роботу справлятся с серьезными аномалиями поля/моторов. Например, ограничить интегральную 
составляющую последними 10 значениями (очередь).
*/

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