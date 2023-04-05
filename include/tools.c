#include "init.c"

char* floatToString(float val){
    char str[10];
    snprintf(str, 10, "%f", val);
    return str;
}

bool difSignsFloat(float val1, float val2){
    return (val1 < 0)? (val2 >= 0): (val2 < 0);
}

float cmToDeg(float cm){
    return 360 * cm / (PI * WHEEL_DIAMETER);
}

float min(float a, float b){
    return a < b ? a : b;
}

float max(float a, float b){
    return a > b ? a : b;
}