typedef struct SenSettings
{
    short minR;
    short minG;
    short minB;
    short maxR;
    short maxG;
    short maxB;
} SenSettings;

typedef struct SenRGBVals
{
    float R;
    float G;
    float B;
} SenRGBVals;

typedef struct SenHSVVals
{
    float H;
    float S;
    float V;
} SenHSVVals;

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