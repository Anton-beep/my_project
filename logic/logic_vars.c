#include "define_vals.c"

#define DEG_AFTER_LINE_FOR_TANK 96
#define DEG_AFTER_LINE_FOR_LINE_MOVE 80

// ONLY bool _______________________________________________________________
bool MANIPS_READY = false;
bool MANIP_A_READY_TO_TAKE = false;
bool MANIP_D_READY_TO_TAKE = false;
bool LEFT_CUBE_READY = false;
bool RIGHT_CUBE_READY = false;
bool MANIP_A_RELEASED = false;
bool MANIP_D_RELEASED = false;
bool TAKE_FROM_STORAGE_LEFT = false;
bool TAKE_FROM_STORAGE_RIGHT = false;
bool CORRECT_WORKING = false;
// TODO: chnage numbers for colors to defines (#define GREEN 3, etc.)
// 0 - *DO NOT USE*, 1 - no cube, 2 - *DO NOT USE*, 3 - green, 4 - blue, 5 - white, 6 - red (left to right on field)
int LEFT_MANIP_CUBE = 1;
int RIGHT_MANIP_CUBE = 1;

// ONLY int *______________________________________________________________
int IN_PTRS_INDICATORS[2];
int IN_PTRS_CUBES[2];

// 0 - completed, 1 - no cube, 2 - any, 3 - green, 4 - blue, 5 - white, 6 - red
int FIRST_CUBE_TO_TAKE = 3;
int SECOND_CUBE_TO_TAKE = 3;

// ONLY char *_____________________________________________________________
char *FILENAME = "deb.txt";

// 0 - *DO NOT USE*, 1 - no cube, 2 - *DO NOT USE*, 3 - green, 4 - blue, 5 - white, 6 - red (left to right on field)
int CUBES_COLORS[4] = {3, 4, 4, 3};

// ONLY int________________________________________________________________
int RGB_SUM_SEN1_BEFORE_SEN_CUBES = 510;
int RGB_SUM_SEN2_BEFORE_SEN_CUBES = 510;
int RGB_SUM_SEN1_BEFORE_CUBES = 510;
int RGB_SUM_SEN2_BEFORE_CUBES = 510;
#if WRITE_DATA_IN_FILE == 1
int DEBUG_FILE_HND = fileOpenWrite(FILENAME);
#endif

// ONLY SenRGBVals_________________________________________________________
SenHSVVals GREEN_CUBE_INDICATORS;
SenHSVVals BLUE_CUBE_INDICATORS;
SenHSVVals NOTHING_INDICATORS;
SenHSVVals GREEN_CUBE_CUBES;
SenHSVVals BLUE_CUBE_CUBES;
SenHSVVals ANOTHER_GREEN_CUBE_CUBES;
SenHSVVals ANOTHER_BLUE_CUBE_CUBES;
SenHSVVals NOTHING_CUBES;

void defineForLogic()
{
    // ONLY SenRGBVals________________________________________________________
    GREEN_CUBE_INDICATORS.H = 138.5;
    GREEN_CUBE_INDICATORS.S = 78.64;
    GREEN_CUBE_INDICATORS.V = 29;

    BLUE_CUBE_INDICATORS.H = 219.9;
    BLUE_CUBE_INDICATORS.S = 88.39;
    BLUE_CUBE_INDICATORS.V = 46.6;

    NOTHING_INDICATORS.H = 0;
    NOTHING_INDICATORS.S = 0;
    NOTHING_INDICATORS.V = 0;

    GREEN_CUBE_CUBES.H = 122;
    GREEN_CUBE_CUBES.S = 64.2;
    GREEN_CUBE_CUBES.V = 35.4;

    BLUE_CUBE_CUBES.H = 225.1;
    BLUE_CUBE_CUBES.S = 73.6;
    BLUE_CUBE_CUBES.V = 37;

    NOTHING_CUBES.H = 0;
    NOTHING_CUBES.S = 0;
    NOTHING_CUBES.V = 0;

    // ONLY int *_____________________________________________________________

    IN_PTRS_INDICATORS[0] = (int)&GREEN_CUBE_INDICATORS;
    IN_PTRS_INDICATORS[1] = (int)&BLUE_CUBE_INDICATORS;

    IN_PTRS_CUBES[0] = (int)&GREEN_CUBE_CUBES;
    IN_PTRS_CUBES[1] = (int)&BLUE_CUBE_CUBES;
}