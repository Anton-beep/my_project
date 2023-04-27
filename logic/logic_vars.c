// ONLY int *______________________________________________________________
int IN_PTRS_FIRST_SCAN[3];
int IN_PTRS_CUBES[3];
int IN_PTRS_1CUBE[3];

// 0 - green, 1 - blue, -1 - no cube
int CUBES_COLORS[4] = {0, 1, 0, 1};

// ONLY int________________________________________________________________
int DEFAULT_BLACK_LINE = 5;

// 0 - green, 1 - blue
int FIRST_CUBE_TO_TAKE = 0;
int SECOND_CUBE_TO_TAKE = 1;

// ONLY SenRGBVals_________________________________________________________
SenRGBVals GREEN_CUBE_FIRST_SCAN;
SenRGBVals BLUE_CUBE_FIRST_SCAN;
SenRGBVals NOTHING_FIRST_SCAN;
SenRGBVals GREEN_CUBE_CUBES;
SenRGBVals BLUE_CUBE_CUBES;
SenRGBVals ANOTHER_GREEN_CUBE_CUBES;
SenRGBVals ANOTHER_BLUE_CUBE_CUBES;
SenRGBVals NOTHING_CUBES;

void defineForLogic()
{
    // ONLY SenRGBVals________________________________________________________
    GREEN_CUBE_FIRST_SCAN.R = 10;
    GREEN_CUBE_FIRST_SCAN.G = 97;
    GREEN_CUBE_FIRST_SCAN.B = 25;

    BLUE_CUBE_FIRST_SCAN.R = 30;
    BLUE_CUBE_FIRST_SCAN.G = 72;
    BLUE_CUBE_FIRST_SCAN.B = 110;

    NOTHING_FIRST_SCAN.R = 0;
    NOTHING_FIRST_SCAN.G = 0;
    NOTHING_FIRST_SCAN.B = 0;

    GREEN_CUBE_CUBES.R = 19;
    GREEN_CUBE_CUBES.G = 96;
    GREEN_CUBE_CUBES.B = 37;

    ANOTHER_GREEN_CUBE_CUBES.R = 4;
    ANOTHER_GREEN_CUBE_CUBES.G = 73;
    ANOTHER_GREEN_CUBE_CUBES.B = 13;

    BLUE_CUBE_CUBES.R = 16;
    BLUE_CUBE_CUBES.G = 42;
    BLUE_CUBE_CUBES.B = 134;

    ANOTHER_BLUE_CUBE_CUBES.R = 6;
    ANOTHER_BLUE_CUBE_CUBES.G = 49;
    ANOTHER_BLUE_CUBE_CUBES.B = 128;

    NOTHING_CUBES.R = 0;
    NOTHING_CUBES.G = 0;
    NOTHING_CUBES.B = 0;

    // ONLY int *_____________________________________________________________

    IN_PTRS_FIRST_SCAN[0] = (int)&GREEN_CUBE_FIRST_SCAN;
    IN_PTRS_FIRST_SCAN[1] = (int)&BLUE_CUBE_FIRST_SCAN;
    IN_PTRS_FIRST_SCAN[2] = (int)&NOTHING_FIRST_SCAN;

    IN_PTRS_CUBES[0] = (int)&GREEN_CUBE_CUBES;
    IN_PTRS_CUBES[1] = (int)&BLUE_CUBE_CUBES;
    IN_PTRS_CUBES[2] = (int)&NOTHING_CUBES;

    IN_PTRS_1CUBE[0] = (int)&ANOTHER_GREEN_CUBE_CUBES;
    IN_PTRS_1CUBE[1] = (int)&ANOTHER_BLUE_CUBE_CUBES;
    IN_PTRS_1CUBE[2] = (int)&NOTHING_CUBES;
}