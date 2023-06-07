#define DEG_AFTER_LINE_FOR_MOVE 100
#define DEG_AFTER_LINE_FOR_TURN 130

// ONLY int *______________________________________________________________
int IN_PTRS_FIRST_SCAN[3];
int IN_PTRS_CUBES[3];
int IN_PTRS_1CUBE[3];

// ONLY char *_____________________________________________________________
char *FILENAME = "deb.txt";

// 0 - green, 1 - blue, -1 - no cube
int CUBES_COLORS[4] = {0, 1, 0, 1};

// ONLY int________________________________________________________________
int RGB_SUM_SEN1_BEFORE_SEN_CUBES = 510;
int RGB_SUM_SEN2_BEFORE_SEN_CUBES = 510;
int RGB_SUM_SEN1_BEFORE_CUBES = 510;
int RGB_SUM_SEN2_BEFORE_CUBES = 510;
int DEBUG_FILE_HND = fileOpenWrite(FILENAME);

// -2 - err, -1 - no cube, 0 - green, 1 - blue, 
int FIRST_CUBE_TO_TAKE = 0;
int SECOND_CUBE_TO_TAKE = 1;

// ONLY SenRGBVals_________________________________________________________
SenHSVVals GREEN_CUBE_FIRST_SCAN;
SenHSVVals BLUE_CUBE_FIRST_SCAN;
SenHSVVals NOTHING_FIRST_SCAN;
SenHSVVals GREEN_CUBE_CUBES;
SenHSVVals BLUE_CUBE_CUBES;
SenHSVVals ANOTHER_GREEN_CUBE_CUBES;
SenHSVVals ANOTHER_BLUE_CUBE_CUBES;
SenHSVVals NOTHING_CUBES;

void defineForLogic()
{
    // ONLY SenRGBVals________________________________________________________
    GREEN_CUBE_FIRST_SCAN.H = 141;
    GREEN_CUBE_FIRST_SCAN.S = 138.5;
    GREEN_CUBE_FIRST_SCAN.V = 10.7;

    BLUE_CUBE_FIRST_SCAN.H = 220;
    BLUE_CUBE_FIRST_SCAN.S = 104.6;
    BLUE_CUBE_FIRST_SCAN.V = 24.6;

    NOTHING_FIRST_SCAN.H = 0;
    NOTHING_FIRST_SCAN.S = 0;
    NOTHING_FIRST_SCAN.V = 0;

    ANOTHER_GREEN_CUBE_CUBES.H = 130.1;
    ANOTHER_GREEN_CUBE_CUBES.S = 95.2;
    ANOTHER_GREEN_CUBE_CUBES.V = 24.6;

    ANOTHER_BLUE_CUBE_CUBES.H = 221.5;
    ANOTHER_BLUE_CUBE_CUBES.S = 95.7;
    ANOTHER_BLUE_CUBE_CUBES.V = 45;

    GREEN_CUBE_CUBES.H = 134.1;
    GREEN_CUBE_CUBES.S = 82.5;
    GREEN_CUBE_CUBES.V = 30.4;

    BLUE_CUBE_CUBES.H = 221.2;
    BLUE_CUBE_CUBES.S = 89.8;
    BLUE_CUBE_CUBES.V = 48.7;

    NOTHING_CUBES.H = 0;
    NOTHING_CUBES.S = 0;
    NOTHING_CUBES.V = 0;

    // ONLY int *_____________________________________________________________

    IN_PTRS_FIRST_SCAN[0] = (int)&GREEN_CUBE_FIRST_SCAN;
    IN_PTRS_FIRST_SCAN[1] = (int)&BLUE_CUBE_FIRST_SCAN;

    IN_PTRS_1CUBE[0] = (int)&ANOTHER_GREEN_CUBE_CUBES;
    IN_PTRS_1CUBE[1] = (int)&ANOTHER_BLUE_CUBE_CUBES;

    IN_PTRS_CUBES[0] = (int)&GREEN_CUBE_CUBES;
    IN_PTRS_CUBES[1] = (int)&BLUE_CUBE_CUBES;
}