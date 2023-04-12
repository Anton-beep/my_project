#include "include/PID_c_motors.c"
#include "include/motion.c"
#include "include/sen_for_colors.c"
#include "include/motion_with_sensors.c"
#include "include/PID_c_manip.c"

void mainLogic()
{
    tankTurnNS3Parts(30, 30, 30, 20, 20, 70, 70, 0, 0);
}