#include "include/PID_c_motors.c"
#include "include/motion.c"
#include "include/sen_for_colors.c"
#include "include/motion_with_sensors.c"
#include "include/PID_c_manip.c"

void mainLogic()
{
    startTimeD(800, -30, -19);
    sleep(5000);
    startTimeD(500, 20, 40);
    sleep(1000);
    startTimeD(2000, -30, -20);
    sleep(5000);
}