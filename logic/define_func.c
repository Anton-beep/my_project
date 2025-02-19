#include "define_vals.c"

void defineMain()
{
    #if DEB_DISPLAY_CALIBRATED_RGB_SEN1 == 1
        debReadAndShowRGB(sen1, &SEN1_CALIBRATION);
    #elif DEB_DISPLAY_CALIBRATED_RGB_SEN2 == 1
        debReadAndShowRGB(sen2, &SEN2_CALIBRATION);
    #elif DEB_DISPLAY_CALIBRATED_RGB_SEN3 == 1
        debReadAndShowRGB(sen3, &SEN3_CALIBRATION);
    #elif DEB_DISPLAY_CALIBRATED_RGB_SEN4 == 1
        debReadAndShowRGB(sen4, &SEN4_CALIBRATION);
    #elif DEB_DISPLAY_MEAN_CALIBRATED_RGB_SEN1 == 1
        displayMeanCalibratedRGB(sen1, &SEN1_CALIBRATION);
    #elif DEB_DISPLAY_MEAN_CALIBRATED_RGB_SEN2 == 1
        displayMeanCalibratedRGB(sen2, &SEN2_CALIBRATION);
    #elif DEB_DISPLAY_MEAN_CALIBRATED_RGB_SEN3 == 1
        displayMeanCalibratedRGB(sen3, &SEN3_CALIBRATION);
    #elif DEB_DISPLAY_MEAN_CALIBRATED_RGB_SEN4 == 1
        displayMeanCalibratedRGB(sen4, &SEN4_CALIBRATION);
    #elif DEB_DISPLAY_MEAN_CALIBRATED_HSV_SEN1 == 1
        displayMeanCalibratedHSV(sen1, &SEN1_CALIBRATION);
    #elif DEB_DISPLAY_MEAN_CALIBRATED_HSV_SEN2 == 1
        displayMeanCalibratedHSV(sen2, &SEN2_CALIBRATION);
    #elif DEB_DISPLAY_MEAN_CALIBRATED_HSV_SEN3 == 1
        displayMeanCalibratedHSV(sen3, &SEN3_CALIBRATION);
    #elif DEB_DISPLAY_MEAN_CALIBRATED_HSV_SEN4 == 1
        displayMeanCalibratedHSV(sen4, &SEN4_CALIBRATION);
    #elif DEB_DISPLAY_CALIBRATON_VALUES == 1
        displayCalibrationValues();
    #elif TEST_MOTOR_CALIBRATION == 1
        testMotCalibration(10, 100, 100);
        stopAllTasks();
    #elif ONLY_SHOW_VOLTAGE == 1
        startTask(checkBat);
        waitForButtonPress();
        stopAllTasks();
    #endif


}