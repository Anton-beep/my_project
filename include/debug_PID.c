#include "tools.c"

task debugFileMotBCPID(){
    short desc = fileOpenWrite("MotBCPID.txt");
    fileWriteData(desc, "seconds; powerMotB; powerMotC; encMotB; encMotC\n", 48);
    clearTimer(T1);
    float secs;
    float powerMotB;
    float powerMotC;
    float encMotB;
    float encMotC;
    while (true){
        secs = time1[T1];
        powerMotB = motor[motB];
        powerMotC = motor[motC];
        encMotB = nMotorEncoder[motB];
        encMotC = nMotorEncoder[motC];
        fileWriteData(desc, floatToString(secs), 8);
        fileWriteData(desc, ";", 1);
        fileWriteData(desc, floatToString(powerMotB), 8);
        fileWriteData(desc, ";", 1);
        fileWriteData(desc, floatToString(powerMotC), 8);
        fileWriteData(desc, ";", 1);
        fileWriteData(desc, floatToString(encMotB), 8);
        fileWriteData(desc, ";", 1);
        fileWriteData(desc, floatToString(encMotC), 8);
        fileWriteData(desc, "\n", 1);
        sleep(5);
    }
}