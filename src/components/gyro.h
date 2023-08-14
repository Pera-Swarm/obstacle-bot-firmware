#ifndef GYRO_H
#define GYRO_H

#include "define.h"
#include <Wire.h>
#include <Arduino.h>

class Gyro
{
    public:
        Gyro(int MPU, float *angle, float *GyroErrorX);

        void updateGyro();
        void calculate_IMU_error();

    private:
        int MPU;
        float *GyroErrorXP;                             // Gyro error
        float *angleP;                                  // Gyro angle
        float GyroX, GyroY, GyroZ;
        float elapsedTime, currentTime, previousTime; // time stamps for gyro calculaions
        int c = 0; // temp

};

#endif