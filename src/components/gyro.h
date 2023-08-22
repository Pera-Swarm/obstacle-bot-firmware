#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include <Wire.h>

class Gyro
{
    public:
        Gyro(int MPU, float *angle, float *GyroErrorX);

        void updateGyro();
        void calculate_IMU_error();
        float getAngle();

    private:
        int MPU;
        float *GyroErrorXP;                             // Gyro error
        float *angleP;                                  // Gyro angle
        float GyroX, GyroY, GyroZ;
        float elapsedTime, currentTime, previousTime; // time stamps for gyro calculaions

};

#endif