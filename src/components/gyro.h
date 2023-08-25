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
    void kalmanUpdate(float newAngle, float newRate);

private:
    int MPU;
    float *GyroErrorXP; // Gyro error
    float *angleP;      // Gyro angle
    float GyroX, GyroY, GyroZ;
    float elapsedTime, currentTime, previousTime; // time stamps for gyro calculaions
    int c = 0;                                    // temp

    // Kalman filter variables
    float kalmanAngle = 0;
    float bias = 0;
    float P[2][2] = {0}; // Covariance matrix
    float Q[2][2] = {0}; // Process noise matrix
};

#endif