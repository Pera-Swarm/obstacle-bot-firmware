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
    float kalmanAngle;
    float bias;
    float P[2][2]; // Covariance matrix
    float Q[2][2]; // Process noise matrix
};

#endif