#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include <Wire.h>

// define the number of iterations
#define AVGINTER 10
#define ERRORINTER 500

// Gyro class
// contains the basic functions with MPU6050
class Gyro
{

public:
    Gyro(int MPU, float *angle);

    void updateGyro();          // update the angle
    void calculate_IMU_error(); // calculate the error in yaw
    float getAngle();           // return the angle

private:
    int MPU;
    float GyroErrorXP = 0; // Gyro error
    float *angleP;         // Gyro angle
    float GyroX, GyroY, GyroZ;
    float elapsedTime, currentTime, previousTime; // time stamps for gyro calculaions
    int c = 0;                                    // temp
};

// double radToDegree(double rads);

#endif