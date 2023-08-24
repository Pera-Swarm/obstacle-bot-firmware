#include "gyro.h"

Gyro::Gyro(int MPU, float *angle, float *GyroErrorX)
{
    this->MPU = MPU;
    this->GyroErrorXP = GyroErrorX;
    this->angleP = angle;
}

void Gyro::updateGyro()
{

    previousTime = currentTime;                        // Previous time is stored before the actual time read
    currentTime = millis();                            // Current time actual time read
    elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

    Wire.beginTransmission(MPU);
    Wire.write(0x43); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers

    GyroX = (Wire.read() << 8 | Wire.read()) / 32.75; // For a 1000deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 32.75;
    GyroZ = GyroZ - (-0.25) + *GyroErrorXP; // GyroErrorX; // GyroErrorX ~(-0.56)

    *angleP = *angleP + GyroZ * elapsedTime; // deg/s * s = deg
    float newAngle = *angleP;
    kalmanUpdate(newAngle, GyroY);

    Serial.println(*angleP);
}

void Gyro::calculate_IMU_error()
{

    Wire.begin();                // Initialize comunication
    Wire.beginTransmission(MPU); // Start communication with MPU6050 // MPU=0x68
    Wire.write(0x6B);            // Talk to 0 register 6B
    Wire.write(0x00);            // reset
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU);
    Wire.write(0x1B); // Talk to the GYRO_CONFIG register (1B hex)
    Wire.write(0x10); // Set the register bits as 00010000 (1000deg/s full scale)
    Wire.endTransmission(true);
    delay(20);

    // initialize c to 0
    int c = 0;
    // Read gyro values 200 times
    while (c < 200)
    {
        Wire.beginTransmission(MPU);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);
        GyroX = Wire.read() << 8 | Wire.read();
        GyroY = Wire.read() << 8 | Wire.read();
        GyroZ = Wire.read() << 8 | Wire.read();
        // Sum all readings
        *GyroErrorXP = *GyroErrorXP + (GyroZ / 32.75);
        c++;
    }

    // Divide the sum by 200 to get the error value
    *GyroErrorXP = *GyroErrorXP / 200;

    // Print the error values on the Serial Monitor
    //  Serial.print("GyroErrorZ: ");
    //  Serial.println(*GyroErrorX);
}

float Gyro::getAngle()
{
    return *angleP;
}

// Kalman filter update
void Gyro::kalmanUpdate(float newAngle, float newRate)
{
    float S, K[2];

    // Prediction step
    *angleP += elapsedTime * (newRate - bias);
    P[0][0] += elapsedTime * (elapsedTime * P[1][1] - P[0][1] - P[1][0] + Q[0][0]);
    P[0][1] -= elapsedTime * P[1][1];
    P[1][0] -= elapsedTime * P[1][1];
    P[1][1] += Q[1][1] * elapsedTime;

    // Update step
    S = P[0][0] + 50;
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    *angleP += K[0] * (newAngle - *angleP);
    bias += K[1] * (newAngle - *angleP);

    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];
}
