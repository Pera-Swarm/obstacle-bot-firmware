#include "gyro.h"

Gyro::Gyro(int MPU, float *angle)
{
    this->MPU = MPU;
    this->angleP = angle;
}

void Gyro::updateGyro()
{

    previousTime = currentTime;                        // Previous time is stored before the actual time read
    currentTime = millis();                            // Current time actual time read
    elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

    // get the angle as a avg value
    float AvgGyroZ = 0;
    for (int i = 0; i < AVGINTER; i++)
    {

        Wire.beginTransmission(MPU);
        Wire.write(0x43); // Gyro data first register address 0x43
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers

        GyroX = (Wire.read() << 8 | Wire.read()) / 32.75; // For a 1000deg/s range we have to divide first the raw value by 131.0, according to the datasheet
        GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
        GyroZ = (Wire.read() << 8 | Wire.read()) / 32.75;

        AvgGyroZ += GyroZ;
    }
    // to get the avg value
    GyroZ = AvgGyroZ / AVGINTER;

    GyroZ = GyroZ - GyroErrorXP; // GyroErrorX; // GyroErrorX ~(-0.56)

    // deg/s * s = deg
    *angleP = *angleP + GyroZ * elapsedTime;
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
    GyroErrorXP = 0;
    // Read gyro values 200 times
    while (c < ERRORINTER)
    {
        Wire.beginTransmission(MPU);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);
        GyroX = Wire.read() << 8 | Wire.read();
        GyroY = Wire.read() << 8 | Wire.read();
        GyroZ = Wire.read() << 8 | Wire.read();

        // Sum all readings
        GyroErrorXP = GyroErrorXP + (GyroZ / 32.75);
        c++;
    }

    // Divide the sum by ERRORINTER to get the error value
    GyroErrorXP = GyroErrorXP / ERRORINTER;

    // Print the error values on the Serial Monitor
    //  Serial.print("GyroErrorZ: ");
    //  Serial.println(*GyroErrorXP);
}

float Gyro::getAngle()
{
    return *angleP;
}
