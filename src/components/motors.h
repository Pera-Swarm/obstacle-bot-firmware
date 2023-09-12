#ifndef MOTOR_H
#define MOTOR_H

#include "define.h"

// starting address of EEPROM
#define ADDRESS 1

// constants for PID
struct PID_CONST
{
    // define the const for the PID
    double KP_FOWARD = 0.5;   // 0.6
    double KI_FOWARD = 0.005; // 0.01
    double KD_FOWARD = 0.005; // 0.01
    // define the rates of kp, ki and kd
    double KP_RATE = 0.02;
    double KI_RATE = 0.005;
    double KD_RATE = 0.005;

    int SET_TIME_FORWARD = 20;
};

// motor class that contains basic functionality for the motors
class Motor
{
public:
    Motor();

    void setup_motors();  // initalize the left and right motor
    void ML(int16_t val); // write to left motor
    void MR(int16_t val); // write to right motor

    void motorWrite(int16_t leftSpeed, int16_t rightSpeed); // write to left motor and right motor

    void updateOutput();                                 // update to output respect to the angle error and const
    void tunning(int16_t leftSpeed, int16_t rightSpeed); // tune the Kp, KI and Kd

    bool setPIDConstToEEPROM(double kpForward, double kiForward, double kdForward, double kpRate, double kiRate, double kdRate, int setTimeForward); // update the PID const in EEPROM
    bool getPIDConstFromEEPROM();

    void updateSetPoint(); // get the PID const from EEPROM
    void setGoingStraight(bool goingStraight);

    void stop();

private:
    PID_CONST pid_const; // values instance

    bool goingStraight = false; // check if the bot is going staight forward
    double setPoint, input, output;

    PID pid = PID(&input, &output, &setPoint, pid_const.KP_FOWARD, pid_const.KI_FOWARD, pid_const.KD_FOWARD, DIRECT); // PID instance for the motors
};

void pulse(int pulsetime, int time);

#endif