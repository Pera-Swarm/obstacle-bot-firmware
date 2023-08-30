#ifndef MOTOR_H
#define MOTOR_H

#include "define.h"

// define the const for the pid that control follow staight line
#define KP_FOWARD 0.8
#define KI_FOWARD 0.01
#define KD_FOWARD 0.01
#define SET_TIME_FOWARD 20

// motor class that contains basic functionality for the motors
class Motor
{
    public:
        Motor();

        void setup_motors();    // initalize the left and right motor
        void ML(int16_t val);   // write to left motor
        void MR(int16_t val);   // write to right motor

        void motorWrite(int8_t leftSpeed, int8_t rightSpeed);   // write to left motor and right motor

        double updateOutput();  // update to output respect to the angle error and const

    private:
        bool goingStraight = false; // check if the bot is going staight forward
        double setPoint, input, output;

        PID pid = PID(&input, &output, &setPoint, KP_FOWARD, KI_FOWARD, KD_FOWARD, DIRECT);     // pid instance for the motors
       
};

void pulse(int pulsetime, int time);

#endif