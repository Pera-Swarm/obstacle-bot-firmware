#ifndef MOTOR_H
#define MOTOR_H

#include "define.h"

// define the const for the pid 
#define KP_FOWARD 0.5 // 0.6
#define KI_FOWARD 0.005 // 0.01
#define KD_FOWARD 0.005 // 0.01
// define the rates of kp, ki and kd
#define KP_RATE 0.02
#define KI_RATE 0.005 
#define KD_RATE 0.005 

#define SET_TIME_FOWARD 20

// motor class that contains basic functionality for the motors
class Motor
{
    public:
        Motor();

        void setup_motors();    // initalize the left and right motor
        void ML(int16_t val);   // write to left motor
        void MR(int16_t val);   // write to right motor

        void motorWrite(int16_t leftSpeed, int16_t rightSpeed);   // write to left motor and right motor

        double updateOutput();  // update to output respect to the angle error and const
        void tunning(int16_t leftSpeed, int16_t rightSpeed);    // tune the Kp, KI and Kd

    private:
        bool goingStraight = false; // check if the bot is going staight forward
        double setPoint, input, output;

        PID pid = PID(&input, &output, &setPoint, KP_FOWARD, KI_FOWARD, KD_FOWARD, DIRECT);     // pid instance for the motors
       
};

void pulse(int pulsetime, int time);

#endif