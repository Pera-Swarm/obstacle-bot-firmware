#ifndef MOTOR_H
#define MOTOR_H

#include "define.h"

class Motor
{
    public:
        Motor();
        void setup_motors();

        void ML(int8_t val);
        void MR(int8_t val);

        void motorWrite(int8_t leftSpeed, int8_t rightSpeed);

    private:
        bool goingStraight = false;
        double setPoint, input, output;
        PID *pid;
       
};

void pulse(int pulsetime, int time);

#endif