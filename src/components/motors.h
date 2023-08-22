#ifndef MOTOR_H
#define MOTOR_H

#include "define.h"

#define KP 16
#define KI 0
#define KD 0.23
#define SET_TIME 20

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
        PID pid = PID(&input, &output, &setPoint, KP, KI, KD, DIRECT);
       
};

void pulse(int pulsetime, int time);

#endif