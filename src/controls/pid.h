#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include <PID_v1.h>

class MyPID
{
    public:
        MyPID(double *input, double *output, double *setpoint, double Kp, double Ki, double Kd);

        bool Compute();

        PID *myPID;


    private:
        double *setPoint, *input, *output;
        double Kp, Ki, Kd;

};

#endif