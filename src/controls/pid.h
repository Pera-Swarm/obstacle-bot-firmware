#ifndef PID_H
#define PID_H

#include <PID_v1.h>

class MyPID
{
    public:
        MyPID(double *input, double *output, double *setpoint, double Kp, double Ki, double Kd);

        bool Compute();
        double Compute(double input, double setPoint);
        void setKp(double Kp);
        void setKi(double Ki);
        void setKd(double Kd);

        PID *myPID;

    private:
        double *setPoint, *input, *output;
        double Kp, Ki, Kd;

};

#endif