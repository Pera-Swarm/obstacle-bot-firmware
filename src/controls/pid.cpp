#include "pid.h"

MyPID::MyPID(double *input, double *output, double *setpoint, double Kp, double Ki, double Kd){
    
    this->input = input;
    this->output = output;
    this->setPoint = setpoint;
    this->Kp = Kp;
    this->Kd = Kd;
    this->Ki = Ki; 

    this->myPID =  &PID(input, output, setpoint, Kp, Ki, Kd, DIRECT);

    this->myPID->SetOutputLimits(-255, 255); // limits of the PID output
    this->myPID->SetSampleTime(20);          // refresh rate of the PID
    this->myPID->SetMode(AUTOMATIC);
}

bool MyPID::Compute(){
    return this->myPID->Compute();
}

double MyPID::Compute(double input, double setPoint){
    return input * setPoint;
}

void MyPID::setKp(double Kp){
    this->Kp = Kp;
}

void MyPID::setKd(double Kd){
    this->Kd = Kd;
}

void MyPID::setKi(double Ki){
    this->Ki = Ki;
}