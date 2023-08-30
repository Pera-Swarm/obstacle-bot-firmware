#include "motors.h"

Motor::Motor()
{
}

void Motor::setup_motors()
{
    pid.SetOutputLimits(-255, 255); // limits of the PID output
    pid.SetSampleTime(SET_TIME_FOWARD);    // refresh rate of the PID
    pid.SetMode(AUTOMATIC);

    pinMode(EN_R, OUTPUT);
    pinMode(EN_L, OUTPUT);
    pinMode(MR_A1, OUTPUT);
    pinMode(MR_A2, OUTPUT);
    pinMode(ML_A1, OUTPUT);
    pinMode(ML_A2, OUTPUT);
}

// motor function left(val : speed value)
void Motor::ML(int16_t val)
{
    val = constrain(val, -255, 255);

    digitalWrite(ML_A1, (val > 0) ? HIGH : LOW);
    digitalWrite(ML_A2, (val > 0) ? LOW : HIGH);
    analogWrite(EN_L, abs(val));

}

// motor function right(val : speed value)
void Motor::MR(int16_t val)
{
    val = constrain(val, -255, 255);

    digitalWrite(MR_A1, (val > 0) ? HIGH : LOW);
    digitalWrite(MR_A2, (val > 0) ? LOW : HIGH);
    analogWrite(EN_R, abs(val)); // give pwm signal to motor enable

}

void Motor::motorWrite(int16_t leftSpeed, int16_t rightSpeed)
{

    if (leftSpeed == rightSpeed)
    {
        tunning(leftSpeed, rightSpeed);
        updateOutput();

        ML(leftSpeed - output);
        MR(rightSpeed + output);
        return;
    }
    else
    {
        goingStraight = false;
    }

    ML(leftSpeed);
    MR(rightSpeed);
}

// update the output varible 
// update the angle and use it with pid
double Motor::updateOutput(){

    if (!goingStraight)
    {
        gyro.updateGyro();
        setPoint = (double)gyro.getAngle();
        goingStraight = true;
    }
    else
        goingStraight = true;

    gyro.updateGyro();
    input = (double)gyro.getAngle();

    pid.Compute();

}

// use a linear mapping to get the kp, ki and kd values
// for the given speed
void Motor::tunning(int16_t leftSpeed, int16_t rightSpeed){
    
    double avg = (leftSpeed + rightSpeed)/2.0;
    double kp = avg * KP_RATE + KP_FOWARD;
    double ki = avg * KI_RATE + KI_FOWARD;
    double kd = avg * KD_RATE + KD_FOWARD;

    pid.SetTunings(kp, ki, kd);
}

void pulse(int pulsetime, int time)
{
    digitalWrite(ML_A1, HIGH);
    digitalWrite(ML_A2, LOW);
    digitalWrite(MR_A1, HIGH);
    digitalWrite(MR_A2, LOW);

    for (int i = 0; i < time; i++)
    {
        digitalWrite(EN_L, HIGH);
        digitalWrite(EN_R, HIGH);
        delayMicroseconds(pulsetime / 10);

        digitalWrite(EN_L, LOW);
        digitalWrite(EN_R, LOW);
        delayMicroseconds(pulsetime * 9 / 10);
    }
}