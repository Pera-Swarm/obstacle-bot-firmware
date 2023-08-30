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

    Serial.print(" | >>>>>> ");
    Serial.print(val);
}

// motor function right(val : speed value)
void Motor::MR(int16_t val)
{
    val = constrain(val, -255, 255);

    digitalWrite(MR_A1, (val > 0) ? HIGH : LOW);
    digitalWrite(MR_A2, (val > 0) ? LOW : HIGH);
    analogWrite(EN_R, abs(val)); // give pwm signal to motor enable

    
    Serial.print(" | >>>>>> ");
    Serial.println(val);
}

void Motor::motorWrite(int8_t leftSpeed, int8_t rightSpeed)
{

    if (leftSpeed == rightSpeed)
    {
        updateOutput();

        Serial.print("> ");
        Serial.print(setPoint);
        Serial.print(" >> ");
        Serial.print(input);
        Serial.print(" >>>> ");
        Serial.print(output);
        Serial.print(" >>>>>> ");
        Serial.print(leftSpeed - output);
        Serial.print(" >>>>>> ");
        Serial.print(rightSpeed + output);

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