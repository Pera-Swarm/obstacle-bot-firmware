#include "motors.h"

void setup_motors()
{
    pinMode(EN_R, OUTPUT);
    pinMode(EN_L, OUTPUT);
    pinMode(MR_A1, OUTPUT);
    pinMode(MR_A2, OUTPUT);
    pinMode(ML_A1, OUTPUT);
    pinMode(ML_A2, OUTPUT);
}

// motor function left(val : speed value)
void ML(int8_t val)
{
    val = constrain(val, -255, 255);

    digitalWrite(ML_A1, (val > 0) ? HIGH : LOW);
    digitalWrite(ML_A2, (val > 0) ? LOW : HIGH);
    analogWrite(EN_L, abs(val));

    Serial.println(val);
}

// motor function right(val : speed value)
void MR(int8_t val)
{
    val = constrain(val, -255, 255);

    digitalWrite(MR_A1, (val > 0) ? HIGH : LOW);
    digitalWrite(MR_A2, (val > 0) ? LOW : HIGH);
    analogWrite(EN_R, abs(val)); // give pwm signal to motor enable

    Serial.println(val);
}

void motorWrite(int8_t leftSpeed, int8_t rightSpeed)
{
    ML(leftSpeed);
    MR(rightSpeed);
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