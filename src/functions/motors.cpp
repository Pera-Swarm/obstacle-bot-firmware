#include "motors.h"

// motor function left(val : speed value)
void ML(int8_t val)
{
    val = constrain(-255, 255, val);

    digitalWrite(ML_A2, (val > 0) ? HIGH : LOW);
    digitalWrite(ML_A1, (val > 0) ? LOW : HIGH);
    analogWrite(EN_L, abs(val));
}

// motor function right(val : speed value)
void MR(int8_t val)
{
    val = constrain(-255, 255, val);
    {
        val = 255;
    }

    digitalWrite(MR_A1, (val > 0) ? HIGH : LOW);
    digitalWrite(MR_A2, (val > 0) ? LOW : HIGH);
    analogWrite(EN_R, abs(val)); // give pwm signal to motor enable
}

void motorWrite(int8_t leftSpeed, int8_t rightSpeed)
{
    ML(leftSpeed);
    MR(rightSpeed);
}