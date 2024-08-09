#include "motors.h"

Motor::Motor()
{
}

void Motor::setup_motors()
{
    pid.SetOutputLimits(-255, 255);                // limits of the PID output
    pid.SetSampleTime(pid_const.SET_TIME_FORWARD); // refresh rate of the PID
    pid.SetMode(AUTOMATIC);

    // set the eerpom
    setPIDConstToEEPROM(0.5, 0.005, 0.005, 0.02, 0.005, 0.005, 20, 70, 70, 90);

    // update the pid values from eeprom
    getPIDConstFromEEPROM();

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

    if ((leftSpeed == rightSpeed) && (leftSpeed != 0))
    {
        tunning(leftSpeed, rightSpeed);
        updateOutput();

        ML(leftSpeed - output);
        MR(rightSpeed + output);
        return;
    }
    else
    {
        motorState = RANDOM;
    }

    ML(leftSpeed);
    MR(rightSpeed);
}

// update the output variable
// update the angle and use it with PID
void Motor::updateOutput()
{

    if (motorState != GOING_STRAIGHT)
    {
        updateSetPoint();
        motorState = GOING_STRAIGHT;
    }
    else
        motorState = GOING_STRAIGHT;

    gyro.updateGyro();
    input = (double)gyro.getAngle();

    pid.Compute();
}

// use a linear mapping to get the kp, ki and kd values
// for the given speed
void Motor::tunning(int16_t leftSpeed, int16_t rightSpeed)
{

    double avg = (leftSpeed + rightSpeed) / 2.0;
    double kp = avg * pid_const.KP_RATE + pid_const.KP_FOWARD;
    double ki = avg * pid_const.KI_RATE + pid_const.KI_FOWARD;
    double kd = avg * pid_const.KD_RATE + pid_const.KD_FOWARD;

    pid.SetTunings(kp, ki, kd);
}

// store the pid_const in EEPROM
bool Motor::setPIDConstToEEPROM(double kpForward, double kiForward, double kdForward, double kpRate, double kiRate, double kdRate, int setTimeForward, int turningSpeed, int turningSpeedRes, int angle)
{
    pid_const.KP_FOWARD = kpForward;
    pid_const.KD_FOWARD = kdForward;
    pid_const.KI_FOWARD = kiForward;
    pid_const.KP_RATE = kpRate;
    pid_const.KI_RATE = kiRate;
    pid_const.KD_RATE = kdRate;
    pid_const.SET_TIME_FORWARD = setTimeForward;
    pid_const.TURING_SPEED = turningSpeed;
    pid_const.TURING_SPEED_REVERSE = turningSpeedRes;
    pid_const.ANGLE_90 = angle;

    return EEPROM_write_struct(ADDRESS, pid_const);
}

// get the pid_const from EEPROM
bool Motor::getPIDConstFromEEPROM()
{
    return EEPROM_read_struct(ADDRESS, pid_const);
}

void Motor::updateSetPoint()
{
    gyro.updateGyro();
    setPoint = (double)gyro.getAngle();
}

void Motor::setState(int state)
{
    this->motorState = state;
}

void Motor::stop()
{
    ML(0);
    MR(0);
    setState(STOP);
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

void Motor::turn90DegLeft()
{
    if (motorState != TURNING)
    {
        setPoint = setPoint + pid_const.ANGLE_90;
        motorState = TURNING;
    }

    if (motorState == TURNING)
    {
        gyro.updateGyro();
        double startangle = (double)gyro.getAngle();
        double err = setPoint - startangle;

        if ((err > 1.0) || (err < -1.0))
        {
            input = startangle;
            tunning(pid_const.TURING_SPEED, pid_const.TURING_SPEED);
            pid.Compute();

            ML((err >= -1.0) ? -pid_const.TURING_SPEED : pid_const.TURING_SPEED_REVERSE - output);
            MR((err >= -1.0) ? pid_const.TURING_SPEED : -pid_const.TURING_SPEED_REVERSE + output);
        }
        else
        {
            motorState = STOP;
            stop();
        }
    }
}

void Motor::turn90DegRight()
{
    if (motorState != TURNING)
    {
        setPoint = setPoint - pid_const.ANGLE_90;
        motorState = TURNING;
    }

    if (motorState == TURNING)
    {
        gyro.updateGyro();
        double startangle = (double)gyro.getAngle();
        double err = startangle - setPoint;

        if ((err > 1.0) || (err < -1.0))
        {
            input = startangle;
            tunning(pid_const.TURING_SPEED, pid_const.TURING_SPEED);
            pid.Compute();

            ML((err >= -1.0) ? pid_const.TURING_SPEED : -pid_const.TURING_SPEED_REVERSE - output);
            MR((err >= -1.0) ? -pid_const.TURING_SPEED : pid_const.TURING_SPEED_REVERSE + output);
        }
        else
        {
            motorState = STOP;
            stop();
        }
    }
}
