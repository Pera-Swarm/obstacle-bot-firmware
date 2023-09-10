#include "define.h"

// id of the bot
// TODO: Store this in the EEPROM of the microcontroller
String myID = "1";

// create Motor instance
Motor motor;

float angle; // Gyro angle
// Gyro configuration
Gyro gyro(0x68, &angle);

const float turningThresh = 0.15; // threshold to stop turning
const double distThresh = 20;     // threshold to stop moving

// json decoded
double startAngle, endAngle, travelDis;

// PID variables
double Setpoint, Input, Output;
bool newData = false;

// PID configuration
PID pid(&Input, &Output, &Setpoint, 16, 0, 0.23, DIRECT);

bool turningDone = false; // flag true if tuning is done
bool movingDone = false;  // flag true if robot at the destination
double prvstartAngle = 0; // vaiable used to track start angle changes
double spd = 125;         // speed of the movements: [-255, 255]

int tcount = 0;
double dirCorrection = -1;
double prevDist = 0;

// HC-12 comunication config
HC12 hc12(&startAngle, &endAngle, &travelDis, &newData, &tcount, myID);

void turn()
{
    turningDone = false;
    angle = 0;                               // set the current angle to zer0
    Setpoint = -1 * radToDegree(startAngle); // set the setpoint as the startAngle

    prvstartAngle = startAngle; // update the prvstartAngle
                                //  Serial.println("started turning PID " + String(startAngle));
    while (!turningDone)
    {
        LED(2); // green
        if (Serial.available() > 0)
        {
            // parsing the json string
            hc12.dataDecoder(Serial.read());
        }

        if (prvstartAngle != startAngle) // if there any changes in startAngle, set the current angle to zero and set the set point
        {
            Setpoint = -1 * radToDegree(startAngle);
            angle = 0;
            prvstartAngle = startAngle;
        }

        gyro.updateGyro();
        Input = (double)angle;
        pid.Compute();

        // Serial.println(String(startAngle) + ", " + String(Setpoint) + ", " + String(Input) + ", " + String(Output) + ", ");

        motor.motorWrite(-Output, -Output);

        if ((-turningThresh < startAngle) && (turningThresh > startAngle)) // exit form the loop if the startAngle is bounded in threshold
        {
            //      Serial.println("turning done");
            turningDone = true;
        }
        LED(0); // off
    }

    angle = 0;
    motor.motorWrite(0, 0);
}

void algorithm()
{
    if (Serial.available() > 0)
    {
        hc12.dataDecoder(Serial.read()); // parsing the json string
        LED(0);
    }

    // start turning process if the start angle is above the "turningThresh"
    if ((-turningThresh > startAngle) || (turningThresh < startAngle))
    {
        turn();
        LED(0);
    }

    // set the movingDone flag if the robo is at the destination
    if (travelDis < distThresh)
    {
        movingDone = true;
    }
    else
    {
        movingDone = false;
    }

    if ((tcount < 40) && turningDone && newData && !movingDone) // run motors with PID if conditions are satisfied
    {
        Setpoint = 0; // set the gyro setpoint to 0
        gyro.updateGyro();
        Input = (double)angle;
        pid.Compute();
        motor.motorWrite(spd - Output, spd + Output);
    }
    else
    {
        LED(0);
        newData = false;
        motor.motorWrite(0, 0);
    }

    tcount++;
    delay(5);
}

void setup()
{
    // begining the serial commiunication
    Serial.begin(9600);

    pid.SetOutputLimits(-255, 255); // limits of the PID output
    pid.SetSampleTime(20);          // refresh rate of the PID
    pid.SetMode(AUTOMATIC);

    motor.setup_motors();

    Setpoint = 0;

    gyro.calculate_IMU_error(); // calculate the Gyro module error
    delay(20);

    intShow();
    Serial.println("Bot initiated");
}

void loop()
{
    // Serial.println("Loop");
    unsigned long startTime = millis();
    while ((millis() - startTime) < 3000)
    {
        motor.motorWrite(80, 80);
        delay(10);
    }

    startTime = millis();

    while ((millis() - startTime) < 3000)
    {
        motor.motorWrite(-80, -80);
        delay(10);
    }
}
