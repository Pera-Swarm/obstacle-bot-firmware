#include "define.h"

// create Motor instance
Motor motor;

const int MPU = 0x68;             // MPU6050 I2C address
float angle;                                  // Gyro angle
float GyroErrorX;                             // Gyro error

// Gyro configuration
Gyro gyro(MPU, &angle, &GyroErrorX);

const float turningThresh = 0.15; // threshold to stop turning
const double distThresh = 20;     // threshold to stop moving

bool idflag = false;
String id = "";
double arr[3]{};   // arr to hold startAngle, travelDis, endAngle
int idx = 0;       // index to track the arr index
bool good = false; // bool to check the correct id

// json decoded
double startAngle, endAngle, travelDis;

// PID variables
double Setpoint, Input, Output;
bool newData = false;

// PID configuration
PID pid(&Input, &Output, &Setpoint, 16, 0, 0.23, DIRECT);

// id of the bot
// TODO: Store this in the EEPROM of the microcontroller
String myID = "1";

// creating software serial object

// variables to hold temp data
String reciveStr = "";

bool turningDone = false; // flag true if tuning is done
bool movingDone = false;  // flag true if robot at the destination
double prvstartAngle = 0; // vaiable used to track start angle changes
double spd = 125;         // speed of the movements: [-255, 255]

int tcount = 0;
double dirCorrection = -1;
double prevDist = 0;

void dataDecoder(char c)
{
    LED(4);        // red
    if (c == '\n') // if the endline char
    {
        idflag = true; // start to read the id
        good = false;  // id is not good
        idx = 0;       // reset the index
    }
    else
    {
        if (c == ',') // if comma found
        {
            if (good) // if id is good
            {

                arr[idx] = +id.toDouble(); // update the arr
                if (idx == 2)
                {
                    newData = true; // set the newdata flag
                    tcount = 0;     // when tcount < delay_constant the motor PID will start

                    //          Serial.println("data recieved");
                    startAngle = arr[0]; // do what you want
                    endAngle = arr[2];
                    travelDis = arr[1];
                    //          Serial.println("data:" + String(st  artAngle) + " , " + String(endAngle) + " , " + String(travelDis) + ", " + String(angle));
                }
                idx = (idx + 1) % 3; // increment the index
            }

            if (idflag) // if id is getting
            {
                if (id == myID)
                {
                    LED(1);      // blue
                    good = true; // id is good
                    delay(20);
                    LED(0); // blue
                }
            }

            id = "";        // reset the id
            idflag = false; // id reading done`

        }
        else
            id += c; // append char to the id
    }

    if (Serial.available() > 0)
    {
        dataDecoder(Serial.read());
    }
}

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
            dataDecoder(Serial.read());
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
        dataDecoder(Serial.read()); // parsing the json string
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
    pid.SetOutputLimits(-255, 255); // limits of the PID output
    pid.SetSampleTime(20);          // refresh rate of the PID
    pid.SetMode(AUTOMATIC);

    motor.setup_motors();

    // begining the serial commiunication
    Serial.begin(9600);

    Setpoint = 0;

    gyro.calculate_IMU_error(); // calculate the Gyro module error
    delay(20);

    intShow();
    Serial.println("Bot initiated");
}

void loop()
{
    // Serial.println("Loop");
    // motor.motorWrite(100, 100);
    // delay(1000);
    // motor.motorWrite(-100, -100);
    // delay(1000);
    // motor.motorWrite(0, 0);
    // delay(500);

    // algorithm();
}
