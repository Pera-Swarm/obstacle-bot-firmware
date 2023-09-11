#include "hc12.h"

HC12::HC12(double *startAngle, double *endAngle, double *travelDis, bool *newData, int *tcount, String myID)
{
    this->startAngle = startAngle;
    this->endAngle = endAngle;
    this->travelDis = travelDis;
    this->myID = myID;
    this->tcount = tcount;
    this->newData = newData;
}

void HC12::listen()
{
    reciveData = "";

    while (Serial.available())
    {
        LED(COLOR_BLUE);
        char c = Serial.read();
        if (c == '\n')
            break;
        reciveData += c;
        Serial.print(c);
    }
}

String HC12::getReceivedData()
{
    return reciveData;
}

void HC12::dataDecoder(char c)
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
                    *newData = true; // set the newdata flag
                    *tcount = 0;     // when tcount < delay_constant the motor PID will start

                    //          Serial.println("data recieved");
                    *startAngle = arr[0]; // do what you want
                    *endAngle = arr[2];
                    *travelDis = arr[1];
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