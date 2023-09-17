#ifndef HC12_H
#define HC12_H

#include <Arduino.h>
#include "components/leds.h"

class HC12
{
public:
    HC12(double *startAngle, double *endAngle, double *travelDis, bool *newData, int *tcount, int myID);

    void listen();
    String getReceivedData();

    void dataDecoder(char c);

private:
    int myID = 1;

    String reciveData = "";

    bool idflag = false;
    String id = "";
    double arr[3]{};   // arr to hold startAngle, travelDis, endAngle
    int idx = 0;       // index to track the arr index
    bool good = false; // bool to check the correct id

    bool *newData;
    int *tcount;

    // json decoded
    double *startAngle, *endAngle, *travelDis;
};

#endif