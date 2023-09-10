#include "hc12.h"

HC12::HC12()
{
}

void HC12::listen()
{
    reciveData = "";

    while (Serial.available())
    {
        reciveData += Serial.read();
    }
}

String HC12::getReceivedData()
{
    return reciveData;
}