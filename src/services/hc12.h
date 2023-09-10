#ifndef HC12_H
#define HC12_H

#include "define.h"

class HC12
{
public:
    HC12();

    void listen();
    String getReceivedData();

private:
    String reciveData = "";
};

#endif