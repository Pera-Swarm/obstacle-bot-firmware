
#include <Arduino.h>
#include "define.h"

void setup_motors();

void ML(int8_t val);
void MR(int8_t val);

void motorWrite(int8_t leftSpeed, int8_t rightSpeed);
void pulse(int pulsetime, int time);