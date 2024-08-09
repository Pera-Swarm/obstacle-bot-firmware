#include "leds.h"

void setup_leds()
{
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
}

void LED(byte color)
{
    digitalWrite(LED_R, color & 1);
    digitalWrite(LED_G, (color >> 1) & 1);
    digitalWrite(LED_B, (color >> 2) & 1);
}