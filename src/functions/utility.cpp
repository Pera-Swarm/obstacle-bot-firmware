#include "utility.h"

double radToDegree(double rads)
{
    return (float)(rads * 180 / PI);
}

void intShow()
{
    LED(COLOR_BLUE);
    pulse(200, 400);
    LED(COLOR_GREEN);
    pulse(200, 400);
    pulse(100, 800);
    LED(COLOR_RED);
    pulse(200, 200);
    LED(COLOR_NO);
    delay(1000);
}