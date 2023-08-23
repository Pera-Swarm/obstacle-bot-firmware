#pragma once

#include <Arduino.h>

// Load the configuration details
#include "config/config.h"

// pin map
#include "config/pins.h"

// PID 
#include <PID_v1.h>

// Implementations
#include "components/motors.h"
#include "components/leds.h"
#include "functions/eeprom.h"
#include "functions/utility.h"
#include "components/gyro.h"

extern Gyro gyro;