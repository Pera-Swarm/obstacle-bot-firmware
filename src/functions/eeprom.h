#include "define.h"
#include <EEPROM.h>

#define EEPROM_ADDRESS_MIN 0
#define EEPROM_ADDRESS_MAX 1023

uint8_t eeprom_read(uint16_t address);
bool eeprom_write(uint16_t address, uint8_t value);
bool eeprom_address_check(uint16_t address);