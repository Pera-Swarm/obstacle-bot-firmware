#include "define.h"
#include <EEPROM.h>

#define EEPROM_ADDRESS_MIN 0
#define EEPROM_ADDRESS_MAX 1023

uint8_t eeprom_read_int(uint16_t address);
bool eeprom_write_int(uint16_t address, uint8_t value);
bool eeprom_address_check(uint16_t address);

template <typename T>
bool eeprom_write_struct(uint16_t address, const T &t);

template <typename T>
bool eeprom_read_struct(uint16_t address, T &t);