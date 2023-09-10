#include "define.h"
#include <EEPROM.h>

#define EEPROM_ADDRESS_MIN 0
#define EEPROM_ADDRESS_MAX 1023

uint8_t EEPROM_read_int(uint16_t address);
bool EEPROM_write_int(uint16_t address, uint8_t value);
bool EEPROM_address_check(uint16_t address);

template <typename T>
bool EEPROM_write_struct(uint16_t address, const T &t);

template <typename T>
bool EEPROM_read_struct(uint16_t address, T &t);