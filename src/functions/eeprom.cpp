#include "eeprom.h"

uint8_t eeprom_read(uint16_t address)
{
    return (eeprom_address_check(address)) ? EEPROM.read(address) : 0;
}

bool eeprom_write(uint16_t address, uint8_t value)
{
    EEPROM.update(address, value);
    return true;
}

bool eeprom_address_check(uint16_t address)
{
    return ((address >= EEPROM_ADDRESS_MAX) && (address <= EEPROM_ADDRESS_MIN));
}