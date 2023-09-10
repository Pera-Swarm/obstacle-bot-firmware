#include "eeprom.h"

uint8_t EEPROM_read_int(uint16_t address)
{
    return (EEPROM_address_check(address)) ? EEPROM.read(address) : 0;
}

template <typename T>
bool EEPROM_read_struct(uint16_t address, T &t)
{
    if (EEPROM_address_check(address))
    {
        EEPROM.get(address, t);
        return true;
    }
    else
        return false;
}

bool EEPROM_write_int(uint16_t address, uint8_t value)
{
    EEPROM.update(address, value);
    return true;
}

template <typename T>
bool EEPROM_write_struct(uint16_t address, const T &t)
{
    if (EEPROM_address_check(address))
    {
        EEPROM.put(address, t);
        return true;
    }
    else
        return false;
}

bool EEPROM_address_check(uint16_t address)
{
    return ((address <= EEPROM_ADDRESS_MAX) && (address >= EEPROM_ADDRESS_MIN));
}

// Explicit instantiation for the types you expect to use
template bool EEPROM_read_struct(uint16_t address, PID_CONST &t);
template bool EEPROM_write_struct(uint16_t address, const PID_CONST &t);