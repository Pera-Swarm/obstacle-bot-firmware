#include "eeprom.h"

uint8_t eeprom_read_int(uint16_t address)
{
    return (eeprom_address_check(address)) ? EEPROM.read(address) : 0;
}

template <typename T>
bool eeprom_read_struct(uint16_t address, T &t)
{
    if (eeprom_address_check(address))
    {
        EEPROM.get(address, t);
        return true;
    }
    else
        return false;
}

bool eeprom_write_int(uint16_t address, uint8_t value)
{
    EEPROM.update(address, value);
    return true;
}

template <typename T>
bool eeprom_write_struct(uint16_t address, const T &t)
{
    if (eeprom_address_check(address))
    {
        EEPROM.put(address, t);
        return true;
    }
    else
        return false;
}

bool eeprom_address_check(uint16_t address)
{
    return ((address <= EEPROM_ADDRESS_MAX) && (address >= EEPROM_ADDRESS_MIN));
}

// Explicit instantiation for the types you expect to use
template bool eeprom_read_struct(uint16_t address, PID_CONST &t);
template bool eeprom_write_struct(uint16_t address, const PID_CONST &t);