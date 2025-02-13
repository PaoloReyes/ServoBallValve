#include <utils.h>

/// @brief Separate 16-bit value into two bytes and save them to EEPROM
/// @param value 16-bit value to be saved
/// @param address Starting address in EEPROM
void save_16_bit_value_to_eeprom(uint16_t value, uint8_t address) {
    uint8_t low_byte = value & 0xFF;
    uint8_t high_byte = (value >> 8) & 0xFF;
    EEPROM.write(address, low_byte);
    EEPROM.write(address+1, high_byte);
}

/// @brief Read two bytes from EEPROM and combine them into 16-bit value
/// @param address Starting address in EEPROM
/// @return 16-bit value
uint16_t read_16_bit_value_from_eeprom(uint8_t address) {
    uint8_t low_byte = EEPROM.read(address);
    uint8_t high_byte = EEPROM.read(address+1);
    return (high_byte << 8) | low_byte;
}

/// @brief Map a value from one range to another with floating point precision
/// @param x Value to be mapped
/// @param in_min Minimum value of input range
/// @param in_max Maximum value of input range
/// @param out_min Minimum value of output range
/// @param out_max Maximum value of output range
/// @return Mapped value
double map_double(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/// @brief Constrain a value to a range
/// @param x Value to be constrained
/// @param min Minimum value of range
/// @param max Maximum value of range
/// @return Constrained value
double constrain_double(double x, double min, double max) {
    if (x < min) {
        return min;
    } else if (x > max) {
        return max;
    } else {
        return x;
    }
}