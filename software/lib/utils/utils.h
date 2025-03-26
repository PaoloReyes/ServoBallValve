#ifndef UTILS_H
    #define UTILS_H

    #include <Arduino.h>
    #include <EEPROM.h>

    void save_16_bit_value_to_eeprom(uint16_t value, uint8_t address);
    uint16_t read_16_bit_value_from_eeprom(uint8_t address);

    double map_double(double x, double in_min, double in_max, double out_min, double out_max);
    double constrain_double(double x, double min, double max);

#endif