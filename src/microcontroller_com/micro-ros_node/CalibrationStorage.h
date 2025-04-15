#ifndef CALIBRATION_STORAGE_H
#define CALIBRATION_STORAGE_H


#include <EEPROM.h>

class CalibrationStorage {
public:
    static const int FLOAT_COUNT = 12;
    static const int EEPROM_START_ADDR = 0;
    static const int EEPROM_FLAG_ADDR = 1000;
    static const uint8_t EEPROM_VALID_FLAG = 0xAA;

    CalibrationStorage();

    void load();
    void save();
    void set(const float data[FLOAT_COUNT]);
    void get(float out_data[FLOAT_COUNT]);
    float getAt(int index) const;
    void setAt(int index, float value);

private:
    float values[FLOAT_COUNT];
};

#endif