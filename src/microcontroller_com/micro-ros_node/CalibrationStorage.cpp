#include "CalibrationStorage.h"
#include <cstring> // for memcmp

CalibrationStorage::CalibrationStorage() {
    // Carga automática al instanciar
    load();
}

void CalibrationStorage::load() {
    uint8_t flag;
    EEPROM.get(EEPROM_FLAG_ADDR, flag);

    if (flag == EEPROM_VALID_FLAG) {
        for (int i = 0; i < FLOAT_COUNT; ++i) {
            EEPROM.get(EEPROM_START_ADDR + i * sizeof(float), values[i]);
        }
    } else {
        // Valores por defecto
        float defaults[FLOAT_COUNT] = {
            -1.08333, -1.06667, -1.07778,
            -1.03333,  0.97778,  1.01111,
             1.03333,  1.05556,  1.07778,
             1.07500, -1.07778, -1.00000
        };
        memcpy(values, defaults, sizeof(values));
        save(); // Guarda los valores por defecto
    }
}

void CalibrationStorage::save() {
    // Solo guarda si hay diferencias (evitar desgaste)
    float current[FLOAT_COUNT];
    for (int i = 0; i < FLOAT_COUNT; ++i) {
        EEPROM.get(EEPROM_START_ADDR + i * sizeof(float), current[i]);
    }

    if (memcmp(current, values, sizeof(values)) != 0) {
        for (int i = 0; i < FLOAT_COUNT; ++i) {
            EEPROM.put(EEPROM_START_ADDR + i * sizeof(float), values[i]);
        }
        EEPROM.put(EEPROM_FLAG_ADDR, EEPROM_VALID_FLAG);
    }
}

void CalibrationStorage::set(const float data[FLOAT_COUNT]) {
    memcpy(values, data, sizeof(values));
}

void CalibrationStorage::get(float out_data[FLOAT_COUNT]) {
    memcpy(out_data, values, sizeof(values));
}

float CalibrationStorage::getAt(int index) const {
    if (index >= 0 && index < FLOAT_COUNT) {
        return values[index];
    }
    return 0.0f;
}

void CalibrationStorage::setAt(int index, float value) {
    if (index >= 0 && index < FLOAT_COUNT) {
        values[index] = value;
    }
}