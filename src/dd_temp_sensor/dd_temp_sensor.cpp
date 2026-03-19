#include <Arduino.h>

#include "dd_temp_sensor.h"

void ddTempSensorSetup(uint8_t analogPin) {
    (void)analogPin;  // Analog pins default to input
}

int ddTempSensorReadRaw(uint8_t analogPin) {
    return analogRead(analogPin);
}

float ddTempSensorRawToTempC(int rawAdc) {
    // Clamp to valid ADC range
    if (rawAdc < 0)    rawAdc = 0;
    if (rawAdc > 1023) rawAdc = 1023;

    // Linear mapping:  ADC 0..1023  →  TEMP_MIN..TEMP_MAX
    return TEMP_RANGE_MIN + (TEMP_RANGE_MAX - TEMP_RANGE_MIN) * ((float)rawAdc / 1023.0f);
}
