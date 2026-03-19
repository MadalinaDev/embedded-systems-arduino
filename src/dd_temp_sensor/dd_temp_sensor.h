#ifndef DD_TEMP_SENSOR_H
#define DD_TEMP_SENSOR_H

#include <Arduino.h>

// Potentiometer-based temperature simulation
// ADC 0..1023  →  TEMP_MIN..TEMP_MAX  (linear mapping)
#define TEMP_RANGE_MIN  (-20.0f)
#define TEMP_RANGE_MAX  (100.0f)

void  ddTempSensorSetup(uint8_t analogPin);
int   ddTempSensorReadRaw(uint8_t analogPin);
float ddTempSensorRawToTempC(int rawAdc);

#endif
