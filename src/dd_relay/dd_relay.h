#ifndef DD_RELAY_H
#define DD_RELAY_H

#include <Arduino.h>

// driver for relay (binary actuator, ON/OFF)

void ddRelayInit(uint8_t pin);
void ddRelayOn(uint8_t pin);
void ddRelayOff(uint8_t pin);
void ddRelaySetState(uint8_t pin, bool state);
bool ddRelayGetState(uint8_t pin);

#endif
