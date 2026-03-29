#include <Arduino.h>
#include "dd_relay.h"

void ddRelayInit(uint8_t pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void ddRelayOn(uint8_t pin) {
    digitalWrite(pin, HIGH);
}

void ddRelayOff(uint8_t pin) {
    digitalWrite(pin, LOW);
}

void ddRelaySetState(uint8_t pin, bool state) {
    digitalWrite(pin, state ? HIGH : LOW);
}

bool ddRelayGetState(uint8_t pin) {
    return (digitalRead(pin) == HIGH);
}
