#include "Arduino.h"
#include "dd_button.h"

void ddButtonSetup() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
}

int ddButtonIsPressed() {
    if (digitalRead(BUTTON_PIN) == LOW) return 1; else return 0;
}

void ddButtonInitPin(uint8_t pin) {
    pinMode(pin, INPUT_PULLUP);
}

int ddButtonReadPin(uint8_t pin) {
    return (digitalRead(pin) == LOW) ? 1 : 0;
}

int ddButtonReadDebouncedPin(uint8_t pin, uint8_t *debounceCounter, int *stableState) {
    int raw = (digitalRead(pin) == LOW) ? 1 : 0;
    if (raw == *stableState) {
        *debounceCounter = 0;
    } else {
        (*debounceCounter)++;
        if (*debounceCounter >= DEBOUNCE_COUNT) {
            *stableState = raw;
            *debounceCounter = 0;
        }
    }
    return *stableState;
}