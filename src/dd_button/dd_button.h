#ifndef DD_BUTTON_H
#define DD_BUTTON_H

#include <Arduino.h>

#define BUTTON_PIN 12
#define DEBOUNCE_COUNT 5

void ddButtonSetup();
int ddButtonIsPressed();

void ddButtonInitPin(uint8_t pin);
int ddButtonReadPin(uint8_t pin);
int ddButtonReadDebouncedPin(uint8_t pin, uint8_t *debounceCounter, int *stableState);

#endif