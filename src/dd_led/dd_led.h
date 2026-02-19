#ifndef DD_LED_H
#define DD_LED_H

#define LED_PIN 8

void ddLedSetup();
void ddLedToggle();
void ddLedOn();
void ddLedOff();

void ddLedInitPin(uint8_t pin);
void ddLedOnPin(uint8_t pin);
void ddLedOffPin(uint8_t pin);

#endif 