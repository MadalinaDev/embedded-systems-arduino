#include "Arduino.h"
#include "dd_led.h"

void ddLedSetup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

void ddLedToggle() {
    if (digitalRead(LED_PIN) == LOW) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

void ddLedOn() {
  digitalWrite(LED_PIN, HIGH);
}

void ddLedOff() {
  digitalWrite(LED_PIN, LOW);
}

void ddLedInitPin(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void ddLedOnPin(uint8_t pin) {
  digitalWrite(pin, HIGH);
}

void ddLedOffPin(uint8_t pin) {
  digitalWrite(pin, LOW);
}