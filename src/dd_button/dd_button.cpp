#include "Arduino.h"
#include "dd_button.h"

void ddButtonSetup() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
}

int ddButtonIsPressed() {
    if (digitalRead(BUTTON_PIN) == LOW) return 1; else return 0;
}