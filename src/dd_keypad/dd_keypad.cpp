#include <Arduino.h>

#include "dd_keypad.h"

static const uint8_t rowPins[KEYPAD_ROWS] = {
  KEYPAD_ROW1_PIN, KEYPAD_ROW2_PIN,
  KEYPAD_ROW3_PIN, KEYPAD_ROW4_PIN
};

static const uint8_t colPins[KEYPAD_COLS] = {
  KEYPAD_COL1_PIN, KEYPAD_COL2_PIN,
  KEYPAD_COL3_PIN, KEYPAD_COL4_PIN
};

static const char keys[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

void ddKeypadSetup() {
  for (uint8_t i = 0; i < KEYPAD_ROWS; i++) {
    pinMode(rowPins[i], OUTPUT);
    digitalWrite(rowPins[i], HIGH);
  }
  for (uint8_t i = 0; i < KEYPAD_COLS; i++) {
    pinMode(colPins[i], INPUT_PULLUP);
  }
}

char ddKeypadGetKey() {
  for (uint8_t r = 0; r < KEYPAD_ROWS; r++) {
    digitalWrite(rowPins[r], LOW);
    delayMicroseconds(50);

    for (uint8_t c = 0; c < KEYPAD_COLS; c++) {
      if (digitalRead(colPins[c]) == LOW) {
        char key = keys[r][c];
        delay(20);
        while (digitalRead(colPins[c]) == LOW) {
          delay(10);
        }
        delay(20);
        digitalWrite(rowPins[r], HIGH);
        return key;
      }
    }

    digitalWrite(rowPins[r], HIGH);
  }

  return 0;
}

char ddKeypadWaitKey() {
  char key = 0;
  while (key == 0) {
    key = ddKeypadGetKey();
    delay(10);
  }
  return key;
}
