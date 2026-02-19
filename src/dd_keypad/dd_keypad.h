#ifndef DD_KEYPAD_H
#define DD_KEYPAD_H

#include <Arduino.h>

#define KEYPAD_ROWS 4
#define KEYPAD_COLS 4

#define KEYPAD_ROW1_PIN 22
#define KEYPAD_ROW2_PIN 23
#define KEYPAD_ROW3_PIN 24
#define KEYPAD_ROW4_PIN 25

#define KEYPAD_COL1_PIN 26
#define KEYPAD_COL2_PIN 27
#define KEYPAD_COL3_PIN 28
#define KEYPAD_COL4_PIN 29

void ddKeypadSetup();
char ddKeypadGetKey();
char ddKeypadWaitKey();

#endif
