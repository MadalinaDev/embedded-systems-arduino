#ifndef DD_LCD_H
#define DD_LCD_H

#include <Arduino.h>

#define LCD_RS_PIN 12
#define LCD_EN_PIN 11
#define LCD_D4_PIN 5
#define LCD_D5_PIN 4
#define LCD_D6_PIN 3
#define LCD_D7_PIN 2
#define LCD_COLS   16
#define LCD_ROWS   2

void ddLcdSetup();
void ddLcdClear();
void ddLcdSetCursor(uint8_t col, uint8_t row);
void ddLcdPrint(const char* text);
void ddLcdPrintChar(char c);

#endif
