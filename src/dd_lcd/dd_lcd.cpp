#include <Arduino.h>
#include <LiquidCrystal.h>

#include "dd_lcd.h"

static LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN,
                          LCD_D4_PIN, LCD_D5_PIN,
                          LCD_D6_PIN, LCD_D7_PIN);

void ddLcdSetup() {
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.clear();
}

void ddLcdClear() {
  lcd.clear();
}

void ddLcdSetCursor(uint8_t col, uint8_t row) {
  lcd.setCursor(col, row);
}

void ddLcdPrint(const char* text) {
  lcd.print(text);
}

void ddLcdPrintChar(char c) {
  lcd.write(c);
}
