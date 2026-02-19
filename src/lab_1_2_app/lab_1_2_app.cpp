#include <Arduino.h>
#include <stdio.h>

#include "lab_1_2_app.h"

#include "dd_lcd/dd_lcd.h"
#include "dd_keypad/dd_keypad.h"
#include "dd_led/dd_led.h"

// ─── Pin definitions ─────────────────────────────────
static const uint8_t kGreenLedPin = 9;
static const uint8_t kRedLedPin   = 10;

// ─── Secret code ─────────────────────────────────────
static const char    kSecretCode[] = "1234";
static const uint8_t kCodeLength   = 4;

// ─── Input buffer ────────────────────────────────────
static char    codeBuffer[5];
static uint8_t codeIndex;

// ─── STDIO streams (AVR libc) ────────────────────────
static FILE lcdOut;

static int lcdPutchar(char c, FILE *stream) {
  if (c == '\n') {
    ddLcdSetCursor(0, 1);
  } else {
    ddLcdPrintChar(c);
  }
  return 0;
}

// ─── Helper: reset to prompt ─────────────────────────
static void showPrompt() {
  ddLcdClear();
  printf("Enter code:");
  ddLcdSetCursor(0, 1);
  codeIndex = 0;
  memset(codeBuffer, 0, sizeof(codeBuffer));
}

// ═════════════════════════════════════════════════════
void lab1_2AppSetup() {
  ddLcdSetup();
  ddKeypadSetup();
  ddLedInitPin(kGreenLedPin);
  ddLedInitPin(kRedLedPin);

  // Redirect stdout -> LCD via STDIO
  fdev_setup_stream(&lcdOut, lcdPutchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &lcdOut;

  showPrompt();
}

// ═════════════════════════════════════════════════════
void lab1_2AppLoop() {
  char key = ddKeypadGetKey();
  if (key == 0) return;

  if (key == '#') {
    // ── Confirm code ──
    codeBuffer[codeIndex] = '\0';

    ddLcdClear();

    if (strcmp(codeBuffer, kSecretCode) == 0) {
      printf("Access Granted!");
      ddLedOnPin(kGreenLedPin);
      ddLedOffPin(kRedLedPin);
    } else {
      printf("Access Denied!");
      ddLedOffPin(kGreenLedPin);
      ddLedOnPin(kRedLedPin);
    }

    delay(2000);

    ddLedOffPin(kGreenLedPin);
    ddLedOffPin(kRedLedPin);
    showPrompt();

  } else if (key == '*') {
    // ── Clear entry ──
    ddLcdSetCursor(0, 1);
    printf("                ");
    ddLcdSetCursor(0, 1);
    codeIndex = 0;
    memset(codeBuffer, 0, sizeof(codeBuffer));

  } else if (codeIndex < kCodeLength) {
    // ── Accumulate digit (masked) ──
    codeBuffer[codeIndex++] = key;
    ddLcdPrintChar('*');
  }
}
