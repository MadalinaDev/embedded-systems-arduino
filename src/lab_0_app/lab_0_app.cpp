#include <Arduino.h>

#include "lab_0_app.h"

#include "dd_button/dd_button.h"
#include "dd_led/dd_led.h"


void lab0AppSetup() {
  ddLedSetup();
  ddButtonSetup();
}

void lab0AppLoop() {

  // wait for button press
  while (!ddButtonIsPressed()){};
  delay(20);

  // toggle led
  ddLedToggle();

  // wait for button release
  while (ddButtonIsPressed()){};
  delay(20);
}