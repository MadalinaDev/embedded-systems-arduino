#include <Arduino.h>

#include "lab_0_app/lab_0_app.h"
#include "lab_1_1_app/lab_1_1_app.h"
#include "lab_1_2_app/lab_1_2_app.h"

enum class AppSelection {
  Lab0,
  Lab1_1,
  Lab1_2
};

const AppSelection kActiveApp = AppSelection::Lab1_2;

void setup() {
  switch (kActiveApp) {
    case AppSelection::Lab0:   lab0AppSetup();   break;
    case AppSelection::Lab1_1: lab1_1AppSetup(); break;
    case AppSelection::Lab1_2: lab1_2AppSetup(); break;
  }
}

void loop() {
  switch (kActiveApp) {
    case AppSelection::Lab0:   lab0AppLoop();   break;
    case AppSelection::Lab1_1: lab1_1AppLoop(); break;
    case AppSelection::Lab1_2: lab1_2AppLoop(); break;
  }
}
