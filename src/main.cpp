#include <Arduino.h>

#include "lab_0_app/lab_0_app.h"
#include "lab_1_1_app/lab_1_1_app.h"
#include "lab_1_2_app/lab_1_2_app.h"
#include "lab_3_1_app/lab_3_1_app.h"
#include "lab_3_2_app/lab_3_2_app.h"

enum class AppSelection {
  Lab0,
  Lab1_1,
  Lab1_2,
  Lab3_1,
  Lab3_2
};

const AppSelection kActiveApp = AppSelection::Lab3_2;

void setup() {
  switch (kActiveApp) {
    case AppSelection::Lab0:   lab0AppSetup();   break;
    case AppSelection::Lab1_1: lab1_1AppSetup(); break;
    case AppSelection::Lab1_2: lab1_2AppSetup(); break;
    case AppSelection::Lab3_1: lab3_1AppSetup(); break;
    case AppSelection::Lab3_2: lab3_2AppSetup(); break;
  }
}

void loop() {
  switch (kActiveApp) {
    case AppSelection::Lab0:   lab0AppLoop();   break;
    case AppSelection::Lab1_1: lab1_1AppLoop(); break;
    case AppSelection::Lab1_2: lab1_2AppLoop(); break;
    case AppSelection::Lab3_1: lab3_1AppLoop(); break;
    case AppSelection::Lab3_2: lab3_2AppLoop(); break;
  }
}
