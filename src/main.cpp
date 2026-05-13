#include <Arduino.h>

#include "lab_0_app/lab_0_app.h"
#include "lab_1_1_app/lab_1_1_app.h"
#include "lab_1_2_app/lab_1_2_app.h"
#include "lab_3_1_app/lab_3_1_app.h"
#include "lab_3_2_app/lab_3_2_app.h"
#include "lab_4_1_app/lab_4_1_app.h"
#include "lab_4_2_app/lab_4_2_app.h"
#include "lab_5_1_app/lab_5_1_app.h"
#include "lab_5_2_app/lab_5_2_app.h"
#include "lab_6_1_app/lab_6_1_app.h"
#include "lab_6_2_app/lab_6_2_app.h"

enum class AppSelection {
  Lab0,
  Lab1_1,
  Lab1_2,
  Lab3_1,
  Lab3_2,
  Lab4_1,
  Lab4_2,
  Lab5_1,
  Lab5_2,
  Lab6_1,
  Lab6_2
};

const AppSelection kActiveApp = AppSelection::Lab6_2;

void setup() {
  switch (kActiveApp) {
    case AppSelection::Lab0:   lab0AppSetup();   break;
    case AppSelection::Lab1_1: lab1_1AppSetup(); break;
    case AppSelection::Lab1_2: lab1_2AppSetup(); break;
    case AppSelection::Lab3_1: lab3_1AppSetup(); break;
    case AppSelection::Lab3_2: lab3_2AppSetup(); break;
    case AppSelection::Lab4_1: lab4_1AppSetup(); break;
    case AppSelection::Lab4_2: lab4_2AppSetup(); break;
    case AppSelection::Lab5_1: lab5_1AppSetup(); break;
    case AppSelection::Lab5_2: lab5_2AppSetup(); break;
    case AppSelection::Lab6_1: lab6_1AppSetup(); break;
    case AppSelection::Lab6_2: lab6_2AppSetup(); break;
  }
}

void loop() {
  switch (kActiveApp) {
    case AppSelection::Lab0:   lab0AppLoop();   break;
    case AppSelection::Lab1_1: lab1_1AppLoop(); break;
    case AppSelection::Lab1_2: lab1_2AppLoop(); break;
    case AppSelection::Lab3_1: lab3_1AppLoop(); break;
    case AppSelection::Lab3_2: lab3_2AppLoop(); break;
    case AppSelection::Lab4_1: lab4_1AppLoop(); break;
    case AppSelection::Lab4_2: lab4_2AppLoop(); break;
    case AppSelection::Lab5_1: lab5_1AppLoop(); break;
    case AppSelection::Lab5_2: lab5_2AppLoop(); break;
    case AppSelection::Lab6_1: lab6_1AppLoop(); break;
    case AppSelection::Lab6_2: lab6_2AppLoop(); break;
  }
}
