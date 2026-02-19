#include <Arduino.h>

#include "lab_0_app/lab_0_app.h"
#include "lab_1_app/lab_1_app.h"

enum class AppSelection {
    Lab0,
    Lab1
};

const AppSelection kActiveApp = AppSelection::Lab1;

void setup() {
    if (kActiveApp == AppSelection::Lab0) {
        lab0AppSetup();
    } else {
        lab1AppSetup();
    }
}

void loop() {
    if (kActiveApp == AppSelection::Lab0) {
        lab0AppLoop();
    } else {
        lab1AppLoop();
    }
}
