#include <Arduino.h>

#include "lab_1_1_app.h"

#include "dd_led/dd_led.h"

namespace {
	const int kBaudRate = 9600;
	String inputBuffer;
}

static void lab1_1ProcessCommand(String cmd) {
	cmd.trim();
	cmd.toLowerCase();

	if (cmd == "led on") {
		ddLedOn();
		Serial.println("OK: LED is ON");
	} else if (cmd == "led off") {
		ddLedOff();
		Serial.println("OK: LED is OFF");
	} else if (cmd.length() > 0) {
		Serial.println("ERROR: Unknown command");
		Serial.println("Valid commands: led on | led off");
	}
}

void lab1_1AppSetup() {
	Serial.begin(kBaudRate);
	ddLedSetup();

	Serial.println("System ready.");
	Serial.println("Type: led on  OR  led off");
}

void lab1_1AppLoop() {
	while (Serial.available()) {
		char c = Serial.read();

		if (c == '\n' || c == '\r') {
			lab1_1ProcessCommand(inputBuffer);
			inputBuffer = "";
		} else {
			inputBuffer += c;
		}
	}
}
