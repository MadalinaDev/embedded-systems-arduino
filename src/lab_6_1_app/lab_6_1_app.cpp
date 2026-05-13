// Lab 6.1 - ON-OFF Control with Hysteresis (Variant A)
// Reads temperature from a potentiometer-simulated sensor (A0).
// Controls a relay (pin 7) as a heater actuator.
// Set point and hysteresis adjustable via serial commands.
// Displays values on LCD and sends Serial Plotter data.
// FreeRTOS tasks: Acquisition, CommandInput, Control, Display.

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "lab_6_1_app.h"
#include "../dd_temp_sensor/dd_temp_sensor.h"
#include "../dd_relay/dd_relay.h"
#include "../dd_lcd/dd_lcd.h"
#include "../dd_led/dd_led.h"

// redirect stdout to Serial (printf) and stdin from Serial (getchar)
static FILE serialOut;
static FILE serialIn;

static int serialPutchar(char c, FILE *stream) {
    (void)stream;
    if (c == '\n') Serial.write('\r');
    Serial.write(c);
    return 0;
}

static int serialGetchar(FILE *stream) {
    (void)stream;
    while (Serial.available() == 0) taskYIELD();
    return Serial.read();
}

// ─── Pin Definitions ─────────────────────────────────────────────────────────
static const uint8_t kSensorPin   = A0;  // potentiometer → temperature sim
static const uint8_t kRelayPin    = 7;   // relay → heater actuator
static const uint8_t kGreenLedPin = 9;   // green LED: heater OFF
static const uint8_t kRedLedPin   = 10;  // red LED:   heater ON

// ─── Task Periods (ms) ───────────────────────────────────────────────────────
#define REC_ACQUISITION  100   // read sensor every 100 ms
#define REC_CONTROL      100   // evaluate control logic every 100 ms
#define REC_DISPLAY      500   // update LCD + plotter every 500 ms

// ─── Control Parameters ──────────────────────────────────────────────────────
#define SP_DEFAULT   50.0f   // initial set point in °C
#define SP_STEP       1.0f   // SP+/SP- step size in °C
#define SP_MIN        0.0f   // minimum allowed set point
#define SP_MAX      100.0f   // maximum allowed set point
#define HYST_DEFAULT  2.0f   // hysteresis half-band in °C (±)
#define HYST_MIN      0.1f
#define HYST_MAX     20.0f

// ─── FreeRTOS Synchronisation ────────────────────────────────────────────────
static SemaphoreHandle_t xDataMutex;

// ─── Shared State (all protected by xDataMutex) ──────────────────────────────
static volatile float sharedTemp       = 0.0f;   // current temperature (°C)
static volatile float sharedSetPoint   = SP_DEFAULT;
static volatile float sharedHysteresis = HYST_DEFAULT;
static volatile bool  sharedRelay      = false;   // heater on/off

// ─── Serial Input Buffer size ───────────────────────────────────────────────
#define INPUT_BUF_SIZE 32

// ─── Task 1: Sensor Acquisition ──────────────────────────────────────────────
// Reads raw ADC from the potentiometer, converts to °C, and stores the result
// in sharedTemp every 100 ms.
static void taskAcquisition(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        int   raw  = ddTempSensorReadRaw(kSensorPin);
        float temp = ddTempSensorRawToTempC(raw);

        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        sharedTemp = temp;
        xSemaphoreGive(xDataMutex);

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(REC_ACQUISITION));
    }
}

// ─── Task 2: Serial Command Input ────────────────────────────────────────────
// Blocking getchar()-based input (lowest priority task).
// Supported commands:
//   SP+          – increase set point by 1 °C
//   SP-          – decrease set point by 1 °C
//   SP <value>   – set set point directly
//   HY <value>   – set hysteresis half-band (°C)
static void taskCmdInput(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        char buf[INPUT_BUF_SIZE];
        int  i = 0;
        int  c;

        // block here until a full line arrives via getchar()
        while (i < INPUT_BUF_SIZE - 1) {
            c = getchar();
            if (c == '\n' || c == '\r' || c == EOF) break;
            if (c >= 0) buf[i++] = (char)c;
        }
        buf[i] = '\0';

        if (i == 0) continue;

        xSemaphoreTake(xDataMutex, portMAX_DELAY);

        if (strcasecmp(buf, "SP+") == 0) {
            sharedSetPoint += SP_STEP;
            if (sharedSetPoint > SP_MAX) sharedSetPoint = SP_MAX;
            printf("[CMD] SetPoint -> %.1f C\n", sharedSetPoint);
        } else if (strcasecmp(buf, "SP-") == 0) {
            sharedSetPoint -= SP_STEP;
            if (sharedSetPoint < SP_MIN) sharedSetPoint = SP_MIN;
            printf("[CMD] SetPoint -> %.1f C\n", sharedSetPoint);
        } else if (strncasecmp(buf, "SP ", 3) == 0) {
            float val = (float)atof(buf + 3);
            if (val >= SP_MIN && val <= SP_MAX) {
                sharedSetPoint = val;
                printf("[CMD] SetPoint -> %.1f C\n", sharedSetPoint);
            } else {
                printf("[CMD] SP value out of range (%.0f-%.0f)\n",
                       SP_MIN, SP_MAX);
            }
        } else if (strncasecmp(buf, "HY ", 3) == 0) {
            float val = (float)atof(buf + 3);
            if (val >= HYST_MIN && val <= HYST_MAX) {
                sharedHysteresis = val;
                printf("[CMD] Hysteresis -> +/-%.1f C\n", sharedHysteresis);
            } else {
                printf("[CMD] HY value out of range (%.1f-%.0f)\n",
                       HYST_MIN, HYST_MAX);
            }
        } else {
            printf("[CMD] Unknown. Use: SP+ | SP- | SP <val> | HY <val>\n");
        }

        xSemaphoreGive(xDataMutex);
    }
}

// ─── Task 3: ON-OFF Hysteresis Control ───────────────────────────────────────
// Implements the hysteresis control law:
//   relay = ON  when temp < (SP - hyst)    [below lower threshold]
//   relay = OFF when temp > (SP + hyst)    [above upper threshold]
//   relay unchanged                        [inside the hysteresis band]
// Drives the physical relay and status LEDs.
static void taskControl(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        float temp  = sharedTemp;
        float sp    = sharedSetPoint;
        float hyst  = sharedHysteresis;
        bool  relay = sharedRelay;
        xSemaphoreGive(xDataMutex);

        // Hysteresis control law
        float vOff = sp + hyst;   // upper threshold: turn heater OFF
        float vOn  = sp - hyst;   // lower threshold: turn heater ON

        if (temp < vOn) {
            relay = true;    // too cold → start heating
        } else if (temp > vOff) {
            relay = false;   // hot enough → stop heating
        }
        // else: stay in current state (hysteresis prevents chattering)

        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        sharedRelay = relay;
        xSemaphoreGive(xDataMutex);

        // Drive actuator and indicator LEDs
        ddRelaySetState(kRelayPin, relay);
        if (relay) {
            ddLedOnPin(kRedLedPin);
            ddLedOffPin(kGreenLedPin);
        } else {
            ddLedOffPin(kRedLedPin);
            ddLedOnPin(kGreenLedPin);
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(REC_CONTROL));
    }
}

// ─── Task 4: Display ─────────────────────────────────────────────────────────
// Updates the 16×2 LCD every 500 ms and sends Serial Plotter data.
// LCD row 0: "SP:XX.X  T:XX.X"
// LCD row 1: "Heater: ON/OFF  "
// Serial Plotter: SetPoint:XX.X,Value:XX.X,Output:0/1
static void taskDisplay(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        float temp  = sharedTemp;
        float sp    = sharedSetPoint;
        bool  relay = sharedRelay;
        xSemaphoreGive(xDataMutex);

        // LCD row 0
        char buf[17];
        ddLcdSetCursor(0, 0);
        snprintf(buf, sizeof(buf), "SP:%-5.1f T:%-5.1f", sp, temp);
        ddLcdPrint(buf);

        // LCD row 1
        ddLcdSetCursor(0, 1);
        snprintf(buf, sizeof(buf), "Heater: %-7s", relay ? "ON" : "OFF");
        ddLcdPrint(buf);

        // Arduino Serial Plotter format (comma-separated key:value pairs)
        printf("SetPoint:%.1f,Value:%.1f,Output:%d\n",
               sp, temp, relay ? 1 : 0);

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(REC_DISPLAY));
    }
}

// ─── Public API ──────────────────────────────────────────────────────────────
void lab6_1AppSetup() {
    Serial.begin(115200);
    fdev_setup_stream(&serialOut, serialPutchar, nullptr,      _FDEV_SETUP_WRITE);
    fdev_setup_stream(&serialIn,  nullptr,      serialGetchar, _FDEV_SETUP_READ);
    stdout = &serialOut;
    stdin  = &serialIn;

    // Initialise peripherals
    ddTempSensorSetup(kSensorPin);
    ddRelayInit(kRelayPin);
    ddLedInitPin(kGreenLedPin);
    ddLedInitPin(kRedLedPin);
    ddLedOffPin(kGreenLedPin);
    ddLedOffPin(kRedLedPin);
    ddLcdSetup();
    ddLcdClear();

    // Create FreeRTOS mutex
    xDataMutex = xSemaphoreCreateMutex();

    printf("Lab 6.1 - ON-OFF Control with Hysteresis\n");
    printf("SP=%.1f C  HY=+/-%.1f C\n", SP_DEFAULT, HYST_DEFAULT);
    printf("Commands: SP+ | SP- | SP <val> | HY <val>\n");

    // Create tasks: Cmd=lowest (blocking), Acq=highest, Ctrl=3, Disp=2
    xTaskCreate(taskAcquisition, "Acq",  192, nullptr, 4, nullptr);
    xTaskCreate(taskControl,     "Ctrl", 192, nullptr, 3, nullptr);
    xTaskCreate(taskDisplay,     "Disp", 256, nullptr, 2, nullptr);
    xTaskCreate(taskCmdInput,    "Cmd",  256, nullptr, 1, nullptr);

    vTaskStartScheduler();
}

void lab6_1AppLoop() {
    // FreeRTOS scheduler runs; this body is never reached.
}
