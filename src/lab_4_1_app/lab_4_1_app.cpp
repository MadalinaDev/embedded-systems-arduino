#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>

#include "lab_4_1_app.h"
#include "../dd_temp_sensor/dd_temp_sensor.h"
#include "../dd_led/dd_led.h"

// redirect stdout to serial so we can use printf (STDIO)
static FILE serialOut;

static int serialPutchar(char c, FILE *stream) {
    (void)stream;
    if (c == '\n') Serial.write('\r');
    Serial.write(c);
    return 0;
}

// pins used
static const uint8_t kSensorPin   = A0;   // potentiometer goes here
static const uint8_t kGreenLedPin = 9;    // green = normal
static const uint8_t kRedLedPin   = 10;   // red = alert

// how often each task runs (in ms)
#define REC_ACQUISITION  50    // read sensor every 50ms
#define REC_THRESHOLD    50    // check threshold every 50ms
#define REC_DISPLAY      500   // print report every 500ms

// threshold settings for temperature alert
#define TEMP_THRESHOLD   30.0f   // base threshold in celsius
#define TEMP_HYSTERESIS  1.0f    // +/- 1 degree to avoid flickering
#define THRESH_HIGH      (TEMP_THRESHOLD + TEMP_HYSTERESIS)   // 31 C to trigger
#define THRESH_LOW       (TEMP_THRESHOLD - TEMP_HYSTERESIS)   // 29 C to clear
#define ALERT_DEBOUNCE   5       // need 5 consecutive reads to confirm state change

// mutex to protect shared variables between tasks
static SemaphoreHandle_t xDataMutex;

// shared data - all tasks read/write these through the mutex
static volatile int   sharedRawAdc      = 0;
static volatile float sharedTempC       = 0.0f;
static volatile bool  sharedAlertActive = false;
static volatile int   sharedDbCount     = 0;

/*
 * Task 1 - Sensor Acquisition
 * Reads raw ADC value from the potentiometer on A0,
 * converts it to temperature using the driver, and stores
 * the result in shared variables.
 * Runs every 50ms.
 */
static void taskAcquisition(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        int raw    = ddTempSensorReadRaw(kSensorPin);
        float temp = ddTempSensorRawToTempC(raw);

        // save to shared data with mutex protection
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        sharedRawAdc = raw;
        sharedTempC  = temp;
        xSemaphoreGive(xDataMutex);

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(REC_ACQUISITION));
    }
}

/*
 * Task 2 - Threshold Alerting with Hysteresis
 * Checks if the temperature has crossed the threshold.
 * Uses hysteresis (two different thresholds for on/off)
 * to prevent rapid switching when temp is near the edge.
 * Also uses debouncing - state only changes after 5
 * consecutive same readings, similar to button debounce.
 * Controls green/red LEDs based on alert state.
 */
static void taskThresholdAlert(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    bool alertState    = false;   // current confirmed state
    bool pendingState  = false;   // what the next state might be
    int  debounceCount = 0;

    for (;;) {
        // get current temperature
        float temp;
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        temp = sharedTempC;
        xSemaphoreGive(xDataMutex);

        // hysteresis logic:
        // if we're already in alert -> only clear when temp drops below 29 C
        // if we're normal -> only trigger when temp goes above 31 C
        // this prevents rapid toggling around 30 C
        bool rawTrigger;
        if (alertState) {
            rawTrigger = (temp >= THRESH_LOW);
        } else {
            rawTrigger = (temp >= THRESH_HIGH);
        }

        // debounce: count consecutive same readings
        if (rawTrigger != pendingState) {
            pendingState  = rawTrigger;
            debounceCount = 0;
        }

        debounceCount++;

        // only change state after ALERT_DEBOUNCE consecutive confirmations
        if (debounceCount >= ALERT_DEBOUNCE && alertState != pendingState) {
            alertState    = pendingState;
            debounceCount = 0;
        }

        // update LEDs - green for normal, red for alert
        if (alertState) {
            ddLedOnPin(kRedLedPin);
            ddLedOffPin(kGreenLedPin);
        } else {
            ddLedOnPin(kGreenLedPin);
            ddLedOffPin(kRedLedPin);
        }

        // publish alert state so display task can read it
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        sharedAlertActive = alertState;
        sharedDbCount     = debounceCount;
        xSemaphoreGive(xDataMutex);

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(REC_THRESHOLD));
    }
}

/*
 * Task 3 - Display & Reporting via STDIO (printf)
 * Prints a formatted report every 500ms showing
 * raw ADC, temperature, thresholds, debounce counter,
 * and current alert state.
 */
static void taskDisplayReport(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        int   raw;
        float temp;
        bool  alert;
        int   dbCnt;

        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        raw   = sharedRawAdc;
        temp  = sharedTempC;
        alert = sharedAlertActive;
        dbCnt = sharedDbCount;
        xSemaphoreGive(xDataMutex);

        // format floats since AVR printf doesn't support %f
        char tempStr[10], hiStr[10], loStr[10];
        dtostrf(temp, 6, 2, tempStr);
        dtostrf(THRESH_HIGH, 5, 1, hiStr);
        dtostrf(THRESH_LOW, 5, 1, loStr);

        // print report using STDIO printf
        printf("=== Binary Threshold Report ===\n");
        printf("Raw ADC:       %d\n", raw);
        printf("Temperature:   %s C\n", tempStr);
        printf("Thresh High:   %s C\n", hiStr);
        printf("Thresh Low:    %s C\n", loStr);
        printf("Debounce:      %d/%d\n", dbCnt, ALERT_DEBOUNCE);
        printf("State:         %s\n", alert ? "!! ALERT !!" : "NORMAL");
        printf("================================\n");

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(REC_DISPLAY));
    }
}

// setup and start the application
void lab4_1AppSetup() {
    Serial.begin(9600);

    // setup STDIO - redirect stdout to serial port so printf works
    fdev_setup_stream(&serialOut, serialPutchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &serialOut;

    // init hardware
    ddTempSensorSetup(kSensorPin);
    ddLedInitPin(kGreenLedPin);
    ddLedInitPin(kRedLedPin);

    // create mutex for shared data protection
    xDataMutex = xSemaphoreCreateMutex();

    char hiStr[10], loStr[10];
    dtostrf(THRESH_HIGH, 5, 1, hiStr);
    dtostrf(THRESH_LOW, 5, 1, loStr);

    printf("Lab 4.1 - Binary Threshold Alerting (FreeRTOS)\n");
    printf("Threshold: %s / %s C  Debounce: %d reads\n", loStr, hiStr, ALERT_DEBOUNCE);

    // create FreeRTOS tasks with different priorities
    // higher number = higher priority
    xTaskCreate(taskAcquisition,     "Acq",     128, NULL, 3, NULL);
    xTaskCreate(taskThresholdAlert,  "Thresh",  128, NULL, 2, NULL);
    xTaskCreate(taskDisplayReport,   "Display", 256, NULL, 1, NULL);

    // start the scheduler - this never returns
    vTaskStartScheduler();
}

void lab4_1AppLoop() {
    // empty - FreeRTOS scheduler handles everything
}
