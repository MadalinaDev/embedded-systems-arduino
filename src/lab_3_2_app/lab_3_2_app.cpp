#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include "lab_3_2_app.h"
#include "../dd_led/dd_led.h"

// ─── Pin Definitions (same as bare-metal) ────────────
static const uint8_t kButtonPin    = 6;
static const uint8_t kYellowLedPin = 8;
static const uint8_t kGreenLedPin  = 9;
static const uint8_t kRedLedPin    = 10;

// ─── Recurrence & Offset (ms) ────────────────────────
#define REC_BTN     20
#define REC_REPORT  10000

#define OFFS_BTN    0
#define OFFS_STATS  10
#define OFFS_REPORT 20

// ─── Synchronization Primitives ──────────────────────
static SemaphoreHandle_t xPressEventSemaphore;   // binary semaphore: Task1 -> Task2
static SemaphoreHandle_t xStatsMutex;            // mutex: protects shared statistics

// ─── Shared Data with Mutex Protection ───────────────
static unsigned long sharedPressDuration = 0;
static bool          sharedIsLongPress   = false;

// Statistics (protected by xStatsMutex)
static unsigned int  statsTotalPresses  = 0;
static unsigned int  statsShortPresses  = 0;
static unsigned int  statsLongPresses   = 0;
static unsigned long statsTotalShortDur = 0;
static unsigned long statsTotalLongDur  = 0;

// Setter with mutex (called by Task 1)
static void setPressDuration(unsigned long duration, bool isLong) {
    xSemaphoreTake(xStatsMutex, portMAX_DELAY);
    sharedPressDuration = duration;
    sharedIsLongPress   = isLong;
    xSemaphoreGive(xStatsMutex);
}

// Getter with mutex (called by Task 2)
static void getPressDuration(unsigned long *duration, bool *isLong) {
    xSemaphoreTake(xStatsMutex, portMAX_DELAY);
    *duration = sharedPressDuration;
    *isLong   = sharedIsLongPress;
    xSemaphoreGive(xStatsMutex);
}

// Update stats with mutex (called by Task 2)
static void updateStats(unsigned long duration, bool isLong) {
    xSemaphoreTake(xStatsMutex, portMAX_DELAY);
    statsTotalPresses++;
    if (isLong) {
        statsLongPresses++;
        statsTotalLongDur += duration;
    } else {
        statsShortPresses++;
        statsTotalShortDur += duration;
    }
    xSemaphoreGive(xStatsMutex);
}

// Get and reset stats with mutex (called by Task 3)
static void getAndResetStats(unsigned int *tp, unsigned int *sp, unsigned int *lp,
                             unsigned long *tsd, unsigned long *tld) {
    xSemaphoreTake(xStatsMutex, portMAX_DELAY);
    *tp  = statsTotalPresses;
    *sp  = statsShortPresses;
    *lp  = statsLongPresses;
    *tsd = statsTotalShortDur;
    *tld = statsTotalLongDur;
    statsTotalPresses  = 0;
    statsShortPresses  = 0;
    statsLongPresses   = 0;
    statsTotalShortDur = 0;
    statsTotalLongDur  = 0;
    xSemaphoreGive(xStatsMutex);
}

// ─── Task 1: Button Detection & Duration (FreeRTOS) ──
static void taskBtnDetectRTOS(void *pvParameters) {
    (void)pvParameters;

    // Apply initial offset
    if (OFFS_BTN > 0) {
        vTaskDelay(pdMS_TO_TICKS(OFFS_BTN));
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();

    bool prevState         = false;
    unsigned long pressStart = 0;

    for (;;) {
        bool currentState = (digitalRead(kButtonPin) == LOW);

        if (currentState && !prevState) {
            // Button just pressed
            pressStart = millis();
            ddLedOffPin(kGreenLedPin);
            ddLedOffPin(kRedLedPin);
        } else if (!currentState && prevState) {
            // Button just released
            unsigned long duration = millis() - pressStart;
            bool isLong = (duration >= 500);

            if (isLong) {
                ddLedOnPin(kRedLedPin);
                ddLedOffPin(kGreenLedPin);
            } else {
                ddLedOnPin(kGreenLedPin);
                ddLedOffPin(kRedLedPin);
            }

            // Write shared data and signal Task 2 via semaphore
            setPressDuration(duration, isLong);
            xSemaphoreGive(xPressEventSemaphore);
        }

        prevState = currentState;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(REC_BTN));
    }
}

// ─── Task 2: Statistics & Yellow LED Blink (FreeRTOS) ─
static void taskStatsRTOS(void *pvParameters) {
    (void)pvParameters;

    // Apply initial offset
    if (OFFS_STATS > 0) {
        vTaskDelay(pdMS_TO_TICKS(OFFS_STATS));
    }

    for (;;) {
        // Block until Task 1 signals a press event (binary semaphore)
        if (xSemaphoreTake(xPressEventSemaphore, portMAX_DELAY) == pdTRUE) {
            unsigned long duration;
            bool isLong;
            getPressDuration(&duration, &isLong);

            // Update statistics
            updateStats(duration, isLong);

            // Blink yellow LED: 5 blinks (short) or 10 blinks (long)
            int blinks = isLong ? 10 : 5;
            for (int i = 0; i < blinks; i++) {
                ddLedOnPin(kYellowLedPin);
                vTaskDelay(pdMS_TO_TICKS(50));
                ddLedOffPin(kYellowLedPin);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
    }
}

// ─── Task 3: Periodic Reporting (FreeRTOS) ────────────
static void taskReportRTOS(void *pvParameters) {
    (void)pvParameters;

    // Apply initial offset
    if (OFFS_REPORT > 0) {
        vTaskDelay(pdMS_TO_TICKS(OFFS_REPORT));
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        unsigned int  tp, sp, lp;
        unsigned long tsd, tld;
        getAndResetStats(&tp, &sp, &lp, &tsd, &tld);

        Serial.println(F("=== Press Statistics Report ==="));
        Serial.print(F("Total presses:          ")); Serial.println(tp);
        Serial.print(F("Short presses (<500ms): ")); Serial.println(sp);
        Serial.print(F("Long presses (>=500ms): ")); Serial.println(lp);

        if (tp > 0) {
            unsigned long avgDur = (tsd + tld) / tp;
            Serial.print(F("Average duration:       "));
            Serial.print(avgDur);
            Serial.println(F(" ms"));
        } else {
            Serial.println(F("Average duration:       N/A"));
        }

        Serial.println(F("==============================="));

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(REC_REPORT));
    }
}

// ─── App Entry Points ────────────────────────────────
void lab3_2AppSetup() {
    Serial.begin(9600);

    // Setup button (internal pull-up)
    pinMode(kButtonPin, INPUT_PULLUP);

    // Setup LEDs
    ddLedInitPin(kGreenLedPin);
    ddLedInitPin(kRedLedPin);
    ddLedInitPin(kYellowLedPin);

    // Create synchronization primitives
    xPressEventSemaphore = xSemaphoreCreateBinary();
    xStatsMutex          = xSemaphoreCreateMutex();

    Serial.println(F("Lab 3.2 - FreeRTOS Preemptive Scheduler"));
    Serial.println(F("Monitoring button presses..."));

    // Create tasks (priority: btn=2 highest, stats=1, report=1)
    xTaskCreate(taskBtnDetectRTOS, "BtnDet",  128, NULL, 2, NULL);
    xTaskCreate(taskStatsRTOS,     "Stats",   128, NULL, 1, NULL);
    xTaskCreate(taskReportRTOS,    "Report",  256, NULL, 1, NULL);

    // Start FreeRTOS scheduler — does not return
    vTaskStartScheduler();
}

void lab3_2AppLoop() {
    // FreeRTOS scheduler is running; this function should never execute.
}
