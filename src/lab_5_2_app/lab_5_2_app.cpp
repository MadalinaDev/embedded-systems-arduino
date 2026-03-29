#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>

#include "lab_5_2_app.h"
#include "../dd_temp_sensor/dd_temp_sensor.h"
#include "../dd_led/dd_led.h"
#include "../dd_lcd/dd_lcd.h"
#include "../dd_relay/dd_relay.h"

// redirect stdout to serial (STDIO)
static FILE serialOut;

static int serialPutchar(char c, FILE *stream) {
    (void)stream;
    if (c == '\n') Serial.write('\r');
    Serial.write(c);
    return 0;
}

// ── Pin assignments ──
static const uint8_t kAnalogSensorPin  = A0;  // potentiometer 1 (analog temp sensor)
static const uint8_t kDigitalSensorPin = A1;  // potentiometer 2 (digital temp sensor sim)
static const uint8_t kGreenLedPin      = 9;   // normal state indicator
static const uint8_t kRedLedPin        = 10;  // alert indicator
static const uint8_t kRelayPin         = 7;   // relay actuator (cooling/heating)

// ── Task periods (ms) ──
#define REC_ACQUISITION    50     // sensor read every 50ms
#define REC_CONDITIONING   50     // conditioning at same rate
#define REC_DISPLAY        500    // report every 500ms

// ── Signal conditioning parameters ──
#define ADC_MIN            0
#define ADC_MAX            1023
#define MEDIAN_WINDOW      5      // median filter window size
#define EMA_ALPHA          0.3f   // exponential moving average weight

// ── Alert thresholds (in degrees C) ──
#define ALERT_TEMP_HIGH    60.0f  // above this = high temp alert
#define ALERT_TEMP_LOW     0.0f   // below this = low temp alert

// ── Synchronization ──
static SemaphoreHandle_t xDataMutex;
static SemaphoreHandle_t xNewSampleSem;

// ── Shared data for Sensor 1 (Analog) ──
static volatile int   s1RawAdc       = 0;
static volatile int   s1Saturated    = 0;
static volatile int   s1Median       = 0;
static volatile float s1Average      = 0.0f;
static volatile float s1TempRaw      = 0.0f;
static volatile float s1TempFiltered = 0.0f;

// ── Shared data for Sensor 2 (Digital sim) ──
static volatile int   s2RawAdc       = 0;
static volatile int   s2Saturated    = 0;
static volatile int   s2Median       = 0;
static volatile float s2Average      = 0.0f;
static volatile float s2TempRaw      = 0.0f;
static volatile float s2TempFiltered = 0.0f;

// ── Alert state ──
static volatile bool  alertHigh1     = false;
static volatile bool  alertLow1      = false;
static volatile bool  alertHigh2     = false;
static volatile bool  alertLow2      = false;
static volatile bool  relayActive    = false;

// ── Median filter state per sensor ──
typedef struct {
    int      buf[MEDIAN_WINDOW];
    uint8_t  idx;
    bool     full;
} MedianState;

static MedianState medS1 = { {0}, 0, false };
static MedianState medS2 = { {0}, 0, false };

// ── EMA state per sensor ──
typedef struct {
    float value;
    bool  initialized;
} EmaState;

static EmaState emaS1 = { 0.0f, false };
static EmaState emaS2 = { 0.0f, false };

// Saturation: clamp ADC to 0..1023
static int saturate(int value) {
    if (value < ADC_MIN) return ADC_MIN;
    if (value > ADC_MAX) return ADC_MAX;
    return value;
}

// Median filter: removes salt-and-pepper noise
static int medianFilter(MedianState *ms, int newVal) {
    ms->buf[ms->idx] = newVal;
    ms->idx = (ms->idx + 1) % MEDIAN_WINDOW;
    if (ms->idx == 0) ms->full = true;

    uint8_t count = ms->full ? MEDIAN_WINDOW : ms->idx;

    // insertion sort on small array
    int sorted[MEDIAN_WINDOW];
    memcpy(sorted, ms->buf, count * sizeof(int));

    for (uint8_t i = 1; i < count; i++) {
        int key = sorted[i];
        int8_t j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    return sorted[count / 2];
}

// Exponential Moving Average (weighted average)
static float weightedAvg(EmaState *es, float newVal) {
    if (!es->initialized) {
        es->value = newVal;
        es->initialized = true;
    } else {
        es->value = EMA_ALPHA * newVal + (1.0f - EMA_ALPHA) * es->value;
    }
    return es->value;
}

/*
 * Task 1 - Sensor Acquisition
 * Reads raw ADC from both sensors every 50ms.
 * Signals conditioning task that new data is ready.
 */
static void taskAcquisition(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        int raw1 = ddTempSensorReadRaw(kAnalogSensorPin);
        int raw2 = ddTempSensorReadRaw(kDigitalSensorPin);

        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        s1RawAdc = raw1;
        s2RawAdc = raw2;
        xSemaphoreGive(xDataMutex);

        xSemaphoreGive(xNewSampleSem);

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(REC_ACQUISITION));
    }
}

/*
 * Task 2 - Signal Conditioning
 * Pipeline: saturation -> median filter -> EMA for each sensor.
 * Also evaluates alert thresholds and controls relay + LEDs.
 */
static void taskConditioning(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        if (xSemaphoreTake(xNewSampleSem, portMAX_DELAY) == pdTRUE) {
            int raw1, raw2;

            xSemaphoreTake(xDataMutex, portMAX_DELAY);
            raw1 = s1RawAdc;
            raw2 = s2RawAdc;
            xSemaphoreGive(xDataMutex);

            // ── Sensor 1 conditioning pipeline ──
            int   sat1 = saturate(raw1);
            int   med1 = medianFilter(&medS1, sat1);
            float avg1 = weightedAvg(&emaS1, (float)med1);
            float tempRaw1  = ddTempSensorRawToTempC(raw1);
            float tempFilt1 = ddTempSensorRawToTempC((int)(avg1 + 0.5f));

            // ── Sensor 2 conditioning pipeline ──
            int   sat2 = saturate(raw2);
            int   med2 = medianFilter(&medS2, sat2);
            float avg2 = weightedAvg(&emaS2, (float)med2);
            float tempRaw2  = ddTempSensorRawToTempC(raw2);
            float tempFilt2 = ddTempSensorRawToTempC((int)(avg2 + 0.5f));

            // ── Alert evaluation ──
            bool aH1 = (tempFilt1 > ALERT_TEMP_HIGH);
            bool aL1 = (tempFilt1 < ALERT_TEMP_LOW);
            bool aH2 = (tempFilt2 > ALERT_TEMP_HIGH);
            bool aL2 = (tempFilt2 < ALERT_TEMP_LOW);

            bool anyAlert = (aH1 || aL1 || aH2 || aL2);

            // control relay: activate on any high-temp alert (cooling)
            ddRelaySetState(kRelayPin, (aH1 || aH2));

            // LEDs: green=normal, red=alert
            if (anyAlert) {
                ddLedOnPin(kRedLedPin);
                ddLedOffPin(kGreenLedPin);
            } else {
                ddLedOnPin(kGreenLedPin);
                ddLedOffPin(kRedLedPin);
            }

            // ── Store results ──
            xSemaphoreTake(xDataMutex, portMAX_DELAY);
            s1Saturated    = sat1;
            s1Median       = med1;
            s1Average      = avg1;
            s1TempRaw      = tempRaw1;
            s1TempFiltered = tempFilt1;

            s2Saturated    = sat2;
            s2Median       = med2;
            s2Average      = avg2;
            s2TempRaw      = tempRaw2;
            s2TempFiltered = tempFilt2;

            alertHigh1  = aH1;
            alertLow1   = aL1;
            alertHigh2  = aH2;
            alertLow2   = aL2;
            relayActive = (aH1 || aH2);
            xSemaphoreGive(xDataMutex);
        }
    }
}

/*
 * Task 3 - Display & Reporting
 * Every 500ms prints structured report to serial (STDIO)
 * and updates the LCD with current values and alerts.
 */
static void taskDisplayReport(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        int   r1, sa1, md1, r2, sa2, md2;
        float av1, tR1, tF1, av2, tR2, tF2;
        bool  aH1, aL1, aH2, aL2, relay;

        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        r1  = s1RawAdc;    sa1 = s1Saturated;  md1 = s1Median;
        av1 = s1Average;   tR1 = s1TempRaw;    tF1 = s1TempFiltered;
        r2  = s2RawAdc;    sa2 = s2Saturated;  md2 = s2Median;
        av2 = s2Average;   tR2 = s2TempRaw;    tF2 = s2TempFiltered;
        aH1 = alertHigh1;  aL1 = alertLow1;
        aH2 = alertHigh2;  aL2 = alertLow2;
        relay = relayActive;
        xSemaphoreGive(xDataMutex);

        // format floats (AVR doesn't support %f in printf)
        char tF1s[10], tF2s[10], tR1s[10], tR2s[10];
        char av1s[10], av2s[10];
        dtostrf(tF1, 6, 1, tF1s);
        dtostrf(tF2, 6, 1, tF2s);
        dtostrf(tR1, 6, 1, tR1s);
        dtostrf(tR2, 6, 1, tR2s);
        dtostrf(av1, 7, 2, av1s);
        dtostrf(av2, 7, 2, av2s);

        // ── Serial report (STDIO) ──
        printf("=== Dual Sensor Report ===\n");
        printf("-- Sensor 1 (Analog) --\n");
        printf("  Raw: %d  Sat: %d  Med: %d  Avg: %s\n", r1, sa1, md1, av1s);
        printf("  Temp Raw: %s C  Filtered: %s C\n", tR1s, tF1s);
        printf("  Alert: %s\n", aH1 ? "HIGH TEMP" : (aL1 ? "LOW TEMP" : "OK"));
        printf("-- Sensor 2 (Digital) --\n");
        printf("  Raw: %d  Sat: %d  Med: %d  Avg: %s\n", r2, sa2, md2, av2s);
        printf("  Temp Raw: %s C  Filtered: %s C\n", tR2s, tF2s);
        printf("  Alert: %s\n", aH2 ? "HIGH TEMP" : (aL2 ? "LOW TEMP" : "OK"));
        printf("Relay: %s\n", relay ? "ON (Cooling)" : "OFF");
        printf("==========================\n");

        // serial plotter line
        printf("%d %d %d %d\n", r1, md1, r2, md2);

        // ── LCD update ──
        // line 1: both filtered temps
        char line1[17];
        char t1[6], t2[6];
        dtostrf(tF1, 4, 0, t1);
        dtostrf(tF2, 4, 0, t2);
        snprintf(line1, sizeof(line1), "A:%sC D:%sC", t1, t2);

        // line 2: alert status and relay
        char line2[17];
        bool anyAlert = (aH1 || aL1 || aH2 || aL2);
        if (anyAlert) {
            snprintf(line2, sizeof(line2), "ALERT! Rly:%-3s", relay ? "ON" : "OFF");
        } else {
            snprintf(line2, sizeof(line2), "OK     Rly:%-3s", relay ? "ON" : "OFF");
        }

        ddLcdClear();
        ddLcdSetCursor(0, 0);
        ddLcdPrint(line1);
        ddLcdSetCursor(0, 1);
        ddLcdPrint(line2);

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(REC_DISPLAY));
    }
}

void lab5_2AppSetup() {
    Serial.begin(9600);

    // STDIO redirect
    fdev_setup_stream(&serialOut, serialPutchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &serialOut;

    // init hardware
    ddTempSensorSetup(kAnalogSensorPin);
    ddTempSensorSetup(kDigitalSensorPin);
    ddLedInitPin(kGreenLedPin);
    ddLedInitPin(kRedLedPin);
    ddRelayInit(kRelayPin);

    ddLcdSetup();
    ddLcdClear();
    ddLcdSetCursor(0, 0);
    ddLcdPrint("Lab5.2 DualSens");
    ddLcdSetCursor(0, 1);
    ddLcdPrint("Starting...");

    // start with green LED on (normal state)
    ddLedOnPin(kGreenLedPin);

    // create sync primitives
    xDataMutex    = xSemaphoreCreateMutex();
    xNewSampleSem = xSemaphoreCreateBinary();

    printf("Lab 5.2 - Dual Sensor Monitoring (Variant C)\n");
    printf("Sensor 1: Analog (A0)  Sensor 2: Digital sim (A1)\n");
    printf("Pipeline: Saturation -> Median[%d] -> EMA[a=%.1f]\n",
           MEDIAN_WINDOW, (double)EMA_ALPHA);
    printf("Thresholds: High>%.0fC  Low<%.0fC\n",
           (double)ALERT_TEMP_HIGH, (double)ALERT_TEMP_LOW);

    // create tasks: acq > cond > display (priority order)
    xTaskCreate(taskAcquisition,   "Acq",   192, NULL, 3, NULL);
    xTaskCreate(taskConditioning,  "Cond",  256, NULL, 2, NULL);
    xTaskCreate(taskDisplayReport, "Disp",  384, NULL, 1, NULL);

    vTaskStartScheduler();
}

void lab5_2AppLoop() {
    // FreeRTOS handles everything
}
