#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>

#include "lab_4_2_app.h"
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
static const uint8_t kSensorPin   = A0;   // potentiometer for temperature sim
static const uint8_t kGreenLedPin = 9;    // activity indicator (toggles on each sample)

// how often each task runs (in ms)
#define REC_ACQUISITION    50    // read sensor every 50ms
#define REC_DISPLAY        500   // print report every 500ms

// signal conditioning parameters
#define ADC_MIN        0
#define ADC_MAX        1023
#define MEDIAN_WINDOW  5       // median filter uses last 5 samples
#define EMA_ALPHA      0.3f   // weight for new sample in exponential moving avg

// mutex and semaphore for task synchronization
static SemaphoreHandle_t xDataMutex;
static SemaphoreHandle_t xNewSampleSem;   // acquisition signals conditioning

// shared data between tasks (protected by mutex)
static volatile int   sharedRawAdc       = 0;
static volatile int   sharedSaturated    = 0;
static volatile int   sharedMedian       = 0;
static volatile float sharedAverage      = 0.0f;
static volatile float sharedTempRaw      = 0.0f;
static volatile float sharedTempFiltered = 0.0f;

/*
 * Task 1 - Sensor Acquisition
 * Reads raw ADC from the potentiometer every 50ms.
 * Then signals the conditioning task that new data is ready.
 */
static void taskAcquisition(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        int raw = ddTempSensorReadRaw(kSensorPin);

        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        sharedRawAdc = raw;
        xSemaphoreGive(xDataMutex);

        // tell the conditioning task there's a new sample
        xSemaphoreGive(xNewSampleSem);

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(REC_ACQUISITION));
    }
}

/*
 * Saturation - clamp ADC value to 0..1023
 * Protects against any out of range readings
 */
static int saturate(int value) {
    if (value < ADC_MIN) return ADC_MIN;
    if (value > ADC_MAX) return ADC_MAX;
    return value;
}

/*
 * Median filter - removes "salt and pepper" noise
 * Keeps a buffer of the last 5 readings, sorts them,
 * and returns the middle value. This way random spikes
 * get thrown out.
 */
static int   medianBuf[MEDIAN_WINDOW];
static uint8_t medianIdx     = 0;
static bool    medianBufFull = false;

static int medianFilter(int newVal) {
    medianBuf[medianIdx] = newVal;
    medianIdx = (medianIdx + 1) % MEDIAN_WINDOW;
    if (medianIdx == 0) medianBufFull = true;

    uint8_t count = medianBufFull ? MEDIAN_WINDOW : medianIdx;

    // copy and sort with insertion sort (good enough for 5 elements)
    int sorted[MEDIAN_WINDOW];
    memcpy(sorted, medianBuf, count * sizeof(int));

    for (uint8_t i = 1; i < count; i++) {
        int key = sorted[i];
        int8_t j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    return sorted[count / 2];   // middle value
}

/*
 * Exponential Moving Average (weighted average)
 * new_avg = alpha * new_sample + (1-alpha) * old_avg
 * With alpha=0.3, it smooths the signal and reacts
 * gradually to changes instead of jumping instantly.
 */
static float emaValue       = 0.0f;
static bool  emaInitialized = false;

static float weightedAvg(float newVal) {
    if (!emaInitialized) {
        emaValue       = newVal;
        emaInitialized = true;
    } else {
        emaValue = EMA_ALPHA * newVal + (1.0f - EMA_ALPHA) * emaValue;
    }
    return emaValue;
}

/*
 * Task 2 - Signal Conditioning
 * Waits for a new sample from acquisition, then runs it
 * through the pipeline: saturation -> median -> EMA.
 * Toggles the green LED on each processed sample as
 * an activity indicator.
 */
static bool ledState = false;

static void taskConditioning(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        // wait until acquisition sends a new sample
        if (xSemaphoreTake(xNewSampleSem, portMAX_DELAY) == pdTRUE) {
            int raw;
            xSemaphoreTake(xDataMutex, portMAX_DELAY);
            raw = sharedRawAdc;
            xSemaphoreGive(xDataMutex);

            // run the conditioning pipeline
            int   sat = saturate(raw);
            int   med = medianFilter(sat);
            float avg = weightedAvg((float)med);

            // convert both raw and filtered to temperature
            float tempRaw      = ddTempSensorRawToTempC(raw);
            float tempFiltered = ddTempSensorRawToTempC((int)(avg + 0.5f));

            // save processed values
            xSemaphoreTake(xDataMutex, portMAX_DELAY);
            sharedSaturated    = sat;
            sharedMedian       = med;
            sharedAverage      = avg;
            sharedTempRaw      = tempRaw;
            sharedTempFiltered = tempFiltered;
            xSemaphoreGive(xDataMutex);

            // toggle green LED to show the task is running
            ledState = !ledState;
            if (ledState) {
                ddLedOnPin(kGreenLedPin);
            } else {
                ddLedOffPin(kGreenLedPin);
            }
        }
    }
}

/*
 * Task 3 - Display & Reporting via STDIO (printf)
 * Every 500ms prints a structured report showing all
 * intermediate values from the conditioning pipeline.
 * Also outputs a serial plotter compatible line at the end.
 */
static void taskDisplayReport(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        int   raw, sat, med;
        float avg, tRaw, tFilt;

        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        raw   = sharedRawAdc;
        sat   = sharedSaturated;
        med   = sharedMedian;
        avg   = sharedAverage;
        tRaw  = sharedTempRaw;
        tFilt = sharedTempFiltered;
        xSemaphoreGive(xDataMutex);

        // format floats manually (AVR doesn't support %f in printf)
        char avgStr[10], tRawStr[10], tFiltStr[10];
        dtostrf(avg, 7, 2, avgStr);
        dtostrf(tRaw, 6, 2, tRawStr);
        dtostrf(tFilt, 6, 2, tFiltStr);

        // print report using STDIO printf
        printf("=== Analog Conditioning Report ===\n");
        printf("Raw ADC:       %d\n", raw);
        printf("Saturated:     %d\n", sat);
        printf("Median[%d]:     %d\n", MEDIAN_WINDOW, med);
        printf("Weighted Avg:  %s\n", avgStr);
        printf("Temp Raw:      %s C\n", tRawStr);
        printf("Temp Filtered: %s C\n", tFiltStr);
        printf("==================================\n");

        // serial plotter line (space separated values)
        printf("%d %d %d %s\n", raw, sat, med, avgStr);

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(REC_DISPLAY));
    }
}

// setup and start everything
void lab4_2AppSetup() {
    Serial.begin(9600);

    // setup STDIO - redirect stdout so printf goes to serial
    fdev_setup_stream(&serialOut, serialPutchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &serialOut;

    // init hardware
    ddTempSensorSetup(kSensorPin);
    ddLedInitPin(kGreenLedPin);

    // create synchronization primitives
    xDataMutex    = xSemaphoreCreateMutex();
    xNewSampleSem = xSemaphoreCreateBinary();

    printf("Lab 4.2 - Analog Signal Conditioning (FreeRTOS)\n");
    printf("Pipeline: Saturation -> Median[%d] -> EMA[alpha=0.3]\n", MEDIAN_WINDOW);

    // create tasks - higher priority number = runs first
    xTaskCreate(taskAcquisition,   "Acq",   128, NULL, 3, NULL);
    xTaskCreate(taskConditioning,  "Cond",  192, NULL, 2, NULL);
    xTaskCreate(taskDisplayReport, "Disp",  256, NULL, 1, NULL);

    // start FreeRTOS scheduler - doesn't return
    vTaskStartScheduler();
}

void lab4_2AppLoop() {
    // empty - FreeRTOS scheduler handles everything
}
