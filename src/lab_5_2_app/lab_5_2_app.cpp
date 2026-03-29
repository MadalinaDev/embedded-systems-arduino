/*
 * Lab 5.2 - Analog + Binary Actuator Control (Variant C)
 *
 * Two actuators running in parallel under FreeRTOS:
 *   - Binary actuator : relay module on pin 7 (ON/OFF/TOGGLE)
 *   - Analog actuator : PWM LED on pin 6, 0-100% brightness (simulates motor speed)
 *
 * Commands arrive via Serial (STDIO):
 *   PWM <0-100>  or  SPEED <0-100>  or  bare number  -> set analog speed
 *   ON | OFF | T | TOGGLE                             -> control relay
 *
 * Signal conditioning pipeline (analog):
 *   1. Saturation        - clamp to 0..100
 *   2. Median filter[5]  - removes impulse/typo noise
 *   3. EMA (alpha=0.3)   - reduces fluctuations
 *   4. Ramp (5%/50ms)    - smooth start/stop, protects actuator
 *
 * Alerts:
 *   OVERLOAD  - ramped speed at 100% for > 3 s
 *   AT LIMIT  - ramped speed is exactly 0 or 100
 *   RAPID TOG - relay toggled < 1 s after previous toggle
 *
 * Tasks:
 *   taskCommandInput  50 ms  prio 3  reads serial, posts commands
 *   taskConditioning  50 ms  prio 2  pipeline + actuator output + LEDs
 *   taskDisplayReport 500ms  prio 1  serial STDIO report + LCD
 */

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "lab_5_2_app.h"
#include "../dd_relay/dd_relay.h"
#include "../dd_led/dd_led.h"
#include "../dd_lcd/dd_lcd.h"

// redirect stdout to serial (STDIO)
static FILE serialOut;

static int serialPutchar(char c, FILE *stream) {
    (void)stream;
    if (c == '\n') Serial.write('\r');
    Serial.write(c);
    return 0;
}

// â”€â”€ Pin assignments â”€â”€
static const uint8_t kPwmPin      = 6;   // PWM analog actuator (Timer4, no conflict)
static const uint8_t kRelayPin    = 7;   // relay binary actuator
static const uint8_t kGreenLedPin = 9;   // system OK indicator
static const uint8_t kRedLedPin   = 10;  // alert indicator

// â”€â”€ Task periods (ms) â”€â”€
#define REC_CMD_INPUT    50
#define REC_CONDITIONING 50
#define REC_DISPLAY      500

// â”€â”€ Conditioning parameters â”€â”€
#define SPEED_MIN          0
#define SPEED_MAX          100
#define MEDIAN_WINDOW      5
#define EMA_ALPHA          0.3f
#define RAMP_STEP          5     // max % change per conditioning cycle (5%/50ms -> 0-100 in 1s)
#define OVERLOAD_CYCLES    60    // 60 x 50ms = 3s at 100% -> overload

// â”€â”€ Relay debounce â”€â”€
#define DEBOUNCE_THRESHOLD 5

// â”€â”€ Command types â”€â”€
enum CmdType : uint8_t {
    CMD_NONE   = 0,
    CMD_SPEED  = 1,   // set analog actuator speed 0-100
    CMD_ON     = 2,   // relay ON
    CMD_OFF    = 3,   // relay OFF
    CMD_TOGGLE = 4    // relay toggle
};

// â”€â”€ Synchronization â”€â”€
static SemaphoreHandle_t xDataMutex;

// â”€â”€ Shared: last command â”€â”€
static volatile CmdType  sharedCmdType      = CMD_NONE;
static volatile int      sharedSpeedTarget  = 0;    // 0-100, raw from serial
static volatile bool     sharedRelayDesired = false;
static volatile uint32_t sharedCmdCount     = 0;

// â”€â”€ Shared: conditioned actuator state â”€â”€
static volatile int   sharedSaturated  = 0;
static volatile int   sharedMedian     = 0;
static volatile float sharedEma        = 0.0f;
static volatile int   sharedRamped     = 0;    // actual output %
static volatile bool  sharedRelayState = false;

// â”€â”€ Shared: alerts â”€â”€
static volatile bool sharedOverload    = false;
static volatile bool sharedAtLimit     = false;
static volatile bool sharedRapidToggle = false;

// â”€â”€ Serial input buffer â”€â”€
#define INPUT_BUF_SIZE 24
static char    inputBuf[INPUT_BUF_SIZE];
static uint8_t inputIdx = 0;

/*
 * parseCommand - converts a serial string to a CmdType + optional value
 *   "PWM 75"    -> CMD_SPEED, 75
 *   "SPEED 75"  -> CMD_SPEED, 75
 *   "75"        -> CMD_SPEED, 75
 *   "ON"        -> CMD_ON
 *   "OFF"       -> CMD_OFF
 *   "T"/"TOGGLE"-> CMD_TOGGLE
 */
static CmdType parseCommand(const char *str, int *outValue) {
    *outValue = 0;

    if (strncasecmp(str, "PWM ", 4) == 0) {
        *outValue = atoi(str + 4);
        return CMD_SPEED;
    }
    if (strncasecmp(str, "SPEED ", 6) == 0) {
        *outValue = atoi(str + 6);
        return CMD_SPEED;
    }
    // bare number
    bool allDigits = (str[0] != '\0');
    for (uint8_t i = 0; str[i] != '\0'; i++) {
        if (str[i] < '0' || str[i] > '9') { allDigits = false; break; }
    }
    if (allDigits) {
        *outValue = atoi(str);
        return CMD_SPEED;
    }
    if (strcasecmp(str, "ON") == 0)     return CMD_ON;
    if (strcasecmp(str, "OFF") == 0)    return CMD_OFF;
    if (strcasecmp(str, "T") == 0)      return CMD_TOGGLE;
    if (strcasecmp(str, "TOGGLE") == 0) return CMD_TOGGLE;
    return CMD_NONE;
}

/*
 * Task 1 - Command Input (50 ms period, priority 3)
 * Reads characters from serial. On newline, parses the command
 * and updates the shared desired state under mutex.
 */
static void taskCommandInput(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        while (Serial.available() > 0) {
            char c = (char)Serial.read();

            if (c == '\n' || c == '\r') {
                if (inputIdx > 0) {
                    inputBuf[inputIdx] = '\0';
                    inputIdx = 0;

                    int val = 0;
                    CmdType cmd = parseCommand(inputBuf, &val);

                    if (cmd != CMD_NONE) {
                        xSemaphoreTake(xDataMutex, portMAX_DELAY);
                        sharedCmdType = cmd;
                        sharedCmdCount++;
                        if (cmd == CMD_SPEED) {
                            sharedSpeedTarget = val;
                        } else if (cmd == CMD_ON) {
                            sharedRelayDesired = true;
                        } else if (cmd == CMD_OFF) {
                            sharedRelayDesired = false;
                        } else if (cmd == CMD_TOGGLE) {
                            sharedRelayDesired = !sharedRelayState;
                        }
                        xSemaphoreGive(xDataMutex);

                        if (cmd == CMD_SPEED) {
                            printf("[CMD] Speed target -> %d%%\n", val);
                        } else {
                            printf("[CMD] Relay -> %s\n",
                                   cmd == CMD_ON ? "ON" :
                                   cmd == CMD_OFF ? "OFF" : "TOGGLE");
                        }
                    } else {
                        printf("[CMD] Unknown. Use: PWM <0-100> | ON | OFF | T\n");
                    }
                }
            } else {
                if (inputIdx < INPUT_BUF_SIZE - 1) {
                    inputBuf[inputIdx++] = c;
                }
            }
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(REC_CMD_INPUT));
    }
}

// â”€â”€ Median filter (global state, for analog speed) â”€â”€
static int     medBuf[MEDIAN_WINDOW] = {0};
static uint8_t medIdx  = 0;
static bool    medFull = false;

static int medianFilter(int newVal) {
    medBuf[medIdx] = newVal;
    medIdx = (medIdx + 1) % MEDIAN_WINDOW;
    if (medIdx == 0) medFull = true;

    uint8_t count = medFull ? MEDIAN_WINDOW : medIdx;
    int sorted[MEDIAN_WINDOW];
    memcpy(sorted, medBuf, count * sizeof(int));

    for (uint8_t i = 1; i < count; i++) {
        int key = sorted[i];
        int8_t j = i - 1;
        while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
        sorted[j + 1] = key;
    }
    return sorted[count / 2];
}

// â”€â”€ EMA filter (global state, for analog speed) â”€â”€
static float emaVal  = 0.0f;
static bool  emaInit = false;

static float emaFilter(float newVal) {
    if (!emaInit) { emaVal = newVal; emaInit = true; }
    else          { emaVal = EMA_ALPHA * newVal + (1.0f - EMA_ALPHA) * emaVal; }
    return emaVal;
}

/*
 * Task 2 - Signal Conditioning (50 ms period, priority 2)
 *
 * Analog actuator pipeline each cycle:
 *   raw -> saturate -> median[5] -> EMA -> ramp -> analogWrite
 *
 * Binary actuator:
 *   desired state debounced over 5 stable cycles -> ddRelaySetState
 *
 * Also detects: overload, at-limit, rapid-toggle alerts.
 * Controls green/red LEDs based on alert state.
 */
static void taskConditioning(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    int      currentRamped    = 0;
    bool     currentRelay     = false;
    int      debounceCount    = 0;
    bool     pendingRelay     = false;
    uint32_t overloadCycles   = 0;
    uint32_t lastToggleMs     = 0;
    uint8_t  rapidToggleTimer = 0;  // holds alert visible for display task

    for (;;) {
        // read desired state under mutex
        int  target;
        bool desiredRelay;
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        target       = sharedSpeedTarget;
        desiredRelay = sharedRelayDesired;
        xSemaphoreGive(xDataMutex);

        // â”€â”€â”€â”€ Analog actuator conditioning â”€â”€â”€â”€

        // 1. Saturation
        int sat = target;
        if (sat < SPEED_MIN) sat = SPEED_MIN;
        if (sat > SPEED_MAX) sat = SPEED_MAX;

        // 2. Median filter
        int med = medianFilter(sat);

        // 3. EMA
        float ema        = emaFilter((float)med);
        int   emaRounded = (int)(ema + 0.5f);

        // 4. Ramp - limit rate of change for smooth motion
        int diff = emaRounded - currentRamped;
        if      (diff >  RAMP_STEP) currentRamped += RAMP_STEP;
        else if (diff < -RAMP_STEP) currentRamped -= RAMP_STEP;
        else                        currentRamped  = emaRounded;

        // apply PWM: 0-100% -> 0-255
        uint8_t pwmVal = (uint8_t)((long)currentRamped * 255L / 100L);
        analogWrite(kPwmPin, pwmVal);

        // overload: at 100% for too long
        if (currentRamped >= SPEED_MAX) {
            if (overloadCycles < (uint32_t)OVERLOAD_CYCLES) overloadCycles++;
        } else {
            overloadCycles = 0;
        }
        bool overload = (overloadCycles >= (uint32_t)OVERLOAD_CYCLES);
        bool atLimit  = (currentRamped == SPEED_MIN || currentRamped == SPEED_MAX);

        // â”€â”€â”€â”€ Binary actuator conditioning (debounce) â”€â”€â”€â”€

        if (desiredRelay == pendingRelay) {
            if (debounceCount < DEBOUNCE_THRESHOLD) debounceCount++;
        } else {
            pendingRelay  = desiredRelay;
            debounceCount = 1;
        }

        if (debounceCount >= DEBOUNCE_THRESHOLD && currentRelay != pendingRelay) {
            currentRelay = pendingRelay;
            uint32_t now = millis();
            if (lastToggleMs > 0 && (now - lastToggleMs) < 1000U) {
                rapidToggleTimer = 20;  // hold alert visible for 20 x 50ms = 1s
            }
            lastToggleMs = now;
            ddRelaySetState(kRelayPin, currentRelay);
        }

        // count down alert timer
        if (rapidToggleTimer > 0) rapidToggleTimer--;

        // â”€â”€â”€â”€ LED control â”€â”€â”€â”€
        bool anyAlert = overload || (rapidToggleTimer > 0);
        if (anyAlert) {
            ddLedOnPin(kRedLedPin);
            ddLedOffPin(kGreenLedPin);
        } else {
            ddLedOnPin(kGreenLedPin);
            ddLedOffPin(kRedLedPin);
        }

        // â”€â”€â”€â”€ Write results for display task â”€â”€â”€â”€
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        sharedSaturated   = sat;
        sharedMedian      = med;
        sharedEma         = ema;
        sharedRamped      = currentRamped;
        sharedRelayState  = currentRelay;
        sharedOverload    = overload;
        sharedAtLimit     = atLimit;
        sharedRapidToggle = (rapidToggleTimer > 0);
        xSemaphoreGive(xDataMutex);

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(REC_CONDITIONING));
    }
}

/*
 * Task 3 - Display & Reporting (500 ms period, priority 1)
 * Prints a structured STDIO report via printf (serial).
 * Also outputs a serial plotter line: sat med ramped
 * Updates LCD: line 1 = analog speed, line 2 = relay + alerts.
 */
static void taskDisplayReport(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        int   sat, med, ramped;
        float ema;
        bool  relay, overload, atLimit, rapidToggle;
        uint32_t cmdCnt;

        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        sat         = sharedSaturated;
        med         = sharedMedian;
        ema         = sharedEma;
        ramped      = sharedRamped;
        relay       = sharedRelayState;
        overload    = sharedOverload;
        atLimit     = sharedAtLimit;
        rapidToggle = sharedRapidToggle;
        cmdCnt      = sharedCmdCount;
        xSemaphoreGive(xDataMutex);

        char emaStr[10];
        dtostrf(ema, 5, 1, emaStr);

        // â”€â”€ Serial STDIO report â”€â”€
        printf("=== Actuator Report ===\n");
        printf("-- Analog (PWM pin %d) --\n", kPwmPin);
        printf("  Saturated:  %3d%%\n", sat);
        printf("  Median[%d]:  %3d%%\n", MEDIAN_WINDOW, med);
        printf("  EMA:        %s%%\n", emaStr);
        printf("  Ramped:     %3d%%  (PWM %d/255)\n",
               ramped, (int)((long)ramped * 255L / 100L));
        printf("  Alert:  %s\n",
               overload ? "!! OVERLOAD !!" :
               atLimit  ? "AT LIMIT" : "OK");
        printf("-- Binary (Relay pin %d) --\n", kRelayPin);
        printf("  State:  %s\n", relay ? "ON" : "OFF");
        printf("  Alert:  %s\n", rapidToggle ? "!! RAPID TOGGLE !!" : "OK");
        printf("Total cmds: %lu\n", (unsigned long)cmdCnt);
        printf("=======================\n");

        // serial plotter: sat med ramped (space separated for Arduino Serial Plotter)
        printf("%d %d %d\n", sat, med, ramped);

        // â”€â”€ LCD update â”€â”€
        // line 1: analog speed (ramped) + relay state
        char line1[17];
        snprintf(line1, sizeof(line1), "PWM:%3d%% R:%-3s",
                 ramped, relay ? "ON" : "OFF");

        // line 2: alert status
        char line2[17];
        if (overload) {
            snprintf(line2, sizeof(line2), "!!OVERLOAD!!    ");
        } else if (rapidToggle) {
            snprintf(line2, sizeof(line2), "!!RAPID-TOGGLE! ");
        } else if (atLimit) {
            snprintf(line2, sizeof(line2), "AT LIMIT  Med:%2d", med);
        } else {
            snprintf(line2, sizeof(line2), "OK   Med:%3d%%   ", med);
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

    // STDIO - redirect printf to serial
    fdev_setup_stream(&serialOut, serialPutchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &serialOut;

    // init analog actuator pin
    pinMode(kPwmPin, OUTPUT);
    analogWrite(kPwmPin, 0);

    // init binary actuator + status LEDs
    ddRelayInit(kRelayPin);
    ddLedInitPin(kGreenLedPin);
    ddLedInitPin(kRedLedPin);

    // init LCD
    ddLcdSetup();
    ddLcdClear();
    ddLcdSetCursor(0, 0);
    ddLcdPrint("Lab5.2 ActCtrl");
    ddLcdSetCursor(0, 1);
    ddLcdPrint("PWM 0-100|ON/T");

    // start green LED (system OK)
    ddLedOnPin(kGreenLedPin);

    // create mutex
    xDataMutex = xSemaphoreCreateMutex();

    printf("Lab 5.2 - Analog + Binary Actuator Control (Variant C)\n");
    printf("Analog: PWM <0-100>  |  SPEED <0-100>  |  bare number\n");
    printf("Binary: ON | OFF | T | TOGGLE\n");
    printf("Ramp: %d%%/%dms  Overload: >%ds at 100%%  Debounce: %d cycles\n",
           RAMP_STEP, REC_CONDITIONING,
           (OVERLOAD_CYCLES * REC_CONDITIONING) / 1000,
           DEBOUNCE_THRESHOLD);

    xTaskCreate(taskCommandInput,  "CmdIn", 256, NULL, 3, NULL);
    xTaskCreate(taskConditioning,  "Cond",  256, NULL, 2, NULL);
    xTaskCreate(taskDisplayReport, "Disp",  384, NULL, 1, NULL);

    vTaskStartScheduler();
}

void lab5_2AppLoop() {
    // FreeRTOS handles everything
}

