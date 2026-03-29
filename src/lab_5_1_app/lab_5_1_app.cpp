#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>

#include "lab_5_1_app.h"
#include "../dd_relay/dd_relay.h"
#include "../dd_led/dd_led.h"
#include "../dd_lcd/dd_lcd.h"

// printf redirect so we can use STDIO over serial
static FILE serialOut;

static int serialPutchar(char c, FILE *stream) {
    (void)stream;
    if (c == '\n') Serial.write('\r');
    Serial.write(c);
    return 0;
}

// pins
static const uint8_t kRelayPin    = 7;   // relay output
static const uint8_t kGreenLedPin = 9;   // green = actuator off
static const uint8_t kRedLedPin   = 10;  // red = actuator on

// task periods in ms
#define REC_CMD_INPUT       50
#define REC_CONDITIONING    50
#define REC_DISPLAY         500

// need 5 same readings in a row before we accept a state change
#define DEBOUNCE_THRESHOLD  5

// possible commands from the user
enum Command : uint8_t {
    CMD_NONE   = 0,
    CMD_ON     = 1,
    CMD_OFF    = 2,
    CMD_TOGGLE = 3
};

// mutex and semaphore for inter-task communication
static SemaphoreHandle_t xDataMutex;
static SemaphoreHandle_t xNewCommandSem;

// shared variables (all protected by xDataMutex)
static volatile Command  sharedRawCommand       = CMD_NONE;
static volatile bool     sharedDesiredState      = false;
static volatile bool     sharedConditionedState  = false;
static volatile bool     sharedActuatorState     = false;
static volatile int      sharedDebounceCount     = 0;
static volatile uint32_t sharedCmdCount          = 0;
static volatile uint32_t sharedToggleCount       = 0;
static volatile bool     sharedAlertActive       = false;
static volatile uint32_t sharedLastToggleMs      = 0;

// buffer for reading serial input
#define INPUT_BUF_SIZE 16
static char inputBuf[INPUT_BUF_SIZE];
static uint8_t inputIdx = 0;

// turns a string like "ON", "1", "OFF", "0", "T" into the right command
static Command parseCommand(const char *str) {
    if (str[0] == '1' && str[1] == '\0')                           return CMD_ON;
    if (str[0] == '0' && str[1] == '\0')                           return CMD_OFF;
    if ((str[0] == 'T' || str[0] == 't') && str[1] == '\0')       return CMD_TOGGLE;
    if (strcasecmp(str, "ON") == 0)                                return CMD_ON;
    if (strcasecmp(str, "OFF") == 0)                               return CMD_OFF;
    if (strcasecmp(str, "TOGGLE") == 0)                            return CMD_TOGGLE;
    return CMD_NONE;
}

// Task 1 - reads commands from serial, runs every 50ms
static void taskCommandInput(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        while (Serial.available() > 0) {
            char c = (char)Serial.read();

            if (c == '\n' || c == '\r') {
                if (inputIdx > 0) {
                    inputBuf[inputIdx] = '\0';
                    Command cmd = parseCommand(inputBuf);
                    inputIdx = 0;

                    if (cmd != CMD_NONE) {
                        xSemaphoreTake(xDataMutex, portMAX_DELAY);

                        sharedRawCommand = cmd;
                        sharedCmdCount++;

                        // figure out what state user wants
                        if (cmd == CMD_ON) {
                            sharedDesiredState = true;
                        } else if (cmd == CMD_OFF) {
                            sharedDesiredState = false;
                        } else if (cmd == CMD_TOGGLE) {
                            sharedDesiredState = !sharedConditionedState;
                        }

                        xSemaphoreGive(xDataMutex);
                        xSemaphoreGive(xNewCommandSem);

                        printf("[CMD] Received: %s -> %s\n",
                               inputBuf,
                               (cmd == CMD_ON) ? "ON" :
                               (cmd == CMD_OFF) ? "OFF" : "TOGGLE");
                    } else {
                        printf("[CMD] Unknown command: %s\n", inputBuf);
                        printf("[CMD] Use: ON/1, OFF/0, T/TOGGLE\n");
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

// Task 2 - signal conditioning: debouncing + saturation + alert detection
// runs every 50ms, only changes actuator after 5 stable reads
static void taskConditioning(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    bool currentConditioned = false;
    bool pendingState       = false;
    int  debounceCounter    = 0;

    for (;;) {
        bool desired;
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        desired = sharedDesiredState;
        xSemaphoreGive(xDataMutex);

        // saturation - make sure its a valid bool
        bool saturated = desired ? true : false;

        // debounce logic - same idea as button debounce but for commands
        if (saturated == pendingState) {
            if (debounceCounter < DEBOUNCE_THRESHOLD) {
                debounceCounter++;
            }
        } else {
            pendingState    = saturated;
            debounceCounter = 1;
        }

        // only apply the change after enough stable readings
        if (debounceCounter >= DEBOUNCE_THRESHOLD && currentConditioned != pendingState) {
            currentConditioned = pendingState;

            // check if toggling too fast (less than 1s between changes)
            uint32_t now = millis();
            bool rapidToggle = false;

            xSemaphoreTake(xDataMutex, portMAX_DELAY);
            if (sharedLastToggleMs > 0 && (now - sharedLastToggleMs) < 1000) {
                rapidToggle = true;
            }
            sharedLastToggleMs = now;
            sharedToggleCount++;
            sharedAlertActive = rapidToggle;
            xSemaphoreGive(xDataMutex);

            // set the relay
            ddRelaySetState(kRelayPin, currentConditioned);

            // update LEDs to show whats happening
            if (currentConditioned) {
                ddLedOnPin(kRedLedPin);
                ddLedOffPin(kGreenLedPin);
            } else {
                ddLedOnPin(kGreenLedPin);
                ddLedOffPin(kRedLedPin);
            }
        }

        // write back the conditioned state
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        sharedConditionedState = currentConditioned;
        sharedActuatorState    = currentConditioned;
        sharedDebounceCount    = debounceCounter;
        xSemaphoreGive(xDataMutex);

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(REC_CONDITIONING));
    }
}

// Task 3 - prints report to serial and updates LCD, every 500ms
static void taskDisplayReport(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        bool     actState;
        bool     alertActive;
        uint32_t cmdCnt;
        uint32_t toggleCnt;
        int      dbCnt;
        Command  lastCmd;

        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        actState    = sharedActuatorState;
        alertActive = sharedAlertActive;
        cmdCnt      = sharedCmdCount;
        toggleCnt   = sharedToggleCount;
        dbCnt       = sharedDebounceCount;
        lastCmd     = sharedRawCommand;
        xSemaphoreGive(xDataMutex);

        // serial report
        printf("=== Binary Actuator Report ===\n");
        printf("Actuator State: %s\n", actState ? "ON" : "OFF");
        printf("Last Command:   %s\n",
               (lastCmd == CMD_ON) ? "ON" :
               (lastCmd == CMD_OFF) ? "OFF" :
               (lastCmd == CMD_TOGGLE) ? "TOGGLE" : "NONE");
        printf("Commands Recv:  %lu\n", (unsigned long)cmdCnt);
        printf("State Changes:  %lu\n", (unsigned long)toggleCnt);
        printf("Debounce:       %d/%d\n", dbCnt, DEBOUNCE_THRESHOLD);
        printf("Alert:          %s\n", alertActive ? "!! RAPID TOGGLE !!" : "OK");
        printf("==============================\n");

        // lcd update
        char line1[17];
        if (alertActive) {
            snprintf(line1, sizeof(line1), "Act:%-3s  !ALERT!", actState ? "ON" : "OFF");
        } else {
            snprintf(line1, sizeof(line1), "Act:%-3s  State:OK", actState ? "ON" : "OFF");
        }

        char line2[17];
        snprintf(line2, sizeof(line2), "Cmd:%-4lu Chg:%-3lu",
                 (unsigned long)cmdCnt, (unsigned long)toggleCnt);

        ddLcdClear();
        ddLcdSetCursor(0, 0);
        ddLcdPrint(line1);
        ddLcdSetCursor(0, 1);
        ddLcdPrint(line2);

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(REC_DISPLAY));
    }
}

void lab5_1AppSetup() {
    Serial.begin(9600);

    // setup printf over serial
    fdev_setup_stream(&serialOut, serialPutchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &serialOut;

    // init hardware
    ddRelayInit(kRelayPin);
    ddLedInitPin(kGreenLedPin);
    ddLedInitPin(kRedLedPin);

    ddLcdSetup();
    ddLcdClear();
    ddLcdSetCursor(0, 0);
    ddLcdPrint("Lab 5.1 BinAct");
    ddLcdSetCursor(0, 1);
    ddLcdPrint("Ready: ON/OFF/T");

    // actuator starts OFF
    ddLedOnPin(kGreenLedPin);

    // create mutex and semaphore
    xDataMutex     = xSemaphoreCreateMutex();
    xNewCommandSem = xSemaphoreCreateBinary();

    printf("Lab 5.1 - Binary Actuator Control (FreeRTOS)\n");
    printf("Commands: ON/1, OFF/0, T/TOGGLE (via Serial)\n");
    printf("Debounce: %d cycles @ %d ms = %d ms\n",
           DEBOUNCE_THRESHOLD, REC_CONDITIONING,
           DEBOUNCE_THRESHOLD * REC_CONDITIONING);

    // task1=cmd input (highest prio), task2=conditioning, task3=display (lowest)
    xTaskCreate(taskCommandInput,   "CmdIn",  256, NULL, 3, NULL);
    xTaskCreate(taskConditioning,   "Cond",   192, NULL, 2, NULL);
    xTaskCreate(taskDisplayReport,  "Disp",   256, NULL, 1, NULL);

    vTaskStartScheduler();
}

void lab5_1AppLoop() {
    // freertos handles everything
}
