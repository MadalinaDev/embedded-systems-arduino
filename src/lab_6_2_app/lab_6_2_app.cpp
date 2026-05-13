// Lab 6.2 - PID Control (Variant A: Temperature)
// Reads temperature from a potentiometer-simulated sensor (A0).
// Controls a PWM-driven heater (pin 6, 0-255) using a discrete PID algorithm.
// Set point adjustable via serial. Kp/Ki/Kd configurable at runtime.
// Displays values on LCD and sends Serial Plotter data.
// FreeRTOS tasks: Acquisition, CommandInput, Control, Display.

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "lab_6_2_app.h"
#include "../dd_temp_sensor/dd_temp_sensor.h"
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
static const uint8_t kHeaterPin   = 6;   // PWM output → heater actuator
static const uint8_t kGreenLedPin = 9;   // green: output < 50 %
static const uint8_t kRedLedPin   = 10;  // red:   output >= 50 %

// ─── Task Periods (ms) ───────────────────────────────────────────────────────
#define REC_ACQUISITION  100   // read sensor every 100 ms
#define REC_CONTROL      100   // PID computation every 100 ms  (dt = 0.1 s)
#define REC_DISPLAY      500   // LCD + plotter every 500 ms

// ─── PID Configuration ───────────────────────────────────────────────────────
#define SP_DEFAULT   50.0f   // initial set point in °C
#define SP_STEP       1.0f   // SP+/SP- step size
#define SP_MIN        0.0f
#define SP_MAX      100.0f

// Default PID gains (tuned for temperature control simulation)
#define KP_DEFAULT    3.0f   // proportional gain
#define KI_DEFAULT    0.2f   // integral gain
#define KD_DEFAULT    0.5f   // derivative gain

// Control output clamping (PWM: 0-255)
#define OUTPUT_MIN    0.0f
#define OUTPUT_MAX  255.0f

// Anti-windup: clamp the integral accumulator (°C·s)
#define INTEGRAL_MAX  500.0f
#define INTEGRAL_MIN -500.0f

// PID sample period in seconds (matches REC_CONTROL)
#define DT  (REC_CONTROL / 1000.0f)

// ─── FreeRTOS Synchronisation ────────────────────────────────────────────────
static SemaphoreHandle_t xDataMutex;

// ─── Shared State (all protected by xDataMutex) ──────────────────────────────
static volatile float sharedTemp      = 0.0f;  // current temperature (°C)
static volatile float sharedSetPoint  = SP_DEFAULT;
static volatile float sharedKp        = KP_DEFAULT;
static volatile float sharedKi        = KI_DEFAULT;
static volatile float sharedKd        = KD_DEFAULT;
static volatile float sharedOutput    = 0.0f;  // PID output (0-255)
static volatile float sharedError     = 0.0f;  // current error (°C)

// ─── Serial Input Buffer size ───────────────────────────────────────────────
#define INPUT_BUF_SIZE 32

// ─── Task 1: Sensor Acquisition ──────────────────────────────────────────────
// Reads raw ADC, converts to °C, stores in sharedTemp every 100 ms.
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
//   KP <value>   – set proportional gain
//   KI <value>   – set integral gain
//   KD <value>   – set derivative gain
//   RESET        – reset PID integrator and derivative memory
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
                printf("[CMD] SP out of range (%.0f-%.0f)\n", SP_MIN, SP_MAX);
            }
        } else if (strncasecmp(buf, "KP ", 3) == 0) {
            float val = (float)atof(buf + 3);
            if (val >= 0.0f) {
                sharedKp = val;
                printf("[CMD] Kp -> %.3f\n", sharedKp);
            }
        } else if (strncasecmp(buf, "KI ", 3) == 0) {
            float val = (float)atof(buf + 3);
            if (val >= 0.0f) {
                sharedKi = val;
                printf("[CMD] Ki -> %.3f\n", sharedKi);
            }
        } else if (strncasecmp(buf, "KD ", 3) == 0) {
            float val = (float)atof(buf + 3);
            if (val >= 0.0f) {
                sharedKd = val;
                printf("[CMD] Kd -> %.3f\n", sharedKd);
            }
        } else if (strcasecmp(buf, "RESET") == 0) {
            sharedOutput = 0.0f;
            sharedError  = 0.0f;
            printf("[CMD] PID state reset\n");
        } else if (strcasecmp(buf, "PARAMS") == 0) {
            printf("[INFO] SP=%.1f Kp=%.3f Ki=%.3f Kd=%.3f\n",
                   sharedSetPoint, sharedKp, sharedKi, sharedKd);
        } else {
            printf("[CMD] Unknown. Use: SP+|SP-|SP <v>|"
                   "KP <v>|KI <v>|KD <v>|RESET|PARAMS\n");
        }

        xSemaphoreGive(xDataMutex);
    }
}

// ─── Task 3: PID Control ─────────────────────────────────────────────────────
// Discrete PID algorithm (position form):
//   e(t)  = SP - Temp
//   P(t)  = Kp * e(t)
//   I(t)  = integral_acc + Ki * e(t) * dt   (with anti-windup clamping)
//   D(t)  = Kd * (e(t) - e(t-1)) / dt
//   out   = clamp(P + I + D, 0, 255)
// Drives the PWM heater and indicator LEDs every 100 ms.
static void taskControl(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    float integralAcc = 0.0f;   // integral accumulator (°C·s)
    float prevError   = 0.0f;   // e(t-1) for derivative term
    bool  firstCycle  = true;   // skip D on the very first cycle

    for (;;) {
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        float temp  = sharedTemp;
        float sp    = sharedSetPoint;
        float kp    = sharedKp;
        float ki    = sharedKi;
        float kd    = sharedKd;
        // detect external reset request (output forced to 0 by RESET command)
        if (sharedOutput == 0.0f && sharedError == 0.0f && !firstCycle) {
            integralAcc = 0.0f;
            prevError   = 0.0f;
            firstCycle  = true;
        }
        xSemaphoreGive(xDataMutex);

        // PID computation
        float error = sp - temp;

        // Proportional term
        float pTerm = kp * error;

        // Integral term with anti-windup clamping
        integralAcc += ki * error * DT;
        if (integralAcc > INTEGRAL_MAX) integralAcc = INTEGRAL_MAX;
        if (integralAcc < INTEGRAL_MIN) integralAcc = INTEGRAL_MIN;
        float iTerm = integralAcc;

        // Derivative term (skip on first cycle to avoid derivative kick)
        float dTerm = 0.0f;
        if (!firstCycle) {
            dTerm = kd * (error - prevError) / DT;
        }
        firstCycle = false;
        prevError  = error;

        // PID output clamped to PWM range [0, 255]
        float output = pTerm + iTerm + dTerm;
        if (output < OUTPUT_MIN) output = OUTPUT_MIN;
        if (output > OUTPUT_MAX) output = OUTPUT_MAX;

        // Write back shared results
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        sharedOutput = output;
        sharedError  = error;
        xSemaphoreGive(xDataMutex);

        // Drive heater via PWM
        analogWrite(kHeaterPin, (uint8_t)output);

        // Status LEDs: red when heater > 50 %, green otherwise
        if (output >= (OUTPUT_MAX / 2.0f)) {
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
// LCD row 1: "OUT:XXX  E:XX.X"
// Serial Plotter: SetPoint:XX.X,Value:XX.X,Output:XXX
static void taskDisplay(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        float temp   = sharedTemp;
        float sp     = sharedSetPoint;
        float output = sharedOutput;
        float err    = sharedError;
        xSemaphoreGive(xDataMutex);

        // LCD row 0: set point and measured temperature
        char buf[17];
        ddLcdSetCursor(0, 0);
        snprintf(buf, sizeof(buf), "SP:%-5.1f T:%-5.1f", sp, temp);
        ddLcdPrint(buf);

        // LCD row 1: PID output (0-255) and current error
        ddLcdSetCursor(0, 1);
        snprintf(buf, sizeof(buf), "OUT:%-3d  E:%-5.1f", (int)output, err);
        ddLcdPrint(buf);

        // Arduino Serial Plotter (comma-separated key:value)
        printf("SetPoint:%.1f,Value:%.1f,Output:%.0f\n", sp, temp, output);

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(REC_DISPLAY));
    }
}

// ─── Public API ──────────────────────────────────────────────────────────────
void lab6_2AppSetup() {
    Serial.begin(115200);
    fdev_setup_stream(&serialOut, serialPutchar, nullptr,      _FDEV_SETUP_WRITE);
    fdev_setup_stream(&serialIn,  nullptr,      serialGetchar, _FDEV_SETUP_READ);
    stdout = &serialOut;
    stdin  = &serialIn;

    // Initialise peripherals
    ddTempSensorSetup(kSensorPin);
    pinMode(kHeaterPin, OUTPUT);
    analogWrite(kHeaterPin, 0);
    ddLedInitPin(kGreenLedPin);
    ddLedInitPin(kRedLedPin);
    ddLedOffPin(kGreenLedPin);
    ddLedOffPin(kRedLedPin);
    ddLcdSetup();
    ddLcdClear();

    // Create FreeRTOS mutex
    xDataMutex = xSemaphoreCreateMutex();

    printf("Lab 6.2 - PID Temperature Control\n");
    printf("SP=%.1f C  Kp=%.2f Ki=%.2f Kd=%.2f\n",
           SP_DEFAULT, KP_DEFAULT, KI_DEFAULT, KD_DEFAULT);
    printf("Commands: SP+|SP-|SP <v>|KP <v>|KI <v>|KD <v>|RESET|PARAMS\n");

    // Create tasks: Cmd=lowest (blocking), Acq=highest, Ctrl=3, Disp=2
    xTaskCreate(taskAcquisition, "Acq",  192, nullptr, 4, nullptr);
    xTaskCreate(taskControl,     "Ctrl", 256, nullptr, 3, nullptr);
    xTaskCreate(taskDisplay,     "Disp", 256, nullptr, 2, nullptr);
    xTaskCreate(taskCmdInput,    "Cmd",  256, nullptr, 1, nullptr);

    vTaskStartScheduler();
}

void lab6_2AppLoop() {
    // FreeRTOS scheduler runs; this body is never reached.
}
