#include <Arduino.h>
#include "lab_3_1_app.h"
#include "../dd_led/dd_led.h"
#include "../dd_button/dd_button.h"

// ─── Pin Definitions ─────────────────────────────────
static const uint8_t kButtonPin    = 6;
static const uint8_t kYellowLedPin = 8;
static const uint8_t kGreenLedPin  = 9;
static const uint8_t kRedLedPin    = 10;

// ─── Sequential Scheduler ────────────────────────────

typedef struct {
    void (*task_func)(void);   // pointer to task function
    int rec;                   // recurrence in ms (ticks)
    int offset;                // initial offset in ms (ticks)
    int rec_cnt;               // recurrence counter
} Task_t;

enum {
    TASK_BTN_ID = 0,
    TASK_STATS_ID,
    TASK_REPORT_ID,
    MAX_OF_TASKS
};

#define REC_BTN     20      // Button scan every 20ms
#define REC_STATS   50      // Stats/blink every 50ms
#define REC_REPORT  10000   // Report every 10s

#define OFFS_BTN    0       // Button starts immediately
#define OFFS_STATS  10      // Stats offset 10ms
#define OFFS_REPORT 20      // Report offset 20ms

// Task prototypes
static void taskBtnDetect(void);
static void taskStats(void);
static void taskReport(void);

static Task_t tasks[MAX_OF_TASKS] = {
    { taskBtnDetect, REC_BTN,    OFFS_BTN,    0 },
    { taskStats,     REC_STATS,  OFFS_STATS,  0 },
    { taskReport,    REC_REPORT, OFFS_REPORT,  0 }
};

// ─── Shared Data (Provider/Consumer) ─────────────────
// Set by Task 1, consumed by Task 2
static bool          sharedNewEvent      = false;
static unsigned long sharedPressDuration = 0;
static bool          sharedIsLongPress   = false;

// Setter (Task 1 = provider)
static void setNewPressEvent(unsigned long duration, bool isLong) {
    sharedPressDuration = duration;
    sharedIsLongPress   = isLong;
    sharedNewEvent      = true;
}

// Getter (Task 2 = consumer)
static bool getNewPressEvent(unsigned long *duration, bool *isLong) {
    if (!sharedNewEvent) return false;
    *duration     = sharedPressDuration;
    *isLong       = sharedIsLongPress;
    sharedNewEvent = false;
    return true;
}

// Managed by Task 2, read/reset by Task 3
static unsigned int  statsTotalPresses     = 0;
static unsigned int  statsShortPresses     = 0;
static unsigned int  statsLongPresses      = 0;
static unsigned long statsTotalShortDur    = 0;
static unsigned long statsTotalLongDur     = 0;

static void updateStats(unsigned long duration, bool isLong) {
    statsTotalPresses++;
    if (isLong) {
        statsLongPresses++;
        statsTotalLongDur += duration;
    } else {
        statsShortPresses++;
        statsTotalShortDur += duration;
    }
}

static void getAndResetStats(unsigned int *tp, unsigned int *sp, unsigned int *lp,
                             unsigned long *tsd, unsigned long *tld) {
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
}

// ─── Task 1: Button Detection & Duration Measurement ─
static bool          btnPrevState      = false;
static unsigned long btnPressStart     = 0;
static uint8_t       btnDebounceCount  = 0;
static int           btnStableState    = 0;

static void taskBtnDetect(void) {
    // Debounced read via dd_button driver (5 consecutive same-value reads)
    int debounced = ddButtonReadDebouncedPin(kButtonPin, &btnDebounceCount, &btnStableState);
    bool currentState = (debounced == 1);

    if (currentState && !btnPrevState) {
        // Button just pressed — start timing
        btnPressStart = millis();
        ddLedOffPin(kGreenLedPin);
        ddLedOffPin(kRedLedPin);
    } else if (!currentState && btnPrevState) {
        // Button just released — measure duration
        unsigned long duration = millis() - btnPressStart;
        bool isLong = (duration >= 500);

        if (isLong) {
            ddLedOnPin(kRedLedPin);
            ddLedOffPin(kGreenLedPin);
        } else {
            ddLedOnPin(kGreenLedPin);
            ddLedOffPin(kRedLedPin);
        }

        // Provide event to Task 2
        setNewPressEvent(duration, isLong);
    }

    btnPrevState = currentState;
}

// ─── Task 2: Statistics & Yellow LED Blink ────────────
static int blinkRemaining = 0;

static void taskStats(void) {
    // Consume press event from Task 1
    unsigned long duration;
    bool isLong;
    if (getNewPressEvent(&duration, &isLong)) {
        updateStats(duration, isLong);
        blinkRemaining = isLong ? 20 : 10; // 10 blinks=20 toggles, 5 blinks=10 toggles
    }

    // State machine for yellow LED blink
    if (blinkRemaining > 0) {
        if (digitalRead(kYellowLedPin) == HIGH) {
            ddLedOffPin(kYellowLedPin);
        } else {
            ddLedOnPin(kYellowLedPin);
        }
        blinkRemaining--;
    } else {
        ddLedOffPin(kYellowLedPin);
    }
}

// ─── Task 3: Periodic Reporting via STDIO ─────────────
static void taskReport(void) {
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
}

// ─── Scheduler Functions ─────────────────────────────
static void os_seq_scheduler_setup(void) {
    for (int i = 0; i < MAX_OF_TASKS; i++) {
        tasks[i].rec_cnt = tasks[i].offset;
    }
}

static void os_seq_scheduler_loop(void) {
    for (int i = 0; i < MAX_OF_TASKS; i++) {
        if (--tasks[i].rec_cnt <= 0) {
            tasks[i].rec_cnt = tasks[i].rec;
            tasks[i].task_func();
        }
    }
}

// ─── App Entry Points ────────────────────────────────
void lab3_1AppSetup() {
    Serial.begin(9600);

    // Setup button via dd_button driver (internal pull-up)
    ddButtonInitPin(kButtonPin);

    // Setup LEDs
    ddLedInitPin(kGreenLedPin);
    ddLedInitPin(kRedLedPin);
    ddLedInitPin(kYellowLedPin);

    // Init scheduler offsets
    os_seq_scheduler_setup();

    Serial.println(F("Lab 3.1 - Bare-metal Sequential Scheduler"));
    Serial.println(F("Monitoring button presses..."));
}

void lab3_1AppLoop() {
    static unsigned long lastTick = 0;
    unsigned long now = millis();
    if (now != lastTick) {
        lastTick = now;
        os_seq_scheduler_loop();   // 1 ms tick
    }
}
