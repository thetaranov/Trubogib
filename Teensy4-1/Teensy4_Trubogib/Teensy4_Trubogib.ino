/*
 * Teensy4_Trubogib.ino — PLC-free runtime for TubeBender
 * v4.3: unified status JSON for EXE + encoder/pedal/SON telemetry + I2C LCD
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include "ModbusRTU.h"

// I2C LCD address is typically 0x27 for PCF8574T
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ═════════════════════════════════════════════════════════════════════
// LCD_SCENARIOS — Display scenarios for 16x2 LCD (embedded)
// ═════════════════════════════════════════════════════════════════════
class LCDScenarios {
public:
    enum State {
        STATE_IDLE = 0,
        STATE_FEED = 1,
        STATE_ROTATION = 2,
        STATE_BENDING = 3,
        STATE_CLAMP_CLOSE = 4,
        STATE_CLAMP_OPEN = 5,
        STATE_ERROR = 6,
        STATE_CONNECTION = 7,
        STATE_DIAGNOSTICS = 8,
        STATE_HOMING = 9,
        STATE_CALIBRATION = 10
    };

    struct DisplayData {
        double currentZ_mm = 0.0;
        double targetZ_mm = 0.0;
        double currentC_deg = 0.0;
        double targetC_deg = 0.0;
        int currentBend_deg = 0;
        int targetBend_deg = 0;
        int currentStep = 0;
        int totalSteps = 0;
        State currentState = STATE_IDLE;
        uint32_t loopHz = 0;
        uint32_t stepHz = 0;
        bool isConnected = false;
        bool servoOn = false;
        bool limitZMinActive = false;
        bool limitZMaxActive = false;
        uint16_t errorCode = 0;
    };

    static void displayIdle(LiquidCrystal_I2C& lcd, const DisplayData& data) {
        char line1[17], line2[17];
        snprintf(line1, sizeof(line1), "C:%.1f%c", data.currentC_deg, '\xDF');
        snprintf(line2, sizeof(line2), "Z:%.1f%c Ready", data.currentZ_mm, 'm');
        lcd.setCursor(0, 0); lcd.print(line1);
        lcd.setCursor(0, 1); lcd.print(line2);
    }

    static void displayFeed(LiquidCrystal_I2C& lcd, const DisplayData& data) {
        char line1[25], line2[17];
        if (data.totalSteps > 0)
            snprintf(line1, sizeof(line1), "S %d/%d PODACHA", data.currentStep, data.totalSteps);
        else
            snprintf(line1, sizeof(line1), "PODACHA KapeTka");
        snprintf(line2, sizeof(line2), "%.1f/%.1f%c", data.currentZ_mm, data.targetZ_mm, 'm');
        lcd.setCursor(0, 0); lcd.print(line1);
        lcd.setCursor(0, 1); lcd.print(line2);
    }

    static void displayRotation(LiquidCrystal_I2C& lcd, const DisplayData& data) {
        char line1[25], line2[17];
        if (data.totalSteps > 0)
            snprintf(line1, sizeof(line1), "S %d/%d ROTACIA", data.currentStep, data.totalSteps);
        else
            snprintf(line1, sizeof(line1), "ROTACIA Tuba");
        snprintf(line2, sizeof(line2), "%.0f%c/%.0f%c", data.currentC_deg, '\xDF', data.targetC_deg, '\xDF');
        lcd.setCursor(0, 0); lcd.print(line1);
        lcd.setCursor(0, 1); lcd.print(line2);
    }

    static void displayBending(LiquidCrystal_I2C& lcd, const DisplayData& data) {
        char line1[25], line2[17];
        if (data.totalSteps > 0)
            snprintf(line1, sizeof(line1), "S %d/%d GIBKA", data.currentStep, data.totalSteps);
        else
            snprintf(line1, sizeof(line1), "GIBKA Rolik");
        snprintf(line2, sizeof(line2), "%d%c/%d%c", data.currentBend_deg, '\xDF', data.targetBend_deg, '\xDF');
        lcd.setCursor(0, 0); lcd.print(line1);
        lcd.setCursor(0, 1); lcd.print(line2);
    }

    static void displayClampClose(LiquidCrystal_I2C& lcd, const DisplayData& data) {
        char line1[17], line2[20];
        snprintf(line1, sizeof(line1), "S %d/%d PRIZHIM", data.currentStep, data.totalSteps);
        strncpy(line2, "████░░░░░░░░░░", sizeof(line2)-1);
        line2[sizeof(line2)-1] = 0;
        lcd.setCursor(0, 0); lcd.print(line1);
        lcd.setCursor(0, 1); lcd.print(line2);
    }

    static void displayClampOpen(LiquidCrystal_I2C& lcd, const DisplayData& data) {
        char line1[17], line2[20];
        snprintf(line1, sizeof(line1), "S %d/%d RAZZHIM", data.currentStep, data.totalSteps);
        strncpy(line2, "░░░░░░░░████░░░░", sizeof(line2)-1);
        line2[sizeof(line2)-1] = 0;
        lcd.setCursor(0, 0); lcd.print(line1);
        lcd.setCursor(0, 1); lcd.print(line2);
    }

    static void displayError(LiquidCrystal_I2C& lcd, const DisplayData& data) {
        char line1[17], line2[17];
        snprintf(line1, sizeof(line1), "!! OSHIBKA !!");
        if (data.limitZMinActive)
            snprintf(line2, sizeof(line2), "Limit KapeTka-");
        else if (data.limitZMaxActive)
            snprintf(line2, sizeof(line2), "Limit KapeTka+");
        else
            snprintf(line2, sizeof(line2), "ERR: 0x%04X", data.errorCode);
        lcd.setCursor(0, 0); lcd.print(line1);
        lcd.setCursor(0, 1); lcd.print(line2);
    }

    static void displayConnection(LiquidCrystal_I2C& lcd, const DisplayData& data) {
        char line1[17], line2[17];
        if (data.isConnected) {
            snprintf(line1, sizeof(line1), "CONNECTED");
            snprintf(line2, sizeof(line2), "v4.21 Ready");
        } else {
            snprintf(line1, sizeof(line1), "WAITING...");
            snprintf(line2, sizeof(line2), "Connect EXE");
        }
        lcd.setCursor(0, 0); lcd.print(line1);
        lcd.setCursor(0, 1); lcd.print(line2);
    }

    static void displayDiagnostics(LiquidCrystal_I2C& lcd, const DisplayData& data) {
        char line1[17], line2[17];
        snprintf(line1, sizeof(line1), "DIAG %luHz", data.stepHz);
        snprintf(line2, sizeof(line2), "Loop %luHz", data.loopHz);
        lcd.setCursor(0, 0); lcd.print(line1);
        lcd.setCursor(0, 1); lcd.print(line2);
    }

    static void displayHoming(LiquidCrystal_I2C& lcd, const DisplayData& data) {
        char line1[17], line2[20];
        snprintf(line1, sizeof(line1), "HOME KapeTka");
        strncpy(line2, "████░░░░░░░░░░", sizeof(line2)-1);
        line2[sizeof(line2)-1] = 0;
        lcd.setCursor(0, 0); lcd.print(line1);
        lcd.setCursor(0, 1); lcd.print(line2);
    }

    static void displayCalibration(LiquidCrystal_I2C& lcd, const DisplayData& data) {
        char line1[17], line2[17];
        snprintf(line1, sizeof(line1), "KALIBR");
        snprintf(line2, sizeof(line2), "Z:%.1f C:%.0f%c", data.currentZ_mm, data.currentC_deg, '\xDF');
        lcd.setCursor(0, 0); lcd.print(line1);
        lcd.setCursor(0, 1); lcd.print(line2);
    }

    static void update(LiquidCrystal_I2C& lcd, const DisplayData& data, State state) {
        switch (state) {
            case STATE_IDLE: displayIdle(lcd, data); break;
            case STATE_FEED: displayFeed(lcd, data); break;
            case STATE_ROTATION: displayRotation(lcd, data); break;
            case STATE_BENDING: displayBending(lcd, data); break;
            case STATE_CLAMP_CLOSE: displayClampClose(lcd, data); break;
            case STATE_CLAMP_OPEN: displayClampOpen(lcd, data); break;
            case STATE_ERROR: displayError(lcd, data); break;
            case STATE_CONNECTION: displayConnection(lcd, data); break;
            case STATE_DIAGNOSTICS: displayDiagnostics(lcd, data); break;
            case STATE_HOMING: displayHoming(lcd, data); break;
            case STATE_CALIBRATION: displayCalibration(lcd, data); break;
            default: displayIdle(lcd, data);
        }
    }
};

// ── LCD SCENARIOS ──
LCDScenarios::DisplayData lcdData;
LCDScenarios::State currentLCDState = LCDScenarios::STATE_IDLE;

#define FW_VERSION        "4.21"

// Production mode: SON commands toggle physical outputs.
#define SON_DRYRUN        0

// RS-485 Modbus -> VFD
#define RS485_DE          24
#define VFD_BAUD          9600

// Motion outputs (kept for compatibility)
#define PIN_STEP_Z        2
#define PIN_DIR_Z         3
#define PIN_STEP_C        4
#define PIN_DIR_C         5

// Bend encoder via PC817
#define PIN_ENC_BEND_A    6
#define PIN_ENC_BEND_B    7

// Servo feedback encoder inputs
#define PIN_ENC_Z_A       10
#define PIN_ENC_Z_B       11
#define PIN_ENC_C_A       12
#define PIN_ENC_C_B       13

// Inputs via PC817 (active LOW)
#define PIN_PEDAL_FWD     27
#define PIN_PEDAL_REV     28
#define PIN_MODE_SW       29
#define PIN_LIM_Z_MIN     31
#define PIN_LIM_Z_MAX     32

// Outputs via XY-MOS
#define PIN_SON_Z         33
#define PIN_SON_C         34

// Pneumatic cylinder control (separate pins for CLAMP/UNCLAMP)
// JQC-3FF-S-Z 5V relay (ACTIVE LOW logic)
#define PIN_CLAMP         35  // LOW = close/press (active LOW!)
#define PIN_UNCLAMP       36  // LOW = open/release (active LOW!)

// ─────────────────────────────────────────────────────────────────────
// Unified 1:1 scale (EXE ↔ Teensy ↔ Servo). The gearbox lives HERE, in
// Teensy only. EXE speaks mm/deg/s only; servo electronic gear is 1:1
// (PA06=PA07=1), so 1 command pulse = 1 increment of PA05 "FBP" at the
// servo amp, i.e. 10000 command pulses = 1 motor rev.
//
// Two DIFFERENT scales are used:
//   • *_CMD_TICKS_PER_*  — pulses Teensy SENDS (step out). Derived from
//                           PA05 = 10000 cmd pulses per motor rev.
//   • *_FB_TICKS_PER_*   — ticks Teensy DECODES from LA/LB output
//                           (x4 of PA15 = 16000 ticks per motor rev).
//
// Mechanics:
//   Z: rack M2, Z35, gearbox 1:8 → 27.49 mm per motor rev
//   C: Z35/Z105 × 1:20 → i=60:1, 6° output per motor rev
// ─────────────────────────────────────────────────────────────────────
// MR-JE-A (Z/C axes) — command side:
//   PA05 FBP = 10000 (command pulses/rev), PA06=1, PA07=1 (electronic gear 1:1)
// Feedback side (PA15 ENR encoder output pulses/rev, ×4 after quadrature):
//   This constant is CALIBRATED EMPIRICALLY via hand-rotation test:
//     1 pinion revolution = 8 motor revs = Z_MM_PER_REV × 8 = 219.92 mm.
//   Measurement 2026-04-16 with fixed main-loop (no VFD 100 ms blocks, LCD 500 ms):
//     hand-rotation 1 pinion rev → 901 mm displayed at ENC_X4=964
//     → capture was 98.7 % of 32 000 physical ticks → PA15 = 1000 in drive.
//   Setting ENC_X4 = PA15 × 4 = 4000 makes the display read the true 220 mm.
// IF YOU CHANGE PA15 IN THE DRIVE: recompute ENC_X4 = new_PA15 × 4 and rerun the
// hand-rotation test. Polled-tick count per 1 pinion rev appears in the log as
// the difference in gZfbTicks — divide by 8 to get PA15.
#define ENC_X4               946.0          // EMPIRICALLY CALIBRATED: hand-rotation 1 full Z35 rev shows 220mm, was showing 6.51mm → scale factor 220/6.51 = 33.8
#define SERVO_CMD_PULSES_REV 10000.0        // PA05 command base
#define Z_MM_PER_REV         27.49          // M2 Z35 pinion (π×70=219.91 mm) ÷ 1:8 reducer
#define C_DEG_PER_REV        (360.0 / 60.0) // 1:60 total reduction → 6°/motor rev

// Command side (Teensy → servo step/dir)
#define Z_CMD_TICKS_PER_MM   (SERVO_CMD_PULSES_REV / Z_MM_PER_REV)   // ≈ 363.77
#define C_CMD_TICKS_PER_DEG  (SERVO_CMD_PULSES_REV / C_DEG_PER_REV)  // ≈ 1666.67

// Feedback side
#define Z_FB_TICKS_PER_MM    (ENC_X4 / Z_MM_PER_REV)                 // ≈ 34.4 (946 / 27.49) - CALIBRATED
#define C_FB_TICKS_PER_DEG   (ENC_X4 / C_DEG_PER_REV)                // ≈ 157.67

// Scale factor between feedback ticks and command pulses (10000/946 = 10.57).
#define CMD_PER_FB           (SERVO_CMD_PULSES_REV / ENC_X4)         // = 10.57

// Simplified runtime state
volatile long gBendTicks = 0;
volatile long gZfbTicks = 0;
volatile long gCfbTicks = 0;
long gBendTicksLast = 0;
int gBendTargetDeg = 0;
int gBendCycle = 1;
bool gBendDone = false;
int gBendingState = 0;
bool gClampClosed = false; // State of clamp (true = closed, false = open)
bool gVfdRunning = false;
bool gVfdForward = true;
uint16_t gVfdFreqHz100 = 0;

// VFD Modbus control variables
ModbusRTU* gVfdModbus = nullptr;
bool gVfdSimulationMode = false;      // When true, simulate VFD instead of real communication
float gVfdSimFreq = 0.0;              // Simulated frequency (Hz)
uint16_t gVfdTargetFreqCode = 0;      // Target frequency in 0.01Hz units (0-6000 = 0-60Hz)
uint16_t gVfdControlWord = 0;         // Control word: bit 0=RUN, bit 1=FWD(0)/REV(1)
uint32_t gVfdLastUpdateMs = 0;
uint16_t gVfdCurrentFreqCode = 0;     // Feedback: current frequency (0.01Hz units)
uint16_t gVfdCurrentCurrent = 0;      // Feedback: current in 0.1A units
bool gVfdStatusValid = false;
uint32_t gVfdPollLastMs = 0;

double gZmm = 0.0;
double gCdeg = 0.0;
int gZst = 0; // 0 idle
int gCst = 0;

unsigned long gMbRx = 0;
unsigned long gMbTx = 0;
unsigned long gMbErr = 0;
unsigned long gMbLink = 0;
unsigned long gResetCause = 0;

String gRxLine;

// Diagnostics counters
volatile uint32_t gStepIsrCount = 0;
volatile uint32_t gBendIsrCount = 0;
volatile uint32_t gZfbIsrCount = 0;
volatile uint32_t gCfbIsrCount = 0;
uint32_t gDiagStepHz = 0;
uint32_t gDiagBendHz = 0;
uint32_t gDiagZfbHz = 0;
uint32_t gDiagCfbHz = 0;
uint32_t gDiagLoopHz = 0;
uint32_t gDiagLoopMaxUs = 0;
uint32_t gDiagRxCharsPerSec = 0;
uint32_t gDiagRxOverflowPerSec = 0;
uint32_t gRxCharsAcc = 0;
uint32_t gRxOverflowAcc = 0;

// ─────────────────────────────────────────────────────────────────────
//  Калибровка осей: персистентное хранение в EEPROM Teensy 4.1.
//  При включении читаем сохранённые тики энкодеров и восстанавливаем.
//  При получении CAL/BZ от EXE — обновляем и сохраняем.
// ─────────────────────────────────────────────────────────────────────
struct CalibPersist {
  uint32_t magic;        // 0xCA1B4242
  long     bendTicks;
  long     zFbTicks;
  long     cFbTicks;
  uint32_t crc;          // simple XOR sum
};
static const uint32_t CALIB_MAGIC = 0xCA1B4242UL;
static const int      CALIB_ADDR  = 0;

static uint32_t calibCrc(const CalibPersist& c) {
  uint32_t s = 0;
  s ^= c.magic;
  s ^= (uint32_t)c.bendTicks;
  s ^= (uint32_t)c.zFbTicks;
  s ^= (uint32_t)c.cFbTicks;
  return s ^ 0xA5A5A5A5UL;
}

void saveCalibration() {
  CalibPersist c;
  c.magic = CALIB_MAGIC;
  noInterrupts();
  c.bendTicks = gBendTicks;
  c.zFbTicks  = gZfbTicks;
  c.cFbTicks  = gCfbTicks;
  interrupts();
  c.crc = calibCrc(c);
  EEPROM.put(CALIB_ADDR, c);
}

bool loadCalibration() {
  CalibPersist c;
  EEPROM.get(CALIB_ADDR, c);
  if (c.magic != CALIB_MAGIC) return false;
  if (c.crc != calibCrc(c)) return false;
  noInterrupts();
  gBendTicks = c.bendTicks;
  gZfbTicks  = c.zFbTicks;
  gCfbTicks  = c.cFbTicks;
  interrupts();
  gZmm  = (double)c.zFbTicks / Z_FB_TICKS_PER_MM;
  gCdeg = (double)c.cFbTicks / C_FB_TICKS_PER_DEG;
  return true;
}

static inline bool pedalFwdPressed() { return digitalRead(PIN_PEDAL_FWD) == LOW; }
static inline bool pedalRevPressed() { return digitalRead(PIN_PEDAL_REV) == LOW; }
static inline bool modeAuto() { return digitalRead(PIN_MODE_SW) == LOW; }
static inline bool limZMin() { return digitalRead(PIN_LIM_Z_MIN) == LOW; }
static inline bool limZMax() { return digitalRead(PIN_LIM_Z_MAX) == LOW; }

void isrBendEncoder() {
  gBendIsrCount++;
  // Proper X4 quadrature decode using lookup table
  static uint8_t gBendPrevISR = 0;
  uint8_t a = digitalReadFast(PIN_ENC_BEND_A) ? 1 : 0;
  uint8_t b = digitalReadFast(PIN_ENC_BEND_B) ? 1 : 0;
  uint8_t cur = (a << 1) | b;
  
  static const int8_t lut[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };
  int8_t d = lut[(gBendPrevISR << 2) | cur];
  gBendTicks += d;
  gBendPrevISR = cur;
}

void isrZfbEncoder() {
  gZfbIsrCount++;
  // Proper X4 quadrature decode using lookup table
  static uint8_t gZPrevISR = 0;
  uint8_t a = digitalReadFast(PIN_ENC_Z_A) ? 1 : 0;
  uint8_t b = digitalReadFast(PIN_ENC_Z_B) ? 1 : 0;
  uint8_t cur = (a << 1) | b;
  
  static const int8_t lut[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };
  int8_t d = lut[(gZPrevISR << 2) | cur];
  gZfbTicks += d;
  gZPrevISR = cur;
}

void isrCfbEncoder() {
  gCfbIsrCount++;
  // Proper X4 quadrature decode using lookup table
  static uint8_t gCPrevISR = 0;
  uint8_t a = digitalReadFast(PIN_ENC_C_A) ? 1 : 0;
  uint8_t b = digitalReadFast(PIN_ENC_C_B) ? 1 : 0;
  uint8_t cur = (a << 1) | b;
  
  static const int8_t lut[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };
  int8_t d = lut[(gCPrevISR << 2) | cur];
  gCfbTicks += d;
  gCPrevISR = cur;
}

// Polled quadrature decode for Z/C feedback (no interrupts).
// This keeps encoder feedback available but avoids ISR storms when SON is enabled.
static uint8_t gZPrevQ = 0;
static uint8_t gCPrevQ = 0;

static inline int8_t quadDelta(uint8_t prev, uint8_t cur) {
  static const int8_t lut[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };
  return lut[(prev << 2) | cur];
}

void pollFeedbackEncoders() {
  uint8_t za = digitalReadFast(PIN_ENC_Z_A) ? 1 : 0;
  uint8_t zb = digitalReadFast(PIN_ENC_Z_B) ? 1 : 0;
  uint8_t zq = (za << 1) | zb;
  int8_t dz = quadDelta(gZPrevQ, zq);
  if (dz != 0) {
    gZfbTicks += dz;
  }
  gZPrevQ = zq;

  uint8_t ca = digitalReadFast(PIN_ENC_C_A) ? 1 : 0;
  uint8_t cb = digitalReadFast(PIN_ENC_C_B) ? 1 : 0;
  uint8_t cq = (ca << 1) | cb;
  int8_t dc = quadDelta(gCPrevQ, cq);
  if (dc != 0) {
    gCfbTicks += dc;
  }
  gCPrevQ = cq;
}

static inline int bendDegFromTicks(long ticks) {
  // Bend encoder: 360 PPR simple + quadrature X4 decode = 1440 ticks per revolution
  // 1440 ticks = 360 degrees, so: 1 degree = 4 ticks
  // Therefore: degrees = ticks / 4
  return (int)lround(ticks / 4.0);
}

// ═════════════════════════════════════════════════════════════════════
// Low-Pass Filter for encoder feedback (eliminates pulsation)
// ═════════════════════════════════════════════════════════════════════
#define ENC_FILTER_SAMPLES 16  // Number of samples for moving average
static double zMmFilterBuffer[ENC_FILTER_SAMPLES] = {0};
static double cDegFilterBuffer[ENC_FILTER_SAMPLES] = {0};
static uint8_t zMmFilterIndex = 0;
static uint8_t cDegFilterIndex = 0;

// Helper: compute moving average (called with new raw value)
static double updateMovingAverage(double newValue, double* buffer, uint8_t bufLen, uint8_t& idx) {
  buffer[idx] = newValue;
  idx = (idx + 1) % bufLen;
  double sum = 0.0;
  for (uint8_t i = 0; i < bufLen; i++) {
    sum += buffer[i];
  }
  return sum / bufLen;
}

// --- Step Generator Variables ---
IntervalTimer stepTimer;
bool gStepTimerRunning = false;
volatile bool gArcActive = false;
volatile long gArcStartBendTicks = 0;
volatile double gArcRadius = 0.0;
volatile long gArcStartZPulse = 0;
volatile double gArcTargetAngle = 0.0;

volatile long gZTargetPulse = 0;
volatile long gZCurrentPulse = 0;
volatile bool gZStepHigh = false;
volatile double gZVirtualPulse = 0.0;
volatile double gZSpeedStep = 0.0;

volatile long gCTargetPulse = 0;
volatile long gCCurrentPulse = 0;
volatile bool gCStepHigh = false;
volatile double gCVirtualPulse = 0.0;
volatile double gCSpeedStep = 0.0;

// Trapezoidal acceleration profile
#define DEFAULT_Z_ACCEL     250.0   // mm/s²  (ramp ~0.06s at 100mm/s)
#define DEFAULT_C_ACCEL     100.0   // deg/s² (ramp ~0.05s at 30deg/s)
#define MIN_Z_SPEED        20.0    // mm/s (start/stop speed) - INCREASED from 2.0
#define MIN_C_SPEED        1.0    // deg/s

volatile double gZMaxSpeed = 0.0;
volatile double gZCurSpeed = 0.0;
volatile double gZAccelRate = 0.0;
volatile double gZMinSpeed = 0.0;

volatile double gCMaxSpeed = 0.0;
volatile double gCCurSpeed = 0.0;
volatile double gCAccelRate = 0.0;
volatile double gCMinSpeed = 0.0;

// Step info from EXE for LCD display
volatile int gStepCurrent = 0;
volatile int gStepTotal = 0;

// Step ISR frequency. Step generator uses a 2-ISR-tick HIGH/LOW cycle, so
// max pulse rate = STEP_ISR_HZ / 2.
//   40 kHz ISR →  20 kHz STEP → ~55  mm/s feed (old, too slow)
//   80 kHz ISR →  40 kHz STEP → ~110 mm/s feed (previous)
//  200 kHz ISR → 100 kHz STEP → ~275 mm/s feed (current — matches docs: 300 mm/s working speed)
// Teensy 4.1 at 600 MHz handles 200 kHz FP ISR comfortably.
static constexpr double STEP_ISR_HZ = 200000.0;

void stepIsr() {
  gStepIsrCount++;
  // 1) During ARCFEED: recompute the synchronized Z target from bend ticks.
  //    Do NOT instantly jump gZVirtualPulse — let the rate limiter below
  //    catch up at gZSpeedStep ticks per ISR. Instant-jumping caused the
  //    "servo speed runaway" where the carriage flew at the maximum step
  //    rate (~hundreds of mm/s) during bending, far above the configured
  //    feed speed.
  if (gArcActive) {
    long ticksMoved = abs(gBendTicks - gArcStartBendTicks);
    double degreesMoved = ticksMoved / 4.0;  // Quadrature X4: 4 ticks = 1 degree
    
    // Check if target angle has been reached
    if (degreesMoved >= gArcTargetAngle) {
      // Auto-stop the arc when target angle is achieved
      gArcActive = false;
      gZTargetPulse = (long)gZVirtualPulse;
    } else {
      // Still bending: update Z position to follow arc
      double mmMoved = (gArcRadius * 3.1415926535 * degreesMoved) / 180.0;
      long additionalPulse = (long)(mmMoved * Z_CMD_TICKS_PER_MM);
      gZTargetPulse = gArcStartZPulse + additionalPulse;
      gZst = 2;
    }
  }

  // 2) Trapezoidal acceleration for Z axis (non-arc moves).
  //    During ARCFEED, use constant rate limiter (gZSpeedStep) since
  //    the target moves continuously with bend encoder.
  if (!gArcActive && gZst == 2 && gZAccelRate > 1e-15) {
    double remaining = fabs((double)gZTargetPulse - gZVirtualPulse);
    if (remaining > 1.0) {
      double stopDist = (gZCurSpeed * gZCurSpeed) / (2.0 * gZAccelRate);
      if (stopDist >= remaining) {
        gZCurSpeed -= gZAccelRate;
        if (gZCurSpeed < gZMinSpeed) gZCurSpeed = gZMinSpeed;
      } else if (gZCurSpeed < gZMaxSpeed) {
        gZCurSpeed += gZAccelRate;
        if (gZCurSpeed > gZMaxSpeed) gZCurSpeed = gZMaxSpeed;
      }
    } else {
      gZCurSpeed = gZMinSpeed;
    }
  }

  double zStep = (gArcActive || gZAccelRate < 1e-15) ? gZSpeedStep : gZCurSpeed;

  if (gZVirtualPulse < gZTargetPulse) {
    gZVirtualPulse += zStep;
    if (gZVirtualPulse > gZTargetPulse) gZVirtualPulse = gZTargetPulse;
  } else if (gZVirtualPulse > gZTargetPulse) {
    gZVirtualPulse -= zStep;
    if (gZVirtualPulse < gZTargetPulse) gZVirtualPulse = gZTargetPulse;
  }

  // 3) Trapezoidal acceleration for C axis
  if (gCst == 2 && gCAccelRate > 1e-15) {
    double remaining = fabs((double)gCTargetPulse - gCVirtualPulse);
    if (remaining > 1.0) {
      double stopDist = (gCCurSpeed * gCCurSpeed) / (2.0 * gCAccelRate);
      if (stopDist >= remaining) {
        gCCurSpeed -= gCAccelRate;
        if (gCCurSpeed < gCMinSpeed) gCCurSpeed = gCMinSpeed;
      } else if (gCCurSpeed < gCMaxSpeed) {
        gCCurSpeed += gCAccelRate;
        if (gCCurSpeed > gCMaxSpeed) gCCurSpeed = gCMaxSpeed;
      }
    } else {
      gCCurSpeed = gCMinSpeed;
    }
  }

  double cStep = (gCAccelRate < 1e-15) ? gCSpeedStep : gCCurSpeed;

  if (gCVirtualPulse < gCTargetPulse) {
    gCVirtualPulse += cStep;
    if (gCVirtualPulse > gCTargetPulse) gCVirtualPulse = gCTargetPulse;
  } else if (gCVirtualPulse > gCTargetPulse) {
    gCVirtualPulse -= cStep;
    if (gCVirtualPulse < gCTargetPulse) gCVirtualPulse = gCTargetPulse;
  }

  // Z axis
  // DIR convention: positive Z (carriage feed toward bender) = LOW (INVERTED - FIXED).
  // Servo DIR signal is electrically inverted on hardware
  if (gZst == 2) {
    long zt = (long)gZVirtualPulse;
    if (zt - gZCurrentPulse > 0) {
      if (!gZStepHigh) {
        digitalWriteFast(PIN_DIR_Z, LOW); // forward = toward bender (INVERTED)
        digitalWriteFast(PIN_STEP_Z, HIGH);
        gZStepHigh = true;
      } else {
        digitalWriteFast(PIN_STEP_Z, LOW);
        gZStepHigh = false;
        gZCurrentPulse += 1;
      }
    } else if (gZCurrentPulse - zt > 0) {
      if (!gZStepHigh) {
        digitalWriteFast(PIN_DIR_Z, HIGH); // reverse = away from bender (INVERTED)
        digitalWriteFast(PIN_STEP_Z, HIGH);
        gZStepHigh = true;
      } else {
        digitalWriteFast(PIN_STEP_Z, LOW);
        gZStepHigh = false;
        gZCurrentPulse -= 1;
      }
    } else {
      if ((long)gZVirtualPulse == gZTargetPulse) gZst = 0;
    }
  }

  // C axis
  if (gCst == 2) {
    long ct = (long)gCVirtualPulse;
    if (ct - gCCurrentPulse > 0) {
      if (!gCStepHigh) {
        digitalWriteFast(PIN_DIR_C, LOW); // ORIGINAL DIR
        digitalWriteFast(PIN_STEP_C, HIGH);
        gCStepHigh = true;
      } else {
        digitalWriteFast(PIN_STEP_C, LOW);
        gCStepHigh = false;
        gCCurrentPulse += 1;
      }
    } else if (gCCurrentPulse - ct > 0) {
      if (!gCStepHigh) {
        digitalWriteFast(PIN_DIR_C, HIGH); // ORIGINAL DIR
        digitalWriteFast(PIN_STEP_C, HIGH);
        gCStepHigh = true;
      } else {
        digitalWriteFast(PIN_STEP_C, LOW);
        gCStepHigh = false;
        gCCurrentPulse -= 1;
      }
    } else {
      if ((long)gCVirtualPulse == gCTargetPulse && ct == gCCurrentPulse) gCst = 0;
    }
  }
}

void updateStepTimerState() {
  // Reverted: running intervalTimer dynamically at 40kHz was causing issues or dropping USB.
  // Instead it will permanently run like it used to.
}

bool gServoOn = false;

void sendStatusJson() {
  long rawTicks = gBendTicks;
  long zFb, cFb;
  noInterrupts();
  zFb = gZfbTicks;
  cFb = gCfbTicks;
  interrupts();
  double zFbMm_raw  = (double)zFb / Z_FB_TICKS_PER_MM;
  double cFbDeg_raw = (double)cFb / C_FB_TICKS_PER_DEG;
  
  // Apply Low-Pass Filter (moving average) to eliminate encoder pulsation
  double zFbMm  = updateMovingAverage(zFbMm_raw, zMmFilterBuffer, ENC_FILTER_SAMPLES, zMmFilterIndex);
  double cFbDeg = updateMovingAverage(cFbDeg_raw, cDegFilterBuffer, ENC_FILTER_SAMPLES, cDegFilterIndex);
  
  int bendCur = bendDegFromTicks(rawTicks);
  bool fwd = pedalFwdPressed();
  bool rev = pedalRevPressed();
  bool autoMode = modeAuto();

  int sonZOut = digitalRead(PIN_SON_Z) ? 1 : 0;
  int sonCOut = digitalRead(PIN_SON_C) ? 1 : 0;
  int pedalIn = pedalFwdPressed() ? 1 : 0;
  int modeIn = modeAuto() ? 1 : 0;

  // Determine VFD online status.
  // True ONLINE requires: at least one successful RX reply AND the error rate is
  // not dominating the traffic. Previously we reported "ok=1" after firmware
  // boot simply because gMbTx>0, which gave a false-positive "VFD online"
  // indicator in EXE even though every transaction timed out.
  int mbOk = 0;
  if (gMbRx > 0 && gMbTx > 0) {
    // Require successful replies to outnumber hard errors (allow 10% margin)
    if (gMbErr == 0 || gMbRx * 10 > gMbErr) mbOk = 1;
  }

  char out[768];
  snprintf(out, sizeof(out),
    "{\"v\":\"%s\"," \
    "\"mb\":{\"rx\":%lu,\"tx\":%lu,\"err\":%lu,\"link\":%lu,\"ok\":%d}," \
    "\"z\":{\"mm\":%.3f,\"st\":%d,\"son\":%d}," \
    "\"c\":{\"deg\":%.3f,\"st\":%d,\"son\":%d}," \
    "\"enc\":{\"zCnt\":%ld,\"cCnt\":%ld,\"zMm\":%.3f,\"cDeg\":%.3f}," \
    "\"bend\":{\"cur\":%d,\"tgt\":%d,\"cyc\":%d,\"mode\":%d,\"done\":%d,\"pedalFwd\":%d,\"pedalRev\":%d,\"clampClosed\":%d}," \
    "\"vfd\":{\"freq\":%.2f,\"freqTgt\":%.2f,\"cur\":%.2f,\"ctrlWord\":\"0x%04X\",\"sim\":%d}," \
    "\"lim\":{\"zMin\":%d,\"zMax\":%d}," \
    "\"boot\":{\"rst\":%lu,\"upMs\":%lu}," \
    "\"diag\":{\"loopHz\":%lu,\"loopMaxUs\":%lu,\"stepHz\":%lu,\"bendIsrHz\":%lu,\"zIsrHz\":%lu,\"cIsrHz\":%lu,\"rxHz\":%lu,\"rxOvHz\":%lu,\"sonZOut\":%d,\"sonCOut\":%d,\"pedalIn\":%d,\"modeIn\":%d}," \
    "\"wd\":0}",
    FW_VERSION,
    gMbRx, gMbTx, gMbErr, gMbLink, mbOk,
    gZmm, gZst, gServoOn ? 1 : 0,
    gCdeg, gCst, gServoOn ? 1 : 0,
    zFb, cFb, zFbMm, cFbDeg,
    bendCur, gBendTargetDeg, gBendCycle, autoMode ? 1 : 0, gBendDone ? 1 : 0, fwd ? 1 : 0, rev ? 1 : 0, gClampClosed ? 1 : 0,
    gVfdCurrentFreqCode / 100.0,
    gVfdTargetFreqCode / 100.0,
    gVfdCurrentCurrent / 10.0,
    gVfdControlWord,
    (gVfdSimulationMode ? 1 : 0),
    limZMin() ? 1 : 0, limZMax() ? 1 : 0,
    gResetCause, millis(),
    (unsigned long)gDiagLoopHz,
    (unsigned long)gDiagLoopMaxUs,
    (unsigned long)gDiagStepHz,
    (unsigned long)gDiagBendHz,
    (unsigned long)gDiagZfbHz,
    (unsigned long)gDiagCfbHz,
    (unsigned long)gDiagRxCharsPerSec,
    (unsigned long)gDiagRxOverflowPerSec,
    sonZOut, sonCOut, pedalIn, modeIn
  );
  Serial.println(out);
}

void cmdSon(int on) {
  gServoOn = (on != 0);
#if SON_DRYRUN
  digitalWrite(PIN_SON_Z, LOW);
  digitalWrite(PIN_SON_C, LOW);
  Serial.println("OK SON DRYRUN");
#else
  digitalWrite(PIN_SON_Z, on ? HIGH : LOW);
  digitalWrite(PIN_SON_C, on ? HIGH : LOW);

  Serial.println("OK SON");
#endif
}

// ──── VFD Control Functions ────

void cmdVfdFrequency(float freqHz, int direction) {
  // Convert frequency to Modbus code (0.01Hz units)
  // 50Hz = 5000 code, 60Hz = 6000 code
  uint16_t freqCode = (uint16_t)(freqHz * 100.0);
  
  // Clamp to valid range (0-6000 = 0-60Hz)
  if (freqCode > 6000) freqCode = 6000;
  
  // Build control word
  uint16_t controlWord = 0;
  if (freqCode > 0) {
    controlWord |= 0x0001;  // bit 0: RUN
    if (direction == 0) {
      // Forward = bit 1 clear
    } else {
      controlWord |= 0x0002;  // bit 1: REVERSE
    }
  }
  
  gVfdTargetFreqCode = freqCode;
  gVfdControlWord = controlWord;
  gVfdLastUpdateMs = millis();
  
  if (gVfdSimulationMode) {
    // Simulate VFD frequency ramp
    gVfdSimFreq = freqHz;
    gVfdCurrentFreqCode = freqCode;
    Serial.println("OK VFDFREQ (simulated)");
  } else {
    // Send to real VFD via Modbus
    if (gVfdModbus) {
      bool freqOk = gVfdModbus->writeFrequency(freqCode);
      bool ctrlOk = gVfdModbus->writeControl(controlWord);
      if (freqOk && ctrlOk) {
        gVfdCurrentFreqCode = freqCode;
        gMbTx += 2;
        Serial.println("OK VFDFREQ");
      } else {
        gMbErr++;
        Serial.println("ERR VFDFREQ modbus failed");
      }
    }
  }
}

void cmdVfdStop() {
  gVfdTargetFreqCode = 0;
  gVfdControlWord = 0;
  
  if (gVfdSimulationMode) {
    gVfdSimFreq = 0.0;
    gVfdCurrentFreqCode = 0;
    Serial.println("OK VFDSTOP (simulated)");
  } else {
    if (gVfdModbus) {
      gVfdModbus->writeControl(0);  // Stop bit
      gVfdModbus->writeFrequency(0);
      gVfdCurrentFreqCode = 0;
      gMbTx += 2;
      Serial.println("OK VFDSTOP");
    }
  }
}

void pollVfdStatus() {
  if (gVfdSimulationMode) {
    // Simulate VFD response with frequency ramp
    float targetFreq = gVfdTargetFreqCode / 100.0;
    if (gVfdSimFreq < targetFreq) {
      gVfdSimFreq += 2.0;  // Ramp at 2 Hz/update
      if (gVfdSimFreq > targetFreq) {
        gVfdSimFreq = targetFreq;
      }
    } else if (gVfdSimFreq > targetFreq) {
      gVfdSimFreq -= 2.0;
      if (gVfdSimFreq < targetFreq) {
        gVfdSimFreq = targetFreq;
      }
    }
    gVfdCurrentFreqCode = (uint16_t)(gVfdSimFreq * 100.0);
    gVfdCurrentCurrent = 0;  // No current feedback in simulation
    gVfdStatusValid = true;
    gMbRx++;
  } else {
    // Read actual VFD status via Modbus
    if (gVfdModbus) {
      uint16_t statusRegs[3];
      if (gVfdModbus->readRegisters(0x0100, 3, statusRegs)) {
        gVfdCurrentFreqCode = statusRegs[1];  // Frequency feedback
        gVfdCurrentCurrent = statusRegs[2];   // Current feedback
        gVfdStatusValid = true;
        gMbRx++;
      } else {
        gMbErr++;
      }
    }
  }
}

// ──── End VFD Control Functions ────

void cmdEStop() {
  // ESTOP must be as thorough as STOP: freeze step generators immediately
  // to prevent ISR from chasing stale targets while servos are being disabled.
  noInterrupts();
  gVfdRunning = false;
  gBendDone = true;
  gArcActive = false;
  gZst = 0;
  gCst = 0;
  // Freeze step generators at current physical position
  gZTargetPulse = (long)(gZfbTicks * CMD_PER_FB);
  gZVirtualPulse = (double)(gZfbTicks * CMD_PER_FB);
  gZCurrentPulse = (long)(gZfbTicks * CMD_PER_FB);
  gCTargetPulse = (long)(gCfbTicks * CMD_PER_FB);
  gCVirtualPulse = (double)(gCfbTicks * CMD_PER_FB);
  gCCurrentPulse = (long)(gCfbTicks * CMD_PER_FB);
  // Reset trapezoidal speeds
  gZCurSpeed = 0;
  gCCurSpeed = 0;
  // Reset bending state
  gBendingState = 0;
  gBendTargetDeg = 0;
  gStepCurrent = 0;
  gStepTotal = 0;
  interrupts();
  // Disable servos, VFD, and clamps
  gServoOn = false;
  digitalWrite(PIN_SON_Z, LOW);
  digitalWrite(PIN_SON_C, LOW);
  digitalWrite(PIN_CLAMP, HIGH);    // Both HIGH = both disabled (active LOW!)
  digitalWrite(PIN_UNCLAMP, HIGH);  // Both HIGH = both disabled (active LOW!)
  
  // Stop VFD
  cmdVfdStop();
  
  Serial.println("OK ESTOP");
}

unsigned long gLastExePing = 0;

void handleCommand(const String& line) {
  gLastExePing = millis();
  if (line.length() == 0) return;

  if (line == "?") {
    sendStatusJson();
    return;
  }
  if (line == "STOP" || line == "CMD 2") {
    noInterrupts();
    gVfdRunning = false;
    gBendDone = true;
    gZst = 0;
    gCst = 0;
    gArcActive = false;
    // Freeze step generator at its current physical position so the
    // rate limiter does not chase a stale target after STOP.
    gZTargetPulse = (long)(gZfbTicks * CMD_PER_FB);
    gZVirtualPulse = (double)(gZfbTicks * CMD_PER_FB);
    gZCurrentPulse = (long)(gZfbTicks * CMD_PER_FB);
    gCTargetPulse = (long)(gCfbTicks * CMD_PER_FB);
    gCVirtualPulse = (double)(gCfbTicks * CMD_PER_FB);
    gCCurrentPulse = (long)(gCfbTicks * CMD_PER_FB);
    // Reset bending state so LCD does not stay on a step screen
    gBendingState = 0;
    gBendTargetDeg = 0;
    // Reset trapezoidal speeds
    gZCurSpeed = 0;
    gCCurSpeed = 0;
    gStepCurrent = 0;
    gStepTotal = 0;
    interrupts();
    Serial.println("OK STOP");
    return;
  }
  if (line == "ESTOP") {
    cmdEStop();
    return;
  }
  if (line == "CLAMP") {
    gClampClosed = true;
    // Ensure unclamp is OFF first (ACTIVE LOW: HIGH = OFF)
    digitalWrite(PIN_UNCLAMP, HIGH);
    delay(50);  // Wait for relay to release before activating clamp
    // Now activate clamp (ACTIVE LOW: LOW = ON)
    digitalWrite(PIN_CLAMP, LOW);
    Serial.println("OK CLAMP");
    return;
  }
  if (line == "UNCLAMP") {
    gClampClosed = false;
    // Ensure clamp is OFF first (ACTIVE LOW: HIGH = OFF)
    digitalWrite(PIN_CLAMP, HIGH);
    delay(50);  // Wait for relay to release before activating unclamp
    // Now activate unclamp (ACTIVE LOW: LOW = ON)
    digitalWrite(PIN_UNCLAMP, LOW);
    Serial.println("OK UNCLAMP");
    return;
  }
  // RELAYOFF — de-energize BOTH pneumatic relays simultaneously.
  // Needed so the operator can completely cut clamp air from the UI
  // (previously toggling either button only swapped between the two
  // active states and never let both go OFF at once).
  if (line == "RELAYOFF") {
    gClampClosed = false;
    digitalWrite(PIN_CLAMP,   HIGH);  // ACTIVE LOW: HIGH = OFF
    digitalWrite(PIN_UNCLAMP, HIGH);
    Serial.println("OK RELAYOFF");
    return;
  }
  // Individual pin control for testing (PIN35 ON/OFF, PIN36 ON/OFF)
  if (line == "PIN35 ON") {
    digitalWrite(PIN_CLAMP, HIGH);  // ACTIVE LOW: HIGH = OFF
    Serial.println("OK PIN35 ON");
    return;
  }
  if (line == "PIN35 OFF") {
    digitalWrite(PIN_CLAMP, LOW);   // ACTIVE LOW: LOW = ON
    Serial.println("OK PIN35 OFF");
    return;
  }
  if (line == "PIN36 ON") {
    digitalWrite(PIN_UNCLAMP, HIGH); // ACTIVE LOW: HIGH = OFF
    Serial.println("OK PIN36 ON");
    return;
  }
  if (line == "PIN36 OFF") {
    digitalWrite(PIN_UNCLAMP, LOW);  // ACTIVE LOW: LOW = ON
    Serial.println("OK PIN36 OFF");
    return;
  }
  // Relay test command: RELAY_TEST <pin> <state>
  if (line.startsWith("RELAY_TEST")) {
    int spaceIdx = line.indexOf(' ');
    if (spaceIdx > 0) {
      int pin = line.substring(spaceIdx + 1).toInt();
      int spaceIdx2 = line.indexOf(' ', spaceIdx + 1);
      int state = 0;
      if (spaceIdx2 > 0) {
        state = line.substring(spaceIdx2 + 1).toInt();
      }
      digitalWrite(pin, state ? HIGH : LOW);
      Serial.print("OK RELAY_TEST ");
      Serial.print(pin);
      Serial.print(" ");
      Serial.println(state);
    }
    return;
  }
  if (line == "ZP") {
    gZmm = 0.0;
    gCdeg = 0.0;
    gBendTicks = 0;
    gBendTicksLast = 0; // Fix: reset tracking variable too
    noInterrupts();
    gZfbTicks = 0;
    gCfbTicks = 0;
    interrupts();
    // NOTE: ZP is runtime zeroing for current cycle, do not persist to EEPROM.
    Serial.println("OK ZP");
    return;
  }
  if (line == "SYNC") {
    Serial.println("OK SYNC");
    return;
  }
  if (line == "BZ") {
    // Zero bend encoder ONLY (carriage Z and chuck C untouched), persist
    noInterrupts();
    gBendTicks = 0;
    gBendTicksLast = 0; // Fix: reset tracking variable too
    interrupts();
    saveCalibration();
    Serial.println("OK BZ");
    return;
  }
  if (line.startsWith("CAL ")) {
    // CAL z,c,bend  — set all three axis displays at once and persist
    String s = line.substring(4);
    int c1 = s.indexOf(',');
    int c2 = (c1 > 0) ? s.indexOf(',', c1 + 1) : -1;
    if (c1 > 0 && c2 > 0) {
      double newZ = s.substring(0, c1).toFloat();
      double newC = s.substring(c1 + 1, c2).toFloat();
      int    newB = s.substring(c2 + 1).toInt();
      gZmm  = newZ;
      gCdeg = newC;
      noInterrupts();
      gZfbTicks  = (long)(newZ * Z_FB_TICKS_PER_MM);
      gCfbTicks  = (long)(newC * C_FB_TICKS_PER_DEG);
      gBendTicks = (long)(((long)newB * 360L) / 360L);  // Direct: 1 degree = 1 tick
        gBendTicksLast = gBendTicks;
        interrupts();
      saveCalibration();
      Serial.println("OK CAL");
    } else {
      Serial.println("ERR CAL format: CAL z,c,bend");
    }
    return;
  }
  if (line.startsWith("SON ")) {
    int on = line.substring(4).toInt();
    cmdSon(on ? 1 : 0);
    return;
  }
  if (line.startsWith("TGT ")) {
    gBendTargetDeg = line.substring(4).toInt();
    gBendDone = false;
    Serial.println("OK TGT");
    return;
  }
  if (line == "CMD 4") {
    if (gBendCycle == 1) {
      gBendingState = 1;
    } else {
      gBendingState = 2;
    }
    gBendDone = false;
    Serial.println("OK BEND START");
    return;
  }
  if (line.startsWith("ARCFEED ")) {
    int comma = line.indexOf(',');
    if (comma > 0) {
      gArcRadius = line.substring(8, comma).toFloat();
      gArcTargetAngle = line.substring(comma+1).toFloat();
      gArcStartBendTicks = gBendTicks;
        gArcStartZPulse = gZCurrentPulse;
        gArcActive = true;
      gZst = 2; // Use step generator
      Serial.println("OK ARCFEED");
    }
    return;
  }
  if (line.startsWith("ARCSTOP")) {
    noInterrupts();
    gArcActive = false;
    // Freeze the arc target where it ended so the rate limiter doesn't
    // try to chase a stale reference until the next MZ arrives.
    gZTargetPulse = (long)gZVirtualPulse;
    interrupts();
    Serial.println("OK ARCSTOP");
    return;
  }
  if (line.startsWith("MZ ")) {
    int comma = line.indexOf(',');
    String sz = (comma > 0) ? line.substring(3, comma) : line.substring(3);
    double targetZ = sz.toFloat();
    double speedZ = 20.0;
    if (comma > 0) speedZ = line.substring(comma+1).toFloat();
    if (speedZ <= 0.0) speedZ = 20.0;

    noInterrupts();
    // Always disarm any leftover arc-sync before starting a linear move
    gArcActive = false;
    gZTargetPulse = (long)(targetZ * Z_CMD_TICKS_PER_MM);
    // Sync step generator state from the encoder so we move from the
    // physical position, not from a stale virtual one.
    // Convert FB-scale encoder ticks to CMD-scale for step generator.
    gZCurrentPulse = (long)(gZfbTicks * CMD_PER_FB);
    gZVirtualPulse = gZCurrentPulse;
    gZSpeedStep = (speedZ * Z_CMD_TICKS_PER_MM) / STEP_ISR_HZ;
    // Trapezoidal acceleration profile
    gZMaxSpeed = gZSpeedStep;
    gZMinSpeed = MIN_Z_SPEED * Z_CMD_TICKS_PER_MM / STEP_ISR_HZ;
    gZCurSpeed = gZMinSpeed;  // start from minimum speed
    gZAccelRate = DEFAULT_Z_ACCEL * Z_CMD_TICKS_PER_MM / (STEP_ISR_HZ * STEP_ISR_HZ);
    gZst = 2;
    interrupts();
    gZmm = targetZ;  // update display target
    Serial.println("OK MZ");
    return;
  }
  if (line.startsWith("MC ")) {
    int comma = line.indexOf(',');
    String sc = (comma > 0) ? line.substring(3, comma) : line.substring(3);
    double targetC = sc.toFloat();
    double speedC = 20.0;
    if (comma > 0) speedC = line.substring(comma+1).toFloat();
    if (speedC <= 0.0) speedC = 20.0;

    noInterrupts();
    gCTargetPulse = (long)(targetC * C_CMD_TICKS_PER_DEG);
    // Convert FB-scale encoder ticks to CMD-scale for step generator.
    gCCurrentPulse = (long)(gCfbTicks * CMD_PER_FB);
    gCVirtualPulse = gCCurrentPulse;
    gCSpeedStep = (speedC * C_CMD_TICKS_PER_DEG) / STEP_ISR_HZ;
    // Trapezoidal acceleration profile
    gCMaxSpeed = gCSpeedStep;
    gCMinSpeed = MIN_C_SPEED * C_CMD_TICKS_PER_DEG / STEP_ISR_HZ;
    gCCurSpeed = gCMinSpeed;  // start from minimum speed
    gCAccelRate = DEFAULT_C_ACCEL * C_CMD_TICKS_PER_DEG / (STEP_ISR_HZ * STEP_ISR_HZ);
    gCst = 2;
    interrupts();
    gCdeg = targetC;  // update display target
    Serial.println("OK MC");
    return;
  }

  if (line.startsWith("STEP ")) {
    int comma = line.indexOf(',');
    if (comma > 0) {
      gStepCurrent = line.substring(5, comma).toInt();
      gStepTotal = line.substring(comma + 1).toInt();
    }
    Serial.println("OK STEP");
    return;
  }

  // ──── VFD Commands ────
  if (line.startsWith("VFDFREQ ")) {
    int spaceIdx = line.indexOf(' ', 8);
    if (spaceIdx > 0) {
      float freq = line.substring(8, spaceIdx).toFloat();
      int dir = line.substring(spaceIdx + 1).toInt();
      cmdVfdFrequency(freq, dir);
    }
    return;
  }

  if (line.startsWith("VFDSIM ")) {
    int sim = line.substring(7).toInt();
    gVfdSimulationMode = (sim ? true : false);
    Serial.println(gVfdSimulationMode ? "OK VFDSIM ON" : "OK VFDSIM OFF");
    return;
  }

  if (line == "VFDSTOP") {
    cmdVfdStop();
    return;
  }

  if (line == "VFDSTATUS") {
    Serial.print("VFD Status: freq=");
    Serial.print(gVfdCurrentFreqCode / 100.0);
    Serial.print("Hz, tgt=");
    Serial.print(gVfdTargetFreqCode / 100.0);
    Serial.print("Hz, cur=");
    Serial.print(gVfdCurrentCurrent / 10.0);
    Serial.print("A, ctrl=0x");
    Serial.print(gVfdControlWord, HEX);
    Serial.print(", sim=");
    Serial.println(gVfdSimulationMode ? "ON" : "OFF");
    return;
  }

  // ──── End VFD Commands ────

}

void setup() {
  Wire.begin(); // Join I2C bus (SDA=18, SCL=19)
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Trubogib ready");

  Serial.begin(115200);
  gResetCause = SRC_SRSR;
  Serial1.begin(VFD_BAUD, SERIAL_8N2);

  // ──── VFD Modbus Initialization ────
  gVfdModbus = new ModbusRTU(Serial1, RS485_DE);
  Serial.println("[VFD] Modbus RTU initialized on Serial1");
  gVfdSimulationMode = false;  // Set to true for testing without hardware

  if (CrashReport) {
    Serial.println("[CRASH] Previous crash report:");
    Serial.print(CrashReport);
    Serial.println();
  }

  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);

  pinMode(PIN_STEP_Z, OUTPUT);
  pinMode(PIN_DIR_Z, OUTPUT);
  pinMode(PIN_STEP_C, OUTPUT);
  pinMode(PIN_DIR_C, OUTPUT);

  pinMode(PIN_SON_Z, OUTPUT);
  pinMode(PIN_SON_C, OUTPUT);
  pinMode(PIN_CLAMP, OUTPUT);
  pinMode(PIN_UNCLAMP, OUTPUT);
  
  gServoOn = false;
  digitalWrite(PIN_SON_Z, LOW);
  digitalWrite(PIN_SON_C, LOW);
  digitalWrite(PIN_CLAMP, HIGH);     // Initial state: both HIGH = disabled (ACTIVE LOW!)
  digitalWrite(PIN_UNCLAMP, HIGH);   // Initial state: both HIGH = disabled (ACTIVE LOW!)

  // IMPORTANT: open-collector outputs from PC817 require pull-up
  pinMode(PIN_ENC_BEND_A, INPUT_PULLUP);
  pinMode(PIN_ENC_BEND_B, INPUT_PULLUP);
  // A and B channel interrupts for proper quadrature decode (both transitions needed)
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_BEND_A), isrBendEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_BEND_B), isrBendEncoder, CHANGE);

  // Servo feedback encoders via MAX490 (differential → single-ended, 5 V TTL)
  // Polled from main loop — ISR-based decoding caused USB instability previously.
  pinMode(PIN_ENC_Z_A, INPUT_PULLUP);
  pinMode(PIN_ENC_Z_B, INPUT_PULLUP);
  pinMode(PIN_ENC_C_A, INPUT_PULLUP);
  pinMode(PIN_ENC_C_B, INPUT_PULLUP);

  pinMode(PIN_PEDAL_FWD, INPUT_PULLUP);
  pinMode(PIN_PEDAL_REV, INPUT_PULLUP);
  pinMode(PIN_MODE_SW, INPUT_PULLUP);
  pinMode(PIN_LIM_Z_MIN, INPUT_PULLUP);
  pinMode(PIN_LIM_Z_MAX, INPUT_PULLUP);

  // Restore last calibration from EEPROM (survives reboot)
  if (loadCalibration()) {
    Serial.println("=== Calibration restored from EEPROM ===");
  } else {
    Serial.println("=== No calibration in EEPROM, starting from 0 ===");
  }

  // Step ISR period = 1e6 / STEP_ISR_HZ microseconds.
  // 5.0 μs → 200 kHz ISR → 100 kHz max STEP pulse rate → ~275 mm/s max feed.
  stepTimer.begin(stepIsr, 1e6 / STEP_ISR_HZ);
  gStepTimerRunning = true;


  Serial.println("=== Teensy TubeBender v4.21 ===");
}

// ═════════════════════════════════════════════════════════════════════
// LCD Display Update Function
// ═════════════════════════════════════════════════════════════════════
void updateLCDDisplay() {
    // ─────────────────────────────────────────────────────────────
    // Заполнить структуру DisplayData из глобальных переменных
    // ─────────────────────────────────────────────────────────────
    
    // Позиции осей
    lcdData.currentZ_mm = gZmm;
    lcdData.targetZ_mm = (double)gZTargetPulse / Z_CMD_TICKS_PER_MM;
    
    lcdData.currentC_deg = gCdeg;
    lcdData.targetC_deg = (double)gCTargetPulse / C_CMD_TICKS_PER_DEG;
    
    // Гибка
    lcdData.currentBend_deg = bendDegFromTicks(gBendTicks);
    lcdData.targetBend_deg = gBendTargetDeg;
    
    // Программа
    lcdData.currentStep = gStepCurrent;
    lcdData.totalSteps = gStepTotal;
    
    // Диагностика
    lcdData.loopHz = gDiagLoopHz;
    lcdData.stepHz = gDiagStepHz;
    
    // Статус подключения и оборудования
    lcdData.isConnected = (millis() - gLastExePing < 5000);
    lcdData.servoOn = gServoOn;
    
    // Ограничители
    lcdData.limitZMinActive = limZMin();
    lcdData.limitZMaxActive = limZMax();
    
    // ─────────────────────────────────────────────────────────────
    // LOGIC: Определить текущее состояние машины
    // ─────────────────────────────────────────────────────────────
    
    // 1. ПЕРВЫЙ ПРИОРИТЕТ: Ошибки (limit switches)
    if (lcdData.limitZMinActive) {
        currentLCDState = LCDScenarios::STATE_ERROR;
        lcdData.errorCode = 0x0001;  // Limit Z Min
    }
    else if (lcdData.limitZMaxActive) {
        currentLCDState = LCDScenarios::STATE_ERROR;
        lcdData.errorCode = 0x0002;  // Limit Z Max
    }
    
    // 2. ВТОРОЙ ПРИОРИТЕТ: Подключение
    else if (!lcdData.isConnected) {
        currentLCDState = LCDScenarios::STATE_CONNECTION;
    }
    
    // 3. ТРЕТИЙ ПРИОРИТЕТ: Текущая фаза движения
    else if (!gBendDone && gBendTargetDeg > 0) {
        // ГИБКА
        currentLCDState = LCDScenarios::STATE_BENDING;
    }
    
    else if (gZst == 2 || (gZCurrentPulse != (long)gZTargetPulse)) {
        // ПОДАЧА Z
        currentLCDState = LCDScenarios::STATE_FEED;
    }
    
    else if (gCst == 2 || (gCCurrentPulse != (long)gCTargetPulse)) {
        // РОТАЦИЯ C
        currentLCDState = LCDScenarios::STATE_ROTATION;
    }
    
    else if (gArcActive) {
        // ARC FEED (синхронная подача во время гибки)
        currentLCDState = LCDScenarios::STATE_FEED;
    }
    
    // 4. ЧЕТВЁРТЫЙ ПРИОРИТЕТ: Idle
    else {
        currentLCDState = LCDScenarios::STATE_IDLE;
    }
    
    // ─────────────────────────────────────────────────────────────
    // Обновить дисплей
    // ─────────────────────────────────────────────────────────────
    LCDScenarios::update(lcd, lcdData, currentLCDState);
}

// ═════════════════════════════════════════════════════════════════════
// Main Loop
// ═════════════════════════════════════════════════════════════════════
void loop() {
  pollFeedbackEncoders();

  // Keep runtime position fields synchronized to encoder feedback.
  // This avoids reporting stale command targets in status JSON.
  {
    long zFb, cFb;
    noInterrupts();
    zFb = gZfbTicks;
    cFb = gCfbTicks;
    interrupts();
    gZmm  = (double)zFb / Z_FB_TICKS_PER_MM;
    gCdeg = (double)cFb / C_FB_TICKS_PER_DEG;
  }

  // --- Mock PLC Bending Logic ---
  noInterrupts();
  long tempTicks = gBendTicks;
  interrupts();
  int currentBend = (int)round(tempTicks / 4.0);  // Quadrature X4: 4 ticks = 1 degree
  
  if (gBendingState == 1) {
    if (abs(currentBend) >= gBendTargetDeg) {
      gBendCycle = 2;
      gBendDone = true;
      gBendingState = 0;
    }
  } else if (gBendingState == 2) {
    if (abs(currentBend) <= 1) {
      gBendCycle = 1;
      gBendDone = true;
      gBendingState = 0;
    }
  }

  // ──── VFD Status Poll ────
  // Back-off poll: every 100 ms while VFD talks; if it falls silent, the
  // interval ramps to 2 s so the blocking Modbus timeout (~100 ms per failed
  // read) stops starving pollFeedbackEncoders() every main-loop pass.
  static uint32_t vfdPollLastMs = 0;
  static uint32_t vfdPollIntervalMs = 100;
  static uint32_t vfdPrevMbErr = 0;
  static uint32_t vfdPrevMbRx  = 0;
  if (millis() - vfdPollLastMs >= vfdPollIntervalMs) {
    vfdPollLastMs = millis();
    pollVfdStatus();
    if (gMbRx != vfdPrevMbRx) {
      vfdPollIntervalMs = 100;               // healthy link → fast poll
    } else if (gMbErr != vfdPrevMbErr) {
      vfdPollIntervalMs = 2000;              // timeout → back off to 2 s
    }
    vfdPrevMbErr = gMbErr;
    vfdPrevMbRx  = gMbRx;
  }
  // ──── End VFD Poll ────

  // LCD refresh throttled to 500 ms: I2C char LCD takes tens of ms per
  // frame — any faster starves pollFeedbackEncoders() of main-loop time.
  static unsigned long lastLcdUpdate = 0;
  if (millis() - lastLcdUpdate > 500) {
    lastLcdUpdate = millis();
    updateLCDDisplay();  // ← LCD обновляется через LCD_SCENARIOS
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    gRxCharsAcc++;
    if (c == '\n' || c == '\r') {
      if (gRxLine.length() > 0) {
        handleCommand(gRxLine);
        gRxLine = "";
      }
    } else {
      gRxLine += c;
      if (gRxLine.length() > 120) {
        gRxLine = "";
        gRxOverflowAcc++;
      }
    }
  }

  // Update diagnostic rates once per second
  static uint32_t diagLastMs = 0;
  static uint32_t diagPrevStep = 0;
  static uint32_t diagPrevBend = 0;
  static uint32_t diagPrevZ = 0;
  static uint32_t diagPrevC = 0;
  static uint32_t diagPrevRx = 0;
  static uint32_t diagPrevOv = 0;
  static uint32_t diagLoopCount = 0;
  static uint32_t diagLastLoopUs = 0;
  static uint32_t diagMaxLoopUs = 0;

  uint32_t nowUs = micros();
  if (diagLastLoopUs != 0) {
    uint32_t dtUs = nowUs - diagLastLoopUs;
    if (dtUs > diagMaxLoopUs) diagMaxLoopUs = dtUs;
  }
  diagLastLoopUs = nowUs;
  diagLoopCount++;

  uint32_t nowMs = millis();
  if (nowMs - diagLastMs >= 1000) {
    noInterrupts();
    uint32_t stepCnt = gStepIsrCount;
    uint32_t bendCnt = gBendIsrCount;
    uint32_t zCnt = gZfbIsrCount;
    uint32_t cCnt = gCfbIsrCount;
    interrupts();

    gDiagStepHz = stepCnt - diagPrevStep;
    gDiagBendHz = bendCnt - diagPrevBend;
    gDiagZfbHz = zCnt - diagPrevZ;
    gDiagCfbHz = cCnt - diagPrevC;
    gDiagLoopHz = diagLoopCount;
    gDiagLoopMaxUs = diagMaxLoopUs;
    gDiagRxCharsPerSec = gRxCharsAcc - diagPrevRx;
    gDiagRxOverflowPerSec = gRxOverflowAcc - diagPrevOv;

    diagPrevStep = stepCnt;
    diagPrevBend = bendCnt;
    diagPrevZ = zCnt;
    diagPrevC = cCnt;
    diagPrevRx = gRxCharsAcc;
    diagPrevOv = gRxOverflowAcc;
    diagLoopCount = 0;
    diagMaxLoopUs = 0;
    diagLastMs = nowMs;
  }

  // Runtime local manual control fallback
  bool fwd = pedalFwdPressed();
  bool rev = pedalRevPressed();
  bool autoMode = modeAuto();

  if (!autoMode) {
    if (fwd && !rev) {
      gVfdRunning = true;
      gVfdForward = true;
      gVfdFreqHz100 = 3000;
    } else if (rev && !fwd) {
      gVfdRunning = true;
      gVfdForward = false;
      gVfdFreqHz100 = 3000;
    } else {
      gVfdRunning = false;
      gVfdFreqHz100 = 0;
    }
  }

  // Mark bend done transition whenever encoder changed enough
  long nowTicks = gBendTicks;
  if (nowTicks != gBendTicksLast) {
    gBendDone = false;
    gBendTicksLast = nowTicks;
  }

  // IMPORTANT: disable unsolicited status transmission.
  // Host must be the single source of polling via '?' to keep USB traffic
  // deterministic during SON transitions.
}
