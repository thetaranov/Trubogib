#pragma once
// ─────────────────────────────────────────────────────────────────────
// machine.h — Machine runtime: Teensy USB-Serial protocol + bending sequence
// ─────────────────────────────────────────────────────────────────────
#include "serial_port.h"
#include "bending_program.h"
#include <string>
#include <chrono>
#include <functional>

// Teensy status parsed from JSON response to '?' command
struct TeensyStatus {
    // Firmware
    std::string version;

    // VFD Modbus link (Teensy -> Delta CP2000)
    unsigned long mbRx = 0, mbTx = 0, mbErr = 0, mbLink = 99999;
    bool vfdOnline = false;
    double vfdFreqHz = 0.0;
    double vfdCurrentA = 0.0;

    // Axis Z (feed)
    double zMm = 0;
    int    zSt = 0;   // 0=idle,1=moving,2=done,3=error,4=homing
    bool   zSon = false;

    // Axis C (rotation)
    double cDeg = 0;
    int    cSt = 0;
    bool   cSon = false;

    // Bending state (from Teensy)
    int bendCur = 0;   // gCurrentAngle
    int bendTgt = 0;   // gTargetAngle
    int bendCyc = 0;   // gCycleNum
    int bendMode = 0;  // 0=manual, 1=auto
    int bendDone = 0;  // 0/1
    bool pedalFwd = false;
    bool pedalRev = false;
    bool modeAuto = false;

    // Watchdog
    bool wdTrip = false;

    // Servo alarm / diagnostics
    bool almZ = false;   // Servo Z alarm
    bool almC = false;   // Servo C alarm

    // Limit switches (linear carriage)
    bool limZMin = false;
    bool limZMax = false;

    // External command registers (diagnostics)
    int extCmd8 = 0;
    int extCmd9 = 0;
    int extTgt10 = 0;
    int extPulse = 0;
    int extFast = 0;
    int extDo7 = 0;
    int extDo8 = 0;

    // Pneumatic relay state (from Teensy pins 35/36 feedback)
    bool clampClosed = false;  // PIN_CLAMP (35): LOW = closed/pressed, HIGH = released
    bool unclampOpen = false;  // PIN_UNCLAMP (36): inverse of clamp (when clamp closed, unclamp open)

    // Servo encoder feedback via CN1 LA/LB -> MAX490 receivers
    bool encValid = false;
    long zFbCount = 0;
    long cFbCount = 0;
    double zFbMm = 0;
    double cFbDeg = 0;

    // Boot/diagnostics from Teensy firmware
    unsigned long bootRst = 0;
    unsigned long bootUpMs = 0;
    unsigned long diagLoopHz = 0;
    unsigned long diagLoopMaxUs = 0;
    unsigned long diagStepHz = 0;
    unsigned long diagBendIsrHz = 0;
    unsigned long diagZIsrHz = 0;
    unsigned long diagCIsrHz = 0;
    unsigned long diagRxHz = 0;
    unsigned long diagRxOvHz = 0;
    bool diagSonZOut = false;
    bool diagSonCOut = false;
    bool diagPedalIn = false;
    bool diagModeIn = false;

    bool valid = false;
    std::string raw;
};

// ─────────────────────────────────────────────────────────────────────
// MachineState — единый источник истины о состоянии станка в текущем кадре.
// Заполняется методом Machine::getSnapshot() ОДИН РАЗ за кадр; SceneAnimator
// и все потребители обязаны работать ТОЛЬКО с этой структурой, чтобы
// позиция каретки, длина трубы, угол гиба и др. никогда не расходились
// между собой (даже если реальная телеметрия изменилась за кадр).
// ─────────────────────────────────────────────────────────────────────
struct MachineState {
    bool isSimulation = false;
    bool isConnected  = false;
    bool hasValidData = false;

    double zAbs = 0.0;   // mm, absolute Z (feed)
    double cAbs = 0.0;   // deg, absolute C (rotation)
    double bAbs = 0.0;   // deg, current bend angle

    bool clampClosed = false;

    int    phaseId        = 0;
    int    stepIdx        = 0;
    int    totalSteps     = 0;
    bool   programRunning = false;

    double baseFeedAcc = 0.0;
    double baseRotAcc  = 0.0;
    double feedAccum   = 0.0;
    double rotAccum    = 0.0;

    // Teensy firmware encodes axis motion state as: 0=idle, 2=moving, 3=error, 4=homing.
    // (It never reports 1 for moving — see gZst/gCst in stepIsr / MZ/MC handlers.)
    bool zMoving  = false;
    bool cMoving  = false;
    bool bendDone = false;
};

// Machine execution phases for a single bending step
enum class MachPhase {
    Idle = 0,
    ServoOn,         // SON 1
    HomingZ,         // HZ
    HomingC,         // HC
    UnclampWait1,    // ensure chuck is open before feed
    Feed,            // MZ L_feed
    Rotation,        // MC Rotation
    LaserCutBefore,  // Start sync laser cut before bend
    LaserCutBeforeWait, // Wait for sync move
    ClampWait,
    SimulatedBendWait,
    BendStart,       // TGT angle + CMD 4
    BendWait,        // wait for bend done
    UnclampWait2,    // delay for clamp to open
    LaserCutAfter,   // Start sync laser cut after bend
    LaserCutAfterWait, // Wait for sync move
    Clearance,       // MZ +clampLength (push tube for bend roller clearance)
    ReturnBend,      // CMD 4 (cycle 2 = return to 0)
    ReturnBendWait,  // wait for return done
    StepDone,        // step complete
    ProgramDone,     // all steps complete
    Error
};

// Watchdog state для контроля движения в режиме Hardware
struct WatchdogState {
    bool active = false;
    char axis = 0;  // 'Z' или 'C'
    std::chrono::steady_clock::time_point cmdTime;
    double cmdStartPos = 0.0;
    double cmdTarget = 0.0;
    static constexpr int TIMEOUT_MS = 500;
};

class Machine {
public:
    Machine() = default;

    // Connection
    bool connect(const std::string& port);
    void disconnect();
    bool isConnected() const { return serial_.isOpen(); }
    std::string portName() const { return portName_; }

    // Simulation mode (no hardware — fake telemetry from internal model)
    bool isSimulated() const { return simulated_; }
    void startSimulation();
    void stopSimulation();

    // Poll — call every frame, processes serial responses
    void poll(float dt);

    // Request status from Teensy
    void requestStatus();

    // Direct commands
    void servoOn();
    void servoOff();
    void homeZ();
    void homeC();
    void moveZ(double mm, double speed = 100);
    void moveC(double deg, double speed = 30);
    void stop();
    void estop();
    void sendBendTarget(int angle);
    void sendBendStart();    // CMD 4 — auto-bend
    void sendBendStop();     // CMD 2
    void sendZero();         // CMD 3
    void sendZeroPos();      // ZP — zero Teensy axis positions
    void sendSync();         // SYNC — sync Teensy step pos from encoder
    void sendArcFeed(double R, double targetAngle);  // ARCFEED R,angle
    void sendArcStop();      // ARCSTOP — stop arc feed
    void sendBendZero();     // BZ — zero ONLY bend encoder ticks (persistent in Teensy EEPROM)
    void sendClamp();        // CLAMP — close clamp solenoid
    void sendUnclamp();      // UNCLAMP — open clamp solenoid
    void sendRelayOff();     // RELAYOFF — de-energize BOTH clamp/unclamp relays (no air pressure either way)
    void sendCalibration(double zMm, double cDeg, int bendDeg); // CAL z,c,bend — set all 3 axes (persistent)
    void sendStep(int cur, int total);  // STEP cur,total — update LCD step display

    // Program execution
    void startProgram(const BendingProgram& prog, double clampLength, const std::string& programName = "");
    void stopProgram();
    bool isProgramRunning() const { return phase_ != MachPhase::Idle && phase_ != MachPhase::ProgramDone && phase_ != MachPhase::Error; }

    // State
    // ⚠️ ЕДИНЫЙ ИСТОЧНИК ИСТИНЫ: Вызывается один раз за кадр. Возвращает
    // консистентный срез состояния станка (sim/hw прозрачно для потребителя).
    // SceneAnimator ОБЯЗАН получать данные только через getSnapshot().
    MachineState getSnapshot() const;

    const TeensyStatus& status() const { return status_; }
    MachPhase phase() const { return phase_; }
    int currentStep() const { return curStep_; }
    int totalSteps() const { return totalSteps_; }
    double feedAccum() const { return feedAccum_; }
    double rotAccum() const { return rotAccum_; }
    int simBendSubPhase() const { return simBendSubPhase_; }
    
    // ⚠️ КРИТИЧЕСКИЙ ФИХ: Возвращать ФИКСИРОВАННЫЕ базы для текущего шага
    // Эти значения не меняются до перехода в НОВЫЙ шаг
    double baseFeedAccForCurrentStep() const { return baseFeedAccCurrentStep_; }
    double baseRotAccForCurrentStep() const { return baseRotAccCurrentStep_; }
    
    // Функция синхронизации - вызывается machine.cpp при входе в новый шаг
    void updateBaseAccumulators();
    std::string runningProgramName() const { return programName_; }
    const BendingProgram& runningProgram() const { return program_; }
    std::string phaseText() const;
    std::string lastError() const { return lastError_; }

    // Log
    const std::vector<std::string>& log() const { return log_; }
    void clearLog() { log_.clear(); }

private:
    bool tryHome(char axis);
    void processLine(const std::string& line);
    TeensyStatus parseStatus(const std::string& json);
    void advanceProgram();
    void addLog(const std::string& msg);

    SerialPort serial_;
    std::string portName_;
    TeensyStatus status_;
    std::string lastError_;
    std::vector<std::string> log_;

    // Status polling
    std::chrono::steady_clock::time_point lastStatusReq_;
    static constexpr int STATUS_INTERVAL_MS = 10;  // ⚠️ КРИТИЧНО: 10ms для высокочастотного опроса координат
    std::chrono::steady_clock::time_point statusPollSuspendUntil_{};
    static constexpr int STATUS_SUSPEND_AFTER_SON_MS = 1200;

    // Program execution state
    MachPhase phase_ = MachPhase::Idle;
    BendingProgram program_;
    std::string programName_;
    double clampLength_ = 60;
    int curStep_ = 0;
    int totalSteps_ = 0;
    double feedAccum_ = 0;  // accumulated Z feed position
    double rotAccum_ = 0;   // accumulated C rotation position
    double bendStartFeedAccum_ = 0.0; // Zabs snapshot at BendStart for arc accumulation
    
    // ⚠️ КРИТИЧЕСКИЙ ФИХ: Фиксированные базы для текущего шага (не меняются до перехода в НОВЫЙ шаг)
    double baseFeedAccCurrentStep_ = 0.0;  // Сумма Z до начала текущего шага
    double baseRotAccCurrentStep_ = 0.0;   // Сумма C до начала текущего шага
    int lastCachedStepIdx_ = -1;  // отслеживаем когда обновлять кэш

    // Laser cut state
    int laserCutStepNum_ = 0;
    int laserCutTotalSteps_ = 36;
    double laserCutStartZ_ = 0;
    double laserCutStartC_ = 0;


    // Wait state
    bool waitingForAxis_ = false;
    bool waitingForBend_ = false;
    std::chrono::steady_clock::time_point delayStart_;
    int delayMs_ = 0;
    int  prevBendDone_ = 0;
    int  prevBendCyc_ = 0;
    bool homeMoveIssued_ = false;

    // Simulated bend (ignoreBendEncoder_) sub-phase: 0=ramp up, 1=ramp down
    int  simBendSubPhase_ = 0;

    // SON stabilization delay
    std::chrono::steady_clock::time_point sonWaitStart_;
    bool sonWaitStarted_ = false;
    static constexpr int SON_RELAY_DELAY_MS = 500;

    // Timeout waiting for SON confirmation from Teensy status
    std::chrono::steady_clock::time_point servoOnStart_;
    bool servoOnStarted_ = false;
    static constexpr int SERVO_ON_TIMEOUT_MS = 10000;

    // Timeout waiting for axis motion start after MZ/MC command
    std::chrono::steady_clock::time_point axisCmdStart_;
    char axisCmd_ = 0;               // 'Z' or 'C'      
    bool phaseStarted_ = false;    
    double axisCmdStartPos_ = 0.0;   // position at command time
    double axisCmdTarget_ = 0.0;     // commanded target
    bool axisCmdPending_ = false;

    // Stall watchdog — catches "motion started but froze mid-move".
    // Complements axisCmdPending_ (which only catches "motion never started").
    // Example failure: 3.json step 4, cDeg froze at 11.34° while cSt stayed 2
    // for ~98s. We now trip Error if the reported position doesn't advance for
    // STALL_TIMEOUT_MS while status_.?St indicates "moving".
    std::chrono::steady_clock::time_point stallLastProgressAt_;
    double stallLastPos_ = 0.0;
    bool   stallArmed_   = false;
    static constexpr int STALL_TIMEOUT_MS = 2000;
    static constexpr double STALL_Z_EPS   = 0.05;  // mm
    static constexpr double STALL_C_EPS   = 0.1;   // deg
    
    // Hardware speed limits driven by Teensy step generator.
    // Step ISR = 200 kHz, 2 ISR ticks per STEP pulse → STEP cap = 100 kHz.
    //   Z: Z_CMD_TICKS_PER_MM ≈ 363.77  → 100e3 / 363.77 ≈ 275 mm/s cap.
    //   C: C_CMD_TICKS_PER_DEG ≈ 1666.67 → 100e3 / 1666.67 ≈ 60 deg/s cap.
    // Leave ~10 % margin under the cap so acceleration doesn't saturate.
    static constexpr double HW_MAX_FEED_SPEED = 250.0;  // mm/s
    static constexpr double HW_MAX_ROT_SPEED = 55.0;    // deg/s

public:
    bool pedalBypass_ = false;
    bool ignoreBendEncoder_ = false;
    double feedSpeed_ = 300.0;
    double rotSpeed_ = 120.0;
    double bendSpeedDegSec_ = 0.0;
    double simBendSpeed_ = 60.0;  // deg/s for simulation visual speed (ТЗ v2.5)
    int clampDelayMs_ = 300;
    int unclampDelayMs_ = 300;
    bool pedalInterlockPaused_ = false;

private:
    void setPedalBypass(bool b) { pedalBypass_ = b; }
    static constexpr int AXIS_START_TIMEOUT_MS = 5000;

    // Diagnostics: reduce ambiguity in logs when machine "stops in the middle"
    MachPhase lastLoggedPhase_ = MachPhase::Idle;
    int lastLoggedStep_ = -1;

    bool isServoOnConfirmed() const {
        return (status_.zSon && status_.cSon);
    }

    // ── PC-side interpolation for smooth hardware motion (hw-режим) ──
    // Encoder feedback ~ 10 Hz, but user sees ~60 FPS. Interpolate between
    // feedback updates to smooth out the discrete jumps.
    // Marked mutable since getSnapshot() is const but updates these during interpolation.
    mutable double lastZAbs_ = 0.0;
    mutable double lastCAbs_ = 0.0;
    mutable std::chrono::steady_clock::time_point lastZTime_;
    mutable std::chrono::steady_clock::time_point lastCTime_;
    mutable double zVelMmPerSec_ = 0.0;  // Computed velocity for Z axis (mm/s)
    mutable double cVelDegPerSec_ = 0.0; // Computed velocity for C axis (deg/s)

    // ── Simulation backend ──────────────────────────────────────────
public:
    bool simulated_ = false;
    double simZPos_ = 0.0;        // simulated Z position (mm)
    double simCPos_ = 0.0;        // simulated C position (deg)
    double simBendPos_ = 0.0;     // simulated bend angle (deg)
    double simZTarget_ = 0.0;
    double simCTarget_ = 0.0;
    double simZSpeed_ = 100.0;    // mm/s
    double simCSpeed_ = 30.0;     // deg/s
    int    simBendTargetDeg_ = 0;
    double simBendStartZ_ = 0.0;  // Z at BendStart — used to lock Z to B progress in interpolZAndB
    int    simBendCycle_ = 1;     // 1=fwd, 2=return
    bool   simBendRunning_ = false;
    std::chrono::steady_clock::time_point simLastTime_;
    void updateSimulation(float dt);
    void generateSimStatus();
    
    // Phase-dependent interpolation methods
    void interpolZOnly(float dt);      // Only Z moves forward
    void interpolCOnly(float dt);      // Only C rotates
    void interpolBOnly(float dt);      // Only B bends forward
    void interpolZAndB(float dt);      // Z and B move SIMULTANEOUSLY (critical for bend)
    void interpolBReturn(float dt);    // B returns to 0
    void interpolZReturn(float dt);    // Z returns to 0

    // ── Global mode flag ────────────────────────────────────────────
    enum class MachMode { Simulation = 0, Hardware = 1 };
    MachMode machineMode() const { return simulated_ ? MachMode::Simulation : MachMode::Hardware; }

    // ── Watchdog для контроля движения (только в режиме Hardware) ──
    WatchdogState watchdog_;

    // ── Реле управление ────────────────────────────────────────────
    // Флаги активных реле (на основе TeensyStatus, полученного от оборудования)
    struct RelayState {
        bool clampActive = false;     // PIN 35: прижим активен
        bool unclampActive = false;   // PIN 36: разжим активен
        std::chrono::steady_clock::time_point clampActivatedAt;
        std::chrono::steady_clock::time_point unclampActivatedAt;
        static constexpr int PULSE_DURATION_MS = 1500;  // Импульс 1.5 секунды
    };
    RelayState relayState_;

    // Запретить одновременное включение двух реле
    bool canActivateRelay(int pinNum) const;
    void updateRelayStatus();  // Обновить флаги на основе Teensy feedback

private:
    // Отслеживание импульса реле для контроля длительности
    std::chrono::steady_clock::time_point clampPulseStart_;
    std::chrono::steady_clock::time_point unclampPulseStart_;
    bool clampPulseActive_ = false;
    bool unclampPulseActive_ = false;
};
