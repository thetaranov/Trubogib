// ─────────────────────────────────────────────────────────────────────
// machine.cpp — Machine runtime implementation
// ─────────────────────────────────────────────────────────────────────
#include "machine.h"
#include "logger.h"
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <thread>

namespace {
constexpr double kHomeZReturnSpeedMm = 100.0;
constexpr double kHomeCReturnSpeedDeg = 30.0;

inline double zAxisPos(const TeensyStatus& st) {
    return st.encValid ? st.zFbMm : st.zMm;
}

inline double cAxisPos(const TeensyStatus& st) {
    return st.encValid ? st.cFbDeg : st.cDeg;
}
}

// ─────────────────────────────────────────────────────────────────────
// getSnapshot — ЕДИНЫЙ ИСТОЧНИК ИСТИНЫ состояния станка.
// Вызывается ОДИН раз за кадр. Возвращает консистентный срез:
// позиции осей, фаза, аккумуляторы. SceneAnimator и app.cpp обязаны
// работать только с этим снимком — тогда каретка, длина трубы и
// угол гиба никогда не расходятся между собой.
// Источник:
//   simulated_ == true → sim поля (simZPos_, simCPos_, simBendPos_)
//   иначе              → TeensyStatus с PC-side интерполяцией для гладкости
// ─────────────────────────────────────────────────────────────────────
MachineState Machine::getSnapshot() const {
    MachineState s;
    s.isSimulation   = simulated_;
    s.isConnected    = simulated_ ? true : serial_.isOpen();
    s.phaseId        = (int)phase_;
    s.stepIdx        = curStep_;
    s.totalSteps     = totalSteps_;
    s.programRunning = isProgramRunning();
    s.baseFeedAcc    = baseFeedAccCurrentStep_;
    s.baseRotAcc     = baseRotAccCurrentStep_;
    s.feedAccum      = feedAccum_;
    s.rotAccum       = rotAccum_;
    s.bendDone       = (status_.bendDone != 0);
    s.clampClosed    = status_.clampClosed;

    if (simulated_) {
        s.zAbs         = simZPos_;
        s.cAbs         = simCPos_;
        s.bAbs         = simBendPos_;
        s.zMoving      = (std::fabs(simZPos_ - simZTarget_) > 0.01);
        s.cMoving      = (std::fabs(simCPos_ - simCTarget_) > 0.01);
        s.hasValidData = true;
    } else {
        // Hardware mode: use encoder feedback if available, with PC-side interpolation
        // to smooth out the ~10 Hz telemetry updates to ~60 FPS visual smoothness.
        const auto now = std::chrono::steady_clock::now();
        
        // Initialize timing on first call
        static bool first = true;
        if (first) {
            lastZTime_ = now;
            lastCTime_ = now;
            first = false;
        }
        
        // Z axis
        double zCurrent = status_.encValid ? status_.zFbMm : status_.zMm;
        double dt_z = std::chrono::duration<double>(now - lastZTime_).count();
        if (dt_z > 0.001) {  // New telemetry update from Teensy
            lastZAbs_ = zCurrent;
            lastZTime_ = now;
        }
        // Simple display: just use encoder feedback without extrapolation (prevents jitter)
        s.zAbs = zCurrent;
        
        // C axis
        double cCurrent = status_.encValid ? status_.cFbDeg : status_.cDeg;
        double dt_c = std::chrono::duration<double>(now - lastCTime_).count();
        if (dt_c > 0.001) {  // New telemetry update from Teensy
            lastCAbs_ = cCurrent;
            lastCTime_ = now;
        }
        // Simple display: just use encoder feedback without extrapolation (prevents jitter)
        s.cAbs = cCurrent;
        
        // DEBUG: Log position selection
        static int snapLogCount = 0;
        static bool lastWasSimulation = true;  // Track mode switches
        
        // Detect mode switch (SIM→HW or HW→SIM)
        if (lastWasSimulation != simulated_) {
            Logger::log(std::string("[MODE_SWITCH] ") + 
                       (lastWasSimulation ? "SIM→HW" : "HW→SIM") +
                       " detected in getSnapshot()! " +
                       "zAbs=" + std::to_string(s.zAbs) + 
                       " cAbs=" + std::to_string(s.cAbs) +
                       " encValid=" + std::to_string(status_.encValid));
            lastWasSimulation = simulated_;
        }
        
        if (++snapLogCount % 120 == 0) {  // ~2Hz if called 60x per frame
            Logger::log(std::string("[SNAPSHOT] MODE=") + (simulated_ ? "SIM" : "HW") +
                        " encValid=" + std::to_string(status_.encValid) +
                        " zAbs=" + std::to_string(s.zAbs) + "mm (src=" + (status_.encValid ? "FB" : "CALC") + ")" +
                        " cAbs=" + std::to_string(s.cAbs) + "° (src=" + (status_.encValid ? "FB" : "CALC") + ")" +
                        " zFbMm=" + std::to_string(status_.zFbMm) +
                        " zMm=" + std::to_string(status_.zMm) +
                        " feedAccum=" + std::to_string(feedAccum_));
        }
        
        // B axis: use simulated interpolation if encoder bypass is enabled
        // (because Teensy doesn't send bendCur when ignoreBendEncoder=true)
        if (ignoreBendEncoder_ && phase_ == MachPhase::SimulatedBendWait) {
            s.bAbs = simBendPos_;  // ✅ Use interpolated sim value for visualization
        } else {
            s.bAbs = (double)status_.bendCur;  // Use real encoder feedback
        }
        
        s.zMoving      = (status_.zSt == 2);
        s.cMoving      = (status_.cSt == 2);
        s.hasValidData = status_.valid;
    }
    return s;
}

// ─── Connection ───────────────────────────────────────────────────────

bool Machine::connect(const std::string& port) {
    if (!serial_.open(port, 115200)) {
        lastError_ = serial_.lastError();
        return false;
    }
    portName_ = port;
    lastError_.clear();
    addLog("Connected to " + port);
    // Request initial status after a short delay
    lastStatusReq_ = std::chrono::steady_clock::now();
    return true;
}

void Machine::disconnect() {
    if (isProgramRunning()) stopProgram();
    serial_.close();
    portName_.clear();
    status_ = {};
    addLog("Disconnected");
}

// ─── Polling ──────────────────────────────────────────────────────────

void Machine::poll(float dt) {
    // Clamp dt to reasonable values (prevent spikes)
    dt = std::clamp(dt, 0.0001f, 0.05f);
    
    // ── Simulation mode OR ignore-bend-encoder mode (hybrid) ──
    bool needSimUpdate = simulated_ || (ignoreBendEncoder_ && isProgramRunning());
    if (simulated_) {
        static int pollCount = 0;
        if (++pollCount % 50 == 0) {
            Logger::log(std::string("[POLL_SIM] Call #") + std::to_string(pollCount) +
                       " simulated_=" + std::to_string(simulated_) +
                       " isProgramRunning=" + std::to_string(isProgramRunning()) +
                       " phase=" + std::to_string((int)phase_) +
                       " simZPos=" + std::to_string(simZPos_));
        }
        updateSimulation(dt);
        if (isProgramRunning())
            advanceProgram();
        return;
    }
    
    // ── Hardware mode WITH bend encoder bypass (Path C) ──
    // When ignoreBendEncoder_=true, we still need to interpolate Z/B during SimulatedBendWait
    if (needSimUpdate && phase_ == MachPhase::SimulatedBendWait) {
        updateSimulation(dt);
        if (isProgramRunning())
            advanceProgram();
        return;
    }

    // ── Hardware mode ──
    if (!serial_.isOpen()) {
        if (isProgramRunning()) {
            std::ostringstream oss;
            oss << "USB/COM link lost during program execution";
            if (status_.valid) {
                oss << " | zSon=" << (status_.zSon ? 1 : 0)
                    << " cSon=" << (status_.cSon ? 1 : 0)
                    << " boot.rst=" << status_.bootRst
                    << " upMs=" << status_.bootUpMs
                    << " stepHz=" << status_.diagStepHz
                    << " zIsrHz=" << status_.diagZIsrHz
                    << " cIsrHz=" << status_.diagCIsrHz
                    << " loopMaxUs=" << status_.diagLoopMaxUs;
            }
            lastError_ = oss.str();
            phase_ = MachPhase::Error;
            addLog("!!! " + lastError_);
        }
        return;
    }

    serial_.poll();
    auto lines = serial_.getLines();
    for (auto& line : lines)
        processLine(line);

    // Auto-request status periodically
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastStatusReq_).count();
    
    // ⚠️ ДИАГНОСТИКА: Если пропуски опроса
    static int pollSkipCount = 0;
    if (elapsed > STATUS_INTERVAL_MS * 2) {
        pollSkipCount++;
        if (pollSkipCount % 50 == 0) {  // логируем каждый 50-й пропуск
            Logger::log(std::string("[POLL] WARNING: elapsed=") + std::to_string(elapsed) + 
                       "ms (expected ~" + std::to_string(STATUS_INTERVAL_MS) + "ms), skips=" + 
                       std::to_string(pollSkipCount));
        }
    }
    
    if (elapsed >= STATUS_INTERVAL_MS) {
        requestStatus();
        lastStatusReq_ = now;
    }

    // Обновить состояние реле (импульсы, обратная связь)
    updateRelayStatus();

    // Advance program state machine
    if (isProgramRunning())
        advanceProgram();
}

void Machine::processLine(const std::string& line) {
    if (line.empty()) return;
    addLog("< " + line);

    // JSON status response
    if (line[0] == '{') {
        // DEBUG: Log raw JSON arrival
        static int jsonArrivalCount = 0;
        if (++jsonArrivalCount % 10 == 0) {
            Logger::log(std::string("[HW_JSON_ARRIVAL] #") + std::to_string(jsonArrivalCount) +
                        " len=" + std::to_string(line.length()) +
                        " has_enc=" + (line.find("\"enc\":{") != std::string::npos ? "YES" : "NO") +
                        " snippet=" + line.substr(0, 80) + "...");
        }
        
        TeensyStatus ns = parseStatus(line);
        if (status_.valid) {
            static auto lastTime = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - lastTime).count();
            if (dt > 0.05 && ns.bendCur != status_.bendCur) {
                double instSpeed = std::abs(ns.bendCur - status_.bendCur) / dt;
                // simple low pass filter
                bendSpeedDegSec_ = (bendSpeedDegSec_ * 0.7) + (instSpeed * 0.3);
            }
            if (ns.bendCyc == 0 && phase_ != MachPhase::BendWait) {
                bendSpeedDegSec_ = 0.0;
            }
            lastTime = now;

            if (ns.bendCur != status_.bendCur) {
                addLog("DBG bend.cur: " + std::to_string(status_.bendCur) + " -> " + std::to_string(ns.bendCur)
                    + " (mbLink=" + std::to_string(ns.mbLink) + "ms)");
            }
            if (ns.zSon != status_.zSon || ns.cSon != status_.cSon) {
                addLog(std::string("DBG SON: Z=") + (ns.zSon ? "ON" : "OFF") +
                    " C=" + (ns.cSon ? "ON" : "OFF") +
                    " mode=" + std::string(ns.modeAuto ? "AUTO" : "MANUAL"));
            }
            if (ns.limZMin != status_.limZMin || ns.limZMax != status_.limZMax) {
                addLog("DBG LIMITS: ZMIN=" + std::to_string((int)ns.limZMin)
                    + " ZMAX=" + std::to_string((int)ns.limZMax));
            }
        }
        status_ = ns;
        return;
    }

    // OK/ERR responses
    if (line.substr(0, 2) == "OK" || line.substr(0, 3) == "ERR") {
        addLog(line);
    }

    // Startup banner
    if (line.find("Teensy") != std::string::npos || line.find("===") != std::string::npos) {
        addLog(line);
    }
}

// ─── Status Parsing ───────────────────────────────────────────────────

// Minimal JSON parser for Teensy status (no external dependency needed)
static double jsonDouble(const std::string& s, const char* key) {
    std::string k = std::string("\"") + key + "\":";
    auto pos = s.find(k);
    if (pos == std::string::npos) return 0;
    return atof(s.c_str() + pos + k.size());
}
static int jsonInt(const std::string& s, const char* key) {
    return (int)jsonDouble(s, key);
}
static std::string jsonStr(const std::string& s, const char* key) {
    std::string k = std::string("\"") + key + "\":\"";
    auto pos = s.find(k);
    if (pos == std::string::npos) return "";
    auto start = pos + k.size();
    auto end = s.find('"', start);
    if (end == std::string::npos) return "";
    return s.substr(start, end - start);
}

TeensyStatus Machine::parseStatus(const std::string& json) {
    TeensyStatus st;
    st.raw = json;

    st.version = jsonStr(json, "v");
    if (st.version.empty()) st.version = jsonStr(json, "version");

    // Modbus sub-object: "mb":{...}
    auto mbPos = json.find("\"mb\":{");
    if (mbPos != std::string::npos) {
        std::string mb = json.substr(mbPos);
        st.mbRx   = (unsigned long)jsonInt(mb, "rx");
        st.mbTx   = (unsigned long)jsonInt(mb, "tx");
        st.mbErr  = (unsigned long)jsonInt(mb, "err");
        st.mbLink = (unsigned long)jsonInt(mb, "link");
        st.vfdOnline = jsonInt(mb, "ok") != 0;
    }

    // Z axis: "z":{...}
    auto zPos = json.find("\"z\":{");
    if (zPos != std::string::npos) {
        std::string z = json.substr(zPos);
        st.zMm  = jsonDouble(z, "mm");
        st.zSt  = jsonInt(z, "st");
        st.zSon = jsonInt(z, "son") != 0;
    }

    // C axis: "c":{...}
    auto cPos = json.find("\"c\":{");
    if (cPos != std::string::npos) {
        std::string c = json.substr(cPos);
        st.cDeg = jsonDouble(c, "deg");
        st.cSt  = jsonInt(c, "st");
        st.cSon = jsonInt(c, "son") != 0;
    }

    // DEBUG: Log Z/C before encoder override
    static int zCDbgCount = 0;
    if (++zCDbgCount % 10 == 0) {
        Logger::log(std::string("[Z/C_PARSED] zMm=") + std::to_string(st.zMm) +
                    " cDeg=" + std::to_string(st.cDeg) +
                    " (these are from 'z'/'c' fields in JSON - may be TARGET, not current!)");
    }

    // Bend: "bend":{...}
    auto bPos = json.find("\"bend\":{");
    if (bPos != std::string::npos) {
        std::string b = json.substr(bPos);
        st.bendCur  = jsonInt(b, "cur");
        st.bendTgt  = jsonInt(b, "tgt");
        st.bendCyc  = jsonInt(b, "cyc");
        st.bendMode = jsonInt(b, "mode");
        st.bendDone = jsonInt(b, "done");
        st.pedalFwd = jsonInt(b, "pedalFwd") != 0;
        st.pedalRev = jsonInt(b, "pedalRev") != 0;
        st.modeAuto = st.bendMode == 1;
        // Relay state from Bend sub-object (v4.4+: clampClosed field added)
        st.clampClosed = jsonInt(b, "clampClosed") != 0;
        st.unclampOpen = !st.clampClosed;  // Always inverse (mutex logic)

        // ДИАГНОСТИКА: логируем состояние реле
        static int relayDiagCount_ = 0;
        if (++relayDiagCount_ % 200 == 0) {
            addLog(std::string("[RELAY_STATE_JSON] clampClosed=") + (st.clampClosed ? "1" : "0") +
                   ", unclampOpen=" + (st.unclampOpen ? "1" : "0") +
                   " | JSON field present=" + (json.find("\"clampClosed\":") != std::string::npos ? "YES" : "NO"));
        }
    }

    if (bPos == std::string::npos) {
        st.bendCur = jsonInt(json, "angle");
        st.bendTgt = jsonInt(json, "target");
        st.pedalFwd = jsonInt(json, "fwd") != 0;
        st.pedalRev = jsonInt(json, "rev") != 0;
        st.modeAuto = jsonInt(json, "auto") != 0;
        st.bendMode = st.modeAuto ? 1 : 0;
    }

    // If clampClosed not set from JSON, sync from internal state (simulation mode)
    if (!st.clampClosed && !st.unclampOpen && bPos != std::string::npos) {
        // Use relayState_ from poll loop
        st.clampClosed = relayState_.clampActive;  
        st.unclampOpen = relayState_.unclampActive;
    }

    st.wdTrip = jsonInt(json, "wd") != 0;

    // Alarm sub-object: "alm":{...}
    auto almPos = json.find("\"alm\":{");
    if (almPos != std::string::npos) {
        std::string alm = json.substr(almPos);
        st.almZ = jsonInt(alm, "z") != 0;
        st.almC = jsonInt(alm, "c") != 0;
    }

    auto limPos = json.find("\"lim\":{");
    if (limPos != std::string::npos) {
        std::string lim = json.substr(limPos);
        st.limZMin = jsonInt(lim, "zMin") != 0;
        st.limZMax = jsonInt(lim, "zMax") != 0;
    }

    auto vfdPos = json.find("\"vfd\":{");
    if (vfdPos != std::string::npos) {
        std::string vfd = json.substr(vfdPos);
        st.vfdFreqHz = jsonDouble(vfd, "freq");
        st.vfdCurrentA = jsonDouble(vfd, "cur");
    }

    auto bootPos = json.find("\"boot\":{");
    if (bootPos != std::string::npos) {
        std::string boot = json.substr(bootPos);
        st.bootRst = (unsigned long)jsonInt(boot, "rst");
        st.bootUpMs = (unsigned long)jsonInt(boot, "upMs");
    }

    auto diagPos = json.find("\"diag\":{");
    if (diagPos != std::string::npos) {
        std::string diag = json.substr(diagPos);
        st.diagLoopHz = (unsigned long)jsonInt(diag, "loopHz");
        st.diagLoopMaxUs = (unsigned long)jsonInt(diag, "loopMaxUs");
        st.diagStepHz = (unsigned long)jsonInt(diag, "stepHz");
        st.diagBendIsrHz = (unsigned long)jsonInt(diag, "bendIsrHz");
        st.diagZIsrHz = (unsigned long)jsonInt(diag, "zIsrHz");
        st.diagCIsrHz = (unsigned long)jsonInt(diag, "cIsrHz");
        st.diagRxHz = (unsigned long)jsonInt(diag, "rxHz");
        st.diagRxOvHz = (unsigned long)jsonInt(diag, "rxOvHz");
        st.diagSonZOut = jsonInt(diag, "sonZOut") != 0;
        st.diagSonCOut = jsonInt(diag, "sonCOut") != 0;
        st.diagPedalIn = jsonInt(diag, "pedalIn") != 0;
        st.diagModeIn = jsonInt(diag, "modeIn") != 0;
    }

    // External command diagnostics: "ext":{...}
    auto extPos = json.find("\"ext\":{");
    if (extPos != std::string::npos) {
        std::string ext = json.substr(extPos);
        st.extCmd8  = jsonInt(ext, "cmd8");
        st.extCmd9  = jsonInt(ext, "cmd9");
        st.extTgt10 = jsonInt(ext, "tgt10");
        st.extPulse = jsonInt(ext, "pulse");
        st.extFast  = jsonInt(ext, "fast");
        st.extDo7   = jsonInt(ext, "do7");
        st.extDo8   = jsonInt(ext, "do8");
    }

    auto encPos = json.find("\"enc\":{");
    if (encPos != std::string::npos) {
        std::string enc = json.substr(encPos);
        st.zFbCount = (long)jsonDouble(enc, "zCnt");
        st.cFbCount = (long)jsonDouble(enc, "cCnt");
        st.zFbMm = jsonDouble(enc, "zMm");
        st.cFbDeg = jsonDouble(enc, "cDeg");
        st.encValid = true;
        
        // DEBUG: Log encoder values
        static int encLogCount = 0;
        if (++encLogCount % 5 == 0) {  // Log every 5th update (~2Hz if 10Hz Teensy)
            Logger::log(std::string("[ENC_PARSE] zCnt=") + std::to_string(st.zFbCount) +
                        " cCnt=" + std::to_string(st.cFbCount) +
                        " zMm=" + std::to_string(st.zFbMm) +
                        " cDeg=" + std::to_string(st.cFbDeg) +
                        " encValid=" + std::to_string(st.encValid));
        }
    }

    st.valid = true;
    return st;
}

// ─── Direct Commands ──────────────────────────────────────────────────

void Machine::requestStatus() { if (!simulated_) serial_.send("?"); }

void Machine::servoOn()  {
    if (!simulated_) serial_.send("SON 1");
    addLog("> SON 1 (sent directly)");
}
void Machine::servoOff() {
    if (!simulated_) serial_.send("SON 0");
    addLog("> SON 0");
}
bool Machine::tryHome(char axis) {
    if (!isConnected()) {
        lastError_ = "Home rejected: machine is not connected";
        addLog("!!! " + lastError_);
        return false;
    }

    if (isProgramRunning()) {
        lastError_ = "Home rejected: stop program first";
        addLog("!!! " + lastError_);
        return false;
    }

    if (!status_.valid) {
        lastError_ = "Home rejected: no machine status yet";
        addLog("!!! " + lastError_);
        return false;
    }

    if (status_.almZ || status_.almC) {
        lastError_ = "Home rejected: servo alarm active";
        addLog("!!! " + lastError_);
        return false;
    }

    if (axis == 'Z') {
        phase_ = MachPhase::HomingZ;
        homeMoveIssued_ = false;
        axisCmdPending_ = false;
        lastError_.clear();
        addLog("Home Z: return to encoder zero requested");
        if (!isServoOnConfirmed()) {
            sonWaitStarted_ = false;
            servoOnStart_ = std::chrono::steady_clock::now();
            servoOnStarted_ = true;
            servoOn();
            addLog("Home Z: waiting for SON confirmation before move to 0");
        }
        return true;
    }
    if (axis == 'C') {
        phase_ = MachPhase::HomingC;
        homeMoveIssued_ = false;
        axisCmdPending_ = false;
        lastError_.clear();
        addLog("Home C: return to encoder zero requested");
        if (!isServoOnConfirmed()) {
            sonWaitStarted_ = false;
            servoOnStart_ = std::chrono::steady_clock::now();
            servoOnStarted_ = true;
            servoOn();
            addLog("Home C: waiting for SON confirmation before move to 0");
        }
        return true;
    }

    lastError_ = "Home rejected: unknown axis";
    addLog("!!! " + lastError_);
    return false;
}

void Machine::homeZ()    { tryHome('Z'); }
void Machine::homeC()    { tryHome('C'); }
void Machine::stop() {
    if (!simulated_) serial_.send("STOP");
    addLog("> STOP");
    homeMoveIssued_ = false;
    if (simulated_) {
        // Freeze simulation at current position
        simZTarget_ = simZPos_;
        simCTarget_ = simCPos_;
        simBendRunning_ = false;
    }
}
void Machine::estop() {
    if (serial_.isOpen())
        serial_.send("ESTOP");
    addLog("> ESTOP");
    phase_ = MachPhase::Idle;
    homeMoveIssued_ = false;
    waitingForAxis_ = false;
    waitingForBend_ = false;
    axisCmdPending_ = false;
    pedalInterlockPaused_ = false;
    simBendSubPhase_ = 0;
    sonWaitStarted_ = false;
    servoOnStarted_ = false;
    lastLoggedPhase_ = MachPhase::Idle;
    lastLoggedStep_ = -1;
}

void Machine::moveZ(double mm, double speed) {
    char buf[64];
    snprintf(buf, sizeof(buf), "MZ %.2f,%.1f", mm, speed);
    if (!simulated_) serial_.send(buf);
    addLog(std::string("> ") + buf);

    if (simulated_) {
        // ⚠️ КРИТИЧНО: ЗАПРЕТИТЬ Z ДВИЖЕНИЕ НАЗАД! Исключение: только во время лазерной резки
        bool isLaserCutPhase = (phase_ == MachPhase::LaserCutBefore || phase_ == MachPhase::LaserCutBeforeWait);
        
        if (mm < simZPos_ && !isLaserCutPhase) {
            Logger::log(std::string("[Z_SAFETY_BLOCK] BACKWARD TARGET BLOCKED! current=") + 
                       std::to_string(simZPos_) + " requested=" + std::to_string(mm) + 
                       " phase=" + std::to_string((int)phase_) + " IGNORING");
            // Не устанавливаем target если он назад и не резка!
        } else {
            simZTarget_ = mm;
            if (isLaserCutPhase && mm < simZPos_) {
                Logger::log(std::string("[Z_LASER_OK] Backward during cut: ") + std::to_string(simZPos_) + 
                           " -> " + std::to_string(mm));
            }
        }
        simZSpeed_ = speed;
        axisCmdPending_ = false;
    } else {
        axisCmd_ = 'Z';
        axisCmdStart_ = std::chrono::steady_clock::now();
        axisCmdStartPos_ = zAxisPos(status_);
        axisCmdTarget_ = mm;
        axisCmdPending_ = std::abs(axisCmdTarget_ - axisCmdStartPos_) > 0.05;
    }
}

void Machine::moveC(double deg, double speed) {
    char buf[64];
    snprintf(buf, sizeof(buf), "MC %.2f,%.1f", deg, speed);
    if (!simulated_) serial_.send(buf);
    addLog(std::string("> ") + buf);

    if (simulated_) {
        simCTarget_ = deg;
        simCSpeed_ = speed;
        axisCmdPending_ = false;
    } else {
        axisCmd_ = 'C';
        axisCmdStart_ = std::chrono::steady_clock::now();
        axisCmdStartPos_ = cAxisPos(status_);
        axisCmdTarget_ = deg;
        axisCmdPending_ = std::abs(axisCmdTarget_ - axisCmdStartPos_) > 0.2;
    }
}

void Machine::sendBendTarget(int angle) {
    char buf[32];
    snprintf(buf, sizeof(buf), "TGT %d", angle);
    if (!simulated_) serial_.send(buf);
    addLog(std::string("> ") + buf);
    if (simulated_) {
        simBendTargetDeg_ = angle;
    }
}

void Machine::sendBendStart() {
    if (!simulated_) serial_.send("CMD 4");
    addLog("> CMD 4 (auto-bend)");
    if (simulated_) {
        simBendRunning_ = true;
        // Cycle direction: 1=forward, 2=return
        if (simBendCycle_ == 1) {
            // Start bending forward
        } else {
            // Return to zero
        }
    }
}

void Machine::sendBendStop() {
    if (!simulated_) serial_.send("CMD 2");
    addLog("> CMD 2 (stop)");
    if (simulated_) {
        simBendRunning_ = false;
    }
}

void Machine::sendZero() {
    if (!simulated_) serial_.send("CMD 3");
    addLog("> CMD 3 (zero)");
}

void Machine::sendZeroPos() {
    if (!simulated_) serial_.send("ZP");
    addLog("> ZP (zero position both axes)");
}

void Machine::sendSync() {
    if (!simulated_) serial_.send("SYNC");
    addLog("> SYNC (sync step position from encoder)");
}

void Machine::sendArcFeed(double R, double targetAngle) {
    char buf[64];
    snprintf(buf, sizeof(buf), "ARCFEED %.2f,%.1f", R, targetAngle);
    if (!simulated_) serial_.send(buf);
    addLog(std::string("> ") + buf + " (arc feed sync)");
}

void Machine::sendArcStop() {
    if (!simulated_) serial_.send("ARCSTOP");
    addLog("> ARCSTOP");
}

void Machine::sendBendZero() {
    if (!simulated_) serial_.send("BZ");
    addLog("> BZ (zero bend encoder, persist)");
}

void Machine::sendClamp() {
    if (!simulated_) serial_.send("CLAMP");
    addLog("> CLAMP (close clamp)");
    
    // Активировать импульс реле
    if (canActivateRelay(35)) {
        clampPulseActive_ = true;
        clampPulseStart_ = std::chrono::steady_clock::now();
        relayState_.clampActive = true;
        relayState_.clampActivatedAt = clampPulseStart_;
        addLog("Relay CLAMP activated (1.5s pulse started)");
    } else {
        addLog("WARNING: Cannot activate CLAMP relay - UNCLAMP is active (interlock)");
    }
}

void Machine::sendUnclamp() {
    if (!simulated_) serial_.send("UNCLAMP");
    addLog("> UNCLAMP (open clamp)");
    
    // Активировать импульс реле
    if (canActivateRelay(36)) {
        unclampPulseActive_ = true;
        unclampPulseStart_ = std::chrono::steady_clock::now();
        relayState_.unclampActive = true;
        relayState_.unclampActivatedAt = unclampPulseStart_;
        addLog("Relay UNCLAMP activated (1.5s pulse started)");
    } else {
        addLog("WARNING: Cannot activate UNCLAMP relay - CLAMP is active (interlock)");
    }
}

void Machine::sendRelayOff() {
    if (!simulated_) serial_.send("RELAYOFF");
    addLog("> RELAYOFF (both relays de-energized)");
    clampPulseActive_ = false;
    unclampPulseActive_ = false;
    relayState_.clampActive = false;
    relayState_.unclampActive = false;
}

void Machine::sendStep(int cur, int total) {
    char buf[32];
    snprintf(buf, sizeof(buf), "STEP %d,%d", cur, total);
    if (!simulated_) serial_.send(buf);
}

void Machine::sendCalibration(double zMm, double cDeg, int bendDeg) {
    char buf[96];
    snprintf(buf, sizeof(buf), "CAL %.3f,%.3f,%d", zMm, cDeg, bendDeg);
    if (!simulated_) serial_.send(buf);
    addLog(std::string("> ") + buf + " (calibrate axes, persist)");
}

// ─── Program Execution ────────────────────────────────────────────────

void Machine::startProgram(const BendingProgram& prog, double clampLength, const std::string& programName) {
    program_ = prog;
    program_.deriveCuts();
    program_.steps = program_.bendSteps();
    programName_ = programName;
    clampLength_ = clampLength;
    curStep_ = 0;
    totalSteps_ = (int)program_.steps.size();
    
    // Program identification logging
    Logger::log(std::string("[PROGRAM_START] Program: ") + programName + 
               " | steps=" + std::to_string(totalSteps_) + 
               " | clampLength=" + std::to_string(clampLength));
    feedAccum_ = 0;
    rotAccum_ = 0;
    bendStartFeedAccum_ = 0.0;
    prevBendDone_ = 0;
    prevBendCyc_ = 0;
    simBendSubPhase_ = 0;
    lastCachedStepIdx_ = -1;  // Обнулить кэш
    updateBaseAccumulators();  // Кэшировать базы для шага 0
    phaseStarted_ = false;
    if (totalSteps_ == 0) {
        lastError_ = "Empty program";
        phase_ = MachPhase::Error;
        return;
    }

    // Apply hardware speed limits if not in simulation mode
    if (!simulated_) {
        if (feedSpeed_ > HW_MAX_FEED_SPEED) {
            addLog("⚠️ Feed speed clamped from " + std::to_string(feedSpeed_) + 
                   " to " + std::to_string(HW_MAX_FEED_SPEED) + " mm/s (HW limit)");
            feedSpeed_ = HW_MAX_FEED_SPEED;
        }
        if (rotSpeed_ > HW_MAX_ROT_SPEED) {
            addLog("⚠️ Rotation speed clamped from " + std::to_string(rotSpeed_) + 
                   " to " + std::to_string(HW_MAX_ROT_SPEED) + " deg/s (HW limit)");
            rotSpeed_ = HW_MAX_ROT_SPEED;
        }
    }

    // In simulation mode, force bypass pedal and simulate bend encoder
    if (simulated_) {
        pedalBypass_ = true;
        ignoreBendEncoder_ = true;
        simZPos_ = 0.0;
        simCPos_ = 0.0;
        simBendPos_ = 0.0;
        simZTarget_ = 0.0;
        simCTarget_ = 0.0;
        simBendTargetDeg_ = 0;
        simBendCycle_ = 1;
        simBendRunning_ = false;
        simLastTime_ = std::chrono::steady_clock::now();
    }

    if (!programName_.empty()) {
        addLog("=== Program start: " + programName_ + " (" + std::to_string(totalSteps_) + " steps) ===");
    } else {
        addLog("=== Program start: " + std::to_string(totalSteps_) + " steps ===");
    }
    addLog("CFG runtime: pedalBypass=" + std::string(pedalBypass_ ? "1" : "0") +
           " ignoreBendEncoder=" + std::string(ignoreBendEncoder_ ? "1" : "0") +
           " feedSpeed=" + std::to_string(feedSpeed_) + " mm/s" +
           " rotSpeed=" + std::to_string(rotSpeed_) + " deg/s" +
           " clampDelay=" + std::to_string(clampDelayMs_) + " ms" +
           " unclampDelay=" + std::to_string(unclampDelayMs_) + " ms" +
           " statusPoll=" + std::to_string(STATUS_INTERVAL_MS) + " ms");
    addLog("CFG program: D=" + std::to_string(program_.D) +
           " R=" + std::to_string(program_.R) +
           " clamp=" + std::to_string(clampLength_) +
           " lTotal=" + std::to_string(program_.lTotal()));
    for (int i = 0; i < totalSteps_; ++i) {
        const auto& s = program_.steps[i];
        addLog("STEP " + std::to_string(i + 1) + "/" + std::to_string(totalSteps_) +
               ": L=" + std::to_string(s.feedLength) +
               " C=" + std::to_string(s.rotation) +
               " B=" + std::to_string(s.bendAngle));
    }
    lastLoggedPhase_ = MachPhase::Idle;
    lastLoggedStep_ = -1;

    // Start with servo on
    homeMoveIssued_ = false;
    sonWaitStarted_ = false;
    servoOnStart_ = std::chrono::steady_clock::now();
    servoOnStarted_ = true;
    phase_ = MachPhase::ServoOn;
    servoOn();
    waitingForAxis_ = true;
}

void Machine::stopProgram() {
    stop();
    // Disable servos when stopping program
    if (serial_.isOpen()) serial_.send("SON 0");
    addLog("> SON 0 (disable servos on stop)");
    phase_ = MachPhase::Idle;
    waitingForAxis_ = false;
    waitingForBend_ = false;
    axisCmdPending_ = false;
    pedalInterlockPaused_ = false;
    simBendSubPhase_ = 0;
    sonWaitStarted_ = false;
    servoOnStarted_ = false;
    homeMoveIssued_ = false;
    lastLoggedPhase_ = MachPhase::Idle;
    lastLoggedStep_ = -1;
    addLog("=== Program stopped ===");
}

std::string Machine::phaseText() const {
    switch (phase_) {
        case MachPhase::Idle:            return u8"Ожидание";
        case MachPhase::ServoOn:         return u8"Servo ON...";
        case MachPhase::HomingZ:         return u8"Home Z...";
        case MachPhase::HomingC:         return u8"Home C...";
        case MachPhase::Feed:            return u8"Подача Z";
        case MachPhase::Rotation:        return u8"Вращение C";
        case MachPhase::ClampWait:       return u8"Ожидание прижима...";
        case MachPhase::SimulatedBendWait: return u8"Гибка (симуляция)";
        case MachPhase::BendStart:       return u8"Гибка: старт";
        case MachPhase::BendWait:        return u8"Гибка: ожидание";
        case MachPhase::UnclampWait1:    return u8"Разжим прижима";
        case MachPhase::Clearance:       return u8"Подача на длину прижима";
        case MachPhase::ReturnBend:      return u8"Возврат ролика";
        case MachPhase::ReturnBendWait:  return u8"Возврат: ожидание";
        case MachPhase::StepDone:        return u8"Шаг завершён";
        case MachPhase::ProgramDone:     return u8"Программа завершена";
        case MachPhase::Error:           return u8"Ошибка";
    }
    return "?";
}

// ─── State Machine ────────────────────────────────────────────────────

void Machine::advanceProgram() {
    if (!status_.valid) return;

    if (phase_ != lastLoggedPhase_ || curStep_ != lastLoggedStep_) {
        addLog("PHASE " + phaseText() + " | step=" + std::to_string(curStep_ + 1) + "/" + std::to_string(totalSteps_) +
               " | z=" + std::to_string(zAxisPos(status_)) +
               " | c=" + std::to_string(cAxisPos(status_)) +
               " | bend=" + std::to_string(status_.bendCur));
        lastLoggedPhase_ = phase_;
        lastLoggedStep_ = curStep_;
    }

    // Servo alarm interlock: stop immediately on any active drive alarm.
    if (status_.almZ || status_.almC) {
        stop();
        lastError_ = status_.almZ && status_.almC
            ? u8"Аларм серво Z и C"
            : (status_.almZ ? u8"Аларм серво Z" : u8"Аларм серво C");
        phase_ = MachPhase::Error;
        addLog("!!! " + lastError_);
        return;
    }

    // ====== HARD PEDAL D7 INTERLOCK ======
    bool pedalHoldRequired = (phase_ == MachPhase::Feed || phase_ == MachPhase::Rotation || phase_ == MachPhase::BendStart
                              || phase_ == MachPhase::Clearance);
    if (!pedalBypass_ && pedalHoldRequired && !status_.pedalFwd) {
        if (!pedalInterlockPaused_) {
            addLog("Warning: Pedal released during active running phase, pausing motion.");
            stop();
            pedalInterlockPaused_ = true;
        }
        return;
    }
    if (pedalInterlockPaused_ && (status_.pedalFwd || pedalBypass_)) {
        addLog("Pedal pressed, resuming motion...");
        if (phase_ == MachPhase::Feed || phase_ == MachPhase::Clearance) {
            moveZ(feedAccum_, feedSpeed_);
        } else if (phase_ == MachPhase::Rotation) {
            moveC(rotAccum_, rotSpeed_);
        } else if (phase_ == MachPhase::BendStart) {
            auto& step = program_.steps[curStep_];
            sendBendTarget((int)std::round(step.bendAngle));
        }
        pedalInterlockPaused_ = false;
    }

    // Detect command accepted but motion not started (typical wiring/pulse path issue)
    if (axisCmdPending_) {
        const auto waited = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - axisCmdStart_).count();

        if (axisCmd_ == 'Z') {
            const double zNow = zAxisPos(status_);
            const bool moved = std::abs(zNow - axisCmdStartPos_) > 0.05;
            const bool started = moved || status_.zSt == 1 || status_.zSt == 2;
            if (started) {
                axisCmdPending_ = false;
            } else if (waited > AXIS_START_TIMEOUT_MS) {
                std::ostringstream oss;
                oss << "Z motion did not start in " << waited
                    << " ms (zSt=" << status_.zSt
                    << ", zSon=" << (status_.zSon ? 1 : 0)
                    << ", cSon=" << (status_.cSon ? 1 : 0)
                    << ", do7=" << status_.extDo7
                    << ", do8=" << status_.extDo8
                    << ", cmd8=" << status_.extCmd8
                    << ", cmd9=" << status_.extCmd9
                    << ", pulse=" << status_.extPulse
                    << ", fast=" << status_.extFast
                    << ", from=" << axisCmdStartPos_
                    << ", to=" << axisCmdTarget_
                    << ", now=" << zNow << ")";
                lastError_ = oss.str();
                addLog("!!! " + lastError_);
                stop();
                phase_ = MachPhase::Error;
                axisCmdPending_ = false;
                return;
            }
        } else if (axisCmd_ == 'C') {
            const double cNow = cAxisPos(status_);
            const bool moved = std::abs(cNow - axisCmdStartPos_) > 0.2;
            const bool started = moved || status_.cSt == 1 || status_.cSt == 2;
            if (started) {
                axisCmdPending_ = false;
            } else if (waited > AXIS_START_TIMEOUT_MS) {
                std::ostringstream oss;
                oss << "C motion did not start in " << waited
                    << " ms (cSt=" << status_.cSt
                    << ", zSon=" << (status_.zSon ? 1 : 0)
                    << ", cSon=" << (status_.cSon ? 1 : 0)
                    << ", do7=" << status_.extDo7
                    << ", do8=" << status_.extDo8
                    << ", cmd8=" << status_.extCmd8
                    << ", cmd9=" << status_.extCmd9
                    << ", pulse=" << status_.extPulse
                    << ", fast=" << status_.extFast
                    << ", from=" << axisCmdStartPos_
                    << ", to=" << axisCmdTarget_
                    << ", now=" << cNow << ")";
                lastError_ = oss.str();
                addLog("!!! " + lastError_);
                stop();
                phase_ = MachPhase::Error;
                axisCmdPending_ = false;
                return;
            }
        }
        // Motion command sent but not yet confirmed — block state machine
        return;
    }

    switch (phase_) {

    case MachPhase::HomingZ:
    case MachPhase::HomingC: {
        const char axis = (phase_ == MachPhase::HomingZ) ? 'Z' : 'C';
        const char* axisName = (axis == 'Z') ? "Z" : "C";
        const bool axisDone = (axis == 'Z') ? (status_.zSt == 0) : (status_.cSt == 0);
        const bool axisError = (axis == 'Z') ? (status_.zSt == 3) : (status_.cSt == 3);

        if (!homeMoveIssued_) {
            if (!isServoOnConfirmed()) {
                if (!servoOnStarted_) {
                    servoOnStart_ = std::chrono::steady_clock::now();
                    servoOnStarted_ = true;
                    sonWaitStarted_ = false;
                    servoOn();
                }
                auto waited = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - servoOnStart_).count();
                if (waited > SERVO_ON_TIMEOUT_MS) {
                    std::ostringstream oss;
                    oss << "Home " << axisName << ": Servo ON not confirmed in " << waited
                        << " ms (zSon=" << (status_.zSon ? 1 : 0)
                        << ", cSon=" << (status_.cSon ? 1 : 0)
                        << ", mbLink=" << status_.mbLink << "ms)";
                    lastError_ = oss.str();
                    addLog("!!! " + lastError_);
                    phase_ = MachPhase::Error;
                    servoOnStarted_ = false;
                }
                break;
            }

            if (!sonWaitStarted_) {
                sonWaitStart_ = std::chrono::steady_clock::now();
                sonWaitStarted_ = true;
                addLog(std::string("Home ") + axisName + ": SON confirmed, waiting settle...");
                break;
            }

            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - sonWaitStart_).count();
            if (elapsed < SON_RELAY_DELAY_MS) {
                break;
            }

            sonWaitStarted_ = false;
            servoOnStarted_ = false;
            homeMoveIssued_ = true;
            sendSync();
            if (axis == 'Z') {
                moveZ(0.0, kHomeZReturnSpeedMm);
                addLog("Home Z: SYNC + move to encoder zero started");
            } else {
                moveC(0.0, kHomeCReturnSpeedDeg);
                addLog("Home C: SYNC + move to encoder zero started");
            }
            break;
        }

        if (axisError) {
            lastError_ = std::string("Home ") + axisName + ": axis error";
            addLog("!!! " + lastError_);
            phase_ = MachPhase::Error;
            homeMoveIssued_ = false;
            break;
        }

        // ✅ FIXED: Support simulated mode for homing
        bool homeComplete = false;
        if (simulated_) {
            // In simulation, check if sim target reached
            if (axis == 'Z') {
                homeComplete = (std::abs(simZPos_ - 0.0) < 0.1);
                if (homeComplete) simZPos_ = 0.0;
            } else {
                homeComplete = (std::abs(simCPos_ - 0.0) < 0.1);
                if (homeComplete) simCPos_ = 0.0;
            }
        } else {
            homeComplete = (axisDone && !axisCmdPending_);
        }

        if (homeComplete) {
            addLog(std::string("Home ") + axisName + ": axis returned to encoder zero");
            phase_ = MachPhase::Idle;
            homeMoveIssued_ = false;
        }
        break;
    }

    case MachPhase::ServoOn:
        if (status_.zSon && status_.cSon) {
            phase_ = MachPhase::UnclampWait1; // Переход к П.1
            phaseStarted_ = false;
        }
        break;

    case MachPhase::UnclampWait1: // П.1: Разжим перед подачей
        if (!phaseStarted_) {
            delayStart_ = std::chrono::steady_clock::now();
            delayMs_ = unclampDelayMs_;
            sendUnclamp();  // Command Teensy to open clamp
            phaseStarted_ = true;
            addLog(u8"Phase: Initial Unclamp");
        }
        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - delayStart_).count() >= delayMs_) {
            phase_ = MachPhase::Feed;
            phaseStarted_ = false;
        }
        break;

    case MachPhase::Feed: // П.2: Подача Z
        if (!phaseStarted_) {
            auto& step = program_.steps[curStep_];
            feedAccum_ += step.feedLength; // Накапливаем подачу
            sendStep(curStep_ + 1, totalSteps_);
            moveZ(feedAccum_, feedSpeed_);
            phaseStarted_ = true;
            stallArmed_ = false;
            addLog("Step " + std::to_string(curStep_ + 1) + "/" + std::to_string(totalSteps_) + " (feed)");
            Logger::log(std::string("[FEED_START] feedAccum=") + std::to_string(feedAccum_) +
                       " simZTarget=" + std::to_string(simZTarget_) +
                       " simZPos=" + std::to_string(simZPos_) +
                       " feedSpeed=" + std::to_string(feedSpeed_));
        }
        // Hardware stall watchdog: once Z is actually moving (zSt==2), make sure
        // the reported position advances at least STALL_Z_EPS within STALL_TIMEOUT_MS.
        if (!simulated_ && !axisCmdPending_ && status_.zSt == 2) {
            const double zNow = zAxisPos(status_);
            const auto now = std::chrono::steady_clock::now();
            if (!stallArmed_) {
                stallLastPos_ = zNow;
                stallLastProgressAt_ = now;
                stallArmed_ = true;
            } else if (std::abs(zNow - stallLastPos_) > STALL_Z_EPS) {
                stallLastPos_ = zNow;
                stallLastProgressAt_ = now;
            } else {
                auto stalledMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - stallLastProgressAt_).count();
                if (stalledMs > STALL_TIMEOUT_MS) {
                    std::ostringstream oss;
                    oss << "Z stalled mid-feed: no progress for " << stalledMs
                        << " ms (zSt=" << status_.zSt
                        << ", z=" << zNow
                        << ", target=" << feedAccum_
                        << ", almZ=" << (status_.almZ ? 1 : 0) << ")";
                    lastError_ = oss.str();
                    addLog("!!! " + lastError_);
                    stop();
                    phase_ = MachPhase::Error;
                    stallArmed_ = false;
                    break;
                }
            }
        } else if (status_.zSt != 2) {
            stallArmed_ = false;
        }
        // ⚠️ КРИТИЧЕСКОЕ ИЗМЕНЕНИЕ: В симуляции мы НЕ выходим из Feed до тех пор пока Z НЕ ДОСТИГНЕТ целевой позиции!
        // В режиме Hardware мы проверяем status_.zSt (ось завершила движение)
        if (simulated_) {
            double zDiff = std::abs(simZTarget_ - simZPos_);
            if (zDiff <= 0.1) {  // Z достигла целевой позиции (с допуском 0.1 мм)
                Logger::log(std::string("[FEED_DONE_SIM] zDiff=") + 
                           std::to_string(zDiff) + 
                           " simZPos=" + std::to_string(simZPos_) + 
                           " simZTarget=" + std::to_string(simZTarget_));
                simZPos_ = simZTarget_;  // Принудительно синхронизируем
                phase_ = MachPhase::Rotation;
                phaseStarted_ = false;
            }
        } else {
            // Hardware mode: проверяем if Z stopped
            if (status_.zSt == 0 && !axisCmdPending_) {
                Logger::log(std::string("[FEED_DONE_HW] zSt=") + std::to_string(status_.zSt) + 
                           " simZPos=" + std::to_string(simZPos_));
                phase_ = MachPhase::Rotation;
                phaseStarted_ = false;
            }
        }
        if (status_.zSt == 3) { lastError_ = "Z axis error"; phase_ = MachPhase::Error; }
        break;

    case MachPhase::Rotation: { // П.3: Ротация C
        if (!phaseStarted_) {
            auto& step = program_.steps[curStep_];
            rotAccum_ += step.rotation;
            if (std::abs(step.rotation) > 0.01) {
                moveC(rotAccum_, rotSpeed_);
            }
            phaseStarted_ = true;
            stallArmed_ = false;
        }
        // Hardware stall watchdog for C rotation (mirrors Feed). Trips on
        // the step-4 failure mode: cSt stays == 2 but cDeg doesn't advance.
        if (!simulated_ && !axisCmdPending_ && status_.cSt == 2) {
            const double cNow = cAxisPos(status_);
            const auto now = std::chrono::steady_clock::now();
            if (!stallArmed_) {
                stallLastPos_ = cNow;
                stallLastProgressAt_ = now;
                stallArmed_ = true;
            } else if (std::abs(cNow - stallLastPos_) > STALL_C_EPS) {
                stallLastPos_ = cNow;
                stallLastProgressAt_ = now;
            } else {
                auto stalledMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - stallLastProgressAt_).count();
                if (stalledMs > STALL_TIMEOUT_MS) {
                    std::ostringstream oss;
                    oss << "C stalled mid-rotation: no progress for " << stalledMs
                        << " ms (cSt=" << status_.cSt
                        << ", c=" << cNow
                        << ", target=" << rotAccum_
                        << ", almC=" << (status_.almC ? 1 : 0) << ")";
                    lastError_ = oss.str();
                    addLog("!!! " + lastError_);
                    stop();
                    phase_ = MachPhase::Error;
                    stallArmed_ = false;
                    break;
                }
            }
        } else if (status_.cSt != 2) {
            stallArmed_ = false;
        }
        // ⚠️ КРИТИЧЕСКОЕ ИЗМЕНЕНИЕ: В симуляции мы НЕ выходим из Rotation до тех пор пока C НЕ ДОСТИГНЕТ целевой позиции!
        // В режиме Hardware мы проверяем status_.cSt (ось завершила движение)
        if (simulated_) {
            auto& step = program_.steps[curStep_];
            if (std::abs(step.rotation) <= 0.01) {
                // No rotation needed, exit immediately
                Logger::log("[ROTATION_NO_MOVE] step.rotation < 0.01");
                if (step.isCut) {
                    phase_ = MachPhase::LaserCutBefore;
                    addLog(u8"Резка");
                } else if (std::abs(step.bendAngle) > 0.01) {
                    phase_ = MachPhase::ClampWait;
                } else {
                    phase_ = MachPhase::StepDone;
                }
                phaseStarted_ = false;
            } else {
                double cDiff = std::abs(simCTarget_ - simCPos_);
                if (cDiff <= 0.1) {  // C достигла целевой позиции (с допуском 0.1 град)
                    Logger::log(std::string("[ROTATION_DONE_SIM] cDiff=") + 
                               std::to_string(cDiff) + 
                               " simCPos=" + std::to_string(simCPos_) + 
                               " simCTarget=" + std::to_string(simCTarget_));
                    simCPos_ = simCTarget_;  // Принудительно синхронизируем
                    if (step.isCut) {
                        phase_ = MachPhase::LaserCutBefore;
                        addLog(u8"Резка");
                    } else if (std::abs(step.bendAngle) > 0.01) {
                        phase_ = MachPhase::ClampWait;
                    } else {
                        phase_ = MachPhase::StepDone;
                    }
                    phaseStarted_ = false;
                }
            }
        } else {
            // Hardware mode: проверяем if C stopped
            auto& step = program_.steps[curStep_];
            if (std::abs(step.rotation) <= 0.01) {
                // No rotation, just exit
                if (step.isCut) {
                    phase_ = MachPhase::LaserCutBefore;
                    addLog(u8"Резка");
                } else if (std::abs(step.bendAngle) > 0.01) {
                    phase_ = MachPhase::ClampWait;
                } else {
                    phase_ = MachPhase::StepDone;
                }
                phaseStarted_ = false;
            } else if (status_.cSt == 0 && !axisCmdPending_) {
                Logger::log(std::string("[ROTATION_DONE_HW] cSt=") + std::to_string(status_.cSt));
                if (step.isCut) {
                    phase_ = MachPhase::LaserCutBefore;
                    addLog(u8"Резка");
                } else if (std::abs(step.bendAngle) > 0.01) {
                    phase_ = MachPhase::ClampWait;
                } else {
                    phase_ = MachPhase::StepDone;
                }
                phaseStarted_ = false;
            }
        }
        if (status_.cSt == 3) { lastError_ = "C axis error"; phase_ = MachPhase::Error; }
        break;
    }

    case MachPhase::LaserCutBefore: {
        if (!phaseStarted_) {
            delayStart_ = std::chrono::steady_clock::now();
            delayMs_ = 400; // v2.5: cut phase is handled by Machine state machine
            phaseStarted_ = true;
            addLog(u8"Резка: старт");
            phase_ = MachPhase::LaserCutBeforeWait;
        }
        break;
    }

    case MachPhase::LaserCutBeforeWait: {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - delayStart_).count();
        if (elapsed >= delayMs_) {
            addLog(u8"Резка: завершена");
            phase_ = MachPhase::StepDone;
            phaseStarted_ = false;
        }
        break;
    }

    case MachPhase::ClampWait: // П.4: Прижим
        if (!phaseStarted_) {
            delayStart_ = std::chrono::steady_clock::now();
            delayMs_ = clampDelayMs_;
            sendClamp();  // Command Teensy to close clamp
            phaseStarted_ = true;
            addLog(u8"Прижим");
        }
        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - delayStart_).count() >= delayMs_) {
            phase_ = MachPhase::BendStart;
            phaseStarted_ = false;
        }
        break;

    case MachPhase::BendStart: // П.5: Начало гибки
        {
            auto& step = program_.steps[curStep_];

            sendBendTarget((int)std::round(step.bendAngle));

            // Path A — true HARDWARE with working bend encoder:
            //   Teensy runs ARCFEED sync, we wait for bendCyc==2 from firmware.
            // Path B — SIMULATION (no hardware at all):
            //   Drive the bend purely via EXE sim state, wait a fixed delay.
            // Path C — HARDWARE but the bend encoder is unreliable and the user
            //   checked «Обход энкодера гибки». We STILL send the real Teensy
            //   commands (otherwise the physical bender won't move!), but we
            //   exit on a computed timeout instead of waiting for bendCyc==2.
            const bool isSim = simulated_;
            if (!isSim && !ignoreBendEncoder_) {
                // Path A — HARDWARE with encoder
                sendArcFeed(program_.R, step.bendAngle);
                sendBendStart();
                prevBendDone_ = status_.bendDone;
                prevBendCyc_  = status_.bendCyc;
                phase_ = MachPhase::BendWait;
                addLog(u8"Гибка старт (HARDWARE)");
            } else {
                // Path B/C — timeout-driven completion
                if (!isSim) {
                    // Path C: still command the physical bender and Z sync
                    sendArcFeed(program_.R, step.bendAngle);
                    sendBendStart();
                    prevBendDone_ = status_.bendDone;
                    prevBendCyc_  = status_.bendCyc;
                }

                // Set sim-state targets so SceneAnimator can animate smoothly
                simBendTargetDeg_ = step.bendAngle;
                double arcLen = 3.14159265358979 * program_.R * std::abs(step.bendAngle) / 180.0;
                simBendStartZ_ = simZPos_;
                // 🔧 ИСПРАВЛЕНИЕ: Z подача при гибке
                // Во время гибки каретка подает вперед на arcLen, ролик гибет, труба неподвижна
                simZTarget_    = simZPos_ + arcLen;

                phase_ = MachPhase::SimulatedBendWait;
                delayStart_ = std::chrono::steady_clock::now();
                double speed = (simBendSpeed_ > 1.0) ? simBendSpeed_ : 60.0;
                delayMs_ = (int)((std::abs(step.bendAngle) / speed) * 1000.0);
                // In hardware-ignore-encoder mode give physical motion a small
                // safety margin since Teensy's bend speed may be slower than
                // the animation speed.
                if (!isSim) delayMs_ += 500;
                if (delayMs_ < 50) delayMs_ = 50;
                addLog(std::string(isSim ? u8"Гибка старт (SIMULATION, "
                                         : u8"Гибка старт (HW ignore-encoder, ")
                        + std::to_string(delayMs_) + u8"ms)");
            }
        }
        break;

    case MachPhase::SimulatedBendWait: // ТОЛЬКО для SIMULATION режима
        {
            auto now = std::chrono::steady_clock::now();
            auto diffMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - delayStart_).count();
            auto& step = program_.steps[curStep_];
            
            if (diffMs >= delayMs_) {
                // Timeout elapsed → treat bend as complete.
                status_.bendCur = (int)std::round(step.bendAngle);
                simBendPos_ = step.bendAngle;

                double arcLen = 3.14159265358979 * program_.R * std::abs(step.bendAngle) / 180.0;
                feedAccum_ += arcLen;

                // If we are actually on hardware (ignoreBendEncoder_ path),
                // stop the Teensy ARCFEED sync that was started in BendStart.
                if (!simulated_) {
                    sendArcStop();
                }

                Logger::log(std::string("[SIMULATED_BENDWAIT_DONE] bendAngle=") + std::to_string(step.bendAngle) +
                           " arcLen=" + std::to_string(arcLen) +
                           " simZPos=" + std::to_string(simZPos_) +
                           " simZTarget=" + std::to_string(simZTarget_) +
                           " feedAccum=" + std::to_string(feedAccum_));

                // ✅ Переход к UnclampWait2 (разжатие)
                phase_ = MachPhase::UnclampWait2;
                phaseStarted_ = false;
                sendUnclamp();

                addLog(u8"BendWait завершена (Z+B синхронно)");
            } else {
                // В этой фазе interpolZAndB() уже движет Z и B!
                // Здесь просто ждем завершения гибки
            }
        }
        break;

    case MachPhase::BendWait: // П.5: Ожидание гибки (ТОЛЬКО HARDWARE режим!)
        {
            // ✅ HARDWARE: Teensy управляет гибкой и Z через ARCFEED
            // Просто ждем пока гибка завершится (bendCyc: 1→2)
            if (status_.bendCyc == 2 && prevBendCyc_ != 2) {
                Logger::log(std::string("[BENDWAIT_HARDWARE_DONE] bendCur=") + std::to_string(status_.bendCur));
                
                // Считаем дугу которую потребила труба
                double arcLen = 3.14159265358979 * program_.R * std::abs(status_.bendCur) / 180.0;
                feedAccum_ += arcLen;
                
                Logger::log(std::string("[BEND_DONE_HARDWARE] arcLen=") + std::to_string(arcLen) +
                           " feedAccum=" + std::to_string(feedAccum_));
                
                sendArcStop();  // Stop synchronized Z feed
                
                // ✅ ФИКСИРОВАНО: Пропускаем Clearance - переходим прямо в UnclampWait2!
                // Clearance фаза полностью удалена (была избыточной подачей)
                phase_ = MachPhase::UnclampWait2;
                phaseStarted_ = false;
                
                Logger::log(std::string("[LASER_CUT_DONE] feedAccum=") + std::to_string(feedAccum_));
                addLog(u8"Лазерная резка завершена, переходим к разжиму");
            }
        }
        break;

    case MachPhase::UnclampWait2: // П.6: Разжим после гибки
        if (!phaseStarted_) {
            delayStart_ = std::chrono::steady_clock::now();
            delayMs_ = unclampDelayMs_;
            if (!ignoreBendEncoder_) {
                sendUnclamp();  // Отправить команду UNCLAMP в Teensy (HARDWARE)
            }
            phaseStarted_ = true;
            addLog(u8"[UNCLAMP] Разжим 2");
        }
        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - delayStart_).count() >= delayMs_) {
            // ✅ Переходим в Clearance - подача трубы вперед на clampLength
            // Это КРИТИЧНО чтобы ролик при возврате не уперся в согнутое колено!
            feedAccum_ += clampLength_;
            simZTarget_ = feedAccum_;
            moveZ(feedAccum_, feedSpeed_);
            
            Logger::log(std::string("[CLEARANCE_START] feedAccum=") + std::to_string(feedAccum_));
            
            phase_ = MachPhase::Clearance;
            phaseStarted_ = false;
        }
        break;

    case MachPhase::Clearance: // П.7: Подача на длину прижима (толкание трубы)
        if (!phaseStarted_) {
            phaseStarted_ = true;
            stallArmed_ = false;
            addLog(u8"Clearance: подача трубы вперед");
        }
        // Hardware stall watchdog for Z clearance (push phase)
        if (!simulated_ && !axisCmdPending_ && status_.zSt == 2) {
            const double zNow = zAxisPos(status_);
            const auto now = std::chrono::steady_clock::now();
            if (!stallArmed_) {
                stallLastPos_ = zNow;
                stallLastProgressAt_ = now;
                stallArmed_ = true;
            } else if (std::abs(zNow - stallLastPos_) > STALL_Z_EPS) {
                stallLastPos_ = zNow;
                stallLastProgressAt_ = now;
            } else {
                auto stalledMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - stallLastProgressAt_).count();
                if (stalledMs > STALL_TIMEOUT_MS) {
                    std::ostringstream oss;
                    oss << "Z stalled during clearance: no progress for " << stalledMs
                        << " ms (zSt=" << status_.zSt
                        << ", z=" << zNow
                        << ", target=" << feedAccum_
                        << ", almZ=" << (status_.almZ ? 1 : 0) << ")";
                    lastError_ = oss.str();
                    addLog("!!! " + lastError_);
                    stop();
                    phase_ = MachPhase::Error;
                    stallArmed_ = false;
                    break;
                }
            }
        } else if (status_.zSt != 2) {
            stallArmed_ = false;
        }
        
        if (simulated_) {
            double zDiff = std::abs(simZTarget_ - simZPos_);
            if (zDiff <= 0.1) {  // Z достигла целевой позиции
                simZPos_ = simZTarget_;
                phase_ = MachPhase::ReturnBend;
                phaseStarted_ = false;
            }
        } else {
            if (status_.zSt == 0 && !axisCmdPending_) {
                Logger::log(std::string("[CLEARANCE_DONE_HW] zSt=") + std::to_string(status_.zSt));
                phase_ = MachPhase::ReturnBend;
                phaseStarted_ = false;
            }
        }
        if (status_.zSt == 3) { lastError_ = "Z axis error"; phase_ = MachPhase::Error; }
        break;

    case MachPhase::ReturnBend: // П.8: Возврат ролика
        if (!phaseStarted_) {
            auto& step = program_.steps[curStep_];
            sendBendTarget(0);
            if (!ignoreBendEncoder_) {
                sendBendStart();
            } else {
                delayStart_ = std::chrono::steady_clock::now();
                double retSpeed = (simBendSpeed_ > 1.0) ? simBendSpeed_ * 2.0 : 120.0;
                delayMs_ = (int)((std::abs(step.bendAngle) / retSpeed) * 1000.0);
                if (delayMs_ < 50) delayMs_ = 50;
            }
            prevBendCyc_ = status_.bendCyc;
            phase_ = MachPhase::ReturnBendWait;
            phaseStarted_ = true;
        }
        break;

    case MachPhase::ReturnBendWait:
        {
            auto& step = program_.steps[curStep_];
            bool done = false;
            
            if (ignoreBendEncoder_) {
                auto now = std::chrono::steady_clock::now();
                auto diffMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - delayStart_).count();
                // ⚠️ ИСПРАВЛЕНИЕ v2.5: Линейная имитация энкодера - энкодер плавно возвращается к нулю
                if (diffMs >= delayMs_) {
                    status_.bendCur = 0;
                    simBendPos_ = 0.0;
                    done = true;
                    Logger::log(std::string("[BEND_SIM] ReturnBend complete: bendCur=0 at ") + 
                               std::to_string(diffMs) + "ms");
                } else {
                    // Линейная интерполяция: от bendAngle к 0
                    double progress = (double)diffMs / (double)delayMs_;
                    double bendCurDouble = step.bendAngle * (1.0 - progress);
                    status_.bendCur = (int)bendCurDouble;
                    simBendPos_ = bendCurDouble;  // Синхронизируем симуляцию
                }
            } else if (status_.bendCyc == 1 && prevBendCyc_ != 1) { 
                done = true; 
            }
            
            if (done) {
                Logger::log(std::string("[MACHINE] ReturnBendWait DONE: bendCur=") + 
                           std::to_string(status_.bendCur) + ", curStep_=" + std::to_string(curStep_) + 
                           ", SKIP ReturnClearance (no backward movement allowed!)");
                // ✅ FIXED: НЕ ВОЗВРАЩАЕМ КАРЕТКУ НАЗАД! Это вызывает заклинивание!
                // Переходим прямо к StepDone (готово к следующему шагу)
                phase_ = MachPhase::StepDone;
                phaseStarted_ = false;
                stallArmed_ = false;
            }
        }
        break;

    case MachPhase::StepDone: // П.9: Переход
        {
            int oldStep = curStep_;
            curStep_++;
            updateBaseAccumulators();  // ⚠️ КРИТИЧЕСКИЙ ФИХ: Кэшировать базы для НОВОГО шага
            Logger::log(std::string("[MACHINE] StepDone: curStep_ ") + std::to_string(oldStep) + 
                       " -> " + std::to_string(curStep_) + 
                       ", zMm=" + std::to_string(status_.zMm) +
                       ", bendCur=" + std::to_string(status_.bendCur));
            if (curStep_ >= totalSteps_) {
                Logger::log(std::string("[MACHINE] Program complete at curStep_=") + std::to_string(curStep_));
                // Disable servos at end of program
                if (serial_.isOpen()) serial_.send("SON 0");
                addLog("> SON 0 (disable servos at program end)");
                phase_ = MachPhase::ProgramDone;
                sendStep(0, 0);
                addLog("=== Program complete ===");
            } else {
                Logger::log(std::string("[MACHINE] Moving to UnclampWait1, new step=") + std::to_string(curStep_));
                phase_ = MachPhase::UnclampWait1;
                phaseStarted_ = false;
            }
        }
        break;

    default:
        break;
}
}

// ─── Simulation Backend ────────────────────────────────────────────────

void Machine::updateSimulation(float dt) {
    // Phase-dependent interpolation: ONLY move axes that should move in CURRENT phase
    static int updateSimCallCount = 0;
    if (++updateSimCallCount % 30 == 0) {  // Log every 30th call
        Logger::log(std::string("[UPDATE_SIM] Call #") + std::to_string(updateSimCallCount) +
                   " dt=" + std::to_string(dt) +
                   " phase=" + std::to_string((int)phase_) +
                   " simZ=" + std::to_string(simZPos_) +
                   " target=" + std::to_string(simZTarget_));
    }
    
    switch (phase_) {
    
    case MachPhase::Feed:
        interpolZOnly(dt);  // ONLY Z moves forward, C and B frozen
        break;
        
    case MachPhase::Rotation:
        interpolCOnly(dt);  // ONLY C rotates, Z and B frozen
        break;
        
    case MachPhase::BendStart:
        interpolBOnly(dt);  // ONLY B starts bending, Z and C frozen
        break;
        
    case MachPhase::SimulatedBendWait:
        interpolZAndB(dt);  // 🔴 CRITICAL: Z and B move SIMULTANEOUSLY!
        break;
        
    case MachPhase::Clearance:
        interpolZOnly(dt);  // ONLY Z moves forward (clearance for bend roller)
        break;
        
    case MachPhase::ReturnBend:
        interpolBReturn(dt);  // ONLY B returns to 0, Z and C frozen
        break;
        
    case MachPhase::ReturnBendWait:
        // If using encoder bypass, continue interpolating B return (Z frozen)
        if (ignoreBendEncoder_) {
            interpolBReturn(dt);
        }
        break;
        
    default:
        // Other phases don't require interpolation
        break;
    }
    
    // Update status with interpolated positions
    generateSimStatus();
}

// ─── Interpolation Methods (Phase-Dependent) ──────────────────────────

void Machine::interpolZOnly(float dt) {
    // Z moves toward target, C and B frozen
    double diff = simZTarget_ - simZPos_;
    double maxMove = simZSpeed_ * dt;
    
    if (diff > maxMove) {
        simZPos_ += maxMove;
    } else if (diff > 0) {
        simZPos_ = simZTarget_;
    }
    // If diff <= 0, Z doesn't move (already reached target)
}

void Machine::interpolCOnly(float dt) {
    // C rotates toward target, Z and B frozen
    double diff = simCTarget_ - simCPos_;
    double maxMove = simCSpeed_ * dt;
    
    if (std::abs(diff) > maxMove) {
        simCPos_ += (diff > 0 ? 1.0 : -1.0) * maxMove;
    } else if (std::abs(diff) > 0.01) {
        simCPos_ = simCTarget_;
    }
    // If diff ≈ 0, C doesn't move (already reached target)
}

void Machine::interpolBOnly(float dt) {
    // B starts bending, Z and C frozen
    double maxMove = simBendSpeed_ * dt;
    
    if (simBendPos_ < simBendTargetDeg_) {
        simBendPos_ += maxMove;
        if (simBendPos_ >= simBendTargetDeg_) {
            simBendPos_ = simBendTargetDeg_;
        }
    }
}

void Machine::interpolZAndB(float dt) {
    // 🔴 CRITICAL: Z and B must move in LOCK-STEP during the bend —
    // otherwise zAbs and the arc-length implied by bAbs drift apart
    // and the 3D tube jitters during every bend.
    //
    // Drive B by its own speed, then derive Z purely from bend progress:
    //     zAbs = bendStartZ + (bAbs / targetB) * (zTarget - bendStartZ)
    // so that whenever the caller reads (zAbs, bAbs) they are always
    // consistent with arcCurrent = π·R·bAbs/180.

    // B advances by its configured speed.
    if (simBendTargetDeg_ > 0) {
        double maxMove = simBendSpeed_ * dt;
        if (simBendPos_ < simBendTargetDeg_) {
            simBendPos_ += maxMove;
            if (simBendPos_ >= simBendTargetDeg_) {
                simBendPos_ = simBendTargetDeg_;
            }
        }
    }

    // Z is slaved to B's progress — no independent speed, no drift.
    if (simBendTargetDeg_ > 0) {
        const double frac = std::clamp(simBendPos_ / (double)simBendTargetDeg_, 0.0, 1.0);
        simZPos_ = simBendStartZ_ + (simZTarget_ - simBendStartZ_) * frac;
    }
}

void Machine::interpolBReturn(float dt) {
    // B returns to 0, Z and C frozen
    double maxMove = simBendSpeed_ * 1.5 * dt;  // Return faster than bend forward
    
    if (simBendPos_ > 0.0) {
        simBendPos_ -= maxMove;
        if (simBendPos_ < 0.0) {
            simBendPos_ = 0.0;
        }
    }
}

void Machine::interpolZReturn(float dt) {
    // Z returns to 0, B and C frozen
    double maxMove = simZSpeed_ * dt;
    
    if (simZPos_ > 0.0) {
        simZPos_ -= maxMove;
        if (simZPos_ < 0.0) {
            simZPos_ = 0.0;
        }
    }
}

void Machine::startSimulation() {
    simulated_ = true;
    simZPos_ = 0.0;
    simCPos_ = 0.0;
    simBendPos_ = 0.0;
    simZTarget_ = 0.0;
    simCTarget_ = 0.0;
    simBendTargetDeg_ = 0;
    simBendCycle_ = 1;
    simBendRunning_ = false;
    simLastTime_ = std::chrono::steady_clock::now();
    // Generate initial valid status
    generateSimStatus();
    addLog("=== Simulation mode started ===");
}

void Machine::stopSimulation() {
    simulated_ = false;
    phase_ = MachPhase::Idle;
    
    // ⚠️ КРИТИЧЕСКИЙ FIX: Когда выходим из SIMULATION в HW режим,
    // ДОЛЖНЫ очистить status_ чтобы не использовать stale/old values!
    // При этом сохраняем valid=true но обнуляем positions,
    // чтобы первый JSON от Teensy с реальным encoder overwrite это
    status_.zMm = 0.0;
    status_.cDeg = 0.0;
    status_.zFbMm = 0.0;  // Это критично! Иначе getSnapshot() будет читать старое значение
    status_.cFbDeg = 0.0;
    status_.encValid = false;  // Дождаться первого реального JSON от Teensy
    status_.zSt = 0;
    status_.cSt = 0;
    
    Logger::log("[SIMULATION_EXIT] status_ reset for HW mode transition: zFbMm=0, cFbDeg=0, encValid=false");
    addLog("=== Simulation mode stopped ===");
}

void Machine::generateSimStatus() {
    status_.valid = true;
    status_.version = "SIM";
    status_.zMm = simZPos_;
    status_.zSon = true;
    status_.cDeg = simCPos_;
    status_.cSon = true;
    status_.encValid = true;
    status_.zFbMm = simZPos_;
    status_.cFbDeg = simCPos_;
    status_.zFbCount = (long)(simZPos_ * 581.66);  // approx ticks
    status_.cFbCount = (long)(simCPos_ * 266.67);
    
    // ⚠️ ДИАГНОСТИКА: Логируем каждое обновление статуса
    static int genStatusCallCount = 0;
    static double lastLoggedZMm = -999.0;
    genStatusCallCount++;
    
    if (phase_ == MachPhase::BendWait || phase_ == MachPhase::SimulatedBendWait || 
        phase_ == MachPhase::ReturnBendWait) {
        if (genStatusCallCount % 3 == 0 || std::abs(status_.zMm - lastLoggedZMm) > 0.5) {
            Logger::log(std::string("[GENSTAT] call=") + std::to_string(genStatusCallCount) +
                       " phase=BEND zMm=" + std::to_string(status_.zMm) +
                       " cDeg=" + std::to_string(status_.cDeg) +
                       " simZPos=" + std::to_string(simZPos_) +
                       " simCPos=" + std::to_string(simCPos_));
            lastLoggedZMm = status_.zMm;
        }
    }
    
    if (!ignoreBendEncoder_) {
        status_.bendCur = (int)std::round(simBendPos_);
        status_.bendTgt = simBendTargetDeg_;
        status_.bendCyc = simBendCycle_;
        status_.bendDone = !simBendRunning_ ? 1 : 0;
    }
    status_.bendMode = 1;
    status_.modeAuto = true;
    status_.pedalFwd = true; // simulation: pedal always pressed
    status_.pedalRev = false;

    // Z axis status
    double zDiff = std::abs(simZTarget_ - simZPos_);
    int newZSt = (zDiff > 0.05) ? 2 : 0;
    if (newZSt != status_.zSt && phase_ == MachPhase::Feed) {
        Logger::log(std::string("[Z_STATUS_CHANGE] zSt: ") + std::to_string(status_.zSt) + 
                   " -> " + std::to_string(newZSt) + 
                   " zDiff=" + std::to_string(zDiff) + 
                   " simZPos=" + std::to_string(simZPos_) + 
                   " simZTarget=" + std::to_string(simZTarget_));
    }
    status_.zSt = newZSt;

    // C axis status
    double cDiff = std::abs(simCTarget_ - simCPos_);
    status_.cSt = (cDiff > 0.2) ? 2 : 0;
}

// ⚠️ КРИТИЧЕСКИЙ ФИХ: Вычислить и КЭШИРОВАТЬ базовые координаты для текущего шага
// Вызывать ОДИН РАЗ при входе в новый шаг, НЕ каждый кадр!
void Machine::updateBaseAccumulators() {
    if (curStep_ == lastCachedStepIdx_) {
        return;  // Уже кэширован для этого шага, не пересчитывать
    }
    
    baseFeedAccCurrentStep_ = 0.0;
    baseRotAccCurrentStep_ = 0.0;
    
    // Суммируем ВСЕ предыдущие шаги
    for (int i = 0; i < curStep_; ++i) {
        const auto& s = program_.steps[i];
        baseFeedAccCurrentStep_ += s.feedLength;                      // Feed
        baseFeedAccCurrentStep_ += s.arcLength(program_.R);           // Bend
        if (std::abs(s.bendAngle) > 0.01)
            baseFeedAccCurrentStep_ += clampLength_;                  // Clearance
        baseRotAccCurrentStep_ += s.rotation;                         // Rotation
    }
    
    lastCachedStepIdx_ = curStep_;
    Logger::log(std::string("[BASE_ACCUM] Updated for step ") + std::to_string(curStep_) + 
               " | baseFeedAcc=" + std::to_string(baseFeedAccCurrentStep_) +
               " | baseRotAcc=" + std::to_string(baseRotAccCurrentStep_));
}

void Machine::addLog(const std::string& msg) {
    if (log_.size() > 500) log_.erase(log_.begin());
    log_.push_back(msg);
    Logger::log(std::string("[MACHINE] ") + msg);
}

// ─── Relay Control ────────────────────────────────────────────────────

bool Machine::canActivateRelay(int pinNum) const {
    // Взаимоисключение: не может быть двух активных реле одновременно
    if (pinNum == 35) {
        // PIN 35 (CLAMP) — проверяем что PIN 36 неактивен
        return !relayState_.unclampActive && !unclampPulseActive_;
    } else if (pinNum == 36) {
        // PIN 36 (UNCLAMP) — проверяем что PIN 35 неактивен
        return !relayState_.clampActive && !clampPulseActive_;
    }
    return true;
}

void Machine::updateRelayStatus() {
    // ⚠️ КРИТИЧЕСКИЙ ФИХ v2.5: Обновлять relayState_ на основе РЕАЛЬНОГО статуса от Teensy!
    // PRIMARY сигнал - это JSON от Teensy (status_.clampClosed), а не локальный таймер
    if (status_.valid) {
        bool newClampActive = status_.clampClosed;
        bool newUnclampActive = status_.unclampOpen;

        // Синхронизировать LOCAL флаги с РЕАЛЬНЫМ состоянием от Teensy
        relayState_.clampActive = newClampActive;
        relayState_.unclampActive = newUnclampActive;

        // ДИАГНОСТИКА: если есть разрыв между ожидаемым и реальным
        static bool prevClampActive = false;
        static bool prevUnclampActive = false;
        static int feedbackDiagCount = 0;

        if (newClampActive != prevClampActive || newUnclampActive != prevUnclampActive ||
            ++feedbackDiagCount % 500 == 0) {
            if (newClampActive != prevClampActive || newUnclampActive != prevUnclampActive) {
                addLog(std::string("[RELAY_FEEDBACK_CHANGE] clampActive: ") +
                       (prevClampActive ? "1" : "0") + " -> " + (newClampActive ? "1" : "0") +
                       ", unclampActive: " + (prevUnclampActive ? "1" : "0") + " -> " +
                       (newUnclampActive ? "1" : "0"));
            }
            if (feedbackDiagCount % 500 == 0) {
                addLog(std::string("[RELAY_STATUS_PERIODIC] clampActive=") +
                       (newClampActive ? "1" : "0") + ", unclampActive=" +
                       (newUnclampActive ? "1" : "0"));
            }
            prevClampActive = newClampActive;
            prevUnclampActive = newUnclampActive;
        }
    } else {
        // Если Teensy не отправляет статус, использовать локальные импульсные флаги (fallback)
        // Отслеживание импульса CLAMP (PIN 35)
        if (clampPulseActive_) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - clampPulseStart_).count();
            if (elapsed >= RelayState::PULSE_DURATION_MS) {
                clampPulseActive_ = false;
                relayState_.clampActive = false;
                addLog("Relay CLAMP pulse completed (1.5s)");
            }
        }

        // Отслеживание импульса UNCLAMP (PIN 36)
        if (unclampPulseActive_) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - unclampPulseStart_).count();
            if (elapsed >= RelayState::PULSE_DURATION_MS) {
                unclampPulseActive_ = false;
                relayState_.unclampActive = false;
                addLog("Relay UNCLAMP pulse completed (1.5s)");
            }
        }
    }
}










