// ─────────────────────────────────────────────────────────────────────
// app.cpp — Application orchestrator
// ─────────────────────────────────────────────────────────────────────
#include "app.h"
#include "step_import.h"
#include "step_export.h"
#include "tube_geometry.h"
#include "stl_loader.h"
#include "logger.h"

#include <fstream>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <cfloat>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// OCCT IGES export
#include <IGESControl_Writer.hxx>
#include <Interface_Static.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>

#ifdef _WIN32
#include <windows.h>
#include <gdiplus.h>
#pragma comment(lib, "gdiplus.lib")
#endif

// Point0 — bend point position (dynamic, follows bend radius R)
// (-(float)m_program.R) = -(float)R, PT0_Y = 0, PT0_Z = 397 (from STP bend point model)
static constexpr float PT0_Y =   0.0f;
static constexpr float PT0_Z = 397.0f;

namespace fs = std::filesystem;

// ─── Helpers ────────────────────────────────────────────────────────
static fs::path getExeDirPath() {
#ifdef _WIN32
    wchar_t buf[MAX_PATH];
    GetModuleFileNameW(nullptr, buf, MAX_PATH);
    return fs::path(buf).parent_path();
#else
    return fs::current_path();
#endif
}

// ─── Init / Shutdown ────────────────────────────────────────────────
bool App::init(int width, int height, GLFWwindow* window) {
    m_window = window;
    m_exeDir = getExeDirPath();
    m_lastFrameTime = std::chrono::steady_clock::now();  // Initialize frame timer
    Logger::log("[APP] init() called");

    // Bender model is now loaded from bender_models/ in loadBenderModel()
    // (no more STL candidate search needed)

    m_renderer.init(width, height);

    // Show splash screen while heavy models are loading
    loadSplashImage();
    renderSplash(u8"Инициализация...", 0.0f);

    m_ui.init(m_exeDir.u8string());

    // Set up callbacks
    m_callbacks.onImportSTP       = [this](const std::string& p) { importSTP(p); };
    m_callbacks.onExportSTP       = [this](const std::string& p) { exportSTP(p); };
    m_callbacks.onSaveJSON        = [this](const std::string& p) { saveJSON(p); };
    m_callbacks.onLoadJSON        = [this](const std::string& p) { loadJSON(p); };
    m_callbacks.onRebuildGeometry = [this]()                      { rebuildGeometry(); };
    m_callbacks.onVisualize       = [this]()                      {
        Logger::log("[UI] SIM RUN");
        // v2.5: unified execution via Machine (simulation mode), no legacy App timer path.
        m_animating = false;
        m_laserAnimating = false;
        m_renderer.setShowLaserBeam(false);
        m_renderer.clearCutContour();

        if (m_machine.isProgramRunning()) {
            m_machine.stopProgram();
        }
        if (!m_machine.isSimulated()) {
            m_machine.startSimulation();
        }

        m_ui.syncActiveProject(m_program);
        m_machine.startProgram(m_program, m_program.clampLength, m_ui.activeProjectName());
        m_resumeAvailable = false;
        m_resumeIsLaser = false;
        m_pendingNextPart = false;
        m_hasIdleVisualState = false;
        m_ui.infoText = u8"Симуляция запущена";
    };
    m_callbacks.onLaserVisualize  = [this]()                      {
        Logger::log("[UI] SIM RUN (laser mode via Machine)");
        // v2.5: laser mode is also executed by Machine state machine in simulation mode.
        m_animating = false;
        m_laserAnimating = false;
        m_renderer.setShowLaserBeam(false);
        m_renderer.clearCutContour();

        if (m_machine.isProgramRunning()) {
            m_machine.stopProgram();
        }
        if (!m_machine.isSimulated()) {
            m_machine.startSimulation();
        }

        m_ui.syncActiveProject(m_program);
        m_machine.startProgram(m_program, m_program.clampLength, m_ui.activeProjectName());
        m_resumeAvailable = false;
        m_resumeIsLaser = false;
        m_pendingNextPart = false;
        m_hasIdleVisualState = false;
        m_ui.infoText = u8"Симуляция (лазер) запущена";
    };
    m_callbacks.onExportIGES      = [this](const std::string& p) { exportIGES(p); };
    m_callbacks.onCheckCollisions = [this]()                      { if (m_ui.useLogic2) checkCollisionsV2(); else checkCollisions(); };
    m_callbacks.onAnimContinue    = [this]()                      { continueAnimation(); };
    m_callbacks.onStop            = [this]()                      { stopAnimation(); };
    m_callbacks.onCancel          = [this]()                      { cancelAnimation(); };

    // Machine callbacks
    m_callbacks.onMachineConnect    = [this](const std::string& port) {
        Logger::log(std::string("[UI] Machine connect: ") + port);
        clearStoppedCycleSnapshot();
        m_machineSceneValid = false;
        m_machineScenePhase = MachPhase::Idle;
        m_machineSceneStep = -1;
        m_machine.connect(port);
        m_ui.infoText = u8"Станок подключён";
    };
    m_callbacks.onMachineDisconnect = [this]()    {
        Logger::log("[UI] Machine disconnect");
        m_machine.disconnect();
        clearStoppedCycleSnapshot();
        m_machineSceneValid = false;
        m_machineScenePhase = MachPhase::Idle;
        m_machineSceneStep = -1;
        m_ui.infoText = u8"Станок отключён";
    };
    m_callbacks.onMachineRun        = [this]()    {
        Logger::log("[UI] Machine RUN");
        m_animating = false;
        m_laserAnimating = false;
        m_renderer.setShowLaserBeam(false);
        m_renderer.clearCutContour();
        if (m_machine.isSimulated()) {
            m_machine.stopSimulation();
        }
        m_ui.syncActiveProject(m_program);
        m_machine.startProgram(m_program, m_program.clampLength, m_ui.activeProjectName());
    };
    m_callbacks.onMachineStop       = [this]()    { Logger::log("[UI] Machine STOP"); m_machine.stopProgram(); };
    m_callbacks.onMachineHome       = [this]()    { Logger::log("[UI] Machine HOME Z"); m_machine.homeZ(); };
    m_callbacks.onMachineHomeC      = [this]()    { Logger::log("[UI] Machine HOME C"); m_machine.homeC(); };
    m_callbacks.onMachineServoOn    = [this]()    { Logger::log("[UI] Machine SERVO ON"); m_machine.servoOn(); };
    m_callbacks.onMachineServoOff   = [this]()    { Logger::log("[UI] Machine SERVO OFF"); m_machine.servoOff(); };
    m_callbacks.onMachineEstop      = [this]()    { Logger::log("[UI] Machine ESTOP"); m_machine.estop(); };
    m_callbacks.onMachineZero       = [this]()    { Logger::log("[UI] Machine ZERO ENCODER"); m_machine.sendZero(); };
    m_callbacks.onMachineCalibrate  = [this](double zMm, double cDeg, int bendDeg) {
        Logger::log(std::string("[UI] Machine CALIBRATE Z=") + std::to_string(zMm) + 
                    " C=" + std::to_string(cDeg) + " B=" + std::to_string(bendDeg));
        m_machine.sendCalibration(zMm, cDeg, bendDeg);
    };

    // Give UI a pointer to Machine and App for status display and animations
    m_ui.setMachine(&m_machine);
    m_ui.setApp(this);

    renderSplash(u8"Загрузка точки гибки...", 0.10f);
    loadBenderModel();

    renderSplash(u8"Загрузка моделей станка...", 0.25f);
    loadLogic2Models();

    renderSplash(u8"Подготовка сцены...", 0.95f);

    // Populate settings with actually loaded model paths
    if (!m_bendPointPath.empty())
        m_ui.settings.benderModelPath = m_bendPointPath.u8string();
    if (!m_rollerPath.empty())
        m_ui.settings.rollerModelPath = m_rollerPath.u8string();
    if (!m_clampPath.empty())
        m_ui.settings.clampModelPath = m_clampPath.u8string();
    if (!m_staticPath.empty())
        m_ui.settings.staticModelPath = m_staticPath.u8string();
    if (!m_carriagePath.empty())
        m_ui.settings.carriageModelPath = m_carriagePath.u8string();

    // Default tube params
    m_program.D = 38;
    m_program.R = 76;
    m_program.clampLength = 60;

    // Build default test geometry so the 3D scene is not empty
    m_program.steps.clear();
    BendStep s1; s1.id = 1; s1.feedLength = 100; s1.rotation = 0;  s1.bendAngle = 90;
    BendStep s2; s2.id = 2; s2.feedLength = 200; s2.rotation = 0;  s2.bendAngle = 0;
    m_program.steps.push_back(s1);
    m_program.steps.push_back(s2);
    rebuildGeometry();

    renderSplash(u8"Готово!", 1.0f);
    destroySplashImage();

    return true;
}

void App::shutdown() {
    Logger::log("[APP] shutdown() called");
    m_renderer.shutdown();
}

double App::currentSceneWhipLength() const {
    return (m_visualWhipLength > 0.01) ? m_visualWhipLength : m_ui.whipLength;
}

void App::prepareAnimationProgram() {
    m_animationProgram = m_program;
    m_animationProgram.deriveCuts();
    m_animationStepMap.clear();
    m_animationStepMap.reserve(m_animationProgram.steps.size());

    std::vector<BendStep> bendSteps;
    bendSteps.reserve(m_animationProgram.steps.size());
    for (int i = 0; i < (int)m_animationProgram.steps.size(); ++i) {
        if (m_animationProgram.steps[i].isCut)
            continue;
        m_animationStepMap.push_back(i);
        bendSteps.push_back(m_animationProgram.steps[i]);
    }
    m_animationProgram.steps = std::move(bendSteps);
}

int App::mapAnimatedStepIndex(int stepIdx) const {
    if (stepIdx < 0 || stepIdx >= (int)m_animationStepMap.size())
        return stepIdx;
    return m_animationStepMap[stepIdx];
}

double App::computeNormalCarriageFeed() const {
    const auto& program = m_animationProgram;
    auto arcLenFor = [&](const BendStep& s) -> double {
        if (std::abs(s.bendAngle) <= 0.01) return 0.0;
        return 3.14159265358979323846 * program.R * std::abs(s.bendAngle) / 180.0;
    };

    double fed = 0.0;
    for (int i = 0; i < m_animStep && i < (int)program.steps.size(); i++) {
        fed += program.steps[i].feedLength;
        fed += arcLenFor(program.steps[i]);
        if (program.steps[i].bendAngle > 0.01)
            fed += program.clampLength;
    }
    if (m_animStep < (int)program.steps.size()) {
        const auto& curStep = program.steps[m_animStep];
        const double curArc = arcLenFor(curStep);
        switch (m_animPhase) {
        case AnimFeed: {
            double alreadyFed = (m_animStep > 0 && program.steps[m_animStep - 1].bendAngle > 0.01)
                ? program.clampLength : 0.0;
            fed += alreadyFed + (curStep.feedLength - alreadyFed) * m_animProgress;
            break;
        }
        case AnimRotation:
        case AnimClampClose:
            fed += curStep.feedLength;
            break;
        case AnimBend:
            fed += curStep.feedLength + curArc * m_animProgress;
            break;
        case AnimClampOpen:
            fed += curStep.feedLength + curArc;
            break;
        case AnimTubePush:
            fed += curStep.feedLength + curArc + program.clampLength * m_animProgress;
            break;
        case AnimBenderReturn:
            fed += curStep.feedLength + curArc + ((curStep.bendAngle > 0.01) ? program.clampLength : 0.0);
            break;
        }
    }
    return fed;
}

double App::computeAbsoluteCarriageFeed() const {
    if (m_laserAnimating)
        return m_cycleBaseCarriageFeed + m_laserTotalFed;
    if (m_animating)
        return m_cycleBaseCarriageFeed + computeNormalCarriageFeed();
    return m_idleCarriageFeed;
}

double App::computeCarriageRenderFeed(double logicalFeed) const {
    // TZ v2.5 Absolute Path:
    // Ytarget = WhipLength - Zabs
    // OffsetY = Ytarget - Ybase, where CAD carriage base is Y=6000.
    static constexpr double CAD_CARRIAGE_Y_BASE = 6000.0;
    // Renderer applies final Y translation as worldY += -m_carriageYOffset,
    // so we pass inverted offset to preserve physical direction.
    static constexpr double RENDERER_CARRIAGE_SIGN = -1.0;
    const double yTarget = currentSceneWhipLength() - std::max(0.0, logicalFeed);
    const double offsetModel = yTarget - CAD_CARRIAGE_Y_BASE;
    return RENDERER_CARRIAGE_SIGN * offsetModel;
}

double App::computeCurrentChuckRotation() const {
    if (m_laserAnimating) {
        if (m_laserAnimPhase == LCutBefore || m_laserAnimPhase == LCutAfter)
            return m_laserCutRotAngle;

        double rot = 0.0;
        for (int i = 0; i < m_laserAnimStep && i < (int)m_animationProgram.steps.size(); i++)
            rot += m_animationProgram.steps[i].rotation;
        if (m_laserAnimStep < (int)m_animationProgram.steps.size() && m_laserAnimPhase == LRotation)
            rot += m_animationProgram.steps[m_laserAnimStep].rotation * m_laserAnimProgress;
        return rot;
    }

    if (m_animating) {
        double rot = 0.0;
        for (int i = 0; i < m_animStep && i < (int)m_animationProgram.steps.size(); i++)
            rot += m_animationProgram.steps[i].rotation;
        if (m_animStep < (int)m_animationProgram.steps.size() && m_animPhase == AnimRotation)
            rot += m_animationProgram.steps[m_animStep].rotation * m_animProgress;
        return rot;
    }

    return m_idleChuckRotation;
}

void App::captureIdleVisualState() {
    m_hasIdleVisualState = true;

    // v2.5 unified runtime: snapshot from current machine telemetry first.
    const TeensyStatus& st = m_machine.status();
    if (st.valid) {
        m_idleCarriageFeed = machineFeedPosition(st);
        m_idleChuckRotation = machineChuckPosition(st);
        // Telemetry runtime reconstructs tube geometry directly.
        // Do not apply legacy rigid tube transform in idle snapshot.
        m_idleTubeTransform = false;
        m_idleTubeRotY = 0.0;
        m_idleTubeFeedY = 0.0;
        return;
    }

    m_idleCarriageFeed = computeAbsoluteCarriageFeed();
    m_idleChuckRotation = computeCurrentChuckRotation();
    m_idleTubeTransform = false;
    m_idleTubeRotY = 0.0;
    m_idleTubeFeedY = 0.0;

    if (m_laserAnimating) {
        switch (m_laserAnimPhase) {
        case LCutBefore:
            m_idleTubeTransform = true;
            m_idleTubeRotY = m_laserCutRotAngle;
            m_idleTubeFeedY = -m_laserCutFeedDelta;
            break;
        case LReturnFromCut:
            m_idleTubeTransform = true;
            m_idleTubeFeedY = m_laserCutFeedBase * (1.0 - m_laserAnimProgress);
            break;
        case LCutAfterFeed:
            m_idleTubeTransform = true;
            m_idleTubeFeedY = m_laserCutFeedBase;
            break;
        case LCutAfter:
            m_idleTubeTransform = true;
            m_idleTubeRotY = m_laserCutRotAngle;
            m_idleTubeFeedY = m_laserCutFeedBase + m_laserCutFeedDelta;
            break;
        case LReturnAfterCutAfter:
            m_idleTubeTransform = true;
            m_idleTubeFeedY = m_laserCutFeedBase * (1.0 - m_laserAnimProgress);
            break;
        default:
            break;
        }
        return;
    }

    if (!m_animating) return;

    if (m_animStep < (int)m_animationProgram.steps.size()) {
        switch (m_animPhase) {
        case AnimTubePush:
            m_idleTubeTransform = true;
            m_idleTubeFeedY = m_animationProgram.clampLength * m_animProgress;
            break;
        case AnimBenderReturn:
            m_idleTubeTransform = true;
            m_idleTubeFeedY = m_animationProgram.clampLength;
            break;
        default:
            break;
        }
    }
}

bool App::canContinueWithRemainingStock() const {
    return !m_program.steps.empty() && m_remainingWhipLength + 0.01 >= m_program.lTotal();
}

void App::updateContinueUiState() {
    // v2.5: legacy local animation resume is removed from runtime path.
    // Continue button is intentionally disabled until/if explicit Machine-level resume is introduced.
    m_ui.canContinueCycle = false;
    m_ui.continueButtonLabel = u8"Продолжить";
}

void App::clearStoppedCycleSnapshot() {
    // ТЗ v2.5 раздел 7.2: Cancel/Stop должны сохранить ПОСЛЕДНЕЕ состояние без геометрического отката.
    // Не вызываем rebuildGeometry() - это было бы "сбросом в ноль", что нарушает ТЗ.
    m_resumeAvailable = false;
    m_resumeIsLaser = false;
    m_pendingNextPart = false;
    m_hasIdleVisualState = false;
    m_idleTubeTransform = false;
    m_ui.animActive = false;
    m_ui.animPaused = false;
    m_ui.animPauseMsg.clear();
    m_ui.animShowPopup = false;
    m_ui.animStepIndex = -1;
    m_ui.animProgressFrac = 0.0f;
    m_ui.animPhaseName.clear();
    m_ui.animPhaseType = -1;
}

bool App::isPedalHeld() const {
    if (m_machine.pedalBypass_) return true;
    if (!m_machine.isConnected()) return false;
    const TeensyStatus& st = m_machine.status();
    if (!st.valid) return false;
    return st.pedalFwd;
}

double App::machineFeedPosition(const TeensyStatus& st) const {
    return st.encValid ? st.zFbMm : st.zMm;
}

double App::machineChuckPosition(const TeensyStatus& st) const {
    return st.encValid ? st.cFbDeg : st.cDeg;
}

// ─────────────────────────────────────────────────────────────────────
// syncMachineVisualState — ТОНКАЯ ОБЁРТКА.
// Вся логика 3D визуализации живёт в SceneAnimator. Здесь мы:
//   1. Снимаем единый snapshot состояния станка (getSnapshot).
//   2. Передаём его SceneAnimator'у, который применяет его к Renderer.
//   3. Читаем из аниматора желаемое состояние прижима и проецируем его
//      на m_clampTargetOffset (App владеет интерполяцией клампа).
//   4. Обновляем UI state (шаг / фаза).
// ─────────────────────────────────────────────────────────────────────
void App::syncMachineVisualState(const TeensyStatus& /*st*/) {
    const MachineState snap = m_machine.getSnapshot();
    const BendingProgram& program =
        (!m_machine.runningProgram().steps.empty()) ? m_machine.runningProgram() : m_program;

    // DEBUG: Log snap before apply
    static int syncLogCount = 0;
    if (++syncLogCount % 120 == 0) {  // ~2Hz at 60FPS
        Logger::log(std::string("[SYNC_VISUAL_STATE] isSimulation=") + (snap.isSimulation ? "1" : "0") +
                    " phase=" + std::to_string((int)snap.phaseId) +
                    " step=" + std::to_string(snap.stepIdx) + "/" + std::to_string(snap.totalSteps) +
                    " zAbs=" + std::to_string(snap.zAbs) +
                    " cAbs=" + std::to_string(snap.cAbs) +
                    " bAbs=" + std::to_string(snap.bAbs) +
                    " → calling SceneAnimator.apply()");
    }

    m_sceneAnimator.setRenderer(&m_renderer);
    m_sceneAnimator.apply(snap, program, currentSceneWhipLength());

    // Clamp target: SceneAnimator говорит нам, должен ли прижим быть открыт под
    // гибку или закрыт. App интерполирует m_clampOffset к m_clampTargetOffset.
    if (m_ui.useLogic2) {
        m_clampTargetOffset = (m_sceneAnimator.clampTarget() == SceneAnimator::ClampOpenForBend)
                                  ? 0.0
                                  : -CLAMP_TRAVEL;
    } else {
        // Logic 1: UI-driven реле toggle управляет цельным offset.
        if (m_ui.relayClampToggle_)      m_clampTargetOffset = -CLAMP_TRAVEL;
        else if (m_ui.relayUnclampToggle_) m_clampTargetOffset = 0.0;
    }

    // UI state (шаг/фаза/прогресс)
    const int totalSteps = (int)program.steps.size();
    const int stepIdx = snap.stepIdx;
    m_ui.animActive = snap.programRunning;
    m_ui.animStepIndex = std::clamp(stepIdx, 0, std::max(0, totalSteps - 1));
    m_ui.animPhaseName = std::string(u8"Станок: ") + m_machine.phaseText();
    m_ui.animPhaseType = -1;
    m_ui.animProgressFrac = (snap.phaseId == (int)MachPhase::ProgramDone)
        ? 1.0f
        : std::clamp((float)std::max(stepIdx, 0) / (float)std::max(1, totalSteps), 0.0f, 1.0f);
}

void App::update() {
    static auto s_lastTick = std::chrono::steady_clock::now();
    auto nowTick = std::chrono::steady_clock::now();
    double frameDt = std::chrono::duration<double>(nowTick - s_lastTick).count();
    s_lastTick = nowTick;
    frameDt = std::clamp(frameDt, 0.0, 0.05);

    // Machine runtime polling (with frame delta time for smooth interpolation)
    m_machine.poll((float)frameDt);

    // Handle deferred STP import approval
    if (m_ui.importApproved_) {
        m_program = m_ui.importPending_;
        m_ui.importApproved_ = false;
        m_ui.importPending_ = BendingProgram();
        rebuildGeometry();
        std::cout << "[App] Import approved after changeover: " << m_program.steps.size() << " steps\n";
    }

    m_renderer.setShowBender(m_ui.showBender && !m_ui.useLogic2);
    m_renderer.setBenderRotation((float)m_benderAngle);
    m_renderer.setBenderOpaque(m_ui.benderOpaque);
    m_renderer.setBackfaceCulling(m_ui.settings.backfaceCulling);

    // Exclusion zone visualization
    m_renderer.setShowExclZone(m_ui.showExclZone);
    if (m_ui.showExclZone) {
        double D = m_program.D;
        double R = m_program.R;
        double tubeRadius = D / 2.0;
        double exclR = D * 1.2 / 2.0;
        float yMin = (float)(-(m_program.clampLength + tubeRadius));
        float pt0x = -(float)R;
        m_renderer.updateExclZone(pt0x, PT0_Z, (float)exclR, yMin,
                                  (float)R, (float)m_benderAngle);
    }

    // Logic 2: toggle new models visibility + clamp offset
    m_renderer.setShowLogic2(m_ui.useLogic2 && m_ui.showBender);
    
    // ⚠️ ИНТЕРПОЛЯЦИЯ ПРИЖИМА: ВСЕГДА работает, в любом режиме!
    // m_clampTargetOffset устанавливается выше в зависимости от UI флагов или Teensy статуса
    // m_clampOffset плавно интерполируется к целевому значению
    const double maxStep = CLAMP_SPEED * frameDt;
    const double delta = m_clampTargetOffset - m_clampOffset;
    if (std::abs(delta) <= maxStep || maxStep <= 1e-9) {
        m_clampOffset = m_clampTargetOffset;
    } else {
        m_clampOffset += (delta > 0.0 ? 1.0 : -1.0) * maxStep;
    }
    m_renderer.setClampOffset((float)m_clampOffset);

    // Laser head visibility & position
    bool laserVisible = m_ui.showLaserHead;
    m_renderer.setShowLaserHead(laserVisible);
    if (laserVisible) {
        float lx = -(float)m_program.R;
        float ly = -(float)m_ui.laserOffsetY;
        float lz = PT0_Z + (float)(m_program.D / 2.0) + (float)m_ui.laserGapZ;
        float tubeTopZ = PT0_Z + (float)(m_program.D / 2.0);
        m_renderer.setLaserHeadPos(lx, ly, lz, tubeTopZ);
    }
    m_renderer.setShowLaserBeam(false);

    // Auto-rebuild 3D when program changes (and not animating)
    if (!m_machine.isProgramRunning() && !m_laserAnimating) {
        size_t h = computeProgramHash();
        if (h != m_lastProgramHash) {
            m_lastProgramHash = h;
            m_ui.collisionChecked = false;
            rebuildGeometry();
        }
    }

    // v2.5: runtime animation is fully driven by Machine telemetry.
    // Legacy App animation loops are intentionally not executed in update().

    m_ui.animActive = m_machine.isProgramRunning();
    // NOTE: updateAnimProgress() and legacy animation paths REMOVED per ТЗ v2.5 section 2.3
    // All visualization now comes from syncMachineVisualState() using TeensyStatus telemetry.
    updateContinueUiState();

    const TeensyStatus& st = m_machine.status();
    m_ui.pedalPressed = isPedalHeld();
    m_ui.pedalRequired = m_machine.isProgramRunning() && !m_machine.pedalBypass_;
    bool hwSync = (m_machine.isConnected() || m_machine.isSimulated()) && st.valid;
    if (hwSync) {
        syncMachineVisualState(st);
    } else {
        if (m_hasIdleVisualState && m_idleTubeTransform) {
            m_renderer.setTubeTransform((float)m_idleTubeRotY, (float)m_idleTubeFeedY,
                                        (-(float)m_program.R), PT0_Y, PT0_Z);
        } else {
            m_renderer.clearTubeTransform();
        }
    }

    // Write current axis values for debug display
    if (hwSync) {
        m_ui.axisBend = st.bendCur;
        m_ui.axisCarriage = machineFeedPosition(st);
        m_ui.axisChuck = machineChuckPosition(st);
    } else {
        m_ui.axisBend = m_benderAngle;
        double fed = m_hasIdleVisualState ? m_idleCarriageFeed : 0.0;
        m_ui.axisCarriage = fed;
        m_renderer.setCarriageOffset((float)computeCarriageRenderFeed(fed));
        m_ui.axisChuck = computeCurrentChuckRotation();
    }

    // Camera orientation for UI gizmo
    m_ui.cameraYaw = m_renderer.yaw();
    m_ui.cameraPitch = m_renderer.pitch();
}

void App::render(int width, int height) {
    m_fbW = width;
    m_fbH = height;
    m_renderer.resize(width, height);

    // Ensure FBO matches the Scene window content size
    float renderScale = std::clamp(m_ui.settings.sceneRenderScale, 0.45f, 1.0f);
    int fboW = std::max(1, (int)(m_ui.sceneContentW * renderScale));
    int fboH = std::max(1, (int)(m_ui.sceneContentH * renderScale));
    if (fboW < 1) fboW = 1;
    if (fboH < 1) fboH = 1;
    m_renderer.ensureFBO(fboW, fboH);

    // Render 3D scene into FBO
    if (m_ui.showSceneWin && fboW > 0 && fboH > 0) {
        m_renderer.renderToFBO();
    }

    // Clear the main screen (ImGui will draw on top)
    m_renderer.beginFrame();
}

void App::drawUI() {
    unsigned int texId = m_renderer.fboTextureId();
    m_ui.draw(m_program, m_callbacks, m_fbW, m_fbH, texId);
}

// ─── Input (polling via ImGui IO) ───────────────────────────────────
void App::processInput() {
    ImGuiIO& io = ImGui::GetIO();
    // Allow 3D input when mouse hovers the transparent scene window
    if (io.WantCaptureMouse && !m_ui.sceneHovered) return;

    // Left-drag: orbit
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 1.0f)) {
        ImVec2 d = io.MouseDelta;
        if (d.x != 0 || d.y != 0)
            m_renderer.orbit(d.x * 0.3f, -d.y * 0.3f);
    }
    // Right-drag: pan
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right, 1.0f)) {
        ImVec2 d = io.MouseDelta;
        if (d.x != 0 || d.y != 0)
            m_renderer.pan(d.x, d.y);
    }
    // Scroll: zoom (only when Ctrl is NOT held — Ctrl+scroll is UI scaling)
    if (io.MouseWheel != 0 && !io.KeyCtrl)
        m_renderer.zoom(io.MouseWheel);
}

void App::onResize(int w, int h) {
    m_renderer.resize(w, h);
}

// ─── Geometry rebuild ───────────────────────────────────────────────
void App::rebuildGeometry() {
    m_program.deriveCuts();  // sync cutBefore/cutAfter from unified step list
    
    
    m_machineSceneValid = false;
    m_machineScenePhase = MachPhase::Idle;
    m_machineSceneStep = -1;
    m_benderAngle = 0.0;
    m_visualWhipLength = m_ui.whipLength;
    m_remainingWhipLength = m_ui.whipLength;
    m_completedParts = 0;
    m_resumeAvailable = false;
    m_resumeIsLaser = false;
    m_pendingNextPart = false;
    m_hasIdleVisualState = false;
    m_cycleBaseCarriageFeed = 0.0;
    m_ui.animStepIndex = -1;
    m_ui.animProgressFrac = 0.0f;
    m_renderer.clearTubeTransform();
    m_renderer.clearCollisionMarker();
    m_renderer.setShowLaserBeam(false);
    auto meshes = buildTubeGeometry(m_program, currentSceneWhipLength());
    m_renderer.uploadTubeSegments(meshes);
}

void App::rebuildGeometryUpToStep(int maxStep) {
    auto meshes = buildTubeGeometry(m_program, currentSceneWhipLength(), maxStep, m_animProgress);
    m_renderer.uploadTubeSegments(meshes);
}

void App::showCollisionStepPreview(int stepIdx) {
    if (stepIdx >= 0) {
        m_ui.selectedStep = stepIdx;
        rebuildGeometryUpToStep(stepIdx + 1);
    }
}

void App::clearCollisionPreview(bool rebuildFullGeometry) {
    m_renderer.clearCollisionMarker();
    if (rebuildFullGeometry && !m_machine.isProgramRunning() && !m_machine.isSimulated())
        rebuildGeometry();
}

// ─── Animation ──────────────────────────────────────────────────────
static const char* phaseNames[] = {
    u8"подача", u8"ротация", u8"прижим", u8"гиб", u8"отвод прижима", u8"отвод трубы", u8"возврат ролика"
};

void App::startAnimation() {
    // v2.5: legacy timer animation removed from runtime.
    // Keep method as compatibility entry-point and run unified Machine simulation path.
    m_animating = false;
    m_laserAnimating = false;
    m_renderer.setShowLaserBeam(false);
    m_renderer.clearCutContour();

    if (m_machine.isProgramRunning()) {
        m_machine.stopProgram();
    }
    if (!m_machine.isSimulated()) {
        m_machine.startSimulation();
    }

    m_ui.syncActiveProject(m_program);
    m_machine.startProgram(m_program, m_program.clampLength, m_ui.activeProjectName());
    m_resumeAvailable = false;
    m_resumeIsLaser = false;
    m_pendingNextPart = false;
    m_hasIdleVisualState = false;
    m_ui.infoText = u8"Симуляция запущена";
}

void App::updateAnimation() {
    // v2.5: legacy timer animation loop removed from runtime.
    return;
}

// ─── Laser animation (10-phase cycle) ───────────────────────────────
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char* laserPhaseNames[] = {
    u8"подача (рез до)", u8"рез (до)", u8"возврат для захвата",
    u8"подача", u8"ротация", u8"прижим", u8"гиб", u8"отвод прижима", u8"отвод трубы", u8"возврат ролика",
    u8"подача (рез после)", u8"рез (после)", u8"возврат (после)"
};

void App::startLaserAnimation() {
    // v2.5: laser visualize entry-point also uses unified Machine path.
    startAnimation();
}

void App::rebuildLaserBendGeometry() {
    // Deprecated in v2.5 runtime.
}

void App::advanceLaserStep() {
    // Deprecated in v2.5 runtime.
}

void App::updateLaserAnimation() {
    // v2.5: legacy laser timer loop removed from runtime.
    return;

    if (!m_laserAnimating || m_animationProgram.steps.empty()) return;
    if (m_ui.animPaused) return;
    if (!isPedalHeld()) {
        m_ui.infoText = u8"Зажмите педаль D7";
        return;
    }

    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - m_animStartTime).count();
    m_animStartTime = now;

    const auto& program = m_animationProgram;
    bool phaseComplete = false;

    switch (m_laserAnimPhase) {
    // ═══════════════════════════════════════════════════════════════
    //  CUT-BEFORE: straight blue tube + transform
    // ═══════════════════════════════════════════════════════════════
    case LCutBeforeFeed: {
        // Feed tube progressively: red segment grows from Point0 toward laser
        double feedDist = m_laserCutBeforeFeedDist;
        if (feedDist < 0.1) { phaseComplete = true; break; }
        m_laserAnimProgress += (ANIM_FEED_SPEED * dt) / feedDist;
        if (m_laserAnimProgress >= 1.0) { m_laserAnimProgress = 1.0; phaseComplete = true; }
        double currentFeed = feedDist * m_laserAnimProgress;
        m_renderer.clearTubeTransform();
        m_renderer.setShowLaserBeam(false);
        // Rebuild progressive geometry: blue tail + growing red segment (no cut contour yet)
        {
            BendingProgram tempProg;
            tempProg.D = program.D;
            tempProg.R = program.R;
            tempProg.clampLength = program.clampLength;
            // No cutBefore — cut hasn't happened yet, tip is flat
            BendStep feedStep;
            feedStep.id = 0;
            feedStep.feedLength = currentFeed;
            feedStep.bendAngle = 0;
            feedStep.rotation = 0;
            tempProg.steps.push_back(feedStep);
            auto meshes = buildTubeGeometry(tempProg, m_laserWhipRemaining);
            m_renderer.uploadTubeSegments(meshes);
        }
        break;
    }
    case LCutBefore: {
        // Rotate 360° with coordinated ΔY. Beam ON.
        // When tube rotates by α, the point under the laser is at local angle -α.
        // feedY = -feedDelta positions current θ-vertex exactly at laser Y.
        m_laserAnimProgress += (ANIM_CUT_ROT_SPEED * dt) / 360.0;
        if (m_laserAnimProgress >= 1.0) { m_laserAnimProgress = 1.0; phaseComplete = true; }
        m_laserCutRotAngle = m_laserAnimProgress * 360.0;
        m_laserCutFeedDelta = 0;
        if (!program.cutBefore.empty()) {
            // Point under laser is at local angle (-rotAngle), NOT (+rotAngle)
            double theta = -m_laserCutRotAngle * M_PI / 180.0;
            double r = program.D / 2.0;
            m_laserCutFeedDelta = program.cutBefore.contourY(theta, r)
                                  - program.cutBefore.planes[0].offsetAlongAxis;
        }
        m_renderer.setTubeTransform(
            (float)m_laserCutRotAngle,
            (float)(-m_laserCutFeedDelta),
            (-(float)program.R), PT0_Y, PT0_Z);
        m_renderer.setShowLaserBeam(true);
        // Update cut contour visualization on tube surface
        {
            std::vector<float> contour(360);
            double r = program.D / 2.0;
            double off = program.cutBefore.planes[0].offsetAlongAxis;
            for (int k = 0; k < 360; k++) {
                double th = 2.0 * M_PI * k / 360.0;
                double fY = program.cutBefore.contourY(th, r) - off;
                // Mesh cut edge at θ: Y_mesh = -feedDist + maxDeltaCB - fY
                // = -(laserOffsetY + maxDeltaCB) + maxDeltaCB - fY
                // = -laserOffsetY - fY
                contour[k] = (float)(-m_ui.laserOffsetY - fY);
            }
            float tubeR = (float)(program.D / 2.0);
            m_renderer.setCutContour(contour, (-(float)program.R), PT0_Z, tubeR, 0.0f);
        }
        break;
    }
    // ═══════════════════════════════════════════════════════════════
    //  RETURN FROM CUT: straight tube slides back to bender position
    // ═══════════════════════════════════════════════════════════════
    case LReturnFromCut: {
        // Smooth return: animate feedY from last CutBefore value to 0
        double returnDist = std::abs(m_laserCutFeedBase);
        if (returnDist < 0.1) { phaseComplete = true; break; }
        m_laserAnimProgress += (ANIM_FEED_SPEED * dt) / returnDist;
        if (m_laserAnimProgress >= 1.0) { m_laserAnimProgress = 1.0; phaseComplete = true; }
        double currentFeed = m_laserCutFeedBase * (1.0 - m_laserAnimProgress);
        m_renderer.setTubeTransform(0, (float)currentFeed, (-(float)program.R), PT0_Y, PT0_Z);
        m_renderer.setShowLaserBeam(false);
        break;
    }
    // ═══════════════════════════════════════════════════════════════
    //  BENDING PHASES: progressive geometry, no transform
    // ═══════════════════════════════════════════════════════════════
    case LFeed: {
        auto& step = program.steps[m_laserAnimStep];
        double alreadyFed = 0.0;
        if (m_laserAnimStep == 0 && !program.cutBefore.empty()) {
            alreadyFed = m_laserCutBeforeFeedDist;
        } else if (m_laserAnimStep > 0 &&
            program.steps[m_laserAnimStep - 1].bendAngle > 0.01) {
            alreadyFed = program.clampLength;
        }
        double remaining = step.feedLength - alreadyFed;
        if (remaining < 0.01) { phaseComplete = true; break; }
        m_laserAnimProgress += (ANIM_FEED_SPEED * dt) / remaining;
        if (m_laserAnimProgress >= 1.0) { m_laserAnimProgress = 1.0; phaseComplete = true; }
        m_benderAngle = 0.0;
        if (m_ui.useLogic2) m_clampOffset = -CLAMP_TRAVEL; // released during feed
        m_renderer.clearTubeTransform();
        m_renderer.setShowLaserBeam(false);
        rebuildLaserBendGeometry();
        break;
    }
    case LRotation: {
        auto& step = program.steps[m_laserAnimStep];
        double absRot = std::abs(step.rotation);
        if (absRot < 0.01) { phaseComplete = true; break; }
        m_laserAnimProgress += (ANIM_ROT_SPEED * dt) / absRot;
        if (m_laserAnimProgress >= 1.0) { m_laserAnimProgress = 1.0; phaseComplete = true; }
        m_benderAngle = 0.0;
        if (m_ui.useLogic2) m_clampOffset = -CLAMP_TRAVEL; // released
        m_renderer.clearTubeTransform();
        m_renderer.setShowLaserBeam(false);
        rebuildLaserBendGeometry();
        break;
    }
    case LClampClose: {
        // Clamp closes STRICTLY before bend
        m_laserAnimProgress += (CLAMP_SPEED * dt) / CLAMP_TRAVEL;
        if (m_laserAnimProgress >= 1.0) { m_laserAnimProgress = 1.0; phaseComplete = true; }
        m_benderAngle = 0.0;
        if (m_ui.useLogic2) m_clampOffset = -CLAMP_TRAVEL * (1.0 - m_laserAnimProgress);
        m_renderer.clearTubeTransform();
        m_renderer.setShowLaserBeam(false);
        rebuildLaserBendGeometry();
        break;
    }
    case LBend: {
        auto& step = program.steps[m_laserAnimStep];
        if (step.bendAngle < 0.01) { phaseComplete = true; break; }
        m_laserAnimProgress += (ANIM_BEND_SPEED * dt) / step.bendAngle;
        if (m_laserAnimProgress >= 1.0) { m_laserAnimProgress = 1.0; phaseComplete = true; }
        m_benderAngle = step.bendAngle * m_laserAnimProgress;
        if (m_ui.useLogic2) m_clampOffset = 0.0; // clamped during bend
        m_renderer.clearTubeTransform();
        m_renderer.setShowLaserBeam(false);
        rebuildLaserBendGeometry();
        break;
    }
    case LClampOpen: {
        // Clamp opens AFTER bend, BEFORE tube push (critical: prevents tube jamming)
        m_laserAnimProgress += (CLAMP_SPEED * dt) / CLAMP_TRAVEL;
        if (m_laserAnimProgress >= 1.0) { m_laserAnimProgress = 1.0; phaseComplete = true; }
        m_benderAngle = program.steps[m_laserAnimStep].bendAngle;
        if (m_ui.useLogic2) m_clampOffset = -CLAMP_TRAVEL * m_laserAnimProgress;
        m_renderer.clearTubeTransform();
        m_renderer.setShowLaserBeam(false);
        rebuildLaserBendGeometry();
        break;
    }
    case LTubePush: {
        double cl = program.clampLength;
        if (cl < 0.01) { phaseComplete = true; break; }
        m_laserAnimProgress += (ANIM_FEED_SPEED * dt) / cl;
        if (m_laserAnimProgress >= 1.0) { m_laserAnimProgress = 1.0; phaseComplete = true; }
        m_benderAngle = program.steps[m_laserAnimStep].bendAngle;
        if (m_ui.useLogic2) m_clampOffset = -CLAMP_TRAVEL; // released
        m_renderer.clearTubeTransform();
        m_renderer.setShowLaserBeam(false);
        rebuildLaserBendGeometry();
        break;
    }
    case LBenderReturn: {
        auto& step = program.steps[m_laserAnimStep];
        if (step.bendAngle < 0.01) { phaseComplete = true; break; }
        m_laserAnimProgress += (ANIM_RETURN_SPEED * dt) / step.bendAngle;
        if (m_laserAnimProgress >= 1.0) { m_laserAnimProgress = 1.0; phaseComplete = true; }
        m_benderAngle = step.bendAngle * (1.0 - m_laserAnimProgress);
        if (m_ui.useLogic2) m_clampOffset = -CLAMP_TRAVEL; // released
        m_renderer.clearTubeTransform();
        m_renderer.setShowLaserBeam(false);
        rebuildLaserBendGeometry();
        break;
    }
    // ═══════════════════════════════════════════════════════════════
    //  CUT-AFTER: fully bent tube + transform
    // ═══════════════════════════════════════════════════════════════
    case LCutAfterFeed: {
        // Feed fully bent tube so Point0 (blue/red junction at Y=0 on mesh) reaches laser (Y=-60)
        // Point0 is the visible tube end — CutAfter cut happens here
        double additionalFeed = m_ui.laserOffsetY;
        if (additionalFeed < 0.1) additionalFeed = 60.0;
        m_laserAnimProgress += (ANIM_FEED_SPEED * dt) / std::max(additionalFeed, 1.0);
        if (m_laserAnimProgress >= 1.0) { m_laserAnimProgress = 1.0; phaseComplete = true; }
        m_laserCutFeedBase = additionalFeed * m_laserAnimProgress;
        m_renderer.setTubeTransform(0, (float)m_laserCutFeedBase, (-(float)program.R), PT0_Y, PT0_Z);
        m_renderer.setShowLaserBeam(false);
        break;
    }
    case LCutAfter: {
        // Rotate 360° with coordinated ΔY. Beam ON.
        m_laserAnimProgress += (ANIM_CUT_ROT_SPEED * dt) / 360.0;
        if (m_laserAnimProgress >= 1.0) { m_laserAnimProgress = 1.0; phaseComplete = true; }
        m_laserCutRotAngle = m_laserAnimProgress * 360.0;
        if (!program.cutAfter.empty()) {
            // Point under laser is at local angle (-rotAngle), NOT (+rotAngle)
            double theta = -m_laserCutRotAngle * M_PI / 180.0;
            double r = program.D / 2.0;
            m_laserCutFeedDelta = program.cutAfter.contourY(theta, r)
                                  - program.cutAfter.planes[0].offsetAlongAxis;
        }
        m_renderer.setTubeTransform(
            (float)m_laserCutRotAngle,
            (float)(m_laserCutFeedBase + m_laserCutFeedDelta),
            (-(float)program.R), PT0_Y, PT0_Z);
        m_renderer.setShowLaserBeam(true);
        // Update cut contour visualization on tube surface (CutAfter contour at Point0)
        if (!program.cutAfter.empty()) {
            std::vector<float> contour(360);
            double r = program.D / 2.0;
            double off = program.cutAfter.planes[0].offsetAlongAxis;
            for (int k = 0; k < 360; k++) {
                double th = 2.0 * M_PI * k / 360.0;
                double fY = program.cutAfter.contourY(th, r) - off;
                // CutAfter contour is at Point0 (Y=0 on mesh), offset by fY
                contour[k] = (float)fY;
            }
            float tubeR = (float)(program.D / 2.0);
            m_renderer.setCutContour(contour, (-(float)program.R), PT0_Z, tubeR, 0.0f);
        }
        break;
    }
    // ═══════════════════════════════════════════════════════════════
    //  RETURN AFTER CUT-AFTER: smooth return to origin
    // ═══════════════════════════════════════════════════════════════
    case LReturnAfterCutAfter: {
        double returnDist = std::abs(m_laserCutFeedBase);
        if (returnDist < 0.1) { phaseComplete = true; break; }
        m_laserAnimProgress += (ANIM_FEED_SPEED * dt) / returnDist;
        if (m_laserAnimProgress >= 1.0) { m_laserAnimProgress = 1.0; phaseComplete = true; }
        double currentFeed = m_laserCutFeedBase * (1.0 - m_laserAnimProgress);
        m_renderer.setTubeTransform(0, (float)currentFeed, (-(float)program.R), PT0_Y, PT0_Z);
        m_renderer.setShowLaserBeam(false);
        break;
    }
    } // switch

    // ═══════════════════════════════════════════════════════════════
    //  PHASE TRANSITIONS
    // ═══════════════════════════════════════════════════════════════
    if (phaseComplete) {
        m_laserAnimProgress = 0.0;

        switch (m_laserAnimPhase) {
        case LCutBeforeFeed:
            // Done feeding to laser → start cutting (CutBefore)
            m_laserAnimPhase = LCutBefore;
            m_laserCutRotAngle = 0;
            m_laserCutFeedDelta = 0;
            m_ui.infoText = u8"Лазер: рез (до)";
            break;

        case LCutBefore: {
            // Save last feedY for smooth return transition
            m_laserCutFeedBase = -m_laserCutFeedDelta; // last feedY value from cutting
            m_laserCutRotAngle = 0;
            m_laserCutFeedDelta = 0;
            m_renderer.setShowLaserBeam(false);
            // Rebuild mesh WITH cutBefore contour (saddle appears = cut piece removed)
            {
                BendingProgram tempProg;
                tempProg.D = program.D;
                tempProg.R = program.R;
                tempProg.clampLength = program.clampLength;
                tempProg.cutBefore = program.cutBefore; // saddle contour appears!
                BendStep feedStep;
                feedStep.id = 0;
                feedStep.feedLength = m_laserCutBeforeFeedDist;
                feedStep.bendAngle = 0;
                feedStep.rotation = 0;
                tempProg.steps.push_back(feedStep);
                auto meshes = buildTubeGeometry(tempProg, m_laserWhipRemaining);
                m_renderer.uploadTubeSegments(meshes);
            }
            // Keep transform with last feedY (rotation cleared) for smooth return
            m_renderer.setTubeTransform(0, (float)m_laserCutFeedBase, (-(float)program.R), PT0_Y, PT0_Z);
            m_laserAnimPhase = LReturnFromCut;
            m_ui.infoText = u8"Лазер: возврат после реза";
            break;
        }

        case LReturnFromCut:
            // Done returning → begin bending (clear transform, build progressive geometry)
            m_renderer.clearTubeTransform();
            m_benderAngle = 0;
            m_laserAnimPhase = LFeed;
            rebuildLaserBendGeometry(); // starts with alreadyFed = cutBeforeFeedDist
            {
                char buf[128];
                snprintf(buf, sizeof(buf), u8"Лазер: %s — шаг %d",
                         laserPhaseNames[LFeed], mapAnimatedStepIndex(m_laserAnimStep) + 1);
                m_ui.infoText = buf;
            }
            // Check if Feed should be manual (skip when feedLength ≈ 0)
            if (!m_ui.autoFeed && program.steps[m_laserAnimStep].feedLength >= 0.01)
                pauseOnPhase(u8"Выполните подачу вручную");
            break;

        case LFeed: {
            auto& step = program.steps[m_laserAnimStep];
            double alreadyFed = 0.0;
            if (m_laserAnimStep == 0 && !program.cutBefore.empty()) {
                alreadyFed = m_laserCutBeforeFeedDist;
            } else if (m_laserAnimStep > 0 &&
                program.steps[m_laserAnimStep - 1].bendAngle > 0.01) {
                alreadyFed = program.clampLength;
            }
            m_laserTotalFed += std::max(0.0, step.feedLength - alreadyFed);
            m_laserAnimPhase = LRotation;
            {
                char buf[128];
                snprintf(buf, sizeof(buf), u8"Лазер: %s — шаг %d",
                         laserPhaseNames[LRotation], mapAnimatedStepIndex(m_laserAnimStep) + 1);
                m_ui.infoText = buf;
            }
            // Check if Rotation should be manual (skip when rotation ≈ 0)
            if (!m_ui.autoRotation && std::abs(program.steps[m_laserAnimStep].rotation) >= 0.01)
                pauseOnPhase(u8"Выполните ротацию вручную");
            break;
        }
        case LRotation:
            m_laserAnimPhase = LClampClose;
            {
                char buf[128];
                snprintf(buf, sizeof(buf), u8"Лазер: %s — шаг %d",
                         laserPhaseNames[LClampClose], mapAnimatedStepIndex(m_laserAnimStep) + 1);
                m_ui.infoText = buf;
            }
            // Check if Clamp should be manual
            if (!m_ui.autoClamp)
                pauseOnPhase(u8"Выполните прижим вручную");
            break;

        case LClampClose: {
            auto& step = program.steps[m_laserAnimStep];
            if (step.bendAngle < 0.01) {
                // No bend — skip Bend/ClampOpen/TubePush/BenderReturn
                advanceLaserStep();
            } else {
                m_laserAnimPhase = LBend;
                char buf[128];
                snprintf(buf, sizeof(buf), u8"Лазер: %s — шаг %d",
                         laserPhaseNames[LBend], mapAnimatedStepIndex(m_laserAnimStep) + 1);
                m_ui.infoText = buf;
            }
            break;
        }

        case LBend: {
            auto& step = program.steps[m_laserAnimStep];
            if (step.bendAngle < 0.01) {
                advanceLaserStep();
            } else {
                m_laserAnimPhase = LClampOpen;
                char buf[128];
                snprintf(buf, sizeof(buf), u8"Лазер: %s — шаг %d",
                         laserPhaseNames[LClampOpen], mapAnimatedStepIndex(m_laserAnimStep) + 1);
                m_ui.infoText = buf;
            }
            break;
        }
        case LClampOpen:
            m_laserAnimPhase = LTubePush;
            {
                char buf[128];
                snprintf(buf, sizeof(buf), u8"Лазер: %s — шаг %d",
                         laserPhaseNames[LTubePush], mapAnimatedStepIndex(m_laserAnimStep) + 1);
                m_ui.infoText = buf;
            }
            break;
        case LTubePush:
            m_laserTotalFed += program.clampLength;
            m_laserAnimPhase = LBenderReturn;
            {
                char buf[128];
                snprintf(buf, sizeof(buf), u8"Лазер: %s — шаг %d",
                         laserPhaseNames[LBenderReturn], mapAnimatedStepIndex(m_laserAnimStep) + 1);
                m_ui.infoText = buf;
            }
            break;

        case LBenderReturn:
            advanceLaserStep();
            break;

        case LCutAfterFeed:
            m_laserAnimPhase = LCutAfter;
            m_laserCutRotAngle = 0;
            m_laserCutFeedDelta = 0;
            m_ui.infoText = u8"Лазер: рез (после)";
            break;

        case LCutAfter:
            // Smooth return before starting next part
            m_renderer.setShowLaserBeam(false);
            m_renderer.clearCutContour();
            // Reset rotation, keep feed offset for smooth slide back
            m_renderer.setTubeTransform(0, (float)m_laserCutFeedBase, (-(float)program.R), PT0_Y, PT0_Z);
            m_laserAnimPhase = LReturnAfterCutAfter;
            m_ui.infoText = u8"Лазер: возврат после реза (после)";
            break;

        case LReturnAfterCutAfter:
            m_renderer.clearTubeTransform();
            finishCurrentPart();
            break;
        } // switch transition
    }
}

// ─── Интерполяция координат между фазами ────────────────────────────────
// Вместо скачков, используем плавный переход 0.0 -> 1.0 в течение фазы
// ⚠️ КЛЮЧ: Целевые координаты вычисляются на основе шага программы!

// Вспомогательная функция: получить "основную" фазу для интерполяции
// Краткие фазы (Wait, LaserCut, Homing и т.д.) игнорируются
static MachPhase getMainPhaseForInterpolation(MachPhase phase) {
    switch (phase) {
        case MachPhase::Feed:
        case MachPhase::Rotation:
        case MachPhase::BendWait:
        case MachPhase::Clearance:
            return phase;  // Основные фазы движения
        default:
            return MachPhase::Idle;  // Все остальные фазы - не интерполируем
    }
}

void App::updatePhaseInterpolation(double dt, MachPhase currentPhase, int stepIdx,
                                   double currentZ, double currentC, double currentB,
                                   const BendingProgram& program, double baseFeedAcc, double baseRotAcc) {
    // ⚠️ При смене шага переинициализируем фазу, но НЕ форсируем интерполяцию!
    // Интерполяция должна достичь 1.0 естественно через dt (плавно)
    static int lastInterpolStepIdx = -1;
    if (stepIdx != lastInterpolStepIdx) {
        // ВАЖНО: НЕ присваиваем m_phaseInterpolProgress = 1.0 (это нарушает плавность)
        m_lastRenderedPhase = MachPhase::Idle;  // Переинициализируем фазу
        lastInterpolStepIdx = stepIdx;
        Logger::log(std::string("[INTERP_RESET] Step changed: ") + std::to_string(stepIdx) +
                   " | phase reinitialized, interp continues smoothly");
    }
    
    // ⚠️ КЛЮЧ: Используем только "основные" фазы для интерполяции
    MachPhase mainPhase = getMainPhaseForInterpolation(currentPhase);
    
    // Если это краткая фаза (Idle), продолжаем интерполяцию плавно
    if (mainPhase == MachPhase::Idle) {
        m_phaseInterpolProgress += (dt / 0.15);  // Плавное завершение интерполяции
        if (m_phaseInterpolProgress > 1.0) {
            m_phaseInterpolProgress = 1.0;
        }
        return;
    }
    
    // Если основная фаза изменилась, инициализируем новую интерполяцию
    if (m_lastRenderedPhase != mainPhase) {
        m_lastRenderedPhase = mainPhase;
        m_phaseInterpolProgress = 0.0;
        
        // ⚠️ КРИТИЧЕСКИЙ ФИХ: Используем ПОСЛЕДНЕЕ ОТРЕНДЕРЕННОЕ значение, не текущее машинное!
        // Это предотвращает скачки когда actualZ != targetZ из-за инерции/задержек
        // Если это первая фаза, используем текущие координаты машины
        if (m_lastRenderZ > 0.0 || m_lastRenderC > 0.0 || m_lastRenderB > 0.0) {
            m_interpStartZ = m_lastRenderZ;
            m_interpStartC = m_lastRenderC;
            m_interpStartB = m_lastRenderB;
            Logger::log(std::string("[INTERP_START_FROM_RENDER] Using last render values instead of machine actual"));
        } else {
            m_interpStartZ = currentZ;
            m_interpStartC = currentC;
            m_interpStartB = currentB;
            Logger::log(std::string("[INTERP_START_FROM_ACTUAL] First phase - using machine actual values"));
        }
        
        // ⚠️ КРИТИЧЕСКАЯ ЧАСТЬ: Вычисляем целевые координаты на основе ТЗ 8.1.3
        // Целевые значения зависят от фазы и текущего шага
        const int totalSteps = (int)program.steps.size();
        const int stepIdxClamped = std::clamp(stepIdx, 0, totalSteps - 1);
        
        if (stepIdxClamped >= 0 && stepIdxClamped < totalSteps) {
            const auto& step = program.steps[stepIdxClamped];
            
            // Вычисляем целевые абсолютные координаты для каждой фазы
            const double Z_afterFeed = baseFeedAcc + step.feedLength;
            const double Z_afterBend = Z_afterFeed + step.arcLength(program.R);
            const double C_afterRot = baseRotAcc + step.rotation;
            
            // Целевые значения в зависимости от фазы
            // Это то, к чему должны стремиться интерполированные координаты
            switch (mainPhase) {
                case MachPhase::Feed:
                    m_interpTargetZ = Z_afterFeed;   // После Feed Z = base + feedLength
                    m_interpTargetC = baseRotAcc;    // C не меняется во время Feed
                    m_interpTargetB = 0.0;           // B = 0 во время Feed
                    break;
                case MachPhase::Rotation:
                    m_interpTargetZ = Z_afterFeed;   // Z фиксирована на конце Feed
                    m_interpTargetC = C_afterRot;    // После Rotation C = base + rotation
                    m_interpTargetB = 0.0;           // B = 0 во время Rotation
                    break;
                case MachPhase::BendWait:
                    m_interpTargetZ = Z_afterBend;   // После Bend Z увеличивается
                    m_interpTargetC = C_afterRot;    // C не меняется во время Bend
                    m_interpTargetB = step.bendAngle; // B = bendAngle после полного Bend
                    break;
                case MachPhase::Clearance:
                    // После Clearance Z движется вперед на clampLength
                    m_interpTargetZ = Z_afterBend + program.clampLength;
                    m_interpTargetC = C_afterRot;    // C не меняется
                    m_interpTargetB = 0.0;           // B = 0 (вернулась из гибки)
                    break;
                default:
                    // Не должно быть
                    m_interpTargetZ = currentZ;
                    m_interpTargetC = currentC;
                    m_interpTargetB = currentB;
            }
        } else {
            // Нет шага - целевые = текущие
            m_interpTargetZ = currentZ;
            m_interpTargetC = currentC;
            m_interpTargetB = currentB;
        }
        
        Logger::log(std::string("[INTERP_START] Phase: ") + 
                   (mainPhase == MachPhase::Feed ? "Feed" :
                    mainPhase == MachPhase::Rotation ? "Rotation" :
                    mainPhase == MachPhase::BendWait ? "Bend" :
                    mainPhase == MachPhase::Clearance ? "Clearance" :
                    "ERROR") +
                   " | step=" + std::to_string(stepIdxClamped) +
                   " | actualZ=" + std::to_string(currentZ) +
                   " | actualC=" + std::to_string(currentC) +
                   " | actualB=" + std::to_string(currentB) +
                   " | START: Z=" + std::to_string(m_interpStartZ) +
                   ", C=" + std::to_string(m_interpStartC) +
                   " | TARGET: Z=" + std::to_string(m_interpTargetZ) +
                   ", C=" + std::to_string(m_interpTargetC) +
                   ", B=" + std::to_string(m_interpTargetB) +
                   " | baseFeedAcc=" + std::to_string(baseFeedAcc));
    }
    
    // Определяем ДИНАМИЧЕСКУЮ длительность фазы (как в v2.2!)
    // Длительность зависит от расстояния/угла движения и скорости
    // Это обеспечивает синхронизацию анимации с реальным движением машины
    double phaseDuration = 0.1;  // по умолчанию
    
    const int totalSteps = (int)program.steps.size();
    const int stepIdxClamped = std::clamp(stepIdx, 0, totalSteps - 1);
    
    if (stepIdxClamped >= 0 && stepIdxClamped < totalSteps) {
        const auto& step = program.steps[stepIdxClamped];
        
        // ⚠️ КРИТИЧЕСКОЕ ИЗМЕНЕНИЕ: Вычисляем длительность как distance/speed (как в v2.2)
        // Это автоматически делает анимацию синхронной с реальным движением
        switch (mainPhase) {
            case MachPhase::Feed: {
                // Как в v2.2: alreadyFed = если предыдущий шаг имел гибку
                double alreadyFed = (stepIdxClamped > 0 && std::abs(program.steps[stepIdxClamped - 1].bendAngle) > 0.01)
                    ? program.clampLength : 0.0;
                double remaining = step.feedLength - alreadyFed;
                if (remaining < 0.01) remaining = 0.01;
                // duration = distance / speed (как в v2.2: remaining / ANIM_FEED_SPEED)
                phaseDuration = remaining / 300.0;  // 300 mm/s
                break;
            }
            case MachPhase::Rotation: {
                double absRot = std::abs(step.rotation);
                if (absRot < 0.01) absRot = 0.01;
                // duration = angle / speed (как в v2.2: absRot / ANIM_ROT_SPEED)
                phaseDuration = absRot / 120.0;  // 120 deg/s
                break;
            }
            case MachPhase::BendWait: {
                double bendAngle = std::abs(step.bendAngle);
                if (bendAngle < 0.01) bendAngle = 0.01;
                // duration = angle / speed (как в v2.2: bendAngle / ANIM_BEND_SPEED)
                phaseDuration = bendAngle / 60.0;  // 60 deg/s
                break;
            }
            case MachPhase::Clearance: {
                double cl = program.clampLength;
                if (cl < 0.01) cl = 0.01;
                // duration = distance / speed (как в v2.2: cl / ANIM_FEED_SPEED)
                phaseDuration = cl / 300.0;  // 300 mm/s
                break;
            }
            default:
                phaseDuration = 0.1;
        }
    }
    
    // Инкрементируем прогресс интерполяции
    // ⚠️ Формула: progress += dt / phaseDuration (плавно в обоих режимах)
    // Где phaseDuration = distance / speed, поэтому:
    // progress = dt / (distance / speed) = (speed * dt) / distance
    if (phaseDuration > 0.0001) {  // Safety check
        m_phaseInterpolProgress += (dt / phaseDuration);
    }
    if (m_phaseInterpolProgress > 1.0) {
        m_phaseInterpolProgress = 1.0;  // Natural completion, not forced
    }
    
    // ДИАГНОСТИКА
    static int logCounter = 0;
    if (++logCounter % 30 == 0) {  // логируем каждые ~30 фреймов
        Logger::log(std::string("[INTERP_PROGRESS] Phase: ") + 
                   (mainPhase == MachPhase::Feed ? "Feed" :
                    mainPhase == MachPhase::Rotation ? "Rotation" :
                    mainPhase == MachPhase::BendWait ? "Bend" :
                    mainPhase == MachPhase::Clearance ? "Clearance" :
                    "Idle") +
                   " | dt=" + std::to_string(dt) +
                   " | phaseDuration=" + std::to_string(phaseDuration) +
                   " | progress=" + std::to_string(m_phaseInterpolProgress));
    }
}

// ─── STP import ─────────────────────────────────────────────────────
void App::importSTP(const std::string& path) {
    std::cout << "[App] Importing STP: " << path << "\n";
    try {
        auto parsed = importStep(path);

        // Convert cutBefore/cutAfter from STP parser into unified step entries
        if (!parsed.cutBefore.empty()) {
            BendStep cs;
            cs.isCut = true;
            cs.cutDef = parsed.cutBefore;
            parsed.steps.insert(parsed.steps.begin(), cs);
        }
        if (!parsed.cutAfter.empty()) {
            BendStep cs;
            cs.isCut = true;
            cs.cutDef = parsed.cutAfter;
            parsed.steps.push_back(cs);
        }
        for (int i = 0; i < (int)parsed.steps.size(); i++)
            parsed.steps[i].id = i + 1;

        // Check D and R against current tooling
        double fileD = parsed.D;
        double fileR = parsed.R;
        bool dMismatch = std::abs(fileD - m_ui.toolingD_) > 0.1;
        bool rMismatch = std::abs(fileR - m_ui.toolingR_) > 0.1;

        if (dMismatch || rMismatch) {
            // Store pending import, show conflict dialog via UI
            m_ui.importD_ = fileD;
            m_ui.importR_ = fileR;
            m_ui.importPending_ = parsed;
            m_ui.showImportConflict_ = true;
            m_ui.importApproved_ = false;
            std::cout << "[App] STP D/R mismatch: file D=" << fileD << " R=" << fileR
                      << ", tooling D=" << m_ui.toolingD_ << " R=" << m_ui.toolingR_ << "\n";
            return; // don't apply yet — wait for user confirmation
        }

        // D/R match — apply directly
        m_program = parsed;
        rebuildGeometry();
        std::cout << "[App] Import OK: " << m_program.steps.size() << " steps\n";
    } catch (const std::exception& e) {
        std::cerr << "[App] STP import failed: " << e.what() << "\n";
    }
}

// ─── STP export ─────────────────────────────────────────────────────
void App::exportSTP(const std::string& path) {
    std::cout << "[App] Exporting STP: " << path << "\n";
    if (exportStep(m_program, path)) {
        std::cout << "[App] Export OK\n";
    } else {
        std::cerr << "[App] STP export failed\n";
    }
}

// ─── JSON save/load ─────────────────────────────────────────────────
void App::saveJSON(const std::string& path) {
    // Convert UTF-8 path to wide for proper file creation
    nlohmann::json j = m_program.toJson();
#ifdef _WIN32
    int wlen = MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, nullptr, 0);
    std::wstring wpath(wlen, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, wpath.data(), wlen);
    std::ofstream f(wpath);
#else
    std::ofstream f(path);
#endif
    if (f) {
        f << j.dump(2);
        std::cout << "[App] Saved JSON: " << path << "\n";
    }
}

void App::loadJSON(const std::string& path) {
#ifdef _WIN32
    int wlen = MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, nullptr, 0);
    std::wstring wpath(wlen, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, wpath.data(), wlen);
    std::ifstream f(wpath);
#else
    std::ifstream f(path);
#endif
    if (!f) { std::cerr << "[App] Cannot open: " << path << "\n"; return; }
    nlohmann::json j;
    f >> j;
    m_program = BendingProgram::fromJson(j);
    rebuildGeometry();
    std::cout << "[App] Loaded JSON: " << m_program.steps.size() << " steps\n";
}

// ─── IGES export — flat development for laser tube cutters ──────────
// Format: IGES (*.igs) — compatible with most CAD/CAM systems.
// Convention: X = position along tube circumference (0..πD), Y = position along tube axis.
// Origin (0,0) = top of tube (θ=0), start of first segment (Y=0).
void App::exportIGES(const std::string& path) {
    std::cout << "[App] Exporting IGES: " << path << "\n";

    double D = m_program.D;
    double R = m_program.R;
    double r = D / 2.0;
    double circumference = M_PI * D;    // width of development
    double lTotal = m_program.lTotal();  // height of development

    // --- Build wire geometry using OCCT ---
    IGESControl_Writer writer(Interface_Static::CVal("write.iges.unit"), 1); // 1 = mm

    auto addLine = [&](double x1, double y1, double x2, double y2) {
        BRepBuilderAPI_MakeEdge edge(gp_Pnt(x1, y1, 0), gp_Pnt(x2, y2, 0));
        if (edge.IsDone())
            writer.AddShape(edge.Shape());
    };

    auto addPolyline = [&](const std::vector<std::pair<double,double>>& pts) {
        if (pts.size() < 2) return;
        BRepBuilderAPI_MakeWire wireBuilder;
        for (size_t i = 0; i + 1 < pts.size(); i++) {
            BRepBuilderAPI_MakeEdge edge(
                gp_Pnt(pts[i].first, pts[i].second, 0),
                gp_Pnt(pts[i+1].first, pts[i+1].second, 0));
            if (edge.IsDone())
                wireBuilder.Add(edge.Edge());
        }
        if (wireBuilder.IsDone())
            writer.AddShape(wireBuilder.Wire());
    };

    // Compute segment boundaries along Y axis (from Y=0 = far end / CutBefore side)
    struct SegBound { double yStart; double yEnd; bool isBend; double angle; };
    std::vector<SegBound> segments;
    {
        double consumed = 0;
        for (int i = (int)m_program.steps.size() - 1; i >= 0; i--) {
            auto& s = m_program.steps[i];
            if (std::abs(s.bendAngle) > 0.01) {
                double arcLen = s.arcLength(R);
                segments.push_back({consumed, consumed + arcLen, true, s.bendAngle});
                consumed += arcLen;
            }
            if (s.feedLength > 0.01) {
                segments.push_back({consumed, consumed + s.feedLength, false, 0});
                consumed += s.feedLength;
            }
        }
        if (m_program.clampLength > 0.01) {
            segments.push_back({consumed, consumed + m_program.clampLength, false, 0});
        }
    }

    // Compute cut contour points (360 points per contour)
    auto computeContour = [&](const CutDefinition& cd, double yBase, int numPts) {
        std::vector<std::pair<double,double>> pts;
        if (cd.empty()) return pts;
        double off = cd.planes[0].offsetAlongAxis;
        pts.reserve(numPts + 1);
        for (int k = 0; k <= numPts; k++) {
            double theta = 2.0 * M_PI * k / numPts;
            double dy = cd.contourY(theta, r) - off;
            double xDev = theta / (2.0 * M_PI) * circumference;
            double yDev = yBase + dy;
            pts.push_back({xDev, yDev});
        }
        return pts;
    };

    auto cutBeforePts = computeContour(m_program.cutBefore, 0.0, 360);
    auto cutAfterPts  = computeContour(m_program.cutAfter,  lTotal, 360);

    // 1. Outline rectangle
    addLine(0, 0, circumference, 0);
    addLine(circumference, 0, circumference, lTotal);
    addLine(circumference, lTotal, 0, lTotal);
    addLine(0, lTotal, 0, 0);

    // 2. Centerline
    addLine(circumference / 2.0, 0, circumference / 2.0, lTotal);

    // 3. Segment boundaries
    {
        double yAccum = 0;
        for (int i = (int)m_program.steps.size() - 1; i >= 0; i--) {
            auto& s = m_program.steps[i];
            if (std::abs(s.bendAngle) > 0.01) {
                double arcLen = s.arcLength(R);
                yAccum += arcLen;
                addLine(0, yAccum, circumference, yAccum);
            }
            if (s.feedLength > 0.01) {
                yAccum += s.feedLength;
                if (i > 0)
                    addLine(0, yAccum, circumference, yAccum);
            }
        }
    }

    // 4. Cut contours
    if (!cutBeforePts.empty()) addPolyline(cutBeforePts);
    if (!cutAfterPts.empty())  addPolyline(cutAfterPts);

    // Write IGES file
    writer.ComputeModel();

#ifdef _WIN32
    int wlen = MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, nullptr, 0);
    std::wstring wpath(wlen, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, wpath.data(), wlen);
    // IGES writer needs narrow ANSI path — convert via GetShortPathName
    char ansi[MAX_PATH];
    wchar_t shortPath[MAX_PATH];
    GetShortPathNameW(wpath.c_str(), shortPath, MAX_PATH);
    WideCharToMultiByte(CP_ACP, 0, shortPath, -1, ansi, MAX_PATH, nullptr, nullptr);
    Standard_Boolean ok = writer.Write(ansi);
#else
    Standard_Boolean ok = writer.Write(path.c_str());
#endif

    if (ok) {
        std::cout << "[App] IGES exported: " << path << "\n";
        m_ui.infoText = u8"IGES развёртка экспортирована";
    } else {
        std::cerr << "[App] IGES export failed: " << path << "\n";
        m_ui.infoText = u8"Ошибка экспорта IGES!";
    }
}

// ─── Collision check ────────────────────────────────────────────────
void App::checkCollisions() {
    if (m_program.steps.empty()) {
        m_ui.infoText = u8"Нет шагов для проверки столкновений";
        clearCollisionPreview(false);
        return;
    }

    m_renderer.clearCollisionMarker();

    // Simple geometric collision check:
    // After each bend, check if the tube end-point comes too close to
    // earlier segments or the bender fixture.
    double D = m_program.D;
    double R = m_program.R;
    double minClearance = D;  // minimum clearance = one tube diameter

    struct Vec3 { double x, y, z; };
    std::vector<Vec3> endPoints;  // end-point of tube after each segment

    Vec3 pos = {0, 0, 0};
    Vec3 dir = {0, 1, 0};  // initial direction along Y
    Vec3 up  = {0, 0, 1};  // Z up

    endPoints.push_back(pos);
    bool collisionFound = false;
    std::string collisionDetail;
    int collisionStep = -1;

    for (int i = 0; i < (int)m_program.steps.size(); i++) {
        auto& s = m_program.steps[i];

        // Check straight section length between adjacent bends
        if (i > 0 && std::abs(m_program.steps[i-1].bendAngle) > 0.01
                   && std::abs(s.bendAngle) > 0.01) {
            if (s.feedLength < m_ui.toolingStraightLen_ - 0.01) {
                collisionFound = true;
                collisionStep = i;
                char buf[256];
                snprintf(buf, sizeof(buf),
                    u8"Шаг %d: прямой участок %.1f мм < мин. %.1f мм (длина ролика)",
                    i+1, s.feedLength, m_ui.toolingStraightLen_);
                collisionDetail = buf;
                break;
            }
        }

        // Feed
        pos.x += dir.x * s.feedLength;
        pos.y += dir.y * s.feedLength;
        pos.z += dir.z * s.feedLength;
        endPoints.push_back(pos);

        // Bend (rotate direction)
        if (std::abs(s.bendAngle) > 0.01) {
            double angle = s.bendAngle * M_PI / 180.0;
            // Bend happens in the plane defined by dir and up
            double cosA = std::cos(angle);
            double sinA = std::sin(angle);
            Vec3 newDir;
            newDir.x = dir.x * cosA + up.x * sinA;
            newDir.y = dir.y * cosA + up.y * sinA;
            newDir.z = dir.z * cosA + up.z * sinA;
            dir = newDir;
        }

        // Rotation (rotate the bend plane)
        if (i + 1 < (int)m_program.steps.size() && std::abs(m_program.steps[i+1].rotation) > 0.01) {
            double rot = m_program.steps[i+1].rotation * M_PI / 180.0;
            double cosR = std::cos(rot);
            double sinR = std::sin(rot);
            // Rotate 'up' around 'dir'
            // up_new = up * cos(rot) + cross(dir, up) * sin(rot)
            Vec3 cross;
            cross.x = dir.y * up.z - dir.z * up.y;
            cross.y = dir.z * up.x - dir.x * up.z;
            cross.z = dir.x * up.y - dir.y * up.x;
            Vec3 newUp;
            newUp.x = up.x * cosR + cross.x * sinR;
            newUp.y = up.y * cosR + cross.y * sinR;
            newUp.z = up.z * cosR + cross.z * sinR;
            up = newUp;
        }

        // Check distance from current end-point to all earlier segments
        for (int j = 0; j < (int)endPoints.size() - 2; j++) {
            double dx = pos.x - endPoints[j].x;
            double dy = pos.y - endPoints[j].y;
            double dz = pos.z - endPoints[j].z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (dist < minClearance && dist > 0.01) {
                collisionFound = true;
                collisionStep = i;
                char buf[256];
                snprintf(buf, sizeof(buf),
                    u8"Столкновение! Шаг %d слишком близко к сегменту %d (%.1f мм < %.1f мм)",
                    i+1, j+1, dist, minClearance);
                collisionDetail = buf;
                break;
            }
        }
        if (collisionFound) break;
    }

    if (collisionFound) {
        m_ui.collisionStepIdx = collisionStep;
        m_ui.collisionInfo = collisionDetail;
        m_ui.infoText = collisionDetail;
        m_ui.collisionChecked = false;
        showCollisionStepPreview(collisionStep);
    } else {
        m_ui.collisionStepIdx = -1;
        m_ui.collisionInfo.clear();
        m_ui.collisionChecked = true;
        clearCollisionPreview();
        char buf[128];
        snprintf(buf, sizeof(buf),
            u8"Столкновений не обнаружено (%d шагов проверено)",
            (int)m_program.steps.size());
        m_ui.infoText = buf;
    }
    std::cout << "[App] Collision check: " << m_ui.infoText << "\n";
}

// ─── Batch production: finish current part, start next or end ───────
void App::finishCurrentPart() {
    captureIdleVisualState();
    m_laserCurrentPart++;
    m_ui.animCurrentPart = m_laserCurrentPart;
    m_laserWhipRemaining = std::max(0.0, m_laserWhipRemaining - m_program.lTotal());
    m_remainingWhipLength = m_laserWhipRemaining;
    m_laserAnimating = false;
    m_ui.animActive = false;
    m_renderer.setShowLaserBeam(false);
    m_renderer.clearCutContour();

    if (m_laserCurrentPart < m_ui.partCount && canContinueWithRemainingStock()) {
        m_resumeAvailable = true;
        m_resumeIsLaser = true;
        m_pendingNextPart = true;
        char buf[160];
        snprintf(buf, sizeof(buf), u8"Деталь %d/%d завершена. Заготовка осталась — можно продолжить",
                 m_laserCurrentPart, m_ui.partCount);
        m_ui.infoText = buf;
    } else {
        m_resumeAvailable = false;
        m_resumeIsLaser = false;
        m_pendingNextPart = false;
        char buf[128];
        if (m_laserCurrentPart < m_ui.partCount)
            snprintf(buf, sizeof(buf), u8"Заготовка закончилась после %d дет.", m_laserCurrentPart);
        else
            snprintf(buf, sizeof(buf), u8"Все %d дет. изготовлены", m_ui.partCount);
        m_ui.infoText = buf;
    }
}

void App::prepareNextLaserPart() {
    m_visualWhipLength = m_laserWhipRemaining;
    m_cycleBaseCarriageFeed = m_idleCarriageFeed;
    m_laserAnimating = true;
    m_ui.animActive = true;
    m_ui.animPaused = false;
    m_ui.animPauseMsg.clear();
    m_ui.animCurrentPart = m_laserCurrentPart;
    m_resumeAvailable = false;
    m_resumeIsLaser = false;
    m_pendingNextPart = false;
    m_hasIdleVisualState = false;

    m_laserAnimStep = 0;
    m_laserAnimProgress = 0.0;
    m_laserCutRotAngle = 0.0;
    m_laserCutFeedDelta = 0.0;
    m_laserCutFeedBase = 0.0;
    m_laserTotalFed = 0.0;
    m_benderAngle = 0.0;
    m_renderer.clearTubeTransform();
    m_renderer.setShowLaserBeam(false);
    m_renderer.clearCutContour();
    m_animStartTime = std::chrono::steady_clock::now();

    BendingProgram straightProg;
    straightProg.D = m_animationProgram.D;
    straightProg.R = m_animationProgram.R;
    straightProg.clampLength = m_animationProgram.clampLength;
    auto meshes = buildTubeGeometry(straightProg, m_laserWhipRemaining);
    m_renderer.uploadTubeSegments(meshes);

    if (!m_animationProgram.cutBefore.empty()) {
        m_laserAnimPhase = LCutBeforeFeed;
        char buf[128];
        snprintf(buf, sizeof(buf), u8"Деталь %d/%d — подача к резу (до)",
                 m_laserCurrentPart + 1, m_ui.partCount);
        m_ui.infoText = buf;
    } else {
        m_laserAnimPhase = LFeed;
        m_renderer.clearTubeTransform();
        rebuildLaserBendGeometry();
        char buf[128];
        snprintf(buf, sizeof(buf), u8"Деталь %d/%d — %s — шаг %d",
                 m_laserCurrentPart + 1, m_ui.partCount,
                 laserPhaseNames[LFeed], mapAnimatedStepIndex(0) + 1);
        m_ui.infoText = buf;
        if (!m_ui.autoFeed && m_animationProgram.steps[0].feedLength >= 0.01)
            pauseOnPhase(u8"Выполните подачу вручную");
    }
}

// ─── Continue animation (unpause manual step) ───────────────────────
void App::continueAnimation() {
    // v2.5: restart execution only through Machine unified path.
    if (m_machine.isProgramRunning()) return;

    if (!m_machine.isSimulated()) {
        m_machine.startSimulation();
    }

    m_ui.syncActiveProject(m_program);
    m_machine.startProgram(m_program, m_program.clampLength, m_ui.activeProjectName());
    m_hasIdleVisualState = false;
    m_ui.animPaused = false;
    m_ui.animPauseMsg.clear();
    m_ui.infoText = u8"Цикл перезапущен";
    m_resumeAvailable = false;
    m_resumeIsLaser = false;
    m_pendingNextPart = false;
}

// ─── Animation progress calculation ─────────────────────────────────
void App::updateAnimProgress() {
    if (m_machine.isProgramRunning()) {
        const int totalSteps = std::max(1, m_machine.totalSteps());
        const int stepIdx = std::clamp(m_machine.currentStep(), 0, totalSteps);
        m_ui.animStepIndex = (stepIdx < totalSteps) ? stepIdx : (totalSteps - 1);
        m_ui.animPhaseName = std::string(u8"Станок: ") + m_machine.phaseText();
        m_ui.animPhaseType = -1;
        m_ui.animProgressFrac = std::clamp((float)stepIdx / (float)totalSteps, 0.0f, 1.0f);
        return;
    }

    if (m_laserAnimating) {
        bool hasCB = !m_animationProgram.cutBefore.empty();
        bool hasCA = !m_animationProgram.cutAfter.empty();
        int cbPhases = hasCB ? 3 : 0;
        int bendPhases = (int)m_animationProgram.steps.size() * 7;
        int caPhases = hasCA ? 3 : 0; // CutAfterFeed + CutAfter + ReturnAfterCutAfter
        int phasesPerPart = cbPhases + bendPhases + caPhases;
        if (phasesPerPart == 0) phasesPerPart = 1;

        int phaseInPart = 0;
        if (m_laserAnimPhase <= LReturnFromCut && hasCB) {
            phaseInPart = m_laserAnimPhase;
        } else if (m_laserAnimPhase >= LFeed && m_laserAnimPhase <= LBenderReturn) {
            phaseInPart = cbPhases + m_laserAnimStep * 7 + (m_laserAnimPhase - LFeed);
        } else if (m_laserAnimPhase >= LCutAfterFeed) {
            phaseInPart = cbPhases + bendPhases + (m_laserAnimPhase - LCutAfterFeed);
        }

        int totalPhases = phasesPerPart * m_ui.animTotalParts;
        int completedPhases = m_laserCurrentPart * phasesPerPart + phaseInPart;
        double prog = m_laserAnimProgress;
        if (prog > 1.0) prog = 1.0;
        m_ui.animProgressFrac = (float)(completedPhases + prog) / (float)totalPhases;
        m_ui.animProgressFrac = std::clamp(m_ui.animProgressFrac, 0.0f, 1.0f);

        // Set step index and phase name for UI highlighting
        if (m_laserAnimPhase <= LReturnFromCut) {
            m_ui.animStepIndex = -2; // CutBefore
        } else if (m_laserAnimPhase >= LCutAfterFeed) {
            m_ui.animStepIndex = -3; // CutAfter
        } else {
            m_ui.animStepIndex = mapAnimatedStepIndex(m_laserAnimStep);
        }
        if (m_laserAnimPhase >= 0 && m_laserAnimPhase <= LReturnAfterCutAfter)
            m_ui.animPhaseName = laserPhaseNames[m_laserAnimPhase];

        // Map laser phase → column type for cell highlighting
        if (m_laserAnimPhase == LFeed || m_laserAnimPhase == LCutBeforeFeed ||
            m_laserAnimPhase == LCutAfterFeed || m_laserAnimPhase == LTubePush)
            m_ui.animPhaseType = 0;
        else if (m_laserAnimPhase == LRotation)
            m_ui.animPhaseType = 1;
        else if (m_laserAnimPhase == LBend || m_laserAnimPhase == LBenderReturn)
            m_ui.animPhaseType = 2;
        else
            m_ui.animPhaseType = -1;

    } else if (m_animating) {
        int phasesPerStep = 7;
        int totalPhases = (int)m_animationProgram.steps.size() * phasesPerStep;
        if (totalPhases == 0) totalPhases = 1;
        int completed = m_animStep * phasesPerStep + m_animPhase;
        double prog = m_animProgress;
        if (prog > 1.0) prog = 1.0;
        m_ui.animProgressFrac = (float)(completed + prog) / (float)totalPhases;
        m_ui.animProgressFrac = std::clamp(m_ui.animProgressFrac, 0.0f, 1.0f);

        // Set step index and phase name for UI highlighting
        m_ui.animStepIndex = mapAnimatedStepIndex(m_animStep);
        if (m_animPhase >= 0 && m_animPhase <= AnimBenderReturn)
            m_ui.animPhaseName = phaseNames[m_animPhase];

        // Map phase → column type for cell highlighting
        if (m_animPhase == AnimFeed || m_animPhase == AnimTubePush)
            m_ui.animPhaseType = 0; // feed column
        else if (m_animPhase == AnimRotation)
            m_ui.animPhaseType = 1; // rotation column
        else if (m_animPhase == AnimBend || m_animPhase == AnimBenderReturn)
            m_ui.animPhaseType = 2; // bend angle column
        else
            m_ui.animPhaseType = -1;
    }
}

// ─── Pause on manual phase ──────────────────────────────────────────
void App::pauseOnPhase(const char* msg) {
    m_ui.animPaused = true;
    m_ui.animPauseMsg = msg;
    m_ui.animShowPopup = true;
    m_ui.animPopupIsCollision = false;
}

// ─── Stop animation ─────────────────────────────────────────────────
void App::stopAnimation() {
    if (!m_animating && !m_laserAnimating && !m_machine.isProgramRunning()) return;

    captureIdleVisualState();
    // v2.5: STOP freezes current pose, resume of legacy timers is disabled.
    m_resumeAvailable = false;
    m_resumeIsLaser = false;
    m_pendingNextPart = false;
    m_animating = false;
    m_laserAnimating = false;
    if (m_machine.isProgramRunning()) m_machine.stopProgram();
    // Keep simulation backend alive to preserve last telemetry pose in scene.

    m_renderer.setShowLaserBeam(false);
    m_renderer.clearCutContour();
    m_ui.animActive = false;
    m_ui.animPaused = false;
    m_ui.animPauseMsg.clear();
    m_ui.infoText = u8"Цикл остановлен. Состояние сохранено";
}

void App::cancelAnimation() {
    if (!m_animating && !m_laserAnimating && !m_ui.animPaused && !m_resumeAvailable && !m_machine.isProgramRunning()) return;

    if (m_animating || m_laserAnimating || m_machine.isProgramRunning())
        captureIdleVisualState();

    m_animating = false;
    m_laserAnimating = false;
    if (m_machine.isProgramRunning()) m_machine.stopProgram();
    // Keep simulation backend alive to preserve last telemetry pose in scene.

    m_resumeAvailable = false;
    m_resumeIsLaser = false;
    m_pendingNextPart = false;
    m_renderer.setShowLaserBeam(false);
    m_renderer.clearCutContour();
    m_ui.animActive = false;
    m_ui.animPaused = false;
    m_ui.animPauseMsg.clear();
    m_ui.animStepIndex = -1;
    m_ui.animProgressFrac = 0.0f;
    m_ui.animPhaseName.clear();
    m_ui.animPhaseType = -1;
    // TZ: Cancel must freeze current pose, no jump back to origin.
    // Keep idle snapshot captured above and do not rebuild full geometry here.
    m_ui.infoText = u8"Выполнение отменено";
}

// ─── Program hash for auto-rebuild ──────────────────────────────────
size_t App::computeProgramHash() const {
    size_t h = 0;
    auto combine = [&](double v) {
        size_t bits;
        static_assert(sizeof(bits) == sizeof(v), "");
        std::memcpy(&bits, &v, sizeof(v));
        h ^= bits + 0x9e3779b9 + (h << 6) + (h >> 2);
    };
    combine(m_program.D);
    combine(m_program.R);
    combine(m_program.clampLength);
    combine(currentSceneWhipLength());
    combine((double)m_program.steps.size());
    for (auto& s : m_program.steps) {
        combine(s.feedLength);
        combine(s.rotation);
        combine(s.bendAngle);
    }
    // Include cut definitions
    combine((double)m_program.cutBefore.planes.size());
    for (auto& p : m_program.cutBefore.planes) {
        combine(p.nx); combine(p.ny); combine(p.nz);
        combine(p.offsetAlongAxis);
    }
    combine((double)m_program.cutAfter.planes.size());
    for (auto& p : m_program.cutAfter.planes) {
        combine(p.nx); combine(p.ny); combine(p.nz);
        combine(p.offsetAlongAxis);
    }
    return h;
}

// ─── Bender STL ─────────────────────────────────────────────────────
void App::loadBenderModel() {
    // Search for bender_models/ directory
    std::vector<fs::path> searchDirs = {
        m_exeDir / L"bender_models",
        m_exeDir / L".." / L"bender_models",
        m_exeDir / L".." / L".." / L"bender_models",
        m_exeDir / L".." / L".." / L".." / L"bender_models",
    };
    fs::path baseDir;
    for (auto& d : searchDirs) {
        if (fs::exists(d) && fs::is_directory(d)) { baseDir = d; break; }
    }
    if (baseDir.empty()) {
        std::cout << "[App] bender_models/ directory not found for bender model\n";
        m_ui.infoText = u8"Модель станка не найдена!";
        return;
    }

    // Load bend point model as the bender
    fs::path bendPointP = baseDir / L"\u0422\u043e\u0447\u043a\u0430 \u0433\u0438\u0431\u043a\u0438 R76 \u042438.stp";
    if (!fs::exists(bendPointP)) {
        std::cout << "[App] Bend point STP not found: " << bendPointP.u8string() << "\n";
        m_ui.infoText = u8"Модель точки гибки не найдена!";
        return;
    }
    m_bendPointPath = bendPointP;
    std::cout << "[App] Loading bender STP: " << bendPointP.u8string() << "\n";
    StlMesh mesh;
    if (tessellateStepCached(bendPointP, mesh, 0.5, 0.5)) {
        m_renderer.uploadBenderSTL(mesh);
        m_collisionMeshes.push_back(mesh);
        std::cout << "[App] Bender STP loaded: " << mesh.triangleCount << " triangles\n";
        std::cout << "[App] Bender bounds: X[" << mesh.minX << ".." << mesh.maxX
                  << "] Y[" << mesh.minY << ".." << mesh.maxY
                  << "] Z[" << mesh.minZ << ".." << mesh.maxZ << "]\n";

        // Auto-center camera on bender model
        float cx = (mesh.minX + mesh.maxX) * 0.5f;
        float cy = (mesh.minY + mesh.maxY) * 0.5f;
        float cz = (mesh.minZ + mesh.maxZ) * 0.5f;
        float sizeX = mesh.maxX - mesh.minX;
        float sizeY = mesh.maxY - mesh.minY;
        float sizeZ = mesh.maxZ - mesh.minZ;
        float maxSize = std::max({sizeX, sizeY, sizeZ});
        float dist = maxSize * 1.8f;
        if (dist < 500) dist = 500;
        m_renderer.setCamera(-90.0f, 20.0f, dist, cx, cy, cz);
    } else {
        std::cerr << "[App] Failed to tessellate bender STP\n";
        m_ui.infoText = u8"Ошибка тесселяции STP модели!";
    }
}

// ─── Logic 2 models (STP tessellation) ──────────────────────────────
void App::loadLogic2Models() {
    // Search for STP files in bender_models/ directory
    std::vector<fs::path> searchDirs = {
        m_exeDir / L"bender_models",
        m_exeDir / L".." / L"bender_models",
        m_exeDir / L".." / L".." / L"bender_models",
        m_exeDir / L".." / L".." / L".." / L"bender_models",
    };

    fs::path baseDir;
    for (auto& d : searchDirs) {
        if (fs::exists(d) && fs::is_directory(d)) { baseDir = d; break; }
    }
    if (baseDir.empty()) {
        std::cout << "[App] bender_models/ directory not found\n";
        return;
    }

    // Roller (Ролик.stp) — rotates around Z (bending axis)
    renderSplash(u8"Загрузка: Ролик...", 0.30f);
    fs::path rollerP = baseDir / L"\u0420\u043e\u043b\u0438\u043a.stp";
    if (fs::exists(rollerP)) {
        m_rollerPath = rollerP;
        StlMesh mesh;
        if (tessellateStepCached(rollerP, mesh, 0.5, 0.5)) {
            m_renderer.uploadRollerSTL(mesh);
            m_collisionMeshes.push_back(mesh);
            std::cout << "[App] Roller STP loaded: " << mesh.triangleCount << " triangles\n";
        }
    }

    // Clamp (Прижим.stp) — translates along Y
    renderSplash(u8"Загрузка: Прижим...", 0.40f);
    fs::path clampP = baseDir / L"\u041f\u0440\u0438\u0436\u0438\u043c.stp";
    if (fs::exists(clampP)) {
        m_clampPath = clampP;
        StlMesh mesh;
        if (tessellateStepCached(clampP, mesh, 0.5, 0.5)) {
            m_renderer.uploadClampSTL(mesh);
            m_collisionMeshes.push_back(mesh);
            std::cout << "[App] Clamp STP loaded: " << mesh.triangleCount << " triangles\n";
        }
    }

    // Static frame (Статичная.stp) — no movement
    renderSplash(u8"Загрузка: Станина...", 0.50f);
    fs::path staticP = baseDir / L"\u0421\u0442\u0430\u0442\u0438\u0447\u043d\u0430\u044f.stp";
    if (fs::exists(staticP)) {
        m_staticPath = staticP;
        StlMesh mesh;
        if (tessellateStepCached(staticP, mesh, 0.5, 0.5)) {
            m_renderer.uploadStaticSTL(mesh);
            m_floorZ = mesh.minZ;
            m_collisionMeshes.push_back(mesh);
            m_renderer.setFloorLevel(m_floorZ);
            std::cout << "[App] Static STP loaded: " << mesh.triangleCount
                      << " triangles, floorZ=" << m_floorZ << "\n";
        }
    }

    // Carriage (Каретка.stp) — translates along Y (tube feed)
    renderSplash(u8"Загрузка: Каретка...", 0.70f);
    fs::path carriageP = baseDir / L"\u041a\u0430\u0440\u0435\u0442\u043a\u0430.stp";
    if (fs::exists(carriageP)) {
        m_carriagePath = carriageP;
        StlMesh mesh;
        if (tessellateStepCached(carriageP, mesh, 0.5, 0.5)) {
            m_renderer.uploadCarriageSTL(mesh);
            m_collisionMeshes.push_back(mesh);
            std::cout << "[App] Carriage STP loaded: " << mesh.triangleCount << " triangles\n";
        }
    }

    m_logic2ModelsLoaded = !m_rollerPath.empty() || !m_clampPath.empty()
                        || !m_staticPath.empty() || !m_carriagePath.empty();
    if (m_logic2ModelsLoaded)
        std::cout << "[App] Logic 2 STP models loaded successfully\n";
}

// ─── Enhanced collision detection (Logic 2) ─────────────────────────
void App::checkCollisionsV2() {
    if (m_program.steps.empty()) {
        m_ui.collisionInfo = u8"Нет шагов для проверки";
        m_ui.infoText = m_ui.collisionInfo;
        clearCollisionPreview(false);
        return;
    }

    m_renderer.clearCollisionMarker();

    double D = m_program.D;
    double R = m_program.R;
    double tubeRadius = D / 2.0;

    // Exclusion zones (3 sections):
    // 1. Static cylinder at Point0 (Y from 0 to +R) — feed channel
    // 2. Torus section from 0° to step bendAngle
    // 3. Rotating cylinder at end of torus along tangent (clamp, length = clampLen)
    double exclusionRadius = D * 1.5 / 2.0;
    double clampExclLen = m_program.clampLength + tubeRadius;
    double pt0x = (double)(-(float)m_program.R);
    double pt0z = (double)PT0_Z;
    double bendCenterX = pt0x + R;
    // Use last step's bend angle (step N-1 is closest to Point0)
    int N = (int)m_program.steps.size();
    double lastBendAngleRad = (N > 0) ? m_program.steps[N-1].bendAngle * M_PI / 180.0 : 0.0;

    struct Vec3 { double x, y, z; };
    auto rotAround = [](Vec3 v, Vec3 axis, double angle) -> Vec3 {
        double c = std::cos(angle), s = std::sin(angle);
        double dot = v.x*axis.x + v.y*axis.y + v.z*axis.z;
        Vec3 cross = {axis.y*v.z - axis.z*v.y, axis.z*v.x - axis.x*v.z, axis.x*v.y - axis.y*v.x};
        return {v.x*c + cross.x*s + axis.x*dot*(1-c),
                v.y*c + cross.y*s + axis.y*dot*(1-c),
                v.z*c + cross.z*s + axis.z*dot*(1-c)};
    };

    // Build tube centerline starting at Point0 (matching tube_geometry.cpp)
    std::vector<Vec3> centerline;
    std::vector<int> centerlineStep;
    Vec3 pos = {pt0x, 0, pt0z};
    Vec3 fwd = {0, -1, 0};
    Vec3 up  = {0, 0, 1};
    Vec3 right = {1, 0, 0};  // up.cross(fwd) = Z cross (-Y) = +X

    centerline.push_back(pos);
    centerlineStep.push_back(-1);

    bool collisionFound = false;
    std::string collisionDetail;
    int collisionStep = -1;
    Vec3 collisionPoint = {};
    bool hasCollisionPoint = false;
    bool collisionWithFloor = false;

    // Pre-check: straight section between adjacent bends >= roller straight length
    for (int i = 1; i < N; i++) {
        auto& prev = m_program.steps[i-1];
        auto& cur  = m_program.steps[i];
        if (std::abs(prev.bendAngle) > 0.01 && std::abs(cur.bendAngle) > 0.01) {
            if (cur.feedLength < m_ui.toolingStraightLen_ - 0.01) {
                collisionFound = true;
                collisionStep = i;
                char buf[256];
                snprintf(buf, sizeof(buf),
                    u8"Шаг %d: прямой участок %.1f мм < мин. %.1f мм (длина ролика)",
                    i+1, cur.feedLength, m_ui.toolingStraightLen_);
                collisionDetail = buf;
                break;
            }
        }
    }

    // Iterate in REVERSE order to match tube_geometry.cpp
    // Step N-1 is closest to Point0, step 0 is furthest
    for (int i = N - 1; i >= 0; i--) {
        auto& s = m_program.steps[i];

        // 1. Bend (arc around up axis, center at pos + right*R)
        if (std::abs(s.bendAngle) > 0.01) {
            double angle = s.bendAngle * M_PI / 180.0;
            Vec3 center = {pos.x + right.x * R, pos.y + right.y * R, pos.z + right.z * R};
            Vec3 spoke = {pos.x - center.x, pos.y - center.y, pos.z - center.z};

            for (int j = 1; j <= 10; j++) {
                double t = angle * j / 10.0;
                Vec3 rSpoke = rotAround(spoke, up, t);
                Vec3 bp = {center.x + rSpoke.x, center.y + rSpoke.y, center.z + rSpoke.z};
                centerline.push_back(bp);
                centerlineStep.push_back(i);
            }

            // Update frame after bend
            Vec3 finalSpoke = rotAround(spoke, up, angle);
            pos = {center.x + finalSpoke.x, center.y + finalSpoke.y, center.z + finalSpoke.z};
            fwd = rotAround(fwd, up, angle);
            right = rotAround(right, up, angle);
            // up stays the same (rotation around up)
        }

        // 2. Rotation (twist around fwd axis)
        if (std::abs(s.rotation) > 0.01) {
            double rot = s.rotation * M_PI / 180.0;
            up = rotAround(up, fwd, rot);
            right = rotAround(right, fwd, rot);
        }

        // 3. Feed (straight segment in fwd direction)
        int feedSamples = std::max(2, (int)(s.feedLength / 10.0));
        for (int j = 1; j <= feedSamples; j++) {
            double t = (double)j / feedSamples;
            Vec3 p = {pos.x + fwd.x * s.feedLength * t,
                      pos.y + fwd.y * s.feedLength * t,
                      pos.z + fwd.z * s.feedLength * t};
            centerline.push_back(p);
            centerlineStep.push_back(i);
        }
        pos.x += fwd.x * s.feedLength;
        pos.y += fwd.y * s.feedLength;
        pos.z += fwd.z * s.feedLength;
    }

    // Check each centerline point for collisions
    for (size_t k = 0; k < centerline.size(); k++) {
        auto& p = centerline[k];

        // Skip points in exclusion zone:
        // 1. Static cylinder at Point0 (Y from 0 to +R)
        double rXZ = std::sqrt((p.x - pt0x) * (p.x - pt0x) + (p.z - pt0z) * (p.z - pt0z));
        if (rXZ < exclusionRadius && p.y >= 0.0 && p.y <= R)
            continue;
        // 2. Torus section (bent cylinder following bend arc)
        if (lastBendAngleRad > 0.01) {
            double relX = p.x - bendCenterX;
            double relY = p.y;
            double rXY = std::sqrt(relX * relX + relY * relY);
            if (rXY > 0.01) {
                double theta = std::atan2(-relY, -relX);
                if (theta < 0) theta += 2.0 * M_PI;
                theta = std::max(0.0, std::min(theta, lastBendAngleRad));
                double arcX = bendCenterX - R * std::cos(theta);
                double arcY = -R * std::sin(theta);
                double arcZ = pt0z;
                double ddx = p.x - arcX, ddy = p.y - arcY, ddz = p.z - arcZ;
                if (std::sqrt(ddx*ddx + ddy*ddy + ddz*ddz) < exclusionRadius)
                    continue;
            }
        }
        // 3. Rotating cylinder (clamp) at end of torus
        {
            double ct = std::cos(lastBendAngleRad), st = std::sin(lastBendAngleRad);
            double endX = bendCenterX - R * ct, endY = -R * st;
            double tx = st, ty = -ct;
            double rx = p.x - endX, ry = p.y - endY, rz = p.z - pt0z;
            double along = rx * tx + ry * ty;
            if (along >= 0.0 && along <= clampExclLen) {
                double dist2 = rx*rx + ry*ry + rz*rz - along*along;
                if (dist2 < exclusionRadius * exclusionRadius)
                    continue;
            }
        }

        // Check 1: Floor collision
        double tubeBottomZ = p.z - tubeRadius;
        if (tubeBottomZ < m_floorZ) {
            collisionFound = true;
            collisionStep = (k < centerlineStep.size()) ? centerlineStep[k] : collisionStep;
            collisionPoint = p;
            hasCollisionPoint = true;
            collisionWithFloor = true;
            char buf[256];
            snprintf(buf, sizeof(buf),
                u8"Столкновение с полом! Точка (%.1f, %.1f, %.1f), Z трубы=%.1f < пол=%.1f",
                p.x, p.y, p.z, tubeBottomZ, m_floorZ);
            collisionDetail = buf;
            break;
        }

        // Check 2: Self-intersection (skip nearby points on centerline)
        size_t skipN = std::max((size_t)5, (size_t)(D / 5.0) + 2);
        for (size_t j = 0; j + skipN < k; j++) {
            double dx = p.x - centerline[j].x;
            double dy = p.y - centerline[j].y;
            double dz = p.z - centerline[j].z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (dist < D && dist > 0.01) {
                collisionFound = true;
                collisionStep = (k < centerlineStep.size()) ? centerlineStep[k] : collisionStep;
                collisionPoint = p;
                hasCollisionPoint = true;
                char buf[256];
                snprintf(buf, sizeof(buf),
                    u8"Самопересечение трубы! Расстояние %.1f мм < D=%.1f мм",
                    dist, D);
                collisionDetail = buf;
                break;
            }
        }
        if (collisionFound) break;

        // Check 3: Collision with ALL loaded STL meshes (machine parts)
        for (auto& meshRef : m_collisionMeshes) {
            if (meshRef.vertices.empty()) continue;
            float px = (float)p.x, py = (float)p.y, pz = (float)p.z;
            float checkR = (float)tubeRadius;
            // Global AABB reject
            if (px < meshRef.minX - checkR || px > meshRef.maxX + checkR ||
                py < meshRef.minY - checkR || py > meshRef.maxY + checkR ||
                pz < meshRef.minZ - checkR || pz > meshRef.maxZ + checkR)
                continue;

            const float* verts = meshRef.vertices.data();
            for (int t = 0; t < meshRef.triangleCount; t++) {
                int base = t * 18;
                float ax = verts[base],    ay = verts[base+1],  az = verts[base+2];
                float bx = verts[base+6],  by = verts[base+7],  bz = verts[base+8];
                float cx = verts[base+12], cy = verts[base+13], cz = verts[base+14];
                // Per-triangle AABB
                if (px < std::min({ax,bx,cx}) - checkR || px > std::max({ax,bx,cx}) + checkR) continue;
                if (py < std::min({ay,by,cy}) - checkR || py > std::max({ay,by,cy}) + checkR) continue;
                if (pz < std::min({az,bz,cz}) - checkR || pz > std::max({az,bz,cz}) + checkR) continue;
                // Closest point on triangle
                float e0x=bx-ax, e0y=by-ay, e0z=bz-az;
                float e1x=cx-ax, e1y=cy-ay, e1z=cz-az;
                float vvx=px-ax, vvy=py-ay, vvz=pz-az;
                float d00=e0x*e0x+e0y*e0y+e0z*e0z;
                float d01=e0x*e1x+e0y*e1y+e0z*e1z;
                float d11=e1x*e1x+e1y*e1y+e1z*e1z;
                float d20=vvx*e0x+vvy*e0y+vvz*e0z;
                float d21=vvx*e1x+vvy*e1y+vvz*e1z;
                float denom=d00*d11-d01*d01;
                if (std::abs(denom)<1e-12f) continue;
                float inv=1.0f/denom;
                float ss=(d11*d20-d01*d21)*inv;
                float tt=(d00*d21-d01*d20)*inv;
                ss=std::max(0.0f,std::min(1.0f,ss));
                tt=std::max(0.0f,std::min(1.0f,tt));
                if(ss+tt>1.0f){float sc=1.0f/(ss+tt);ss*=sc;tt*=sc;}
                float cpx=ax+ss*e0x+tt*e1x;
                float cpy=ay+ss*e0y+tt*e1y;
                float cpz=az+ss*e0z+tt*e1z;
                float ddx=px-cpx,ddy=py-cpy,ddz=pz-cpz;
                float dist=std::sqrt(ddx*ddx+ddy*ddy+ddz*ddz);
                if (dist < checkR) {
                    collisionFound = true;
                    collisionStep = (k < centerlineStep.size()) ? centerlineStep[k] : collisionStep;
                    collisionPoint = p;
                    hasCollisionPoint = true;
                    char buf[256];
                    snprintf(buf, sizeof(buf),
                        u8"Столкновение со станком! Точка (%.1f, %.1f, %.1f), расстояние %.1f мм",
                        p.x, p.y, p.z, dist);
                    collisionDetail = buf;
                    break;
                }
            }
            if (collisionFound) break;
        }
        if (collisionFound) break;
    }

    if (collisionFound) {
        m_ui.collisionStepIdx = collisionStep;
        m_ui.collisionInfo = collisionDetail;
        m_ui.infoText = collisionDetail;
        m_ui.collisionChecked = false;
        showCollisionStepPreview(collisionStep);
        if (hasCollisionPoint)
            m_renderer.setCollisionMarker((float)collisionPoint.x, (float)collisionPoint.y,
                                          (float)collisionPoint.z, collisionWithFloor, m_floorZ);
    } else {
        m_ui.collisionStepIdx = -1;
        m_ui.collisionInfo.clear();
        m_ui.collisionChecked = true;
        clearCollisionPreview();
        char buf[128];
        snprintf(buf, sizeof(buf),
            u8"Столкновений не обнаружено (%d шагов, %d точек проверено)",
            (int)m_program.steps.size(), (int)centerline.size());
        m_ui.infoText = buf;
    }
    std::cout << "[App] CollisionV2: " << m_ui.collisionInfo << "\n";
}

// ─── Segment-segment squared distance (for capsule collision) ────────
static float segSegDist2(
    float p0x, float p0y, float p0z, float p1x, float p1y, float p1z,
    float q0x, float q0y, float q0z, float q1x, float q1y, float q1z)
{
    float dx = p1x-p0x, dy = p1y-p0y, dz = p1z-p0z;
    float ex = q1x-q0x, ey = q1y-q0y, ez = q1z-q0z;
    float rx = p0x-q0x, ry = p0y-q0y, rz = p0z-q0z;
    float a = dx*dx+dy*dy+dz*dz;
    float e2 = ex*ex+ey*ey+ez*ez;
    float f = dx*rx+dy*ry+dz*rz;
    float s, t;
    if (a <= 1e-8f && e2 <= 1e-8f) {
        s = t = 0.0f;
    } else if (a <= 1e-8f) {
        s = 0.0f;
        float g = ex*rx+ey*ry+ez*rz;
        t = std::max(0.0f, std::min(1.0f, g / e2));
    } else {
        float b = dx*ex+dy*ey+dz*ez;
        if (e2 <= 1e-8f) {
            t = 0.0f;
            s = std::max(0.0f, std::min(1.0f, -f / a));
        } else {
            float denom = a*e2 - b*b;
            float g = ex*rx+ey*ry+ez*rz;
            s = (denom > 1e-8f) ? std::max(0.0f, std::min(1.0f, (b*g - e2*f) / denom)) : 0.0f;
            t = (b*s + g) / e2;
            if (t < 0.0f) {
                t = 0.0f;
                s = std::max(0.0f, std::min(1.0f, -f / a));
            } else if (t > 1.0f) {
                t = 1.0f;
                s = std::max(0.0f, std::min(1.0f, (b - f) / a));
            }
        }
    }
    float cx = p0x+s*dx - q0x-t*ex;
    float cy = p0y+s*dy - q0y-t*ey;
    float cz = p0z+s*dz - q0z-t*ez;
    return cx*cx + cy*cy + cz*cz;
}

// ─── Squared distance from line segment to triangle ─────────────────
// Checks: segment vs 3 edges, segment piercing triangle, endpoints to face
static float segTriDist2(
    float p0x, float p0y, float p0z, float p1x, float p1y, float p1z,
    float ax, float ay, float az, float bx, float by, float bz,
    float cx, float cy, float cz)
{
    float best = FLT_MAX;
    // 1. Segment vs 3 triangle edges
    float d2;
    d2 = segSegDist2(p0x,p0y,p0z, p1x,p1y,p1z, ax,ay,az, bx,by,bz);
    if (d2 < best) best = d2;
    d2 = segSegDist2(p0x,p0y,p0z, p1x,p1y,p1z, bx,by,bz, cx,cy,cz);
    if (d2 < best) best = d2;
    d2 = segSegDist2(p0x,p0y,p0z, p1x,p1y,p1z, cx,cy,cz, ax,ay,az);
    if (d2 < best) best = d2;

    // Triangle edges & normal
    float e0x = bx-ax, e0y = by-ay, e0z = bz-az;
    float e1x = cx-ax, e1y = cy-ay, e1z = cz-az;
    float nx = e0y*e1z - e0z*e1y;
    float ny = e0z*e1x - e0x*e1z;
    float nz = e0x*e1y - e0y*e1x;
    float nlen2 = nx*nx+ny*ny+nz*nz;
    if (nlen2 > 1e-12f) {
        // 2. Check if segment pierces triangle
        float dp0 = (p0x-ax)*nx + (p0y-ay)*ny + (p0z-az)*nz;
        float dp1 = (p1x-ax)*nx + (p1y-ay)*ny + (p1z-az)*nz;
        if (dp0 * dp1 <= 0.0f) {
            float denom = dp0 - dp1;
            if (std::abs(denom) > 1e-8f) {
                float t = dp0 / denom;
                float ix = p0x + t*(p1x-p0x);
                float iy = p0y + t*(p1y-p0y);
                float iz = p0z + t*(p1z-p0z);
                float vx = ix-ax, vy = iy-ay, vz = iz-az;
                float d00 = e0x*e0x+e0y*e0y+e0z*e0z;
                float d01 = e0x*e1x+e0y*e1y+e0z*e1z;
                float d11 = e1x*e1x+e1y*e1y+e1z*e1z;
                float d20 = vx*e0x+vy*e0y+vz*e0z;
                float d21 = vx*e1x+vy*e1y+vz*e1z;
                float den = d00*d11-d01*d01;
                if (std::abs(den) > 1e-12f) {
                    float inv = 1.0f / den;
                    float u = (d11*d20-d01*d21)*inv;
                    float v = (d00*d21-d01*d20)*inv;
                    if (u >= -1e-6f && v >= -1e-6f && u+v <= 1.0f+1e-6f)
                        return 0.0f; // segment pierces triangle
                }
            }
        }
        // 3. Each segment endpoint to closest point on triangle face
        for (int ep = 0; ep < 2; ep++) {
            float px = ep==0 ? p0x : p1x;
            float py = ep==0 ? p0y : p1y;
            float pz = ep==0 ? p0z : p1z;
            float vx = px-ax, vy = py-ay, vz = pz-az;
            float d00 = e0x*e0x+e0y*e0y+e0z*e0z;
            float d01 = e0x*e1x+e0y*e1y+e0z*e1z;
            float d11 = e1x*e1x+e1y*e1y+e1z*e1z;
            float d20 = vx*e0x+vy*e0y+vz*e0z;
            float d21 = vx*e1x+vy*e1y+vz*e1z;
            float den = d00*d11-d01*d01;
            if (std::abs(den) < 1e-12f) continue;
            float inv = 1.0f / den;
            float ss = (d11*d20-d01*d21)*inv;
            float tt = (d00*d21-d01*d20)*inv;
            ss = std::max(0.0f, std::min(1.0f, ss));
            tt = std::max(0.0f, std::min(1.0f, tt));
            if (ss+tt > 1.0f) { float sc = 1.0f/(ss+tt); ss*=sc; tt*=sc; }
            float cpx = ax+ss*e0x+tt*e1x;
            float cpy = ay+ss*e0y+tt*e1y;
            float cpz = az+ss*e0z+tt*e1z;
            float ddx = px-cpx, ddy = py-cpy, ddz = pz-cpz;
            d2 = ddx*ddx+ddy*ddy+ddz*ddz;
            if (d2 < best) best = d2;
        }
    }
    return best;
}

// ─── Realtime collision check during animation ───────────────────────
// Returns true if a collision is detected between the animated tube
// geometry and the static machine mesh.  Called each frame from
// updateAnimation() after the partial geometry is rebuilt.
// Uses capsule-triangle distance (segment + radius) for precise detection.
bool App::checkAnimCollision() {
    if (m_collisionMeshes.empty() || m_program.steps.empty())
        return false;

    double D = m_program.D;
    double R = m_program.R;
    double tubeRadius = D / 2.0;

    // Exclusion zones (3 sections):
    // 1. Static cylinder at Point0 (Y from 0 to +R) — feed channel
    // 2. Torus from 0° to step bendAngle (where tube actually is)
    // 3. Rotating cylinder at end of torus (clamp)
    // NOTE: use STEP bend angle for exclusion, not m_benderAngle (roller may return
    //       but tube stays at full bend angle)
    double exclusionRadius = D * 1.5 / 2.0;
    double clampExclLen_d = m_program.clampLength + tubeRadius;
    double PT0_X_d = -m_program.R;
    static constexpr double PT0_Z_d = 397.0;
    double bendCenterX_d = PT0_X_d + R;
    auto& curStep = m_program.steps[m_animStep];
    double exclBendAngleRad = curStep.bendAngle * M_PI / 180.0;

    struct Vec3 { double x, y, z; };
    auto rotAround = [](Vec3 v, Vec3 axis, double angle) -> Vec3 {
        double c = std::cos(angle), s = std::sin(angle);
        double dot = v.x*axis.x + v.y*axis.y + v.z*axis.z;
        Vec3 cross = {axis.y*v.z - axis.z*v.y, axis.z*v.x - axis.x*v.z, axis.x*v.y - axis.y*v.x};
        return {v.x*c + cross.x*s + axis.x*dot*(1-c),
                v.y*c + cross.y*s + axis.y*dot*(1-c),
                v.z*c + cross.z*s + axis.z*dot*(1-c)};
    };

    // Build centerline for the *current* animation state
    // Matches tube_geometry.cpp: reverse-order iteration (bend → rotation → feed)
    std::vector<Vec3> centerline;
    Vec3 pos = {PT0_X_d, 0, PT0_Z_d};
    Vec3 fwd = {0, -1, 0};
    Vec3 up  = {0, 0, 1};
    Vec3 right = {1, 0, 0};

    centerline.push_back(pos);

    int lastStep = m_animStep;

    // Iterate in REVERSE order to match tube_geometry.cpp
    // Step lastStep (= m_animStep) is closest to Point0, step 0 is furthest
    for (int i = lastStep; i >= 0; i--) {
        auto s = m_program.steps[i]; // copy

        // For the current animation step, apply partial progress
        if (i == m_animStep) {
            switch (m_animPhase) {
            case AnimFeed: {
                double alreadyFed = (m_animStep > 0 && m_program.steps[m_animStep - 1].bendAngle > 0.01)
                    ? m_program.clampLength : 0.0;
                s.feedLength = alreadyFed + (s.feedLength - alreadyFed) * m_animProgress;
                s.bendAngle = 0;
                s.rotation = 0;
                break;
            }
            case AnimRotation:
                s.bendAngle = 0;
                s.rotation = s.rotation * m_animProgress;
                break;
            case AnimClampClose:
                s.bendAngle = 0;
                break;
            case AnimBend:
                s.bendAngle = s.bendAngle * m_animProgress;
                break;
            case AnimClampOpen:
            case AnimTubePush:
            case AnimBenderReturn:
                break;
            }
        }

        // 1. Bend (arc around up axis, center at pos + right*R)
        if (std::abs(s.bendAngle) > 0.01) {
            double angle = s.bendAngle * M_PI / 180.0;
            Vec3 center = {pos.x + right.x * R, pos.y + right.y * R, pos.z + right.z * R};
            Vec3 spoke = {pos.x - center.x, pos.y - center.y, pos.z - center.z};

            for (int j = 1; j <= 10; j++) {
                double t = angle * j / 10.0;
                Vec3 rSpoke = rotAround(spoke, up, t);
                Vec3 bp = {center.x + rSpoke.x, center.y + rSpoke.y, center.z + rSpoke.z};
                centerline.push_back(bp);
            }

            Vec3 finalSpoke = rotAround(spoke, up, angle);
            pos = {center.x + finalSpoke.x, center.y + finalSpoke.y, center.z + finalSpoke.z};
            fwd = rotAround(fwd, up, angle);
            right = rotAround(right, up, angle);
        }

        // 2. Rotation (twist around fwd)
        if (std::abs(s.rotation) > 0.01) {
            double rot = s.rotation * M_PI / 180.0;
            up = rotAround(up, fwd, rot);
            right = rotAround(right, fwd, rot);
        }

        // 3. Feed (straight in fwd direction)
        int feedSamples = std::max(2, (int)(s.feedLength / 10.0));
        for (int j = 1; j <= feedSamples; j++) {
            double t = (double)j / feedSamples;
            Vec3 p = {pos.x + fwd.x * s.feedLength * t,
                      pos.y + fwd.y * s.feedLength * t,
                      pos.z + fwd.z * s.feedLength * t};
            centerline.push_back(p);
        }
        pos.x += fwd.x * s.feedLength;
        pos.y += fwd.y * s.feedLength;
        pos.z += fwd.z * s.feedLength;
    }

    // ── Mark exclusion zone for each centerline point ──
    auto inExclZone = [&](const Vec3& p) -> bool {
        // 1. Static cylinder at Point0 (Y from 0 to +R)
        double rXZ = std::sqrt((p.x - PT0_X_d)*(p.x - PT0_X_d) + (p.z - PT0_Z_d)*(p.z - PT0_Z_d));
        if (rXZ < exclusionRadius && p.y >= 0.0 && p.y <= R)
            return true;
        // 2. Torus section (using full step bend angle)
        if (exclBendAngleRad > 0.01) {
            double relX = p.x - bendCenterX_d;
            double relY = p.y;
            double rXY = std::sqrt(relX*relX + relY*relY);
            if (rXY > 0.01) {
                double theta = std::atan2(-relY, -relX);
                if (theta < 0) theta += 2.0 * M_PI;
                theta = std::max(0.0, std::min(theta, exclBendAngleRad));
                double arcX = bendCenterX_d - R * std::cos(theta);
                double arcY = -R * std::sin(theta);
                double arcZ = PT0_Z_d;
                double ddx = p.x - arcX, ddy = p.y - arcY, ddz = p.z - arcZ;
                if (std::sqrt(ddx*ddx + ddy*ddy + ddz*ddz) < exclusionRadius)
                    return true;
            }
        }
        // 3. Rotating cylinder (clamp) at end of torus
        {
            double ct = std::cos(exclBendAngleRad), st = std::sin(exclBendAngleRad);
            double endX = bendCenterX_d - R * ct, endY = -R * st;
            double tx = st, ty = -ct;
            double rx = p.x - endX, ry = p.y - endY, rz = p.z - PT0_Z_d;
            double along = rx * tx + ry * ty;
            if (along >= 0.0 && along <= clampExclLen_d) {
                double dist2 = rx*rx + ry*ry + rz*rz - along*along;
                if (dist2 < exclusionRadius * exclusionRadius)
                    return true;
            }
        }
        return false;
    };

    // ── Capsule-based collision: check each segment vs meshes ──
    float checkR = (float)tubeRadius;
    float checkR2 = checkR * checkR;

    for (size_t k = 0; k + 1 < centerline.size(); k++) {
        auto& pa = centerline[k];
        auto& pb = centerline[k + 1];

        // Skip segment if either endpoint is in exclusion zone
        if (inExclZone(pa) || inExclZone(pb))
            continue;

        float s0x = (float)pa.x, s0y = (float)pa.y, s0z = (float)pa.z;
        float s1x = (float)pb.x, s1y = (float)pb.y, s1z = (float)pb.z;

        // Segment AABB
        float sMinX = std::min(s0x, s1x), sMaxX = std::max(s0x, s1x);
        float sMinY = std::min(s0y, s1y), sMaxY = std::max(s0y, s1y);
        float sMinZ = std::min(s0z, s1z), sMaxZ = std::max(s0z, s1z);

        // Floor collision (any point along segment)
        if (sMinZ - checkR < m_floorZ) {
            m_ui.collisionStepIdx = m_animStep;
            m_ui.selectedStep = m_animStep;
            char buf[256];
            snprintf(buf, sizeof(buf),
                u8"Столкновение с полом! Сегмент (%.1f,%.1f,%.1f)-(%.1f,%.1f,%.1f)",
                pa.x, pa.y, pa.z, pb.x, pb.y, pb.z);
            m_ui.collisionInfo = buf;
            m_ui.infoText = buf;
            m_renderer.setCollisionMarker((float)pa.x, (float)pa.y, (float)pa.z, true, m_floorZ);
            return true;
        }

        // Capsule vs triangle mesh
        for (auto& meshRef : m_collisionMeshes) {
            if (meshRef.vertices.empty()) continue;
            // Global AABB reject per mesh (segment AABB + radius)
            if (sMaxX < meshRef.minX - checkR || sMinX > meshRef.maxX + checkR ||
                sMaxY < meshRef.minY - checkR || sMinY > meshRef.maxY + checkR ||
                sMaxZ < meshRef.minZ - checkR || sMinZ > meshRef.maxZ + checkR)
                continue;

            const float* verts = meshRef.vertices.data();
            for (int t = 0; t < meshRef.triangleCount; t++) {
                int base = t * 18;
                float ax = verts[base],      ay = verts[base + 1],  az = verts[base + 2];
                float bx = verts[base + 6],  by = verts[base + 7],  bz = verts[base + 8];
                float cx = verts[base + 12], cy = verts[base + 13], cz = verts[base + 14];

                // Per-triangle AABB rejection against segment AABB
                float tMinX = std::min({ax,bx,cx}), tMaxX = std::max({ax,bx,cx});
                float tMinY = std::min({ay,by,cy}), tMaxY = std::max({ay,by,cy});
                float tMinZ = std::min({az,bz,cz}), tMaxZ = std::max({az,bz,cz});
                if (sMaxX < tMinX - checkR || sMinX > tMaxX + checkR) continue;
                if (sMaxY < tMinY - checkR || sMinY > tMaxY + checkR) continue;
                if (sMaxZ < tMinZ - checkR || sMinZ > tMaxZ + checkR) continue;

                // Precise segment-triangle squared distance
                float d2 = segTriDist2(s0x,s0y,s0z, s1x,s1y,s1z,
                                       ax,ay,az, bx,by,bz, cx,cy,cz);
                if (d2 < checkR2) {
                    float dist = std::sqrt(d2);
                    m_ui.collisionStepIdx = m_animStep;
                    m_ui.selectedStep = m_animStep;
                    char buf[256];
                    snprintf(buf, sizeof(buf),
                        u8"Столкновение со станком! Расстояние %.1f мм < R=%.1f мм",
                        dist, checkR);
                    m_ui.collisionInfo = buf;
                    m_ui.infoText = buf;
                    m_renderer.setCollisionMarker((float)pa.x, (float)pa.y, (float)pa.z, false, m_floorZ);
                    return true;
                }
            }
        }
    }

    return false;
}

// ─── Splash screen during loading ───────────────────────────────────
void App::loadSplashImage() {
#ifdef _WIN32
    // Use GDI+ to load JPEG/PNG image as OpenGL texture
    Gdiplus::GdiplusStartupInput gdiplusStartupInput;
    ULONG_PTR gdiplusToken;
    Gdiplus::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, nullptr);

    // Search for Loading.jpg in images/ directory
    std::vector<fs::path> candidates = {
        m_exeDir / L"images" / L"Loading.jpg",
        m_exeDir / L"images" / L"Loading.png",
        m_exeDir / L".." / L"images" / L"Loading.jpg",
        m_exeDir / L".." / L"images" / L"Loading.png",
        m_exeDir / L".." / L".." / L"images" / L"Loading.jpg",
        m_exeDir / L".." / L".." / L"images" / L"Loading.png",
    };

    fs::path imgPath;
    for (auto& p : candidates) {
        if (fs::exists(p)) { imgPath = p; break; }
    }
    if (imgPath.empty()) {
        std::cout << "[App] Loading.jpg not found, splash without background\n";
        Gdiplus::GdiplusShutdown(gdiplusToken);
        return;
    }

    Gdiplus::Bitmap* bitmap = new Gdiplus::Bitmap(imgPath.wstring().c_str());
    if (bitmap->GetLastStatus() != Gdiplus::Ok) {
        std::cerr << "[App] Failed to load splash image\n";
        delete bitmap;
        Gdiplus::GdiplusShutdown(gdiplusToken);
        return;
    }

    int w = (int)bitmap->GetWidth();
    int h = (int)bitmap->GetHeight();
    Gdiplus::BitmapData bmpData;
    Gdiplus::Rect rect(0, 0, w, h);
    bitmap->LockBits(&rect, Gdiplus::ImageLockModeRead, PixelFormat32bppARGB, &bmpData);

    // Convert BGRA to RGBA
    std::vector<unsigned char> rgba(w * h * 4);
    for (int y = 0; y < h; y++) {
        unsigned char* src = (unsigned char*)bmpData.Scan0 + y * bmpData.Stride;
        unsigned char* dst = rgba.data() + y * w * 4;
        for (int x = 0; x < w; x++) {
            dst[x*4+0] = src[x*4+2]; // R
            dst[x*4+1] = src[x*4+1]; // G
            dst[x*4+2] = src[x*4+0]; // B
            dst[x*4+3] = src[x*4+3]; // A
        }
    }
    bitmap->UnlockBits(&bmpData);
    delete bitmap;
    Gdiplus::GdiplusShutdown(gdiplusToken);

    glGenTextures(1, &m_splashTex);
    glBindTexture(GL_TEXTURE_2D, m_splashTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
    m_splashTexW = w;
    m_splashTexH = h;
    std::cout << "[App] Splash image loaded: " << w << "x" << h << "\n";
#endif
}

void App::destroySplashImage() {
    if (m_splashTex) {
        glDeleteTextures(1, &m_splashTex);
        m_splashTex = 0;
    }
}

void App::renderSplash(const char* status, float progress) {
    if (!m_window) return;

    // Process pending OS events to keep window responsive
    glfwPollEvents();

    int fbW, fbH;
    glfwGetFramebufferSize(m_window, &fbW, &fbH);
    if (fbW <= 0 || fbH <= 0) return;

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Full-screen splash window
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2((float)fbW, (float)fbH));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.06f, 0.06f, 0.07f, 1.0f));
    ImGui::Begin("##splash", nullptr,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoDocking |
        ImGuiWindowFlags_NoBringToFrontOnFocus);

    // Background image (centered, aspect-fit)
    if (m_splashTex) {
        float imgAR = (float)m_splashTexW / (float)m_splashTexH;
        float winAR = (float)fbW / (float)fbH;
        float drawW, drawH;
        if (imgAR > winAR) {
            drawW = (float)fbW;
            drawH = drawW / imgAR;
        } else {
            drawH = (float)fbH;
            drawW = drawH * imgAR;
        }
        float ox = ((float)fbW - drawW) * 0.5f;
        float oy = ((float)fbH - drawH) * 0.5f;
        ImGui::SetCursorPos(ImVec2(ox, oy));
        ImGui::Image((ImTextureID)(intptr_t)m_splashTex, ImVec2(drawW, drawH));
    }

    // Title text — centered upper area
    {
        const char* title = u8"3\u0445 \u043e\u0441\u0435\u0432\u043e\u0439 \u0442\u0440\u0443\u0431\u043e\u0433\u0438\u0431 \u0441 \u0444\u0443\u043d\u043a\u0446\u0438\u0435\u0439 \u043b\u0430\u0437\u0435\u0440\u043d\u043e\u0439 \u0440\u0435\u0437\u043a\u0438";
        ImVec2 ts = ImGui::CalcTextSize(title);
        ImGui::SetCursorPos(ImVec2(((float)fbW - ts.x) * 0.5f, (float)fbH * 0.15f));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.9f, 0.9f, 0.95f, 1.0f));
        ImGui::Text("%s", title);
        ImGui::PopStyleColor();
    }

    // Version sub-title
    {
        const char* ver = "TubeBender v2.2";
        ImVec2 vs = ImGui::CalcTextSize(ver);
        ImGui::SetCursorPos(ImVec2(((float)fbW - vs.x) * 0.5f, (float)fbH * 0.15f + 24));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 0.5f, 0.55f, 1.0f));
        ImGui::Text("%s", ver);
        ImGui::PopStyleColor();
    }

    // Progress bar at the bottom
    float barH = 6.0f;
    float barY = (float)fbH - 80.0f;
    float barMargin = (float)fbW * 0.15f;
    float barW = (float)fbW - barMargin * 2.0f;

    // Background bar
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(ImVec2(barMargin, barY), ImVec2(barMargin + barW, barY + barH),
        IM_COL32(50, 50, 55, 255), 3.0f);
    // Filled portion
    float fillW = barW * progress;
    if (fillW > 0) {
        dl->AddRectFilled(ImVec2(barMargin, barY), ImVec2(barMargin + fillW, barY + barH),
            IM_COL32(14, 99, 156, 255), 3.0f);
    }

    // Status text centered below progress bar
    {
        ImVec2 ss = ImGui::CalcTextSize(status);
        ImGui::SetCursorPos(ImVec2(((float)fbW - ss.x) * 0.5f, barY + barH + 12.0f));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.65f, 0.65f, 0.7f, 1.0f));
        ImGui::Text("%s", status);
        ImGui::PopStyleColor();
    }

    // Percentage text
    {
        char pctBuf[16];
        snprintf(pctBuf, sizeof(pctBuf), "%d%%", (int)(progress * 100));
        ImVec2 ps = ImGui::CalcTextSize(pctBuf);
        ImGui::SetCursorPos(ImVec2(((float)fbW - ps.x) * 0.5f, barY - 20.0f));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.7f, 0.7f, 0.75f, 1.0f));
        ImGui::Text("%s", pctBuf);
        ImGui::PopStyleColor();
    }

    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);

    ImGui::Render();
    glViewport(0, 0, fbW, fbH);
    glClearColor(0.06f, 0.06f, 0.07f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(m_window);
}
