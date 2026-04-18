#pragma once
// ─────────────────────────────────────────────────────────────────────
// app.h — Application orchestrator
// ─────────────────────────────────────────────────────────────────────
#include "bending_program.h"
#include "renderer.h"
#include "ui.h"
#include "stl_loader.h"
#include "step_tessellator.h"
#include "tube_geometry.h"
#include "machine.h"
#include "scene_animator.h"
#include <string>
#include <filesystem>
#include <chrono>

struct GLFWwindow; // forward decl

class App {
public:
    bool init(int width, int height, GLFWwindow* window);
    void shutdown();

    // Per-frame
    void processInput();   // 3D camera from ImGui IO (orbit/pan/zoom)
    void update();
    void render(int width, int height);
    void drawUI();

    // Input forwarding from GLFW
    void onResize(int w, int h);

    // UI scale access for main.cpp font rebuild
    bool  uiNeedsFontRebuild() const { return m_ui.needFontRebuild; }
    float uiScale() const { return m_ui.uiScale; }
    void  clearUiFontRebuild() { m_ui.needFontRebuild = false; }
    void  saveLayout() { m_ui.saveLayoutNow(); }

    // Animation control (for manual relay testing)
    void triggerClampAnimation() { m_clampState = ClampClosing; m_clampTargetOffset = 0.0; }
    void triggerUnclampAnimation() { m_clampState = ClampOpening; m_clampTargetOffset = -CLAMP_TRAVEL; }

private:
    void rebuildGeometry();
    void rebuildGeometryUpToStep(int maxStep);
    void startAnimation();
    void updateAnimation();
    void importSTP(const std::string& path);
    void exportSTP(const std::string& path);
    void saveJSON(const std::string& path);
    void loadJSON(const std::string& path);
    void exportIGES(const std::string& path);
    void checkCollisions();
    void showCollisionStepPreview(int stepIdx);
    void clearCollisionPreview(bool rebuildFullGeometry = true);

    void loadBenderModel();
    void loadLogic2Models();
    void checkCollisionsV2(); // enhanced collision with floor + machine
    bool checkAnimCollision(); // realtime collision during animation, returns true if collision

    BendingProgram m_program;
    Renderer       m_renderer;
    UI             m_ui;
    UICallbacks    m_callbacks;

    // Framebuffer size
    int m_fbW = 1400, m_fbH = 850;

    // Paths
    std::filesystem::path m_exeDir;      // directory containing the exe
    std::filesystem::path m_benderPath;  // path to bender STL model

    // Logic 2 model paths
    std::filesystem::path m_rollerPath;
    std::filesystem::path m_clampPath;
    std::filesystem::path m_staticPath;
    std::filesystem::path m_carriagePath;
    std::filesystem::path m_bendPointPath;
    bool m_logic2ModelsLoaded = false;

    // Clamp animation state
    double m_clampOffset = 0.0;  // 0=clamped, -150=released
    double m_clampTargetOffset = 0.0;
    static constexpr double CLAMP_TRAVEL = 150.0; // mm
    static constexpr double CLAMP_SPEED = 600.0;  // mm/sec
    enum ClampState { ClampClosed, ClampOpening, ClampOpen, ClampClosing };
    ClampState m_clampState = ClampClosed;

    // Floor collision Z (bottom of static model)
    float m_floorZ = 0.0f;

    // ALL loaded STL meshes for collision detection (bender + roller + clamp + static)
    std::vector<StlMesh> m_collisionMeshes;

    // Animation state — 7-phase per step:
    // 0=Feed, 1=Rotation, 2=ClampClose, 3=Bend, 4=ClampOpen, 5=TubePush, 6=BenderReturn
    enum AnimPhase { AnimFeed=0, AnimRotation=1, AnimClampClose=2, AnimBend=3, AnimClampOpen=4, AnimTubePush=5, AnimBenderReturn=6 };
    static constexpr double ANIM_FEED_SPEED   = 300.0; // mm/sec
    static constexpr double ANIM_BEND_SPEED   =  60.0; // deg/sec
    static constexpr double ANIM_RETURN_SPEED  = 120.0; // deg/sec
    static constexpr double ANIM_ROT_SPEED    = 120.0; // deg/sec

    bool m_animating = false;
    int  m_animStep = 0;           // current step being animated (0..N-1)
    int  m_animPhase = 0;          // AnimPhase enum
    double m_animProgress = 0.0;   // 0..1 within current phase
    double m_benderAngle = 0.0;    // current angle of bender STL model (degrees)
    std::chrono::steady_clock::time_point m_animStartTime;

    // Laser animation (10-phase cycle)
    enum LaserAnimPhase {
        LCutBeforeFeed = 0, LCutBefore,
        LReturnFromCut,
        LFeed, LRotation, LClampClose, LBend, LClampOpen, LTubePush, LBenderReturn,
        LCutAfterFeed, LCutAfter,
        LReturnAfterCutAfter
    };
    static constexpr double ANIM_CUT_ROT_SPEED = 60.0; // deg/sec

    bool   m_laserAnimating = false;
    int    m_laserAnimStep = 0;
    int    m_laserAnimPhase = 0;
    double m_laserAnimProgress = 0.0;
    double m_laserCutRotAngle = 0.0;
    double m_laserCutFeedDelta = 0.0;
    double m_laserCutFeedBase = 0.0;
    double m_laserTotalFed = 0.0;
    double m_laserMaxDeltaCB = 0.0;  // max(contourY-offset) for CutBefore
    double m_laserMaxDeltaCA = 0.0;  // max(contourY-offset) for CutAfter
    double m_laserCutBeforeFeedDist = 0.0; // feed distance during CutBefore (laserOffsetY + maxDeltaCB)

    // Batch production state
    int    m_laserCurrentPart = 0;
    double m_laserWhipRemaining = 0.0;
    double m_visualWhipLength = 0.0;
    double m_remainingWhipLength = 0.0;
    int    m_completedParts = 0;
    bool   m_resumeAvailable = false;
    bool   m_resumeIsLaser = false;
    bool   m_pendingNextPart = false;
    bool   m_hasIdleVisualState = false;
    double m_idleCarriageFeed = 0.0;
    double m_idleChuckRotation = 0.0;
    double m_cycleBaseCarriageFeed = 0.0;
    bool   m_idleTubeTransform = false;
    double m_idleTubeRotY = 0.0;
    double m_idleTubeFeedY = 0.0;
    
    // ⚠️ КРИТИЧЕСКИЙ ФИХ: Флаг для сохранения tubeTransform между кадрами
    // Используется для предотвращения микротелепортации при переходе UnclampWait1 -> Feed
    bool   m_preserveTubeTransformNextFrame = false;
    double m_preservedTubeTransformFeed = 0.0;
    
    // ⚠️ ИНТЕРПОЛЯЦИЯ КООРДИНАТ: Плавный переход между фазами вместо скачков
    // Это копия v2.2 механизма - вместо прямого использования actualZ/actualC/actualB,
    // мы интерполируем от начального значения к целевому в течение фазы
    MachPhase  m_lastRenderedPhase = MachPhase::Idle;
    double     m_phaseInterpolProgress = 0.0;  // 0..1 within current phase transition
    double     m_interpStartZ = 0.0;   // машинные координаты в начале фазы
    double     m_interpStartC = 0.0;
    double     m_interpStartB = 0.0;
    double     m_interpTargetZ = 0.0;  // целевые координаты в конце фазы  
    double     m_interpTargetC = 0.0;
    double     m_interpTargetB = 0.0;
    
    // ⚠️ КРИТИЧЕСКИЙ ФИХ: Сохранение последнего отрендеренного значения
    // Чтобы при переходе между фазами не было скачков координат
    // startZ должен быть renderZ предыдущей фазы, а не actualZ машины!
    double     m_lastRenderZ = 0.0;
    double     m_lastRenderC = 0.0;
    double     m_lastRenderB = 0.0;
    
    // ⚠️ ОТСЛЕЖИВАНИЕ ВРЕМЕНИ для корректного delta-time в интерполяции
    std::chrono::steady_clock::time_point m_lastFrameTime;
    double m_frameDtAccum = 0.0;  // накопленный delta-time
    
    // ⚠️ ДИНАМИЧЕСКИЕ ДЛИТЕЛЬНОСТИ (вычисляются на основе distance/speed, не фиксированы!)
    // УДАЛЕНЫ: PHASE_DURATION_* константы теперь заменены на формулу:
    // phaseDuration = movement_distance / movement_speed
    // Это обеспечивает синхронизацию анимации с реальным движением машины
    // v2.2 пример:
    //   Feed:     phaseDuration = feedLength / 300.0   mm/s
    //   Rotation: phaseDuration = rotation / 120.0      deg/s  
    //   Bend:     phaseDuration = bendAngle / 60.0      deg/s
    //   Clearance:phaseDuration = clampLength / 300.0   mm/s
    
    BendingProgram m_animationProgram;
    std::vector<int> m_animationStepMap;

    void startLaserAnimation();
    void updateLaserAnimation();
    void advanceLaserStep();
    void rebuildLaserBendGeometry();
    void prepareNextLaserPart();
    void finishCurrentPart();
    void continueAnimation();
    void updateAnimProgress();
    void pauseOnPhase(const char* msg);
    void stopAnimation();
    void cancelAnimation();
    void prepareAnimationProgram();
    int mapAnimatedStepIndex(int stepIdx) const;
    double currentSceneWhipLength() const;
    double computeNormalCarriageFeed() const;
    double computeAbsoluteCarriageFeed() const;
    double computeCarriageRenderFeed(double logicalFeed) const;
    double computeCurrentChuckRotation() const;
    void captureIdleVisualState();
    bool canContinueWithRemainingStock() const;
    void updateContinueUiState();
    bool isPedalHeld() const;
    void clearStoppedCycleSnapshot();
    double machineFeedPosition(const TeensyStatus& st) const;
    double machineChuckPosition(const TeensyStatus& st) const;
    void syncMachineVisualState(const TeensyStatus& st);
    
    // ⚠️ ИНТЕРПОЛЯЦИЯ: Обновление прогресса интерполяции между фазами
    // Вместо скачков координат используем плавный переход 0.0 -> 1.0
    void updatePhaseInterpolation(double dt, MachPhase currentPhase, int stepIdx,
                                   double currentZ, double currentC, double currentB,
                                   const BendingProgram& program, double baseFeedAcc, double baseRotAcc);

    // Auto-rebuild: snapshot of program state
    size_t m_lastProgramHash = 0;
    size_t computeProgramHash() const;

    // Machine runtime (Teensy USB-Serial)
    Machine m_machine;
    // ⚠️ ЕДИНАЯ логика визуализации: sim и hw — один источник истины
    SceneAnimator m_sceneAnimator;
    bool m_machineSceneValid = false;
    MachPhase m_machineScenePhase = MachPhase::Idle;
    int m_machineSceneStep = -1;

    // Splash screen during loading
    GLFWwindow* m_window = nullptr;
    unsigned int m_splashTex = 0;
    int m_splashTexW = 0, m_splashTexH = 0;
    void loadSplashImage();
    void renderSplash(const char* status, float progress);
    void destroySplashImage();
};
