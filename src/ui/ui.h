#pragma once
// ─────────────────────────────────────────────────────────────────────
// ui.h — Dear ImGui UI panels for TubeBender v2.0
//   Uses ImGui Docking for professional window management.
// ─────────────────────────────────────────────────────────────────────
#include "bending_program.h"
#include "machine.h"
#include <string>
#include <vector>
#include <functional>

// Forward declaration
class App;

// Callbacks from UI to application
struct UICallbacks {
    std::function<void(const std::string&)> onImportSTP;
    std::function<void(const std::string&)> onExportSTP;
    std::function<void(const std::string&)> onSaveJSON;
    std::function<void(const std::string&)> onLoadJSON;
    std::function<void()>                   onRebuildGeometry;
    std::function<void()>                   onVisualize;
    std::function<void()>                   onLaserVisualize;
    std::function<void(const std::string&)> onExportIGES;
    std::function<void()>                   onCheckCollisions;
    std::function<void()>                   onAnimContinue;
    std::function<void()>                   onStop;
    std::function<void()>                   onCancel;
    // Machine
    std::function<void(const std::string&)> onMachineConnect;
    std::function<void()>                   onMachineDisconnect;
    std::function<void()>                   onMachineRun;
    std::function<void()>                   onMachineStop;
    std::function<void()>                   onMachineHome;
    std::function<void()>                   onMachineHomeC;
    std::function<void()>                   onMachineServoOn;
    std::function<void()>                   onMachineServoOff;
    std::function<void()>                   onMachineEstop;
    std::function<void()>                   onMachineZero;
    std::function<void(double,double,int)>  onMachineCalibrate; // (zMm, cDeg, bendDeg)
};

// Persistent settings saved to settings.json
struct AppSettings {
    std::string benderModelPath;        // STL model of bender
    std::string benderAxisModelPath;    // STP: axis + bend point
    std::string rollerModelPath;        // STL: roller (movable)
    std::string carriageModelPath;      // STL: carriage (movable)
    std::string clampModelPath;         // STL: clamp (movable)
    std::string chuckModelPath;         // STL: chuck (movable)
    std::string staticModelPath;        // STL: static parts
    std::string lastPreset;

    // Global tooling parameters (machine state)
    double toolingD = 38;
    double toolingR = 76;
    double toolingClampLength = 60;
    double toolingStraightLen = 60;   // min straight section (roller)
    double toolingFeedSpeed   = 300.0;
    double toolingRotSpeed    = 120.0;
    double toolingBendSpeed   = 60.0;
    double laserOffsetY = 60;
    double laserGapZ = 5;
    double whipLength = 6000;
    int partCount = 1;
    double springBackCoeff = 1.0;
    int steelGradeIdx = 0;

    // Logic 2 mode (new models + collision)
    bool useLogic2 = false;
    bool backfaceCulling = false;
    float sceneRenderScale = 0.65f;

    // Last known machine axis positions (persistent — survive EXE restart).
    // Written at program finish / app shutdown, restored at startup so that
    // the 3D scene and axis indicators don't snap back to zero.
    double lastAxisBend     = 0.0;
    double lastAxisCarriage = 0.0;
    double lastAxisChuck    = 0.0;

    // Sidebar state
    bool osnastkaSidebarOpen = true;
    bool spravkaSidebarOpen = false;

    // Window visibility (persisted)
    bool showScene    = true;
    bool showProgram  = true;
    bool showOsnastka = true;
    bool showDebug    = false;
    bool showControl  = true;

    void loadFromFile(const std::string& path);
    void saveToFile(const std::string& path) const;
};

// Project workspace
struct Project {
    std::string name = u8"\u041d\u043e\u0432\u044b\u0439 \u043f\u0440\u043e\u0435\u043a\u0442";
    std::string filePath;       // empty = unsaved
    BendingProgram program;
    double laserOffsetY = 60;
    double laserGapZ = 5;
    double whipLength = 6000;
    int partCount = 1;
    double springBackCoeff = 1.0;
    int steelGradeIdx = 0;
    std::string presetName;
    int selectedStep = -1;
};

class UI {
public:
    void init(const std::string& exeDir);
    void draw(BendingProgram& prog, UICallbacks& cb, int fbW, int fbH,
             unsigned int sceneTexId);
    void saveLayoutNow();  // manually save imgui.ini (wide path)

    // Toggles readable by renderer
    bool showBender    = true;
    bool benderOpaque  = true;  // true = opaque, false = semi-transparent
    bool showExclZone  = false; // show exclusion zone cylinder in scene
    bool showLaserHead = false;
    int  selectedStep  = -1;

    // Multi-select checkboxes for steps
    std::vector<char> stepChecked;  // char instead of bool to avoid vector<bool> proxy

    // Laser-specific params (shared with app)
    double laserOffsetY = 60;
    double laserGapZ    = 5;

    // Whip (bar stock) and part count
    double whipLength = 6000;
    int    partCount  = 1;

    // Status text
    std::string infoText;

    // UI scaling
    float  uiScale         = 1.0f;
    bool   touchMode       = false;
    bool   needFontRebuild = false;

    // Semi-automatic mode checkboxes
    bool autoFeed     = true;
    bool autoRotation = true;
    bool autoClamp    = true;

    // Mode: 0=Гибка, 1=Гибка+Лазер, 2=Ручной
    int animMode = 0;

    // Preset name
    std::string presetName = u8"\u044438*1,5 R76 08\u043f\u0441";

    // Window visibility toggles
    bool showProgramWin  = true;
    bool showSceneWin    = true;
    bool showOsnastkaWin = true;
    bool showDebugWin    = false;
    bool showControlWin  = true;
    bool showConnectionWin = false;

    // 3D scene viewport — content size for FBO allocation
    float sceneContentW = 700, sceneContentH = 400;
    bool  sceneHovered = false;

    // Animation state (written by App each frame)
    bool        animActive       = false;
    bool        animPaused       = false;
    std::string animPauseMsg;
    bool        animShowPopup    = false;  // trigger modal popup
    bool        animPopupIsCollision = false; // true = collision, false = manual action
    bool        canContinueCycle = false;
    std::string continueButtonLabel = u8"Продолжить";
    bool        pedalRequired = false;
    bool        pedalPressed = false;
    std::string pedalPrompt = u8"Зажмите педаль D7";
    float       animProgressFrac = 0.0f;
    int         animCurrentPart  = 0;
    int         animTotalParts   = 1;
    int         animStepIndex    = -1;
    std::string animPhaseName;
    int         animPhaseType    = -1;  // 0=feed, 1=rotation, 2=bend, -1=none

    // Current axis values for debug display (written by App)
    double axisBend     = 0.0; // degrees — bender roller angle
    double axisCarriage = 0.0; // mm — feed/carriage position
    double axisChuck    = 0.0; // degrees — chuck/rotation

    // Auto-rebuild flag
    bool needsRebuild = false;

    // Logic 2 toggle (new models + clamp animation + collision v2)
    bool useLogic2 = false;
    bool stockEnough = true;

    // Collision results for display
    std::string collisionInfo;
    int collisionStepIdx = -1;  // step index with collision (-1 = none)

    // Osnastka: spring-back coefficient and steel grade
    double springBackCoeff = 1.0;   // multiplier for overbend
    int    steelGradeIdx   = 0;     // index into steelGrades[]

    // Debug: manual axis input
    double manualBend     = 0.0;
    double manualCarriage = 0.0;
    double manualChuck    = 0.0;

    // Settings: save confirmation timer
    float settingsSavedTimer_ = 0.0f;
    std::string settingsSavedMsg_;

    // Camera orientation for gizmo (written by App each frame)
    float cameraYaw = -90.0f;
    float cameraPitch = 25.0f;

    // Undo / Redo (public API)
    void pushUndo(const BendingProgram& prog);
    void undo(BendingProgram& prog);
    void redo(BendingProgram& prog);
    bool canUndo() const { return !undoStack_.empty(); }
    bool canRedo() const { return !redoStack_.empty(); }

    // Settings
    AppSettings settings;
    std::string exeDir_;

    // Global tooling (machine state, not per-project)
    double toolingD_ = 38;
    double toolingR_ = 76;
    double toolingClampLength_ = 60;
    double toolingStraightLen_ = 60;  // min straight section between bends (roller)
    double toolingFeedSpeed_   = 300.0;
    double toolingRotSpeed_    = 120.0;
    double toolingBendSpeed_   = 60.0;

    // Collision check flag — true after last check passed OK
    bool collisionChecked = false;
    bool showCollisionPrompt_ = false; // popup: запустить просчет?
    bool showMachineZeroConfirm_ = false;
    bool showCarriageHomeConfirm_ = false;

    // STP import conflict
    bool showImportConflict_ = false;
    bool importApproved_ = false;
    double importD_ = 0, importR_ = 0;
    BendingProgram importPending_;

    // File dialog helpers (Windows native)
    static std::string openFileDialog(const char* filter, const char* defaultExt);
    static std::string saveFileDialog(const char* filter, const char* defaultExt);

private:
    void drawMenuBar(BendingProgram& prog, UICallbacks& cb);
    void drawDockSpace();
    void setupDefaultLayout();
    void drawSceneWindow(UICallbacks& cb, unsigned int texId);
    void drawProgramWindow(BendingProgram& prog, UICallbacks& cb);
    void drawOsnastkaSidebar(BendingProgram& prog);
    void drawDebugWindow(UICallbacks& cb);
    void drawControlWindow(UICallbacks& cb);
    void drawConnectionWindow(UICallbacks& cb);
    void drawSpravkaSidebar();
    void drawSettingsWindow();

    void drawProgramTable(BendingProgram& prog);
    void drawProgressBar(UICallbacks& cb);
    void drawActionButtons(BendingProgram& prog, UICallbacks& cb);
    void drawCutEditorWindow(BendingProgram& prog);
    void drawProjectTabs(BendingProgram& prog);
    void drawAxisGizmo();

    void scanPresets();

    // Project file I/O
    void openProjectFromFile(const std::string& path, BendingProgram& prog);
    void saveProjectToFile(BendingProgram& prog);
    void saveProjectToFileAs(const std::string& path, BendingProgram& prog);
    void writeProjectFile(const Project& p);
    void saveActiveProject(BendingProgram& prog);
    void loadProject(int idx, BendingProgram& prog);

    bool showSpravka_    = false;
    bool showSettings_   = false;
    bool showCutEditor_  = false;
    int  cutEditorStep_  = -1;   // index into prog.steps of the cut being edited
    bool layoutInitialized_ = false;

    // Settings page index (0=Модели, 1=Интерфейс)
    int settingsPage_ = 0;

    // Drag-drop reorder state
    int dragSourceStep_ = -1;

    // Cached preset list from presets/ folder
    std::vector<std::string> presetFiles_;
    std::vector<std::string> presetNames_;
    bool presetsScanned_ = false;

    // Sidebar dimensions
    float osnastkaSidebarWidth_ = 280.0f;
    float spravkaSidebarWidth_ = 300.0f;

    // Wide path for imgui.ini (Cyrillic-safe)
    std::wstring iniWidePath_;

    // DockSpace ID computed inside drawDockSpace (window-context dependent)
    unsigned int dockSpaceId_ = 0;

    // Undo/Redo internals
    struct UndoState {
        BendingProgram program;
        double laserOffsetY = 60, laserGapZ = 5, whipLength = 6000, springBackCoeff = 1.0;
        int partCount = 1, steelGradeIdx = 0, selectedStep = -1;
        std::string presetName;
    };
    std::vector<UndoState> undoStack_;
    std::vector<UndoState> redoStack_;
    static constexpr int MAX_UNDO = 50;
    UndoState captureState(const BendingProgram& prog) const;
    void restoreState(const UndoState& state, BendingProgram& prog);
    size_t lastStateHash_ = 0;
    UndoState lastState_;
    bool undoJustApplied_ = false;

    // Project system
    std::vector<Project> projects_;
    int activeProject_ = 0;
    int pendingProjectSwitch_ = -1;
    int pendingProjectClose_ = -1;
    int pendingCloseConfirm_ = -1;
    bool pendingNewProject_ = false;

    bool coloredButton(const char* label, float r, float g, float b, float w = 0, float h = 0);

    // Machine and App state (set by App each frame)
    Machine* machinePtr_ = nullptr;
    class App* appPtr_ = nullptr;
public:
    void setMachine(Machine* m) { machinePtr_ = m; }
    void setApp(App* a) { appPtr_ = a; }
    void syncActiveProject(BendingProgram& prog);
    std::string activeProjectName() const;
    int activeProjectStepCount() const;

    // COM port selection
    int selectedPort_ = 0;
    std::vector<std::string> comPorts_;
    float comPortScanTimer_ = 0;

    // Clamp/Unclamp animations
    bool clampAnimationActive_ = false;
    float clampAnimationTimer_ = 0.0f;
    bool unclampAnimationActive_ = false;
    float unclampAnimationTimer_ = 0.0f;

    // Relay toggle state
    bool relayClampToggle_ = false;   // ПРИЖИМ on/off
    bool relayUnclampToggle_ = false; // РАЗЖИМ on/off
    // 0=none (both OFF), 1=last sent CLAMP, 2=last sent UNCLAMP
    int relayLastCommand_ = 0;
    // Pending command (1/2) waiting for Teensy confirmation, with timeout
    int relayPendingCommand_ = 0;
    double relayCommandSendTime_ = 0.0;
    static constexpr double RELAY_CONFIRM_TIMEOUT = 0.5; // 500ms for Teensy to respond
};
