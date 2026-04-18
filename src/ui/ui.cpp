// ─────────────────────────────────────────────────────────────────────

// ui.cpp — Dear ImGui UI panels for TubeBender v2.2

//   ImGui Docking for window management (Win11-like snapping).

// ─────────────────────────────────────────────────────────────────────

#include "ui.h"
#include <GLFW/glfw3.h>

#include <imgui.h>

#include <imgui_internal.h>

#include <cstdio>

#include <cstring>

#include <cmath>

#include <algorithm>

#include <fstream>

#include <filesystem>

#include <nlohmann/json.hpp>



#ifdef _WIN32

#include <windows.h>

#include <commdlg.h>

#include <shlobj.h>

#endif



#ifndef M_PI

#define M_PI 3.14159265358979323846

#endif



namespace fs = std::filesystem;



// DockSpace ID (constant across frames)

static const char* DOCKSPACE_ID = "MainDock";



// Cut type names for dropdown editing

static const char* cutTypeNames[] = {

    u8"\u041f\u0440\u044f\u043c\u043e\u0439",     // Прямой

    u8"\u041a\u043e\u0441\u043e\u0439",       // Косой

    u8"\u0421\u0435\u0434\u043b\u043e\u0432\u043e\u0439"    // Седловой

};

static const int cutTypeCount = 3;



// Map CutType enum to dropdown index

static int cutTypeToIndex(CutType t) {

    switch (t) {

        case CutType::Straight: return 0;

        case CutType::Oblique:  return 1;

        case CutType::Saddle:   return 2;

        default: return 0;

    }

}



// Compute oblique angle (degrees) from CutPlane normal

static double obliqueAngleDeg(const CutPlane& p) {

    double lateral = std::sqrt(p.nx * p.nx + p.nz * p.nz);

    return std::atan2(lateral, std::abs(p.ny)) * 180.0 / M_PI;

}



// Set CutPlane normal from oblique angle (tilt in Z direction)

static void setObliqueAngle(CutPlane& p, double angleDeg) {

    double a = angleDeg * M_PI / 180.0;

    p.nx = 0;

    p.ny = -std::cos(a);

    p.nz = std::sin(a);

}



// State hash for undo change detection

static size_t computeStateHash(const BendingProgram& prog) {

    size_t h = 0;

    auto combine = [&h](size_t v) {

        h ^= v + 0x9e3779b9 + (h << 6) + (h >> 2);

    };

    combine(prog.steps.size());

    combine(std::hash<double>()(prog.D));

    combine(std::hash<double>()(prog.R));

    combine(std::hash<double>()(prog.clampLength));

    for (auto& s : prog.steps) {

        combine(std::hash<double>()(s.feedLength));

        combine(std::hash<double>()(s.rotation));

        combine(std::hash<double>()(s.bendAngle));

        combine(std::hash<int>()(s.isCut ? 1 : 0));

        combine(s.cutDef.planes.size());

        for (auto& p : s.cutDef.planes) {

            combine(std::hash<double>()(p.nx));

            combine(std::hash<double>()(p.ny));

            combine(std::hash<double>()(p.nz));

            combine(std::hash<double>()(p.offsetAlongAxis));

        }

    }

    return h;

}



// Wide-path file read helper (for imgui.ini on Cyrillic paths)

#ifdef _WIN32

static std::string readFileWide(const std::wstring& wpath) {

    FILE* f = _wfopen(wpath.c_str(), L"rb");

    if (!f) return {};

    fseek(f, 0, SEEK_END);

    long sz = ftell(f);

    fseek(f, 0, SEEK_SET);

    std::string data(sz, '\0');

    fread(data.data(), 1, sz, f);

    fclose(f);

    return data;

}

static bool writeFileWide(const std::wstring& wpath, const char* data, size_t len) {

    FILE* f = _wfopen(wpath.c_str(), L"wb");

    if (!f) return false;

    fwrite(data, 1, len, f);

    fclose(f);

    return true;

}

#endif



// ═══════════════════════════════════════════════════════════════

//  AppSettings — load/save settings.json

// ═══════════════════════════════════════════════════════════════

void AppSettings::loadFromFile(const std::string& path) {

    try {

#ifdef _WIN32

        int wlen = MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, nullptr, 0);

        std::wstring wp(wlen, L'\0');

        MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, wp.data(), wlen);

        std::ifstream f(wp);

#else

        std::ifstream f(path);

#endif

        if (!f) return;

        nlohmann::json j;

        f >> j;

        if (j.contains("benderModelPath"))    benderModelPath    = j["benderModelPath"].get<std::string>();

        if (j.contains("benderAxisModelPath"))benderAxisModelPath= j["benderAxisModelPath"].get<std::string>();

        if (j.contains("rollerModelPath"))    rollerModelPath    = j["rollerModelPath"].get<std::string>();

        if (j.contains("carriageModelPath"))  carriageModelPath  = j["carriageModelPath"].get<std::string>();

        if (j.contains("clampModelPath"))     clampModelPath     = j["clampModelPath"].get<std::string>();

        if (j.contains("chuckModelPath"))     chuckModelPath     = j["chuckModelPath"].get<std::string>();

        if (j.contains("staticModelPath"))    staticModelPath    = j["staticModelPath"].get<std::string>();

        if (j.contains("lastPreset"))         lastPreset         = j["lastPreset"].get<std::string>();

        if (j.contains("showScene"))          showScene          = j["showScene"].get<bool>();

        if (j.contains("showProgram"))        showProgram        = j["showProgram"].get<bool>();

        if (j.contains("showOsnastka"))       showOsnastka       = j["showOsnastka"].get<bool>();

        if (j.contains("showDebug"))          showDebug          = j["showDebug"].get<bool>();

        if (j.contains("showControl"))        showControl        = j["showControl"].get<bool>();

        if (j.contains("toolingD"))          toolingD          = j["toolingD"].get<double>();

        if (j.contains("toolingR"))          toolingR          = j["toolingR"].get<double>();

        if (j.contains("toolingClampLength"))toolingClampLength = j["toolingClampLength"].get<double>();

        if (j.contains("toolingStraightLen"))toolingStraightLen = j["toolingStraightLen"].get<double>();

        if (j.contains("laserOffsetY"))      laserOffsetY      = j["laserOffsetY"].get<double>();

        if (j.contains("laserGapZ"))         laserGapZ         = j["laserGapZ"].get<double>();

        if (j.contains("whipLength"))        whipLength        = j["whipLength"].get<double>();

        if (j.contains("partCount"))         partCount         = j["partCount"].get<int>();

        if (j.contains("springBackCoeff"))   springBackCoeff   = j["springBackCoeff"].get<double>();

        if (j.contains("steelGradeIdx"))     steelGradeIdx     = j["steelGradeIdx"].get<int>();

        if (j.contains("osnastkaSidebarOpen")) osnastkaSidebarOpen = j["osnastkaSidebarOpen"].get<bool>();

        if (j.contains("spravkaSidebarOpen"))  spravkaSidebarOpen  = j["spravkaSidebarOpen"].get<bool>();

        if (j.contains("useLogic2"))            useLogic2            = j["useLogic2"].get<bool>();

        if (j.contains("backfaceCulling"))     backfaceCulling      = false; // Forced to show all polygons

        if (j.contains("sceneRenderScale"))    sceneRenderScale     = j["sceneRenderScale"].get<float>();

        if (j.contains("lastAxisBend"))        lastAxisBend         = j["lastAxisBend"].get<double>();

        if (j.contains("lastAxisCarriage"))    lastAxisCarriage     = j["lastAxisCarriage"].get<double>();

        if (j.contains("lastAxisChuck"))       lastAxisChuck        = j["lastAxisChuck"].get<double>();

    } catch (...) {}

}



void AppSettings::saveToFile(const std::string& path) const {

    nlohmann::json j;

    j["benderModelPath"]     = benderModelPath;

    j["benderAxisModelPath"] = benderAxisModelPath;

    j["rollerModelPath"]     = rollerModelPath;

    j["carriageModelPath"]   = carriageModelPath;

    j["clampModelPath"]      = clampModelPath;

    j["chuckModelPath"]      = chuckModelPath;

    j["staticModelPath"]     = staticModelPath;

    j["lastPreset"]          = lastPreset;

    j["showScene"]           = showScene;

    j["showProgram"]         = showProgram;

    j["showOsnastka"]        = showOsnastka;

    j["showDebug"]           = showDebug;

    j["showControl"]         = showControl;

    j["toolingD"]            = toolingD;

    j["toolingR"]            = toolingR;

    j["toolingClampLength"]  = toolingClampLength;

    j["toolingStraightLen"]  = toolingStraightLen;

    j["laserOffsetY"]        = laserOffsetY;

    j["laserGapZ"]           = laserGapZ;

    j["whipLength"]          = whipLength;

    j["partCount"]           = partCount;

    j["springBackCoeff"]     = springBackCoeff;

    j["steelGradeIdx"]       = steelGradeIdx;

    j["osnastkaSidebarOpen"] = osnastkaSidebarOpen;

    j["spravkaSidebarOpen"]  = spravkaSidebarOpen;

    j["useLogic2"]           = useLogic2;

    j["backfaceCulling"]     = backfaceCulling;

    j["sceneRenderScale"]    = sceneRenderScale;

    j["lastAxisBend"]        = lastAxisBend;

    j["lastAxisCarriage"]    = lastAxisCarriage;

    j["lastAxisChuck"]       = lastAxisChuck;

    try {

#ifdef _WIN32

        int wlen = MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, nullptr, 0);

        std::wstring wp(wlen, L'\0');

        MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, wp.data(), wlen);

        std::ofstream f(wp);

#else

        std::ofstream f(path);

#endif

        if (f) f << j.dump(2);

    } catch (...) {}

}



// ═══════════════════════════════════════════════════════════════

void UI::init(const std::string& exeDir) {

    exeDir_ = exeDir;

    // Compute wide ini path for manual save

#ifdef _WIN32

    {

        int wlen = MultiByteToWideChar(CP_UTF8, 0, exeDir.c_str(), -1, nullptr, 0);

        std::wstring wdir(wlen, L'\0');

        MultiByteToWideChar(CP_UTF8, 0, exeDir.c_str(), -1, wdir.data(), wlen);

        iniWidePath_ = wdir.substr(0, wdir.size()-1) + L"\\imgui.ini";

    }

#endif

    // Load settings

    std::string settingsPath = exeDir + "/settings.json";

    settings.loadFromFile(settingsPath);



    // Apply persisted window visibility

    showSceneWin    = settings.showScene;

    showProgramWin  = settings.showProgram;

    showOsnastkaWin = settings.showOsnastka;

    showDebugWin    = settings.showDebug;

    showControlWin  = settings.showControl;



    // Load global tooling from settings

    toolingD_ = settings.toolingD;

    toolingR_ = settings.toolingR;

    toolingClampLength_ = settings.toolingClampLength;

    toolingStraightLen_ = settings.toolingStraightLen;

    toolingFeedSpeed_ = settings.toolingFeedSpeed;

    toolingRotSpeed_ = settings.toolingRotSpeed;

    toolingBendSpeed_ = settings.toolingBendSpeed;

    laserOffsetY = settings.laserOffsetY;

    laserGapZ = settings.laserGapZ;

    whipLength = settings.whipLength;

    partCount = settings.partCount;

    springBackCoeff = settings.springBackCoeff;

    steelGradeIdx = settings.steelGradeIdx;

    useLogic2 = settings.useLogic2;



    // Restore last known axis positions so the scene doesn't snap to zero.

    axisBend     = settings.lastAxisBend;

    axisCarriage = settings.lastAxisCarriage;

    axisChuck    = settings.lastAxisChuck;

    manualBend     = axisBend;

    manualCarriage = axisCarriage;

    manualChuck    = axisChuck;



    // Create projects directory

    try { fs::create_directories(fs::u8path(exeDir) / "projects"); } catch (...) {}



    // Init default project

    projects_.clear();

    Project p;

    p.name = u8"\u041f\u0440\u043e\u0435\u043a\u0442 1";

    projects_.push_back(p);

    activeProject_ = 0;

}



void UI::saveLayoutNow() {

#ifdef _WIN32

    size_t iniLen = 0;

    const char* iniData = ImGui::SaveIniSettingsToMemory(&iniLen);

    if (iniData && iniLen > 0) {

        writeFileWide(iniWidePath_, iniData, iniLen);

    }

#else

    ImGui::SaveIniSettingsToDisk("imgui.ini");

#endif

    // Persist window visibility to settings.json

    settings.showScene    = showSceneWin;

    settings.showProgram  = showProgramWin;

    settings.showOsnastka = showOsnastkaWin;

    settings.showDebug    = showDebugWin;

    settings.showControl  = showControlWin;



    // Save global tooling to settings

    settings.toolingD = toolingD_;

    settings.toolingR = toolingR_;

    settings.toolingClampLength = toolingClampLength_;

    settings.toolingStraightLen = toolingStraightLen_;

    settings.toolingFeedSpeed = toolingFeedSpeed_;

    settings.toolingRotSpeed = toolingRotSpeed_;

    settings.toolingBendSpeed = toolingBendSpeed_;

    settings.laserOffsetY = laserOffsetY;

    settings.laserGapZ = laserGapZ;

    settings.whipLength = whipLength;

    settings.partCount = partCount;

    settings.springBackCoeff = springBackCoeff;

    settings.steelGradeIdx = steelGradeIdx;

    settings.useLogic2 = useLogic2;

    settings.lastAxisBend     = axisBend;

    settings.lastAxisCarriage = axisCarriage;

    settings.lastAxisChuck    = axisChuck;

    std::string settingsPath = exeDir_ + "/settings.json";

    settings.saveToFile(settingsPath);

}



// helper: get filename from full path

static std::string filenameFromPath(const std::string& p) {

    auto pos = p.find_last_of("/\\");

    return (pos != std::string::npos) ? p.substr(pos + 1) : p;

}



// ═══════════════════════════════════════════════════════════════

//  Scan presets folder

// ═══════════════════════════════════════════════════════════════

void UI::scanPresets() {

    presetFiles_.clear();

    presetNames_.clear();

    presetsScanned_ = true;

    try {

        fs::path presetsDir = fs::u8path(exeDir_) / "presets";

        if (!fs::exists(presetsDir)) return;

        for (auto& entry : fs::directory_iterator(presetsDir)) {

            if (entry.path().extension() == ".json") {

                std::string p = entry.path().u8string();

                presetFiles_.push_back(p);

                // Try to read name from JSON

                std::string name = entry.path().stem().u8string();

                try {

                    std::ifstream f(entry.path());

                    if (f) {

                        nlohmann::json j;

                        f >> j;

                        if (j.contains("name")) name = j["name"].get<std::string>();

                    }

                } catch (...) {}

                presetNames_.push_back(name);

            }

        }

    } catch (...) {}

}



// ═══════════════════════════════════════════════════════════════

//  DOCK SPACE — fills entire viewport below menu bar

// ═══════════════════════════════════════════════════════════════

void UI::drawDockSpace() {

    ImGuiViewport* vp = ImGui::GetMainViewport();

    float projectBarH = (projects_.size() > 0) ? (ImGui::GetFrameHeight() + 6) : 0;

    float leftW = showOsnastkaWin ? (settings.osnastkaSidebarOpen ? osnastkaSidebarWidth_ : 28.0f) : 0.0f;

    float rightW = showSpravka_ ? (settings.spravkaSidebarOpen ? spravkaSidebarWidth_ : 28.0f) : 0.0f;



    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + leftW, vp->WorkPos.y + projectBarH));

    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x - leftW - rightW, vp->WorkSize.y - projectBarH));

    ImGui::SetNextWindowViewport(vp->ID);



    ImGuiWindowFlags hostFlags =

        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |

        ImGuiWindowFlags_NoResize   | ImGuiWindowFlags_NoMove |

        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |

        ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoDocking;



    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));



    ImGui::Begin("##DockHost", nullptr, hostFlags);

    ImGui::PopStyleVar(3);



    ImGuiID dockId = ImGui::GetID(DOCKSPACE_ID);

    dockSpaceId_ = dockId;  // cache for setupDefaultLayout

    ImGui::DockSpace(dockId, ImVec2(0, 0), ImGuiDockNodeFlags_None);



    ImGui::End();

}



// ═══════════════════════════════════════════════════════════════

//  DEFAULT LAYOUT

// ═══════════════════════════════════════════════════════════════

void UI::setupDefaultLayout() {

    if (layoutInitialized_) return;

    layoutInitialized_ = true;



    ImGuiID dockId = dockSpaceId_;  // use cached ID from drawDockSpace (same window context)

    if (dockId == 0) return;       // not yet computed

    ImGuiDockNode* node = ImGui::DockBuilderGetNode(dockId);

    if (node && node->IsSplitNode()) return;



    ImGui::DockBuilderRemoveNode(dockId);

    ImGui::DockBuilderAddNode(dockId, ImGuiDockNodeFlags_DockSpace);



    ImGuiViewport* vp = ImGui::GetMainViewport();

    ImGui::DockBuilderSetNodeSize(dockId, vp->WorkSize);



    ImGuiID leftId, rightId;

    ImGui::DockBuilderSplitNode(dockId, ImGuiDir_Left, 0.55f, &leftId, &rightId);



    ImGuiID topLeftId, bottomLeftId;

    ImGui::DockBuilderSplitNode(leftId, ImGuiDir_Up, 0.6f, &topLeftId, &bottomLeftId);



    // Split right panel: program on top, control at bottom

    ImGuiID rightTopId, rightBottomId;

    ImGui::DockBuilderSplitNode(rightId, ImGuiDir_Down, 0.15f, &rightBottomId, &rightTopId);



    ImGui::DockBuilderDockWindow(u8"\u0421\u0446\u0435\u043d\u0430###Scene",      topLeftId);

    ImGui::DockBuilderDockWindow(u8"\u041f\u0440\u043e\u0433\u0440\u0430\u043c\u043c\u0430###Program", rightTopId);

    ImGui::DockBuilderDockWindow(u8"\u0423\u043f\u0440\u0430\u0432\u043b\u0435\u043d\u0438\u0435###Control", rightBottomId);

    ImGui::DockBuilderDockWindow(u8"\u041e\u0442\u043b\u0430\u0434\u043a\u0430###Debug",    bottomLeftId);



    ImGui::DockBuilderFinish(dockId);

}



// ═══════════════════════════════════════════════════════════════

//  MAIN DRAW

// ═══════════════════════════════════════════════════════════════

void UI::draw(BendingProgram& prog, UICallbacks& cb, int fbW, int fbH,

              unsigned int sceneTexId) {

    bool machineRunLocked = (machinePtr_ && machinePtr_->isProgramRunning());

    // ── Update animation timers ──

    float deltaTime = ImGui::GetIO().DeltaTime;

    if (clampAnimationActive_) {

        clampAnimationTimer_ += deltaTime;

        if (clampAnimationTimer_ >= 0.5f) {

            clampAnimationActive_ = false;

            clampAnimationTimer_ = 0.0f;

        }

    }

    if (unclampAnimationActive_) {

        unclampAnimationTimer_ += deltaTime;

        if (unclampAnimationTimer_ >= 0.5f) {

            unclampAnimationActive_ = false;

            unclampAnimationTimer_ = 0.0f;

        }

    }

    // ── Sync global tooling to program ──

    prog.D = toolingD_;

    prog.R = toolingR_;

    prog.clampLength = toolingClampLength_;



    // ── Handle pending project operations ──

    if (!machineRunLocked && pendingProjectSwitch_ >= 0 && pendingProjectSwitch_ != activeProject_) {

        saveActiveProject(prog);

        activeProject_ = pendingProjectSwitch_;

        loadProject(activeProject_, prog);

        needsRebuild = true;

        pendingProjectSwitch_ = -1;

    }

    // Close confirmation dialog

    if (!machineRunLocked && pendingCloseConfirm_ >= 0 && (int)projects_.size() > 1) {

        ImGui::OpenPopup(u8"\u0421\u043e\u0445\u0440\u0430\u043d\u0438\u0442\u044c?##CloseConfirm");

    }

    if (!machineRunLocked && ImGui::BeginPopupModal(u8"\u0421\u043e\u0445\u0440\u0430\u043d\u0438\u0442\u044c?##CloseConfirm", nullptr,

            ImGuiWindowFlags_AlwaysAutoResize)) {

        int ci = pendingCloseConfirm_;

        if (ci >= 0 && ci < (int)projects_.size()) {

            ImGui::Text(u8"\u0421\u043e\u0445\u0440\u0430\u043d\u0438\u0442\u044c \u043f\u0440\u043e\u0435\u043a\u0442 \"%s\" \u043f\u0435\u0440\u0435\u0434 \u0437\u0430\u043a\u0440\u044b\u0442\u0438\u0435\u043c?",

                projects_[ci].name.c_str());

            ImGui::Spacing();

            float bw = 140;

            if (coloredButton(u8"\u0421\u043e\u0445\u0440\u0430\u043d\u0438\u0442\u044c", 0.306f, 0.561f, 0.235f, bw)) {

                saveProjectToFile(prog);

                pendingProjectClose_ = ci;

                pendingCloseConfirm_ = -1;

                ImGui::CloseCurrentPopup();

            }

            ImGui::SameLine();

            if (ImGui::Button(u8"\u041d\u0435 \u0441\u043e\u0445\u0440\u0430\u043d\u044f\u0442\u044c", ImVec2(bw, 0))) {

                pendingProjectClose_ = ci;

                pendingCloseConfirm_ = -1;

                ImGui::CloseCurrentPopup();

            }

            ImGui::SameLine();

            if (ImGui::Button(u8"\u041e\u0442\u043c\u0435\u043d\u0430", ImVec2(bw, 0))) {

                pendingCloseConfirm_ = -1;

                ImGui::CloseCurrentPopup();

            }

        } else {

            pendingCloseConfirm_ = -1;

            ImGui::CloseCurrentPopup();

        }

        ImGui::EndPopup();

    }

    // STP import conflict dialog

    if (showImportConflict_) {

        ImGui::OpenPopup(u8"\u041d\u0435\u0441\u043e\u043e\u0442\u0432\u0435\u0442\u0441\u0442\u0432\u0438\u0435 \u043e\u0441\u043d\u0430\u0441\u0442\u043a\u0438##ImportConflict");

    }

    if (ImGui::BeginPopupModal(u8"\u041d\u0435\u0441\u043e\u043e\u0442\u0432\u0435\u0442\u0441\u0442\u0432\u0438\u0435 \u043e\u0441\u043d\u0430\u0441\u0442\u043a\u0438##ImportConflict", nullptr,

            ImGuiWindowFlags_AlwaysAutoResize)) {

        ImGui::Text(u8"\u0422\u0440\u0443\u0431\u043e\u0433\u0438\u0431 \u043d\u0430\u0441\u0442\u0440\u043e\u0435\u043d \u043d\u0430 D=%.1f R=%.1f", toolingD_, toolingR_);

        ImGui::Text(u8"\u0424\u0430\u0439\u043b \u0441\u043e\u0434\u0435\u0440\u0436\u0438\u0442 D=%.1f R=%.1f", importD_, importR_);

        ImGui::Spacing();

        ImGui::Text(u8"\u0412\u044b\u043f\u043e\u043b\u043d\u0438\u0442\u044c \u043f\u0435\u0440\u0435\u043e\u0441\u043d\u0430\u0441\u0442\u043a\u0443?");

        ImGui::Spacing();

        float bw = 160;

        if (coloredButton(u8"\u0414\u0430, \u043f\u0435\u0440\u0435\u043e\u0441\u043d\u0430\u0441\u0442\u0438\u0442\u044c", 0.306f, 0.561f, 0.235f, bw)) {

            toolingD_ = importD_;

            toolingR_ = importR_;

            importApproved_ = true;

            showImportConflict_ = false;

            ImGui::CloseCurrentPopup();

        }

        ImGui::SameLine();

        if (ImGui::Button(u8"\u041d\u0435\u0442, \u043e\u0442\u043c\u0435\u043d\u0438\u0442\u044c", ImVec2(bw, 0))) {

            showImportConflict_ = false;

            ImGui::CloseCurrentPopup();

        }

        ImGui::EndPopup();

    }

    if (!machineRunLocked && pendingProjectClose_ >= 0 && (int)projects_.size() > 1) {

        int ci = pendingProjectClose_;

        if (ci == activeProject_) {

            int newIdx = (ci > 0) ? ci - 1 : 0;

            saveActiveProject(prog);

            projects_.erase(projects_.begin() + ci);

            activeProject_ = std::min(newIdx, (int)projects_.size() - 1);

            loadProject(activeProject_, prog);

            needsRebuild = true;

        } else {

            projects_.erase(projects_.begin() + ci);

            if (activeProject_ > ci) activeProject_--;

        }

        pendingProjectClose_ = -1;

    }

    if (!machineRunLocked && pendingNewProject_) {

        saveActiveProject(prog);

        Project p;

        p.name = u8"\u041f\u0440\u043e\u0435\u043a\u0442 " + std::to_string(projects_.size() + 1);

        p.program.D = toolingD_; p.program.R = toolingR_; p.program.clampLength = toolingClampLength_;

        BendStep s1; s1.id = 1; s1.feedLength = 100; s1.bendAngle = 90;

        BendStep s2; s2.id = 2; s2.feedLength = 200;

        p.program.steps = { s1, s2 };

        projects_.push_back(p);

        activeProject_ = (int)projects_.size() - 1;

        loadProject(activeProject_, prog);

        needsRebuild = true;

        pendingNewProject_ = false;

    }



    // ── Manual action / collision popup modal ──

    if (animShowPopup) {

        ImGui::OpenPopup(u8"##ManualActionPopup");

        animShowPopup = false;

    }

    if (ImGui::BeginPopupModal(u8"##ManualActionPopup", nullptr,

            ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar)) {

        // Icon/color based on type

        if (animPopupIsCollision) {

            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));

            ImGui::Text(u8"\u26a0 \u0421\u0442\u043e\u043b\u043a\u043d\u043e\u0432\u0435\u043d\u0438\u0435!");

        } else {

            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.0f, 1.0f));

            ImGui::Text(u8"\u2699 \u0420\u0443\u0447\u043d\u043e\u0435 \u0434\u0435\u0439\u0441\u0442\u0432\u0438\u0435");

        }

        ImGui::PopStyleColor();

        ImGui::Separator();

        ImGui::Spacing();

        ImGui::TextWrapped("%s", animPauseMsg.c_str());

        ImGui::Spacing();

        ImGui::Separator();

        ImGui::Spacing();

        float bw = 140;

        if (coloredButton(u8"OK", 0.306f, 0.561f, 0.235f, bw)) {

            if (cb.onAnimContinue) cb.onAnimContinue();

            ImGui::CloseCurrentPopup();

        }

        ImGui::SameLine();

        if (ImGui::Button(u8"\u041e\u0442\u043c\u0435\u043d\u0430", ImVec2(bw, 0))) {

            if (cb.onStop) cb.onStop();

            ImGui::CloseCurrentPopup();

        }

        ImGui::EndPopup();

    }



    // ── Collision prompt popup (before start) ──

    if (showCollisionPrompt_) {

        ImGui::OpenPopup(u8"##CollisionPrompt");

        showCollisionPrompt_ = false;

    }

    if (ImGui::BeginPopupModal(u8"##CollisionPrompt", nullptr,

            ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar)) {

        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.0f, 1.0f));

        ImGui::Text(u8"\u26a0 \u041f\u0440\u043e\u0432\u0435\u0440\u043a\u0430 \u0441\u0442\u043e\u043b\u043a\u043d\u043e\u0432\u0435\u043d\u0438\u0439");

        ImGui::PopStyleColor();

        ImGui::Separator();

        ImGui::Spacing();

        ImGui::TextWrapped(u8"\u041f\u0440\u043e\u0433\u0440\u0430\u043c\u043c\u0430 \u043d\u0435 \u043f\u0440\u043e\u0432\u0435\u0440\u0435\u043d\u0430. \u0417\u0430\u043f\u0443\u0441\u0442\u0438\u0442\u044c \u043f\u0440\u043e\u0441\u0447\u0451\u0442 \u0441\u0442\u043e\u043b\u043a\u043d\u043e\u0432\u0435\u043d\u0438\u0439?");

        ImGui::Spacing();

        ImGui::Separator();

        ImGui::Spacing();

        float bw2 = 140;

        if (coloredButton(u8"\u0414\u0430", 0.306f, 0.561f, 0.235f, bw2)) {

            if (cb.onCheckCollisions) cb.onCheckCollisions();

            ImGui::CloseCurrentPopup();

        }

        ImGui::SameLine();

        if (ImGui::Button(u8"\u041e\u0442\u043c\u0435\u043d\u0430", ImVec2(bw2, 0))) {

            ImGui::CloseCurrentPopup();

        }

        ImGui::EndPopup();

    }



    drawMenuBar(prog, cb);

    drawProjectTabs(prog);



    // Draw sidebars (outside dock space)

    if (showOsnastkaWin) drawOsnastkaSidebar(prog);

    if (showSpravka_)    drawSpravkaSidebar();



    drawDockSpace();

    setupDefaultLayout();



    if (showSceneWin)    drawSceneWindow(cb, sceneTexId);

    if (showProgramWin)  drawProgramWindow(prog, cb);

    if (showDebugWin)    drawDebugWindow(cb);

    if (showControlWin)  drawControlWindow(cb);

    if (showConnectionWin) drawConnectionWindow(cb);

    if (showSettings_)   drawSettingsWindow();

    if (showCutEditor_)  drawCutEditorWindow(prog);



    // ── Undo tracking: detect state changes at end of frame ──

    size_t curHash = computeStateHash(prog);

    if (lastStateHash_ != 0 && curHash != lastStateHash_ && !undoJustApplied_) {

        undoStack_.push_back(lastState_);

        if ((int)undoStack_.size() > MAX_UNDO)

            undoStack_.erase(undoStack_.begin());

        redoStack_.clear();

    }

    undoJustApplied_ = false;

    lastState_ = captureState(prog);

    lastStateHash_ = curHash;

}



// ═══════════════════════════════════════════════════════════════

//  СЦЕНА WINDOW — FBO texture + progress bar

// ═══════════════════════════════════════════════════════════════

void UI::drawSceneWindow(UICallbacks& cb, unsigned int texId) {

    ImGuiWindowFlags sceneFlags =

        ImGuiWindowFlags_NoScrollbar |

        ImGuiWindowFlags_NoScrollWithMouse |

        ImGuiWindowFlags_NoCollapse;



    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));



    bool open = showSceneWin;

    ImGui::Begin(u8"\u0421\u0446\u0435\u043d\u0430###Scene", &open, sceneFlags);

    if (!open) showSceneWin = false;



    ImGui::PopStyleVar();



    ImVec2 avail = ImGui::GetContentRegionAvail();

    // Динамический статус-бар: высота зависит от состояния анимации/паузы.

    float barH;

    if (animActive || canContinueCycle || (animStepIndex >= 0 && animProgressFrac > 0.0f)) {

        barH = animPaused ? 110.0f : 70.0f;

        if (pedalRequired && !pedalPressed) barH += 20.0f;

    } else {

        barH = 44.0f;

    }

    if (barH > avail.y * 0.5f) barH = avail.y * 0.5f;

    // Compensate for ItemSpacing.y between Image and progress bar widgets,

    // otherwise the bar gets pushed below the window's content region.

    const float itemSpY = ImGui::GetStyle().ItemSpacing.y;

    float imgH = avail.y - barH - itemSpY;

    if (imgH < 50.0f) imgH = avail.y;

    float imgW = avail.x;

    if (imgW < 1.0f) imgW = 1.0f;

    if (imgH < 1.0f) imgH = 1.0f;



    sceneContentW = imgW;

    sceneContentH = imgH;



    ImVec2 imgCursorPos = ImGui::GetCursorScreenPos();

    if (texId != 0) {

        ImGui::Image((ImTextureID)(intptr_t)texId, ImVec2(imgW, imgH),

                     ImVec2(0, 1), ImVec2(1, 0));

    } else {

        ImGui::Dummy(ImVec2(imgW, imgH));

    }



    sceneHovered = ImGui::IsItemHovered();



    // ─────────────────────────────────────────────────────────────

    //  Overlay: «Режим симуляции» при отсутствии подключения

    // ─────────────────────────────────────────────────────────────

    {

        bool simMode = !(machinePtr_ && machinePtr_->isConnected());

        if (simMode) {

            ImDrawList* dl = ImGui::GetWindowDrawList();

            const char* simText = u8"\u0420\u0415\u0416\u0418\u041c \u0421\u0418\u041c\u0423\u041b\u042f\u0426\u0418\u0418";

            ImVec2 ts = ImGui::CalcTextSize(simText);

            float padX = 14.0f, padY = 6.0f;

            ImVec2 boxMin = ImVec2(imgCursorPos.x + 12.0f, imgCursorPos.y + 12.0f);

            ImVec2 boxMax = ImVec2(boxMin.x + ts.x + padX * 2, boxMin.y + ts.y + padY * 2);

            dl->AddRectFilled(boxMin, boxMax, IM_COL32(20, 20, 25, 200), 6.0f);

            dl->AddRect(boxMin, boxMax, IM_COL32(255, 200, 40, 230), 6.0f, 0, 1.5f);

            dl->AddText(ImVec2(boxMin.x + padX, boxMin.y + padY),

                        IM_COL32(255, 215, 60, 255), simText);

        }

    }



    // 3D axis gizmo overlay (top-right corner)

    drawAxisGizmo();



    // ─────────────────────────────────────────────────────────────

    //  Status bar — занимает фиксированный кусок снизу окна, не вылезает.

    // ─────────────────────────────────────────────────────────────

    {

        // Force cursor exactly at the bottom-left of the reserved bar area

        // (eliminates accumulated padding/spacing between Image and bar).

        ImVec2 winPos = ImGui::GetWindowPos();

        ImVec2 winSize = ImGui::GetWindowSize();

        // Give an extra 6px margin at the bottom so 3D view window border isn't clipped

        ImGui::SetCursorScreenPos(ImVec2(winPos.x, winPos.y + winSize.y - barH - 6.0f));



        ImGui::BeginChild("##sceneStatusBar", ImVec2(imgW, barH), false,

            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

        ImGui::PushItemWidth(imgW - 8.0f);

        drawProgressBar(cb);

        ImGui::PopItemWidth();

        ImGui::EndChild();

    }



    ImGui::End();

}



// ═══════════════════════════════════════════════════════════════

//  ОСНАСТКА WINDOW — preset dropdown + params

// ═══════════════════════════════════════════════════════════════

void UI::drawOsnastkaSidebar(BendingProgram& prog) {

    ImGuiViewport* vp = ImGui::GetMainViewport();

    float projectBarH = (projects_.size() > 0) ? (ImGui::GetFrameHeight() + 6) : 0;

    float topY = vp->WorkPos.y + projectBarH;

    float height = vp->WorkSize.y - projectBarH;

    float collapsedW = 28.0f;

    float width = settings.osnastkaSidebarOpen ? osnastkaSidebarWidth_ : collapsedW;



    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x, topY));

    ImGui::SetNextWindowSize(ImVec2(width, height));



    ImGuiWindowFlags flags =

        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |

        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |

        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking;



    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.11f, 0.11f, 0.115f, 1.0f));

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,

        settings.osnastkaSidebarOpen ? ImVec2(8, 6) : ImVec2(2, 6));

    ImGui::Begin("##OsnastkaSidebar", nullptr, flags);

    ImGui::PopStyleVar();

    ImGui::PopStyleColor();



    if (!settings.osnastkaSidebarOpen) {

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.16f, 1.0f));

        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.25f, 0.27f, 1.0f));

        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.30f, 0.30f, 0.32f, 1.0f));

        if (ImGui::ArrowButton("##osnOpen", ImGuiDir_Right))

            settings.osnastkaSidebarOpen = true;

        ImGui::PopStyleColor(3);

        if (ImGui::IsItemHovered()) ImGui::SetTooltip(u8"\u041f\u043e\u043a\u0430\u0437\u0430\u0442\u044c \u043e\u0441\u043d\u0430\u0441\u0442\u043a\u0443");

        ImGui::End();

        return;

    }



    // Header

    if (ImGui::Button("?", ImVec2(20, 0)))

        settings.osnastkaSidebarOpen = false;

    if (ImGui::IsItemHovered()) ImGui::SetTooltip(u8"\u0421\u043a\u0440\u044b\u0442\u044c \u043e\u0441\u043d\u0430\u0441\u0442\u043a\u0443");

    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.878f, 0.424f, 0.467f, 1.0f));

    ImGui::Text(u8"\u041e\u0421\u041d\u0410\u0421\u0422\u041a\u0410");

    ImGui::PopStyleColor();

    ImGui::Separator();



    // Scrollable content

    ImGui::BeginChild("##OsnContent", ImVec2(0, 0), false);



    // Scan presets on first draw

    if (!presetsScanned_) scanPresets();



    // Preset dropdown

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

    ImGui::Text(u8"\u041f\u0440\u0435\u0441\u0435\u0442:");

    ImGui::PopStyleColor();



    ImGui::PushItemWidth(-1);

    if (ImGui::BeginCombo("##presetCombo", presetName.c_str())) {

        for (int i = 0; i < (int)presetNames_.size(); i++) {

            bool isSel = (presetName == presetNames_[i]);

            if (ImGui::Selectable(presetNames_[i].c_str(), isSel)) {

                try {

#ifdef _WIN32

                    int wlen = MultiByteToWideChar(CP_UTF8, 0, presetFiles_[i].c_str(), -1, nullptr, 0);

                    std::wstring wp(wlen, L'\0');

                    MultiByteToWideChar(CP_UTF8, 0, presetFiles_[i].c_str(), -1, wp.data(), wlen);

                    std::ifstream f(wp);

#else

                    std::ifstream f(presetFiles_[i]);

#endif

                    if (f) {

                        nlohmann::json j;

                        f >> j;

                        if (j.contains("name")) presetName = j["name"].get<std::string>();

                        if (j.contains("D")) toolingD_ = j["D"].get<double>();

                        if (j.contains("R")) toolingR_ = j["R"].get<double>();

                        if (j.contains("clampLength")) toolingClampLength_ = j["clampLength"].get<double>();

                        if (j.contains("straightLen")) toolingStraightLen_ = j["straightLen"].get<double>();

                        if (j.contains("feedSpeed")) toolingFeedSpeed_ = j["feedSpeed"].get<double>();

                        if (j.contains("rotSpeed")) toolingRotSpeed_ = j["rotSpeed"].get<double>();

                        if (j.contains("bendSpeed")) toolingBendSpeed_ = j["bendSpeed"].get<double>();

                        if (j.contains("laserOffsetY")) laserOffsetY = j["laserOffsetY"].get<double>();

                        if (j.contains("springBackCoeff")) springBackCoeff = j["springBackCoeff"].get<double>();

                        if (j.contains("steelGradeIdx")) steelGradeIdx = j["steelGradeIdx"].get<int>();

                        needsRebuild = true;

                    }

                } catch (...) {}

            }

            if (isSel) ImGui::SetItemDefaultFocus();

        }

        ImGui::EndCombo();

    }

    ImGui::PopItemWidth();



    // Buttons row

    float btnW3 = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 2) / 3;

    if (ImGui::Button(u8"\u0417\u0430\u0433\u0440.##presetLoad", ImVec2(btnW3, 0))) {

        auto p = openFileDialog("Preset JSON\0*.json\0All\0*.*\0", "json");

        if (!p.empty()) {

            try {

#ifdef _WIN32

                int wlen = MultiByteToWideChar(CP_UTF8, 0, p.c_str(), -1, nullptr, 0);

                std::wstring wp(wlen, L'\0');

                MultiByteToWideChar(CP_UTF8, 0, p.c_str(), -1, wp.data(), wlen);

                std::ifstream f(wp);

#else

                std::ifstream f(p);

#endif

                if (f) {

                    nlohmann::json j;

                    f >> j;

                    if (j.contains("name")) presetName = j["name"].get<std::string>();

                    if (j.contains("D")) toolingD_ = j["D"].get<double>();

                    if (j.contains("R")) toolingR_ = j["R"].get<double>();

                    if (j.contains("clampLength")) toolingClampLength_ = j["clampLength"].get<double>();

                    if (j.contains("straightLen")) toolingStraightLen_ = j["straightLen"].get<double>();

                    if (j.contains("feedSpeed")) toolingFeedSpeed_ = j["feedSpeed"].get<double>();

                    if (j.contains("rotSpeed")) toolingRotSpeed_ = j["rotSpeed"].get<double>();

                    if (j.contains("bendSpeed")) toolingBendSpeed_ = j["bendSpeed"].get<double>();

                    if (j.contains("laserOffsetY")) laserOffsetY = j["laserOffsetY"].get<double>();

                    if (j.contains("springBackCoeff")) springBackCoeff = j["springBackCoeff"].get<double>();

                    if (j.contains("steelGradeIdx")) steelGradeIdx = j["steelGradeIdx"].get<int>();

                    needsRebuild = true;

                }

            } catch (...) {}

        }

    }

    ImGui::SameLine();

    if (ImGui::Button(u8"\u0421\u043e\u0445\u0440.##preset", ImVec2(btnW3, 0))) {

        std::string dir = exeDir_ + "/presets";

        try { fs::create_directories(fs::u8path(dir)); } catch (...) {}

        auto p = saveFileDialog("Preset JSON\0*.json\0", "json");

        if (!p.empty()) {

            nlohmann::json j;

            j["name"] = presetName;

            j["D"] = toolingD_;

            j["R"] = toolingR_;

            j["clampLength"] = toolingClampLength_;

            j["straightLen"] = toolingStraightLen_;

            j["feedSpeed"] = toolingFeedSpeed_;

            j["rotSpeed"] = toolingRotSpeed_;

            j["bendSpeed"] = toolingBendSpeed_;

            j["laserOffsetY"] = laserOffsetY;

            j["springBackCoeff"] = springBackCoeff;

            j["steelGradeIdx"] = steelGradeIdx;

#ifdef _WIN32

            int wlen = MultiByteToWideChar(CP_UTF8, 0, p.c_str(), -1, nullptr, 0);

            std::wstring wp(wlen, L'\0');

            MultiByteToWideChar(CP_UTF8, 0, p.c_str(), -1, wp.data(), wlen);

            std::ofstream f(wp);

#else

            std::ofstream f(p);

#endif

            if (f) f << j.dump(2);

            presetsScanned_ = false;

        }

    }

    ImGui::SameLine();

    if (ImGui::Button(u8"\u041e\u0431\u043d.##rescan", ImVec2(btnW3, 0))) {

        presetsScanned_ = false;

    }



    ImGui::Separator();



    // Grid-aligned params

    if (ImGui::BeginTable("OsnParams", 2, ImGuiTableFlags_SizingStretchProp)) {

        ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, 100);

        ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);



        auto paramRow = [](const char* label, auto drawFn) {

            ImGui::TableNextRow();

            ImGui::TableSetColumnIndex(0);

            ImGui::AlignTextToFramePadding();

            ImGui::Text("%s", label);

            ImGui::TableSetColumnIndex(1);

            ImGui::SetNextItemWidth(-1);

            drawFn();

        };



        paramRow(u8"D, \u043c\u043c", [&]() {

            if (ImGui::InputDouble("##pD", &toolingD_, 0, 0, "%.1f"))

                needsRebuild = true;

        });

        paramRow(u8"R, \u043c\u043c", [&]() {

            if (ImGui::InputDouble("##pR", &toolingR_, 0, 0, "%.1f"))

                needsRebuild = true;

        });

        paramRow(u8"\u0417\u0430\u0436\u0438\u043c, \u043c\u043c", [&]() {

            if (ImGui::InputDouble("##pClamp", &toolingClampLength_, 0, 0, "%.1f"))

                needsRebuild = true;

            if (ImGui::IsItemHovered()) ImGui::SetTooltip(u8"\u0414\u043b\u0438\u043d\u0430 \u043f\u0440\u044f\u043c\u043e\u0433\u043e \u043f\u0440\u0438\u0436\u0438\u043c\u043d\u043e\u0433\u043e \u0443\u0447\u0430\u0441\u0442\u043a\u0430 (\u043c\u043c)");

        });

        paramRow(u8"\u041f\u0440\u044f\u043c\u043e\u0439 \u0440\u043e\u043b., \u043c\u043c", [&]() {

            if (ImGui::InputDouble("##pStraight", &toolingStraightLen_, 0, 0, "%.1f"))

                collisionChecked = false;

            if (ImGui::IsItemHovered()) ImGui::SetTooltip(u8"\u041c\u0438\u043d. \u0434\u043b\u0438\u043d\u0430 \u043f\u0440\u044f\u043c\u043e\u0433\u043e \u0443\u0447\u0430\u0441\u0442\u043a\u0430 \u0440\u043e\u043b\u0438\u043a\u0430 (\u043c\u043c)");

        });

        paramRow(u8"\u0421\u043a\u043e\u0440\u043e\u0441\u0442\u044c \u043f\u043e\u0434\u0430\u0447\u0438", [&]() {

            ImGui::InputDouble("##mFeedSpeed", &toolingFeedSpeed_, 1.0, 10.0, "%.1f");

        });

        paramRow(u8"\u0421\u043a\u043e\u0440\u043e\u0441\u0442\u044c \u0432\u0440\u0430\u0449\u0435\u043d\u0438\u044f", [&]() {

            ImGui::InputDouble("##mRotSpeed", &toolingRotSpeed_, 1.0, 10.0, "%.1f");

        });

        paramRow(u8"\u0421\u043a\u043e\u0440\u043e\u0441\u0442\u044c \u0433\u0438\u0431\u0430", [&]() {

            ImGui::InputDouble("##mBendSpeed", &toolingBendSpeed_, 1.0, 10.0, "%.1f");

        });

        paramRow(u8"\u041b\u0430\u0437\u0435\u0440 Y, \u043c\u043c", [&]() { ImGui::InputDouble("##pLY", &laserOffsetY, 0, 0, "%.1f"); });

        paramRow(u8"\u0417\u0430\u0437\u043e\u0440 Z, \u043c\u043c", [&]() { ImGui::InputDouble("##pGZ", &laserGapZ, 0, 0, "%.1f"); });

        paramRow(u8"\u041a\u043e\u043b-\u0432\u043e", [&]() {

            ImGui::InputInt("##pCnt", &partCount);

            if (partCount < 1) partCount = 1;

        });

        paramRow(u8"\u0425\u043b\u044b\u0441\u0442, \u043c\u043c", [&]() { ImGui::InputDouble("##pWhip", &whipLength, 0, 0, "%.0f"); });



        ImGui::TableNextRow();

        ImGui::TableSetColumnIndex(0);

        ImGui::Separator();

        ImGui::TableSetColumnIndex(1);

        ImGui::Separator();



        // Steel grade dropdown

        static const char* steelGrades[] = {

            u8"08\u043f\u0441", u8"10", u8"20", u8"45", u8"09\u04132\u0421",

            u8"12\u041118\u041d10\u0422", u8"08\u041118\u041d10\u0422",

            u8"AISI 304", u8"AISI 316", u8"AISI 321"

        };

        static const int steelGradeCount = 10;

        paramRow(u8"\u041c\u0430\u0440\u043a\u0430 \u0441\u0442\u0430\u043b\u0438", [&]() {

            if (steelGradeIdx < 0 || steelGradeIdx >= steelGradeCount) steelGradeIdx = 0;

            ImGui::Combo("##steelGrade", &steelGradeIdx, steelGrades, steelGradeCount);

        });

        paramRow(u8"\u041a\u043e\u044d\u0444. \u043f\u0440\u0443\u0436.", [&]() {

            ImGui::InputDouble("##pSpring", &springBackCoeff, 0, 0, "%.3f");

            if (ImGui::IsItemHovered()) ImGui::SetTooltip(u8"\u041a\u043e\u044d\u0444\u0444\u0438\u0446\u0438\u0435\u043d\u0442 \u043f\u0440\u0443\u0436\u0438\u043d\u0435\u043d\u0438\u044f (1.0 = \u0431\u0435\u0437 \u043a\u043e\u043c\u043f\u0435\u043d\u0441\u0430\u0446\u0438\u0438)");

        });



        ImGui::EndTable();

    }



    // Info line

    double lTotal = prog.lTotal();

    int maxParts = (lTotal > 0.01) ? (int)(whipLength / lTotal) : 0;

    bool warn = (partCount > maxParts);

    stockEnough = !warn && maxParts > 0;

    if (warn) {

        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.9f, 0.0f, 1.0f));

        ImGui::Text(u8"L=%.1f  \u041c\u0430\u043a\u0441: %d  \u041d\u0435 \u0445\u0432\u0430\u0442\u0430\u0435\u0442!", lTotal, maxParts);

        ImGui::PopStyleColor();

    } else {

        ImGui::Text(u8"L=%.1f  \u041c\u0430\u043a\u0441: %d  \u041e\u0441\u0442: %.0f \u043c\u043c",

            lTotal, maxParts, whipLength - lTotal * partCount);

    }



    ImGui::EndChild();

    ImGui::End();

}



// ═══════════════════════════════════════════════════════════════

//  ПРОГРАММА WINDOW — buttons + table

// ═══════════════════════════════════════════════════════════════

void UI::drawProgramWindow(BendingProgram& prog, UICallbacks& cb) {

    ImGuiWindowFlags prFlags = ImGuiWindowFlags_NoCollapse;



    bool open = showProgramWin;

    ImGui::Begin(u8"\u041f\u0440\u043e\u0433\u0440\u0430\u043c\u043c\u0430###Program", &open, prFlags);

    if (!open) showProgramWin = false;



    bool machineRunLocked = (machinePtr_ && machinePtr_->isProgramRunning());

    if (machineRunLocked) {

        ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.2f, 1.0f),

            u8"Редактирование шагов заблокировано во время выполнения программы на станке");

    }

    if (machineRunLocked) ImGui::BeginDisabled();

    drawProgramTable(prog);

    if (machineRunLocked) ImGui::EndDisabled();



    if (!infoText.empty()) {

        ImGui::Separator();

        ImGui::TextWrapped("%s", infoText.c_str());

    }



    ImGui::End();

}



// ═══════════════════════════════════════════════════════════════

//  PROGRAM TABLE — unified list with Шаг/Рез type + buttons

// ═══════════════════════════════════════════════════════════════

void UI::drawProgramTable(BendingProgram& prog) {

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.878f, 0.424f, 0.467f, 1.0f));

    ImGui::Text(u8"\u041f\u0440\u043e\u0433\u0440\u0430\u043c\u043c\u0430");

    ImGui::PopStyleColor();



    // ─── Buttons: two vertical columns + Реверс ──────────────────

    {

        float spacing = ImGui::GetStyle().ItemSpacing.x;

        float totalW  = ImGui::GetContentRegionAvail().x;

        float colW    = (totalW - spacing * 2) / 3.0f;

        float btnH    = ImGui::GetFrameHeight();



        // Column 1: Добавить / Удалить (vertical)

        ImGui::BeginGroup();

        if (coloredButton(u8"\u0414\u043e\u0431\u0430\u0432\u0438\u0442\u044c", 0.055f, 0.392f, 0.612f, colW, btnH)) {

            BendStep s;

            s.id = (int)prog.steps.size() + 1;

            prog.steps.push_back(s);

            stepChecked.resize(prog.steps.size(), false);

        }

        if (coloredButton(u8"\u0423\u0434\u0430\u043b\u0438\u0442\u044c", 0.055f, 0.392f, 0.612f, colW, btnH)) {

            // Delete checked steps, or selectedStep

            stepChecked.resize(prog.steps.size(), false);

            bool anyChecked = false;

            for (int k = 0; k < (int)stepChecked.size(); k++)

                if (stepChecked[k]) { anyChecked = true; break; }

            if (anyChecked) {

                for (int k = (int)prog.steps.size() - 1; k >= 0; k--) {

                    if (k < (int)stepChecked.size() && stepChecked[k])

                        prog.steps.erase(prog.steps.begin() + k);

                }

                selectedStep = -1;

            } else if (selectedStep >= 0 && selectedStep < (int)prog.steps.size()) {

                prog.steps.erase(prog.steps.begin() + selectedStep);

                if (selectedStep >= (int)prog.steps.size())

                    selectedStep = (int)prog.steps.size() - 1;

            }

            for (int k = 0; k < (int)prog.steps.size(); k++)

                prog.steps[k].id = k + 1;

            stepChecked.assign(prog.steps.size(), false);

            needsRebuild = true;

        }

        ImGui::EndGroup();



        ImGui::SameLine();



        // Column 2: Вверх / Вниз (vertical)

        ImGui::BeginGroup();

        if (coloredButton(u8"\u0412\u0432\u0435\u0440\u0445", 0.25f, 0.35f, 0.55f, colW, btnH)) {

            stepChecked.resize(prog.steps.size(), false);

            bool anyChecked = false;

            for (int k = 0; k < (int)stepChecked.size(); k++)

                if (stepChecked[k]) { anyChecked = true; break; }

            if (anyChecked) {

                for (int k = 1; k < (int)prog.steps.size(); k++) {

                    if (k < (int)stepChecked.size() && stepChecked[k] &&

                        (k - 1 >= (int)stepChecked.size() || !stepChecked[k-1])) {

                        std::swap(prog.steps[k], prog.steps[k-1]);

                        std::swap(stepChecked[k], stepChecked[k-1]);

                    }

                }

                needsRebuild = true;

            } else if (selectedStep > 0 && selectedStep < (int)prog.steps.size()) {

                std::swap(prog.steps[selectedStep], prog.steps[selectedStep-1]);

                selectedStep--;

            }

        }

        if (coloredButton(u8"\u0412\u043d\u0438\u0437", 0.25f, 0.35f, 0.55f, colW, btnH)) {

            stepChecked.resize(prog.steps.size(), false);

            bool anyChecked = false;

            for (int k = 0; k < (int)stepChecked.size(); k++)

                if (stepChecked[k]) { anyChecked = true; break; }

            if (anyChecked) {

                for (int k = (int)prog.steps.size() - 2; k >= 0; k--) {

                    if (k < (int)stepChecked.size() && stepChecked[k] &&

                        (k + 1 >= (int)stepChecked.size() || !stepChecked[k+1])) {

                        std::swap(prog.steps[k], prog.steps[k+1]);

                        std::swap(stepChecked[k], stepChecked[k+1]);

                    }

                }

                needsRebuild = true;

            } else if (selectedStep >= 0 && selectedStep < (int)prog.steps.size()-1) {

                std::swap(prog.steps[selectedStep], prog.steps[selectedStep+1]);

                selectedStep++;

            }

        }

        ImGui::EndGroup();



        ImGui::SameLine();



        // Column 3: Реверс (centered vertically)

        ImGui::BeginGroup();

        if (coloredButton(u8"\u0420\u0435\u0432\u0435\u0440\u0441", 0.45f, 0.35f, 0.2f, colW, btnH * 2 + ImGui::GetStyle().ItemSpacing.y)) {

            std::reverse(prog.steps.begin(), prog.steps.end());

            for (int i = 0; i < (int)prog.steps.size(); i++)

                prog.steps[i].id = i + 1;

            selectedStep = -1;

            needsRebuild = true;

        }

        ImGui::EndGroup();

    }



    // ─── Table with units in headers ─────────────────────────────

    float tableH = ImGui::GetContentRegionAvail().y - 30;

    if (tableH < 100) tableH = 100;

    if (ImGui::BeginTable("Steps", 7,

            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |

            ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable,

            ImVec2(0, tableH)))

    {

        ImGui::TableSetupColumn("",                     ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize, 22);

        ImGui::TableSetupColumn("#",                    ImGuiTableColumnFlags_WidthFixed, 28);

        ImGui::TableSetupColumn(u8"\u0422\u0438\u043f",                ImGuiTableColumnFlags_WidthFixed, 80);

        ImGui::TableSetupColumn(u8"\u041f\u043e\u0434\u0430\u0447\u0430, \u043c\u043c",      ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableSetupColumn(u8"\u0420\u043e\u0442\u0430\u0446., \u00b0",      ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableSetupColumn(u8"\u0423\u0433\u043e\u043b, \u00b0",          ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableSetupColumn(u8"\u0414\u0443\u0433\u0430, \u043c\u043c",       ImGuiTableColumnFlags_WidthFixed, 58);

        ImGui::TableHeadersRow();



        ImU32 cutColor    = IM_COL32(100, 45, 45, 255);

        ImU32 activeColor = IM_COL32(30, 80, 30, 255);



        static const char* stepTypes[] = { u8"\u0428\u0430\u0433", u8"\u0420\u0435\u0437" };



        // Ensure stepChecked vector matches steps size

        stepChecked.resize(prog.steps.size(), false);



        for (int i = 0; i < (int)prog.steps.size(); i++) {

            auto& s = prog.steps[i];

            ImGui::TableNextRow();

            bool isSel  = (selectedStep == i);

            bool isChecked = (i < (int)stepChecked.size() && stepChecked[i]);

            bool active = (animStepIndex == i) && (animActive || canContinueCycle || animProgressFrac > 0.0f);



            // Color rows — highlight only when checkbox is checked

            ImU32 selColor = IM_COL32(30, 50, 120, 255);

            ImU32 collisionColor = IM_COL32(170, 45, 45, 255);

            bool isCollision = (collisionStepIdx == i);

            if (isCollision) {

                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, collisionColor);

                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, collisionColor);

            } else if (isChecked && !active && !s.isCut) {

                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, selColor);

                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, selColor);

            } else if (s.isCut) {

                ImU32 c = (isChecked && !active) ? selColor : (active ? activeColor : cutColor);

                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, c);

                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, c);

            } else if (active) {

                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, activeColor);

                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, activeColor);

            }



            // Column 0: Checkbox

            ImGui::TableSetColumnIndex(0);

            ImGui::PushID(7000 + i);

            bool chk = stepChecked[i] != 0;

            if (ImGui::Checkbox("##chk", &chk))

                stepChecked[i] = chk ? 1 : 0;

            ImGui::PopID();



            // Column 1: # — selectable

            ImGui::TableSetColumnIndex(1);

            char lbl[16];

            snprintf(lbl, sizeof(lbl), "%d##r%d", i + 1, i);

            if (ImGui::Selectable(lbl, isSel, 0, ImVec2(0, 0)))

                selectedStep = i;



            // Drag-drop source

            if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {

                ImGui::SetDragDropPayload("STEP_REORDER", &i, sizeof(int));

                ImGui::Text(u8"\u0428\u0430\u0433 %d", i + 1);

                ImGui::EndDragDropSource();

            }

            // Drag-drop target

            if (ImGui::BeginDragDropTarget()) {

                if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("STEP_REORDER")) {

                    int srcIdx = *(const int*)payload->Data;

                    if (srcIdx != i && srcIdx >= 0 && srcIdx < (int)prog.steps.size()) {

                        BendStep tmp = prog.steps[srcIdx];

                        prog.steps.erase(prog.steps.begin() + srcIdx);

                        int insertAt = (srcIdx < i) ? i - 1 : i;

                        if (insertAt < 0) insertAt = 0;

                        prog.steps.insert(prog.steps.begin() + insertAt, tmp);

                        for (int k = 0; k < (int)prog.steps.size(); k++)

                            prog.steps[k].id = k + 1;

                        if (selectedStep == srcIdx) selectedStep = insertAt;

                        else selectedStep = -1;

                    }

                }

                ImGui::EndDragDropTarget();

            }



            // Column 2: Type dropdown (Шаг / Рез)

            ImGui::TableSetColumnIndex(2);

            if (isCollision) {

                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.3f, 0.3f, 1.f));

                ImGui::Text(u8"Столкн.");

                ImGui::PopStyleColor();

            } else if (active) {

                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.3f, 1.f, 0.3f, 1.f));

                ImGui::Text(u8"*");

                ImGui::PopStyleColor();

            } else {

                ImGui::PushID(5000 + i);

                ImGui::SetNextItemWidth(-1);

                int typeIdx = s.isCut ? 1 : 0;

                if (ImGui::Combo("##type", &typeIdx, stepTypes, 2)) {

                    bool wasCut = s.isCut;

                    s.isCut = (typeIdx == 1);

                    if (s.isCut && !wasCut) {

                        // Switched to Рез — init default cut

                        s.feedLength = 0; s.rotation = 0; s.bendAngle = 0;

                        if (s.cutDef.empty()) {

                            CutPlane cp; cp.nx = 0; cp.ny = -1; cp.nz = 0; cp.offsetAlongAxis = 0;

                            s.cutDef.planes.push_back(cp);

                        }

                    } else if (!s.isCut && wasCut) {

                        // Switched to Шаг — clear cut data

                        s.cutDef = {};

                    }

                    needsRebuild = true;

                }

                ImGui::PopID();

            }



            if (s.isCut) {

                // ─── Рез row: show cut type name + Изменить button ───

                ImGui::TableSetColumnIndex(3);

                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.7f, 0.3f, 1.f));

                ImGui::Text("%s", s.cutDef.typeName().c_str());

                ImGui::PopStyleColor();



                ImGui::TableSetColumnIndex(4);

                ImGui::PushID(6000 + i);

                if (ImGui::SmallButton(u8"\u0418\u0437\u043c\u0435\u043d\u0438\u0442\u044c")) {

                    cutEditorStep_ = i;

                    showCutEditor_ = true;

                }

                ImGui::PopID();



                ImGui::TableSetColumnIndex(5); ImGui::TextDisabled("--");

                ImGui::TableSetColumnIndex(6); ImGui::TextDisabled("--");

            } else {

                // ─── Шаг row: editable feed / rotation / angle ───

                // Column 3: Feed

                ImGui::TableSetColumnIndex(3);

                if (active && animPhaseType == 0)

                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, IM_COL32(60, 180, 60, 100));

                ImGui::PushID(3000 + i*4);

                ImGui::SetNextItemWidth(-1);

                if (ImGui::InputDouble("##f", &s.feedLength, 0, 0, "%.2f",

                        ImGuiInputTextFlags_EnterReturnsTrue))

                    needsRebuild = true;

                ImGui::PopID();



                // Column 4: Rotation

                ImGui::TableSetColumnIndex(4);

                if (active && animPhaseType == 1)

                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, IM_COL32(60, 180, 60, 100));

                ImGui::PushID(3000 + i*4 + 1);

                ImGui::SetNextItemWidth(-1);

                if (ImGui::InputDouble("##r", &s.rotation, 0, 0, "%.2f",

                        ImGuiInputTextFlags_EnterReturnsTrue))

                    needsRebuild = true;

                ImGui::PopID();



                // Column 5: Angle

                ImGui::TableSetColumnIndex(5);

                if (active && animPhaseType == 2)

                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, IM_COL32(60, 180, 60, 100));

                ImGui::PushID(3000 + i*4 + 2);

                ImGui::SetNextItemWidth(-1);

                if (ImGui::InputDouble("##a", &s.bendAngle, 0, 0, "%.2f",

                        ImGuiInputTextFlags_EnterReturnsTrue))

                    needsRebuild = true;

                ImGui::PopID();



                // Column 6: Arc length (computed)

                ImGui::TableSetColumnIndex(6);

                if (std::abs(s.bendAngle) > 0.01) {

                    double arc = prog.R * std::abs(s.bendAngle) * M_PI / 180.0;

                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

                    ImGui::Text("%.1f", arc);

                    ImGui::PopStyleColor();

                } else {

                    ImGui::TextDisabled("--");

                }

            }

        }



        ImGui::EndTable();

    }

}



// ═══════════════════════════════════════════════════════════════

//  PROGRESS BAR

// ═══════════════════════════════════════════════════════════════

void UI::drawProgressBar(UICallbacks& cb) {

    bool showSnapshot = !animActive && (canContinueCycle || (animStepIndex >= 0 && animProgressFrac > 0.0f));

    if (animActive || showSnapshot) {

        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.3f, 1.f, 0.3f, 1.f));

        if (animTotalParts > 1) {

            ImGui::Text(u8"[%d/%d] %s  %.0f%%",

                animCurrentPart + 1, animTotalParts,

                animPhaseName.c_str(), animProgressFrac * 100.0f);

        } else {

            ImGui::Text(u8"%s  %.0f%%",

                animPhaseName.c_str(), animProgressFrac * 100.0f);

        }

        ImGui::PopStyleColor();



        ImGui::ProgressBar(animProgressFrac, ImVec2(-1, 0));



        if (showSnapshot && !animActive) {

            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.8f, 0.2f, 1.f));

            ImGui::TextWrapped(u8"Цикл остановлен/прерван. Показан текущий шаг и прогресс.");

            ImGui::PopStyleColor();

        }



        if (pedalRequired && !pedalPressed) {

            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.9f, 0.f, 1.f));

            ImGui::TextWrapped("%s", pedalPrompt.c_str());

            ImGui::PopStyleColor();

        }



        if (animPaused) {

            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.9f, 0.f, 1.f));

            ImGui::TextWrapped("%s", animPauseMsg.c_str());

            ImGui::PopStyleColor();

            float bw = ImGui::GetContentRegionAvail().x;

            if (coloredButton(u8"\u041f\u0440\u043e\u0434\u043e\u043b\u0436\u0438\u0442\u044c", 0.306f, 0.561f, 0.235f, bw)) {

                if (cb.onAnimContinue) cb.onAnimContinue();

            }

        }

    } else {

        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 0.5f, 0.5f, 1.f));

        ImGui::Text(u8"\u0413\u043e\u0442\u043e\u0432 \u043a \u0440\u0430\u0431\u043e\u0442\u0435");

        ImGui::PopStyleColor();

        ImGui::ProgressBar(0.0f, ImVec2(-1, 0));

    }

}



// ═══════════════════════════════════════════════════════════════

//  ACTION BUTTONS — Progress only (Start/Stop moved to Control window)

// ═══════════════════════════════════════════════════════════════

void UI::drawActionButtons(BendingProgram& prog, UICallbacks& cb) {

    drawProgressBar(cb);

}



// ═══════════════════════════════════════════════════════════════

//  ОТЛАДКА WINDOW — axes, toggles (no scale hint)

// ═══════════════════════════════════════════════════════════════

void UI::drawDebugWindow(UICallbacks& cb) {

    ImGuiWindowFlags dbFlags = ImGuiWindowFlags_NoCollapse;

    bool open = showDebugWin;

    ImGui::Begin(u8"Отладка###Debug", &open, dbFlags);

    if (!open) showDebugWin = false;

    // ⚠️ НЕ ПЕРЕЗАПИСЫВАЕМ флаги из Teensy! Они управляются ТОЛЬКО кнопками!
    // Teensy feedback используется ТОЛЬКО для логирования/отладки, не для переустановки состояния
    // fprintf(stderr, "[DEBUG] Teensy: clampClosed=%d, unclampOpen=%d\n", ts.clampClosed, ts.unclampOpen);

    // ═════════════════════════════════════════════════════════════
    // BLOCK 1: VIEWPORT
    // ═════════════════════════════════════════════════════════════
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.8f, 0.9f, 1.0f, 1.0f));

    ImGui::Text(u8"[ ОКНА / VIEWPORT ]");

    ImGui::PopStyleColor();

    float halfW = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;

    ImGui::Checkbox(u8"Станок (STP)", &showBender);

    ImGui::SameLine(halfW);

    ImGui::Checkbox(u8"Лазер", &showLaserHead);

    ImGui::Checkbox(u8"Неопрозрачный", &benderOpaque);

    ImGui::SameLine(halfW);

    ImGui::Checkbox(u8"Зона исключ.", &showExclZone);

    // ═════════════════════════════════════════════════════════════
    // BLOCK 2: LOGIC & MODE
    // ═════════════════════════════════════════════════════════════
    ImGui::Spacing();

    ImGui::Separator();

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.8f, 0.2f, 1.0f));

    ImGui::Text(u8"[ ЛОГИКА И РЕЖИМ ]");

    ImGui::PopStyleColor();

    ImGui::Checkbox(u8"ЛОГИКА 2", &useLogic2);

    if (!collisionInfo.empty()) {

        ImGui::TextWrapped(u8"! Столкн: %s", collisionInfo.c_str());

    }

    ImGui::Text(u8"Режим:");

    ImGui::SameLine();

    const char* modes[] = { u8"Гибка", u8"Гибка+Лазер", u8"Ручной" };

    ImGui::PushItemWidth(130);

    ImGui::Combo("##mode", &animMode, modes, 3);

    ImGui::PopItemWidth();

    bool isManual = (animMode == 2);

    if (isManual) ImGui::BeginDisabled();

    ImGui::Checkbox(u8"Авто подача", &autoFeed);

    ImGui::SameLine(halfW);

    ImGui::Checkbox(u8"Авто ротация", &autoRotation);

    ImGui::Checkbox(u8"Авто прижим", &autoClamp);

    if (isManual) ImGui::EndDisabled();

    // ═════════════════════════════════════════════════════════════
    // BLOCK 3: CALIBRATION / OSI
    // ═════════════════════════════════════════════════════════════
    ImGui::Spacing();

    ImGui::Separator();

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

    ImGui::Text(u8"[ КАЛИБРОВКА ]");

    ImGui::PopStyleColor();

    float col1W = 80.0f;

    float col2W = 80.0f;

    auto axisRow = [&](const char* name, double cur, double* manVal, const char* unit, const char* manId) {

        ImGui::PushID(manId);

        ImGui::Text("%s", name);

        ImGui::SameLine(col1W);

        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.85f, 0.85f, 0.95f, 1.0f));

        ImGui::Text("%.2f%s", cur, unit);

        ImGui::PopStyleColor();

        ImGui::SameLine(col1W + col2W + 6);

        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x - 5);

        ImGui::InputDouble(manId, manVal, 0, 0, "%.2f");

        ImGui::PopItemWidth();

        ImGui::PopID();

    };

    axisRow(u8"Гибка:", axisBend, &manualBend, u8"°", "##man_bend");

    axisRow(u8"Каретка:", axisCarriage, &manualCarriage, u8"мм", "##man_car");

    axisRow(u8"Патрон:", axisChuck, &manualChuck, u8"°", "##man_chk");

    if (ImGui::Button(u8"ПРИМЕНИТЬ КАЛИБР##axes", ImVec2(-1, 0))) {

        axisBend = manualBend;

        axisCarriage = manualCarriage;

        axisChuck = manualChuck;

        if (cb.onMachineCalibrate) {

            cb.onMachineCalibrate(manualCarriage, manualChuck, (int)std::round(manualBend));

        }

        if (machinePtr_ && machinePtr_->isConnected()) {

            infoText = u8"Kalibrovka sent to Teensy (EEPROM)";

        } else {

            infoText = u8"Sim: axes updated";

        }

    }

    // ═════════════════════════════════════════════════════════════
    // BLOCK 4: PEDAL & CONTROLS
    // ═════════════════════════════════════════════════════════════
    ImGui::Spacing();

    ImGui::Separator();

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

    ImGui::Text(u8"[ ПЕДАЛЬ И УПРАВЛЕНИЕ ]");

    ImGui::PopStyleColor();

    ImGui::Text(u8"Педаль D7:");

    ImGui::SameLine();

    if (pedalPressed)

        ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), u8"нажата");

    else

        ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.85f, 1.0f), u8"отпущена");

    if (machinePtr_ != nullptr) {

        ImGui::Checkbox(u8"Обход педали D7", &machinePtr_->pedalBypass_);

        ImGui::Checkbox(u8"Обход энкодера гибки", &machinePtr_->ignoreBendEncoder_);

    }

    // ═════════════════════════════════════════════════════════════
    // BLOCK 5: SETTINGS
    // ═════════════════════════════════════════════════════════════
    if (machinePtr_ != nullptr) {

        ImGui::Spacing();

        ImGui::Separator();

        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

        ImGui::Text(u8"[ ПАРАМЕТРЫ ]");

        ImGui::PopStyleColor();

        ImGui::SetNextItemWidth(150);

        float fs = (float)machinePtr_->feedSpeed_;

        if (ImGui::SliderFloat(u8"Скорость подачи (мм/сек)", &fs, 10.0f, 1000.0f)) machinePtr_->feedSpeed_ = fs;

        ImGui::SetNextItemWidth(150);

        float rs = (float)machinePtr_->rotSpeed_;

        if (ImGui::SliderFloat(u8"Rotation Speed (deg/s)", &rs, 5.0f, 500.0f)) machinePtr_->rotSpeed_ = rs;

        ImGui::SetNextItemWidth(150);

        ImGui::InputInt(u8"Clamp Time (ms) / Prizhim", &machinePtr_->clampDelayMs_, 100, 1000);

        ImGui::SetNextItemWidth(150);

        ImGui::InputInt(u8"Unclamp Time (ms) / Разжим", &machinePtr_->unclampDelayMs_, 100, 1000);

        // ═════════════════════════════════════════════════════════
        // КНОПКИ УПРАВЛЕНИЯ ПНЕВМАТИКОЙ (РЕЛЕ)
        // ═════════════════════════════════════════════════════════
        ImGui::Spacing();

        ImGui::Separator();

        // ═════════════════════════════════════════════════════════
        // BLOCK 6: ДИАГНОСТИКА
        // ═════════════════════════════════════════════════════════
        ImGui::Spacing();

        ImGui::Separator();

        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 1.0f, 0.4f, 1.0f));

        ImGui::Text(u8"[ ДИАГНОСТИКА ]");

        ImGui::Text(u8"Скор. гибки: %.1f °/сек", machinePtr_->bendSpeedDegSec_);

        ImGui::PopStyleColor();

    }

    ImGui::End();

}



// ═══════════════════════════════════════════════════════════════

//  УПРАВЛЕНИЕ WINDOW — encoder, zero, start/stop

// ═══════════════════════════════════════════════════════════════

void UI::drawControlWindow(UICallbacks& cb) {

    ImGuiWindowFlags ctlFlags = ImGuiWindowFlags_NoCollapse;



    bool open = showControlWin;

    ImGui::Begin(u8"\u0423\u043f\u0440\u0430\u0432\u043b\u0435\u043d\u0438\u0435###Control", &open, ctlFlags);

    if (!open) showControlWin = false;



    Machine* mctl = machinePtr_;

    bool machineConn = mctl && mctl->isConnected();

    bool simMode = !machineConn;



    // Simulation badge

    if (simMode) {

        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.2f, 1.0f));

        ImGui::TextWrapped(u8"\u26a0 \u0420\u0435\u0436\u0438\u043c \u0441\u0438\u043c\u0443\u043b\u044f\u0446\u0438\u0438 (Teensy \u043d\u0435 \u043f\u043e\u0434\u043a\u043b\u044e\u0447\u0451\u043d)");

        ImGui::PopStyleColor();

    }



    // ─────────────────────────────────────────────────────────────

    //  Энкодер гибки + сброс

    // ─────────────────────────────────────────────────────────────

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

    ImGui::Text(u8"\u0423\u0433\u043e\u043b \u044d\u043d\u043a\u043e\u0434\u0435\u0440\u0430:");

    ImGui::PopStyleColor();

    ImGui::SameLine();

    ImGui::Text(u8"%.2f\u00b0", axisBend);



    ImGui::SameLine();

    if (ImGui::SmallButton(u8"\u0421\u0431\u0440\u043e\u0441 \u044d\u043d\u043a\u043e\u0434\u0435\u0440\u0430")) {

        showMachineZeroConfirm_ = true;

    }

    if (ImGui::IsItemHovered())

        ImGui::SetTooltip(u8"\u0421\u0431\u0440\u043e\u0441\u0438\u0442\u044c \u0443\u0433\u043e\u043b \u044d\u043d\u043a\u043e\u0434\u0435\u0440\u0430 \u0433\u0438\u0431\u043a\u0438 \u0432 0\u00b0.\n\u041d\u0430 \u0441\u0442\u0430\u043d\u043a\u0435: \u043e\u0442\u043f\u0440\u0430\u0432\u043b\u044f\u0435\u0442 \u043a\u043e\u043c\u0430\u043d\u0434\u0443 ZERO \u0432 Teensy.\n\u0412 \u0441\u0438\u043c\u0443\u043b\u044f\u0446\u0438\u0438: \u043e\u0431\u043d\u0443\u043b\u044f\u0435\u0442 \u0442\u043e\u043b\u044c\u043a\u043e \u0438\u043d\u0434\u0438\u043a\u0430\u0442\u043e\u0440 \u0432 \u0438\u043d\u0442\u0435\u0440\u0444\u0435\u0439\u0441\u0435.");



    ImGui::Separator();



    // ─────────────────────────────────────────────────────────────

    //  Каретка / патрон — дублирующее управление

    // ─────────────────────────────────────────────────────────────

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

    ImGui::Text(u8"\u041a\u0430\u0440\u0435\u0442\u043a\u0430 \u0438 \u043f\u0430\u0442\u0440\u043e\u043d");

    ImGui::PopStyleColor();

    ImGui::Text(u8"  Z: %.2f \u043c\u043c    C: %.2f\u00b0", axisCarriage, axisChuck);



    {

        float fullW = ImGui::GetContentRegionAvail().x;

        float btnW  = (fullW - ImGui::GetStyle().ItemSpacing.x) * 0.5f;



        if (ImGui::Button(u8"\u041a\u0430\u0440\u0435\u0442\u043a\u0430 \u0432 0 (Home Z)", ImVec2(btnW, 0))) {

            if (machineConn) {

                if (cb.onMachineHome) cb.onMachineHome();

                infoText = u8"Команда Home Z отправлена";

            } else {

                axisCarriage = 0.0;

                infoText = u8"Сим: каретка обнулена в интерфейсе";

            }

        }

        if (ImGui::IsItemHovered())

            ImGui::SetTooltip(u8"\u0412\u044b\u0432\u0435\u0441\u0442\u0438 \u043a\u0430\u0440\u0435\u0442\u043a\u0443 (\u043e\u0441\u044c Z) \u0432 \u043d\u043e\u043b\u044c.");

        ImGui::SameLine();

        if (ImGui::Button(u8"\u041f\u0430\u0442\u0440\u043e\u043d \u0432 0 (Home C)", ImVec2(btnW, 0))) {

            if (machineConn) {

                if (cb.onMachineHomeC) cb.onMachineHomeC();

                infoText = u8"Команда Home C отправлена";

            } else {

                axisChuck = 0.0;

                infoText = u8"Сим: патрон обнулён в интерфейсе";

            }

        }

        if (ImGui::IsItemHovered())

            ImGui::SetTooltip(u8"\u0412\u044b\u0432\u0435\u0441\u0442\u0438 \u043f\u0430\u0442\u0440\u043e\u043d (\u043e\u0441\u044c C) \u0432 \u043d\u043e\u043b\u044c.");



        // Servo ON / OFF row

        if (machineConn) {

            // ─── RELAY CONTROLS: MUTUALLY EXCLUSIVE ──────────────────────────────
            // Sync with Teensy only if no pending command or timeout expired
            // This prevents Teensy state from overwriting user's UI feedback
            {
                const auto& ts = machinePtr_->status();
                if (ts.valid) {
                    double timeSinceSend = ImGui::GetTime() - relayCommandSendTime_;
                    bool canSyncFromTeensy = (relayPendingCommand_ == 0) ||
                                             (timeSinceSend > RELAY_CONFIRM_TIMEOUT);

                    if (canSyncFromTeensy) {
                        // Teensy has confirmed or timeout: apply actual state
                        relayPendingCommand_ = 0;
                        relayClampToggle_   = ts.clampClosed;
                        relayUnclampToggle_ = !ts.clampClosed;
                        relayLastCommand_ = ts.clampClosed ? 1 : 2;
                    }
                }
            }

            // If we have a pending command, show that state to user immediately
            // Don't let Teensy override the button feedback during command processing
            if (relayPendingCommand_ == 1) {
                relayClampToggle_   = true;
                relayUnclampToggle_ = false;
            } else if (relayPendingCommand_ == 2) {
                relayClampToggle_   = false;
                relayUnclampToggle_ = true;
            }

            // Build button labels showing current state
            std::string clampLabel   = u8"ПРИЖИМ";
            if (relayClampToggle_)   clampLabel   += u8" ON";
            std::string unclampLabel = u8"РАЗЖИМ";
            if (relayUnclampToggle_) unclampLabel += u8" ON";

            // BUTTON 1: ПРИЖИМ
            if (coloredButton(clampLabel.c_str(),
                              relayClampToggle_ ? 0.18f : 0.3f,   // green if active
                              relayClampToggle_ ? 0.55f : 0.35f,
                              relayClampToggle_ ? 0.18f : 0.2f,
                              btnW)) {
                if (relayClampToggle_) {
                    // Already ON → turn OFF (send RELAYOFF to de-energize BOTH relays)
                    relayClampToggle_   = false;
                    relayUnclampToggle_ = false;
                    relayPendingCommand_ = 0;
                    relayLastCommand_   = 0;
                    machinePtr_->sendRelayOff();
                    infoText = u8"ПРИЖИМ OFF";
                } else if (relayUnclampToggle_) {
                    // РАЗЖИМ is ON → switch to CLAMP (switch active relay)
                    relayUnclampToggle_ = false;
                    relayClampToggle_   = true;
                    relayPendingCommand_ = 1;
                    relayLastCommand_   = 1;
                    relayCommandSendTime_ = ImGui::GetTime();
                    machinePtr_->sendClamp();
                    infoText = u8"ПРИЖИМ ON (ожидание...)";
                } else {
                    // Both OFF → turn ON CLAMP
                    relayPendingCommand_ = 1;
                    relayLastCommand_   = 1;
                    relayCommandSendTime_ = ImGui::GetTime();
                    machinePtr_->sendClamp();
                    infoText = u8"ПРИЖИМ ON (ожидание...)";
                }
            }

            ImGui::SameLine();

            // BUTTON 2: РАЗЖИМ
            if (coloredButton(unclampLabel.c_str(),
                              relayUnclampToggle_ ? 0.55f : 0.3f,   // orange/red if active
                              relayUnclampToggle_ ? 0.45f : 0.35f,
                              relayUnclampToggle_ ? 0.18f : 0.2f,
                              btnW)) {
                if (relayUnclampToggle_) {
                    // Already ON → turn OFF (send RELAYOFF to de-energize BOTH relays)
                    relayUnclampToggle_ = false;
                    relayClampToggle_   = false;
                    relayPendingCommand_ = 0;
                    relayLastCommand_   = 0;
                    machinePtr_->sendRelayOff();
                    infoText = u8"РАЗЖИМ OFF";
                } else if (relayClampToggle_) {
                    // ПРИЖИМ is ON → switch to UNCLAMP (switch active relay)
                    relayClampToggle_   = false;
                    relayUnclampToggle_ = true;
                    relayPendingCommand_ = 2;
                    relayLastCommand_   = 2;
                    relayCommandSendTime_ = ImGui::GetTime();
                    machinePtr_->sendUnclamp();
                    infoText = u8"РАЗЖИМ ON (ожидание...)";
                } else {
                    // Both OFF → turn ON UNCLAMP
                    relayPendingCommand_ = 2;
                    relayLastCommand_   = 2;
                    relayCommandSendTime_ = ImGui::GetTime();
                    machinePtr_->sendUnclamp();
                    infoText = u8"РАЗЖИМ ON (ожидание...)";
                }
            }

            // Servo ON / OFF row

            if (coloredButton(u8"Servo ON", 0.18f, 0.55f, 0.18f, btnW)) {

                if (cb.onMachineServoOn) cb.onMachineServoOn();

                infoText = u8"Servo ON";

            }

            ImGui::SameLine();

            if (coloredButton(u8"Servo OFF", 0.55f, 0.45f, 0.18f, btnW)) {

                if (cb.onMachineServoOff) cb.onMachineServoOff();

                infoText = u8"Servo OFF";

            }



            // E-Stop full width

            if (coloredButton(u8"E-STOP", 0.75f, 0.10f, 0.10f, fullW)) {

                if (cb.onMachineEstop) cb.onMachineEstop();

                infoText = u8"АВАРИЙНЫЙ СТОП";

            }

            if (ImGui::IsItemHovered())

                ImGui::SetTooltip(u8"\u0410\u0432\u0430\u0440\u0438\u0439\u043d\u0430\u044f \u043e\u0441\u0442\u0430\u043d\u043e\u0432\u043a\u0430 \u0432\u0441\u0435\u0445 \u043e\u0441\u0435\u0439 \u0438 \u0432\u044b\u043a\u043b. SON.");

        }

    }



    ImGui::Separator();



    // ====== HARD PEDAL D7 INTERLOCK UI ======

    if (machinePtr_ != nullptr) {

        if (machinePtr_->pedalInterlockPaused_) {

            // Blinking red text

            float time = (float)ImGui::GetTime();

            float pulse = (sinf(time * 10.0f) + 1.0f) * 0.5f; // 0.0 to 1.0

            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, pulse * 0.5f, pulse * 0.5f, 1.0f));

            ImGui::TextWrapped(u8"⚠ ОСТАНОВКА: Педаль отпущена. Нажмите педаль для продолжения.");

            ImGui::PopStyleColor();

        }

    }

    // ========================================



    // Progress / status

    drawProgressBar(cb);



    ImGui::Separator();



    // Start / Stop

    float bw = ImGui::GetContentRegionAvail().x;

    if (!animActive) {

        bool canAbortStoppedCycle = canContinueCycle || (animStepIndex >= 0 && animProgressFrac > 0.0f);

        bool stockBlocked = !canContinueCycle && !stockEnough;

        if (stockBlocked)

            ImGui::BeginDisabled();

        bool machineOnline = machinePtr_ && machinePtr_->isConnected() && machinePtr_->status().valid;

        const char* startLabel = (canContinueCycle && !machineOnline) ? continueButtonLabel.c_str() : (machineOnline ? u8"\u0421\u0442\u0430\u0440\u0442 \u0441\u0442\u0430\u043d\u043a\u0430" : u8"\u0421\u0442\u0430\u0440\u0442");

        if (coloredButton(startLabel, 0.306f, 0.561f, 0.235f, bw)) {

            if (canContinueCycle && !machineOnline) {

                if (cb.onAnimContinue) cb.onAnimContinue();

            } else if (machineOnline && collisionChecked) {

                // Connected to real machine + collision OK — run program on hardware

                if (cb.onMachineRun) cb.onMachineRun();

            } else if (!machineOnline && collisionChecked) {

                // Offline simulation — no machine connected

                if (animMode == 1) { if (cb.onLaserVisualize) cb.onLaserVisualize(); }

                else               { if (cb.onVisualize) cb.onVisualize(); }

            } else {

                showCollisionPrompt_ = true;

            }

        }

        // Визуализация — direct, no collision check

        if (ImGui::Button(u8"\u0412\u0438\u0437\u0443\u0430\u043b\u0438\u0437\u0430\u0446\u0438\u044f", ImVec2(bw, 0))) {

            if (animMode == 1) { if (cb.onLaserVisualize) cb.onLaserVisualize(); }

            else               { if (cb.onVisualize) cb.onVisualize(); }

        }

        if (canAbortStoppedCycle) {

            if (coloredButton(u8"\u041e\u0442\u043c\u0435\u043d\u0430", 0.45f, 0.20f, 0.20f, bw)) {

                if (cb.onCancel) cb.onCancel();

            }

        }

        if (stockBlocked)

            ImGui::EndDisabled();

        if (stockBlocked) {

            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.9f, 0.0f, 1.0f));

            ImGui::TextWrapped(u8"Недостаточно длины хлыста для выбранного количества деталей.");

            ImGui::PopStyleColor();

        }

        // Show check status

        if (collisionChecked) {

            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.3f, 1.f, 0.3f, 1.f));

            ImGui::Text(u8"[OK] \u041f\u0440\u043e\u0432\u0435\u0440\u043a\u0430 OK");

            ImGui::PopStyleColor();

        } else if (collisionStepIdx >= 0) {

            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.3f, 0.3f, 1.f));

            ImGui::Text(u8"[X] \u041e\u0448\u0438\u0431\u043a\u0430 \u043d\u0430 \u0448\u0430\u0433\u0435 %d", collisionStepIdx + 1);

            ImGui::PopStyleColor();

        }

    } else {

        float halfW = (bw - ImGui::GetStyle().ItemSpacing.x) * 0.5f;

        if (coloredButton(u8"\u0421\u0442\u043e\u043f", 0.7f, 0.1f, 0.1f, halfW)) {

            if (cb.onStop) cb.onStop();

        }

        ImGui::SameLine();

        if (coloredButton(u8"\u041e\u0442\u043c\u0435\u043d\u0430", 0.45f, 0.20f, 0.20f, halfW)) {

            if (cb.onCancel) cb.onCancel();

        }

    }



    if (showMachineZeroConfirm_) {

        ImGui::OpenPopup(u8"##ZeroConfirm");

        showMachineZeroConfirm_ = false;

    }

    if (ImGui::BeginPopupModal(u8"##ZeroConfirm", nullptr,

            ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar)) {

        ImGui::TextWrapped(u8"Подтвердите сброс энкодера.");

        float bw2 = 140;

        if (coloredButton(u8"Да", 0.306f, 0.561f, 0.235f, bw2)) {

            if (cb.onMachineZero) cb.onMachineZero();

            if (machinePtr_ && machinePtr_->isConnected()) {

                

                infoText = u8"Команда ZERO отправлена на станок";

            } else {

                axisBend = 0.0;

                infoText = u8"Станок не подключен: обнулён только индикатор";

            }

            ImGui::CloseCurrentPopup();

        }

        ImGui::SameLine();

        if (ImGui::Button(u8"Отмена", ImVec2(bw2, 0)))

            ImGui::CloseCurrentPopup();

        ImGui::EndPopup();

    }



    if (showCarriageHomeConfirm_) {

        ImGui::OpenPopup(u8"##CarriageHomeConfirm");

        showCarriageHomeConfirm_ = false;

    }

    if (ImGui::BeginPopupModal(u8"##CarriageHomeConfirm", nullptr,

            ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar)) {

        ImGui::TextWrapped(u8"Вернуть каретку в координату 0 по текущему encoder/StepDir-нулю?");

        float bw2 = 180;

        if (coloredButton(u8"Да", 0.306f, 0.561f, 0.235f, bw2)) {

            if (machinePtr_ && machinePtr_->isConnected()) {

                if (cb.onMachineHome) cb.onMachineHome();

                infoText = u8"Команда возврата каретки в 0 отправлена";

            } else {

                axisCarriage = 0.0;

                infoText = u8"Станок не подключен: каретка обнулена только в интерфейсе";

            }

            ImGui::CloseCurrentPopup();

        }

        ImGui::SameLine();

        if (ImGui::Button(u8"Отмена", ImVec2(bw2, 0)))

            ImGui::CloseCurrentPopup();

        ImGui::EndPopup();

    }

    // ═════════════════════════════════════════════════════════════

    //  RELAY CONTROL (WHEN TEENSY CONNECTED)

    // ═════════════════════════════════════════════════════════════

    // (Relay button moved to Servo ON/OFF row above)

    ImGui::End();

}



// ═══════════════════════════════════════════════════════════════

//  ПОДКЛЮЧЕНИЕ WINDOW — debug connection status

// ═══════════════════════════════════════════════════════════════



// Helper: draw a status node box with label and color

static int s_nodeBoxId = 0;

static void drawNodeBox(const char* label, ImVec4 color, float width = 0) {

    ImGui::PushID(s_nodeBoxId++);

    ImGui::PushStyleColor(ImGuiCol_Button, color);

    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, color);

    ImGui::PushStyleColor(ImGuiCol_ButtonActive, color);

    if (width > 0)

        ImGui::Button(label, ImVec2(width, 32));

    else

        ImGui::Button(label, ImVec2(0, 32));

    ImGui::PopStyleColor(3);

    ImGui::PopID();

}



static const ImVec4 COL_OK    = ImVec4(0.15f, 0.55f, 0.15f, 1.0f);

static const ImVec4 COL_WARN  = ImVec4(0.65f, 0.55f, 0.05f, 1.0f);

static const ImVec4 COL_ERR   = ImVec4(0.60f, 0.12f, 0.12f, 1.0f);

static const ImVec4 COL_OFF   = ImVec4(0.30f, 0.30f, 0.30f, 1.0f);



void UI::drawConnectionWindow(UICallbacks& cb) {

    bool open = showConnectionWin;

    ImGui::SetNextWindowSize(ImVec2(440, 720), ImGuiCond_FirstUseEver);

    ImGui::Begin(u8"\u041f\u043e\u0434\u043a\u043b\u044e\u0447\u0435\u043d\u0438\u0435###Connection", &open,

                 ImGuiWindowFlags_NoCollapse);

    if (!open) { showConnectionWin = false; ImGui::End(); return; }



    s_nodeBoxId = 0;  // reset per-frame counter for unique IDs



    Machine* m = machinePtr_;

    bool connected = m && m->isConnected();

    const TeensyStatus& st = m ? m->status() : TeensyStatus{};



    // ════════════════════════════════════════════════════════════

    //  Connect / Disconnect section (merged from Станок)

    // ════════════════════════════════════════════════════════════

    if (m) {

        float bw = ImGui::GetContentRegionAvail().x;

        if (!connected) {

            comPortScanTimer_ -= ImGui::GetIO().DeltaTime;

            if (comPortScanTimer_ <= 0 || comPorts_.empty()) {

                comPorts_ = SerialPort::listPorts();

                comPortScanTimer_ = 3.0f;

            }

            if (comPorts_.empty()) {

                ImGui::TextColored(ImVec4(1,0.5f,0.3f,1), u8"COM-\u043f\u043e\u0440\u0442\u044b \u043d\u0435 \u043d\u0430\u0439\u0434\u0435\u043d\u044b");

            } else {

                ImGui::SetNextItemWidth(bw * 0.5f);

                if (ImGui::BeginCombo("##port", selectedPort_ < (int)comPorts_.size() ? comPorts_[selectedPort_].c_str() : "")) {

                    for (int i = 0; i < (int)comPorts_.size(); i++) {

                        if (ImGui::Selectable(comPorts_[i].c_str(), selectedPort_ == i))

                            selectedPort_ = i;

                    }

                    ImGui::EndCombo();

                }

                ImGui::SameLine();

                if (coloredButton(u8"\u041f\u043e\u0434\u043a\u043b\u044e\u0447\u0438\u0442\u044c", 0.2f, 0.5f, 0.2f)) {

                    if (selectedPort_ < (int)comPorts_.size()) {

                        if (cb.onMachineConnect) cb.onMachineConnect(comPorts_[selectedPort_]);

                    }

                }

            }

        } else {

            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), u8"[OK] %s", m->portName().c_str());

            ImGui::SameLine();

            if (ImGui::SmallButton(u8"\u041e\u0442\u043a\u043b\u044e\u0447\u0438\u0442\u044c")) {

                if (cb.onMachineDisconnect) cb.onMachineDisconnect();

            }

        }

        ImGui::Separator();

    }



    float nodeW = ImGui::GetContentRegionAvail().x * 0.42f;

    float arrowW = ImGui::GetContentRegionAvail().x * 0.12f;



    // ════════════════════════════════════════════════════════════

    // 1. PC (EXE) ↔ Teensy 4.1  —  USB-Serial

    // ════════════════════════════════════════════════════════════

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

    ImGui::Text(u8"1. PC (EXE) \u2194 Teensy 4.1  \u2014  USB-Serial");

    ImGui::PopStyleColor();

    ImGui::Separator();



    // Node: PC

    drawNodeBox(u8"PC (TubeBender.exe)", COL_OK, nodeW);

    ImGui::SameLine();

    // Arrow

    {

        ImVec4 linkCol = connected ? COL_OK : COL_ERR;

        ImGui::PushStyleColor(ImGuiCol_Text, linkCol);

        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 6);

        ImGui::Text(connected ? u8" \u2501\u2501 OK \u2501\u2501 " : u8" \u2501\u2501 [X] \u2501\u2501 ");

        ImGui::PopStyleColor();

    }

    ImGui::SameLine();

    drawNodeBox(u8"Teensy 4.1", connected ? COL_OK : COL_OFF, nodeW);



    // Details

    if (connected) {

        ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), u8"  [OK] COM-\u043f\u043e\u0440\u0442: %s", m->portName().c_str());

        if (st.valid)

            ImGui::Text(u8"  \u041f\u0440\u043e\u0448\u0438\u0432\u043a\u0430: %s", st.version.empty() ? "?" : st.version.c_str());

        else

            ImGui::TextColored(ImVec4(1, 0.5f, 0.3f, 1), u8"  \u041e\u0436\u0438\u0434\u0430\u043d\u0438\u0435 \u043e\u0442\u0432\u0435\u0442\u0430...");

    } else {

        ImGui::TextColored(ImVec4(1, 0.4f, 0.3f, 1), u8"  [X] \u041d\u0435 \u043f\u043e\u0434\u043a\u043b\u044e\u0447\u0435\u043d\u043e. \u0412\u044b\u0431\u0435\u0440\u0438\u0442\u0435 COM-\u043f\u043e\u0440\u0442 \u0432\u0432\u0435\u0440\u0445\u0443.");

    }



    ImGui::Spacing(); ImGui::Spacing();



    // ════════════════════════════════════════════════════════════

    // 2. Teensy ↔ VFD Delta CP2000  —  Modbus RTU RS-485

    // ════════════════════════════════════════════════════════════

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

    ImGui::Text(u8"2. Teensy \u2194 VFD Delta CP2000  \u2014  Modbus RS-485");

    ImGui::PopStyleColor();

    ImGui::Separator();



    bool mbOk = connected && st.valid && st.mbLink < 2000;

    bool mbWarn = connected && st.valid && st.mbLink >= 2000 && st.mbLink < 10000;



    drawNodeBox(u8"Teensy 4.1", connected ? COL_OK : COL_OFF, nodeW);

    ImGui::SameLine();

    {

        ImVec4 linkCol = mbOk ? COL_OK : (mbWarn ? COL_WARN : COL_ERR);

        if (!connected) linkCol = COL_OFF;

        ImGui::PushStyleColor(ImGuiCol_Text, linkCol);

        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 6);

        ImGui::Text(mbOk ? u8" \u2501\u2501 OK \u2501\u2501 " : u8" \u2501\u2501 [X] \u2501\u2501 ");

        ImGui::PopStyleColor();

    }

    ImGui::SameLine();

    drawNodeBox(u8"VFD CP2000", mbOk ? COL_OK : COL_OFF, nodeW);



    if (connected && st.valid) {

        if (mbOk) {

            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), u8"  [OK] Modbus: OK  (\u043f\u043e\u0441\u043b\u0435\u0434\u043d\u0438\u0439 \u043e\u0442\u0432\u0435\u0442 %lu \u043c\u0441 \u043d\u0430\u0437\u0430\u0434)", st.mbLink);

        } else {

            ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), u8"  [X] Modbus: \u041d\u0415\u0422 \u0421\u0412\u042f\u0417\u0418 (link=%lu \u043c\u0441)", st.mbLink);

        }

        ImGui::Text(u8"  RX: %lu  TX: %lu  ERR: %lu", st.mbRx, st.mbTx, st.mbErr);

        ImGui::Text(u8"  VFD: %.2f Гц  |  %.2f A", st.vfdFreqHz, st.vfdCurrentA);

        if (st.mbErr > 0)

            ImGui::TextColored(ImVec4(1, 0.5f, 0.0f, 1), u8"  \u26a0 \u041e\u0448\u0438\u0431\u043a\u0438 \u043e\u0431\u043c\u0435\u043d\u0430: %lu", st.mbErr);



        ImGui::Text(u8"  \u0413\u0438\u0431\u043a\u0430: %d\u00b0/%d\u00b0  \u0446\u0438\u043a\u043b:%d  \u0440\u0435\u0436\u0438\u043c:%s  %s",

                    st.bendCur, st.bendTgt, st.bendCyc,

                    st.modeAuto ? u8"\u0430\u0432\u0442\u043e" : u8"\u0440\u0443\u0447\u043d.",

                    st.bendDone ? u8"\u0433\u043e\u0442\u043e\u0432\u043e" : "");

        if (m->isProgramRunning()) {

            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), u8"  \u041f\u0440\u043e\u0433\u0440\u0430\u043c\u043c\u0430: \u0448\u0430\u0433 %d/%d  |  %s",

                               m->currentStep() + 1, std::max(1, m->totalSteps()), m->phaseText().c_str());

        } else if (m->totalSteps() > 0) {

            ImGui::Text(u8"  \u041f\u043e\u0441\u043b\u0435\u0434\u043d\u0438\u0439 \u0448\u0430\u0433: %d/%d  |  %s",

                        std::min(m->currentStep() + 1, std::max(1, m->totalSteps())),

                        std::max(1, m->totalSteps()), m->phaseText().c_str());

        }

        if (st.wdTrip)

            ImGui::TextColored(ImVec4(1, 0, 0, 1), u8"  !! WATCHDOG TRIP !!");

    } else if (connected) {

        ImGui::TextColored(ImVec4(1, 0.5f, 0.3f, 1), u8"  \u041e\u0436\u0438\u0434\u0430\u043d\u0438\u0435 \u0441\u0442\u0430\u0442\u0443\u0441\u0430...");

    } else {

        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1), u8"  \u2014 \u043d\u0435\u0442 \u0434\u0430\u043d\u043d\u044b\u0445");

    }



    ImGui::Spacing(); ImGui::Spacing();



    // ════════════════════════════════════════════════════════════

    // 3. Teensy → Сервоприводы  —  Step/Dir RS-422

    // ════════════════════════════════════════════════════════════

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

    ImGui::Text(u8"3. Teensy \u2192 \u0421\u0435\u0440\u0432\u043e\u043f\u0440\u0438\u0432\u043e\u0434\u044b  \u2014  Step/Dir RS-422");

    ImGui::PopStyleColor();

    ImGui::Separator();



    const char* stNames[] = { "Idle", "Moving", "Done", "Error", "Homing" };

    auto stName = [&](int s) -> const char* { return (s >= 0 && s < 5) ? stNames[s] : "?"; };



    // ── Servo Z ──

    {

        bool sonZ = connected && st.valid && st.zSon;

        bool errZ = connected && st.valid && st.zSt == 3;

        bool almZ = connected && st.valid && st.almZ;

        ImVec4 colZ = (errZ || almZ) ? COL_ERR : (sonZ ? COL_OK : COL_OFF);



        drawNodeBox(u8"Teensy", connected ? COL_OK : COL_OFF, nodeW * 0.5f);

        ImGui::SameLine();

        ImGui::PushStyleColor(ImGuiCol_Text, sonZ ? COL_OK : ((errZ || almZ) ? COL_ERR : COL_OFF));

        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 6);

        ImGui::Text(u8" \u2501\u2501 ");

        ImGui::PopStyleColor();

        ImGui::SameLine();

        drawNodeBox(u8"\u0421\u0435\u0440\u0432\u043e Z (MR-JE-200A)", colZ, nodeW * 1.2f);



        if (connected && st.valid) {

            ImGui::Text(u8"  SON: %s  |  \u041f\u043e\u0437\u0438\u0446\u0438\u044f: %.2f \u043c\u043c  |  \u0421\u0442\u0430\u0442\u0443\u0441: %s",

                        st.zSon ? "ON" : "OFF", st.zMm, stName(st.zSt));

            if (st.encValid)

                ImGui::Text(u8"  ENC Z: %ld \u0438\u043c\u043f.  |  %.2f \u043c\u043c", st.zFbCount, st.zFbMm);

            if (almZ)

                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), u8"  [X] \u0410\u041b\u0410\u0420\u041c \u0441\u0435\u0440\u0432\u043e Z! (D17)");

            else if (errZ)

                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), u8"  [X] \u041e\u0448\u0438\u0431\u043a\u0430 \u043e\u0441\u0438 Z!");

        } else {

            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1), u8"  \u2014 \u043d\u0435\u0442 \u0434\u0430\u043d\u043d\u044b\u0445");

        }

    }



    ImGui::Spacing();



    // ── Servo C ──

    {

        bool sonC = connected && st.valid && st.cSon;

        bool errC = connected && st.valid && st.cSt == 3;

        bool almC = connected && st.valid && st.almC;

        ImVec4 colC = (errC || almC) ? COL_ERR : (sonC ? COL_OK : COL_OFF);



        drawNodeBox(u8"Teensy", connected ? COL_OK : COL_OFF, nodeW * 0.5f);

        ImGui::SameLine();

        ImGui::PushStyleColor(ImGuiCol_Text, sonC ? COL_OK : ((errC || almC) ? COL_ERR : COL_OFF));

        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 6);

        ImGui::Text(u8" \u2501\u2501 ");

        ImGui::PopStyleColor();

        ImGui::SameLine();

        drawNodeBox(u8"\u0421\u0435\u0440\u0432\u043e C (MR-JE-200A)", colC, nodeW * 1.2f);



        if (connected && st.valid) {

            ImGui::Text(u8"  SON: %s  |  \u041f\u043e\u0437\u0438\u0446\u0438\u044f: %.2f\u00b0  |  \u0421\u0442\u0430\u0442\u0443\u0441: %s",

                        st.cSon ? "ON" : "OFF", st.cDeg, stName(st.cSt));

            if (st.encValid)

                ImGui::Text(u8"  ENC C: %ld \u0438\u043c\u043f.  |  %.2f\u00b0", st.cFbCount, st.cFbDeg);

            if (almC)

                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), u8"  [X] \u0410\u041b\u0410\u0420\u041c \u0441\u0435\u0440\u0432\u043e C! (D18)");

            else if (errC)

                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), u8"  [X] \u041e\u0448\u0438\u0431\u043a\u0430 \u043e\u0441\u0438 C!");

        } else {

            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1), u8"  \u2014 \u043d\u0435\u0442 \u0434\u0430\u043d\u043d\u044b\u0445");

        }

    }



    ImGui::Spacing(); ImGui::Spacing();



    // ════════════════════════════════════════════════════════════

    // 4. Входы Teensy (PC817): педали, режим, концевики

    // ════════════════════════════════════════════════════════════

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

    ImGui::Text(u8"4. Входы Teensy (PC817): педали, режим, концевики");

    ImGui::PopStyleColor();

    ImGui::Separator();



    ImGui::Text(u8"  Педаль FWD: %s", st.pedalFwd ? u8"нажата" : u8"отпущена");

    ImGui::Text(u8"  Педаль REV: %s", st.pedalRev ? u8"нажата" : u8"отпущена");

    ImGui::Text(u8"  Режим: %s", st.modeAuto ? u8"АВТО" : u8"РУЧНОЙ");

    ImGui::Text(u8"  Концевик Z- (линейный): %s", st.limZMin ? u8"СРАБОТАЛ" : u8"норма");

    ImGui::Text(u8"  Концевик Z+ (линейный): %s", st.limZMax ? u8"СРАБОТАЛ" : u8"норма");



    ImGui::Spacing(); ImGui::Spacing();

    ImGui::Separator();



    // ════════════════════════════════════════════════════════════

    // Summary

    // ════════════════════════════════════════════════════════════

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

    ImGui::Text(u8"\u0418\u0442\u043e\u0433:");

    ImGui::PopStyleColor();



    int okCount = 0, totalLinks = 4;

    if (connected) okCount++;

    if (mbOk) okCount++;

    if (connected && st.valid && st.zSon) okCount++;

    if (connected && st.valid && st.cSon) okCount++;



    if (okCount == totalLinks) {

        ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f),

            u8"[OK] \u0412\u0441\u0435 \u0443\u0437\u043b\u044b \u043d\u0430 \u0441\u0432\u044f\u0437\u0438. \u0421\u0438\u0441\u0442\u0435\u043c\u0430 \u0433\u043e\u0442\u043e\u0432\u0430 \u043a \u0440\u0430\u0431\u043e\u0442\u0435.");

    } else {

        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f),

            u8"\u26a0 \u0413\u043e\u0442\u043e\u0432\u043d\u043e\u0441\u0442\u044c: %d / %d \u0443\u0437\u043b\u043e\u0432", okCount, totalLinks);

        if (!connected)

            ImGui::BulletText(u8"\u041f\u043e\u0434\u043a\u043b\u044e\u0447\u0438\u0442\u0435 Teensy \u0447\u0435\u0440\u0435\u0437 USB (\u0432\u0432\u0435\u0440\u0445\u0443 \u044d\u0442\u043e\u0433\u043e \u043e\u043a\u043d\u0430)");

        if (connected && !mbOk)

            ImGui::BulletText(u8"Проверьте RS-485 кабель Teensy ↔ VFD (S+/S−, SG, 120 Ом)");

        if (connected && st.valid && !st.zSon)

            ImGui::BulletText(u8"\u0412\u043a\u043b\u044e\u0447\u0438\u0442\u0435 Servo Z (SON)");

        if (connected && st.valid && !st.cSon)

            ImGui::BulletText(u8"\u0412\u043a\u043b\u044e\u0447\u0438\u0442\u0435 Servo C (SON)");

    }



    // ════════════════════════════════════════════════════════════

    



        // Log

        ImGui::Separator();

        if (ImGui::CollapsingHeader(u8"\u041b\u043e\u0433")) {

            ImGui::BeginChild("##machLog", ImVec2(0, 150), true);

            auto& log = m->log();

            for (auto& line : log)

                ImGui::TextUnformatted(line.c_str());

            if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())

                ImGui::SetScrollHereY(1.0f);

            ImGui::EndChild();

            if (ImGui::SmallButton(u8"\u041e\u0447\u0438\u0441\u0442\u0438\u0442\u044c"))

                m->clearLog();

        }



    ImGui::End();

}



// ═══════════════════════════════════════════════════════════════

//  НАСТРОЙКИ WINDOW

// ═══════════════════════════════════════════════════════════════

void UI::drawSettingsWindow() {

    // ── Center the window on first appearance ──

    ImGuiViewport* vp = ImGui::GetMainViewport();

    ImVec2 winSize(650, 480);

    ImGui::SetNextWindowSize(winSize, ImGuiCond_FirstUseEver);

    ImGui::SetNextWindowPos(

        ImVec2(vp->GetCenter().x - winSize.x * 0.5f,

               vp->GetCenter().y - winSize.y * 0.5f),

        ImGuiCond_FirstUseEver);



    // NoDocking — always a separate floating window

    ImGuiWindowFlags flags =

        ImGuiWindowFlags_NoDocking |

        ImGuiWindowFlags_NoCollapse;



    ImGui::Begin(u8"\u041d\u0430\u0441\u0442\u0440\u043e\u0439\u043a\u0438###Settings", &showSettings_, flags);



    float avail = ImGui::GetContentRegionAvail().x;

    float navW  = 160.0f;  // left navigation panel width

    float pad   = ImGui::GetStyle().ItemSpacing.x;

    float rightW = avail - navW - pad;



    // ════════════ LEFT NAVIGATION ════════════

    ImGui::BeginChild("##nav", ImVec2(navW, -ImGui::GetFrameHeightWithSpacing() - 6), true);

    {

        struct NavItem { const char* label; };

        NavItem items[] = {

            { u8"\u041c\u043e\u0434\u0435\u043b\u0438" },

            { u8"\u0418\u043d\u0442\u0435\u0440\u0444\u0435\u0439\u0441" },

        };

        for (int i = 0; i < 2; i++) {

            bool sel = (settingsPage_ == i);

            if (sel) {

                ImGui::PushStyleColor(ImGuiCol_Header,        ImVec4(0.055f, 0.392f, 0.612f, 1.0f));

                ImGui::PushStyleColor(ImGuiCol_HeaderHovered,  ImVec4(0.055f, 0.392f, 0.612f, 0.8f));

            }

            char lbl[128];

            snprintf(lbl, sizeof(lbl), "  %s", items[i].label);

            if (ImGui::Selectable(lbl, sel, 0, ImVec2(0, 28)))

                settingsPage_ = i;

            if (sel) ImGui::PopStyleColor(2);

        }

    }

    ImGui::EndChild();



    ImGui::SameLine();



    // ════════════ RIGHT CONTENT ════════════

    ImGui::BeginChild("##content", ImVec2(rightW, -ImGui::GetFrameHeightWithSpacing() - 6), true);

    {

        // Helper: model path field (label + read-only input + "..." browse button)

        auto modelField = [this](const char* label, std::string& path, const char* filter, const char* ext) {

            ImGui::Text("%s", label);

            char buf[512];

            strncpy(buf, path.c_str(), sizeof(buf) - 1); buf[511] = 0;

            float w = ImGui::GetContentRegionAvail().x - 80;

            ImGui::PushItemWidth(w);

            char inputId[64];

            snprintf(inputId, sizeof(inputId), "##i_%s", label);

            ImGui::InputText(inputId, buf, sizeof(buf), ImGuiInputTextFlags_ReadOnly);

            ImGui::PopItemWidth();

            ImGui::SameLine();

            char btnId[64];

            snprintf(btnId, sizeof(btnId), "...##%s", label);

            if (ImGui::Button(btnId, ImVec2(70, 0))) {

                auto p = UI::openFileDialog(filter, ext);

                if (!p.empty()) {

                    path = p;

                    settingsSavedTimer_ = 3.0f;

                    settingsSavedMsg_ = u8"\u041c\u043e\u0434\u0435\u043b\u044c \u0434\u043e\u0431\u0430\u0432\u043b\u0435\u043d\u0430";

                }

            }

            if (ImGui::IsItemHovered() && !path.empty())

                ImGui::SetTooltip("%s", path.c_str());

            ImGui::Spacing();

        };



        if (settingsPage_ == 0) {

            // ── Page 0: Модели ──────────────────────────────

            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.878f, 0.424f, 0.467f, 1.0f));

            ImGui::Text(u8"\u041c\u043e\u0434\u0435\u043b\u0438 \u0442\u0440\u0443\u0431\u043e\u0433\u0438\u0431\u0430");

            ImGui::PopStyleColor();

            ImGui::Separator();

            ImGui::Spacing();



            modelField(u8"\u041c\u043e\u0434\u0435\u043b\u044c (\u0441\u0442\u0430\u043d\u043e\u043a)",

                settings.benderModelPath, "STEP Files\0*.stp;*.step\0All\0*.*\0", "stp");

            modelField(u8"\u041e\u0441\u044c + \u0442\u043e\u0447\u043a\u0430 \u0433\u0438\u0431\u0430",

                settings.benderAxisModelPath, "STEP Files\0*.stp;*.step\0All\0*.*\0", "stp");



            ImGui::Spacing();

            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

            ImGui::Text(u8"\u041f\u043e\u0434\u0432\u0438\u0436\u043d\u044b\u0435 \u0447\u0430\u0441\u0442\u0438");

            ImGui::PopStyleColor();

            ImGui::Separator();

            ImGui::Spacing();



            modelField(u8"\u0420\u043e\u043b\u0438\u043a",

                settings.rollerModelPath, "STEP Files\0*.stp;*.step\0All\0*.*\0", "stp");

            modelField(u8"\u041a\u0430\u0440\u0435\u0442\u043a\u0430",

                settings.carriageModelPath, "STEP Files\0*.stp;*.step\0All\0*.*\0", "stp");

            modelField(u8"\u041f\u0440\u0438\u0436\u0438\u043c",

                settings.clampModelPath, "STEP Files\0*.stp;*.step\0All\0*.*\0", "stp");

            modelField(u8"\u041f\u0430\u0442\u0440\u043e\u043d",

                settings.chuckModelPath, "STEP Files\0*.stp;*.step\0All\0*.*\0", "stp");



            ImGui::Spacing();

            ImGui::Separator();

            ImGui::Spacing();



            modelField(u8"\u0421\u0442\u0430\u0442\u0438\u0447\u043d\u0430\u044f \u043c\u043e\u0434\u0435\u043b\u044c",

                settings.staticModelPath, "STEP Files\0*.stp;*.step\0All\0*.*\0", "stp");



        } else if (settingsPage_ == 1) {

            // ── Page 1: Интерфейс ───────────────────────────

            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.878f, 0.424f, 0.467f, 1.0f));

            ImGui::Text(u8"\u0420\u0430\u0431\u043e\u0447\u0430\u044f \u043e\u0431\u043b\u0430\u0441\u0442\u044c");

            ImGui::PopStyleColor();

            ImGui::Separator();

            ImGui::Spacing();



            ImGui::Text(u8"\u0420\u0430\u0441\u043f\u043e\u043b\u043e\u0436\u0435\u043d\u0438\u0435 \u043e\u043a\u043e\u043d \u0441\u043e\u0445\u0440\u0430\u043d\u044f\u0435\u0442\u0441\u044f \u0430\u0432\u0442\u043e\u043c\u0430\u0442\u0438\u0447\u0435\u0441\u043a\u0438 \u043f\u0440\u0438 \u0432\u044b\u0445\u043e\u0434\u0435.");

            ImGui::Spacing();

            if (ImGui::Button(u8"\u0421\u043e\u0445\u0440\u0430\u043d\u0438\u0442\u044c \u0440\u0430\u0441\u043f\u043e\u043b\u043e\u0436\u0435\u043d\u0438\u0435 \u0441\u0435\u0439\u0447\u0430\u0441")) {

                saveLayoutNow();

            }

            ImGui::Spacing();

            if (ImGui::Button(u8"\u0421\u0431\u0440\u043e\u0441\u0438\u0442\u044c \u0440\u0430\u0441\u043f\u043e\u043b\u043e\u0436\u0435\u043d\u0438\u0435 \u043f\u043e \u0443\u043c\u043e\u043b\u0447\u0430\u043d\u0438\u044e")) {

                layoutInitialized_ = false;  // will rebuild next frame

            }



            ImGui::Spacing();

            ImGui::Separator();

            ImGui::Spacing();



            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

            ImGui::Text(u8"\u041c\u0430\u0441\u0448\u0442\u0430\u0431");

            ImGui::PopStyleColor();

            ImGui::Text(u8"\u0422\u0435\u043a\u0443\u0449\u0438\u0439: %.0f%%", uiScale * 100.0f);

            if (ImGui::Button(u8"+  \u0423\u0432\u0435\u043b\u0438\u0447\u0438\u0442\u044c", ImVec2(180, 0))) {

                uiScale = std::min(uiScale + 0.1f, 3.0f);

                needFontRebuild = true;

            }

            ImGui::SameLine();

            if (ImGui::Button(u8"-  \u0423\u043c\u0435\u043d\u044c\u0448\u0438\u0442\u044c", ImVec2(180, 0))) {

                uiScale = std::max(uiScale - 0.1f, 0.5f);

                needFontRebuild = true;

            }

            if (ImGui::Button(u8"\u0421\u0431\u0440\u043e\u0441 (100%)", ImVec2(180, 0))) {

                uiScale = 1.0f;

                touchMode = false;

                needFontRebuild = true;

            }



            ImGui::Spacing();

            ImGui::Separator();

            ImGui::Spacing();



            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

            ImGui::Text(u8"3D \u0440\u0435\u043d\u0434\u0435\u0440");

            ImGui::PopStyleColor();

            // ImGui::Checkbox(u8"\u0421\u043a\u0440\u044b\u0432\u0430\u0442\u044c \u043e\u0431\u0440\u0430\u0442\u043d\u044b\u0435 \u0442\u0440\u0435\u0443\u0433\u043e\u043b\u044c\u043d\u0438\u043a\u0438", &settings.backfaceCulling);

            // ImGui::TextDisabled(u8"\u041e\u0442\u043a\u043b\u044e\u0447\u0430\u0435\u0442 \u0433\u0440\u0430\u043d\u0438, \u043f\u043e\u0432\u0435\u0440\u043d\u0443\u0442\u044b\u0435 \u043e\u0442 \u043a\u0430\u043c\u0435\u0440\u044b.");

            ImGui::SliderFloat(u8"\u0412\u043d\u0443\u0442\u0440\u0435\u043d\u043d\u0435\u0435 \u043a\u0430\u0447\u0435\u0441\u0442\u0432\u043e \u0441\u0446\u0435\u043d\u044b", &settings.sceneRenderScale, 0.45f, 1.0f, "%.2f");

            ImGui::TextDisabled(u8"\u041c\u0435\u043d\u044c\u0448\u0435 \u0437\u043d\u0430\u0447\u0435\u043d\u0438\u0435 = \u043c\u0435\u043d\u044c\u0448\u0435 \u043d\u0430\u0433\u0440\u0443\u0437\u043a\u0430 \u043d\u0430 GPU.");

        }

    }

    ImGui::EndChild();



    // ════════════ BOTTOM BUTTONS ════════════

    // Decay confirmation timer

    if (settingsSavedTimer_ > 0.0f)

        settingsSavedTimer_ -= ImGui::GetIO().DeltaTime;



    float btnW = 180;

    float spacing = ImGui::GetStyle().ItemSpacing.x;

    float msgW = (settingsSavedTimer_ > 0.0f) ? ImGui::CalcTextSize(settingsSavedMsg_.c_str()).x + 20.0f : 0.0f;

    float totalBtnW = msgW + btnW * 2 + spacing * 2;

    ImGui::SetCursorPosX(std::max(0.0f, avail - totalBtnW));

    if (settingsSavedTimer_ > 0.0f) {

        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.3f, 1.f, 0.3f, 1.f));

        ImGui::Text("%s", settingsSavedMsg_.c_str());

        ImGui::PopStyleColor();

        ImGui::SameLine();

    }

    if (coloredButton(u8"\u0421\u043e\u0445\u0440\u0430\u043d\u0438\u0442\u044c", 0.306f, 0.561f, 0.235f, btnW)) {

        std::string settingsPath = exeDir_ + "/settings.json";

        settings.saveToFile(settingsPath);

        saveLayoutNow();

        settingsSavedTimer_ = 3.0f;

        settingsSavedMsg_ = u8"\u0421\u043e\u0445\u0440\u0430\u043d\u0435\u043d\u043e";

    }

    ImGui::SameLine();

    if (ImGui::Button(u8"\u0417\u0430\u043a\u0440\u044b\u0442\u044c", ImVec2(btnW, 0))) {

        showSettings_ = false;

    }



    ImGui::End();

}



// ═══════════════════════════════════════════════════════════════

//  СПРАВКА WINDOW

// ═══════════════════════════════════════════════════════════════

void UI::drawSpravkaSidebar() {

    ImGuiViewport* vp = ImGui::GetMainViewport();

    float projectBarH = (projects_.size() > 0) ? (ImGui::GetFrameHeight() + 6) : 0;

    float topY = vp->WorkPos.y + projectBarH;

    float height = vp->WorkSize.y - projectBarH;

    float collapsedW = 28.0f;

    float width = settings.spravkaSidebarOpen ? spravkaSidebarWidth_ : collapsedW;



    float posX = vp->WorkPos.x + vp->WorkSize.x - width;



    ImGui::SetNextWindowPos(ImVec2(posX, topY));

    ImGui::SetNextWindowSize(ImVec2(width, height));



    ImGuiWindowFlags flags =

        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |

        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |

        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking;



    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.11f, 0.11f, 0.115f, 1.0f));

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,

        settings.spravkaSidebarOpen ? ImVec2(8, 6) : ImVec2(2, 6));

    ImGui::Begin("##SpravkaSidebar", nullptr, flags);

    ImGui::PopStyleVar();

    ImGui::PopStyleColor();



    if (!settings.spravkaSidebarOpen) {

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.16f, 1.0f));

        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.25f, 0.27f, 1.0f));

        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.30f, 0.30f, 0.32f, 1.0f));

        if (ImGui::ArrowButton("##spravOpen", ImGuiDir_Left))

            settings.spravkaSidebarOpen = true;

        ImGui::PopStyleColor(3);

        if (ImGui::IsItemHovered()) ImGui::SetTooltip(u8"\u041f\u043e\u043a\u0430\u0437\u0430\u0442\u044c \u0441\u043f\u0440\u0430\u0432\u043a\u0443");

        ImGui::End();

        return;

    }



    // Header

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.878f, 0.424f, 0.467f, 1.0f));

    ImGui::Text(u8"\u0421\u041f\u0420\u0410\u0412\u041a\u0410");

    ImGui::PopStyleColor();

    ImGui::SameLine(ImGui::GetContentRegionAvail().x - 14);

    if (ImGui::Button("?", ImVec2(20, 0)))

        settings.spravkaSidebarOpen = false;

    if (ImGui::IsItemHovered()) ImGui::SetTooltip(u8"\u0421\u043a\u0440\u044b\u0442\u044c \u0441\u043f\u0440\u0430\u0432\u043a\u0443");

    ImGui::Separator();



    // Scrollable content

    ImGui::BeginChild("##SpravContent", ImVec2(0, 0), false);



    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

    ImGui::Text("TubeBender v2.2");

    ImGui::PopStyleColor();

    ImGui::Text("C++ / OpenCASCADE 7.9 / Dear ImGui / OpenGL 3.3");

    ImGui::Separator();

    ImGui::Spacing();



    // ── 1. Оснастка ──

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.878f, 0.424f, 0.467f, 1.0f));

    ImGui::Text(u8"1. \u041e\u0441\u043d\u0430\u0441\u0442\u043a\u0430");

    ImGui::PopStyleColor();

    ImGui::TextWrapped(u8"\u041d\u0430\u0441\u0442\u0440\u043e\u0439\u0442\u0435 \u043f\u0430\u0440\u0430\u043c\u0435\u0442\u0440\u044b \u0441\u0442\u0430\u043d\u043a\u0430 \u0432 \u043b\u0435\u0432\u043e\u0439 \u043f\u0430\u043d\u0435\u043b\u0438: "

        u8"\u0434\u0438\u0430\u043c\u0435\u0442\u0440 \u0442\u0440\u0443\u0431\u044b (D), \u0440\u0430\u0434\u0438\u0443\u0441 \u0433\u0438\u0431\u0430 (R), \u0434\u043b\u0438\u043d\u0443 \u0437\u0430\u0436\u0438\u043c\u0430, "

        u8"\u043f\u0430\u0440\u0430\u043c\u0435\u0442\u0440\u044b \u043b\u0430\u0437\u0435\u0440\u0430, \u043c\u0430\u0440\u043a\u0443 \u0441\u0442\u0430\u043b\u0438 \u0438 \u043a\u043e\u044d\u0444\u0444\u0438\u0446\u0438\u0435\u043d\u0442 \u043f\u0440\u0443\u0436\u0438\u043d\u0435\u043d\u0438\u044f. "

        u8"\u041c\u043e\u0436\u043d\u043e \u0437\u0430\u0433\u0440\u0443\u0437\u0438\u0442\u044c \u043f\u0440\u0435\u0441\u0435\u0442 \u0438\u043b\u0438 \u0441\u043e\u0445\u0440\u0430\u043d\u0438\u0442\u044c \u0442\u0435\u043a\u0443\u0449\u0443\u044e \u043d\u0430\u0441\u0442\u0440\u043e\u0439\u043a\u0443. "

        u8"\u041e\u0441\u043d\u0430\u0441\u0442\u043a\u0430 \u0433\u043b\u043e\u0431\u0430\u043b\u044c\u043d\u0430 \u0434\u043b\u044f \u0432\u0441\u0435\u0445 \u043f\u0440\u043e\u0435\u043a\u0442\u043e\u0432.");

    ImGui::Spacing();



    // ── 2. Проекты ──

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.878f, 0.424f, 0.467f, 1.0f));

    ImGui::Text(u8"2. \u041f\u0440\u043e\u0435\u043a\u0442\u044b");

    ImGui::PopStyleColor();

    ImGui::TextWrapped(u8"\u041a\u0430\u0436\u0434\u0430\u044f \u0432\u043a\u043b\u0430\u0434\u043a\u0430 \u0432\u0432\u0435\u0440\u0445\u0443 \u2014 \u043e\u0442\u0434\u0435\u043b\u044c\u043d\u044b\u0439 \u043f\u0440\u043e\u0435\u043a\u0442. "

        u8"\u0424\u0430\u0439\u043b \u2192 \u0421\u043e\u0437\u0434\u0430\u0442\u044c/\u041e\u0442\u043a\u0440\u044b\u0442\u044c/\u0421\u043e\u0445\u0440\u0430\u043d\u0438\u0442\u044c \u043f\u0440\u043e\u0435\u043a\u0442. "

        u8"\u041f\u0440\u0438 \u0437\u0430\u043a\u0440\u044b\u0442\u0438\u0438 \u0432\u043a\u043b\u0430\u0434\u043a\u0438 \u043f\u0440\u043e\u0433\u0440\u0430\u043c\u043c\u0430 \u0441\u043f\u0440\u043e\u0441\u0438\u0442 \u043e \u0441\u043e\u0445\u0440\u0430\u043d\u0435\u043d\u0438\u0438.");

    ImGui::Spacing();



    // ── 3. Программа гибки ──

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.878f, 0.424f, 0.467f, 1.0f));

    ImGui::Text(u8"3. \u041f\u0440\u043e\u0433\u0440\u0430\u043c\u043c\u0430 \u0433\u0438\u0431\u043a\u0438");

    ImGui::PopStyleColor();

    ImGui::TextWrapped(u8"\u0414\u043e\u0431\u0430\u0432\u043b\u044f\u0439\u0442\u0435 \u0448\u0430\u0433\u0438 (\u043f\u043e\u0434\u0430\u0447\u0430/\u0440\u043e\u0442\u0430\u0446\u0438\u044f/\u0443\u0433\u043e\u043b) \u0438 \u0440\u0435\u0437\u044b. "

        u8"\u041f\u0435\u0440\u0435\u043c\u0435\u0449\u0430\u0439\u0442\u0435 \u0441\u0442\u0440\u043e\u043a\u0438 \u043a\u043d\u043e\u043f\u043a\u0430\u043c\u0438 \u0412\u0432\u0435\u0440\u0445/\u0412\u043d\u0438\u0437 \u0438\u043b\u0438 \u043f\u0435\u0440\u0435\u0442\u0430\u0441\u043a\u0438\u0432\u0430\u043d\u0438\u0435\u043c. "

        u8"\u0420\u0435\u0432\u0435\u0440\u0441 \u043c\u0435\u043d\u044f\u0435\u0442 \u043f\u043e\u0440\u044f\u0434\u043e\u043a \u0448\u0430\u0433\u043e\u0432 \u043d\u0430 \u043e\u0431\u0440\u0430\u0442\u043d\u044b\u0439. "

        u8"\u0413\u0430\u043b\u043e\u0447\u043a\u0430\u043c\u0438 \u0432\u044b\u0434\u0435\u043b\u044f\u0439\u0442\u0435 \u0433\u0440\u0443\u043f\u043f\u0443 \u0441\u0442\u0440\u043e\u043a \u0434\u043b\u044f \u043f\u0435\u0440\u0435\u043c\u0435\u0449./\u0443\u0434\u0430\u043b.");

    ImGui::Spacing();



    // ── 4. Импорт/Экспорт ──

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.878f, 0.424f, 0.467f, 1.0f));

    ImGui::Text(u8"4. \u0418\u043c\u043f\u043e\u0440\u0442/\u042d\u043a\u0441\u043f\u043e\u0440\u0442");

    ImGui::PopStyleColor();

    ImGui::TextWrapped(u8"\u0422\u0440\u0443\u0431\u0430 \u2192 \u0418\u043c\u043f\u043e\u0440\u0442 STP: \u0437\u0430\u0433\u0440\u0443\u0437\u0438\u0442\u044c \u0433\u0435\u043e\u043c\u0435\u0442\u0440\u0438\u044e \u0438\u0437 STEP. "

        u8"\u041f\u0440\u0438 \u0438\u043c\u043f\u043e\u0440\u0442\u0435 \u043f\u0440\u043e\u0432\u0435\u0440\u044f\u044e\u0442\u0441\u044f D \u0438 R \u043f\u0440\u043e\u0442\u0438\u0432 \u043e\u0441\u043d\u0430\u0441\u0442\u043a\u0438. "

        u8"\u042d\u043a\u0441\u043f\u043e\u0440\u0442 STP \u0438 IGES \u0440\u0430\u0437\u0432\u0451\u0440\u0442\u043a\u0438 \u0434\u043e\u0441\u0442\u0443\u043f\u043d\u044b \u0438\u0437 \u043c\u0435\u043d\u044e \u0422\u0440\u0443\u0431\u0430.");

    ImGui::Spacing();



    // ── 5. 3D сцена ──

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.878f, 0.424f, 0.467f, 1.0f));

    ImGui::Text(u8"5. 3D \u0441\u0446\u0435\u043d\u0430");

    ImGui::PopStyleColor();

    ImGui::BulletText(u8"\u041b\u041a\u041c (\u0437\u0430\u0436\u0430\u0442\u044c)  \u2014  \u0432\u0440\u0430\u0449\u0435\u043d\u0438\u0435 \u043a\u0430\u043c\u0435\u0440\u044b");

    ImGui::BulletText(u8"\u041f\u041a\u041c (\u0437\u0430\u0436\u0430\u0442\u044c)  \u2014  \u043f\u0430\u043d\u043e\u0440\u0430\u043c\u0438\u0440\u043e\u0432\u0430\u043d\u0438\u0435");

    ImGui::BulletText(u8"\u041a\u043e\u043b\u0435\u0441\u043e \u043c\u044b\u0448\u0438     \u2014  \u043c\u0430\u0441\u0448\u0442\u0430\u0431");

    ImGui::BulletText(u8"Ctrl + \u043a\u043e\u043b\u0435\u0441\u043e / +/-  \u2014  \u043c\u0430\u0441\u0448\u0442\u0430\u0431 UI");

    ImGui::Spacing();



    // ── 6. Управление ──

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.878f, 0.424f, 0.467f, 1.0f));

    ImGui::Text(u8"6. \u0423\u043f\u0440\u0430\u0432\u043b\u0435\u043d\u0438\u0435");

    ImGui::PopStyleColor();

    ImGui::TextWrapped(u8"\u041e\u043a\u043d\u043e \u0423\u043f\u0440\u0430\u0432\u043b\u0435\u043d\u0438\u0435: \u0421\u0442\u0430\u0440\u0442/\u0421\u0442\u043e\u043f \u0432\u0438\u0437\u0443\u0430\u043b\u0438\u0437\u0430\u0446\u0438\u0438. "

        u8"\u041e\u043a\u043d\u043e \u041e\u0442\u043b\u0430\u0434\u043a\u0430: \u043f\u043e\u043a\u0430\u0437/\u0441\u043a\u0440\u044b\u0442\u0438\u0435 \u043c\u043e\u0434\u0435\u043b\u0435\u0439, \u0432\u044b\u0431\u043e\u0440 \u0440\u0435\u0436\u0438\u043c\u0430, \u0440\u0443\u0447\u043d\u043e\u0439 \u0432\u0432\u043e\u0434 \u043e\u0441\u0435\u0439.");

    ImGui::Spacing();



    // ── 7. Горячие клавиши ──

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.878f, 0.424f, 0.467f, 1.0f));

    ImGui::Text(u8"7. \u0413\u043e\u0440\u044f\u0447\u0438\u0435 \u043a\u043b\u0430\u0432\u0438\u0448\u0438");

    ImGui::PopStyleColor();

    ImGui::BulletText(u8"Ctrl+N  \u2014  \u043d\u043e\u0432\u044b\u0439 \u043f\u0440\u043e\u0435\u043a\u0442");

    ImGui::BulletText(u8"Ctrl+S  \u2014  \u0441\u043e\u0445\u0440\u0430\u043d\u0438\u0442\u044c \u043f\u0440\u043e\u0435\u043a\u0442");

    ImGui::BulletText(u8"Ctrl+Z  \u2014  \u043e\u0442\u043c\u0435\u043d\u0438\u0442\u044c");

    ImGui::BulletText(u8"Ctrl+Y  \u2014  \u043f\u043e\u0432\u0442\u043e\u0440\u0438\u0442\u044c");

    ImGui::BulletText(u8"Ctrl +/-  \u2014  \u043c\u0430\u0441\u0448\u0442\u0430\u0431 \u0438\u043d\u0442\u0435\u0440\u0444\u0435\u0439\u0441\u0430");



    ImGui::EndChild();

    ImGui::End();

}



// ═══════════════════════════════════════════════════════════════

//  MENU BAR

// ═══════════════════════════════════════════════════════════════

void UI::drawMenuBar(BendingProgram& prog, UICallbacks& cb) {

    bool machineRunLocked = (machinePtr_ && machinePtr_->isProgramRunning());



    // Push a larger font for the main menu bar if available

    ImFont* menuFont = ImGui::GetIO().Fonts->Fonts[0]; 

    if (ImGui::GetIO().Fonts->Fonts.Size > 1) {

        // Assume font 1 is scaled up

        menuFont = ImGui::GetIO().Fonts->Fonts[1];

    }

    ImGui::PushFont(menuFont);

    

    if (ImGui::BeginMainMenuBar()) {



        // ── Файл ──

        if (ImGui::BeginMenu(u8"\u0424\u0430\u0439\u043b")) {

            if (ImGui::MenuItem(u8"\u0421\u043e\u0437\u0434\u0430\u0442\u044c \u043f\u0440\u043e\u0435\u043a\u0442", "Ctrl+N", false, !machineRunLocked)) {

                pendingNewProject_ = true;

            }

            if (ImGui::MenuItem(u8"\u041e\u0442\u043a\u0440\u044b\u0442\u044c \u043f\u0440\u043e\u0435\u043a\u0442...", nullptr, false, !machineRunLocked)) {

                auto p = openFileDialog("TubeBender Project\0*.tbproj\0JSON\0*.json\0All\0*.*\0", "tbproj");

                if (!p.empty()) openProjectFromFile(p, prog);

            }

            if (ImGui::MenuItem(u8"\u0421\u043e\u0445\u0440\u0430\u043d\u0438\u0442\u044c \u043f\u0440\u043e\u0435\u043a\u0442", "Ctrl+S", false, !machineRunLocked)) {

                saveProjectToFile(prog);

            }

            if (ImGui::MenuItem(u8"\u0421\u043e\u0445\u0440\u0430\u043d\u0438\u0442\u044c \u043a\u0430\u043a...", nullptr, false, !machineRunLocked)) {

                auto p = saveFileDialog("TubeBender Project\0*.tbproj\0", "tbproj");

                if (!p.empty()) saveProjectToFileAs(p, prog);

            }

            ImGui::Separator();

            if (ImGui::MenuItem(u8"\u041d\u0430\u0441\u0442\u0440\u043e\u0439\u043a\u0438...")) {

                showSettings_ = true;

            }

            ImGui::Separator();

            if (ImGui::MenuItem(u8"\u0412\u044b\u0445\u043e\u0434")) {}

            ImGui::EndMenu();

        }



        // ── Правка ──

        if (ImGui::BeginMenu(u8"\u041f\u0440\u0430\u0432\u043a\u0430")) {

            if (ImGui::MenuItem(u8"\u041e\u0442\u043c\u0435\u043d\u0438\u0442\u044c", "Ctrl+Z", false, canUndo() && !machineRunLocked)) {

                undo(prog);

            }

            if (ImGui::MenuItem(u8"\u041f\u043e\u0432\u0442\u043e\u0440\u0438\u0442\u044c", "Ctrl+Y", false, canRedo() && !machineRunLocked)) {

                redo(prog);

            }

            ImGui::EndMenu();

        }



        // ── Труба ──

        if (ImGui::BeginMenu(u8"\u0422\u0440\u0443\u0431\u0430")) {

            if (ImGui::MenuItem(u8"\u0418\u043c\u043f\u043e\u0440\u0442 STP...", nullptr, false, !machineRunLocked)) {

                auto p = openFileDialog("STEP Files\0*.stp;*.step\0All\0*.*\0", "stp");

                if (!p.empty() && cb.onImportSTP) cb.onImportSTP(p);

            }

            if (ImGui::MenuItem(u8"\u042d\u043a\u0441\u043f\u043e\u0440\u0442 STP...")) {

                auto p = saveFileDialog("STEP Files\0*.stp;*.step\0", "stp");

                if (!p.empty() && cb.onExportSTP) cb.onExportSTP(p);

            }

            ImGui::Separator();

            if (ImGui::MenuItem(u8"\u042d\u043a\u0441\u043f\u043e\u0440\u0442 IGES \u0440\u0430\u0437\u0432\u0451\u0440\u0442\u043a\u0438...")) {

                auto p = saveFileDialog("IGES Files\0*.igs;*.iges\0", "igs");

                if (!p.empty() && cb.onExportIGES) cb.onExportIGES(p);

            }

            ImGui::EndMenu();

        }



        if (ImGui::BeginMenu(u8"\u041f\u0440\u043e\u0433\u0440\u0430\u043c\u043c\u0430")) {

            if (ImGui::MenuItem(u8"\u0417\u0430\u0433\u0440\u0443\u0437\u0438\u0442\u044c \u043f\u0440\u043e\u0433\u0440\u0430\u043c\u043c\u0443...", nullptr, false, !machineRunLocked)) {

                auto p = openFileDialog("JSON Files\0*.json\0All\0*.*\0", "json");

                if (!p.empty() && cb.onLoadJSON) cb.onLoadJSON(p);

            }

            if (ImGui::MenuItem(u8"\u0421\u043e\u0445\u0440\u0430\u043d\u0438\u0442\u044c \u043f\u0440\u043e\u0433\u0440\u0430\u043c\u043c\u0443...", nullptr, false, !machineRunLocked)) {

                auto p = saveFileDialog("JSON Files\0*.json\0", "json");

                if (!p.empty() && cb.onSaveJSON) cb.onSaveJSON(p);

            }

            ImGui::Separator();

            if (ImGui::MenuItem(u8"\u041f\u0440\u043e\u0432\u0435\u0440\u043a\u0430 \u0441\u0442\u043e\u043b\u043a\u043d\u043e\u0432\u0435\u043d\u0438\u0439")) {

                if (cb.onCheckCollisions) cb.onCheckCollisions();

            }

            ImGui::EndMenu();

        }



        if (ImGui::BeginMenu(u8"\u0412\u0438\u0434")) {

            if (ImGui::MenuItem(u8"\u0421\u0442\u0430\u043d\u0434\u0430\u0440\u0442\u043d\u044b\u0439", nullptr, !touchMode)) {

                touchMode = false;

                uiScale = 1.0f;

                needFontRebuild = true;

            }

            if (ImGui::MenuItem(u8"\u0422\u0430\u0447\u0441\u043a\u0440\u0438\u043d", nullptr, touchMode)) {

                touchMode = true;

                uiScale = 1.5f;

                needFontRebuild = true;

            }

            ImGui::Separator();

            ImGui::Text(u8"\u041c\u0430\u0441\u0448\u0442\u0430\u0431: %.0f%%", uiScale * 100.0f);

            if (ImGui::MenuItem(u8"\u0423\u0432\u0435\u043b\u0438\u0447\u0438\u0442\u044c  (Ctrl +)")) {

                uiScale = std::min(uiScale + 0.1f, 3.0f);

                needFontRebuild = true;

            }

            if (ImGui::MenuItem(u8"\u0423\u043c\u0435\u043d\u044c\u0448\u0438\u0442\u044c  (Ctrl -)")) {

                uiScale = std::max(uiScale - 0.1f, 0.5f);

                needFontRebuild = true;

            }

            if (ImGui::MenuItem(u8"\u0421\u0431\u0440\u043e\u0441\u0438\u0442\u044c (100%)")) {

                uiScale = 1.0f;

                touchMode = false;

                needFontRebuild = true;

            }

            ImGui::EndMenu();

        }



        if (ImGui::BeginMenu(u8"\u041e\u043a\u043d\u0430")) {

            ImGui::MenuItem(u8"\u0421\u0446\u0435\u043d\u0430",      nullptr, &showSceneWin);

            ImGui::MenuItem(u8"\u041f\u0440\u043e\u0433\u0440\u0430\u043c\u043c\u0430",  nullptr, &showProgramWin);

            ImGui::MenuItem(u8"\u041e\u0441\u043d\u0430\u0441\u0442\u043a\u0430",   nullptr, &showOsnastkaWin);

            ImGui::MenuItem(u8"\u0423\u043f\u0440\u0430\u0432\u043b\u0435\u043d\u0438\u0435", nullptr, &showControlWin);

            ImGui::MenuItem(u8"\u041f\u043e\u0434\u043a\u043b\u044e\u0447\u0435\u043d\u0438\u0435", nullptr, &showConnectionWin);

            ImGui::MenuItem(u8"\u041e\u0442\u043b\u0430\u0434\u043a\u0430",    nullptr, &showDebugWin);

            ImGui::Separator();

            ImGui::MenuItem(u8"\u0421\u043f\u0440\u0430\u0432\u043a\u0430",   nullptr, &showSpravka_);

            ImGui::EndMenu();

        }        // ==== Custom Window Controls (Win10 Style) ====
        float windowWidth = ImGui::GetWindowSize().x;
        // Render buttons at the right
        float scaledBtnW = 54.0f * ImGui::GetIO().FontGlobalScale;
        ImGui::SameLine(windowWidth - (scaledBtnW * 3.0f) - ImGui::GetStyle().WindowPadding.x);
        
        GLFWwindow* w = glfwGetCurrentContext();
        if (w) {
            ImVec2 p = ImVec2(ImGui::GetCursorScreenPos().x, ImGui::GetWindowPos().y);
            float btnW = scaledBtnW;
            float btnH = ImGui::GetWindowHeight();
            ImDrawList* drawList = ImGui::GetWindowDrawList();
            ImU32 colNorm = IM_COL32(200, 200, 200, 255);
            ImU32 colHov  = IM_COL32(255, 255, 255, 255);
            float th = 1.0f; // clean crisp line
            
            // Minimize
            ImGui::InvisibleButton("##min", ImVec2(btnW, btnH));
            ImU32 cMin = colNorm;
            if (ImGui::IsItemHovered()) { drawList->AddRectFilled(p, ImVec2(p.x + btnW, p.y + btnH), IM_COL32(255, 255, 255, 25)); cMin = colHov; }
            if (ImGui::IsItemClicked()) glfwIconifyWindow(w);
            drawList->AddLine(ImVec2(p.x + btnW/2 - 5, p.y + btnH/2 + 2), ImVec2(p.x + btnW/2 + 5, p.y + btnH/2 + 2), cMin, th);
            
            ImGui::SameLine(0, 0);
            p = ImGui::GetCursorScreenPos();
            
            // Maximize/Restore
            ImGui::InvisibleButton("##max", ImVec2(btnW, btnH));
            ImU32 cMax = colNorm;
            if (ImGui::IsItemHovered()) { drawList->AddRectFilled(p, ImVec2(p.x + btnW, p.y + btnH), IM_COL32(255, 255, 255, 25)); cMax = colHov; }
            bool isMax = glfwGetWindowAttrib(w, GLFW_MAXIMIZED);
            if (ImGui::IsItemClicked()) {
                if (isMax) glfwRestoreWindow(w);
                else glfwMaximizeWindow(w);
            }
            if (isMax) {
                drawList->AddRect(ImVec2(p.x + btnW/2 - 4, p.y + btnH/2 - 1), ImVec2(p.x + btnW/2 + 3, p.y + btnH/2 + 5), cMax, 0, 0, th);
                drawList->AddLine(ImVec2(p.x + btnW/2 - 2, p.y + btnH/2 - 1), ImVec2(p.x + btnW/2 - 2, p.y + btnH/2 - 4), cMax, th);
                drawList->AddLine(ImVec2(p.x + btnW/2 - 2, p.y + btnH/2 - 4), ImVec2(p.x + btnW/2 + 5, p.y + btnH/2 - 4), cMax, th);
                drawList->AddLine(ImVec2(p.x + btnW/2 + 5, p.y + btnH/2 - 4), ImVec2(p.x + btnW/2 + 5, p.y + btnH/2 + 3), cMax, th);
                drawList->AddLine(ImVec2(p.x + btnW/2 + 3, p.y + btnH/2 + 3), ImVec2(p.x + btnW/2 + 5, p.y + btnH/2 + 3), cMax, th);
            } else {
                drawList->AddRect(ImVec2(p.x + btnW/2 - 5, p.y + btnH/2 - 4), ImVec2(p.x + btnW/2 + 5, p.y + btnH/2 + 5), cMax, 0, 0, th);
            }
            
            ImGui::SameLine(0, 0);
            p = ImGui::GetCursorScreenPos();
            
            // Close
            ImGui::InvisibleButton("##close", ImVec2(btnW, btnH));
            ImU32 cClo = colNorm;
            if (ImGui::IsItemHovered()) { drawList->AddRectFilled(p, ImVec2(p.x + btnW, p.y + btnH), IM_COL32(232, 17, 35, 200)); cClo = IM_COL32(255, 255, 255, 255); }
            if (ImGui::IsItemClicked()) glfwSetWindowShouldClose(w, GLFW_TRUE);
            drawList->AddLine(ImVec2(p.x + btnW/2 - 4, p.y + btnH/2 - 4), ImVec2(p.x + btnW/2 + 5, p.y + btnH/2 + 5), cClo, th);
            drawList->AddLine(ImVec2(p.x + btnW/2 - 4, p.y + btnH/2 + 5), ImVec2(p.x + btnW/2 + 5, p.y + btnH/2 - 4), cClo, th);

            // Window Drag Logic
            if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
#ifdef _WIN32
                HWND hwnd = GetActiveWindow();
                if (hwnd) {
                    ReleaseCapture();
                    // Fix double-click issue after dragging: Windows consumes the WM_LBUTTONUP,
                    // so ImGui still thinks left click is down. We manually clear it here.
                    ImGui::GetIO().MouseDown[0] = false;
                    SendMessage(hwnd, WM_NCLBUTTONDOWN, 2, 0); // HTCAPTION = 2
                }
#else
                // fallback for other OS
#endif
            }

        }



        ImGui::EndMainMenuBar();

    } 

    ImGui::PopFont();



    // Keyboard shortcuts

    ImGuiIO& io = ImGui::GetIO();

    if (io.KeyCtrl) {

        // Undo / Redo

        if (!machineRunLocked && ImGui::IsKeyPressed(ImGuiKey_Z)) {

            undo(prog);

        }

        if (!machineRunLocked && ImGui::IsKeyPressed(ImGuiKey_Y)) {

            redo(prog);

        }

        // New project

        if (!machineRunLocked && ImGui::IsKeyPressed(ImGuiKey_N)) {

            pendingNewProject_ = true;

        }

        // Save project

        if (!machineRunLocked && ImGui::IsKeyPressed(ImGuiKey_S)) {

            saveProjectToFile(prog);

        }

        // Scale

        if (ImGui::IsKeyPressed(ImGuiKey_Equal) || ImGui::IsKeyPressed(ImGuiKey_KeypadAdd)) {

            uiScale = std::min(uiScale + 0.1f, 3.0f);

            needFontRebuild = true;

        }

        if (ImGui::IsKeyPressed(ImGuiKey_Minus) || ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract)) {

            uiScale = std::max(uiScale - 0.1f, 0.5f);

            needFontRebuild = true;

        }

        if (ImGui::IsKeyPressed(ImGuiKey_0)) {

            uiScale = 1.0f;

            touchMode = false;

            needFontRebuild = true;

        }

        if (io.MouseWheel != 0) {

            uiScale = std::clamp(uiScale + io.MouseWheel * 0.1f, 0.5f, 3.0f);

            needFontRebuild = true;

        }

    }

}



// ═══════════════════════════════════════════════════════════════

//  Win32 FILE DIALOGS

// ═══════════════════════════════════════════════════════════════

std::string UI::openFileDialog(const char* filter, const char* defaultExt) {

#ifdef _WIN32

    wchar_t wFilter[512] = {};

    {

        int src = 0, dst = 0;

        while (src < 512 && dst < 510) {

            if (filter[src] == '\0') {

                wFilter[dst++] = L'\0';

                if (filter[src + 1] == '\0') { wFilter[dst++] = L'\0'; break; }

                src++;

            } else {

                int n = MultiByteToWideChar(CP_ACP, 0, &filter[src], -1, &wFilter[dst], 510 - dst);

                int sl = (int)strlen(&filter[src]);

                src += sl;

                dst += n - 1;

            }

        }

    }

    wchar_t wDefExt[32] = {};

    if (defaultExt) MultiByteToWideChar(CP_ACP, 0, defaultExt, -1, wDefExt, 32);



    wchar_t wpath[MAX_PATH] = {};

    OPENFILENAMEW ofn = {};

    ofn.lStructSize = sizeof(ofn);

    ofn.lpstrFilter = wFilter;

    ofn.lpstrFile = wpath;

    ofn.nMaxFile = MAX_PATH;

    ofn.lpstrDefExt = wDefExt;

    ofn.Flags = OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;

    if (GetOpenFileNameW(&ofn)) {

        int len = WideCharToMultiByte(CP_UTF8, 0, wpath, -1, nullptr, 0, nullptr, nullptr);

        std::string result(len - 1, '\0');

        WideCharToMultiByte(CP_UTF8, 0, wpath, -1, result.data(), len, nullptr, nullptr);

        return result;

    }

#endif

    return {};

}



std::string UI::saveFileDialog(const char* filter, const char* defaultExt) {

#ifdef _WIN32

    wchar_t wFilter[512] = {};

    {

        int src = 0, dst = 0;

        while (src < 512 && dst < 510) {

            if (filter[src] == '\0') {

                wFilter[dst++] = L'\0';

                if (filter[src + 1] == '\0') { wFilter[dst++] = L'\0'; break; }

                src++;

            } else {

                int n = MultiByteToWideChar(CP_ACP, 0, &filter[src], -1, &wFilter[dst], 510 - dst);

                int sl = (int)strlen(&filter[src]);

                src += sl;

                dst += n - 1;

            }

        }

    }

    wchar_t wDefExt[32] = {};

    if (defaultExt) MultiByteToWideChar(CP_ACP, 0, defaultExt, -1, wDefExt, 32);



    wchar_t wpath[MAX_PATH] = {};

    OPENFILENAMEW ofn = {};

    ofn.lStructSize = sizeof(ofn);

    ofn.lpstrFilter = wFilter;

    ofn.lpstrFile = wpath;

    ofn.nMaxFile = MAX_PATH;

    ofn.lpstrDefExt = wDefExt;

    ofn.Flags = OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;

    if (GetSaveFileNameW(&ofn)) {

        int len = WideCharToMultiByte(CP_UTF8, 0, wpath, -1, nullptr, 0, nullptr, nullptr);

        std::string result(len - 1, '\0');

        WideCharToMultiByte(CP_UTF8, 0, wpath, -1, result.data(), len, nullptr, nullptr);

        return result;

    }

#endif

    return {};

}



// ═══════════════════════════════════════════════════════════════

//  CUT EDITOR WINDOW — detailed G-code style cut plane editor

// ═══════════════════════════════════════════════════════════════

void UI::drawCutEditorWindow(BendingProgram& prog) {

    // Validate step index

    if (cutEditorStep_ < 0 || cutEditorStep_ >= (int)prog.steps.size() ||

        !prog.steps[cutEditorStep_].isCut) {

        showCutEditor_ = false;

        return;

    }

    auto& step = prog.steps[cutEditorStep_];

    auto& cd   = step.cutDef;



    // ── Center on first appearance ──

    ImGuiViewport* vp = ImGui::GetMainViewport();

    ImVec2 winSize(520, 420);

    ImGui::SetNextWindowSize(winSize, ImGuiCond_FirstUseEver);

    ImGui::SetNextWindowPos(

        ImVec2(vp->GetCenter().x - winSize.x * 0.5f,

               vp->GetCenter().y - winSize.y * 0.5f),

        ImGuiCond_FirstUseEver);



    ImGuiWindowFlags flags =

        ImGuiWindowFlags_NoDocking |

        ImGuiWindowFlags_NoCollapse;



    char title[128];

    snprintf(title, sizeof(title), u8"\u0420\u0435\u0434\u0430\u043a\u0442\u043e\u0440 \u0440\u0435\u0437\u0430  [\u0428\u0430\u0433 %d]###CutEditor", cutEditorStep_ + 1);

    ImGui::Begin(title, &showCutEditor_, flags);



    bool machineRunLocked = (machinePtr_ && machinePtr_->isProgramRunning());

    if (machineRunLocked) {

        ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.2f, 1.0f),

            u8"Редактирование реза заблокировано во время выполнения программы на станке");

        ImGui::Separator();

    }



    ImGui::BeginDisabled(machineRunLocked);



    float fieldW = 200.0f;



    // ── Cut type ──

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.878f, 0.424f, 0.467f, 1.0f));

    ImGui::Text(u8"G-\u041a\u043e\u0434 \u043f\u0430\u0440\u0430\u043c\u0435\u0442\u0440\u044b \u0440\u0435\u0437\u0430");

    ImGui::PopStyleColor();

    ImGui::Separator();

    ImGui::Spacing();



    // Cut type dropdown

    static const char* cutTypeLabels[] = {

        u8"\u041f\u0440\u044f\u043c\u043e\u0439 (G00)",

        u8"\u041a\u043e\u0441\u043e\u0439 (G01)",

        u8"\u0421\u0435\u0434\u043b\u043e\u0432\u043e\u0439 (G02)"

    };

    int ctIdx = cutTypeToIndex(cd.type());

    ImGui::Text(u8"\u0422\u0438\u043f \u0440\u0435\u0437\u0430:");

    ImGui::SameLine();

    ImGui::PushItemWidth(fieldW);

    if (ImGui::Combo("##cuttype", &ctIdx, cutTypeLabels, 3)) {

        if (cd.planes.empty()) {

            CutPlane cp; cp.nx = 0; cp.ny = -1; cp.nz = 0; cp.offsetAlongAxis = 0;

            cd.planes.push_back(cp);

        }

        CutPlane& p0 = cd.planes[0];

        if (ctIdx == 0) { // Прямой

            p0.nx = 0; p0.ny = -1; p0.nz = 0;

            cd.planes.resize(1);

        } else if (ctIdx == 1) { // Косой

            if (p0.isStraight()) setObliqueAngle(p0, 30.0);

            cd.planes.resize(1);

        } else { // Седловой

            if (p0.isStraight()) setObliqueAngle(p0, 30.0);

            if (cd.planes.size() < 2) {

                CutPlane p1;

                setObliqueAngle(p1, -obliqueAngleDeg(p0));

                p1.offsetAlongAxis = p0.offsetAlongAxis;

                cd.planes.push_back(p1);

            }

        }

        needsRebuild = true;

    }

    ImGui::PopItemWidth();



    ImGui::Text(u8"\u041a\u043e\u043b-\u0432\u043e \u043f\u043b\u043e\u0441\u043a\u043e\u0441\u0442\u0435\u0439: %d", (int)cd.planes.size());



    ImGui::Spacing();

    ImGui::Separator();

    ImGui::Spacing();



    // ── Per-plane parameters ──

    for (int p = 0; p < (int)cd.planes.size(); p++) {

        auto& plane = cd.planes[p];

        ImGui::PushID(8000 + p);



        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.627f, 0.769f, 0.910f, 1.0f));

        ImGui::Text(u8"\u041f\u043b\u043e\u0441\u043a\u043e\u0441\u0442\u044c %d:", p + 1);

        ImGui::PopStyleColor();



        // Normal vector (nx, ny, nz)

        ImGui::Text("  N:");

        ImGui::SameLine();

        ImGui::PushItemWidth(80);

        ImGui::Text("X="); ImGui::SameLine();

        if (ImGui::InputDouble("##nx", &plane.nx, 0.0, 0.0, "%.4f", ImGuiInputTextFlags_EnterReturnsTrue))

            needsRebuild = true;

        ImGui::SameLine();

        ImGui::Text("Y="); ImGui::SameLine();

        if (ImGui::InputDouble("##ny", &plane.ny, 0.0, 0.0, "%.4f", ImGuiInputTextFlags_EnterReturnsTrue))

            needsRebuild = true;

        ImGui::SameLine();

        ImGui::Text("Z="); ImGui::SameLine();

        if (ImGui::InputDouble("##nz", &plane.nz, 0.0, 0.0, "%.4f", ImGuiInputTextFlags_EnterReturnsTrue))

            needsRebuild = true;

        ImGui::PopItemWidth();



        // Offset along axis

        ImGui::Text(u8"  \u0421\u043c\u0435\u0449\u0435\u043d\u0438\u0435 \u043f\u043e \u043e\u0441\u0438 (\u043c\u043c):");

        ImGui::SameLine();

        ImGui::PushItemWidth(fieldW);

        if (ImGui::InputDouble("##offset", &plane.offsetAlongAxis, 0.0, 0.0, "%.3f",

                ImGuiInputTextFlags_EnterReturnsTrue))

            needsRebuild = true;

        ImGui::PopItemWidth();



        // Oblique angle (convenience, computed from normal)

        if (!plane.isStraight()) {

            double angleDeg = obliqueAngleDeg(plane);

            ImGui::Text(u8"  \u0423\u0433\u043e\u043b \u043d\u0430\u043a\u043b\u043e\u043d\u0430 (\u00b0):");

            ImGui::SameLine();

            ImGui::PushItemWidth(fieldW);

            if (ImGui::InputDouble("##angle", &angleDeg, 0.0, 0.0, "%.2f",

                    ImGuiInputTextFlags_EnterReturnsTrue)) {

                setObliqueAngle(cd.planes[p], angleDeg);

                needsRebuild = true;

            }

            ImGui::PopItemWidth();

        }



        // isStraight indicator

        ImGui::Text(u8"  \u0422\u0438\u043f: %s", plane.isStraight() ? u8"\u041f\u0440\u044f\u043c\u043e\u0439" : u8"\u041d\u0430\u043a\u043b\u043e\u043d\u043d\u044b\u0439");



        ImGui::Spacing();

        ImGui::PopID();

    }



    // ── Add/remove plane buttons ──

    ImGui::Separator();

    ImGui::Spacing();

    if (cd.planes.size() < 10) {

        if (ImGui::Button(u8"\u0414\u043e\u0431\u0430\u0432\u0438\u0442\u044c \u043f\u043b\u043e\u0441\u043a\u043e\u0441\u0442\u044c")) {

            CutPlane cp; cp.nx = 0; cp.ny = -1; cp.nz = 0; cp.offsetAlongAxis = 0;

            if (!cd.planes.empty()) {

                double a = -obliqueAngleDeg(cd.planes[0]);

                setObliqueAngle(cp, a);

            }

            cd.planes.push_back(cp);

            needsRebuild = true;

        }

    }

    if (cd.planes.size() > 1) {

        ImGui::SameLine();

        if (ImGui::Button(u8"\u0423\u0431\u0440\u0430\u0442\u044c \u043f\u043e\u0441\u043b\u0435\u0434\u043d\u044e\u044e")) {

            cd.planes.pop_back();

            needsRebuild = true;

        }

    }



    ImGui::EndDisabled();



    // ── Bottom: close button ──

    ImGui::Spacing();

    ImGui::Separator();

    float btnW = 180;

    ImGui::SetCursorPosX(ImGui::GetContentRegionAvail().x - btnW);

    if (ImGui::Button(u8"\u0417\u0430\u043a\u0440\u044b\u0442\u044c", ImVec2(btnW, 0))) {

        showCutEditor_ = false;

    }



    ImGui::End();

}



// ═══════════════════════════════════════════════════════════════

//  COLORED BUTTON helper — supports height for square buttons

// ═══════════════════════════════════════════════════════════════

bool UI::coloredButton(const char* label, float r, float g, float b, float w, float h) {

    ImGui::PushStyleColor(ImGuiCol_Button,

        ImVec4(r, g, b, 1.0f));

    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,

        ImVec4(std::min(r*1.2f,1.f), std::min(g*1.2f,1.f), std::min(b*1.2f,1.f), 1.0f));

    ImGui::PushStyleColor(ImGuiCol_ButtonActive,

        ImVec4(r*0.8f, g*0.8f, b*0.8f, 1.0f));

    bool clicked;

    ImVec2 size(w > 0 ? w : 0, h > 0 ? h : 0);

    clicked = ImGui::Button(label, size);

    ImGui::PopStyleColor(3);

    return clicked;

}



// =================================================================

//  PROJECT TABS — horizontal tab bar below menu bar

// =================================================================

void UI::drawProjectTabs(BendingProgram& prog) {

    if (projects_.empty()) return;

    bool machineRunLocked = (machinePtr_ && machinePtr_->isProgramRunning());



    ImGuiViewport* vp = ImGui::GetMainViewport();

    float barH = ImGui::GetFrameHeight() + 6;



    ImGui::SetNextWindowPos(vp->WorkPos);

    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, barH));



    ImGuiWindowFlags flags =

        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |

        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |

        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking |

        ImGuiWindowFlags_NoCollapse;



    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4, 2));

    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

    ImGui::Begin("##ProjectBar", nullptr, flags);

    ImGui::PopStyleVar(2);



    if (ImGui::BeginTabBar("##ProjectTabs",

            ImGuiTabBarFlags_Reorderable | ImGuiTabBarFlags_FittingPolicyScroll |

            ImGuiTabBarFlags_AutoSelectNewTabs))

    {

        if (machineRunLocked) ImGui::BeginDisabled();

        for (int i = 0; i < (int)projects_.size(); i++) {

            ImGui::PushID(9000 + i);

            bool open = true;

            if (ImGui::BeginTabItem(projects_[i].name.c_str(),

                    ((int)projects_.size() > 1) ? &open : nullptr))

            {

                if (i != activeProject_) {

                    pendingProjectSwitch_ = i;

                }

                ImGui::EndTabItem();

            }

            if (!open) {

                pendingCloseConfirm_ = i;

            }

            ImGui::PopID();

        }

        if (ImGui::TabItemButton("+", ImGuiTabItemFlags_Trailing | ImGuiTabItemFlags_NoTooltip)) {

            pendingNewProject_ = true;

        }

        if (machineRunLocked) ImGui::EndDisabled();

        ImGui::EndTabBar();

    }



    if (machineRunLocked) {

        ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.2f, 1.0f),

            u8"Переключение/редактирование проектов заблокировано во время выполнения программы на станке");

    }



    ImGui::End();

}



// =================================================================

//  3D AXIS GIZMO — overlay in top-right of scene window

// =================================================================

void UI::drawAxisGizmo() {

    ImDrawList* dl = ImGui::GetWindowDrawList();

    ImVec2 wpos = ImGui::GetWindowPos();

    ImVec2 wsz = ImGui::GetWindowSize();



    float gizmoR = 35.0f;

    float margin = 12.0f;

    ImVec2 center(wpos.x + wsz.x - gizmoR - margin,

                  wpos.y + ImGui::GetFrameHeight() + gizmoR + margin);



    // Camera basis from yaw/pitch (same math as renderer)

    float yawRad = cameraYaw * (float)M_PI / 180.0f;

    float pitchRad = cameraPitch * (float)M_PI / 180.0f;



    // Project world axis to screen: screen X = dot(axis, Side), screen Y = -dot(axis, Up)

    auto project = [&](float wx, float wy, float wz) -> ImVec2 {

        float sx = -sinf(yawRad) * wx + cosf(yawRad) * wy;

        float sy = -((-cosf(yawRad) * sinf(pitchRad)) * wx +

                     (-sinf(yawRad) * sinf(pitchRad)) * wy +

                     cosf(pitchRad) * wz);

        return ImVec2(center.x + sx * gizmoR, center.y + sy * gizmoR);

    };



    // Sort axes by depth for correct draw order (back to front)

    float fwd_x = cosf(pitchRad) * cosf(yawRad);

    float fwd_y = cosf(pitchRad) * sinf(yawRad);

    float fwd_z = sinf(pitchRad);

    struct AxisInfo { int id; float depth; ImVec2 end; ImU32 col; const char* lbl; };

    AxisInfo axes[3] = {

        { 0, fwd_x, project(1,0,0), IM_COL32(220, 50, 50, 255), "X" },

        { 1, fwd_y, project(0,1,0), IM_COL32(50, 200, 50, 255), "Y" },

        { 2, fwd_z, project(0,0,1), IM_COL32(80, 130, 255, 255), "Z" },

    };

    // Sort: most negative depth (farthest) drawn first

    if (axes[0].depth > axes[1].depth) std::swap(axes[0], axes[1]);

    if (axes[1].depth > axes[2].depth) std::swap(axes[1], axes[2]);

    if (axes[0].depth > axes[1].depth) std::swap(axes[0], axes[1]);



    for (auto& a : axes) {

        dl->AddLine(center, a.end, a.col, 2.2f);

        // Small arrowhead circle at tip

        dl->AddCircleFilled(a.end, 3.5f, a.col);

        dl->AddText(ImVec2(a.end.x + 4, a.end.y - 6), a.col, a.lbl);

    }

}



// =================================================================

//  UNDO / REDO

// =================================================================

UI::UndoState UI::captureState(const BendingProgram& prog) const {

    UndoState s;

    s.program = prog;

    s.laserOffsetY = laserOffsetY;

    s.laserGapZ = laserGapZ;

    s.whipLength = whipLength;

    s.partCount = partCount;

    s.springBackCoeff = springBackCoeff;

    s.steelGradeIdx = steelGradeIdx;

    s.selectedStep = selectedStep;

    s.presetName = presetName;

    return s;

}



void UI::restoreState(const UndoState& state, BendingProgram& prog) {

    prog = state.program;

    // Tooling params are global — don't restore from undo

    selectedStep = state.selectedStep;

    needsRebuild = true;

}



void UI::pushUndo(const BendingProgram& prog) {

    undoStack_.push_back(captureState(prog));

    if ((int)undoStack_.size() > MAX_UNDO)

        undoStack_.erase(undoStack_.begin());

    redoStack_.clear();

}



void UI::undo(BendingProgram& prog) {

    if (undoStack_.empty()) return;

    redoStack_.push_back(captureState(prog));

    restoreState(undoStack_.back(), prog);

    undoStack_.pop_back();

    undoJustApplied_ = true;

    lastState_ = captureState(prog);

    lastStateHash_ = computeStateHash(prog);

}



void UI::redo(BendingProgram& prog) {

    if (redoStack_.empty()) return;

    undoStack_.push_back(captureState(prog));

    restoreState(redoStack_.back(), prog);

    redoStack_.pop_back();

    undoJustApplied_ = true;

    lastState_ = captureState(prog);

    lastStateHash_ = computeStateHash(prog);

}



// =================================================================

//  PROJECT save / load / switch

// =================================================================

void UI::saveActiveProject(BendingProgram& prog) {

    if (activeProject_ < 0 || activeProject_ >= (int)projects_.size()) return;

    auto& p = projects_[activeProject_];

    p.program = prog;

    p.laserOffsetY = laserOffsetY;

    p.laserGapZ = laserGapZ;

    p.whipLength = whipLength;

    p.partCount = partCount;

    p.springBackCoeff = springBackCoeff;

    p.steelGradeIdx = steelGradeIdx;

    p.presetName = presetName;

    p.selectedStep = selectedStep;

}



void UI::loadProject(int idx, BendingProgram& prog) {

    if (idx < 0 || idx >= (int)projects_.size()) return;

    auto& p = projects_[idx];

    prog = p.program;

    // Tooling params are global — don't overwrite from project

    selectedStep = p.selectedStep;

    // Clear undo/redo when switching projects

    undoStack_.clear();

    redoStack_.clear();

    lastStateHash_ = 0;

}



void UI::openProjectFromFile(const std::string& path, BendingProgram& prog) {

    try {

#ifdef _WIN32

        int wlen = MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, nullptr, 0);

        std::wstring wp(wlen, L'\0');

        MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, wp.data(), wlen);

        std::ifstream f(wp);

#else

        std::ifstream f(path);

#endif

        if (!f) return;

        nlohmann::json j;

        f >> j;



        Project p;

        if (j.contains("name")) p.name = j["name"].get<std::string>();

        if (j.contains("program")) p.program = BendingProgram::fromJson(j["program"]);

        if (j.contains("laserOffsetY")) p.laserOffsetY = j["laserOffsetY"].get<double>();

        if (j.contains("laserGapZ")) p.laserGapZ = j["laserGapZ"].get<double>();

        if (j.contains("whipLength")) p.whipLength = j["whipLength"].get<double>();

        if (j.contains("partCount")) p.partCount = j["partCount"].get<int>();

        if (j.contains("springBackCoeff")) p.springBackCoeff = j["springBackCoeff"].get<double>();

        if (j.contains("steelGradeIdx")) p.steelGradeIdx = j["steelGradeIdx"].get<int>();

        if (j.contains("presetName")) p.presetName = j["presetName"].get<std::string>();

        p.filePath = path;



        saveActiveProject(prog);

        projects_.push_back(p);

        activeProject_ = (int)projects_.size() - 1;

        loadProject(activeProject_, prog);

        needsRebuild = true;

    } catch (...) {}

}



void UI::saveProjectToFile(BendingProgram& prog) {

    if (activeProject_ < 0 || activeProject_ >= (int)projects_.size()) return;

    saveActiveProject(prog);

    auto& p = projects_[activeProject_];

    if (p.filePath.empty()) {

        // No file yet — default to projects/ folder

        std::string dir = exeDir_ + "/projects";

        try { fs::create_directories(fs::u8path(dir)); } catch (...) {}

        auto path = saveFileDialog("TubeBender Project\0*.tbproj\0", "tbproj");

        if (path.empty()) return;

        p.filePath = path;

    }

    writeProjectFile(p);

    settingsSavedTimer_ = 3.0f;

    settingsSavedMsg_ = u8"\u041f\u0440\u043e\u0435\u043a\u0442 \u0441\u043e\u0445\u0440\u0430\u043d\u0435\u043d";

}



void UI::saveProjectToFileAs(const std::string& path, BendingProgram& prog) {

    if (activeProject_ < 0 || activeProject_ >= (int)projects_.size()) return;

    saveActiveProject(prog);

    auto& p = projects_[activeProject_];

    p.filePath = path;

    // Update project name from filename

    p.name = fs::u8path(path).stem().u8string();

    writeProjectFile(p);

    settingsSavedTimer_ = 3.0f;

    settingsSavedMsg_ = u8"\u041f\u0440\u043e\u0435\u043a\u0442 \u0441\u043e\u0445\u0440\u0430\u043d\u0435\u043d";

}



void UI::writeProjectFile(const Project& p) {

    nlohmann::json j;

    j["name"] = p.name;

    j["program"] = p.program.toJson();

    j["laserOffsetY"] = p.laserOffsetY;

    j["laserGapZ"] = p.laserGapZ;

    j["whipLength"] = p.whipLength;

    j["partCount"] = p.partCount;

    j["springBackCoeff"] = p.springBackCoeff;

    j["steelGradeIdx"] = p.steelGradeIdx;

    j["presetName"] = p.presetName;



    try {

#ifdef _WIN32

        int wlen = MultiByteToWideChar(CP_UTF8, 0, p.filePath.c_str(), -1, nullptr, 0);

        std::wstring wp(wlen, L'\0');

        MultiByteToWideChar(CP_UTF8, 0, p.filePath.c_str(), -1, wp.data(), wlen);

        std::ofstream f(wp);

#else

        std::ofstream f(p.filePath);

#endif

        if (f) f << j.dump(2);

    } catch (...) {}

}



void UI::syncActiveProject(BendingProgram& prog) {

    saveActiveProject(prog);

}



std::string UI::activeProjectName() const {

    if (activeProject_ < 0 || activeProject_ >= (int)projects_.size()) return {};

    return projects_[activeProject_].name;

}



int UI::activeProjectStepCount() const {

    if (activeProject_ < 0 || activeProject_ >= (int)projects_.size()) return 0;

    return (int)projects_[activeProject_].program.steps.size();

}

