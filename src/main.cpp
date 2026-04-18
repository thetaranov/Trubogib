// ─────────────────────────────────────────────────────────────────────
// main.cpp — Entry point: GLFW window + Dear ImGui + App
// ─────────────────────────────────────────────────────────────────────
#include "app.h"
#include "logger.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <iostream>
#include <clocale>
#include <cmath>
#include <filesystem>

#ifdef _WIN32
#include <windows.h>
#include <dwmapi.h>
#pragma comment(lib, "dwmapi.lib")
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#ifndef DWMWA_USE_IMMERSIVE_DARK_MODE
#define DWMWA_USE_IMMERSIVE_DARK_MODE 20
#endif
#endif

static App* g_app = nullptr;

// ─── GLFW callbacks ─────────────────────────────────────────────────
static void glfwErrorCb(int err, const char* desc) {
    std::cerr << "GLFW error " << err << ": " << desc << "\n";
    Logger::log(std::string("[GLFW] error ") + std::to_string(err) + ": " + (desc ? desc : ""));
}

// NOTE: Mouse button / cursor / scroll callbacks are NOT set manually.
// ImGui_ImplGlfw_InitForOpenGL(window, true) installs its own handlers,
// and our 3D camera input is handled via polling in App::processInput().

static void glfwFramebufferSizeCb(GLFWwindow* w, int width, int height) {
    if (g_app) g_app->onResize(width, height);
}

// ─── Main ───────────────────────────────────────────────────────────
#ifdef _WIN32
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
#else
int main() {
#endif
    std::setlocale(LC_ALL, "");
    std::setlocale(LC_NUMERIC, "C");  // force dot decimal for snprintf/atof (serial protocol + JSON)

#ifdef _WIN32
    {
        wchar_t exeBuf[MAX_PATH];
        GetModuleFileNameW(nullptr, exeBuf, MAX_PATH);
        std::filesystem::path ep(exeBuf);
        Logger::init(ep.parent_path());
        Logger::log("[APP] EXE start");
    }
#endif

    // ── GLFW init ───────────────────────────────────────────────────
    glfwSetErrorCallback(glfwErrorCb);
    if (!glfwInit()) {
        std::cerr << "glfwInit failed\n";
        Logger::log("[APP] glfwInit failed");
        Logger::shutdown();
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);  // borderless for splash
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);     // hidden until centered

    int initW = 1400, initH = 850;
    GLFWwindow* window = glfwCreateWindow(initW, initH,
        "TubeBender v2.2", nullptr, nullptr);
    if (!window) {
        Logger::log("[APP] glfwCreateWindow failed");
        glfwTerminate();
        Logger::shutdown();
        return 1;
    }

    // Center borderless splash window on primary monitor
    {
        GLFWmonitor* primary = glfwGetPrimaryMonitor();
        if (primary) {
            const GLFWvidmode* mode = glfwGetVideoMode(primary);
            if (mode) {
                int mx, my;
                glfwGetMonitorPos(primary, &mx, &my);
                glfwSetWindowPos(window, mx + (mode->width - initW) / 2,
                                         my + (mode->height - initH) / 2);
            }
        }
        glfwShowWindow(window);
#ifdef _WIN32
HWND hwnd = glfwGetWin32Window(window);
if(hwnd){
HICON hIcon = LoadIcon(GetModuleHandle(NULL), MAKEINTRESOURCE(1));
if(hIcon){
SendMessage(hwnd, WM_SETICON, ICON_BIG, (LPARAM)hIcon);
SendMessage(hwnd, WM_SETICON, ICON_SMALL, (LPARAM)hIcon);
}
}
#endif
    }

    // Set Cyrillic window title via Win32 API (reliable UTF-16)
#ifdef _WIN32
    {
        HWND hwnd = glfwGetWin32Window(window);
        SetWindowTextW(hwnd, L"\x0422\x0440\x0443\x0431\x043E\x0433\x0438\x0431 v2.2 \x2014 3D + \x041B\x0430\x0437\x0435\x0440\x043D\x0430\x044F \x0440\x0435\x0437\x043A\x0430");
        
        // Dark mode title bar
        BOOL useDarkMode = TRUE;
        DwmSetWindowAttribute(hwnd, DWMWA_USE_IMMERSIVE_DARK_MODE, &useDarkMode, sizeof(useDarkMode));
    }
#endif

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // vsync

    // ── GLAD ────────────────────────────────────────────────────────
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "gladLoadGL failed\n";
        Logger::log("[APP] gladLoadGL failed");
        Logger::shutdown();
        return 1;
    }

    // ── Dear ImGui init ─────────────────────────────────────────────
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // enable docking

    // Disable ImGui's built-in ini load/save (it uses fopen which can't
    // handle UTF-8 Cyrillic paths on MSVC). We handle it manually.
    io.IniFilename = nullptr;

    // Compute absolute wide path for imgui.ini next to exe
    static std::wstring iniWidePath;
    {
#ifdef _WIN32
        wchar_t exeBuf[MAX_PATH];
        GetModuleFileNameW(nullptr, exeBuf, MAX_PATH);
        std::filesystem::path ep(exeBuf);
        iniWidePath = (ep.parent_path() / L"imgui.ini").wstring();
#endif
    }

    // Manually load imgui.ini using _wfopen
#ifdef _WIN32
    {
        FILE* f = _wfopen(iniWidePath.c_str(), L"rb");
        if (f) {
            fseek(f, 0, SEEK_END);
            long sz = ftell(f);
            fseek(f, 0, SEEK_SET);
            if (sz > 0) {
                std::string data(sz, '\0');
                fread(data.data(), 1, sz, f);
                ImGui::LoadIniSettingsFromMemory(data.c_str(), data.size());
            }
            fclose(f);
        }
    }
#endif

    // Load Cyrillic-capable font
    // Try to load from Windows fonts, fallback to default
    ImFont* font = nullptr;
    const char* fontPaths[] = {
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/tahoma.ttf",
    };
    const char* activeFontPath = nullptr;
    for (auto fp : fontPaths) {
        FILE* test = fopen(fp, "rb");
        if (test) {
            fclose(test);
            activeFontPath = fp;
            font = io.Fonts->AddFontFromFileTTF(fp, 18.0f, nullptr, io.Fonts->GetGlyphRangesCyrillic());
            io.Fonts->AddFontFromFileTTF(fp, 22.0f, nullptr, io.Fonts->GetGlyphRangesCyrillic());
            break;
        }
    }
    if (!font) {
        io.Fonts->AddFontDefault();
    }

    ImGui::StyleColorsDark();

    // Match v1.1 dark theme colors
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 0.0f;
    style.FrameRounding = 2.0f;
    style.GrabRounding = 2.0f;
    style.TabRounding = 2.0f;
    style.ScrollbarRounding = 2.0f;
    style.WindowBorderSize = 1.0f;
    style.FramePadding = ImVec2(8, 4);
    style.ItemSpacing = ImVec2(8, 5);

    ImVec4* c = style.Colors;
    c[ImGuiCol_WindowBg]           = ImVec4(0.105f, 0.105f, 0.110f, 1.0f); // #1B1B1C — darker
    c[ImGuiCol_ChildBg]            = ImVec4(0.105f, 0.105f, 0.110f, 1.0f);
    c[ImGuiCol_PopupBg]            = ImVec4(0.145f, 0.145f, 0.155f, 0.94f);
    c[ImGuiCol_Border]             = ImVec4(0.30f, 0.30f, 0.30f, 1.0f);
    c[ImGuiCol_FrameBg]            = ImVec4(0.20f, 0.20f, 0.215f, 1.0f); // lighter frame — stands out
    c[ImGuiCol_FrameBgHovered]     = ImVec4(0.28f, 0.28f, 0.30f, 1.0f);
    c[ImGuiCol_FrameBgActive]      = ImVec4(0.24f, 0.24f, 0.26f, 1.0f);
    c[ImGuiCol_TitleBg]            = ImVec4(0.085f, 0.085f, 0.085f, 1.0f); // #161616 — darkest
    c[ImGuiCol_TitleBgActive]      = ImVec4(0.085f, 0.085f, 0.085f, 1.0f);
    c[ImGuiCol_MenuBarBg]          = ImVec4(0.145f, 0.145f, 0.155f, 1.0f);
    c[ImGuiCol_Header]             = ImVec4(0.22f, 0.22f, 0.235f, 1.0f);
    c[ImGuiCol_HeaderHovered]      = ImVec4(0.055f, 0.392f, 0.612f, 0.8f); // #0E639C
    c[ImGuiCol_HeaderActive]       = ImVec4(0.055f, 0.392f, 0.612f, 1.0f);
    c[ImGuiCol_Tab]                = ImVec4(0.20f, 0.20f, 0.216f, 1.0f);
    c[ImGuiCol_TabHovered]         = ImVec4(0.055f, 0.392f, 0.612f, 0.8f);
    c[ImGuiCol_TabSelected]        = ImVec4(0.055f, 0.392f, 0.612f, 1.0f); // #0E639C
    c[ImGuiCol_Button]             = ImVec4(0.22f, 0.22f, 0.235f, 1.0f);
    c[ImGuiCol_ButtonHovered]      = ImVec4(0.055f, 0.392f, 0.612f, 0.8f);
    c[ImGuiCol_ButtonActive]       = ImVec4(0.055f, 0.392f, 0.612f, 1.0f);
    c[ImGuiCol_TableHeaderBg]      = ImVec4(0.16f, 0.16f, 0.175f, 1.0f);
    c[ImGuiCol_TableBorderStrong]  = ImVec4(0.30f, 0.30f, 0.30f, 1.0f);
    c[ImGuiCol_TableBorderLight]   = ImVec4(0.20f, 0.20f, 0.216f, 1.0f);
    c[ImGuiCol_TableRowBg]         = ImVec4(0.105f, 0.105f, 0.110f, 1.0f);
    c[ImGuiCol_TableRowBgAlt]      = ImVec4(0.135f, 0.135f, 0.145f, 1.0f);
    c[ImGuiCol_TextSelectedBg]     = ImVec4(0.149f, 0.310f, 0.471f, 0.8f); // #264F78
    c[ImGuiCol_CheckMark]          = ImVec4(0.055f, 0.392f, 0.612f, 1.0f);
    c[ImGuiCol_SliderGrab]         = ImVec4(0.055f, 0.392f, 0.612f, 1.0f);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    // Save the base style for scaling
    ImGuiStyle baseStyle = style;
    float currentScale = 1.0f;

    // ── App init ────────────────────────────────────────────────────
    App app;
    g_app = &app;
    Logger::log("[APP] App::init begin");
    app.init(initW, initH, window);
    Logger::log("[APP] App::init done");

    // Restore window decoration after splash
    Logger::log("[APP] glfwSetWindowAttrib begin"); 
    // glfwSetWindowAttrib(window, GLFW_DECORATED, GLFW_TRUE); // Frameless UI
    Logger::log("[APP] glfwSetWindowAttrib done");
    Logger::log("[APP] glfwSetWindowSize begin"); Logger::log("[APP] glfwSetWindowSize begin"); glfwSetWindowSize(window, initW, initH); Logger::log("[APP] glfwSetWindowSize done"); Logger::log("[APP] glfwSetWindowSize done");

    // Center the main window again
    {
        GLFWmonitor* primary = glfwGetPrimaryMonitor();
        if (primary) {
            const GLFWvidmode* mode = glfwGetVideoMode(primary);
            if (mode) {
                int mx, my;
                glfwGetMonitorPos(primary, &mx, &my);
                glfwSetWindowPos(window, mx + (mode->width - initW) / 2,
                                         my + (mode->height - initH) / 2);
            }
        }
    }

    // ── GLFW callbacks (only resize; mouse handled by ImGui + polling) ──
    glfwSetFramebufferSizeCallback(window, glfwFramebufferSizeCb);

    // ── Main loop ───────────────────────────────────────────────────
    while (!glfwWindowShouldClose(window)) {
        Logger::log("[APP] glfwPollEvents begin"); Logger::log("[APP] glfwPollEvents begin"); glfwPollEvents(); Logger::log("[APP] glfwPollEvents done"); Logger::log("[APP] glfwPollEvents done");

        // ── Dynamic font rebuild on scale change ────────────────────
        if (app.uiNeedsFontRebuild()) {
            float newScale = app.uiScale();
            if (std::abs(newScale - currentScale) > 0.01f) {
                currentScale = newScale;
                io.Fonts->Clear();
                if (activeFontPath) {
                    io.Fonts->AddFontFromFileTTF(activeFontPath, 18.0f * currentScale, nullptr, io.Fonts->GetGlyphRangesCyrillic());
                    io.Fonts->AddFontFromFileTTF(activeFontPath, 22.0f * currentScale, nullptr, io.Fonts->GetGlyphRangesCyrillic());
                } else {
                    io.Fonts->AddFontDefault();
                }
                io.Fonts->Build();

                // Re-apply scaled style
                style = baseStyle;
                style.ScaleAllSizes(currentScale);
            }
            app.clearUiFontRebuild();
        }

        // Get framebuffer size
        int fbW, fbH;
        glfwGetFramebufferSize(window, &fbW, &fbH);
        if (fbW <= 0 || fbH <= 0) {
            glfwWaitEvents();
            continue;
        }

        // ImGui new frame
        Logger::log("[APP] NewFrame begin"); Logger::log("[APP] NewFrame begin"); ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 3D camera input (orbit/pan/zoom via ImGui IO polling)
        Logger::log("[APP] processInput begin"); Logger::log("[APP] processInput begin"); app.processInput(); Logger::log("[APP] processInput done"); Logger::log("[APP] processInput done");

        // App update + render 3D
        Logger::log("[APP] app.update begin"); Logger::log("[APP] app.update begin"); app.update(); Logger::log("[APP] app.update done"); Logger::log("[APP] app.update done");
        Logger::log("[APP] app.render begin"); Logger::log("[APP] app.render begin"); app.render(fbW, fbH); Logger::log("[APP] app.render done"); Logger::log("[APP] app.render done");

        // Draw ImGui UI
        Logger::log("[APP] app.drawUI begin"); Logger::log("[APP] app.drawUI begin"); app.drawUI(); Logger::log("[APP] app.drawUI done"); Logger::log("[APP] app.drawUI done");

        // ImGui render
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // ── Cleanup ─────────────────────────────────────────────────────
    // Save layout + window visibility via App/UI (also saves settings.json)
    app.saveLayout();

    // Manually save imgui.ini on exit using _wfopen for Cyrillic path
#ifdef _WIN32
    {
        size_t iniLen = 0;
        const char* iniData = ImGui::SaveIniSettingsToMemory(&iniLen);
        if (iniData && iniLen > 0) {
            FILE* f = _wfopen(iniWidePath.c_str(), L"wb");
            if (f) {
                fwrite(iniData, 1, iniLen, f);
                fclose(f);
            }
        }
    }
#endif
    app.shutdown();
    Logger::log("[APP] App::shutdown done");
    g_app = nullptr;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    Logger::shutdown();
    return 0;
}
