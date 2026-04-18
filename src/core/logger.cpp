// ─────────────────────────────────────────────────────────────────────
// logger.cpp — simple thread-safe file logger implementation
// ─────────────────────────────────────────────────────────────────────
#include "logger.h"

#include <fstream>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <vector>
#include <algorithm>

namespace {
std::mutex g_mtx;
std::ofstream g_file;
std::filesystem::path g_path;
bool g_inited = false;

std::string nowTs() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto tt = system_clock::to_time_t(now);
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
        << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

std::string fileTs() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto tt = system_clock::to_time_t(now);

    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}
} // namespace

namespace Logger {

void init(const std::filesystem::path& exeDir) {
    std::lock_guard<std::mutex> lock(g_mtx);
    if (g_inited) return;

    std::filesystem::path logDir = exeDir / "logs";
    std::error_code ec;
    std::filesystem::create_directories(logDir, ec);

    // Keep only the last 4 log files (so with the new one we have 5)
    std::vector<std::filesystem::path> logs;
    for (const auto& entry : std::filesystem::directory_iterator(logDir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".log") {
            logs.push_back(entry.path());
        }
    }
    
    // Sort descending by filename usually YYYYMMDD_HHMMSS works correctly
    std::sort(logs.begin(), logs.end(), std::greater<std::filesystem::path>());
    
    // Default config is keep 5, means delete logs[4] and beyond
    if (logs.size() >= 4) {
        for (size_t i = 4; i < logs.size(); ++i) {
            std::filesystem::remove(logs[i], ec);
        }
    }

    g_path = logDir / (fileTs() + ".log");
    g_file.open(g_path, std::ios::out | std::ios::app);
    g_inited = g_file.is_open();
    if (g_inited) {
        g_file << nowTs() << " | [APP] Logger started" << std::endl;
    }
}

void log(const std::string& msg) {
    std::lock_guard<std::mutex> lock(g_mtx);
    if (!g_inited || !g_file.is_open()) return;
    g_file << nowTs() << " | " << msg << std::endl;
}

void shutdown() {
    std::lock_guard<std::mutex> lock(g_mtx);
    if (g_inited && g_file.is_open()) {
        g_file << nowTs() << " | [APP] Logger stopped" << std::endl;
        g_file.flush();
        g_file.close();
    }
    g_inited = false;
}

std::string logFilePath() {
    std::lock_guard<std::mutex> lock(g_mtx);
    return g_path.u8string();
}

} // namespace Logger
