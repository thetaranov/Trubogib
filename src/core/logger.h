#pragma once
// ─────────────────────────────────────────────────────────────────────
// logger.h — simple thread-safe file logger (logs/YYYYMMDD_HHMMSS.log)
// ─────────────────────────────────────────────────────────────────────
#include <string>
#include <filesystem>

namespace Logger {

// Initialize logger using executable directory (creates logs/).
void init(const std::filesystem::path& exeDir);

// Write one line with timestamp.
void log(const std::string& msg);

// Flush + close.
void shutdown();

// Returns current log file path (empty if not initialized).
std::string logFilePath();

} // namespace Logger
