#pragma once
// ─────────────────────────────────────────────────────────────────────
// serial_port.h — Win32 serial port for Teensy USB-Serial communication
// ─────────────────────────────────────────────────────────────────────
#ifdef _WIN32
#include <windows.h>
#endif
#include <string>
#include <vector>
#include <mutex>

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    // Open COM port (e.g. "COM3"), returns true on success
    bool open(const std::string& port, int baudRate = 115200);
    void close();
    bool isOpen() const;

    // Send a command string (appends \n automatically)
    bool send(const std::string& cmd);

    // Read available data into internal buffer, returns lines received
    // Call this each frame from main loop
    int poll();

    // Get and clear received lines
    std::vector<std::string> getLines();

    // Enumerate available COM ports
    static std::vector<std::string> listPorts();

    std::string lastError() const { return lastError_; }

private:
    std::string lastError_;
#ifdef _WIN32
    HANDLE hSerial_ = INVALID_HANDLE_VALUE;
#endif
    std::string rxBuf_;
    std::vector<std::string> lines_;
    std::mutex mtx_;
};
