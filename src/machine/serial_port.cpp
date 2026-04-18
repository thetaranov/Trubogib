// ─────────────────────────────────────────────────────────────────────
// serial_port.cpp — Win32 serial port implementation
// ─────────────────────────────────────────────────────────────────────
#include "serial_port.h"
#include "logger.h"

#include <algorithm>

#ifdef _WIN32
#include <setupapi.h>
#pragma comment(lib, "setupapi.lib")
#pragma comment(lib, "advapi32.lib")
#endif

SerialPort::~SerialPort() { close(); }

bool SerialPort::isOpen() const {
#ifdef _WIN32
    return hSerial_ != INVALID_HANDLE_VALUE;
#else
    return false;
#endif
}

bool SerialPort::open(const std::string& port, int baudRate) {
#ifdef _WIN32
    close();

    // Prefix for port numbers >= 10
    std::string path = "\\\\.\\" + port;
    hSerial_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE,
        0, nullptr, OPEN_EXISTING, 0, nullptr);

    if (hSerial_ == INVALID_HANDLE_VALUE) {
        lastError_ = "Cannot open " + port + " (err " + std::to_string(GetLastError()) + ")";
        Logger::log(std::string("[SERIAL] ") + lastError_);
        return false;
    }

    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(hSerial_, &dcb)) {
        lastError_ = "GetCommState failed";
        Logger::log(std::string("[SERIAL] ") + lastError_);
        close();
        return false;
    }

    dcb.BaudRate = baudRate;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;

    if (!SetCommState(hSerial_, &dcb)) {
        lastError_ = "SetCommState failed";
        Logger::log(std::string("[SERIAL] ") + lastError_);
        close();
        return false;
    }

    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout         = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier  = 0;
    timeouts.ReadTotalTimeoutConstant    = 0;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant   = 100;
    SetCommTimeouts(hSerial_, &timeouts);

    // Flush buffers
    PurgeComm(hSerial_, PURGE_RXCLEAR | PURGE_TXCLEAR);

    lastError_.clear();
    rxBuf_.clear();
    lines_.clear();
    Logger::log(std::string("[SERIAL] Opened ") + port + " @" + std::to_string(baudRate));
    return true;
#else
    lastError_ = "Serial port not supported on this platform";
    Logger::log(std::string("[SERIAL] ") + lastError_);
    return false;
#endif
}

void SerialPort::close() {
#ifdef _WIN32
    if (hSerial_ != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial_);
        hSerial_ = INVALID_HANDLE_VALUE;
        Logger::log("[SERIAL] Closed");
    }
#endif
    rxBuf_.clear();
    lines_.clear();
}

bool SerialPort::send(const std::string& cmd) {
#ifdef _WIN32
    if (!isOpen()) return false;

    std::string data = cmd + "\n";
    DWORD written = 0;
    if (!WriteFile(hSerial_, data.c_str(), (DWORD)data.size(), &written, nullptr)) {
        DWORD err = GetLastError();
        lastError_ = "Write failed (err " + std::to_string(err) + ")";
        Logger::log(std::string("[SERIAL] ") + lastError_);
        // Device disappeared / re-enumerated: close stale handle immediately.
        if (err == ERROR_BAD_COMMAND || err == ERROR_INVALID_HANDLE || err == ERROR_GEN_FAILURE || err == ERROR_DEVICE_NOT_CONNECTED) {
            close();
        }
        return false;
    }
    return written == (DWORD)data.size();
#else
    return false;
#endif
}

int SerialPort::poll() {
#ifdef _WIN32
    if (!isOpen()) return 0;

    int newLines = 0;
    for (;;) {
        char buf[4096];
        DWORD bytesRead = 0;
        if (!ReadFile(hSerial_, buf, sizeof(buf), &bytesRead, nullptr)) {
            DWORD err = GetLastError();
            if (err != ERROR_IO_PENDING) {
                lastError_ = "Read failed (err " + std::to_string(err) + ")";
                Logger::log(std::string("[SERIAL] ") + lastError_);
                if (err == ERROR_BAD_COMMAND || err == ERROR_INVALID_HANDLE || err == ERROR_GEN_FAILURE || err == ERROR_DEVICE_NOT_CONNECTED) {
                    close();
                }
            }
            break;
        }
        if (bytesRead == 0)
            break;

        for (DWORD i = 0; i < bytesRead; i++) {
            char c = buf[i];
            if (c == '\n' || c == '\r') {
                if (!rxBuf_.empty()) {
                    std::lock_guard<std::mutex> lock(mtx_);
                    lines_.push_back(rxBuf_);
                    rxBuf_.clear();
                    newLines++;
                }
            } else {
                rxBuf_ += c;
                if (rxBuf_.size() > 32768) {
                    // Corrupted stream protection
                    rxBuf_.clear();
                }
            }
        }
    }
    return newLines;
#else
    return 0;
#endif
}

std::vector<std::string> SerialPort::getLines() {
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<std::string> result;
    result.swap(lines_);
    return result;
}

std::vector<std::string> SerialPort::listPorts() {
    std::vector<std::string> ports;
#ifdef _WIN32
    // Use registry HARDWARE\DEVICEMAP\SERIALCOMM — non-blocking enumeration.
    // Previous implementation called CreateFileA() on COM1..COM32 to probe
    // existence, which could block for tens of seconds when a device
    // (e.g. Teensy in "device not recognized" state) is mid-enumeration,
    // freezing the UI thread. Reading the registry is instant and never
    // opens the devices.
    HKEY hKey = nullptr;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE,
                      "HARDWARE\\DEVICEMAP\\SERIALCOMM",
                      0, KEY_READ, &hKey) == ERROR_SUCCESS) {
        DWORD idx = 0;
        char valueName[256];
        BYTE  data[256];
        while (true) {
            DWORD nameLen = sizeof(valueName);
            DWORD dataLen = sizeof(data);
            DWORD type = 0;
            LONG r = RegEnumValueA(hKey, idx++, valueName, &nameLen,
                                   nullptr, &type, data, &dataLen);
            if (r == ERROR_NO_MORE_ITEMS) break;
            if (r != ERROR_SUCCESS) break;
            if (type == REG_SZ) {
                // Data is the friendly name like "COM7"
                ports.push_back(std::string(reinterpret_cast<const char*>(data)));
            }
        }
        RegCloseKey(hKey);
        // Natural sort: COM1, COM2, ..., COM10 — compare numeric tail
        std::sort(ports.begin(), ports.end(), [](const std::string& a, const std::string& b) {
            auto num = [](const std::string& s) -> int {
                int n = 0; bool any = false;
                for (char c : s) { if (c >= '0' && c <= '9') { n = n * 10 + (c - '0'); any = true; } }
                return any ? n : 0;
            };
            return num(a) < num(b);
        });
    }
#endif
    return ports;
}
