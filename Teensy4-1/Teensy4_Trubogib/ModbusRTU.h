/*
 * ModbusRTU.h - Simple Modbus RTU library for Delta VFD control
 * Teensy 4.1 with RS485 on Serial1
 * 
 * Delta CP2000 VFD037CP43B-21 Modbus RTU registers:
 * - Write frequency: Function 06 (single register), Address 0x0002
 * - Write control: Function 06 (single register), Address 0x0000
 * - Read status: Function 03 (multiple registers)
 */

#pragma once
#include <Arduino.h>

class ModbusRTU {
public:
    // VFD Modbus Slave ID
    static constexpr uint8_t VFD_SLAVE_ID = 1;
    
    // Register addresses (Delta CP2000)
    enum Register : uint16_t {
        REG_CONTROL = 0x0000,  // Control: bit 0 = RUN, bit 1 = FWD(0)/REV(1), bit 2 = ?
        REG_FREQ    = 0x0002,  // Frequency command (0.01Hz units, 0-6000 = 0-60Hz)
        REG_STATUS  = 0x0100,  // Status register (read-only)
        REG_FREQ_FB = 0x0101,  // Frequency feedback (0.01Hz units)
        REG_CURR_FB = 0x0102,  // Current feedback (0.1A units)
    };
    
    // Control register bits
    static constexpr uint16_t CTRL_RUN     = 0x0001;  // bit 0: 1=RUN, 0=STOP
    static constexpr uint16_t CTRL_REV     = 0x0002;  // bit 1: 0=FORWARD, 1=REVERSE
    static constexpr uint16_t CTRL_JOG     = 0x0004;  // bit 2: Jog mode
    
    // Functions
    static constexpr uint8_t FC_WRITE_SINGLE = 0x06;  // Write single register
    static constexpr uint8_t FC_READ_MULTI   = 0x03;  // Read multiple registers
    
    // Statistics
    struct Stats {
        uint32_t txCount = 0;
        uint32_t rxCount = 0;
        uint32_t errorCount = 0;
        uint16_t lastErrorCode = 0;
    };
    
    ModbusRTU(HardwareSerial& serial, uint8_t dePin) 
        : serial_(serial), dePin_(dePin) {}
    
    void begin(unsigned long baud) {
        pinMode(dePin_, OUTPUT);
        digitalWrite(dePin_, LOW);  // Receive mode
        serial_.begin(baud, SERIAL_8N2);
    }
    
    // Write frequency command (in 0.01Hz units, so 5000 = 50Hz)
    bool writeFrequency(uint16_t freqCode) {
        return writeSingleRegister(REG_FREQ, freqCode);
    }
    
    // Write control register
    bool writeControl(uint16_t controlWord) {
        return writeSingleRegister(REG_CONTROL, controlWord);
    }
    
    // Write single Modbus register
    bool writeSingleRegister(uint16_t regAddr, uint16_t value) {
        uint8_t request[8];
        request[0] = VFD_SLAVE_ID;
        request[1] = FC_WRITE_SINGLE;
        request[2] = (regAddr >> 8) & 0xFF;
        request[3] = regAddr & 0xFF;
        request[4] = (value >> 8) & 0xFF;
        request[5] = value & 0xFF;
        
        uint16_t crc = calculateCRC(request, 6);
        request[6] = crc & 0xFF;
        request[7] = (crc >> 8) & 0xFF;
        
        sendMessage(request, 8);
        stats_.txCount++;
        
        return true;
    }
    
    // Read Modbus registers
    bool readRegisters(uint16_t startAddr, uint16_t count, uint16_t* outBuffer) {
        uint8_t request[8];  // Increased to 8 bytes for CRC storage
        request[0] = VFD_SLAVE_ID;
        request[1] = FC_READ_MULTI;
        request[2] = (startAddr >> 8) & 0xFF;
        request[3] = startAddr & 0xFF;
        request[4] = (count >> 8) & 0xFF;
        request[5] = count & 0xFF;
        
        uint16_t crc = calculateCRC(request, 6);
        request[6] = crc & 0xFF;
        request[7] = (crc >> 8) & 0xFF;
        
        sendMessage(request, 8);
        stats_.txCount++;
        
        // Read response (simplified - expects immediate response)
        // 20 ms cap: at 9600 8N2 a 9-byte reply takes ~10 ms on the wire.
        // Longer blocks starve pollFeedbackEncoders() in the main loop.
        uint32_t timeoutMs = millis() + 20;
        while (millis() < timeoutMs) {
            if (serial_.available() >= 5) {
                uint8_t slaveId = serial_.read();
                uint8_t fc = serial_.read();
                uint8_t byteCount = serial_.read();
                
                if (slaveId != VFD_SLAVE_ID || fc != FC_READ_MULTI) {
                    stats_.errorCount++;
                    return false;
                }
                
                for (int i = 0; i < byteCount / 2; i++) {
                    uint16_t val = ((uint16_t)serial_.read() << 8) | serial_.read();
                    if (i < count) outBuffer[i] = val;
                }
                
                // Read and discard CRC (simplified - not validating)
                (void)serial_.read();  // CRC low byte
                (void)serial_.read();  // CRC high byte
                stats_.rxCount++;
                return true;
            }
        }
        
        stats_.errorCount++;
        stats_.lastErrorCode = 0xFFFF;  // Timeout
        return false;
    }
    
    Stats& getStats() { return stats_; }
    
private:
    HardwareSerial& serial_;
    uint8_t dePin_;
    Stats stats_;
    
    // CRC-16 (Modbus)
    uint16_t calculateCRC(const uint8_t* data, uint8_t len) {
        uint16_t crc = 0xFFFF;
        for (int i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }
    
    void sendMessage(const uint8_t* data, uint8_t len) {
        // Set RS485 to transmit mode
        digitalWrite(dePin_, HIGH);
        delayMicroseconds(100);
        
        // Send data
        serial_.write(data, len);
        serial_.flush();
        
        // Switch back to receive mode
        delayMicroseconds(100);
        digitalWrite(dePin_, LOW);
    }
};
