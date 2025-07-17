#pragma once
#include "ICommunication.h"
#include <Wire.h>

constexpr uint8_t DEFAULT_I2C_ADDR = 0x10;
#define MAX_MSG_LEN 32

class I2CCommunication : public ICommunication {
private:
    static String _received;
    static String _toSend;
    uint8_t _address;
    bool _isMaster;

    static void onReceive(int numBytes) {
        _received = "";
        while (Wire.available()) {
            _received += static_cast<char>(Wire.read());
        }
    }

    static void onRequest() {
        Wire.write(_toSend.c_str(), _toSend.length());
    }

public:
    I2CCommunication(bool isMaster, uint8_t address = DEFAULT_I2C_ADDR)
        : _isMaster(isMaster), _address(address) {}

    void begin(unsigned long = 0) override {
        if (_isMaster) {
            Wire.begin();
        } else {
            Wire.begin(_address);
            Wire.onReceive(onReceive);
            Wire.onRequest(onRequest);
        }
    }

    void sendMessage(const String& msg) override {
        if (_isMaster) {
            Wire.beginTransmission(_address);
            Wire.write(msg.c_str());
            Wire.endTransmission();
        } else {
            _toSend = msg;
        }
    }

    String receiveMessage() override {
        if (_isMaster) {
            Wire.requestFrom(_address, MAX_MSG_LEN);
            String msg = "";
            while (Wire.available()) {
                msg += static_cast<char>(Wire.read());
            }
            return msg;
        } else {
            return _received;
        }
    }

    bool available() override {
        return !_received.isEmpty();
    }
};

// Statické členy
String I2CCommunication::_received = "";
String I2CCommunication::_toSend = "";
