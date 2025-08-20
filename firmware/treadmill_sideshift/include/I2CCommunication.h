/* =============================================================
 * Project:   treadmill_sideshift
 * File:      I2CCommunication.h
 * Author:    Martin Kriz
 * Client:    Ullmanna s.r.o.
 * Created:   2025-08-20
 * -------------------------------------------------------------
 * Description:
 *   This header defines the I2CCommunication class, which implements
 *   the ICommunication interface. All communication rules are defined
 *   by ICommunication. This class provides master/slave communication
 *   over I2C on Raspberry Pi Pico 2 and is designed for the Sideshift Board v0.0.0.
 *
 * Notes:
 * ============================================================= */

#pragma once
#include "ICommunication.h"
#include <Wire.h>

constexpr uint8_t DEFAULT_I2C_ADDR = 0x10;
constexpr uint8_t SDA_PIN = 0;
constexpr uint8_t SCL_PIN = 1;
#define MAX_MSG_LEN 32

class I2CCommunication : public ICommunication {
private:
    static String _received;
    static String _toSend;
    uint8_t _address;
    bool _isMaster;
    unsigned long lastMsg_;

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
        Wire.setSDA(SDA_PIN);
        Wire.setSCL(SCL_PIN);

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
        if (_isMaster) {
            return true;
        } else {
            return !_received.isEmpty();
        }
    }

    bool isConnected(unsigned long timeout = 200) override {
        if (!_isMaster) {
            if (!_received.isEmpty()) {
                lastMsg_ = millis();
            }
            return (millis() - lastMsg_ < timeout);
        } else {
            Wire.requestFrom(_address, (uint8_t)1);
            if (Wire.available() > 0) {
                lastMsg_ = millis();
                return true;
            }
            return (millis() - lastMsg_ < timeout);
        }
    }

};

String I2CCommunication::_received = "";
String I2CCommunication::_toSend = "";
