#pragma once

#include <Arduino.h>
#include <Wire.h>

constexpr int MS_PIN = 4;

// Change communication method here
#define USE_UART
// #define USE_I2C

class Communication {
public:
    bool isMaster = false;

    Communication() {
        pinMode(MS_PIN, INPUT_PULLUP);
        isMaster = (digitalRead(MS_PIN) == LOW);
    }

    void begin(unsigned long baudrate = 115200, uint8_t i2c_addr = 0x10) {
#ifdef USE_UART
        Serial1.begin(baudrate);
#elif defined(USE_I2C)
        if (isMaster) {
            Wire.begin();
        } else {
            Wire.begin(i2c_addr);
        }
#endif
    }

    void sendMessage(const String& msg) {
#ifdef USE_UART
        Serial1.println(msg);
#elif defined(USE_I2C)
        if (isMaster) {
            // Pro I2C master: zadej cílovou adresu
            // Wire.beginTransmission(addr); Wire.write(msg); Wire.endTransmission();
        }
        // Pro I2C slave: nelze posílat, pouze přijímat
#endif
    }

    String receiveMessage() {
#ifdef USE_UART
        if (Serial1.available()) {
            return Serial1.readStringUntil('\n');
        }
#elif defined(USE_I2C)
        // Pro I2C slave použij Wire.onReceive v hlavním souboru
#endif
        return "";
    }

    bool available() {
#ifdef USE_UART
        return Serial1.available() > 0;
#elif defined(USE_I2C)
        // Pro I2C slave použij Wire.onReceive v hlavním souboru
        return false;
#endif
    }
};