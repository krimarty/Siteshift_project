#pragma once

#include <Arduino.h>
#include <Wire.h>

#define MAX_MSG_LEN 32
constexpr int MS_PIN = 4;

// Change communication method here
#define USE_UART
// #define USE_I2C

class Communication {
private:
#ifdef USE_I2C
    static String receiveMessageMsg;
    static String sendMessageMsg;

    static void onReceiveEvent(int numBytes) {
        String msg = "";
        while (Wire.available()) {
            char c = Wire.read();
            msg += c;
        }
        receiveMessageMsg = msg;
    }
    static void onRequestEvent() {
        Wire.write(sendMessageMsg.c_str(), sendMessageMsg.length());
    }
    

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
            Wire.onReceive(onReceiveEvent);
            Wire.onRequest(onRequestEvent);
        }
#endif
    }

    void sendMessage(const String& msg) {
#ifdef USE_UART
        Serial1.println(msg);
#elif defined(USE_I2C)
        if (isMaster) {
            Wire.beginTransmission(0x10); Wire.write(msg); Wire.endTransmission();
        }
        else {
            sendMessageMsg = msg;
        }
#endif
    }

    String receiveMessage() {
#ifdef USE_UART
        if (Serial1.available()) {
            return Serial1.readStringUntil('\n');
        }
#elif defined(USE_I2C)
        if (isMaster) {
            Wire.requestFrom(0x10, MAX_MSG_LEN);
            String msg = "";
            while (Wire.available()) {
                char c = Wire.read();
                msg += c;
            }
            return msg;
        }
        else {
            return receiveMessageMsg;
        }
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