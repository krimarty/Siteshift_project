#pragma once
#include "ICommunication.h"

class UARTCommunication : public ICommunication {
public:
    unsigned long lastMsg_;

    void begin(unsigned long baudrate = 115200) override {
        Serial1.begin(baudrate);
    }

    void sendMessage(const String& msg) override {
        Serial1.println(msg);
    }

    String receiveMessage() override {
        if (Serial1.available()) {
            return Serial1.readStringUntil('\n');
        }
        return "";
    }

    bool available() override {
        return Serial1.available() > 0;
    }

    bool isConnected(unsigned long timeout = 200) override {
        if (Serial1.available()) {
            lastMsg_ = millis();
        }
        return (millis() - lastMsg_ < timeout);
    }
};
