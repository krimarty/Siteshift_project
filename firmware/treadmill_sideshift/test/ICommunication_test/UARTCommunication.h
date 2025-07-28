#pragma once
#include "ICommunication.h"

class UARTCommunication : public ICommunication {
public:
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
};
