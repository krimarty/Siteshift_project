#pragma once
#include <Arduino.h>

constexpr int MS_PIN = 4;

class ICommunication {
public:
    virtual ~ICommunication() = default;

    virtual void begin(unsigned long baudrate = 115200) = 0;
    virtual void sendMessage(const String& msg) = 0;
    virtual String receiveMessage() = 0;
    virtual bool available() = 0;
};
