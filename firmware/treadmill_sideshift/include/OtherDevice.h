#pragma once

#include <Arduino.h>
#include <string.h>

#pragma pack(push, 1)
struct PistonMessage {
    float pistonPos;
    int potVal;
    int dir;
};
#pragma pack(pop)

class OtherDevice {
private:
    PistonMessage _lastMessage;

public:
    OtherDevice() {
        memset(&_lastMessage, 0, sizeof(PistonMessage));
    }

    String send(float pos, int pot, int dir) {
        PistonMessage msg = { pos, pot, dir };
        return String(reinterpret_cast<const char*>(&msg), sizeof(PistonMessage));
    }

    bool update(const String& raw) {
        if (raw.length() == sizeof(PistonMessage)) {
            memcpy(&_lastMessage, raw.c_str(), sizeof(PistonMessage));
            return true;
        }
        return false;
    }

    const PistonMessage& getLastMessage() const {
        return _lastMessage;
    }

    float getPistonPos() const {
        return _lastMessage.pistonPos;
    }

    int getPotVal() const {
        return _lastMessage.potVal;
    }

    int getDirection() const {
        return _lastMessage.dir;
    }

    void reset() {
        memset(&_lastMessage, 0, sizeof(PistonMessage));
    }

};
