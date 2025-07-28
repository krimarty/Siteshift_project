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
        return "<" + String(pos, 2) + "," + String(pot) + "," + String(dir) + ";";
    }

    bool update(const String& raw) {
        int start = raw.indexOf('<');
        int end = raw.indexOf(';', start);

        if (start == -1 || end == -1 || end <= start + 1) return false;

        String content = raw.substring(start + 1, end);
        int firstComma = content.indexOf(',');
        int secondComma = content.indexOf(',', firstComma + 1);
        if (firstComma == -1 || secondComma == -1) return false;

        String posStr = content.substring(0, firstComma);
        String potStr = content.substring(firstComma + 1, secondComma);
        String dirStr = content.substring(secondComma + 1);

        _lastMessage.pistonPos = posStr.toFloat();
        _lastMessage.potVal = potStr.toInt();
        _lastMessage.dir = dirStr.toInt();

        return true;
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
