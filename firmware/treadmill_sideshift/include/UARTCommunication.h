/* =============================================================
 * Project:   treadmill_sideshift
 * File:      UARTCommunication.h
 * Author:    Martin Kriz
 * Company:   Ullmanna s.r.o.
 * Created:   2025-08-20
 * -------------------------------------------------------------
 * Description:
 *   This header defines the UARTCommunication class, which implements
 *   the ICommunication interface. All communication rules are defined
 *   by ICommunication. This class provides serial communication
 *   over UART on Raspberry Pi Pico 2 and is designed for the Sideshift Board v0.0.0.
 *
 * Notes:
 * ============================================================= */

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
