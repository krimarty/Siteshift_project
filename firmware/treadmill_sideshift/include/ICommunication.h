/* =============================================================
 * Project:   treadmill_sideshift
 * File:      ICommunication.h
 * Author:    Martin Kriz
 * Company:   Ullmanna s.r.o.
 * Created:   2025-08-20
 * -------------------------------------------------------------
 * Description:
 *   This header defines the ICommunication interface, which sets
 *   the standard rules for communication classes in the treadmill_sideshift
 *   project. All concrete communication implementations (UART, I2C)
 *   must follow this interface.
 *
 * Notes:
 *   - All communication classes must implement these methods consistently.
 * ============================================================= */

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
    virtual bool isConnected(unsigned long timeout = 200) = 0;

};
