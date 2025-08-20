/* =============================================================
 * Project:   treadmill_sideshift
 * File:      Motor.h
 * Author:    Martin Kriz
 * Company:   Ullmanna s.r.o.
 * Created:   2025-08-20
 * -------------------------------------------------------------
 * Description:
 *   This header defines the Motor class, which provides an interface
 *   to control a motor using the DRV8873H driver. 
 *   Designed for Raspberry Pi Pico 2 and the Sideshift Board v0.0.0.
 *
 * Notes:
 * ============================================================= */

#pragma once
#include <Arduino.h>

constexpr int NSLEEP  = 5;
constexpr int DISABLE = 6;
constexpr int PH     = 7;
constexpr int EN     = 8;
constexpr int NFAULT  = 9;

class Motor {
private:
    int _nSleep, _disable, _ph, _en, _nFault;
    int16_t _currentDuty = 0;
    int16_t _targetDuty = 0;
    uint32_t _lastUpdate = 0;
    uint16_t _rampDelay = 10;

    bool _isInverse = false;

    void setPwmFreq(uint32_t freq = 25000) {
        analogWriteFreq(freq);
    }

    void dutyUpdate_worker() {
        if (_currentDuty == _targetDuty) return;
        if (millis() - _lastUpdate >= _rampDelay) {
            if (_currentDuty < _targetDuty) {
                _currentDuty++;
            } else {
                _currentDuty--;
            }
            if (_currentDuty < 0) {
                digitalWrite(_ph, LOW);
            } else if (_currentDuty > 0) {
                digitalWrite(_ph, HIGH);
            }
            analogWrite(_en, map(abs(_currentDuty), 0, 100, 0, 255));
            _lastUpdate = millis();
        }
    }
public:
    Motor()
        : _nSleep(NSLEEP), _disable(DISABLE), _ph(PH), _en(EN), _nFault(NFAULT), _isInverse(false) {}

    Motor(int nSleep, int disable, int ph, int en, int nFault)
        : _nSleep(nSleep), _disable(disable), _ph(ph), _en(en), _nFault(nFault), _isInverse(false) {}

    void begin() {
        pinMode(_nSleep, OUTPUT);
        pinMode(_disable, OUTPUT);
        pinMode(_ph, OUTPUT);
        pinMode(_en, OUTPUT);
        setPwmFreq();
        pinMode(_nFault, INPUT);
        wake();
        enable();
        stop();
    }

    void dutyUpdate() {
        dutyUpdate_worker();
    }

    void wake() {
        digitalWrite(_nSleep, HIGH);
    }

    void sleep() {
        digitalWrite(_nSleep, LOW);
    }

    void enable() {
        digitalWrite(_disable, LOW); 
    }

    void disable() {
        digitalWrite(_disable, HIGH);
    }

    void left(int16_t duty) {
        if (duty > 100) {duty = 100;}
        if (_isInverse) {
            _targetDuty = duty;
        } else {
            _targetDuty = -duty;
        }
    }

    void right(int16_t duty) {
        if (duty > 100) {duty = 100;}
        if (_isInverse) {
            _targetDuty = -duty;
        }
        else {
            _targetDuty = duty;
        }
    }

    void stop() {
        _targetDuty = 0;
    }

    void resetFault() {
        digitalWrite(_nSleep, LOW);
        delayMicroseconds(12); 
        digitalWrite(_nSleep, HIGH);

    }
    bool isFault() {
        return digitalRead(_nFault) == LOW;
    }

    void setInverse(bool inverse) {
        _isInverse = inverse;
    }

};