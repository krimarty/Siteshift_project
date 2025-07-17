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
    uint8_t _currentDuty = 0;
    uint8_t _targetDuty = 0;
    uint32_t _lastUpdate = 0;
    uint16_t _rampDelay = 10;

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
            analogWrite(_en, _currentDuty);
            _lastUpdate = millis();
        }
    }
public:
    Motor()
        : _nSleep(NSLEEP), _disable(DISABLE), _ph(PH), _en(EN), _nFault(NFAULT) {}

    Motor(int nSleep, int disable, int ph, int en, int nFault)
        : _nSleep(nSleep), _disable(disable), _ph(ph), _en(en), _nFault(nFault) {}

    void begin() {
        pinMode(_nSleep, OUTPUT);
        pinMode(_disable, OUTPUT);
        pinMode(_ph, OUTPUT);
        pinMode(_en, OUTPUT);
        setPwmFreq();
        pinMode(_nFault, INPUT_PULLUP);
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

    void left(uint8_t duty) {
        digitalWrite(_ph, LOW);
        _targetDuty = duty;
    }

    void right(uint8_t duty) {
        digitalWrite(_ph, HIGH);
        _targetDuty = duty;
    }

    void stop() {
        _targetDuty = 0;
    }

    bool isFault() {
        return digitalRead(_nFault) == LOW;
    }

};