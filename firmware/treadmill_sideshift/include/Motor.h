#pragma once

constexpr int MOTOR_NSLEEP  = 5;
constexpr int MOTOR_DISABLE = 6;
constexpr int MOTOR_IN2     = 7;
constexpr int MOTOR_IN1     = 8;
constexpr int MOTOR_NFAULT  = 9;

class Motor {
private:
    int _nSleep, _disable, _in2, _in1, _nFault;

public:
    Motor()
        : _nSleep(MOTOR_NSLEEP), _disable(MOTOR_DISABLE), _in2(MOTOR_IN2), _in1(MOTOR_IN1), _nFault(MOTOR_NFAULT) {}

    Motor(int nSleep, int disable, int in2, int in1, int nFault)
        : _nSleep(nSleep), _disable(disable), _in2(in2), _in1(in1), _nFault(nFault) {}

    void begin() {
        pinMode(_nSleep, OUTPUT);
        pinMode(_disable, OUTPUT);
        pinMode(_in2, OUTPUT);
        pinMode(_in1, OUTPUT);
        pinMode(_nFault, INPUT_PULLUP);
        wake();
        enable();
        stop();
    }

    void wake() {
        digitalWrite(_nSleep, HIGH);
    }

    void sleep() {
        digitalWrite(_nSleep, LOW);
    }

    void enable() {
        digitalWrite(_disable, LOW); // LOW = enabled for DRV8873H
    }

    void disable() {
        digitalWrite(_disable, HIGH); // HIGH = disabled for DRV8873H
    }

    void forward() {
        digitalWrite(_in1, HIGH);
        digitalWrite(_in2, LOW);
    }

    void backward() {
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, HIGH);
    }

    void stop() {
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, LOW);
    }

    bool isFault() {
        return digitalRead(_nFault) == LOW; // LOW = fault active
    }
};