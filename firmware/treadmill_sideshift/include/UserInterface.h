#pragma once
#include <Arduino.h>

// Pin assignment
constexpr int PIN_POT      = 26;
constexpr int PIN_BTN_LEFT = 22;
constexpr int PIN_BTN_RIGHT= 21;
constexpr int PIN_MODE     = 20;
constexpr int PIN_INVERSE  = 19;
constexpr int PIN_MAN      = 18;
constexpr int PIN_SEQ      = 17;
constexpr int PIN_LED_SIGNAL = 13;
constexpr int PIN_LED_LEFT   = 14;
constexpr int PIN_LED_RIGHT  = 15;

enum class ModeType {
    MANUAL,
    RANDOM,
    SEQUENCE
};

enum class Direction {
    LEFT,
    RIGHT,
    STOP
};
class UserInterface {
private:
    float _filteredPot = 0;
    static constexpr float alpha = 0.1; // váha filtru (čím menší, tím pomalejší reakce)
    
    bool isLeftPressed()  { return digitalRead(PIN_BTN_LEFT)  == LOW; }
    bool isRightPressed() { return digitalRead(PIN_BTN_RIGHT) == LOW; }
    
public:
    UserInterface() {
        pinMode(PIN_POT, INPUT);
        pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
        pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
        pinMode(PIN_MODE, INPUT_PULLUP);
        pinMode(PIN_INVERSE, INPUT_PULLUP);
        pinMode(PIN_MAN, INPUT_PULLUP);
        pinMode(PIN_SEQ, INPUT_PULLUP);
        pinMode(PIN_LED_SIGNAL, OUTPUT);
        pinMode(PIN_LED_LEFT, OUTPUT);
        pinMode(PIN_LED_RIGHT, OUTPUT);
    }

    Direction getDirection() {
        if (isLeftPressed() && isRightPressed()) {
            return Direction::STOP;
        }
        else if (isLeftPressed()) {
            return Direction::LEFT;
        } else if (isRightPressed()) {
            return Direction::RIGHT;
        } else {
            return Direction::STOP;
        }
    }

    bool isSyncMode()     { return digitalRead(PIN_MODE)    == LOW; }
    bool isInverse()      { return digitalRead(PIN_INVERSE) == LOW; }

    ModeType getModeType() {
        if (digitalRead(PIN_MAN) == LOW)
            return ModeType::MANUAL;
        else if (digitalRead(PIN_SEQ) == LOW)
            return ModeType::SEQUENCE;
        else
            return ModeType::RANDOM;
    }

    float readFilteredPotPercent() {
        int raw = analogRead(PIN_POT);
        _filteredPot = alpha * raw + (1 - alpha) * _filteredPot;
        return map(_filteredPot, 0, 4095, 20, 100);
    }

    void setSignalLed(bool state) { digitalWrite(PIN_LED_SIGNAL, state ? HIGH : LOW); }
    void setLeftLed(bool state)   { digitalWrite(PIN_LED_LEFT,   state ? HIGH : LOW); }
    void setRightLed(bool state)  { digitalWrite(PIN_LED_RIGHT,  state ? HIGH : LOW); }

};
