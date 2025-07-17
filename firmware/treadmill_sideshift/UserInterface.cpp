#include "include/UserInterface.h"

volatile bool UserInterface::_leftTriggered = false;
volatile bool UserInterface::_rightTriggered = false;


void UserInterface::leftISR() {
    _leftTriggered = true;
}

void UserInterface::rightISR() {
    _rightTriggered = true;
}

void UserInterface::processInputs() {
    static unsigned long lastLeftPress = 0;
    static unsigned long lastRightPress = 0;
    unsigned long now = millis();

    if (_leftTriggered && now - _lastLeftPress > debounceDelay) {
        _leftTriggered = false;
        _lastLeftPress = now;
        _direction = (_direction == LEFT) ? STOP : LEFT;
    }

    if (_rightTriggered && now - _lastRightPress > debounceDelay) {
        _rightTriggered = false;
        _lastRightPress = now;
        _direction = (_direction == RIGHT) ? STOP : RIGHT;
    }
}
