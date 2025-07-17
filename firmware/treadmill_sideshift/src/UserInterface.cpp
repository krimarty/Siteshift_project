#include "UserInterface.h"

void UserInterface::leftISR() {
    leftTriggered = true;
}

void UserInterface::rightISR() {
    rightTriggered = true;
}

void UserInterface::processInputs() {
    static unsigned long lastLeftPress = 0;
    static unsigned long lastRightPress = 0;
    unsigned long now = millis();

    if (leftTriggered && now - lastLeftPress > debounceDelay) {
        leftTriggered = false;
        lastLeftPress = now;
        _direction = (_direction == LEFT) ? STOP : LEFT;
    }

    if (rightTriggered && now - lastRightPress > debounceDelay) {
        rightTriggered = false;
        lastRightPress = now;
        _direction = (_direction == RIGHT) ? STOP : RIGHT;
    }
}
