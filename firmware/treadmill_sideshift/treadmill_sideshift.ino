#include "include/Motor.h"
#include "include/UserInterface.h"
#include "include/MotorFeedback.h"
#include "include/I2CCommunication.h"
#include "include/UARTCommunication.h"
#include "include/Pid.h"
#include "include/PositionCalculator.h"
#include "include/OtherDevice.h"


enum class State {
    CALIB_LEFT,
    CALIB_RIGHT,
    SYN,
    MANUAL,
    RANDOM,
    SEQUENCE
};

// Calibration parameters
constexpr uint8_t calib_duty = 50; // Duty cycle for calibration
constexpr int stableThreshold = 10; // jak dlouho musí být hodnota stejná
constexpr int changeTolerance = 3; // tolerance pro změnu hodnoty
int lastValue = 0;
int stableCount = 0;

// Sequention mode variables
bool goingLeft = true;

ICommunication* comm;
Motor motor;
UserInterface ui;
MotorFeedback feedback;
Pid pidPos(10, 0, 0);
PositionCalculator positionCalculator(0.0, 130.0);
OtherDevice otherDevice;

State currentState = State::CALIB_LEFT;

State nextState(State state)
{
    if(ui.isSyncMode()) return State::SYN;
    else if(ui.getModeType() == ModeType::MANUAL) return State::MANUAL;
    else if(ui.getModeType() == ModeType::RANDOM) return State::RANDOM;
    else return State::SEQUENCE;
}

void setup() {
    pinMode(MS_PIN, INPUT_PULLUP);
    bool isMaster = digitalRead(MS_PIN) == LOW;

    // Choose communication method based on the mode
    comm = new UARTCommunication();
    // comm = new I2CCommunication(isMaster);

    comm->begin();
    motor.begin();
}

void loop() {
    // Periodical class updates
    motor.dutyUpdate();
    ui.readFilteredPotPercent();

    // Second board communication
    //Receive
    if (comm->available()) {otherDevice.update(comm->receiveMessage());}
    // Send
    comm->sendMessage(otherDevice.send(0, 0, 0));


    switch (currentState) {
        case State::CALIB_LEFT:{
        // Set position limits
        motor.left(calib_duty);
        int value = feedback.readRaw();

        if (abs(value - lastValue) < changeTolerance) {
            stableCount++;
        } else {
            stableCount = 0;
        }
        lastValue = value;

        if (stableCount >= stableThreshold) {
            positionCalculator.setAdcMinLimit(value);
            currentState = State::CALIB_RIGHT;
            stableCount = 0;
        }
        break;}

        case State::CALIB_RIGHT:{
        // Set position limits
        motor.right(calib_duty);
        int value = feedback.readRaw();

        if (abs(value - lastValue) < changeTolerance) {
            stableCount++;
        } else {
            stableCount = 0;
        }
        lastValue = value;

        if (stableCount >= stableThreshold) {
            positionCalculator.setAdcMaxLimit(value);
            currentState = nextState(currentState);
            stableCount = 0;
        }
        break;}

        case State::SYN:{
        float error = otherDevice.getPistonPos() - positionCalculator.computePosition(feedback.readRaw());
        float duty = pidPos.step(error, 0.0f);

        duty = std::abs(duty);
        if (duty > 100.0f) {
            duty = 100.0f;
        }

        if (error < 0) {
            motor.left(duty);
        } else {
            motor.right(duty);
        }
        break;
        }

        case State::MANUAL:
        if (ui.getDirection() == Direction::LEFT) {
            motor.left(ui.readFilteredPotPercent());
        } else if (ui.getDirection() == Direction::RIGHT) {
            motor.right(ui.readFilteredPotPercent());
        } else {
            motor.stop();
        }
        break;

        case State::RANDOM:
        break;

        case State::SEQUENCE:
        float duty = ui.readFilteredPotPercent();

        if (goingLeft) {
            motor.left(duty);
            if (positionCalculator.minLimitReached(feedback.readRaw())) {
                goingLeft = false;
            }
        } else {
            motor.right(duty);
            if (positionCalculator.maxLimitReached(feedback.readRaw())) {
                goingLeft = true;
            }
        }
        break;
    }



}