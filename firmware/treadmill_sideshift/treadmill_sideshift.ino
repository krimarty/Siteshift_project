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
constexpr uint8_t calib_duty = 50;
constexpr int stableThreshold = 10;
constexpr int changeTolerance = 3;
int lastValue = 0;
int stableCount = 0;

// Random mode variables
static int targetPos = 0;
static unsigned long lastTargetChange = 0;
static const unsigned long changeInterval = 3000;

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
    //comm = new UARTCommunication();
    comm = new I2CCommunication(isMaster);

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
    comm->sendMessage(otherDevice.send(positionCalculator.computePosition(feedback.readRaw()), ui.readFilteredPotPercent(), ui.getDirectionNum()));

    if (currentState != State::MANUAL)
    {
        ui.setLeftLed(false);
        ui.setRightLed(false);
    }

    bool active =
        currentState == State::SEQUENCE ||
        currentState == State::RANDOM   ||
        currentState == State::SYN;
    ui.setSignalLed(active);

    if(ui.isInverse()) {
        positionCalculator.setInverse(true);
        motor.setInverse(true);
    } else {
        positionCalculator.setInverse(false);
        motor.setInverse(false);
    }

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
            break;
        }

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
            break;
        }

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
                ui.setLeftLed(true);
                ui.setRightLed(false);
            } else if (ui.getDirection() == Direction::RIGHT) {
                motor.right(ui.readFilteredPotPercent());
                ui.setRightLed(true);
                ui.setLeftLed(false);
            } else {
                motor.stop();
                ui.setLeftLed(true);
                ui.setRightLed(true);
            }
            break;

        case State::RANDOM:{
            int currentPos = positionCalculator.computePosition(feedback.readRaw());

            unsigned long changeInterval = map(ui.readFilteredPotPercent(), 0, 100, 10000, 3000);

            if (millis() - lastTargetChange > changeInterval + random(0, 3000)) {
                targetPos = random(positionCalculator.minLimit(), positionCalculator.maxLimit());
                lastTargetChange = millis();
            }

            float error = targetPos - currentPos;
            float duty = pidPos.step(error, 0.0f);
            duty = std::abs(duty);
            if (duty > ui.readFilteredPotPercent()) duty = ui.readFilteredPotPercent();

            if (std::abs(error) < 5) {
                motor.stop();
            } else if (error < 0) {
                motor.left(duty);
            } else {
                motor.right(duty);
            }
            break;
        }

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