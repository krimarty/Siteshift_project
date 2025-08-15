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
constexpr long unsigned int  DELAY_MS = 10;
constexpr uint8_t calib_duty = 75;
constexpr int stableThreshold = 1/(DELAY_MS/1000.0f);
constexpr int changeTolerance = 2;
int lastValue = 0;
int stableCount = 0;

// Last time
unsigned long lastMillis = 0;

// Random mode variables
static int targetPos = 0;

// Sequention mode variables
bool goingLeft = true;

ICommunication* comm;
Motor motor;
UserInterface ui;
MotorFeedback feedback;
Pid pidPos(10, 0.1, 0);
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
    Serial.begin(115200);
    pinMode(MS_PIN, INPUT_PULLUP);
    bool isMaster = digitalRead(MS_PIN) == LOW;

    // Choose communication method based on the mode
    //comm = new UARTCommunication();
    comm = new I2CCommunication(isMaster);

    comm->begin();
    motor.begin();

    pinMode(25, OUTPUT);
    digitalWrite(25, HIGH); 
}

void loop() {
    delay(DELAY_MS);
    unsigned long currentMillis = millis();
    unsigned long deltaMillis = currentMillis - lastMillis;

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
            Serial.println(value);
            if (abs(value - lastValue) < changeTolerance) {
                stableCount++;
            } else {
                stableCount = 0;
            }
            lastValue = value;

            if (stableCount >= stableThreshold) {
                positionCalculator.setAdcMaxLimit(value);
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
                positionCalculator.setAdcMinLimit(value);
                currentState = nextState(currentState);
                stableCount = 0;
            }
            break;
        }

        case State::SYN:{
            Serial.println("SYN");
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
            currentState = nextState(currentState);
            break;
        }

        case State::MANUAL:
            Serial.println("MAN");
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
            currentState = nextState(currentState);
            break;

        case State::RANDOM:{
            Serial.println("RANDOM");
            int currentPos = positionCalculator.computePosition(feedback.readRaw());

            if (targetPos == currentPos)
            {
                targetPos = random(0, 130);
            }
            

            float error =  currentPos - targetPos;

            Serial.print("targetPos: ");
            Serial.println(targetPos);
            Serial.print("currentPos: ");
            Serial.println(currentPos);


            float duty = pidPos.step(error, deltaMillis/1000.0f);
            Serial.print("duty: ");
            Serial.println(duty);
            duty = std::abs(duty);
            if (duty > ui.readFilteredPotPercent()) duty = ui.readFilteredPotPercent();
            else if (duty < 20) duty = 20;

            if (error < 0) {
                motor.left(duty);
            } else {
                motor.right(duty);
            }
            currentState = nextState(currentState);
            break;
        }

        case State::SEQUENCE:
          Serial.println("SEQUENCE");
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
          currentState = nextState(currentState);
          break;
      }


  lastMillis = currentMillis;
}